//============================================================================
//  esc_cpu -- the Konami 056734 (ESC) as an instruction-set core
//
//  Runs each chip's own boot object, kernel and game program (the chip's
//  firmware, from the game ROM) instead of a translation of the programs.
//  Semantics: D:/FPGATools/esc/escemu.py step(), which matches MAME on every
//  byte of the sprite page (ESC_FINAL_REPORT section 11).  The RTL was checked
//  against that model, instruction count and all memory, from power-on through
//  boot, RESET and LOAD for five titles and on 11,115 RUN frames
//  (sim/esc_cpu/tb_esc_cpu.sv, MEASUREMENTS 171).
//
//  The ISA reference is secondary (AI-generated, esc_manual.md); escemu's
//  UNSURE list holds here unchanged.  INFERRED from it and from the kernel's
//  own code, not from a PCB:
//    * power-on: the internal boot ROM copies the boot object -- the longword
//      count at s11[23:0] (0x259 in every title) and that many longwords --
//      into local memory 0.., then runs from 0 (manual 6.1).
//    * IRQ4: a write to s6 with bit 1 set is the interrupt pulse; the kernel
//      ends every packet with ori.b s6,#1 / #3 / #1 (manual 5.1, kernel 38-40).
//    * `ready`: a write to s6 with bit 0 set and bit 1 clear -- the kernel's
//      `ori.b s6,#1` before it waits for the mailbox.
//
//  Multi-cycle, one instruction at a time, every stage registered for 96 MHz:
//    F  local read at pc        D  descramble (XOR K, lane permutation)
//    R  register read           X  execute; host / local / mul / div set up
//    M0 M1  local operand       H  host access      MU  multiply      DV divide
//    W  write back, retire
//  The host port is byte-addressed, 1 / 2 / 4 bytes big-endian, any alignment,
//  req held to a one-clock ack; esc_host splits it for the 16-bit bus.
//============================================================================
`default_nettype none

module esc_cpu #(
    parameter integer LAW = 13                  // local memory: 2^LAW words (8K: the whole of it is used)
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         en,                     // 0 = no chip (a set without a known secret): stay in reset
    input  wire         hold,                   // 1 = do not start the next instruction (testbenches; 0 in the core)

    input  wire [31:0]  K,                      // fetch XOR
    input  wire [63:0]  perm,                   // lane k (canonical) = stored lane perm[4k+3:4k]
    input  wire [31:0]  s10_init,               // the chip's secret
    input  wire [31:0]  s11_init,               // boot object address | configuration nibbles

    input  wire         mbox_we,                // the host's long to cc0000
    input  wire [31:0]  mbox_d,

    output reg          host_req,
    output reg          host_we,
    output reg  [23:0]  host_addr,
    output reg  [2:0]   host_size,
    output reg  [31:0]  host_wdata,
    input  wire         host_ack,
    input  wire [31:0]  host_rdata,

    output reg          irq_pulse,              // one clock: s6 written with bit 1 set
    output reg          ready_pulse,            // one clock: s6 written with bit 0 set, bit 1 clear
    output wire [31:0]  pc_o,
    output wire [31:0]  s7_o,
    output reg          insn_done,              // one clock per retired instruction
    output reg          booted,                 // the boot object is in local memory
    output reg          fault                   // undefined opcode or branch condition
);

// ---------------------------------------------------------------------------
//  state

(* ramstyle = "M10K", max_depth = 2048 *) reg [31:0] lmem [0:(1 << LAW) - 1];   // 2K x 5 slices: 28 M10K, not 32
reg  [31:0] r [1:31];                         // r0 reads 0 and is never written
reg  [31:0] s [0:31];
reg  [31:0] pc;
reg         fZ, fN, fC, fV;

assign pc_o = pc;
assign s7_o = s[7];

localparam [4:0] S_BOOT0 = 5'd0, S_BOOT1 = 5'd1, S_BOOT2 = 5'd2,
                 S_F = 5'd3, S_D = 5'd4, S_R = 5'd5, S_X = 5'd6,
                 S_M0 = 5'd7, S_M1 = 5'd8, S_H = 5'd9, S_MU = 5'd10, S_DV = 5'd11, S_W = 5'd12, S_M2 = 5'd13;
reg  [4:0] st;

// local memory: one registered read port, one write port (driven from registers)
reg  [LAW-1:0] la;
reg  [31:0]    lq;
reg            lw_en;
reg  [LAW-1:0] lw_a;
reg  [31:0]    lw_d;
always @(posedge clk) begin
    lq <= lmem[la];
    if (lw_en) lmem[lw_a] <= lw_d;
end

// SIMULATION ONLY: start from esc_export.py's post-LOAD state (+esc_dir=, and
// not +esc_boot).  Synthesis never sees this.
// synthesis translate_off
reg [31:0] init_regs [0:83];
reg        sim_skip_boot = 1'b0;
initial begin
    string d;
    integer i;
    if ($value$plusargs("esc_dir=%s", d) && !$test$plusargs("esc_boot")) begin
        $readmemh({d, "/init_l.hex"}, lmem);
        $readmemh({d, "/init_regs.hex"}, init_regs);
        for (i = 1; i < 32; i = i + 1) r[i] = init_regs[i];
        for (i = 0; i < 32; i = i + 1) s[i] = init_regs[32 + i];
        pc = init_regs[64];
        {fZ, fN, fC, fV} = init_regs[65][3:0];
        sim_skip_boot = 1'b1;
    end
end
// synthesis translate_on

// ---------------------------------------------------------------------------
//  decode

function automatic [1:0] ln(input [31:0] w, input integer k);
    ln = w[2*k +: 2];
endfunction

reg  [31:0] wx, wd;
integer     pk;
always @(*) begin
    wx = lq ^ K;
    for (pk = 0; pk < 16; pk = pk + 1) wd[2*pk +: 2] = wx[2*perm[4*pk +: 4] +: 2];
end

reg  [31:0] iw;                                 // canonical instruction
reg  [31:0] idx;                                // its pc
wire [5:0]  f_rA   = {ln(iw,5), ln(iw,15), ln(iw,9)};
wire [5:0]  f_rB   = {ln(iw,2), ln(iw,1),  ln(iw,4)};
wire [9:0]  f_o10u = {ln(iw,6), ln(iw,14), ln(iw,13), ln(iw,12), ln(iw,11)};
wire [31:0] f_off  = {{22{f_o10u[9]}}, f_o10u};
wire [15:0] f_im16 = {ln(iw,2), ln(iw,1), ln(iw,4), ln(iw,6), ln(iw,14), ln(iw,13), ln(iw,12), ln(iw,11)};
wire [6:0]  f_rC   = f_o10u[9:3];
wire [1:0]  l0 = ln(iw,0), l3 = ln(iw,3), l7 = ln(iw,7), l8 = ln(iw,8), l10 = ln(iw,10);
wire [1:0]  sz  = l7;
wire [7:0]  key = {l10, l8, l3, l0};            // the pattern without its size digit
wire [9:0]  pat = {l10, l8, l7, l3, l0};
wire [5:0]  cnd = {l10, l8, l3};
wire        is_br  = (ln(iw,5) == 2'd2) && (l7 == 2'd2) && (ln(iw,9) == 2'd2) && (ln(iw,15) == 2'd0) && (l0 == 2'd0);
wire [4:0]  li_reg = {l8[0], l3, l10[1], ~l10[0]};
wire [23:0] imm24  = {l7, f_rA, f_im16};

localparam [7:0]
    K_ADD = 8'b01_10_00_11, K_SUB = 8'b01_10_01_11, K_AND = 8'b01_10_10_11, K_XOR = 8'b01_10_11_11,
    K_OR  = 8'b11_10_01_11, K_NOR = 8'b11_10_11_11,
    K_ADDI= 8'b01_11_00_11, K_SUBI= 8'b01_11_01_11, K_ANDI= 8'b01_11_10_11, K_XORI= 8'b01_11_11_11, K_ORI = 8'b11_11_01_11,
    K_LD  = 8'b10_00_11_11, K_ST  = 8'b11_00_11_11, K_LDL = 8'b10_01_11_11, K_STL = 8'b11_01_11_11,
    K_ADDM= 8'b00_01_00_11, K_SUBM= 8'b00_01_01_11, K_ANDM= 8'b00_01_10_11, K_XORM= 8'b00_01_11_11, K_ORM = 8'b10_01_01_11,
    K_MADD= 8'b01_01_00_11, K_MSUB= 8'b01_01_01_11, K_MAND= 8'b01_01_10_11, K_MXOR= 8'b01_01_11_11,
    K_CMPM= 8'b00_00_01_11, K_CMPMR=8'b01_00_01_11,
    K_SHL = 8'b01_10_00_10, K_ASL = 8'b01_10_01_10, K_ROL = 8'b01_10_10_10,
    K_SHR = 8'b11_10_00_10, K_ASR = 8'b11_10_01_10, K_ROR = 8'b11_10_10_10;
localparam [9:0]
    P_PUSH = 10'b01_01_00_00_10, P_POP  = 10'b11_01_00_00_10, P_RET = 10'b01_01_00_01_10, P_RET2 = 10'b11_01_00_01_10,
    P_LIH  = 10'b01_00_00_11_10, P_SWAP = 10'b11_00_00_01_10, P_EXTH = 10'b01_00_10_01_10, P_EXTB = 10'b01_00_01_01_10,
    P_MUL  = 10'b01_00_00_10_10, P_MULH = 10'b01_00_10_10_10, P_DIVH = 10'b11_00_10_10_10, P_NEG  = 10'b11_00_00_00_10,
    P_JR   = 10'b01_00_10_00_00, P_JSR  = 10'b01_01_10_00_00;

wire sized = (sz != 2'd3);
wire k_ml  = sized && (key == K_LDL || key == K_STL || key == K_ADDM || key == K_SUBM || key == K_ANDM ||
                       key == K_XORM || key == K_ORM || key == K_MADD || key == K_MSUB || key == K_MAND || key == K_MXOR);
wire k_mh  = sized && (key == K_LD || key == K_ST || key == K_CMPM || key == K_CMPMR);
wire k_pop = (pat == P_POP) || (pat == P_RET) || (pat == P_RET2);


// ---- the decode, registered in R: X / M / H read only these (timing, D31)
reg  [7:0]   x_key;
reg  [9:0]   x_pat;
reg  [5:0]   x_cnd;
reg  [1:0]   x_sz;
reg          x_sized;
reg          x_is_br;
reg  [1:0]   x_l0;
reg  [1:0]   x_l8;
reg  [4:0]   x_li_reg;
reg  [23:0]  x_imm24;
reg  [31:0]  x_f_off;
reg  [15:0]  x_f_im16;
reg          x_k_ml;
reg          x_k_mh;
reg          x_k_pop;
reg  [5:0]  x_rA;
reg         x_nop;

// ---------------------------------------------------------------------------
//  arithmetic helpers (sizes: 0 = 32, 1 = 8, 2 = 16 bits)

function automatic [31:0] szmask(input [1:0] z);
    case (z) 2'd1: szmask = 32'h0000_00ff; 2'd2: szmask = 32'h0000_ffff; default: szmask = 32'hffff_ffff; endcase
endfunction
function automatic msb(input [31:0] v, input [1:0] z);
    case (z) 2'd1: msb = v[7]; 2'd2: msb = v[15]; default: msb = v[31]; endcase
endfunction
function automatic [31:0] merge(input [31:0] up, input [31:0] res, input [1:0] z);
    merge = (up & ~szmask(z)) | (res & szmask(z));
endfunction
// {C, V, Z, N, result}
function automatic [35:0] addsub(input [31:0] a0, input [31:0] b0, input [1:0] z, input sub);
    // The carry and the borrow are BITS of one 33-bit adder, not comparisons
    // (timing, D31): with both operands masked to the size, a + b carries
    // into bit 8 / 16 / 32, and {0,a} - {0,b} borrows into bit 32 exactly
    // when a < b -- escemu's `full > m` and `a < b`.
    reg [31:0] m, a, b, res; reg [32:0] full; reg C, V;
    begin
        m = szmask(z); a = a0 & m; b = b0 & m;
        full = sub ? ({1'b0, a} - {1'b0, b}) : ({1'b0, a} + {1'b0, b});
        res  = full[31:0] & m;
        if (sub)            C = full[32];
        else case (z)
            2'd1:    C = full[8];
            2'd2:    C = full[16];
            default: C = full[32];
        endcase
        V = sub ? msb((a ^ b) & (a ^ res), z) : msb(~(a ^ b) & (a ^ res), z);
        addsub = {C, V, (res == 32'd0), msb(res, z), res};
    end
endfunction
function automatic [1:0] zn(input [31:0] res, input [1:0] z);
    zn = {((res & szmask(z)) == 32'd0), msb(res, z)};
endfunction

// ---------------------------------------------------------------------------
//  register read (R) and the values X works from

// (register read: see rp_a..rp_h below)

// The four read ports as plain muxes, not through rd_reg: Quartus 17.0 reports
// an array read only inside a function as "assigned but never read" (10036),
// although the registers are built (2,855 in the c2b77718 fit).
reg  [31:0] rp_a, rp_b, rp_c, rp_h;
wire [6:0] n_a = {1'b0, f_rA}, n_b = {1'b0, f_rB}, n_c = f_rC, n_h = {2'b00, li_reg};
always @(*) begin
    if (n_a == 7'd0)              rp_a = 32'd0;
    else if (n_a < 7'd32)         rp_a = r[n_a[4:0]];
    else if (n_a[4:0] == 5'd2)    rp_a = idx + 32'd2;       // s2: the pc as software sees it
    else                          rp_a = s[n_a[4:0]];
    if (n_b == 7'd0)              rp_b = 32'd0;
    else if (n_b < 7'd32)         rp_b = r[n_b[4:0]];
    else if (n_b[4:0] == 5'd2)    rp_b = idx + 32'd2;       // s2: the pc as software sees it
    else                          rp_b = s[n_b[4:0]];
    if (n_c == 7'd0)              rp_c = 32'd0;
    else if (n_c < 7'd32)         rp_c = r[n_c[4:0]];
    else if (n_c[4:0] == 5'd2)    rp_c = idx + 32'd2;       // s2: the pc as software sees it
    else                          rp_c = s[n_c[4:0]];
    if (n_h == 7'd0)              rp_h = 32'd0;
    else if (n_h < 7'd32)         rp_h = r[n_h[4:0]];
    else if (n_h[4:0] == 5'd2)    rp_h = idx + 32'd2;       // s2: the pc as software sees it
    else                          rp_h = s[n_h[4:0]];
end
reg  [31:0] A, B, Cc, Hv;                       // rA, rB, rC, li_reg -- latched in R
reg  [31:0] ea;                                 // rB + off, latched in X
reg  [31:0] Lr;                                 // the local operand, registered in M1
reg  [31:0] zn_v;                               // Z / N are computed in W from this (timing)
reg  [1:0]  zn_sz;

// write back, applied in W
reg         wb_en;
reg  [6:0]  wb_n;
reg  [31:0] wb_v;
reg  [31:0] pc_n;
reg         s1_we;
reg  [31:0] s1_v;
reg         s8_we;
reg  [31:0] s8_v;
reg         s7_clr;
reg         fzn_we, fcv_we;
reg  [3:0]  f_new;                              // {Z, N, C, V}

// host-read consumers
reg         h_cmp, h_cmpr;
reg  [6:0]  h_rd;
reg  [1:0]  h_sz;
reg         h_ld;                               // the read result goes to a register

// multiply / divide
reg  [31:0] mul_a, mul_b;
reg         mul_h;
reg  [1:0]  mul_ph;
reg  [63:0] prod;
reg  [15:0] dv_q, dv_d;
reg  [16:0] dv_r;
reg  [4:0]  dv_n;
reg  [15:0] dv_n16;                             // the dividend, shifted out MSB first

// boot loader
reg  [9:0]  bl_n, bl_i;                         // longwords to copy, copied so far

integer i;
reg  [35:0] as_;
reg  [1:0]  zz;
reg  [31:0] res, mm, L, v;
reg         take;

task automatic set_wb(input [6:0] n, input [31:0] val);
    begin wb_en <= (n != 7'd0); wb_n <= n; wb_v <= val; end
endtask

always @(posedge clk) begin
    insn_done   <= 1'b0;
    irq_pulse   <= 1'b0;
    ready_pulse <= 1'b0;
    lw_en       <= 1'b0;

    if (rst || !en) begin
        st <= S_BOOT0; fault <= 1'b0; host_req <= 1'b0; booted <= 1'b0;
        la <= {LAW{1'b0}};
        // synthesis translate_off
        if (sim_skip_boot) begin st <= S_F; booted <= 1'b1; la <= pc[LAW-1:0]; end
        // synthesis translate_on
    end else begin
        case (st)
        // ---- power-on: the boot object into local memory (INFERRED, header) --
        S_BOOT0: begin
            for (i = 1; i < 32; i = i + 1) r[i] <= 32'd0;              // r0 reads 0 and is never written
            for (i = 0; i < 32; i = i + 1) s[i] <= 32'd0;
            s[10] <= s10_init; s[11] <= s11_init;
            pc <= 32'd0; {fZ, fN, fC, fV} <= 4'd0;
            host_req <= 1'b1; host_we <= 1'b0; host_size <= 3'd4;
            host_addr <= s11_init[23:0];
            st <= S_BOOT1;
        end
        S_BOOT1: if (host_ack) begin                    // the count
            host_req <= 1'b0;
            bl_n <= host_rdata[9:0]; bl_i <= 10'd0;
            st <= S_BOOT2;
        end
        S_BOOT2: begin
            if (!host_req) begin
                if (bl_i == bl_n) begin st <= S_F; booted <= 1'b1; la <= {LAW{1'b0}}; end
                else begin
                    host_req <= 1'b1; host_we <= 1'b0; host_size <= 3'd4;
                    host_addr <= s11_init[23:0] + 24'd4 + {12'd0, bl_i, 2'b00};
                end
            end else if (host_ack) begin
                host_req <= 1'b0;
                lw_en <= 1'b1; lw_a <= {{(LAW-10){1'b0}}, bl_i}; lw_d <= host_rdata;
                bl_i <= bl_i + 10'd1;
            end
        end

        // ---- fetch ------------------------------------------------------------
        S_F: begin
            if (!fault && !hold) st <= S_D;             // la holds pc; lq next edge
        end
        S_D: begin
            iw <= wd; idx <= pc; st <= S_R;
        end
        S_R: begin
            A  <= rp_a;
            B  <= rp_b;
            Cc <= rp_c;
            Hv <= rp_h;
            x_key <= key;
            x_pat <= pat;
            x_cnd <= cnd;
            x_sz <= sz;
            x_sized <= sized;
            x_is_br <= is_br;
            x_l0 <= l0;
            x_l8 <= l8;
            x_li_reg <= li_reg;
            x_imm24 <= imm24;
            x_f_off <= f_off;
            x_f_im16 <= f_im16;
            x_k_ml <= k_ml;
            x_k_mh <= k_mh;
            x_k_pop <= k_pop;
            x_rA <= f_rA;
            x_nop <= (iw == 32'd0);
            st <= S_X;
        end

        // ---- execute ----------------------------------------------------------
        S_X: begin
            wb_en <= 1'b0; s1_we <= 1'b0; s8_we <= 1'b0; s7_clr <= 1'b0;
            fzn_we <= 1'b0; fcv_we <= 1'b0;
            pc_n <= idx + 32'd1;
            ea   <= B + x_f_off;
            st   <= S_W;
            if (x_nop) begin
                // nop
            end else if (x_is_br) begin
                case (x_cnd)
                    6'b01_00_00, 6'b01_01_00: take = 1'b1;          // jmp, call
                    6'b10_00_00: take = fZ;
                    6'b11_00_00: take = !fZ;
                    6'b00_00_01: take = fN;
                    6'b01_00_01: take = !fN;
                    6'b10_00_01: take = fC;
                    6'b11_00_01: take = !fC;
                    6'b01_00_10: take = !fZ && (fN == fV);
                    6'b11_00_10: take = !fC && !fZ;
                    6'b00_00_11: take = (fN != fV);
                    default:     begin take = 1'b0; fault <= 1'b1; end
                endcase
                if (x_cnd == 6'b01_01_00) begin                       // call: push the return
                    s1_we <= 1'b1; s1_v <= s[1] - 32'd1;
                    lw_en <= 1'b1; lw_a <= s[1][LAW-1:0] - 1'b1; lw_d <= idx + 32'd1;
                end
                if (take) pc_n <= idx + 32'd1 + {{16{x_f_im16[15]}}, x_f_im16};
            end else if (x_l0 == 2'd0) begin                          // cls 0
                if (x_pat == P_JR) pc_n <= A + x_f_off;
                else if (x_pat == P_JSR) begin
                    s1_we <= 1'b1; s1_v <= s[1] - 32'd1;
                    lw_en <= 1'b1; lw_a <= s[1][LAW-1:0] - 1'b1; lw_d <= idx + 32'd1;
                    pc_n <= A;
                end else set_wb({2'b00, x_li_reg}, {8'd0, x_imm24});    // li
            end else if (x_l0 == 2'd1) begin                          // cls 1: ld.abs / st.abs
                if (x_l8[1]) begin
                    if (x_imm24 == 24'hcc0000) s7_clr <= 1'b1;        // the chip empties its mailbox
                    else begin
                        host_req <= 1'b1; host_we <= 1'b1; host_addr <= x_imm24; host_size <= 3'd4;
                        host_wdata <= Hv; h_ld <= 1'b0; h_cmp <= 1'b0; st <= S_H;
                    end
                end else begin
                    host_req <= 1'b1; host_we <= 1'b0; host_addr <= x_imm24; host_size <= 3'd4;
                    h_rd <= {2'b00, x_li_reg}; h_ld <= 1'b1; h_cmp <= 1'b0; st <= S_H;
                end
            end else if (x_k_ml || x_k_pop) begin
                st <= S_M0;                                         // ea / s1 registered: read next
            end else if (x_k_mh) begin
                host_req   <= 1'b1;
                host_addr  <= B[23:0] + x_f_off[23:0];
                host_size  <= (x_sz == 2'd1) ? 3'd1 : (x_sz == 2'd2) ? 3'd2 : 3'd4;
                h_sz       <= x_sz;
                if (x_key == K_ST) begin host_we <= 1'b1; host_wdata <= A & szmask(x_sz); h_ld <= 1'b0; h_cmp <= 1'b0; end
                else begin
                    host_we <= 1'b0; h_rd <= {1'b0, x_rA};
                    h_ld <= (x_key == K_LD); h_cmp <= (x_key != K_LD); h_cmpr <= (x_key == K_CMPMR);
                end
                st <= S_H;
            end else if (x_pat == P_MUL || x_pat == P_MULH) begin
                mul_a <= A; mul_b <= B; mul_h <= (x_pat == P_MULH); mul_ph <= 2'd0;
                st <= S_MU;
            end else if (x_pat == P_DIVH) begin
                dv_n16 <= A[15:0]; dv_d <= B[15:0]; dv_r <= 17'd0; dv_q <= 16'd0; dv_n <= 5'd0;
                st <= S_DV;
            end else if (x_pat == P_PUSH) begin
                s1_we <= 1'b1; s1_v <= s[1] - 32'd1;
                lw_en <= 1'b1; lw_a <= s[1][LAW-1:0] - 1'b1; lw_d <= A;
            end else if (x_pat == P_LIH)  set_wb({1'b0, x_rA}, {x_f_im16, A[15:0]});
            else if (x_pat == P_SWAP)     set_wb({1'b0, x_rA}, {B[15:0], B[31:16]});
            else if (x_pat == P_EXTH)     set_wb({1'b0, x_rA}, {{16{B[15]}}, B[15:0]});
            else if (x_pat == P_EXTB)     set_wb({1'b0, x_rA}, {{24{B[7]}}, B[7:0]});
            else if (x_pat == P_NEG)      set_wb({1'b0, x_rA}, -B);
            else if (x_sized && (x_key == K_ADD || x_key == K_SUB)) begin
                as_ = addsub(B, Cc, x_sz, x_key == K_SUB);
                fzn_we <= 1'b1; fcv_we <= 1'b1; f_new[1:0] <= {as_[35], as_[34]}; zn_v <= as_[31:0]; zn_sz <= x_sz;
                set_wb({1'b0, x_rA}, merge(B, as_[31:0], x_sz));
            end else if (x_sized && (x_key == K_AND || x_key == K_XOR || x_key == K_OR || x_key == K_NOR)) begin
                case (x_key) K_AND: res = B & Cc; K_XOR: res = B ^ Cc; K_OR: res = B | Cc; default: res = ~(B | Cc); endcase
                fzn_we <= 1'b1; zn_v <= res; zn_sz <= x_sz;
                set_wb({1'b0, x_rA}, merge(B, res, x_sz));
            end else if (x_sized && (x_key == K_ADDI || x_key == K_SUBI)) begin
                as_ = addsub(A, {16'd0, x_f_im16}, x_sz, x_key == K_SUBI);
                fzn_we <= 1'b1; fcv_we <= 1'b1; f_new[1:0] <= {as_[35], as_[34]}; zn_v <= as_[31:0]; zn_sz <= x_sz;
                set_wb({1'b0, x_rA}, merge(A, as_[31:0], x_sz));
            end else if (x_sized && (x_key == K_ANDI || x_key == K_XORI || x_key == K_ORI)) begin
                case (x_key) K_ANDI: res = A & {16'd0, x_f_im16}; K_XORI: res = A ^ {16'd0, x_f_im16}; default: res = A | {16'd0, x_f_im16}; endcase
                fzn_we <= 1'b1; zn_v <= res; zn_sz <= x_sz;
                set_wb({1'b0, x_rA}, merge(A, res, x_sz));
                // s6, the host handshake (INFERRED).  The IMMEDIATE decides, not
                // the result: s6 reads back what was written, so after the first
                // #3 bit 1 stays set and every later #1 would look like a pulse.
                if (x_rA == 6'd38 && x_key == K_ORI) begin
                    if (x_f_im16[1])              irq_pulse   <= 1'b1;
                    else if (x_f_im16[0])         ready_pulse <= 1'b1;
                end
            end else if (x_sized && (x_key == K_SHL || x_key == K_ASL || x_key == K_SHR || x_key == K_ASR || x_key == K_ROL || x_key == K_ROR)) begin
                mm = szmask(x_sz); v = B & mm;
                case (x_key)
                    K_SHL, K_ASL: res = (B << 1) & mm;
                    K_SHR:        res = v >> 1;
                    K_ASR:        res = (v >> 1) | (msb(v, x_sz) ? (x_sz == 2'd1 ? 32'h80 : x_sz == 2'd2 ? 32'h8000 : 32'h8000_0000) : 32'd0);
                    K_ROL:        res = ((v << 1) | {31'd0, msb(v, x_sz)}) & mm;
                    default:      res = (v >> 1) | (v[0] ? (x_sz == 2'd1 ? 32'h80 : x_sz == 2'd2 ? 32'h8000 : 32'h8000_0000) : 32'd0);
                endcase
                fzn_we <= 1'b1; zn_v <= res; zn_sz <= x_sz;
                set_wb({1'b0, x_rA}, merge(B, res, x_sz));
            end else begin
                fault <= 1'b1;
            end
        end

        // ---- local operand ----------------------------------------------------
        S_M0: begin                                     // la was loaded in X (below); lq next edge
            st <= S_M1;
        end
        S_M1: begin                                     // the block RAM's output, registered (timing)
            Lr <= lq;
            st <= S_M2;
        end
        S_M2: begin
            L = Lr;
            st <= S_W;
            if (x_k_pop) begin
                s1_we <= 1'b1; s1_v <= s[1] + 32'd1;
                if (x_pat == P_POP) set_wb({1'b0, x_rA}, L);
                else pc_n <= L;
            end else case (x_key)
                K_LDL: set_wb({1'b0, x_rA}, L & szmask(x_sz));
                K_STL: begin lw_en <= 1'b1; lw_a <= ea[LAW-1:0]; lw_d <= merge(L, A, x_sz); end
                K_ADDM, K_SUBM: begin
                    as_ = addsub(A, L, x_sz, x_key == K_SUBM);
                    fzn_we <= 1'b1; fcv_we <= 1'b1; f_new[1:0] <= {as_[35], as_[34]}; zn_v <= as_[31:0]; zn_sz <= x_sz;
                    set_wb({1'b0, x_rA}, merge(A, as_[31:0], x_sz));
                end
                K_ANDM, K_XORM, K_ORM: begin
                    case (x_key) K_ANDM: res = A & L; K_XORM: res = A ^ L; default: res = A | L; endcase
                    fzn_we <= 1'b1; zn_v <= res; zn_sz <= x_sz;
                    set_wb({1'b0, x_rA}, merge(A, res, x_sz));
                end
                K_MADD, K_MSUB: begin
                    as_ = addsub(L, A, x_sz, x_key == K_MSUB);
                    fzn_we <= 1'b1; fcv_we <= 1'b1; f_new[1:0] <= {as_[35], as_[34]}; zn_v <= as_[31:0]; zn_sz <= x_sz;
                    lw_en <= 1'b1; lw_a <= ea[LAW-1:0]; lw_d <= merge(L, as_[31:0], x_sz);
                end
                default: begin                          // K_MAND, K_MXOR
                    res = (x_key == K_MAND) ? (L & A) : (L ^ A);
                    fzn_we <= 1'b1; zn_v <= res; zn_sz <= x_sz;
                    lw_en <= 1'b1; lw_a <= ea[LAW-1:0]; lw_d <= merge(L, res, x_sz);
                end
            endcase
        end

        // ---- host access ------------------------------------------------------
        S_H: if (host_ack) begin
            host_req <= 1'b0;
            st <= S_W;
            if (!host_we) begin
                if (h_cmp) begin
                    as_ = h_cmpr ? addsub(host_rdata, A, h_sz, 1'b1) : addsub(A, host_rdata, h_sz, 1'b1);
                    fzn_we <= 1'b1; fcv_we <= 1'b1; f_new[1:0] <= {as_[35], as_[34]}; zn_v <= as_[31:0]; zn_sz <= h_sz;
                end else if (h_ld) set_wb(h_rd, host_rdata);
            end
        end

        // ---- multiply: operands registered in X, product here, s8 in W --------
        S_MU: begin
            mul_ph <= mul_ph + 2'd1;
            if (mul_ph == 2'd0) prod <= mul_h ? $signed({{16{mul_a[15]}}, mul_a[15:0]}) * $signed({{16{mul_b[15]}}, mul_b[15:0]})
                                              : mul_a * mul_b;
            else begin s8_we <= 1'b1; s8_v <= prod[31:0]; st <= S_W; end
        end

        // ---- div.h: 16 / 16 unsigned, restoring, one bit a clock ---------------
        S_DV: begin
            if (dv_d == 16'd0) begin s8_we <= 1'b1; s8_v <= 32'h0000_ffff; st <= S_W; end
            else if (dv_n == 5'd16) begin s8_we <= 1'b1; s8_v <= {16'd0, dv_q}; st <= S_W; end
            else begin
                if ({dv_r[15:0], dv_n16[15]} >= {1'b0, dv_d}) begin
                    dv_r <= {dv_r[15:0], dv_n16[15]} - {1'b0, dv_d}; dv_q <= {dv_q[14:0], 1'b1};
                end else begin
                    dv_r <= {dv_r[15:0], dv_n16[15]};               dv_q <= {dv_q[14:0], 1'b0};
                end
                dv_n16 <= {dv_n16[14:0], 1'b0};
                dv_n <= dv_n + 5'd1;
            end
        end

        // ---- write back and retire --------------------------------------------
        S_W: begin
            if (s1_we) s[1] <= s1_v;
            if (s8_we) s[8] <= s8_v;
            if (wb_en) begin
                if (wb_n < 7'd32) r[wb_n[4:0]] <= wb_v;
                else              s[wb_n[4:0]] <= wb_v;              // put after the s1 adjust: escemu's pop order
            end
            if (s7_clr) s[7] <= 32'd0;
            if (fzn_we) begin fZ <= ((zn_v & szmask(zn_sz)) == 32'd0); fN <= msb(zn_v, zn_sz); end
            if (fcv_we) {fC, fV} <= f_new[1:0];
            pc <= pc_n;
            la <= pc_n[LAW-1:0];
            insn_done <= 1'b1;
            st <= S_F;
        end
        default: st <= S_F;
        endcase

        // the local operand address, loaded as X hands over to M0
        if (st == S_X && (x_k_ml || x_k_pop)) la <= x_k_pop ? s[1][LAW-1:0] : (B[LAW-1:0] + x_f_off[LAW-1:0]);
    end

    // The host's mailbox write wins over the chip's own clear in the same clock.
    if (mbox_we) s[7] <= mbox_d;
end

endmodule

`default_nettype wire
