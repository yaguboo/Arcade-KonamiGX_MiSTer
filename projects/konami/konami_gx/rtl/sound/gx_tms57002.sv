//============================================================================
//  Konami System GX -- TMS57002 "DASP", the sound board's effects DSP
//
//  Source: MAME src/devices/cpu/tms57002/tms57002.cpp, tms57kdec.cpp,
//  tmsinstr.lst and the expansions tmsmake.py generates from it (BSD-3-Clause,
//  copyright-holders Olivier Galibert).  Every instruction MAME implements is
//  here, with the tmsinstr.lst line it follows; the macro expansions (%a %c %d
//  %d24 %ml %mo %mv %wa %wc %wd %b %sfai) are tmsmake.py:27-111.  The golden this
//  is checked against is MAME's own code compiled standalone, tools/tmsgold/, and
//  sim/tb_gx_tms57002.sv compares every sample (rung 1).
//
//  ---- what it does ----------------------------------------------------------
//  A microcoded DSP.  Per 48 kHz sample (`tick`): the stream update puts the
//  previous run's so[] out, latches si[], and calls sync_w (tms57002.cpp:923-939,
//  :218-235); then the program runs from pc 0 for at most CYCLES instructions or
//  until `idle` (execute_run, :837-921).  One instruction is a pre op (category
//  2a), a category-1 op, a post op (2b) and the ca/id increments, in that order
//  (tms57kdec.cpp decode order; tms57002.cpp:806-819), or one category-3 op.
//  The host port (data_w :104-168, data_r :170-186, pload_w/cload_w :40-74) and
//  the status bits (konamigx.cpp:1161-1166) are MAME's.
//
//  ---- how, and what it costs ------------------------------------------------
//  One instruction is a short state walk: F (opcode in, external-memory byte
//  step, macc pipeline shift, MO/MV precomputed), P (pre op), X (category 1 or
//  3 op; for everything but a multiply also post op, increments and the next
//  pc), and for the nine multiply-accumulate ops Y (the registered multiply) and
//  Z (macc and the end of the instruction).  3 clocks, 5 for a multiply, +1 when
//  a pre op moves ca/id under a category-1 op that reads through them (RA).
//
//  Memories: pmem 256x24, cmem 256x32, dmem0 256x24, dmem1 32x24, as MLAB with a
//  registered write request and a read-during-write bypass -- NO M10K (the board
//  has 3 left).  The multiply is a registered 33x33 (DSP blocks).
//
//  External data memory (konamigx.cpp:1196 gxtmsmap, 256 KB) is NOT here: `xm_*`
//  asks for a whole access (the 2-6 bytes MAME steps over one instruction at a
//  time) the moment rde/wre is accepted, from a 4-deep queue, and MAME's step
//  timing is kept internally -- xrd changes at the instruction whose step
//  completes the access, and only that instruction waits if the bytes have not
//  arrived.  Writes never wait.
//
//  ---- accuracy marking, root CLAUDE.md 1.6 -----------------------------------
//  EMULATION_DERIVED.  This is MAME's reading of the chip, instruction for
//  instruction, including what MAME leaves out: the instructions tmsinstr.lst
//  lists with no body (zacc, zmac, cmpl, the three xor, adds, amac, ampy,
//  mpy creg, lpd, smld, std1, dimh/diml, doml, dos, incd, raov, ld0t, lbrk,
//  bioz) do nothing here either, and an opcode MAME cannot decode is a nop that
//  touches neither ca nor id.  PURE_RTL: all of it runs in the FPGA.
//  TODO(HARDWAREIZE): the real DASP's instruction timing, its external-memory
//  interface and the unimplemented instructions.
//
//  NOT MODELLED: MAME's decode cache applies the 'f'-type st1 instructions
//  (rnd, sfao, sfma, scrm, ldpk, ...) once at DECODE time of a linear run as
//  well as at execution (tmsmake.py EmitCdec).  After the first run of a program
//  that is the same as applying them at execution, which is what this does.
//  A reset in the middle of an external write completes the whole write here;
//  MAME would have stepped only some of its bytes.
//============================================================================
`default_nettype none

module gx_tms57002 #(
    parameter int CYCLES      = 250,   // 12 MHz / 48 kHz: execute_run's icount a sample
    // Clocks between the stream update and the first instruction.  MAME's order
    // for a sample is stream update, THEN the host accesses of that slice, THEN
    // execution (tools/tmsgold/tmsgold.cpp header); a testbench that replays
    // host events at their sample widens this window to fit them.
    parameter int START_DELAY = 1
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        tick,
    input  wire [23:0] si0, si1, si2, si3,
    output reg  [23:0] so0, so1, so2, so3,
    input  wire        data_wr,
    input  wire [7:0]  data_in,
    input  wire        data_rd,
    output reg  [7:0]  data_out,
    input  wire        ctrl_pload,
    input  wire        ctrl_cload,
    input  wire        reset_line,
    output wire [2:0]  status,
    output reg         xm_req,
    output reg         xm_we,
    output reg  [19:0] xm_adr,
    output reg  [2:0]  xm_n,
    output reg  [47:0] xm_wdata,
    input  wire        xm_ack,
    input  wire [47:0] xm_rdata,
    output reg         dbg_overrun,
    //  ---- P2 observability (MEASUREMENTS 72) ---------------------------------
    //  What sets S_HOST and what the DSP's host port looked like when it did.
    //  Defect A is an S_HOST that appears inside the ~3 ms coefficient window,
    //  and U43 says the DSP runs through that window by MAME's own choice --
    //  so the one thing worth seeing on the board is whether S_HOST rises
    //  while `in_cload` is up.  NO new register, and nothing in the core reads
    //  these: an
    //  earlier version added a one-clock lpc pulse and the build after it
    //  missed timing on an unrelated gx_tilemap path -- this design's known
    //  1.4 ns netlist sensitivity (MEASUREMENTS 70).  Nothing is left here
    //  for a placement to react to.
    output wire        dbg_s_host,
    output wire  [2:0] dbg_hidx,
    output wire        dbg_in_cload
);

// ---- sti, tms57002.h:47-58 ----------------------------------------------------
reg        in_pload, in_cload, su_cval;
reg [1:0]  su;                    // SU_MASK >> 3: 0 ST0, 1 ST1, 2 PRG
reg        s_idle, s_read, s_write, s_branch, s_host, s_update;
reg        susp;                  // RESET asserted: execution suspended (diexec.cpp:717-729)

// ---- registers, tms57002.h:128-146 ----------------------------------------------
reg [23:0] st0, st1;
reg [7:0]  pc, ca, id, ba0, ba1, sa, rptc, rptc_next;
reg [31:0] aacc;
reg [63:0] macc, macc_write;
reg [18:0] xba;
reg [23:0] xrd;
//  MAME state the RTL does not need to compute anything -- creg is never read
//  by an implemented op, xoa and xwr are folded straight into the op queue, and
//  %mo / %mv are precomputed from macc_write at F -- kept as mirrors for the
//  testbench's snapshot load and trace compare only.  In a Quartus build they
//  would be "assigned but never read" (warning 10036 x4, build of 06b7ea9).
`ifdef VERILATOR
reg [31:0] creg, xoa;
reg [63:0] macc_read;
reg [23:0] xwr;
`endif
reg [7:0]  host0, host1, host2, host3;
reg [2:0]  hidx;
//  The coefficient update queue and the op queue are flip-flops ON PURPOSE:
//  Quartus inferred all three as block RAM (build of 06b7ea9: 4 M10K, 552 / 553).
(* ramstyle = "logic" *) reg [31:0] upd [0:15];
reg [3:0]  uhead, utail;
reg [23:0] si_r0, si_r1, si_r2, si_r3;
reg [23:0] so_r0, so_r1, so_r2, so_r3;

assign status = {!s_host, pc != 8'd0, uhead == utail};                 // konamigx.cpp:1161

assign dbg_s_host   = s_host;
assign dbg_hidx     = hidx;
assign dbg_in_cload = in_cload;

// ---- st1 fields, tms57002.h:78-92 and tms57kdec.cpp:32-76 ---------------------------
wire       f_sfai = st1[1];
wire       f_sfao = st1[2];
wire       f_aovm = st1[3];
wire       f_movm = st1[5];
wire [1:0] f_sfma = st1[8:7];
wire [1:0] f_sfmo = st1[12:11];
wire [2:0] f_rnd  = (st1[17:15] <= 3'd4) ? st1[17:15] : 3'd0;
wire [1:0] f_crm  = st1[19:18];
wire       f_dbp  = st1[20];

// ============================================================================
//  memories: registered write request, registered read, read-during-write bypass
// ============================================================================
(* ramstyle = "MLAB, no_rw_check" *) reg [23:0] pmem  [0:255];
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] cmem  [0:255];
(* ramstyle = "MLAB, no_rw_check" *) reg [23:0] dmem0 [0:255];
(* ramstyle = "MLAB, no_rw_check" *) reg [23:0] dmem1 [0:31];

reg        pm_we;  reg [7:0] pm_wa;  reg [23:0] pm_wd;
reg        cm_we;  reg [7:0] cm_wa;  reg [31:0] cm_wd;
reg        d0_we;  reg [7:0] d0_wa;  reg [23:0] d0_wd;
reg        d1_we;  reg [4:0] d1_wa;  reg [23:0] d1_wd;
reg [7:0]  pm_ra, cm_ra, d0_ra;
reg [4:0]  d1_ra;
reg [23:0] pmem_q, dmem0_q, dmem1_q, d0_bv, d1_bv;
reg [31:0] cmem_q, cm_bv;
reg        cm_bp, d0_bp, d1_bp;

integer mi;
initial for (mi = 0; mi < 256; mi = mi + 1) begin
    pmem[mi] = 24'd0; cmem[mi] = 32'd0; dmem0[mi] = 24'd0;
    if (mi < 32) dmem1[mi] = 24'd0;
end

always @(posedge clk) begin
    if (pm_we) pmem[pm_wa] <= pm_wd;
    pmem_q <= pmem[pm_ra];
end
always @(posedge clk) begin
    if (cm_we) cmem[cm_wa] <= cm_wd;
    cmem_q <= cmem[cm_ra];
    cm_bp  <= cm_we && cm_wa == cm_ra;
    cm_bv  <= cm_wd;
end
always @(posedge clk) begin
    if (d0_we) dmem0[d0_wa] <= d0_wd;
    dmem0_q <= dmem0[d0_ra];
    d0_bp   <= d0_we && d0_wa == d0_ra;
    d0_bv   <= d0_wd;
end
always @(posedge clk) begin
    if (d1_we) dmem1[d1_wa] <= d1_wd;
    dmem1_q <= dmem1[d1_ra];
    d1_bp   <= d1_we && d1_wa == d1_ra;
    d1_bv   <= d1_wd;
end
wire [31:0] cmem_rd  = cm_bp ? cm_bv : cmem_q;
wire [23:0] dmem0_rd = d0_bp ? d0_bv : dmem0_q;
wire [23:0] dmem1_rd = d1_bp ? d1_bv : dmem1_q;

// ============================================================================
//  decode tables, from the CDEC sections tmsmake.py generates (tms57kdec.cpp)
// ============================================================================
//  {implemented, reads or writes c, reads or writes d}.  Absent = MAME's
//  decode_error: no op and no xmode call, so no increment either.
function automatic [2:0] c1_info(input [5:0] i);
    case (i)
        6'h01, 6'h02, 6'h34, 6'h35:                                    c1_info = 3'b100;
        6'h03, 6'h05, 6'h09, 6'h0b, 6'h11, 6'h14, 6'h17, 6'h25,
        6'h2a, 6'h31, 6'h32:                                           c1_info = 3'b101;
        6'h04, 6'h06, 6'h0a, 6'h0c, 6'h12, 6'h15, 6'h18, 6'h22,
        6'h26, 6'h2e, 6'h33, 6'h39:                                    c1_info = 3'b110;
        6'h07, 6'h0d, 6'h16, 6'h19, 6'h21, 6'h24, 6'h28, 6'h29, 6'h38: c1_info = 3'b111;
        default:                                                       c1_info = 3'b000;
    endcase
endfunction

//  category 2a (the pre op): {implemented, c, d}
function automatic [2:0] c2a_info(input [6:0] i);
    case (i)
        7'h01, 7'h05, 7'h31:                                           c2a_info = 3'b110;
        7'h02, 7'h03, 7'h06, 7'h07, 7'h0f, 7'h10, 7'h11, 7'h12, 7'h13: c2a_info = 3'b101;
        7'h08, 7'h09, 7'h0e, 7'h20, 7'h21, 7'h22, 7'h23:               c2a_info = 3'b100;
        default:                                                       c2a_info = 3'b000;
    endcase
endfunction

//  The post-op increments of ca and id, the expressions X and Z evaluated before
//  2026-09-15, as functions of the opcode so F can decode them.  Used only for
//  category 1 / 2a instructions (X gates them with !x_cat3; Z never sees cat 3).
function automatic inc_ca_of(input [23:0] o);
    reg [2:0] a, b;
    begin
        a = c1_info(o[23:18]);
        b = c2a_info(o[17:11]);
        inc_ca_of = ((a[2] && a[1]) || (b[2] && b[1])) && (o[10] ? (!o[8] && o[7]) : o[9]);
    end
endfunction
function automatic inc_id_of(input [23:0] o);
    reg [2:0] a, b;
    begin
        a = c1_info(o[23:18]);
        b = c2a_info(o[17:11]);
        inc_id_of = ((a[2] && a[0]) || (b[2] && b[0])) && (!o[10] ? (!o[8] && o[7]) : o[9]);
    end
endfunction
//  The category-1 ops that call get_cmem unconditionally (rde / wre call it only
//  when not busy -- cc_x_q).
function automatic called_of(input [5:0] i);
    case (i)
        6'h04, 6'h06, 6'h07, 6'h0a, 6'h0c, 6'h0d, 6'h12, 6'h15, 6'h16,
        6'h18, 6'h19, 6'h21, 6'h22, 6'h24, 6'h26, 6'h28, 6'h29, 6'h2e,
        6'h33:   called_of = 1'b1;
        default: called_of = 1'b0;
    endcase
endfunction

function automatic is_mul(input [5:0] i);
    case (i)
        6'h21, 6'h22, 6'h24, 6'h25, 6'h26, 6'h28, 6'h29, 6'h2a, 6'h2e: is_mul = 1'b1;
        default:                                                      is_mul = 1'b0;
    endcase
endfunction

//  category 2b (the post op): st1 only, tmsinstr.lst 'f' instructions and rmov
function automatic [23:0] post_st1(input [23:0] s, input [6:0] i);
    post_st1 = s;
    case (i)
        7'h3a: post_st1[6]     = 1'b0;                // rmov   :354
        7'h3c: post_st1[3]     = 1'b0;                // raom   :549
        7'h3d: post_st1[3]     = 1'b1;                // saom   :554
        7'h40: post_st1[5]     = 1'b0;                // rmom   :350
        7'h41: post_st1[5]     = 1'b1;                // smom   :493
        7'h44: post_st1[20]    = 1'b0;                // ldpk 0 :201
        7'h45: post_st1[20]    = 1'b1;                // ldpk 1 :205
        7'h48, 7'h49, 7'h4a, 7'h4b: post_st1[19:18] = i[1:0];   // scrm :402-416
        7'h50: post_st1[2]     = 1'b0;                // sfao   :426
        7'h51: post_st1[2]     = 1'b1;
        7'h54: post_st1[1]     = 1'b0;                // sfai   :418
        7'h55: post_st1[1]     = 1'b1;
        7'h58, 7'h59, 7'h5a, 7'h5b: post_st1[8:7]   = i[1:0];   // sfma :434-448
        7'h60, 7'h61, 7'h62, 7'h63: post_st1[12:11] = i[1:0];   // sfmo :454-468
        7'h68, 7'h69, 7'h6a, 7'h6b,
        7'h6c, 7'h6d, 7'h6e, 7'h6f: post_st1[17:15] = i[2:0];   // rnd  :358-388
        default: ;
    endcase
endfunction

// ---- %mo and %mv: macc_to_output_*, check_macc_overflow_*, tms57002.cpp:353-681 --
localparam [63:0] M_OV0 = 64'h000f800000000000, M_OV1 = 64'h000fe00000000000,
                  M_OV2 = 64'h000ff80000000000;
localparam [63:0] SAT_N = 64'hffff800000000000, SAT_P = 64'h00007fffffffffff;

function automatic [64:0] f_mo(input [63:0] m_in, input [1:0] sfmo, input [2:0] rnd, input movm);
    reg [63:0] m, m1, rounding, rmask;
    reg        over;
    begin
        m = m_in; over = 1'b0;
        case (rnd)                                                     // tmsmake.py:48-57
            3'd1:    begin rounding = 64'h0000000000008000; rmask = 64'hffffffffffff0000; end
            3'd2:    begin rounding = 64'h0000000000800000; rmask = 64'hffffffffff000000; end
            3'd3:    begin rounding = 64'h0000000000020000; rmask = 64'hfffffffffffc0000; end
            3'd4:    begin rounding = 64'h0000000080000000; rmask = 64'hffffffff00000000; end
            default: begin rounding = 64'd0;                rmask = 64'hffffffffffffffff; end
        endcase
        case (sfmo)
            2'd0: begin m1 = m & M_OV0; if (m1 != 64'd0 && m1 != M_OV0) over = 1'b1; end
            2'd1: begin m1 = m & M_OV1; if (m1 != 64'd0 && m1 != M_OV1) over = 1'b1; m = m << 2; end
            2'd2: begin m1 = m & M_OV2; if (m1 != 64'd0 && m1 != M_OV2) over = 1'b1; m = m << 4; end
            default: m = $unsigned($signed(m) >>> 8);
        endcase
        m  = (m + rounding) & rmask;
        m1 = m & M_OV0;
        if (m1 != 64'd0 && m1 != M_OV0) over = 1'b1;
        if (over && movm) m = m[51] ? SAT_N : SAT_P;
        f_mo = {over, m};
    end
endfunction

function automatic [64:0] f_mv(input [63:0] m_in, input [1:0] sfmo, input movm);
    reg [63:0] m, mask;
    reg        over;
    begin
        m = m_in; over = 1'b0;
        case (sfmo)
            2'd0:    mask = M_OV0;
            2'd1:    mask = M_OV1;
            2'd2:    mask = M_OV2;
            default: mask = 64'd0;
        endcase
        if (mask != 64'd0 && (m & mask) != 64'd0 && (m & mask) != mask) over = 1'b1;
        if (over && movm) m = m[51] ? SAT_N : SAT_P;
        f_mv = {over, m};
    end
endfunction

// ---- %wa, tmsmake.py:70-75: {AOV, aacc} ------------------------------------------
function automatic [32:0] f_wa(input [63:0] r_in, input aovm);
    reg [63:0] r;
    reg        ov;
    begin
        r  = r_in;
        ov = ($signed(r) < -64'sd2147483648) || ($signed(r) > 64'sd2147483647);
        if (ov && aovm) r = r[63] ? 64'hffffffff80000000 : 64'h000000007fffffff;
        f_wa = {ov, r[31:0]};
    end
endfunction

// ---- xm_init, tms57002.cpp:237-257 ---------------------------------------------------
function automatic [19:0] f_xadr(input [31:0] o, input [18:0] b, input [23:0] s0);
    reg [31:0] a;
    reg [19:0] mask;
    begin
        a = o + {13'd0, b};
        a = s0[14] ? (a << 2) : (a << 1);                              // ST0_WORD
        if (!s0[15]) a = a << 1;                                       // ST0_SEL
        case (s0[17:16])
            2'd0:    mask = 20'h0ffff;
            2'd1:    mask = 20'h3ffff;
            2'd2:    mask = 20'hfffff;
            default: mask = 20'h00000;                                 // no case in MAME's switch
        endcase
        f_xadr = a[19:0] & mask;
    end
endfunction

//  Steps an access takes: xm_step_read / xm_step_write's `done`, :259-351
function automatic [2:0] f_xn(input word, input sel);
    f_xn = word ? (sel ? 3'd3 : 3'd6) : (sel ? 3'd2 : 3'd4);
endfunction

//  The bytes a write puts down, step k in [47-8k -: 8]
function automatic [47:0] f_xwd(input [23:0] w, input word, input sel);
    if (word && sel)       f_xwd = {w, 24'd0};
    else if (word)         f_xwd = {4'h0, w[23:20], 4'h0, w[19:16], 4'h0, w[15:12],
                                    4'h0, w[11:8],  4'h0, w[7:4],   4'h0, w[3:0]};
    else if (sel)          f_xwd = {w[23:8], 32'd0};
    else                   f_xwd = {4'h0, w[23:20], 4'h0, w[19:16], 4'h0, w[15:12], 4'h0, w[11:8], 16'd0};
endfunction

//  txrd once a read's last step lands
function automatic [23:0] f_xrd(input [47:0] b, input word, input sel);
    if (word && sel)       f_xrd = b[47:24];
    else if (word)         f_xrd = {b[43:40], b[35:32], b[27:24], b[19:16], b[11:8], b[3:0]};
    else if (sel)          f_xrd = {b[47:32], 8'd0};
    else                   f_xrd = {b[43:40], b[35:32], b[27:24], b[19:16], 8'd0};
endfunction

//  Addressing, xmode (tms57kdec.cpp:15-30): the field named by bit 10 is
//  "primary" -- direct (param) with bit 8, else through ca/id with bit 7 as the
//  increment -- and the other is always through ca/id with bit 9 as increment.
function automatic [7:0] f_cidx(input [23:0] o, input [7:0] c);
    f_cidx = (o[10] && o[8]) ? o[7:0] : c;
endfunction
function automatic [7:0] f_didx(input [23:0] o, input [7:0] d);
    f_didx = (!o[10] && o[8]) ? o[7:0] : d;
endfunction

// ============================================================================
//  state
// ============================================================================
localparam [3:0] Q_IDLE = 4'd0, Q_UPD = 4'd1, Q_F = 4'd2, Q_P = 4'd3, Q_RA = 4'd4,
                 Q_X = 4'd5, Q_Y = 4'd6, Q_Z = 4'd7;
reg [3:0]  q;
reg        tick_pend;
reg [15:0] dly;
reg [15:0] icount;

reg [23:0] op_q;
reg [7:0]  c_a;                    // the instruction's cmem index
reg [7:0]  d_a0;                   // dmem0 index
reg [4:0]  d_a1;                   // dmem1 index
reg        fwd_c, fwd_d;           // the pre op wrote the word the category-1 op reads
reg [31:0] fwd_cv;
reg [23:0] fwd_dv;
reg [63:0] mo_q, mv_q;
reg        mo_ov, mv_ov;
//  ---- decisions registered a clock early, 2026-09-15 -------------------------------
//  Build of c9d3949: setup -0.452 ns, every failing path in this module --
//    c_a -> get_cmem's "sa == c_a && uhead != utail" -> the Cv pick -> %wa sum
//    aacc -> the branch compare -> pc_n -> pmem's read address
//    op_q -> the post-op increment decode -> ca + 1
//  The DSP side sees the host's sa / uhead one clock late (sa_d, uhead_d: the
//  host writing one clock later, which MAME's order allows -- a host access
//  never lands inside an instruction), so the get_cmem compare can be made in
//  the clock before X (hit_q).  aacc does not change between an instruction's
//  end and the next one's X (F and P never write it), so its sign and zero
//  flags can lag a clock.  The increments are decoded at F from the opcode.
//  Under Verilator every use is checked against the live expression.
reg [7:0]  sa_d;
reg [3:0]  uhead_d;
reg        hit_q;
reg        agz_q, alz_q, anz_q;
reg        inc_ca_q, inc_id_q;
//  Build of d780933: -0.110 ns, op_q -> get_cmem's call decode -> Cv -> %wa sum,
//  and -0.050 ns, utail -> upd[utail] -> Cv -> f_xadr -> qadr.  So the call
//  decode, the branch decode and is_mul are made at F as well, and the update
//  queue's entry for X is read a clock early (updv_q, loaded where hit_q is).
//  An entry the DSP may take was written at least two edges earlier: it becomes
//  visible through uhead_d a clock after uhead moves.
reg        cc_q, cc_x_q;           // get_cmem is called: always / unless busy (rde, wre)
reg [31:0] updv_q;
reg        cat3_q, mul_q;
reg [4:0]  brk_q;                  // {bv, bnz, blz, bgz, b}

reg signed [32:0] mul_a, mul_b;
reg        [5:0]  mul_sh;
reg               mul_acc;
//  ---- the %wa ops finish in Z, 2026-09-15 ------------------------------------------
//  Build of 06b7ea9: setup -3.832 ns at 96 MHz, all 400 worst paths op_q -> aacc --
//  decode, the get_cmem pick, a 64-bit add, f_wa's range compare and saturation
//  and the aacc mux in the one X clock.  X now registers the exact sum (MAME adds
//  int64s of values that fit 49 bits: a 32-bit operand and %mo >> 16 of a 64-bit
//  macc, so 50 bits are exact) and Z does the range check, the saturation, AOV /
//  MOV and aacc, then ends the instruction as it does for a multiply.  One clock
//  more for abs, neg, add and sub only; this game's DSP program uses none of them.
reg signed [49:0] wa_r;
reg               wa_mov;
reg               z_wa;
function automatic is_wa(input [5:0] i);
    case (i)
        6'h01, 6'h02, 6'h03, 6'h04, 6'h05, 6'h06, 6'h07,
        6'h09, 6'h0a, 6'h0b, 6'h0c, 6'h0d:                             is_wa = 1'b1;
        default:                                                      is_wa = 1'b0;
    endcase
endfunction
reg signed [65:0] prod;
//  Both operands signed, so the product is sign-extended to the 66-bit context.
always @(posedge clk) prod <= mul_a * mul_b;

// external memory: the access in progress, MAME's step counter, the queue
reg [2:0]  xm_k, xm_nn;
reg        xm_word, xm_sel;
reg [47:0] rd_buf;
reg        rd_ok;
reg [1:0]  rd_tag, iss_tag;
reg [1:0]  qh, qt;
reg [2:0]  qcnt;
(* ramstyle = "logic" *) reg        qwe  [0:3];
(* ramstyle = "logic" *) reg [19:0] qadr [0:3];
(* ramstyle = "logic" *) reg [2:0]  qn   [0:3];
(* ramstyle = "logic" *) reg [47:0] qwd  [0:3];
(* ramstyle = "logic" *) reg [1:0]  qtag [0:3];

// host control levels, previous values (edge = pload_w / cload_w with a change)
reg        pl_q, cl_q, rl_q;

`ifdef VERILATOR
integer run_insns = 0, run_xr = 0, run_xw = 0;
`endif

// ---- the next pc, combinational, so the opcode is ready at F ------------------------
wire [23:0] op_x   = op_q;
wire        x_cat3 = op_x[23:18] == 6'h3f;
wire [6:0]  x_id2  = op_x[17:11];
reg         br_take;
//  b :54, bgz :58, blz :67, bnz :73, bv :79 -- cat3_q / brk_q are op_q's decode,
//  loaded with it at F
always @* begin
    br_take = cat3_q && (brk_q[0] || (brk_q[1] && agz_q) || (brk_q[2] && alz_q) ||
                         (brk_q[3] && anz_q) || (brk_q[4] && st1[0]));
end
`ifdef VERILATOR
always @(posedge clk)
    if ((q == Q_X || q == Q_Z) &&
        (cat3_q != x_cat3 || mul_q != is_mul(op_x[23:18]) ||
         brk_q != {x_id2 == 7'h78, x_id2 == 7'h60, x_id2 == 7'h58, x_id2 == 7'h50, x_id2 == 7'h48})) begin
        $display("gx_tms57002: the opcode flags decoded at F differ (pc %02x)", pc);
        $stop;
    end
`endif
wire [7:0] pc_after = br_take ? op_x[7:0] : pc;
wire       sb_after = s_branch | br_take;
//  execute_run :896-910
wire [7:0] pc_n     = (rptc != 8'd0 || sb_after) ? pc_after : pc_after + 8'd1;
wire       eoi_now  = (q == Q_X && (cat3_q || !mul_q)) || (q == Q_Z);

// ---- memory read addresses ----------------------------------------------------------
always @* begin
    pm_ra = eoi_now ? pc_n : pc;
    case (q)
        Q_F: begin
            cm_ra = f_cidx(pmem_q, ca);
            d0_ra = f_didx(pmem_q, id) + ba0;
            d1_ra = 5'(f_didx(pmem_q, id) + ba1);
        end
        Q_RA: begin
            cm_ra = f_cidx(op_q, ca);
            d0_ra = f_didx(op_q, id) + ba0;
            d1_ra = 5'(f_didx(op_q, id) + ba1);
        end
        default: begin
            cm_ra = c_a; d0_ra = d_a0; d1_ra = d_a1;
        end
    endcase
end

// ============================================================================
//  the machine
// ============================================================================
reg [23:0] st1_v, sinm;
reg [31:0] A, Cv, Dx, dsf, raw;
reg [23:0] D24;
reg [7:0]  ca_v, id_v, hidx_b;
reg [63:0] r64, ml;
reg [32:0] wa;
reg [3:0]  utail_v;
reg        s_update_v, called_c, hold, s_idle_v, qpush, qpush_we, busy;
reg [7:0]  rptc_v, rptc_next_v;
reg [15:0] icount_v;
reg [19:0] xadr_v;
reg        in_pload_v, in_cload_v, su_cval_v, s_host_v;
reg [1:0]  su_v;
reg [2:0]  hidx_v;
reg [7:0]  h0, h1, h2, h3;
reg [31:0] v32;
reg [1:0]  qt_v, qh_v;
reg [2:0]  qcnt_v;

function automatic [23:0] no_sim(input [23:0] s);                       // tms57002.cpp:927
    reg signed [23:0] v;
    begin
        v = $signed(s);
        no_sim = $unsigned(24'(v / 24'sd256));
    end
endfunction

always @(posedge clk) begin
    pm_we <= 1'b0; cm_we <= 1'b0; d0_we <= 1'b0; d1_we <= 1'b0;
    dbg_overrun <= 1'b0;
    sa_d  <= sa;  uhead_d <= uhead;
    agz_q <= $signed(aacc) > 32'sd0;  alz_q <= aacc[31];  anz_q <= aacc != 32'd0;

    if (rst) begin
        // device_start (:941-943, constructor :23-33) then device_reset (:76-102)
        in_pload <= 1'b0; in_cload <= 1'b0; su_cval <= 1'b0; su <= 2'd0;
        s_idle <= 1'b1; s_read <= 1'b0; s_write <= 1'b0; s_branch <= 1'b0;
        s_host <= 1'b0; s_update <= 1'b0; susp <= 1'b1;
        st0 <= 24'd0; st1 <= 24'd0;
        pc <= 8'd0; ca <= 8'd0; id <= 8'd0; ba0 <= 8'd0; ba1 <= 8'd0; sa <= 8'd0;
        rptc <= 8'd0; rptc_next <= 8'd0;
        aacc <= 32'd0; xba <= 19'd0;
        macc <= 64'd0; macc_write <= 64'd0;
        xrd <= 24'd0;
        z_wa <= 1'b0;
        hit_q <= 1'b0; inc_ca_q <= 1'b0; inc_id_q <= 1'b0;
`ifdef VERILATOR
        creg <= 32'd0; xoa <= 32'd0; macc_read <= 64'd0; xwr <= 24'd0;
`endif
        host0 <= 8'd0; host1 <= 8'd0; host2 <= 8'd0; host3 <= 8'd0; hidx <= 3'd0;
        uhead <= 4'd0; utail <= 4'd0;
        si_r0 <= 24'd0; si_r1 <= 24'd0; si_r2 <= 24'd0; si_r3 <= 24'd0;
        so_r0 <= 24'd0; so_r1 <= 24'd0; so_r2 <= 24'd0; so_r3 <= 24'd0;
        so0 <= 24'd0; so1 <= 24'd0; so2 <= 24'd0; so3 <= 24'd0;
        data_out <= 8'hff;
        q <= Q_IDLE; tick_pend <= 1'b0; dly <= 16'd0; icount <= 16'd0;
        fwd_c <= 1'b0; fwd_d <= 1'b0;
        xm_k <= 3'd0; xm_nn <= 3'd3; rd_ok <= 1'b0; rd_tag <= 2'd0;
        xm_req <= 1'b0; qh <= 2'd0; qt <= 2'd0; qcnt <= 3'd0;
        pl_q <= 1'b1; cl_q <= 1'b1; rl_q <= 1'b1;
    end else begin
        qt_v = qt; qh_v = qh; qcnt_v = qcnt;

        // ------------------------------------------------------------------ run
        if ((q == Q_IDLE || q == Q_UPD) && (tick || tick_pend)) begin
            // the stream update, :923-939
            if (q == Q_UPD) dbg_overrun <= 1'b1;
            so0 <= so_r0; so1 <= so_r1; so2 <= so_r2; so3 <= so_r3;
            si_r0 <= st0[3] ? si0 : no_sim(si0);
            si_r1 <= st0[3] ? si1 : no_sim(si1);
            si_r2 <= st0[3] ? si2 : no_sim(si2);
            si_r3 <= st0[3] ? si3 : no_sim(si3);
            if (!in_pload) begin                                               // sync_w :218-235
                pc <= 8'd0; ca <= 8'd0; id <= 8'd0;
                if (!st0[0]) begin ba0 <= ba0 - 8'd1; ba1 <= ba1 + 8'd1; end
                xba <= xba - 19'd1;
                st1[0] <= 1'b0; st1[6] <= 1'b0;
                s_idle <= 1'b0;
            end
            tick_pend <= 1'b0;
            dly <= (START_DELAY > 1) ? 16'(START_DELAY - 1) : 16'd0;
            q   <= Q_UPD;
`ifdef VERILATOR
            run_insns = 0; run_xr = 0; run_xw = 0;
`endif
        end else begin
            if (tick && q != Q_IDLE) begin
                tick_pend   <= 1'b1;
                dbg_overrun <= 1'b1;
            end
            case (q)
                Q_UPD:
                    if (dly != 16'd0) dly <= dly - 16'd1;
                    else if (!susp && !in_pload && !s_idle) begin              // :841
                        q      <= Q_F;
                        icount <= 16'(CYCLES);
                    end else
                        q <= Q_IDLE;

                // ---- F: opcode, the external-memory step, the macc pipeline ------
                Q_F: begin
                    busy = s_read | s_write;
                    if (busy && s_read && (xm_k == xm_nn - 3'd1) && !rd_ok) begin
                        // the read's last step needs its bytes: wait (only here)
                    end else begin
                        op_q <= pmem_q;
                        inc_ca_q <= inc_ca_of(pmem_q);
                        inc_id_q <= inc_id_of(pmem_q);
                        cat3_q   <= pmem_q[23:18] == 6'h3f;
                        mul_q    <= is_mul(pmem_q[23:18]);
                        brk_q    <= {pmem_q[17:11] == 7'h78, pmem_q[17:11] == 7'h60, pmem_q[17:11] == 7'h58,
                                     pmem_q[17:11] == 7'h50, pmem_q[17:11] == 7'h48};
                        cc_q     <= called_of(pmem_q[23:18]);
                        cc_x_q   <= pmem_q[23:18] == 6'h38 || pmem_q[23:18] == 6'h39;
                        if (busy) begin                                        // :852-858
`ifdef VERILATOR
                            if (s_read) run_xr = run_xr + 1; else run_xw = run_xw + 1;
`endif
                            if (xm_k == xm_nn - 3'd1) begin
                                if (s_read) begin
                                    xrd   <= f_xrd(rd_buf, xm_word, xm_sel);
                                    rd_ok <= 1'b0;
                                end
                                s_read  <= 1'b0;
                                s_write <= 1'b0;
                                xm_k    <= 3'd0;
                            end else
                                xm_k <= xm_k + 3'd1;
                        end
`ifdef VERILATOR
                        macc_read  <= macc_write;                              // :860-861
`endif
                        macc_write <= macc;
                        {mo_ov, mo_q} <= f_mo(macc_write, f_sfmo, f_rnd, f_movm);
                        {mv_ov, mv_q} <= f_mv(macc_write, f_sfmo, f_movm);
                        c_a  <= f_cidx(pmem_q, ca);
                        d_a0 <= f_didx(pmem_q, id) + ba0;
                        d_a1 <= 5'(f_didx(pmem_q, id) + ba1);
`ifdef VERILATOR
                        run_insns = run_insns + 1;
`endif
                        q <= Q_P;
                    end
                end

                // ---- P: the pre op, category 2a ---------------------------------
                Q_P: begin
                    A = f_sfao ? {aacc[24:0], 7'd0} : aacc;                   // %a
                    fwd_c <= 1'b0; fwd_d <= 1'b0;
                    // for X: c_a stays, utail moves only if lpc takes an update below
                    hit_q <= (sa == c_a) && (uhead != utail);
                    updv_q <= upd[utail];
                    if (op_q[23:18] != 6'h3f) begin
                        case (op_q[17:11])
                            7'h01: begin                                       // sacc %c   :394
                                cm_we <= 1'b1; cm_wa <= c_a; cm_wd <= A;
                                fwd_c <= 1'b1; fwd_cv <= A;
                            end
                            7'h02, 7'h03, 7'h06, 7'h07, 7'h0f,
                            7'h10, 7'h11, 7'h12, 7'h13: begin
                                case (op_q[17:11])
                                    7'h02:   D24 = A[31:8];                    // sacd  :398
                                    7'h03:   D24 = mo_q[47:24];                // smhd  :486
                                    7'h06:   D24 = {mv_q[47:32], 8'h00};       // slmh  :474
                                    7'h07:   D24 = mv_q[31:8];                 // slml  :478
                                    7'h0f:   D24 = xrd;                        // srbd  :497
                                    7'h10:   D24 = si_r0;                      // dis   :113
                                    7'h11:   D24 = si_r1;
                                    7'h12:   D24 = si_r2;
                                    default: D24 = si_r3;
                                endcase
                                if (op_q[17:11] == 7'h03) st1[6] <= st1[6] | mo_ov;
                                if (op_q[17:11] == 7'h06 || op_q[17:11] == 7'h07) st1[6] <= st1[6] | mv_ov;
                                if (f_dbp) begin d1_we <= 1'b1; d1_wa <= d_a1; d1_wd <= D24; end
                                else       begin d0_we <= 1'b1; d0_wa <= d_a0; d0_wd <= D24; end
                                fwd_d <= 1'b1; fwd_dv <= D24;
                            end
                            7'h05: begin                                       // smhc %c   :482
                                cm_we <= 1'b1; cm_wa <= c_a; cm_wd <= mo_q[47:16];
                                fwd_c <= 1'b1; fwd_cv <= mo_q[47:16];
                                st1[6] <= st1[6] | mo_ov;
                            end
                            7'h08: ca <= A[31:24];                             // lcaa      :188
                            7'h09: id <= A[31:24];                             // lira      :212
                            7'h20: begin so_r0 <= mo_q[47:24]; st1[6] <= st1[6] | mo_ov; end   // domh :129
                            7'h21: begin so_r1 <= mo_q[47:24]; st1[6] <= st1[6] | mo_ov; end
                            7'h22: begin so_r2 <= mo_q[47:24]; st1[6] <= st1[6] | mo_ov; end
                            7'h23: begin so_r3 <= mo_q[47:24]; st1[6] <= st1[6] | mo_ov; end
                            7'h31: if (!s_host) begin                          // lpc %c    :232
                                // get_cmem, :683-709
                                if (s_update || (sa_d == c_a && uhead_d != utail)) begin
                                    Cv = upd[utail];
                                    cm_we <= 1'b1; cm_wa <= c_a; cm_wd <= Cv;
                                    fwd_c <= 1'b1; fwd_cv <= Cv;
                                    utail   <= utail + 4'd1;
                                    s_update <= (utail + 4'd1) != uhead_d;
                                    hit_q    <= (sa == c_a) && (uhead != utail + 4'd1);
                                    updv_q   <= upd[utail + 4'd1];
                                end else
                                    Cv = (f_crm == 2'd1) ? {cmem_rd[31:16], 16'd0} :
                                         (f_crm == 2'd2) ? {cmem_rd[15:0], 16'd0} : cmem_rd;
                                host0 <= Cv[31:24]; host1 <= Cv[23:16]; host2 <= Cv[15:8]; host3 <= Cv[7:0];
                                hidx   <= 3'd0;
                                s_host <= 1'b1;
                            end
                            default: ;
                        endcase
                    end
                    q <= (op_q[23:18] != 6'h3f && (op_q[17:11] == 7'h08 || op_q[17:11] == 7'h09)) ? Q_RA : Q_X;
                end

                // ---- RA: lcaa / lira moved ca / id; read again through them ----------
                Q_RA: begin
                    c_a  <= f_cidx(op_q, ca);
                    d_a0 <= f_didx(op_q, id) + ba0;
                    d_a1 <= 5'(f_didx(op_q, id) + ba1);
                    fwd_c <= 1'b0; fwd_d <= 1'b0;
                    hit_q <= (sa == f_cidx(op_q, ca)) && (uhead != utail);
                    updv_q <= upd[utail];
                    q <= Q_X;
                end

                // ---- X: the category-1 or category-3 op ---------------------------
                Q_X: begin
                    A    = f_sfao ? {aacc[24:0], 7'd0} : aacc;
                    raw  = fwd_c ? fwd_cv : cmem_rd;
                    D24  = fwd_d ? fwd_dv : (f_dbp ? dmem1_rd : dmem0_rd);
                    Dx   = {D24, 8'd0};                                        // %d
                    dsf  = f_sfai ? {Dx[31], Dx[31:1]} : Dx;                   // %sfai
                    busy = s_read | s_write;
                    st1_v = st1; ca_v = ca; id_v = id;
                    utail_v = utail; s_update_v = s_update;
                    s_idle_v = s_idle; rptc_next_v = rptc_next;
                    called_c = 1'b0; hold = 1'b0; qpush = 1'b0; qpush_we = 1'b0;
                    Cv = raw;
                    // a held X runs again with c_a and utail unchanged
                    hit_q <= (sa == c_a) && (uhead != utail);
                    updv_q <= upd[utail];

                    // an access that would not fit the queue waits before it starts
                    if (!x_cat3 && (op_q[23:18] == 6'h38 || op_q[23:18] == 6'h39) && !busy && qcnt == 3'd4)
                        hold = 1'b1;

                    if (!hold) begin
                        if (!x_cat3) begin
`ifdef VERILATOR
                            if (cc_q != called_of(op_q[23:18]) ||
                                cc_x_q != (op_q[23:18] == 6'h38 || op_q[23:18] == 6'h39)) begin
                                $display("gx_tms57002: get_cmem call decode differs at X (pc %02x)", pc);
                                $stop;
                            end
`endif
                            called_c = cc_q || (cc_x_q && !busy);
                            if (called_c) begin                                // get_cmem :683-709
`ifdef VERILATOR
                                if (hit_q != (sa_d == c_a && uhead_d != utail_v)) begin
                                    $display("gx_tms57002: hit_q %0d but get_cmem's compare %0d (pc %02x)",
                                             hit_q, sa_d == c_a && uhead_d != utail_v, pc);
                                    $stop;
                                end
`endif
                                if (s_update_v || hit_q) s_update_v = 1'b1;
                                if (s_update_v) begin
`ifdef VERILATOR
                                    if (updv_q != upd[utail_v]) begin
                                        $display("gx_tms57002: updv_q %08x but upd[utail] %08x (pc %02x)",
                                                 updv_q, upd[utail_v], pc);
                                        $stop;
                                    end
`endif
                                    Cv = updv_q;
                                    cm_we <= 1'b1; cm_wa <= c_a; cm_wd <= Cv;
                                    utail_v = utail_v + 4'd1;
                                    if (uhead_d == utail_v) s_update_v = 1'b0;
                                end else
                                    Cv = (f_crm == 2'd1) ? {raw[31:16], 16'd0} :
                                         (f_crm == 2'd2) ? {raw[15:0], 16'd0} : raw;
                            end

                            case (op_q[23:18])
                                //  the %wa ops: the exact sum here, the rest in Z (see wa_r)
                                6'h01: begin wa_r <= 50'($signed(A));                              wa_mov <= 1'b0;  end   // abs       :1 (negated in Z)
                                6'h02: begin wa_r <= 50'd0 - {18'd0, A};                           wa_mov <= 1'b0;  end   // neg :317, (int64_t) of the u32
                                6'h03: begin wa_r <= 50'($signed(Dx))  + 50'($signed(A));          wa_mov <= 1'b0;  end   // add %d,a  :10
                                6'h04: begin wa_r <= 50'($signed(Cv))  + 50'($signed(A));          wa_mov <= 1'b0;  end   // add %c,a  :14
                                6'h05: begin wa_r <= 50'($signed(dsf)) + 50'($signed(mo_q) >>> 16); wa_mov <= mo_ov; end   // add %d,m  :18
                                6'h06: begin wa_r <= 50'($signed(Cv))  + 50'($signed(mo_q) >>> 16); wa_mov <= mo_ov; end   // add %c,m  :23
                                6'h07: begin wa_r <= 50'($signed(Dx))  + 50'($signed(Cv));         wa_mov <= 1'b0;  end   // add %d,%c :27
                                6'h09: begin wa_r <= 50'($signed(Dx))  - 50'($signed(A));          wa_mov <= 1'b0;  end   // sub %d,a  :504
                                6'h0a: begin wa_r <= 50'($signed(Cv))  - 50'($signed(A));          wa_mov <= 1'b0;  end   // sub %c,a  :508
                                6'h0b: begin wa_r <= 50'($signed(dsf)) - 50'($signed(mo_q) >>> 16); wa_mov <= mo_ov; end   // sub %d,m  :512
                                6'h0c: begin wa_r <= 50'($signed(Cv))  - 50'($signed(mo_q) >>> 16); wa_mov <= mo_ov; end   // sub %c,m  :517
                                6'h0d: begin wa_r <= 50'($signed(Dx))  - 50'($signed(Cv));         wa_mov <= 1'b0;  end   // sub %d,%c :521
                                6'h11: aacc <= dsf;                            // lacd      :180
                                6'h12: aacc <= Cv;                             // lacc      :176
                                6'h14: aacc <= aacc & dsf;                     // and %d,a  :40
                                6'h15: aacc <= aacc & Cv;                      // and %c,a  :45
                                6'h16: aacc <= Cv & dsf;                       // and %d,%c :49
                                6'h17: aacc <= aacc | dsf;                     // or %d,a   :321
                                6'h18: aacc <= aacc | Cv;                      // or %c,a   :326
                                6'h19: aacc <= Cv | dsf;                       // or %d,%c  :330
                                //  the multiplies: c into mul_a, d into mul_b, :248-315
                                6'h21: begin mul_a <= 33'($signed(Cv)); mul_b <= 33'($signed(D24)); mul_sh <= 6'd7;  mul_acc <= 1'b0; end  // mpy %d,%c :292
                                6'h22: begin mul_a <= 33'($signed(Cv)); mul_b <= 33'($signed(A));   mul_sh <= 6'd15; mul_acc <= 1'b0; end  // mpy %c,a  :301
                                6'h24: begin mul_a <= 33'($signed(Cv)); mul_b <= 33'($signed(D24)); mul_sh <= 6'd7;  mul_acc <= 1'b1; end  // mac %d,%c :248
                                6'h25: begin mul_a <= 33'($signed(A));  mul_b <= 33'($signed(D24)); mul_sh <= 6'd7;  mul_acc <= 1'b1; end  // mac a,%d  :257
                                6'h26: begin mul_a <= 33'($signed(Cv)); mul_b <= 33'($signed(A));   mul_sh <= 6'd15; mul_acc <= 1'b1; end  // mac %c,a  :266
                                6'h28: begin mul_a <= 33'($signed(Cv)); mul_b <= {9'd0, D24};       mul_sh <= 6'd7;  mul_acc <= 1'b0; end  // mpyu      :310
                                6'h29: begin mul_a <= 33'($signed(Cv)); mul_b <= {9'd0, D24};       mul_sh <= 6'd7;  mul_acc <= 1'b1; end  // macu %d,%c :278
                                6'h2a: begin mul_a <= 33'($signed(A));  mul_b <= {9'd0, D24};       mul_sh <= 6'd7;  mul_acc <= 1'b1; end  // macu a,%d :285
                                6'h2e: begin mul_a <= 33'($signed(Cv)); mul_b <= 33'($signed(A));   mul_sh <= 6'd14; mul_acc <= 1'b1; end  // macs      :272
                                6'h31: begin r64 = 64'($signed(Dx)) << 16; macc <= r64; macc_write <= r64; end   // lmhd :224
                                6'h32: begin r64 = {macc[63:24], D24};     macc <= r64; macc_write <= r64; end   // lmld :228
                                6'h33: begin r64 = 64'($signed(Cv)) << 16; macc <= r64; macc_write <= r64; end   // lmhc :220
                                6'h34: macc <= (macc & 64'h0008000000000000) | ((macc << 1) & 64'h0007ffffffffffff);          // sfml :450
                                6'h35: macc <= (macc & 64'h0008000000000000) | ($unsigned($signed(macc) >>> 1) & 64'h0007ffffffffffff);  // sfmr :470
                                6'h38: if (!busy) begin                        // wre %d,%c :525
`ifdef VERILATOR
                                    xwr    <= D24;
                                    xoa    <= Cv;
`endif
                                    xadr_v = f_xadr(Cv, xba, st0);
                                    s_write <= 1'b1;
                                    xm_k   <= 3'd0; xm_nn <= f_xn(st0[14], st0[15]);
                                    xm_word <= st0[14]; xm_sel <= st0[15];
                                    qpush = 1'b1; qpush_we = 1'b1;
                                    qwe[qt_v] <= 1'b1; qadr[qt_v] <= xadr_v; qn[qt_v] <= f_xn(st0[14], st0[15]);
                                    qwd[qt_v] <= f_xwd(D24, st0[14], st0[15]); qtag[qt_v] <= rd_tag;
                                end
                                6'h39: if (!busy) begin                        // rde %c    :338
`ifdef VERILATOR
                                    xoa    <= Cv;
`endif
                                    xadr_v = f_xadr(Cv, xba, st0);
                                    s_read <= 1'b1;
                                    xm_k   <= 3'd0; xm_nn <= f_xn(st0[14], st0[15]);
                                    xm_word <= st0[14]; xm_sel <= st0[15];
                                    rd_ok  <= 1'b0;
                                    rd_tag <= rd_tag + 2'd1;
                                    qpush = 1'b1;
                                    qwe[qt_v] <= 1'b0; qadr[qt_v] <= xadr_v; qn[qt_v] <= f_xn(st0[14], st0[15]);
                                    qwd[qt_v] <= 48'd0; qtag[qt_v] <= rd_tag + 2'd1;
                                end
                                default: ;
                            endcase
                            if (qpush) begin qt_v = qt_v + 2'd1; qcnt_v = qcnt_v + 3'd1; end
`ifdef VERILATOR
                            //  creg, the multiply's c operand (tmsinstr.lst :248-315)
                            if (is_mul(op_q[23:18]))
                                creg <= (op_q[23:18] == 6'h25 || op_q[23:18] == 6'h2a) ? A : Cv;
`endif
                        end else begin
                            `ifdef VERILATOR
                            if (agz_q != ($signed(aacc) > 32'sd0) || alz_q != aacc[31] || anz_q != (aacc != 32'd0)) begin
                                $display("gx_tms57002: the aacc flags lag a write at a branch (pc %02x)", pc);
                                $stop;
                            end
`endif
                            case (x_id2)                                       // category 3
                                7'h08: s_idle_v = 1'b1;                        // idle :169
                                7'h10: rptc_next_v = op_q[7:0];                // rptk :390
                                7'h18: ca_v = op_q[7:0];                       // lcak :197
                                7'h20: id_v = op_q[7:0];                       // lirk :216
                                7'h40: if ($signed(aacc) >= 32'sd0) ca_v = op_q[7:0];   // lcac :192
                                7'h78: if (st1[0]) st1_v[0] = 1'b0;            // bv   :79
                                default: ;
                            endcase
                        end

                        utail    <= utail_v;
                        s_update <= s_update_v;
                        rptc_next <= rptc_next_v;
                        if (!x_cat3 && (is_mul(op_q[23:18]) || is_wa(op_q[23:18]))) begin
                            st1  <= st1_v;
                            ca   <= ca_v; id <= id_v;
                            z_wa <= is_wa(op_q[23:18]);
                            q    <= is_wa(op_q[23:18]) ? Q_Z : Q_Y;
                        end else begin
                            // post op, increments, next pc: the end of the instruction
                            if (!x_cat3) begin
`ifdef VERILATOR
                                if (inc_ca_q != inc_ca_of(op_q) || inc_id_q != inc_id_of(op_q)) begin
                                    $display("gx_tms57002: increments decoded at F differ at X (pc %02x)", pc);
                                    $stop;
                                end
`endif
                                st1_v = post_st1(st1_v, op_q[17:11]);
                                // ca_v / id_v are still ca / id: only category 3 moves them
                                if (inc_ca_q) ca_v = ca + 8'd1;
                                if (inc_id_q) id_v = id + 8'd1;
                            end
                            st1 <= st1_v;
                            ca  <= ca_v; id <= id_v;
                            s_idle <= s_idle_v;
                            // :896-916
                            rptc_v = rptc;
                            if (rptc != 8'd0)   rptc_v = rptc - 8'd1;
                            else if (sb_after)  s_branch <= 1'b0;
                            pc <= pc_n;
                            if (rptc_next_v != 8'd0) begin rptc_v = rptc_next_v; rptc_next <= 8'd0; end
                            if (rptc != 8'd0 && br_take) s_branch <= 1'b1;
                            rptc <= rptc_v;
                            icount_v = icount - 16'd1;
                            icount <= icount_v;
                            q <= (icount_v != 16'd0 && !s_idle_v && !in_pload && !susp && !tick && !tick_pend) ? Q_F : Q_IDLE;
                        end
                    end
                end

                Q_Y: q <= Q_Z;

                // ---- Z: macc from the product, the end of a multiply instruction ------
                Q_Z: begin
                    st1_v = st1;
                    if (z_wa) begin
                        if (op_q[23:18] == 6'h01) begin                        // abs :1
                            v32 = wa_r[31:0];
                            if ($signed(v32) < 32'sd0) begin
                                v32 = -v32;
                                if ($signed(v32) < 32'sd0) st1_v[0] = 1'b1;
                            end
                            aacc <= v32;
                        end else begin                                          // %wa, tmsmake.py:70-75
                            wa = f_wa(64'(wa_r), f_aovm);
                            st1_v[0] = st1_v[0] | wa[32];
                            st1_v[6] = st1_v[6] | wa_mov;
                            aacc <= wa[31:0];
                        end
                    end else begin
                        case (f_sfma)                                          // %ml, tmsmake.py:37
                            2'd0:    ml = macc;
                            2'd1:    ml = macc << 2;
                            2'd2:    ml = macc << 4;
                            default: ml = $unsigned($signed(macc) >>> 16);
                        endcase
                        //  Three constant shifts, not a 6-bit barrel: build of 06b7ea9,
                        //  the multiplier's pipeline -> macc[51] at -0.216 ns.
                        case (mul_sh)
                            6'd7:    r64 = $unsigned($signed(prod[63:0]) >>> 7);
                            6'd14:   r64 = $unsigned($signed(prod[63:0]) >>> 14);
                            default: r64 = $unsigned($signed(prod[63:0]) >>> 15);
                        endcase
                        macc <= mul_acc ? ml + r64 : r64;
                    end
`ifdef VERILATOR
                    if (inc_ca_q != inc_ca_of(op_q) || inc_id_q != inc_id_of(op_q)) begin
                        $display("gx_tms57002: increments decoded at F differ at Z (pc %02x)", pc);
                        $stop;
                    end
`endif
                    st1_v = post_st1(st1_v, op_q[17:11]);
                    ca_v = inc_ca_q ? ca + 8'd1 : ca;
                    id_v = inc_id_q ? id + 8'd1 : id;
                    st1 <= st1_v;
                    ca  <= ca_v; id <= id_v;
                    rptc_v = rptc;
                    if (rptc != 8'd0)  rptc_v = rptc - 8'd1;
                    else if (s_branch) s_branch <= 1'b0;
                    pc <= pc_n;
                    if (rptc_next != 8'd0) begin rptc_v = rptc_next; rptc_next <= 8'd0; end
                    rptc <= rptc_v;
                    icount_v = icount - 16'd1;
                    icount <= icount_v;
                    q <= (icount_v != 16'd0 && !s_idle && !in_pload && !susp && !tick && !tick_pend) ? Q_F : Q_IDLE;
                end

                default: ;   // Q_IDLE
            endcase
        end

        // ---------------------------------------------- the external-memory transport
        if (xm_req && xm_ack) begin
            xm_req <= 1'b0;
            if (!xm_we && iss_tag == rd_tag && !(q == Q_X && qpush && !qpush_we)) begin
                rd_buf <= xm_rdata;
                rd_ok  <= 1'b1;
            end
            qh_v = qh_v + 2'd1; qcnt_v = qcnt_v - 3'd1;
        end else if (!xm_req && qcnt != 3'd0) begin
            xm_req   <= 1'b1;
            xm_we    <= qwe[qh];
            xm_adr   <= qadr[qh];
            xm_n     <= qn[qh];
            xm_wdata <= qwd[qh];
            iss_tag  <= qtag[qh];
        end
        qh <= qh_v; qt <= qt_v; qcnt <= qcnt_v;

        // ---------------------------------------------------- the host port, last
        in_pload_v = in_pload; in_cload_v = in_cload; su_cval_v = su_cval; su_v = su;
        hidx_v = hidx; s_host_v = s_host;
        h0 = host0; h1 = host1; h2 = host2; h3 = host3;

        //  pload_w / cload_w act on a change (:40-74).  The inputs are the control
        //  byte's bits, and the IN_ flags are their inverse.
        if (ctrl_pload != pl_q) begin
            pl_q <= ctrl_pload;
            in_pload_v = !ctrl_pload;
            if (!ctrl_pload) begin
                hidx_v = 3'd0; pc <= 8'd0; ca <= 8'd0; su_v = 2'd0;
            end
        end
        if (ctrl_cload != cl_q) begin
            cl_q <= ctrl_cload;
            in_cload_v = !ctrl_cload;
            if (!ctrl_cload) hidx_v = 3'd0;
        end

        if (data_wr) begin                                                     // data_w :104-168
            case ({in_cload_v, in_pload_v})
                2'b00: begin hidx_v = 3'd0; su_cval_v = 1'b0; end
                2'b01: begin
                    case (hidx_v[1:0]) 2'd0: h0 = data_in; 2'd1: h1 = data_in; 2'd2: h2 = data_in; default: h3 = data_in; endcase
                    hidx_v = hidx_v + 3'd1;
                    if (hidx_v >= 3'd3) begin
                        hidx_v = 3'd0;
                        case (su_v)
                            2'd0: begin st0 <= {h0, h1, h2}; su_v = 2'd1; end
                            2'd1: begin st1 <= {h0, h1, h2}; su_v = 2'd2; end
                            default: begin
                                pm_we <= 1'b1; pm_wa <= pc; pm_wd <= {h0, h1, h2};
                                pc <= pc + 8'd1;
                            end
                        endcase
                    end
                end
                2'b10: begin
                    if (su_cval_v) begin
                        case (hidx_v[1:0]) 2'd0: h0 = data_in; 2'd1: h1 = data_in; 2'd2: h2 = data_in; default: h3 = data_in; endcase
                        hidx_v = hidx_v + 3'd1;
                        if (hidx_v >= 3'd4) begin
                            su_cval_v = 1'b0;
                            upd[uhead] <= {h0, h1, h2, h3};
                            uhead <= uhead + 4'd1;
                            hidx_v = 3'd1;                                     // :146
                        end
                    end else begin
                        sa <= data_in;
                        hidx_v = 3'd0;
                        su_cval_v = 1'b1;
                    end
                end
                default: begin
                    case (hidx_v[1:0]) 2'd0: h0 = data_in; 2'd1: h1 = data_in; 2'd2: h2 = data_in; default: h3 = data_in; endcase
                    hidx_v = hidx_v + 3'd1;
                    if (hidx_v >= 3'd4) begin
                        hidx_v = 3'd0;
                        cm_we <= 1'b1; cm_wa <= ca; cm_wd <= {h0, h1, h2, h3};
                        ca <= ca + 8'd1;
                    end
                end
            endcase
        end

        if (data_rd) begin                                                     // data_r :170-186
            if (!s_host_v)
                data_out <= 8'hff;
            else begin
                case (hidx_v[1:0]) 2'd0: data_out <= h0; 2'd1: data_out <= h1; 2'd2: data_out <= h2; default: data_out <= h3; endcase
                hidx_v = hidx_v + 3'd1;
                if (hidx_v == 3'd4) begin hidx_v = 3'd0; s_host_v = 1'b0; end
            end
        end

        //  DEFECT A, the second door (MEASUREMENTS 172).  Between the end of the
        //  program load (FC) and the start of the coefficient load (F0) the DSP
        //  is not suspended, so a tick can start the new program; an instruction
        //  still in flight when pload / cload starts would then finish AFTER the
        //  host reset pc / ca / hidx above and move the coefficient (or program)
        //  pointer.  MAME's instructions are whole (execute_run): drop it.
        //
        //  PLOAD START ONLY (2026-10-06, MEASUREMENTS 179).  Door 2 is F0, where
        //  pload and cload start TOGETHER and pload_w zeroes pc / ca (:40-56);
        //  that edge is the pload one.  A cload start on its own resets only
        //  hidx (cload_w :58-73, "ca = 0" is commented out there), which no
        //  instruction writes, so there is nothing to race.  And the games
        //  raise cload alone all through play to update coefficients: dropping
        //  the instruction there lost one DSP instruction per update and the
        //  sample it belonged to -- ~12 one-sample spikes a second in
        //  gokuparo's attract on the board (bcf30cbc .. c86d87c4).
        if (ctrl_pload != pl_q && !ctrl_pload) q <= Q_IDLE;

        if (ctrl_pload != pl_q || ctrl_cload != cl_q || data_wr || data_rd) begin
            in_pload <= in_pload_v; in_cload <= in_cload_v; su_cval <= su_cval_v; su <= su_v;
            hidx <= hidx_v; s_host <= s_host_v;
            host0 <= h0; host1 <= h1; host2 <= h2; host3 <= h3;
        end

        //  RESET: asserting suspends (diexec.cpp:717-729); releasing a suspended
        //  device runs device_reset (:76-102).
        rl_q <= reset_line;
        susp <= reset_line;
        if (rl_q && !reset_line) begin
            //  DEFECT A (MEASUREMENTS 172).  An instruction still in flight here --
            //  one waiting in Q_F for an external read, freed by s_read <= 0
            //  below -- would finish AFTER this reset and write pc <= pc_n,
            //  moving the program download that follows by a word.  MAME runs
            //  instructions whole (tms57002.cpp execute_run), so nothing of the
            //  old program outlives device_reset: drop it.
            q <= Q_IDLE;
            su <= 2'd0; s_read <= 1'b0; s_write <= 1'b0; s_branch <= 1'b0;
            s_host <= 1'b0; s_update <= 1'b0; s_idle <= 1'b1;
            pc <= 8'd0; ca <= 8'd0; hidx <= 3'd0; id <= 8'd0; ba0 <= 8'd0; ba1 <= 8'd0;
            sa <= 8'd0; rptc <= 8'd0; rptc_next <= 8'd0; uhead <= 4'd0; utail <= 4'd0;
            sa_d <= 8'd0; uhead_d <= 4'd0; hit_q <= 1'b0;
            st0 <= st0 & ~24'h003fef;
            st1 <= st1 & ~24'h1f99ef;
            xba <= 19'd0;
`ifdef VERILATOR
            xoa <= 32'd0;
`endif
            xm_k <= 3'd0; rd_ok <= 1'b0; rd_tag <= rd_tag + 2'd1;
        end
    end
end

endmodule

`default_nettype wire
