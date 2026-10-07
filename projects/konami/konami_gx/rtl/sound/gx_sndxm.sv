//============================================================================
//  Konami System GX -- the sound half's external memory: both K054539 reverb
//  rings and the TMS57002's delay RAM on one 64-bit-word port
//
//  ---- why external, and why one port -----------------------------------------
//  The two rings are 0x2000 x 16 bits each (k054539.cpp:118, :129, :287) and
//  the DSP's data RAM is 0x40000 bytes (konamigx.cpp:1196-1199; MEASURED 16
//  word reads and 9 word writes a sample, MEASUREMENTS 45).  With 550 of 553
//  M10K used none of it is block RAM, and SDRAM bus time is what the frame-drop
//  work was paid for (MEASUREMENTS 41-43).  So it goes to a memory nobody else
//  on this board uses; the target decides which (MiSTer: DDR3).  This module is
//  board-side and speaks no platform vocabulary: `xm_*` is one word per op,
//  held request, one-clock acknowledge.
//
//  ---- layout, in words of the xm port ----------------------------------------
//      0x0000-0x07FF   chip 1 ring   slot k -> word k >> 2, lane (k & 3) x 16 bits
//      0x0800-0x0FFF   chip 2 ring
//      0x1000-0x8FFF   DSP data RAM  byte a -> word a >> 3, byte lane a & 7
//  Bytes are little-endian lanes: lane i is xm_wdata/xm_rdata[8i +: 8] with
//  xm_be[i].  A DSP access steps over 2..6 bytes that always sit in one word
//  (tms57002.cpp:237-257: the start address is shifted left by 1 or 2, so the
//  lane offset plus the count never passes 8 -- 3 bytes from lane 0 or 4 in the
//  WORD+SEL mode this game uses).  DSP bytes at or above 0x40000 are not RAM on
//  this board (konamigx.cpp:1198): they read 0 and writes are dropped, without
//  an op.
//
//  ---- what the memory starts as ----------------------------------------------
//  k054539.cpp device_reset (:542) clears the ring and MAME's `.ram()` starts
//  at zero, so after `rst` every word is written to zero before any client is
//  served.  DDR3 is not cleared by anything else between core loads.
//
//  ---- scheduling --------------------------------------------------------------
//  Round-robin over ring 1, ring 2, DSP, one op at a time.  All three have the
//  same deadline (one 48 kHz sample, 2,000 clocks) and none can starve another
//  by more than two ops.  A client holds req until its ack and drops it the
//  clock after; the S_GAP clock keeps it from being granted twice.
//============================================================================
`default_nettype none

module gx_sndxm #(
    parameter logic [21:0] RING2_BASE  = 22'h000800,
    parameter logic [21:0] DSP_BASE    = 22'h001000,
    parameter logic [21:0] CLEAR_WORDS = 22'h009000
) (
    input  wire        clk,
    input  wire        rst,

    // --- the reverb rings, [0] chip 1 and [1] chip 2 --------------------------
    input  wire [1:0]  rv_req,
    input  wire [1:0]  rv_we,
    input  wire [25:0] rv_slot,        // {chip 2, chip 1}, 13 bits each
    input  wire [31:0] rv_wdata,       // {chip 2, chip 1}
    output reg  [1:0]  rv_ack,
    output reg  [15:0] rv_rdata,       // valid with either ack

    // --- the TMS57002's data RAM, gx_tms57002's xm port ----------------------
    input  wire        d_req,
    input  wire        d_we,
    input  wire [19:0] d_adr,
    input  wire [2:0]  d_n,
    input  wire [47:0] d_wdata,        // byte k in [47-8k -: 8]
    output reg         d_ack,
    output reg  [47:0] d_rdata,

    // --- the external memory, one word per op ---------------------------------
    output reg         xm_req,         // held until xm_ack
    output reg         xm_we,
    output reg  [21:0] xm_word,
    output reg  [63:0] xm_wdata,
    output reg  [7:0]  xm_be,
    input  wire        xm_ack,         // one clock; xm_rdata valid on it for a read
    input  wire [63:0] xm_rdata,

    output wire        dbg_clearing
);

localparam [2:0] S_CLR = 3'd0, S_IDLE = 3'd1, S_WAIT = 3'd2, S_GAP = 3'd3;
localparam [1:0] C_R1 = 2'd0, C_R2 = 2'd1, C_DSP = 2'd2;

reg [2:0]  st;
reg [21:0] clr;
reg [1:0]  rr;          // the client after the last one served
reg [1:0]  cur;
reg [1:0]  lane_r;      // ring slot & 3
reg [2:0]  lane_d;      // DSP byte address & 7

assign dbg_clearing = (st == S_CLR);

// ---- the next client, round-robin from rr ------------------------------------
wire [2:0] want = {d_req, rv_req[1], rv_req[0]};
reg  [1:0] pick;
reg        any;
always @(*) begin
    any  = 1'b0;
    pick = C_R1;
    case (rr)
        C_R2:    if (want[1]) begin pick = C_R2;  any = 1'b1; end
                 else if (want[2]) begin pick = C_DSP; any = 1'b1; end
                 else if (want[0]) begin pick = C_R1;  any = 1'b1; end
        C_DSP:   if (want[2]) begin pick = C_DSP; any = 1'b1; end
                 else if (want[0]) begin pick = C_R1;  any = 1'b1; end
                 else if (want[1]) begin pick = C_R2;  any = 1'b1; end
        default: if (want[0]) begin pick = C_R1;  any = 1'b1; end
                 else if (want[1]) begin pick = C_R2;  any = 1'b1; end
                 else if (want[2]) begin pick = C_DSP; any = 1'b1; end
    endcase
end

// ---- a DSP access's bytes in and out of a word -------------------------------
function automatic [63:0] d_place(input [47:0] w, input [2:0] off, input [2:0] n);
    integer k, o, nn;
    begin
        d_place = 64'd0;
        o  = {29'd0, off};
        nn = {29'd0, n};
        for (k = 0; k < 6; k = k + 1)
            if (k < nn && (o + k) < 8) d_place[8 * (o + k) +: 8] = w[47 - 8 * k -: 8];
    end
endfunction

function automatic [7:0] d_mask(input [2:0] off, input [2:0] n);
    integer k, o, nn;
    begin
        d_mask = 8'd0;
        o  = {29'd0, off};
        nn = {29'd0, n};
        for (k = 0; k < 6; k = k + 1)
            if (k < nn && (o + k) < 8) d_mask[o + k] = 1'b1;
    end
endfunction

function automatic [47:0] d_take(input [63:0] w, input [2:0] off, input [2:0] n);
    integer k, o, nn;
    begin
        d_take = 48'd0;
        o  = {29'd0, off};
        nn = {29'd0, n};
        for (k = 0; k < 6; k = k + 1)
            if (k < nn && (o + k) < 8) d_take[47 - 8 * k -: 8] = w[8 * (o + k) +: 8];
    end
endfunction

wire [12:0] slot1  = rv_slot[12:0];
wire [12:0] slot2  = rv_slot[25:13];
wire [12:0] slot_p = (pick == C_R2) ? slot2 : slot1;
wire [15:0] wdat_p = (pick == C_R2) ? rv_wdata[31:16] : rv_wdata[15:0];
reg  [2:0]  d_n_q;

always @(posedge clk) begin
    rv_ack <= 2'b00;
    d_ack  <= 1'b0;
    if (rst) begin
        st     <= S_CLR;
        clr    <= 22'd0;
        rr     <= C_R1;
        xm_req <= 1'b0;
    end else begin
        case (st)
            S_CLR: begin
                xm_req   <= 1'b1;
                xm_we    <= 1'b1;
                xm_word  <= clr;
                xm_wdata <= 64'd0;
                xm_be    <= 8'hFF;
                if (xm_req && xm_ack) begin
                    xm_req <= 1'b0;
                    clr    <= clr + 22'd1;
                    if (clr == CLEAR_WORDS - 22'd1) st <= S_IDLE;
                end
            end

            S_IDLE: if (any) begin
                cur <= pick;
                rr  <= (pick == C_DSP) ? C_R1 : pick + 2'd1;
                case (pick)
                    C_R1, C_R2: begin
                        xm_req   <= 1'b1;
                        xm_we    <= rv_we[pick[0]];
                        xm_word  <= ((pick == C_R2) ? RING2_BASE : 22'd0) + {11'd0, slot_p[12:2]};
                        lane_r   <= slot_p[1:0];
                        xm_wdata <= {48'd0, wdat_p} << {slot_p[1:0], 4'd0};
                        xm_be    <= 8'b0000_0011 << {slot_p[1:0], 1'b0};
                        st       <= S_WAIT;
                    end
                    default: begin
                        lane_d <= d_adr[2:0];
                        d_n_q  <= d_n;
                        if (d_adr >= 20'h40000) begin      // not RAM: no op
                            d_rdata <= 48'd0;
                            d_ack   <= 1'b1;
                            st      <= S_GAP;
                        end else begin
                            xm_req   <= 1'b1;
                            xm_we    <= d_we;
                            xm_word  <= DSP_BASE + {5'd0, d_adr[19:3]};
                            xm_wdata <= d_place(d_wdata, d_adr[2:0], d_n);
                            xm_be    <= d_mask(d_adr[2:0], d_n);
                            st       <= S_WAIT;
                        end
                    end
                endcase
            end

            S_WAIT: if (xm_ack) begin
                xm_req <= 1'b0;
                if (cur == C_DSP) begin
                    d_rdata <= d_take(xm_rdata, lane_d, d_n_q);
                    d_ack   <= 1'b1;
                end else begin
                    rv_rdata         <= 16'(xm_rdata >> {lane_r, 4'd0});
                    rv_ack[cur[0]]   <= 1'b1;
                end
                st <= S_GAP;
            end

            S_GAP: st <= S_IDLE;           // the acked client drops req this clock

            default: st <= S_IDLE;
        endcase
    end
end

endmodule

`default_nettype wire
