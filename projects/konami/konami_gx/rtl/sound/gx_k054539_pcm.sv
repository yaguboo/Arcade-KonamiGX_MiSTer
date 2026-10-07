//============================================================================
//  Konami K054539 -- the PCM engine of ONE chip
//
//  Source: MAME src/devices/sound/k054539.cpp, BSD-3-Clause (copyright-holders
//  Olivier Galibert).  `write` (:346-471), `keyon`/`keyoff` (:97-107) and
//  `sound_stream_update` (:109-306) are carried over statement by statement;
//  each block below cites the lines it follows.  tools/gx_k054539_model.py is
//  the same transcription in Python, checked against MAME's own audio
//  (docs/MEASUREMENTS.md 39), and sim/tb_gx_k054539_pcm.sv compares this RTL
//  with it sample for sample.
//
//  docs/REUSE_PLAN.md shelf 2 is empty for this chip: jotego's jt539 is a
//  private repository (github 404).  New RTL, deliberately.
//
//  ---- what it does ----------------------------------------------------------
//  Once per output sample (`tick`, 48 kHz from gx_k054539) it walks the eight
//  channels in order.  An active channel steps its 16.16 position by its pitch,
//  reading one sample byte per step (two for 16-bit PCM) through a req/ack
//  byte port, handles the end marker, the loop point and key-off, decodes 4-bit
//  DPCM, and adds val * volume * pan to the left and right sums.  The chip's
//  own register file stays in gx_k054539 (it echoes every write for the sound
//  self-test); this module keeps the fields it needs and owns the two things
//  MAME's engine writes back: the channel-active bits (0x22c) and each
//  channel's position (0x0c-0x0e).
//
//  ---- the reverb ring, 2026-09-15 (DECISIONS D16) --------------------------
//  k054539.cpp keeps a 0x2000 x 16-bit ring (:118).  Each sample reads the slot
//  at the ring position into both outputs and clears it (:126-129); each active
//  channel adds int16_t(val * rbvol) at (((delay >> 3) + pos) & 0x3fff + pos)
//  & 0x1fff (:164-169, :287) -- the position counted TWICE, MAME's arithmetic
//  and kept (EMULATION_DERIVED; what the chip does is not known); the position
//  steps after the channels (:302).  It was left out when the board had 8 M10K
//  (D13); it now lives in external memory through gx_sndxm (rv_* below), one
//  16-bit slot per op.  The engine skips an op that cannot change a slot: the
//  clear of a slot already 0 and a send that truncates to 0.  MEASURED on the
//  model: 4.2-4.5 reads and 2.7-3.0 writes a sample for both chips.
//
//  ---- what it deliberately does not do ------------------------------------
//  * THE 0x22d ROM/RAM readback and the ring writes through 0x22d with 0x22e =
//    0x80 (:431-437).  gx_k054539 answers reads with zero, as before, and the
//    game's attract loop writes neither (MEASUREMENTS 39's census).
//
//  ---- accuracy marking, root section 1.6 ------------------------------------
//  The stepping, formats, loop and key-off are MAME's reading of the chip.  The
//  volume law is MAME's CHOICE (:509-517) -- EMULATION_DERIVED, tables in
//  gx_k054539_tab.svh.  PURE_RTL: all of it runs in the FPGA.
//============================================================================
`default_nettype none

//  NO ROM INFERENCE, 2026-09-15.  The volume and reverb-send tables are case
//  functions whose results are registered, and Quartus turns each such table
//  into a block ROM: build of d689e7e, 256 x 22 (k539_rbvtab) in each engine,
//  M10K 550 -> 552 of 553, on top of the 256 x 16 voltab blocks already there.
//  As logic they are a few hundred LUTs; the blocks are needed elsewhere.
(* altera_attribute = "-name AUTO_ROM_RECOGNITION OFF" *)
module gx_k054539_pcm (
    input  wire               clk,
    input  wire               rst,
    input  wire               tick,        // one clock per output sample

    // --- register writes, the chip's own numbering ------------------------------
    input  wire               wr,
    input  wire [9:0]         wr_addr,
    input  wire [7:0]         wr_data,

    // --- readback of what the engine owns, one clock after rd_addr --------------
    input  wire [9:0]         rd_addr,
    output reg  [7:0]         rd_data,
    output reg                rd_own,      // rd_data applies (0x22c, or 0x0c-0x0e of a channel)

    // --- sample ROM, one byte per request (req held until ack) ------------------
    output reg                rom_req,
    output reg  [21:0]        rom_addr,
    output reg  [2:0]         rom_ch,      // the channel asking -- the fetcher reads ahead per channel
    output wire               rom_rev,     // and which way it plays
    input  wire               rom_ack,
    input  wire [7:0]         rom_data,

    // --- the reverb ring, one 16-bit slot per op (req held until ack) -----------
    output reg                rv_req,
    output reg                rv_we,
    output reg  [12:0]        rv_slot,
    output reg  [15:0]        rv_wdata,
    input  wire               rv_ack,
    input  wire [15:0]        rv_rdata,

    // --- this chip's output, MAME's put_int scale (32768 = full scale) ----------
    //  TRUNCATED toward zero, as put_int's s32 parameter converts MAME's double
    //  lval (k054539.cpp:303, disound.h put_int) -- it feeds the TMS57002's input
    //  as well as the speaker, and the DSP golden is exact on that (tools/tmsgold).
    output reg  signed [17:0] out_l,
    output reg  signed [17:0] out_r,

    output reg                dbg_overrun, // one clock: a tick came while a sample was being computed

    // --- per-channel gain, MAME k054539_device::set_gain (k054539.cpp:83) -------
    //  Two bits a channel, channel 0 in [1:0]: 0 / 3 = 1.0, 1 = 0.8, 2 = 2.0.
    //  0 everywhere is the chip as MAME models it; gx_sound drives Dragoon Might's
    //  [HACK] gains here only when the OSD asks (MEASUREMENTS 162).
    input  wire [15:0]        ch_gain
);

`include "gx_k054539_tab.svh"

localparam [1:0] K_PCM8 = 2'd0, K_PCM16 = 2'd1, K_DPCM = 2'd2;

// ---- registers the engine reads ---------------------------------------------
reg [23:0] c_delta [0:7];     // 00-02 pitch
//  c_vol and c_pan are read only in ST_LOAD, so Quartus 17.0 infers them as
//  8 x 8 RAMs and places each in a whole block (build 9bb55d2: four blocks,
//  M10K 551 / 553, warning 276020).  Flip-flops: 64 bits each.
(* ramstyle = "logic" *) reg [7:0]  c_vol   [0:7];     // 03
(* ramstyle = "logic" *) reg [7:0]  c_pan   [0:7];     // 05
(* ramstyle = "logic" *) reg [7:0]  c_rvol  [0:7];     // 04 reverb volume
(* ramstyle = "logic" *) reg [15:0] c_rdel  [0:7];     // 06-07 reverb delay
reg [12:0] rpos;              // m_reverb_pos
reg [23:0] c_loop  [0:7];     // 08-0a
reg [23:0] c_rpos  [0:7];     // 0c-0e, as m_regs holds them
reg [23:0] c_latch [0:7];     // m_posreg_latch
reg [7:0]  c_type  [0:7];     // 200 + 2ch: b2-3 type, b5 reverse
reg [7:0]  c_lpf   [0:7];     // 201 + 2ch: b0 loop
reg [7:0]  active;            // 22c
reg        pcm_en;            // 22f b0: enable PCM
reg        reg_hold;          // 22f b7: disable register updates

// ---- channel state (struct channel, k054539.h) -------------------------------
reg signed [25:0] s_pos   [0:7];
reg signed [25:0] s_pfrac [0:7];
reg signed [15:0] s_val   [0:7];
reg signed [15:0] s_pval  [0:7];

// ---- the walk ----------------------------------------------------------------
localparam [4:0] ST_IDLE = 5'd0, ST_LOAD = 5'd1, ST_PREP = 5'd2, ST_CHK = 5'd3,
                 ST_F0 = 5'd4, ST_F0W = 5'd5, ST_F1 = 5'd6, ST_F1W = 5'd7,
                 ST_EVAL = 5'd8, ST_POST = 5'd9, ST_VOL = 5'd10, ST_MUL = 5'd11,
                 ST_ACC = 5'd12, ST_NEXT = 5'd13, ST_OUT = 5'd14, ST_TAB = 5'd15,
                 ST_LOAD2 = 5'd16,
                 // the ring: the read-and-clear at the sample's start ...
                 ST_RV0 = 5'd17, ST_RV0W = 5'd18, ST_RV1 = 5'd19, ST_RV1W = 5'd20,
                 // ... and each active channel's send
                 ST_RB0 = 5'd21, ST_RB1 = 5'd22, ST_RB2 = 5'd23, ST_RB3 = 5'd24,
                 ST_RB4 = 5'd25, ST_RB5 = 5'd26,
                 // set_gain: after the volume product and on the reverb send
                 ST_GAIN = 5'd27, ST_RBG = 5'd28;
reg [4:0]         st;
//  ST_LOAD registers the channel's state; ST_LOAD2 compares.  One state did
//  both: ch -> the 8:1 selects of c_rpos and s_pos -> the compare -> v and the
//  multiplier's input register missed 96 MHz by 0.336 ns in build c4c694b
//  (MEASUREMENTS 43).  A sample has 2,000 clocks; one more per channel is free.
reg signed [25:0] l_spos, l_pf;
reg signed [15:0] l_v, l_pv;
reg               tick_pend;
reg [2:0]         ch;
reg signed [35:0] accl, accr;
reg signed [25:0] pos, pf;
reg signed [15:0] v, pv;
reg [1:0]         kind;
reg               rev, lp, reloaded;
reg [23:0]        w_delta, w_loop;
reg [7:0]         w_vol, w_pan;
reg [7:0]         b0, b1;
reg [15:0]        vt_q;
reg [14:0]        lvol_q, rvol_q;    // at most 16384: voltab[0] x pantab[14]
reg [16:0]        ptl_q, ptr_q;
reg signed [31:0] pl, pr;
reg [7:0]         w_rvol;
reg [15:0]        w_rdel;
reg [22:0]        rbv_q;             // voltab[bval] / 2 x gain, Q24 (k539_rbvtab; x2 needs bit 22)
reg signed [38:0] rb_prod;
reg signed [15:0] rb_send, rb_sum;

wire regupdate = !reg_hold;

// set_gain.  lvol / rvol are Q14 here (16384 = 1.0) and MAME caps them at
// VOL_CAP = 1.80 (k054539.cpp:111, :157) -> 29491.  0.8 is 13107 / 16384.
// The reverb send (rbvol = voltab x gain / 2, :164) never reaches the cap:
// voltab <= 1, so x2 gives at most 1.0.
wire [1:0] ch_g = ch_gain[{ch, 1'b0} +: 2];
function automatic [14:0] gain_vol(input [14:0] x, input [1:0] g);
    reg [15:0] d;
begin
    d = {x, 1'b0};
    case (g)
        2'd1:    gain_vol = 15'((29'(x) * 29'd13107) >> 14);
        2'd2:    gain_vol = (d > 16'd29491) ? 15'd29491 : d[14:0];
        default: gain_vol = x;
    endcase
end
endfunction
function automatic [22:0] gain_rbv(input [22:0] x, input [1:0] g);
begin
    case (g)
        2'd1:    gain_rbv = 23'((37'(x) * 37'd13107) >> 14);
        2'd2:    gain_rbv = {x[21:0], 1'b0};
        default: gain_rbv = x;
    endcase
end
endfunction

// k054539.cpp:141-143 and :164: bval = vol + reverb volume, at most 255
wire [8:0]  rb_bval = {1'b0, w_vol} + {1'b0, w_rvol};
// :168-169 and :287: the slot a channel's send lands in
wire [13:0] rb_dl   = {1'b0, w_rdel[15:3]} + {1'b0, rpos};
wire [12:0] rb_slot = rb_dl[12:0] + rpos;

// C's truncation toward zero -- put_int's s32 (:303) and int16_t(...) (:287)
function automatic signed [17:0] trunc_q16(input signed [35:0] a);
    trunc_q16 = a[35] ? -(18'((-a) >>> 16)) : 18'(a >>> 16);
endfunction
function automatic signed [15:0] trunc_q24(input signed [38:0] a);
    trunc_q24 = a[38] ? -(16'((-a) >>> 24)) : 16'(a >>> 24);
endfunction
assign rom_rev = rev;                                                     // :92

// DPCM steps, k054539.cpp:113
function automatic signed [15:0] dpcm(input [3:0] n);
    case (n)
        4'd0: dpcm = 16'sd0;      4'd1: dpcm = 16'sd256;    4'd2: dpcm = 16'sd512;    4'd3: dpcm = 16'sd1024;
        4'd4: dpcm = 16'sd2048;   4'd5: dpcm = 16'sd4096;   4'd6: dpcm = 16'sd8192;   4'd7: dpcm = 16'sd16384;
        4'd8: dpcm = 16'sd0;      4'd9: dpcm = -16'sd16384; 4'd10: dpcm = -16'sd8192; 4'd11: dpcm = -16'sd4096;
        4'd12: dpcm = -16'sd2048; 4'd13: dpcm = -16'sd1024; 4'd14: dpcm = -16'sd512;  4'd15: dpcm = -16'sd256;
    endcase
endfunction

// pan index, k054539.cpp:145-152
function automatic [3:0] pan_idx(input [7:0] p);
    if (p >= 8'h81 && p <= 8'h8f)      pan_idx = 4'(p - 8'h81);
    else if (p >= 8'h11 && p <= 8'h1f) pan_idx = 4'(p - 8'h11);
    else                               pan_idx = 4'd7;
endfunction

// ---- what the current step reads ---------------------------------------------
wire [23:0]        rd_a0     = (kind == K_DPCM) ? pos[24:1] : pos[23:0];     // read_byte masks to 24 bits
wire [23:0]        rd_a1     = rd_a0 + 24'd1;
wire signed [15:0] dpcm_step = dpcm(pos[0] ? b0[7:4] : b0[3:0]);
wire signed [16:0] dpcm_sum  = {pv[15], pv} + {dpcm_step[15], dpcm_step};
wire signed [15:0] dpcm_val  = (dpcm_sum > 17'sd32767) ? 16'sd32767 : (dpcm_sum < -17'sd32768) ? $signed(16'h8000) : dpcm_sum[15:0];
wire               end_mark  = (kind == K_PCM8)  ? (b0 == 8'h80) :
                               (kind == K_PCM16) ? (b1 == 8'h80 && b0 == 8'h00) :
                                                   (b0 == 8'h88);
wire signed [15:0] step_val  = (kind == K_PCM8)  ? $signed({b0, 8'h00}) :
                               (kind == K_PCM16) ? $signed({b1, b0}) : dpcm_val;
wire signed [25:0] sdelta    = rev ? -$signed({2'b00, w_delta}) : $signed({2'b00, w_delta});
wire signed [25:0] fdelta    = rev ? 26'sh10000 : -26'sh10000;
wire signed [25:0] pdelta    = (kind == K_PCM16) ? (rev ? -26'sd2 : 26'sd2) : (rev ? -26'sd1 : 26'sd1);
wire signed [25:0] pf_x2     = pf <<< 1;

// ---- the channel-active bits (0x22c) ------------------------------------------
//  The engine's key-off first, then the CPU's write: MAME runs the sample, then
//  applies the write (write() calls m_stream->update() before anything else).
wire        eng_keyoff = (st == ST_EVAL) && end_mark && !(lp && !reloaded) && regupdate;
wire [7:0]  wr_keyon   = (wr && wr_addr == 10'h214 && regupdate) ? wr_data : 8'd0;  // :97
wire [7:0]  wr_keyoff  = (wr && wr_addr == 10'h215 && regupdate) ? wr_data : 8'd0;  // :103
reg  [7:0]  act_next;
always @* begin
    act_next = active;
    if (eng_keyoff) act_next[ch] = 1'b0;
    act_next = (act_next | wr_keyon) & ~wr_keyoff;
    if (wr && wr_addr == 10'h22c) act_next = wr_data;
end

integer     i;

always @(posedge clk) begin
    dbg_overrun <= 1'b0;
    if (rst) begin
        st        <= ST_IDLE;
        tick_pend <= 1'b0;
        rom_req   <= 1'b0;
        rv_req    <= 1'b0;
        rv_we     <= 1'b0;
        rpos      <= 13'd0;
        active    <= 8'd0;
        pcm_en    <= 1'b0;
        reg_hold  <= 1'b0;
        out_l     <= 18'sd0;
        out_r     <= 18'sd0;
        for (i = 0; i < 8; i = i + 1) begin
            s_pos[i] <= 26'sd0; s_pfrac[i] <= 26'sd0; s_val[i] <= 16'sd0; s_pval[i] <= 16'sd0;
        end
    end else begin
        if (tick) begin
            if (st != ST_IDLE || tick_pend) dbg_overrun <= 1'b1;
            tick_pend <= 1'b1;
        end

        // ---------------------------------------------------------------- the engine
        case (st)
            ST_IDLE: if (tick_pend && !tick) begin
                tick_pend <= 1'b0;
                if (!pcm_en) begin                                            // :120
                    out_l <= 18'sd0;
                    out_r <= 18'sd0;
                end else begin
                    ch   <= 3'd0;
                    st   <= ST_RV0;
                end
            end

            // ---- lval = rval = rbase[pos]; rbase[pos] = 0   (:125-129) ----------
            ST_RV0: begin
                rv_req  <= 1'b1;
                rv_we   <= 1'b0;
                rv_slot <= rpos;
                st      <= ST_RV0W;
            end
            ST_RV0W: if (rv_ack) begin
                rv_req <= 1'b0;
                accl   <= {{4{rv_rdata[15]}}, rv_rdata, 16'd0};
                accr   <= {{4{rv_rdata[15]}}, rv_rdata, 16'd0};
                st     <= (rv_rdata != 16'd0) ? ST_RV1 : ST_LOAD;
            end
            ST_RV1: begin
                rv_req   <= 1'b1;
                rv_we    <= 1'b1;
                rv_wdata <= 16'd0;
                st       <= ST_RV1W;
            end
            ST_RV1W: if (rv_ack) begin
                rv_req <= 1'b0;
                st     <= ST_LOAD;
            end

            ST_LOAD: if (!active[ch]) st <= ST_NEXT;                           // :132
            else begin
                w_delta <= c_delta[ch];
                w_vol   <= c_vol[ch];
                w_pan   <= c_pan[ch];
                w_rvol  <= c_rvol[ch];
                w_rdel  <= c_rdel[ch];
                w_loop  <= c_loop[ch];
                kind    <= c_type[ch][3:2];
                rev     <= c_type[ch][5];
                lp      <= c_lpf[ch][0];
                pos     <= $signed({2'b00, c_rpos[ch]});                       // :171
                l_spos  <= s_pos[ch];
                l_pf    <= s_pfrac[ch];
                l_v     <= s_val[ch];
                l_pv    <= s_pval[ch];
                st      <= ST_LOAD2;
            end

            ST_LOAD2: begin                                                    // :184
                if (pos != l_spos) begin
                    pf <= 26'sd0; v <= 16'sd0; pv <= 16'sd0;
                end else begin
                    pf <= l_pf; v <= l_v; pv <= l_pv;
                end
                st <= ST_PREP;
            end

            ST_PREP: begin
                reloaded <= 1'b0;
                case (kind)
                    K_PCM8, K_PCM16: begin pf <= pf + sdelta; st <= ST_CHK; end   // :197, :220
                    K_DPCM: begin                                              // :241-248
                        pos <= (pos <<< 1) | {25'd0, pf_x2[16]};
                        pf  <= (pf_x2[16] ? {10'd0, pf_x2[15:0]} : pf_x2) + sdelta;
                        st  <= ST_CHK;
                    end
                    default: st <= ST_POST;                                    // :281 unknown type
                endcase
            end

            ST_CHK: if (pf[25:16] != 10'd0) begin                             // while (cur_pfrac & ~0xffff)
                pf       <= pf + fdelta;
                pos      <= pos + pdelta;
                pv       <= v;
                reloaded <= 1'b0;
                st       <= ST_F0;
            end else
                st <= ST_POST;

            ST_F0: begin
                if (rd_a0 >= 24'h400000) begin                                 // outside the 4 MB region: reads 0
                    b0 <= 8'h00;
                    st <= (kind == K_PCM16) ? ST_F1 : ST_EVAL;
                end else begin
                    rom_req  <= 1'b1;
                    rom_addr <= rd_a0[21:0];
                    rom_ch   <= ch;
                    st       <= ST_F0W;
                end
            end
            ST_F0W: if (rom_ack) begin
                rom_req <= 1'b0;
                b0      <= rom_data;
                st      <= (kind == K_PCM16) ? ST_F1 : ST_EVAL;
            end
            ST_F1: if (!rom_req) begin
                if (rd_a1 >= 24'h400000) begin
                    b1 <= 8'h00;
                    st <= ST_EVAL;
                end else begin
                    rom_req  <= 1'b1;
                    rom_addr <= rd_a1[21:0];
                    rom_ch   <= ch;
                    st       <= ST_F1W;
                end
            end
            ST_F1W: if (rom_ack) begin
                rom_req <= 1'b0;
                b1      <= rom_data;
                st      <= ST_EVAL;
            end

            ST_EVAL: if (end_mark) begin
                if (lp && !reloaded) begin                                     // :204, :227, :255
                    reloaded <= 1'b1;
                    pos      <= (kind == K_DPCM) ? $signed({1'b0, w_loop, 1'b0}) : $signed({2'b00, w_loop});
                    st       <= ST_F0;
                end else begin                                                 // :208, :231, :259
                    // keyoff(ch), :103 -- eng_keyoff clears the active bit
                    v  <= 16'sd0;
                    st <= ST_POST;
                end
            end else begin
                v  <= step_val;
                st <= ST_CHK;
            end

            ST_POST: begin
                if (kind == K_DPCM) begin                                      // :275-278
                    pf  <= (pf >>> 1) | (pos[0] ? 26'sh8000 : 26'sd0);
                    pos <= pos >>> 1;
                end
                st <= ST_TAB;
            end

            ST_TAB: begin                                                      // :145-152
                vt_q  <= k539_voltab(w_vol);
                ptl_q <= k539_pantab(pan_idx(w_pan));
                ptr_q <= k539_pantab(4'd14 - pan_idx(w_pan));
                st    <= ST_VOL;
            end
            ST_VOL: begin                                                      // :156-162, gain 1.0 and never above VOL_CAP
                lvol_q <= 15'((33'(vt_q) * 33'(ptl_q) + 33'd32768) >> 16);
                rvol_q <= 15'((33'(vt_q) * 33'(ptr_q) + 33'd32768) >> 16);
                st     <= ST_GAIN;
            end
            ST_GAIN: begin                                                     // :154-162
                lvol_q <= gain_vol(lvol_q, ch_g);
                rvol_q <= gain_vol(rvol_q, ch_g);
                st     <= ST_MUL;
            end
            ST_MUL: begin
                pl <= v * $signed({1'b0, lvol_q});
                pr <= v * $signed({1'b0, rvol_q});
                st <= ST_ACC;
            end
            ST_ACC: begin                                                      // :285-298
                accl        <= accl + {{4{pl[31]}}, pl};
                accr        <= accr + {{4{pr[31]}}, pr};
                s_pos[ch]   <= pos;
                s_pfrac[ch] <= pf;
                s_val[ch]   <= v;
                s_pval[ch]  <= pv;
                if (regupdate) c_rpos[ch] <= pos[23:0];
                st <= ST_RB0;
            end

            // ---- rbase[(rdelta + pos) & 0x1fff] += int16_t(val * rbvol)  (:287) --
            ST_RB0: begin                                                      // :141-143, :164
                rbv_q <= {1'b0, k539_rbvtab(rb_bval[8] ? 8'hFF : rb_bval[7:0])};
                st    <= ST_RBG;
            end
            ST_RBG: begin                                                      // :164-166
                rbv_q <= gain_rbv(rbv_q, ch_g);
                st    <= ST_RB1;
            end
            ST_RB1: begin
                rb_prod <= 39'(v * $signed({1'b0, rbv_q}));   // at most 2^15 x 2^22: fits
                st      <= ST_RB2;
            end
            ST_RB2: begin
                rb_send <= trunc_q24(rb_prod);
                rv_slot <= rb_slot;
                st      <= (trunc_q24(rb_prod) == 16'sd0) ? ST_NEXT : ST_RB3;
            end
            ST_RB3: begin
                if (!rv_req && !rv_ack) begin
                    rv_req <= 1'b1;
                    rv_we  <= 1'b0;
                end else if (rv_ack) begin
                    rv_req <= 1'b0;
                    rb_sum <= rv_rdata + rb_send;                          // int16 wraps, as the C does
                    st     <= ST_RB4;
                end
            end
            ST_RB4: begin
                rv_req   <= 1'b1;
                rv_we    <= 1'b1;
                rv_wdata <= rb_sum;
                st       <= ST_RB5;
            end
            ST_RB5: if (rv_ack) begin
                rv_req <= 1'b0;
                st     <= ST_NEXT;
            end

            ST_NEXT: if (ch == 3'd7) st <= ST_OUT;
            else begin
                ch <= ch + 3'd1;
                st <= ST_LOAD;
            end

            ST_OUT: begin
                out_l <= trunc_q16(accl);                                      // :303, put_int's s32
                out_r <= trunc_q16(accr);
                rpos  <= rpos + 13'd1;                                         // :302
                st    <= ST_IDLE;
            end

            default: st <= ST_IDLE;
        endcase

        // ------------------------------------------------------- the CPU's writes
        //  After the engine, so a write in the same clock wins -- MAME applies a
        //  write between two samples, never inside one.
        if (wr) begin
            if (pcm_en && wr_addr < 10'h100 &&                                // :370-381
                wr_addr[4:0] >= 5'h0c && wr_addr[4:0] <= 5'h0e) begin
                case (wr_addr[4:0])
                    5'h0c: c_latch[wr_addr[7:5]][7:0]   <= wr_data;
                    5'h0d: c_latch[wr_addr[7:5]][15:8]  <= wr_data;
                    default: c_latch[wr_addr[7:5]][23:16] <= wr_data;
                endcase
            end else begin
                if (wr_addr < 10'h100) begin                                   // m_regs[offset] = data
                    case (wr_addr[4:0])
                        5'h00: c_delta[wr_addr[7:5]][7:0]   <= wr_data;
                        5'h01: c_delta[wr_addr[7:5]][15:8]  <= wr_data;
                        5'h02: c_delta[wr_addr[7:5]][23:16] <= wr_data;
                        5'h03: c_vol[wr_addr[7:5]]          <= wr_data;
                        5'h04: c_rvol[wr_addr[7:5]]         <= wr_data;
                        5'h05: c_pan[wr_addr[7:5]]          <= wr_data;
                        5'h06: c_rdel[wr_addr[7:5]][7:0]    <= wr_data;
                        5'h07: c_rdel[wr_addr[7:5]][15:8]   <= wr_data;
                        5'h08: c_loop[wr_addr[7:5]][7:0]    <= wr_data;
                        5'h09: c_loop[wr_addr[7:5]][15:8]   <= wr_data;
                        5'h0a: c_loop[wr_addr[7:5]][23:16]  <= wr_data;
                        5'h0c: c_rpos[wr_addr[7:5]][7:0]    <= wr_data;
                        5'h0d: c_rpos[wr_addr[7:5]][15:8]   <= wr_data;
                        5'h0e: c_rpos[wr_addr[7:5]][23:16]  <= wr_data;
                        default: ;
                    endcase
                end
                if (wr_addr >= 10'h200 && wr_addr <= 10'h20f) begin
                    if (!wr_addr[0]) c_type[wr_addr[3:1]] <= wr_data;
                    else             c_lpf[wr_addr[3:1]]  <= wr_data;
                end
                if (wr_addr == 10'h214 && pcm_en)                              // :392-404, key-on loads the latch
                    for (i = 0; i < 8; i = i + 1)
                        if (wr_data[i]) c_rpos[i] <= c_latch[i];
                if (wr_addr == 10'h22f) begin
                    pcm_en   <= wr_data[0];
                    reg_hold <= wr_data[7];
                end
            end
        end

        active <= act_next;                                                    // key on / off and 0x22c: act_next
    end
end

// ---- readback -----------------------------------------------------------------
always @(posedge clk) begin
    rd_own  <= (rd_addr == 10'h22c) ||
               (rd_addr < 10'h100 && rd_addr[4:0] >= 5'h0c && rd_addr[4:0] <= 5'h0e);
    rd_data <= (rd_addr == 10'h22c)    ? active :
               (rd_addr[4:0] == 5'h0c) ? c_rpos[rd_addr[7:5]][7:0] :
               (rd_addr[4:0] == 5'h0d) ? c_rpos[rd_addr[7:5]][15:8] :
                                         c_rpos[rd_addr[7:5]][23:16];
end

endmodule

`default_nettype wire
