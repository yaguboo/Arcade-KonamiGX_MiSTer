//============================================================================
//  Konami System GX -- the sound CPU's view of the TMS57002 host port, as a log
//  (DEBUG.  Off by default.  Nothing in the core reads it.)
//
//  ---- why this exists ---------------------------------------------------------
//  Sexy Parodius's RAM CHECK fails on bits 6 and 7, the TMS57002 test at sound
//  program 0x193C, on the board and NOT in sim/tb_gx_sndboot (MEASUREMENTS 60-65;
//  STATUS, the 2026-09-16 handoff).  Everything that could be tried in the
//  simulator has been: every SDRAM latency, every DDR3 latency, the board's own
//  latency spread, byte enables.  What has never been seen on the board is the
//  test's own evidence -- the four bytes the sound CPU reads back at 0x1960..
//  0x1972 and the number of status polls each wait took -- because the only
//  witness is the main CPU's on-screen text, one step removed.
//
//  This module watches the four things the sound CPU does to the DSP's host
//  port and turns each into one 64-bit word.  gx_sound hands the words to the
//  target, which writes them to DDR3 (targets/mister/KonamiGX.sv); the host
//  brings them back with tools/gx_sndtrace.sh.  It logs FACTS at the port and
//  interprets nothing, so it cannot be wrong about what the DSP meant.
//
//  ---- the word -----------------------------------------------------------------
//      [63:60]  type
//      [59:56]  flags  bit 3 = one or more earlier words were LOST (FIFO full)
//      [55:48]  value
//      [47:28]  polls  status-word reads since the previous logged CPU event,
//                      saturating at 0xFFFFF
//      [27:0]   time   the free-running clk count >> 4
//
//    type 1  the STATUS word CHANGED         value = {5'b0, status[2:0]}
//    type 2  a data-port READ completed      value = the byte the CPU received
//    type 3  a data-port WRITE               value = the byte written
//    type 4  a control-word WRITE            value = the byte written
//    type 5  progress, emitted right after every type 1 (a different layout):
//              [59:56] flags   [55:32] overruns so far   [31:0] samples so far
//            "samples" counts smp_tick since reset; MAME's DSP reaches its
//            `lpc` after about 63,600 of them (MEASUREMENTS 61).
//    type 6  snd_run (the main CPU's control bit 22) CHANGED   value = 0 or 1
//            It holds the sound CPU in reset, zeroes sound_ctrl and resets the
//            DSP, so a single blip means no reply at all -- nine BAD lines.
//            The board has it as a live signal and every testbench has it as a
//            constant, so this is the one place it can be observed.
//    type 7  a K056800 access on the SOUND side   (a DIFFERENT layout, below)
//    type 9  a FRAME began -- the video side's vblank, rising edge
//              [55:28] frame   counted since `en`, and counted whether or not
//                              the word got out, so a tick that was dropped
//                              shows as a GAP in the numbers
//              [27:0]  time
//            sexyparo's main CPU does NOT poll once a frame -- MEASUREMENTS 60
//            measured 20 polls over 221 frames, a nested dbra delay between
//            them -- so the poll count is a count and NOT a duration.  Without
//            this tick every frame figure in the report would be clock time
//            divided by a nominal refresh, which is a computed number standing
//            where a measured one belongs.
//    type 10 the DSP's own host port CHANGED (P2, MEASUREMENTS 72)
//              [55]    reserved, always 0 in this build (it carried "an lpc
//                      fired and did not take" until the timing miss of
//                      2026-09-21; the decoder still reads it)
//              [54]    S_HOST now
//              [53]    in_cload    <- the question: does lpc fire in here?
//              [52]    in_pload  -- reserved, always 0 in this build
//              [51]    held in reset -- reserved, always 0 in this build
//              [50:48] hidx
//            A type 5 follows it, as it does a type 1, so every one of these
//            carries the sample number it happened at.  It does NOT reset the
//            poll counter: the wait it lands inside is still the same wait.
//    type 8  a K056800 access on the HOST side -- the MAIN CPU's own mailbox
//            traffic, the only thing in this log that is not the sound CPU's.
//              [55:48] value   the data byte (written, or returned by a read)
//              [47:44] addr    a[4:1] as that CPU gave it; the chip decodes & 7
//              [43]    we      1 = write
//              [42:28] skipped  type 7 only: sound-side READS suppressed since
//                              the previous logged mailbox word, saturating at
//                              32,767.  Type 8 carries zero here.
//                              NOT a DSP poll count and it does not reset one,
//                              or the type-1 word that ends a 160,626-poll wait
//                              would carry a fraction of it: an IRQ2 landing
//                              inside the wait would split the very number
//                              MEASUREMENTS 61 compares with MAME's.
//              [27:0]  time
//
//  A sound-side READ that returns what the previous read of THAT REGISTER
//  returned is counted, not logged -- the same rule the status port has, and
//  for the same reason.  MEASURED on the board, 2026-09-21, RBF 13b1ac65: with
//  every read logged, 51,177 of the log's 65,536 words were the sound CPU's own
//  command-wait loop reading host_to_snd[0], the log was FULL at t = 5.47 s and
//  the 0xFE self-test at about t = 7 s never reached it.  A host-side read is
//  still logged every time: those are the 20 polls of the deadline and losing
//  one loses the measurement.
//
//  ---- why the mailbox is in here (P1, docs/NEXT_SESSION_PROMPT.md) ------------
//  Defect B is a DEADLINE: sexyparo's main CPU gives its sound half 245 frames
//  and the self-test is said to take about 225 (MEASUREMENTS 60, 64).  Both
//  numbers are COMPUTED -- one from a forced control, the other from arithmetic
//  over a one-second screenshot -- and the shortfall they imply disagrees with
//  the one the same section derives from SDRAM latency by about a factor of
//  three.  Types 7 and 8 make both MEASURED, in the board's own units and from
//  a single run:
//
//      type 8, a WRITE of 0xFE to host register 0     the command leaves
//      type 7, the sound CPU READS register 0         the command arrives
//      type 7, WRITES to sound registers 0 and 1      snd_to_host[0]/[1] done
//      type 8, READS of register 0 (0xd52010)         the main CPU's poll loop:
//                                                     20 of them, and the LAST
//                                                     one is the deadline --
//                                                     after it the program runs
//                                                     ori.l #$1FF and the nine
//                                                     lines all read BAD.
//      type 9, the frame ticks between them           the unit both numbers are
//                                                     argued in, measured here
//                                                     rather than converted.
//
//  Two CPUs, one queue: the mux keeps one word when both fire on the same clock
//  and the LOST flag on the next word says the other went missing.
//
//  A status READ that returns the value the previous one did is not logged, only
//  counted: the wait at 0x1948 polls 160,628 times and the log should hold the
//  count, not 160,628 words.  `polls` is reset by every logged CPU event, so the
//  type-1 word that ends a wait carries that wait's whole poll count.
//
//  ---- how each event is detected, and why ---------------------------------------
//  gx_sound hands in strobes; this module does not look at the bus.  A CPU read
//  is logged on the FIRST acknowledge of its bus cycle: dev_ack can pulse more
//  than once while `cyc` stays high, and a poll counted twice would make every
//  count here wrong by a factor nobody could see.  The proof is in the
//  simulation: tb_gx_sndboot prints this module's poll count next to MAME's
//  (482,214).
//============================================================================
`default_nettype none

module gx_sndtrace (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,            // low: hold everything cleared

    input  wire        ev_st,         // one clock: a status read was acknowledged
    input  wire [2:0]  st_val,
    input  wire        ev_dr,         // one clock: a data read was acknowledged
    input  wire [7:0]  dr_val,
    input  wire        ev_dw,         // one clock: a data write
    input  wire [7:0]  dw_val,
    input  wire        ev_cw,         // one clock: a control write
    input  wire [7:0]  cw_val,
    input  wire        smp_tick,
    input  wire        ovr,           // gx_tms57002 dbg_overrun
    input  wire        snd_run,       // control bit 22, a LEVEL: logged on change

    // K056800, both ports.  One-clock strobes, made by gx_sound: the two sides
    // are on two different CPUs' buses and neither of them is this module's.
    input  wire        ev_k56s,       // one clock: an access on the SOUND side
    input  wire [7:0]  k56s_val,
    input  wire [4:1]  k56s_addr,
    input  wire        k56s_we,
    input  wire        ev_k56h,       // one clock: an access on the HOST side
    input  wire [7:0]  k56h_val,
    input  wire [4:1]  k56h_addr,
    input  wire        k56h_we,

    // The video side's vblank, a LEVEL; the rising edge is a frame.  Same
    // clock as everything else here -- gx_top runs its video on `clk` with
    // enables, so this is a wire and not a crossing.
    input  wire        vbl,

    //  ---- the DSP's host port, from INSIDE the chip (type 10, P2) ------------
    //  Every other input here is something the sound CPU did.  These are what
    //  the DSP did about it, and they exist for one question: does `lpc` fire
    //  while `in_cload` is up?  MAME leaves the DSP running through the
    //  coefficient window on purpose and says twice that it is unsure
    //  (UPSTREAM_TODO_AUDIT U43/U44), and defect A appears in that window.
    //  Three wires and no new register on the DSP's side.  An earlier version
    //  also carried an `lpc` pulse, in_pload and the reset line; the build
    //  after it missed timing on an unrelated gx_tilemap path, so the word
    //  keeps their bit positions and sends them as 0.
    input  wire        dsp_s_host,
    input  wire [2:0]  dsp_hidx,
    input  wire        dsp_in_cload,

    output wire        trc_valid,     // a word is waiting
    output wire [63:0] trc_data,
    input  wire        trc_take,      // one clock: the word was written

    output reg  [31:0] dbg_polls      // every status read since `en`, for the testbench
);

// ---------------------------------------------------------------------------
//  INPUT REGISTER STAGE -- every event is taken from a flop in this module
// ---------------------------------------------------------------------------
//  This module taps three regions that are nowhere near each other on the die:
//  the sound CPU's bus, the MAIN CPU's address decode, and the video side's
//  vblank.  Every one of them used to reach the FIFO's write logic
//  combinationally, so the FIFO's setup time carried the routing delay from all
//  three.
//
//  MEASURED, the first build after the mailbox words went in (2026-09-21):
//  SETUP -0.351 ns, and the five worst paths in the whole design were
//
//      gx_main|cpu_a32[22]        -> gx_sndtrace|fifo~222   -0.351
//      fx68k|rFC[0]               -> gx_sndtrace|fifo~222   -0.319
//      fx68k|rFC[0]               -> gx_sndtrace|fifo~231   -0.313
//      gx_main|cpu_a32[22]        -> gx_sndtrace|fifo~218   -0.287
//      gx_main|cpu_a32[22]        -> gx_sndtrace|fifo~227   -0.264
//
//  With this stage the long routes end at a flop and the FIFO's logic starts at
//  one.  It costs ONE CLOCK on every timestamp, which every event pays equally
//  -- the relative order and the intervals are unchanged -- and which is a
//  sixteenth of the log's own 16-clock resolution.
//
//  It is deliberately NOT gated by `en` or `rst`: a pipeline that carries no
//  state worth clearing does not need the reset fan-out, and the block below
//  ignores everything it produces while `en` is low.
reg        i_st, i_dr, i_dw, i_cw, i_k56s, i_k56h, i_vbl_edge;
reg [2:0]  i_st_val;
reg [7:0]  i_dr_val, i_dw_val, i_cw_val, i_k56s_val, i_k56h_val;
reg [4:1]  i_k56s_addr, i_k56h_addr;
reg        i_k56s_we, i_k56h_we;
reg        i_smp, i_ovr, i_run;
reg [14:0] i_k56s_skip;
reg        vbl_q;

//  The filter lives in the input stage so `log_ev` keeps the depth the timing
//  fix above gave it: what reaches the body is already the decision.
//
//  AND IT GETS A STAGE OF ITS OWN IN FRONT.  The mailbox strobe is made from
//  the sound CPU's bus, and feeding it straight into the history compare and
//  the 15-bit suppressed-read counter put the whole chain in one clock:
//  MEASURED 2026-09-21, the build after the borrowed-master exception closed
//  everything else, setup -0.042 with all five worst paths
//  `fx68k|rFC[0] -> gx_sndtrace|k56s_skip[*]`.  The compare and the counter now
//  start at flops.  It costs ONE more clock on a type-7 timestamp than on the
//  other types -- 10 ns against the log's own 167 ns resolution.
//
reg        p_k56s = 1'b0;
reg [7:0]  p_k56s_val;
reg [4:1]  p_k56s_addr;
reg        p_k56s_we;

//  `logic`, like the FIFO below and for the same reason: 64 bits is not a
//  memory.  Left to itself Quartus inferred a RAM node from it and added
//  read-during-write pass-through logic -- Warning (276020), the one warning
//  this lane's baseline of 105 gained on 2026-09-21.
(* ramstyle = "logic" *) reg [7:0]  k56s_hist [0:7];
reg [7:0]  k56s_hseen = 8'd0;      // power-up value: nothing read yet, so the
reg [14:0] k56s_skip  = 15'd0;     // first read of every register is logged
wire [2:0] k56s_r   = p_k56s_addr[3:1];        // the chip's own `offset & 7`
wire       k56s_new = p_k56s && (p_k56s_we || !k56s_hseen[k56s_r]
                                           || k56s_hist[k56s_r] != p_k56s_val);
wire       k56s_sup = p_k56s && !k56s_new;

always @(posedge clk) begin
    i_st        <= ev_st;      i_st_val    <= st_val;
    i_dr        <= ev_dr;      i_dr_val    <= dr_val;
    i_dw        <= ev_dw;      i_dw_val    <= dw_val;
    i_cw        <= ev_cw;      i_cw_val    <= cw_val;
    p_k56s      <= ev_k56s;    p_k56s_val  <= k56s_val;
    p_k56s_addr <= k56s_addr;  p_k56s_we   <= k56s_we;
    i_k56s      <= k56s_new;   i_k56s_val  <= p_k56s_val;
    i_k56s_addr <= p_k56s_addr; i_k56s_we  <= p_k56s_we;
    if (p_k56s && !p_k56s_we) begin
        k56s_hist[k56s_r]  <= p_k56s_val;
        k56s_hseen[k56s_r] <= 1'b1;
    end
    //  new and suppressed are exclusive, so the word carries the count that
    //  led up to it and the counter starts again from zero
    if (k56s_new) begin
        i_k56s_skip <= k56s_skip;
        k56s_skip   <= 15'd0;
    end else if (k56s_sup && k56s_skip != 15'h7FFF)
        k56s_skip <= k56s_skip + 15'd1;
    i_k56h      <= ev_k56h;    i_k56h_val  <= k56h_val;
    i_k56h_addr <= k56h_addr;  i_k56h_we   <= k56h_we;
    vbl_q       <= vbl;        i_vbl_edge  <= vbl && !vbl_q;
    i_smp       <= smp_tick;   i_ovr       <= ovr;
    i_run       <= snd_run;
    i_dhost     <= dsp_s_host; i_dhidx     <= dsp_hidx;
    i_dcl       <= dsp_in_cload;
    //  The only state in this stage worth clearing: a log that starts again
    //  must not inherit a suppression history from the last one.
    if (!en) begin
        k56s_hseen <= 8'd0;
        k56s_skip  <= 15'd0;
    end
end

reg  [31:0] tcnt;
reg  [19:0] polls;
reg  [2:0]  last_st;
reg         st_seen;
reg  [23:0] ovr_n;
reg  [31:0] smp_n;
reg         ovr_d;
reg         lost;

// ---- an eight-word FIFO in registers ---------------------------------------
(* ramstyle = "logic" *) reg [63:0] fifo [0:7];
reg  [2:0]  wp, rp;
reg  [3:0]  cnt;
assign trc_valid = (cnt != 4'd0);
assign trc_data  = fifo[rp];

// One word pending behind a type 1: the type 5 that follows it.
reg         p5;

reg         run_d;
reg         run_seen;
wire        run_chg = !run_seen || (i_run != run_d);

//  ---- the DSP's host port (type 10) -----------------------------------------
//  One event: S_HOST moving, about eighteen times in a boot.  Defect A is one
//  of those rises landing while `in_cload` is up, which the word carries.
reg         i_dhost, i_dcl;
reg  [2:0]  i_dhidx;
reg         dhost_d, dhost_seen;
wire        dsp_chg = !dhost_seen || (i_dhost != dhost_d);

reg  [27:0] frame_n;

wire        st_chg = i_st && (!st_seen || i_st_val != last_st);
wire        log_ev = i_dr | i_dw | i_cw | st_chg | run_chg | i_k56s | i_k56h
                   | i_vbl_edge | dsp_chg;

//  The host side belongs to the OTHER CPU, so it can land on the same clock as
//  a sound event.  The mux below keeps one of them; this marks the other as
//  dropped, so a mailbox timeline that is short by one reads as LOSS and not
//  as a fact about the board.
wire        ev_drop = (i_k56h && (run_chg | dsp_chg | st_chg | i_dr | i_dw | i_cw | i_k56s))
                    | (i_k56s && (run_chg | dsp_chg | st_chg | i_dr | i_dw | i_cw))
                    | (i_vbl_edge && (run_chg | dsp_chg | st_chg | i_dr | i_dw | i_cw
                                             | i_k56s | i_k56h))
                    ;
//  type 10 sits below the sound CPU's events and is HELD rather than dropped
//  when one of them wins, so it never costs a readback byte.  It still sits
//  above the mailbox and frame words, and the three terms above already say so.

reg  [3:0]  w_type;
reg  [7:0]  w_val;
reg  [4:1]  w_addr;
reg         w_we;
reg  [14:0] w_skip;
reg         w_k56;      // this word uses the K056800 layout, not the poll one
reg         w_frm;      // ... and this one uses the frame layout
reg         w_dsp;      // ... and this one is the DSP's, which must not reset polls
always @(*) begin
    w_type = 4'h0;
    w_val  = 8'h00;
    w_addr = 4'h0;
    w_we   = 1'b0;
    w_skip = 15'd0;
    w_k56  = 1'b0;
    w_frm  = 1'b0;
    w_dsp  = 1'b0;
    //  snd_run first: if it moved, everything else on this clock is happening
    //  to a CPU that is being held or released and the change is the news.
    if      (run_chg) begin w_type = 4'h6; w_val = {7'b0, i_run}; end
    else if (st_chg) begin w_type = 4'h1; w_val = {5'b0, i_st_val}; end
    else if (i_dr)  begin w_type = 4'h2; w_val = i_dr_val; end
    else if (i_dw)  begin w_type = 4'h3; w_val = i_dw_val; end
    else if (i_cw)  begin w_type = 4'h4; w_val = i_cw_val; end
    //  The DSP's own word goes BELOW the sound CPU's, because S_HOST falling
    //  is simultaneous with the fourth data read by construction and the
    //  readback byte is the evidence; this word is held and retried instead.
    else if (dsp_chg) begin w_type = 4'hA; w_dsp = 1'b1;
                            w_val = {1'b0, i_dhost, i_dcl, 2'b00, i_dhidx}; end
    else if (i_k56s) begin w_type = 4'h7; w_val  = i_k56s_val;
                            w_addr = i_k56s_addr; w_we = i_k56s_we; w_k56 = 1'b1;
                            w_skip = i_k56s_skip; end
    else if (i_k56h) begin w_type = 4'h8; w_val  = i_k56h_val;
                            w_addr = i_k56h_addr; w_we = i_k56h_we; w_k56 = 1'b1; end
    //  last: a heartbeat gives way to anything that is news, and a tick that
    //  gives way is visible as a gap in the frame numbers
    else if (i_vbl_edge) begin w_type = 4'h9; w_frm = 1'b1; end
end

wire [19:0] polls_now = (i_st && polls != 20'hFFFFF) ? polls + 20'd1 : polls;

// A push and a take in the same clock leave the count where it was.
wire        push = (p5 || log_ev) && (cnt != 4'd8 || trc_take);
wire        take = trc_take && trc_valid;

always @(posedge clk) begin
    ovr_d <= i_ovr;
    if (rst || !en) begin
        tcnt <= 32'd0; polls <= 20'd0; last_st <= 3'd0; st_seen <= 1'b0;
        ovr_n <= 24'd0; smp_n <= 32'd0; lost <= 1'b0;
        wp <= 3'd0; rp <= 3'd0; cnt <= 4'd0; p5 <= 1'b0;
        dbg_polls <= 32'd0; run_d <= 1'b0; run_seen <= 1'b0;
        frame_n <= 28'd0;
        dhost_d <= 1'b0; dhost_seen <= 1'b0;
    end else begin
        tcnt <= tcnt + 32'd1;
        //  counted here and not in the push, so a dropped tick leaves a hole
        //  in the numbers instead of silently shortening the frame count
        if (i_vbl_edge)        frame_n <= frame_n + 28'd1;
        if (i_smp)        smp_n <= smp_n + 32'd1;
        if (i_ovr && !ovr_d && ovr_n != 24'hFFFFFF) ovr_n <= ovr_n + 24'd1;
        if (i_st)           dbg_polls <= dbg_polls + 32'd1;

        if (i_st) begin
            st_seen <= 1'b1;
            last_st <= i_st_val;
        end
        //  Latched whether or not the word got into the FIFO: a level that is
        //  logged once must not be logged again just because the queue was
        //  full, or a full queue turns one blip into a stream.
        run_d    <= i_run;
        run_seen <= 1'b1;
        //  The DSP's level is latched only when its word actually WENT OUT.
        //  Unlike snd_run, type 10 competes with the CPU's events for the mux
        //  and loses on purpose (below), and S_HOST falling lands on the same
        //  clock as the FOURTH data read every single time -- hidx hits 4 and
        //  clears it.  Latching unconditionally therefore dropped one readback
        //  byte per download on the board's first run with the instrument
        //  (18 of the expected 24).  Held here, the word is retried on the
        //  next clock instead, and the readback is whole.
        if (w_dsp) begin
            dhost_d    <= i_dhost;
            dhost_seen <= 1'b1;
        end

        if (p5) begin
            // The follow-up goes first; a CPU event in the same clock cannot
            // happen (a bus cycle lasts many clocks) but if one did it is LOST
            // and says so.
            if (cnt != 4'd8 || trc_take) begin
                fifo[wp] <= {4'h5, lost ? 4'h8 : 4'h0, ovr_n, smp_n};
                wp   <= wp + 3'd1;
                p5   <= 1'b0;
                lost <= 1'b0;
            end else
                lost <= 1'b1;
            if (log_ev) lost <= 1'b1;
        end else if (log_ev) begin
            if (cnt != 4'd8 || trc_take) begin
                fifo[wp] <= w_frm
                    ? {w_type, lost ? 4'h8 : 4'h0, frame_n, tcnt[31:4]}
                    : w_k56
                    ? {w_type, lost ? 4'h8 : 4'h0, w_val, w_addr, w_we, w_skip,
                       tcnt[31:4]}
                    : {w_type, lost ? 4'h8 : 4'h0, w_val, polls_now, tcnt[31:4]};
                wp   <= wp + 3'd1;
                lost <= 1'b0;
                if (st_chg || w_dsp) p5 <= 1'b1;
            end else
                lost <= 1'b1;
            //  after the clear above, so a word dropped on this clock is still
            //  reported on the next one even though this one got through
            if (ev_drop) lost <= 1'b1;
        end

        //  A mailbox access does NOT reset the poll counter.  The wait it lands
        //  inside is still the same wait and its count still has to be whole.
        if (log_ev && !w_k56 && !w_frm && !w_dsp) polls <= 20'd0;
        else if (i_st) polls <= polls_now;

        if (take) rp <= rp + 3'd1;
        cnt <= cnt + (push ? 4'd1 : 4'd0) - (take ? 4'd1 : 4'd0);
    end
end

endmodule

`default_nettype wire
