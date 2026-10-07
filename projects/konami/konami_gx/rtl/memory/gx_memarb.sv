//============================================================================
//  Konami System GX -- SDRAM arbiter
//
//  ---- this is a sibling lane's module, not a new one -----------------------
//  Copied from projects/vsystem/power_spikes/rtl/memory/ps_memarb.sv, which
//  has been through MiSTer hardware and carries two fixes that were paid for
//  in measurement time.  Root CLAUDE.md section 1.2 shelf 3 and the factory
//  memory `check-sibling-lane-first`: an arbiter written fresh here would have
//  arrived at neither of them.
//
//    1. THE ADDRESS IS LATCHED AT GRANT, not driven from the live client bus.
//       Power Spikes' tilemap reassigns its rom_addr every phase, so a fetch
//       that ran late was silently retargeted mid-transaction.  Our gx_tilemap
//       does the same thing -- rtl/video/gx_tilemap.sv state S_CODE assigns
//       rom4_addr unconditionally -- so this board would have hit it too.
//    2. AN URGENCY OVERRIDE IS A PRIORITY INVERSION with a window around it.
//       Power Spikes gave one to a client with 2,549 clocks of slack and it
//       broke the client with 39.  The port is kept because the mechanism is
//       right; it is tied off here.  Nothing on this board has earned one.
//
//  Under root section 1.4 this WAS a `common/` candidate -- second independent
//  board, same module.  IT HAS NOW DIVERGED: this copy carries a burst pass-
//  through (`burst` in, `dout2` out) that power_spikes does not need, because
//  its 4bpp tile fetch is one word and ours is two.  docs/DECISIONS.md D2.
//
//  That does not kill the promotion, it just says what promoting would cost:
//  the common version has to carry the burst ports and power_spikes has to
//  tie them off.  Worth doing only when a THIRD board wants them -- root
//  section 1.4's rule, applied to the port list rather than to the module.
//
//  ---- the clients on this board --------------------------------------------
//  Priority, highest first.  Only the first three exist today; the rest are
//  named so the order is argued once rather than renegotiated each time a
//  client appears.
//
//    0  cpu       68EC020 program and BIOS fetch.  HIGHEST -- and that is the
//                 opposite of Power Spikes, where the CPU sat second from
//                 bottom.  The reason is that here the CPU is the only client
//                 with nothing buffered behind it: a stalled fetch stops the
//                 machine, while both video clients below have a whole 8-pixel
//                 group of slack.  Power Spikes could not do this because its
//                 ADPCM fetch had a hard 60-clock deadline; GX's PCM is a
//                 K054539 with its own DMA and is not here yet.
//                 TODO(REVISIT): when the K054539 and sprite clients arrive,
//                 this order is the thing to re-argue -- with a measurement,
//                 not a guess.  Power Spikes reordered on guesses twice and
//                 reverted both times.
//    1  tile4     K056832 tile fetch, four planes.  One 32-bit read per layer
//                 per 8 pixels, issued as ONE BURST-2 -- so four bursts and
//                 four singles inside the 128 system clocks that 8 dots at
//                 6 MHz allow.  MEASURED at 77.07 of those 128 clocks by
//                 sim/tb_gx_sdram.sv; as six singles per layer it was 109.54,
//                 which is 85.5 % of the budget for the tilemap alone.  The
//                 slack is real but it is not large, and the deadline is
//                 hard: the shifter loads whether the data came or not.
//    2  tile1     the fifth plane, a separate byte-wide region.  Same deadline
//                 as tile4 and always issued with it.  NOT bursted -- it is
//                 one byte and the word that contains it serves only this
//                 group.
//    3  spare     sound CPU program, sprites, PCM.  Not implemented.
//
//  Grant is held until the controller acks, so a client that raises req must
//  keep req up and its address stable until ack.  Same contract the controller
//  itself has.
//============================================================================
`default_nettype none

module gx_memarb #(
    parameter int N = 4,
    // ---- THE TWO-TIER RETURN, 2026-09-15 (MEASUREMENTS 40, Codex's design) ----
    //  A client whose bit is set here gets its words from a response REGISTERED
    //  beside this arbiter and its ack one clock after the controller's -- one
    //  clock more per SDRAM transaction, nothing more.  A client whose bit is
    //  clear gets the controller's ack on the same clock and must take its words
    //  from gx_sdram's `raw_q` / `raw_w1` itself; `dout` / `dout2` here are then
    //  NOT its data.
    //
    //  Why: `dq_in` (the I/O cell) -> gx_sdram's `dout` mux -> this arbiter ->
    //  every client's register was one shared node between fixed pads and sinks
    //  spread over the chip, and each netlist re-solved it: -0.816 ns on 1fa6167,
    //  +0.370 on 9bb55d2 with no RTL on those paths changed.  With it, `dq_in`
    //  drives only registers that sit beside it (these two, gx_sdram's `w1`, and
    //  the tile adapter's capture), and everything far away is register to
    //  register.  The tile client keeps the same clock because its 128-clock
    //  group is the hard deadline; the soft clients pay the clock.
    //
    //  A registered client still holds `req` through the controller's ack clock
    //  and drops it after its OWN ack.  In the clock between, its request is
    //  masked (`ack_d`), or the arbiter would grant it a second time and issue a
    //  duplicate transaction.  tb_gx_mempath counts the READ commands.
    //
    //  0 (the default) is the old single-tier arbiter exactly: `dout` is the
    //  controller's word on the ack clock.
    parameter logic [31:0] RESP_REG = 32'd0
) (
    input  wire            clk,
    input  wire            rst,

    // --- deadline override -------------------------------------------------
    // A client whose bit is set here is picked before any client whose bit is
    // not, regardless of index.  It exists for exactly one situation: a client
    // with a HARD deadline that normally sits low in the order because it is
    // usually not urgent.
    //
    // ADPCM-A is that client.  jt10 presents a new sample address every 666
    // kHz slot -- about 60 clocks at 40 MHz -- and consumes whatever is on
    // `datain` at the end of it.  A byte that arrives after the slot has moved
    // on is a WRONG NIBBLE, and jt10_acc amplifies ADPCM-A by 7.25x on its way
    // into the mix, so one wrong nibble becomes an output step that correct
    // audio never makes: measured at 15 jumps over 2048 in a single frame,
    // against MAME's ZERO in 2.16 million samples (docs/DEBUG_LOG.md O13).
    //
    // This is not the priority reorder that was tried and reverted on
    // 2026-08-31.  That one moved a client up permanently on a guess.  This
    // leaves the order alone and lets one client jump the queue only in the
    // clocks where it would otherwise miss a deadline -- and only that client,
    // only then.
    //
    // ADPCM-A IS THE ONLY CLIENT THAT MAY SET A BIT HERE.  The sprite engine
    // was given one on 2026-09-02 and it undid O11: an override is a priority
    // inversion with a window around it, and the window was drawn against a
    // line length that a fix one commit later made obsolete.  Measured: 0-383
    // tilemap deadline misses a frame, none of them in the first third, while
    // the sprite engine's own overrun counter read 0 the whole time.  The
    // client with 2,549 clocks of slack does not get to interrupt the client
    // with 39.  See ps_top.sv at `arb_urgent`.
    input  wire [N-1:0]    urgent,

    // --- client side (index 0 = highest priority) --------------------------
    input  wire [N-1:0]        req,
    input  wire [25*N-1:0]     addr,     // flattened: word address per client
    input  wire [16*N-1:0]     din,
    input  wire [N-1:0]        we,
    input  wire [N-1:0]        burst,    // read two words -- gx_sdram's contract
    //  With `burst`: FOUR words (gx_sdram's FOUR-WORD BURST).  Passed through
    //  like `burst`; the extra words exist only on gx_sdram's raw tier, so a
    //  client that sets this must be a raw client (RESP_REG bit clear).
    input  wire [N-1:0]        burst4,
    input  wire [2*N-1:0]      ds,
    output reg  [N-1:0]        ack,
    output wire [15:0]         dout,     // shared: valid for the acked client
    output wire [15:0]         dout2,    // shared: the second word of a burst

    // --- controller side ---------------------------------------------------
    output wire [24:0]     m_addr,
    output wire [15:0]     m_din,
    input  wire [15:0]     m_dout,
    output wire            m_req,
    output wire            m_we,
    output wire            m_burst,
    output wire            m_burst4,
    output wire [1:0]      m_ds,
    input  wire [15:0]     m_dout2,
    input  wire            m_ack,

    // --- observability -----------------------------------------------------
    output reg  [15:0]     dbg_cpu_stall,  // clocks the CPU waited, saturating
    // Per-client: this client has been ASKING and NOT BEING GRANTED for 1024
    // CONSECUTIVE clocks.  A LEVEL, and consecutive rather than cumulative --
    // `dbg_cpu_stall` above is a total, and a total cannot tell "waited a lot,
    // in short bursts" from "waited once and never got in".  Only the second
    // is starvation, and this arbiter can produce it: `pick` is the LOWEST set
    // request, `urgent` is tied to 0 on this board, so a client with a higher
    // index is granted only when every lower one is idle.  There is no aging
    // and no round-robin, so the wait is unbounded by construction.
    output wire [N-1:0]    dbg_starved
);

  // ---------------------------------------------------------------------
  // Winner selection.  Locked while a transaction is in flight so the
  // address the controller latched cannot change under it.
  // ---------------------------------------------------------------------
  reg              busy;
  reg  [$clog2(N)-1:0] sel;

  // The registered clients' acks, one clock behind the controller's, and the
  // words that go with them.  `req_m` hides a registered client from selection
  // for exactly that clock -- see RESP_REG.
  reg  [N-1:0]     ack_d;
  reg  [15:0]      resp_q, resp2_q;
  wire [N-1:0]     reg_mask = RESP_REG[N-1:0];
  wire [N-1:0]     req_m    = req & ~ack_d;

  // Lowest set bit of req -- but an urgent request outranks every
  // non-urgent one.  Two passes, urgent second so it wins.
  reg  [$clog2(N)-1:0] pick;
  reg                  any;
  reg                  any_urgent;
  integer i;
  always @(*) begin
    pick       = '0;
    any        = 1'b0;
    any_urgent = 1'b0;
    for (i = N-1; i >= 0; i = i - 1)
      if (req_m[i]) begin pick = i[$clog2(N)-1:0]; any = 1'b1; end
    for (i = N-1; i >= 0; i = i - 1)
      if (req_m[i] && urgent[i]) begin
        pick       = i[$clog2(N)-1:0];
        any_urgent = 1'b1;
      end
    if (any_urgent) any = 1'b1;
  end

  always @(posedge clk) begin
    if (rst) begin
      busy <= 1'b0;
      sel  <= '0;
    end else if (!busy) begin
      if (any) begin sel <= pick; busy <= 1'b1; end
    end else if (m_ack) begin
      busy <= 1'b0;
    end
  end

  // Latched at grant, not driven from the live client bus.  `busy` locked the
  // SELECTION but the address still came straight from the winning client, so
  // a client that changed its address while its own transaction was in flight
  // retargeted that transaction.  The tilemap does exactly that: it assigns
  // `rom_addr` unconditionally at every phase 0, so a fetch that was late by
  // more than one tile was reissued to the wrong address.  Independent of the
  // priority change above, and not measured by dbg_tm_late.
  reg [24:0] l_addr;
  reg [15:0] l_din;
  reg        l_we;
  reg        l_burst;
  reg        l_burst4;
  reg [1:0]  l_ds;

  // ---- why this is a loop and not `addr[25*pick +: 25]` -------------------
  // MEASURED, 2026-09-07, on the Konami GX build.  A variable-offset part
  // select computes the offset in hardware, and 25 IS NOT A POWER OF TWO, so
  // Quartus builds `25*pick` with a DSP multiplier and then a mux tree indexed
  // by its output.  On the critical path that cost
  //
  //     1.316 ns  routing into  u_arb|Mult0~8|ay[0]
  //     3.036 ns  the cell      u_arb|Mult0~8|resulta[4]
  //
  // 4.35 ns of a 17.16 ns path whose budget was 7.71 ns -- and a DSP block
  // spent on an arbiter.  16*pick and 2*pick are shifts and cost nothing; it
  // is the 25 that does the damage, and 25 is just how wide an address is.
  //
  // Written as a loop over a CONSTANT index, `25*j` folds at elaboration and
  // the whole thing is the plain N-way mux it always should have been.
  //
  // The sibling this module came from (power_spikes ps_memarb.sv) has the same
  // expression with N=4.  NOT changed there -- root CLAUDE.md 14, that core is
  // working and its lane decides.  Reported, not fixed.
  reg [24:0] sel_addr;
  reg [15:0] sel_din;
  reg        sel_we;
  reg        sel_burst;
  reg        sel_burst4;
  reg [1:0]  sel_ds;
  integer j;
  always @(*) begin
    sel_addr  = '0;
    sel_din   = '0;
    sel_we    = 1'b0;
    sel_burst = 1'b0;
    sel_burst4 = 1'b0;
    sel_ds    = 2'b00;
    for (j = 0; j < N; j = j + 1)
      if (j[$clog2(N)-1:0] == pick) begin
        sel_addr  = addr [25*j +: 25];
        sel_din   = din  [16*j +: 16];
        sel_we    = we   [j];
        sel_burst = burst[j];
        sel_burst4 = burst4[j];
        sel_ds    = ds   [2*j +: 2];
      end
  end

  always @(posedge clk)
    if (!busy && any) begin
      l_addr  <= sel_addr;
      l_din   <= sel_din;
      l_we    <= sel_we;
      l_burst <= sel_burst;
      l_burst4 <= sel_burst4;
      l_ds    <= sel_ds;
    end

  assign m_req   = busy;
  assign m_addr  = l_addr;
  assign m_din   = l_din;
  assign m_we    = l_we;
  assign m_burst = l_burst;
  assign m_burst4 = l_burst4;
  assign m_ds    = l_ds;

  // Every read's words are registered on the controller's ack, whoever it was
  // for; only a registered client looks at them, on its own delayed ack, and no
  // second transaction can finish in between (a transaction is several clocks).
  always @(posedge clk) begin
    if (rst) ack_d <= '0;
    else begin
      ack_d <= '0;
      if (busy && m_ack && reg_mask[sel]) ack_d[sel] <= 1'b1;
    end
    if (m_ack) begin
      resp_q  <= m_dout;
      resp2_q <= m_dout2;
    end
  end

  generate
    if (RESP_REG == 32'd0) begin : g_direct
      assign dout  = m_dout;
      assign dout2 = m_dout2;
    end else begin : g_resp
      assign dout  = resp_q;
      assign dout2 = resp2_q;
    end
  endgenerate

  always @(*) begin
    ack = ack_d;
    if (busy && m_ack && !reg_mask[sel]) ack[sel] = 1'b1;
  end

  // ---------------------------------------------------------------------
  // How long the CPU actually waits.  Root CLAUDE.md 6.5: a claim needs a
  // measurement, and "the CPU is not starved" is a claim.  CPU is client 3.
  // ---------------------------------------------------------------------
  // Client 1.  Client 0 is the ROM download, which only runs while the CPU is
  // held off the bus, so it can never be the thing that stalls it.
  localparam int CPU = 1;
  always @(posedge clk) begin
    if (rst)
      dbg_cpu_stall <= 16'd0;
    else if (req[CPU] && !ack[CPU] && dbg_cpu_stall != 16'hFFFF)
      dbg_cpu_stall <= dbg_cpu_stall + 16'd1;
  end

  // ---- consecutive starvation, per client --------------------------------
  //  1024 clocks is 10.7 us at 96 MHz.  The slowest legitimate wait is one
  //  SDRAM transaction behind three other clients -- tens of clocks, not
  //  thousands -- so this cannot light on ordinary contention.  Same threshold
  //  and the same reasoning as gx_sound's dbg[7].
  reg [10:0] starve [0:N-1];
  genvar gi;
  generate
    for (gi = 0; gi < N; gi = gi + 1) begin : g_starve
      always @(posedge clk) begin
        if (rst)                        starve[gi] <= 11'd0;
        else if (!req[gi] || ack[gi])   starve[gi] <= 11'd0;
        else if (!starve[gi][10])       starve[gi] <= starve[gi] + 11'd1;
      end
      assign dbg_starved[gi] = starve[gi][10];
    end
  endgenerate

endmodule

`default_nettype wire
