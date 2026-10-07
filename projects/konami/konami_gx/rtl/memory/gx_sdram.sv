//============================================================================
//  Konami System GX -- SDRAM controller for the MiSTer SDRAM module
//
//  ---- this is a sibling lane's module ---------------------------------------
//  Copied from projects/vsystem/power_spikes/rtl/memory/ps_sdram.sv, which has
//  been through MiSTer hardware.  Root CLAUDE.md section 1.2 shelf 3 and the
//  factory memory `check-sibling-lane-first`.  Single port, one 16-bit word per
//  transaction, auto-precharge, CAS latency 2.
//
//  What changed for this board, and nothing else did:
//
//    * CLK_HZ defaults to 96 MHz and the state machine's fixed waits are
//      lengthened to match -- tRCD now takes two states, not one, and the
//      post-access tRC gap is longer.  Every change makes it SLOWER, which is
//      the safe direction.
//    * RD_STATES is a parameter instead of a hardcoded three.
//
//  ---- READ THIS BEFORE TRUSTING ANY PICTURE THIS FEEDS ----------------------
//  TWO things are known to be wrong with this controller on this board, both
//  written up in docs/DECISIONS.md.  Neither is a reason not to build -- the
//  first build exists to read the fitter's RAM summary -- and both are reasons
//  not to believe a picture.
//
//  D2  IT WAS TOO SLOW.  The burst read below is the fix.  MEASURED rather
//      than counted: sim/tb_gx_sdram.sv drives a saturated reader and gets
//      8.96 clocks per word = 10.70 M words/s at 96 MHz.  D2 said "about one
//      access per 8 clocks = 12 M/s" from reading the state list, and that
//      was 12 % optimistic -- the tilemap's 9 M words/s is 84 % of this
//      controller, not 75 %.
//
//      The burst is TWO READ COMMANDS one clock apart into the row that is
//      already open, NOT a burst length in the mode register.  Same saving,
//      and it leaves the mode register, the write path and the
//      read-modify-write path exactly as the sibling lane proved them.  A
//      BL=2 mode register would also impose a wrap: with sequential
//      addressing a burst from an ODD column wraps back to the even one
//      instead of advancing, which is a silent wrong-pixel bug waiting for
//      the first fetcher that is not word-pair aligned.
//
//  D3  ITS READ TIMING AT 96 MHz IS UNVERIFIED AND THE ARITHMETIC SAYS IT IS
//      MARGINAL.  The read-latch count below is derived from the SDRAM clock
//      being 180 degrees out of phase, which is right at 40-50 MHz.  At 96 MHz
//      (period 10.4 ns, tAC about 5.4 ns) the data window falls between two
//      core clock edges instead of on one.  Settle it with a self-test on
//      hardware, the way NA-1/NA-2 established this board's DQM behaviour --
//      not with more arithmetic.  RD_STATES and the PLL phase are the knobs.
//
//  ---- interface -------------------------------------------------------------
//  Matches the `mem_*` port of gx_top exactly, so a behavioural model and this
//  controller are drop-in equivalents:
//      req    held high until ack
//      ack    one clock, read data valid in the same clock
//      addr   WORD address -- see the note in rtl/gx_rommap.svh
//      burst  read TWO consecutive words; dout = addr, dout2 = addr+1
//
//      burst4 with burst: read FOUR consecutive words -- see FOUR-WORD BURST
//
//  BURST CONTRACT: `addr` must be EVEN when `burst` is high.  Not because the
//  arithmetic below needs it -- a_col + 1 is right for any column but the
//  last -- but because column 0x3ff would carry into the bank field and read
//  somewhere else entirely.  Every burst client on this board fetches an
//  aligned 32-bit word, so every burst address is even by construction; the
//  testbench checks that the two columns of every burst it sees really are
//  {2n, 2n+1}, so a future client that breaks the contract fails a test
//  rather than drawing wrong pixels.
//
//  ---- FOUR-WORD BURST, 2026-09-23 (MEASUREMENTS 138) -----------------------
//  `burst4` together with `burst` reads addr .. addr+3 from ONE ACTIVE: four
//  READ commands on consecutive clocks, only the last with auto-precharge.
//  `addr` must then be a MULTIPLE OF FOUR, for the same column-carry reason as
//  the burst-2 contract.  The sprite fetcher is the only client: a tile row's
//  two 8-dot halves of planes 0-3 are four consecutive words, and fetching
//  them as two burst-2s paid for two row activations (MEASUREMENTS 135.1).
//
//  It is the burst-2 sequence with S_CMD2 run three times instead of once, so
//  a burst-2 issues exactly the commands it always did and no state changes
//  its encoding.  Every word still latches RD_STATES states after the READ
//  that fetched it; the chain after the last READ is unchanged, so the
//  auto-precharge -> next ACTIVE distance (tRP) is the burst-2's.  tRAS only
//  grows (the precharging READ is two clocks later).
//
//  Only the RAW tier carries all four words: on the ack clock `raw_b0` /
//  `raw_b1` are words 0 / 1, `raw_w1` word 2 and `raw_q` word 3.  `dout` /
//  `dout2` give words 2 / 3 -- a registered-tier client must not ask for it.
//
//  A write with ds != 2'b11 costs two transactions rather than one: the board
//  does not honour DQM, so byte writes are read-modify-write.  That evidence is
//  inherited from NA-1/NA-2 on this same physical machine and is quoted in the
//  block comment below.
//
//  Address mapping: {row[12:0], bank[1:0], col[9:0]}, so sequential words stay
//  inside one row for as long as possible.  That is what a burst-2 read would
//  exploit and it is why the mapping is worth keeping as-is.
//============================================================================
`default_nettype none

module gx_sdram #(
    parameter int CLK_HZ      = 96_000_000,
    parameter int INIT_US     = 200,        // power-up wait
    // 7.8 us between refreshes = 749 clocks at 96 MHz; 700 keeps margin, the
    // same ratio the sibling used (391 available, 280 chosen).
    parameter int REFRESH_CLK = 700,
    // States between the READ command and latching DQ.  Three is the sibling's
    // derivation for a 180-degree SDRAM clock; see D3 in docs/DECISIONS.md
    // before changing it, and change the PLL phase with it.
    parameter int RD_STATES   = 3,
    // 1: `dout` / `dout2` keep the last read's words after the ack clock
    //    (tb_gx_sdram reads them that way).  0: they are valid ON the ack clock
    //    only, and `dout_q` / `dout2_q` do not exist -- the board's setting since
    //    the two-tier return (gx_memarb RESP_REG), where nothing looks later and
    //    every register on `dq_in` is a load on the one path that does not close
    //    by itself (MEASUREMENTS 40).
    parameter bit HOLD_DOUT   = 1'b1
) (
    input  wire        clk,          // SDRAM clock (same domain as the core)
    input  wire        init,         // hold high to (re)run the init sequence
    // --- refresh phase lock, 2026-09-23 (MEASUREMENTS 120) -------------------
    //  A one-clock pulse that restarts the refresh timer.  The frame is not a
    //  multiple of REFRESH_CLK, so a free-running timer puts each frame's
    //  refreshes at DIFFERENT raster positions -- and a sprite fetcher that is
    //  short of time on 180 lines of 224 then loses picture on DIFFERENT lines
    //  each frame, which is flicker rather than a steady error.  Pulsing this
    //  at a fixed point in the frame makes every frame see the same refresh
    //  pattern.  Restarting the timer only ever refreshes EARLIER than the
    //  free-running one would, and refreshing early is always safe.
    //  Tie it to 0 to keep the free-running behaviour exactly.
    input  wire        ref_sync,

    // --- request port -------------------------------------------------------
    input  wire [24:0] addr,         // word address
    input  wire [15:0] din,
    output wire [15:0] dout,
    output wire [15:0] dout2,        // addr+1; valid at ack only if burst
    //  The RAW return, for the one client that cannot spare a clock: on the ack
    //  clock of a read `raw_q` is the single word, or a burst's SECOND word, and
    //  `raw_w1` a burst's FIRST word.  Both are registers -- `dq_in` is the I/O
    //  cell -- so a client that captures them straight into its own register has
    //  no shared mux and no arbiter in front of it.  The client knows whether it
    //  asked for a burst; this module does not tell it.  gx_top's tile adapter.
    output wire [15:0] raw_q,
    output wire [15:0] raw_w1,
    //  A four-word burst's words 0 and 1, on its ack clock (see FOUR-WORD
    //  BURST).  Registers loaded from `dq_in`, like `w1`.
    output wire [15:0] raw_b0,
    output wire [15:0] raw_b1,
    input  wire        req,
    input  wire        we,
    input  wire        burst,        // read two words -- see BURST CONTRACT
    input  wire        burst4,       // with burst: FOUR words -- FOUR-WORD BURST
    input  wire [1:0]  ds,           // {upper byte, lower byte}
    output reg         ack,

    // --- SDRAM pins ---------------------------------------------------------
    output reg  [12:0] SDRAM_A,
    output reg  [1:0]  SDRAM_BA,
    inout  wire [15:0] SDRAM_DQ,
    output reg         SDRAM_DQML,
    output reg         SDRAM_DQMH,
    output wire        SDRAM_nCS,
    output reg         SDRAM_nWE,
    output reg         SDRAM_nRAS,
    output reg         SDRAM_nCAS,
    output reg         SDRAM_CKE
);

  localparam int INIT_CLKS = (CLK_HZ / 1_000_000) * INIT_US;   // ~10000

  // command encoding {nRAS, nCAS, nWE}
  localparam [2:0] CMD_NOP        = 3'b111,
                   CMD_ACTIVE     = 3'b011,
                   CMD_READ       = 3'b101,
                   CMD_WRITE      = 3'b100,
                   CMD_PRECHARGE  = 3'b010,
                   CMD_REFRESH    = 3'b001,
                   CMD_LOADMODE   = 3'b000;

  // Mode register: burst length 1, sequential, CAS latency 2, single write
  localparam [12:0] MODE = 13'b000_0_00_010_0_000;

  assign SDRAM_nCS = 1'b0;          // always selected

  // ---- bidirectional data bus -------------------------------------------
  reg        dq_oe;
  reg [15:0] dq_out;
  assign SDRAM_DQ = dq_oe ? dq_out : 16'hZZZZ;

  // ---- DQ capture: ONE register per pad, loaded every clock -------------
  // MEASURED 2026-09-14 (docs/MEASUREMENTS.md 20).  This used to latch the
  // pins straight into `dout`, `dout2` and `rmw_data` in different states.
  // sys.tcl asks for FAST_INPUT_REGISTER on SDRAM_DQ, but an I/O cell holds
  // ONE input register: b56f8c0's fitter packed `dout` and left `dout2` and
  // `rmw_data` in the fabric, fed from the pad over whatever route placement
  // gave them.  So a burst's SECOND word had a pad-to-register delay that
  // changed with every netlist, on the input path the .sdc already says does
  // not close at 96 MHz (D3).  On that build DQ[1] kept the first word's 1 in
  // 12.0 % of the reads where the second word's bit was 0 -- 1,061 of 8,858
  // layer-C dots, the "green dots" -- and 0 of 16,571 otherwise; 9bf82a3,
  // same RTL but for one gate, read 0 of 5,916 on the same class.
  //
  // `dq_in` samples on exactly the edges the old registers did, and it is the
  // only thing the pads feed, so it always packs.  Everything after it is a
  // register-to-register path that TimeQuest times.  The words are served
  // straight out of `dq_in` (and `w1`) on the ack clock, so the transaction
  // costs no extra clock -- that clock is bus time D2 cannot spare -- and are
  // held in `dout_q` / `dout2_q` afterwards.
  reg [15:0] dq_in;
  always @(posedge clk) dq_in <= SDRAM_DQ;

  reg        rd_fresh_b = 1'b0;   // this read's ack clock was a burst's
  reg        rmw_cap    = 1'b0;   // dq_in holds the R half of a byte write
  reg [15:0] w1;                  // a burst's first word, copied out of dq_in
  reg [15:0] b0, b1;              // a four-word burst's words 0 and 1
  assign raw_q  = dq_in;
  assign raw_w1 = w1;
  assign raw_b0 = b0;
  assign raw_b1 = b1;

  // ---- address decomposition --------------------------------------------
  wire [9:0]  a_col  = addr[9:0];
  wire [1:0]  a_bank = addr[11:10];
  wire [12:0] a_row  = addr[24:12];

  // ---- sequencer ---------------------------------------------------------
  localparam S_INIT       = 4'd0,
             S_INIT_PRE   = 4'd1,
             S_INIT_REF1  = 4'd2,
             S_INIT_REF2  = 4'd3,
             S_INIT_MODE  = 4'd4,
             S_IDLE       = 4'd5,
             S_ACTIVE     = 4'd6,
             S_RCD        = 4'd7,
             S_RCD2       = 4'd14,
             S_CMD        = 4'd8,
             S_CL1        = 4'd9,
             S_CL2        = 4'd10,
             S_CL3        = 4'd11,
             S_REFRESH    = 4'd12,
             S_REF_WAIT   = 4'd13,
             // The second READ of a burst.  Numbered last so every existing
             // state keeps the encoding the sibling lane shipped -- this
             // module is hardware-proven at 40-50 MHz and a renumber would
             // change placement for no reason (root CLAUDE.md 1.3, 1.5).
             S_CMD2       = 4'd15;

  reg [3:0]  st;
  reg [15:0] timer;
  reg [9:0]  ref_cnt;
  reg        ref_due;
  reg        rd_pending;
  // This transaction is a two-word burst.  Latched at ACTIVE from the live
  // `burst` input, the same clock `rd_pending` is, so it cannot change under
  // an access already in flight.
  reg        burst_pending;
  // ... and a FOUR-word one (implies burst_pending).  `b_off` is the column
  // offset of the READ S_CMD2 issues next: 1 for a burst-2's only one, 1..3
  // for a burst-4's three.
  reg        burst4_pending;
  reg [1:0]  b_off;

  // ---- read-modify-write for byte writes --------------------------------
  // On the DE10-Nano this core runs on, DQM is not honoured on writes.
  //
  // INHERITED EVIDENCE, not measured by this project: the NA-1/NA-2 project
  // established it on the same physical machine.  Its na2_membus self-test
  // wrote 0x0000 as a word, then 0x00A5 with ds=01, then 0x5A00 with ds=10,
  // and read back 0x5A00 -- 256 byte writes, 256 mismatches, while the same
  // 256 word writes read back clean.  Every write puts both bytes down.
  //             HW_CONFIRMED for that board; assumed to hold for this one
  //             because it is the same board.  Re-measure before trusting it
  //             on any other MiSTer.
  //
  // Konami GX needs this for the same reason NA-2 did: the 68EC020 writes
  // bytes, and although work RAM is on-chip here (docs/DECISIONS.md D4) any
  // region that ever moves to SDRAM inherits the problem.  Keeping RMW costs
  // one extra transaction on byte writes only.  Today nothing on this board
  // writes to SDRAM at all, so this path is dead code that is deliberately
  // kept rather than removed.
  //
  // So a write whose ds is not 2'b11 becomes read-modify-write: read the word,
  // merge the enabled lanes, write the whole word back with both DQM low.
  // That is correct whether or not DQM is wired, and it costs one extra
  // transaction on byte writes only.  Per-lane DQM is still driven, so if the
  // board turns out to be fine nothing about the full-word path changes.
  reg        rmw_rd;      // the read in flight is the R half of a byte write
  reg        rmw_wr;      // the next access is the W half of a byte write
  reg [15:0] rmw_data;    // merged word waiting to go back

  wire       byte_wr = we & (ds != 2'b11);

  // ---- the read return, and the hold registers only when asked for ------
  // HOLD_DOUT 0 must not build `rd_fresh`, `dout_q` or `dout2_q` at all:
  // assigned and never read, each is Quartus warning 10036 (build of 741b62a:
  // dout_q / dout2_q, map.rpt 105 -> 107; build of d689e7e: rd_fresh, 106).
  // `rd_fresh` is "this clock is a read's ack clock" -- the edge that ends
  // S_CL3 on a plain read -- written here rather than in the sequencer so that
  // it exists only in the branch that reads it.
  generate
    if (HOLD_DOUT) begin : g_hold
      reg        rd_fresh = 1'b0;
      reg [15:0] dout_q, dout2_q;
      always @(posedge clk) begin
        rd_fresh <= !init && (st == S_CL3) && !rmw_rd;
        // The words of the read acknowledged THIS clock are still in dq_in /
        // w1; keep them for anyone who looks after the ack clock.
        if (rd_fresh) begin
          if (rd_fresh_b) begin dout_q <= w1; dout2_q <= dq_in; end
          else                  dout_q <= dq_in;
        end
      end
      assign dout  = !rd_fresh ? dout_q  : (rd_fresh_b ? w1 : dq_in);
      assign dout2 = !rd_fresh ? dout2_q : dq_in;
    end else begin : g_nohold
      assign dout  = rd_fresh_b ? w1 : dq_in;
      assign dout2 = dq_in;
    end
  endgenerate

  // The READ S_CMD2 is issuing is the burst's last: always for a burst-2,
  // the third time round for a burst-4.
  wire b4_last = !burst4_pending || (b_off == 2'd3);

  task automatic cmd(input [2:0] c);
    begin
      SDRAM_nRAS <= c[2];
      SDRAM_nCAS <= c[1];
      SDRAM_nWE  <= c[0];
    end
  endtask

  always @(posedge clk) begin
    // defaults every clock
    cmd(CMD_NOP);
    ack      <= 1'b0;
    dq_oe    <= 1'b0;
    rmw_cap  <= 1'b0;

    // The R half of a byte write landed in dq_in on the edge before.  Merged
    // here, two clocks before S_IDLE's tRC wait can let the W half start.
    if (rmw_cap)
      rmw_data <= {ds[1] ? din[15:8] : dq_in[15:8],
                   ds[0] ? din[7:0]  : dq_in[7:0]};

    // refresh timer runs regardless of state
    if (ref_sync) begin
      // Phase-lock: restart the interval here.  `ref_due` is NOT forced -- the
      // pending one, if any, is still pending, and the next is a full interval
      // away.  Shortening an interval is safe; lengthening one is not, and
      // this never lengthens.
      ref_cnt <= 10'd0;
    end else if (ref_cnt == REFRESH_CLK[9:0]) begin
      ref_cnt <= 10'd0;
      ref_due <= 1'b1;
    end else
      ref_cnt <= ref_cnt + 10'd1;

    if (init) begin
      st         <= S_INIT;
      timer      <= INIT_CLKS[15:0];
      ref_cnt    <= 10'd0;
      ref_due    <= 1'b0;
      rd_pending <= 1'b0;
      burst_pending <= 1'b0;
      burst4_pending <= 1'b0;
      rmw_rd     <= 1'b0;
      rmw_wr     <= 1'b0;
      SDRAM_CKE  <= 1'b1;
      SDRAM_DQML <= 1'b1;
      SDRAM_DQMH <= 1'b1;
      SDRAM_A    <= 13'd0;
      SDRAM_BA   <= 2'd0;
    end else begin
      case (st)
        // ---------------- power-up sequence ----------------------------
        S_INIT: begin
          if (timer == 0) st <= S_INIT_PRE;
          else            timer <= timer - 16'd1;
        end
        S_INIT_PRE: begin
          cmd(CMD_PRECHARGE);
          SDRAM_A[10] <= 1'b1;              // all banks
          timer       <= 16'd4;
          st          <= S_INIT_REF1;
        end
        S_INIT_REF1: begin
          if (timer == 0) begin cmd(CMD_REFRESH); timer <= 16'd8; st <= S_INIT_REF2; end
          else timer <= timer - 16'd1;
        end
        S_INIT_REF2: begin
          if (timer == 0) begin cmd(CMD_REFRESH); timer <= 16'd8; st <= S_INIT_MODE; end
          else timer <= timer - 16'd1;
        end
        S_INIT_MODE: begin
          if (timer == 0) begin
            cmd(CMD_LOADMODE);
            SDRAM_A  <= MODE;
            SDRAM_BA <= 2'd0;
            timer    <= 16'd4;
            st       <= S_IDLE;
          end else timer <= timer - 16'd1;
        end

        // ---------------- idle -----------------------------------------
        S_IDLE: begin
          SDRAM_DQML <= 1'b1;
          SDRAM_DQMH <= 1'b1;
          if (timer != 0) begin
            timer <= timer - 16'd1;         // honour tRC after the last access
          end else if (ref_due) begin
            ref_due <= 1'b0;
            cmd(CMD_REFRESH);
            timer   <= 16'd8;               // tRFC >= 60-70 ns -> 8 at 96 MHz
            st      <= S_REF_WAIT;
          end else if (req) begin
            cmd(CMD_ACTIVE);
            SDRAM_A    <= a_row;
            SDRAM_BA   <= a_bank;
            // a byte write reads first; rmw_wr marks the write-back half, and
            // a refresh is free to slip in between the two -- req is held by
            // the same master until ack, so nothing else can take the bus.
            rd_pending <= rmw_wr ? 1'b0 : (~we | byte_wr);
            rmw_rd     <= rmw_wr ? 1'b0 : byte_wr;
            // Only a PLAIN read bursts.  The read half of a byte write must
            // not: it fetches one word to merge, and a second READ command
            // would leave data on the bus while the write-back drives it.
            burst_pending <= burst & ~we & ~rmw_wr;
            burst4_pending <= burst & burst4 & ~we & ~rmw_wr;
            st         <= S_RCD;
          end
        end

        S_REF_WAIT: begin
          if (timer == 0) st <= S_IDLE;
          else            timer <= timer - 16'd1;
        end

        // ---------------- one access -----------------------------------
        // tRCD >= 18 ns.  One clock is enough at 50 MHz and is NOT at 96 MHz
        // (10.4 ns), so this is two states here.  Slower, and correct.
        S_RCD:  st <= S_RCD2;
        S_RCD2: st <= S_CMD;

        S_CMD: begin
          // A10 = 1 selects auto precharge, so no explicit PRECHARGE is needed
          // -- EXCEPT on the first READ of a burst, where the row has to stay
          // open one more clock for the second one.  Getting this backwards
          // tears the row down underneath the second command and the part
          // returns whatever the sense amplifiers held.
          SDRAM_A <= {2'b00, ~(rd_pending & burst_pending), a_col};
          if (rd_pending) begin
            cmd(CMD_READ);
            SDRAM_DQML <= 1'b0;
            SDRAM_DQMH <= 1'b0;
            st         <= burst_pending ? S_CMD2 : S_CL1;
            b_off      <= 2'd1;
          end else begin
            cmd(CMD_WRITE);
            dq_oe      <= 1'b1;
            // the write-back half of a byte write already holds a merged word,
            // so it goes down whole and does not depend on DQM at all
            dq_out     <= rmw_wr ? rmw_data : din;
            SDRAM_DQML <= rmw_wr ? 1'b0 : ~ds[0];   // DQM high masks the byte
            SDRAM_DQMH <= rmw_wr ? 1'b0 : ~ds[1];
            rmw_wr     <= 1'b0;
            ack        <= 1'b1;             // writes complete immediately
            // tRC >= 60 ns from ACTIVE to the next ACTIVE.  ACTIVE + 2 tRCD +
            // CMD is 4 clocks = 42 ns at 96 MHz, so 4 more clears it.
            timer      <= 16'd4;
            st         <= S_IDLE;
          end
        end

        // The second READ of a burst, one clock behind the first, into the
        // row the first left open.  This one carries auto precharge.
        //
        //   a_col + b_off cannot carry out of the column field because a
        //   burst address is aligned -- see BURST CONTRACT in the header.
        //   CORRECTED 2026-09-23 (Opus review, MEASUREMENTS 142): this used to
        //   say a violation "reads a wrong ROW".  It cannot: the sum is ten
        //   bits wide inside the concatenation, so a carry is DROPPED and the
        //   READ wraps to column 0 of the SAME row.  What catches a violation
        //   is tb_gx_sdram's alignment check on a run's first column.
        //
        //   A FOUR-word burst comes round here three times (b_off 1, 2, 3)
        //   and only the third READ carries auto precharge.  For a burst-2
        //   b_off is 1 and this is the single READ it always was.
        S_CMD2: begin
          cmd(CMD_READ);
          SDRAM_A    <= {2'b00, b4_last, a_col + {8'd0, b_off}};
          SDRAM_DQML <= 1'b0;
          SDRAM_DQMH <= 1'b0;
          b_off      <= b_off + 2'd1;
          st         <= b4_last ? S_CL1 : S_CMD2;
        end

        // Read data return.  Count the clocks rather than trusting CL=2 to mean
        // "sample two states later", because it does not:
        //
        //   cycle N    st = S_CMD                     READ registered
        //   cycle N+1  READ on the pins;  the SDRAM samples it half a period
        //              in (SDRAM_CLK is 180 degrees out), so the part's command
        //              edge is at N+1.5
        //   N+3.5      CL=2 later the part starts driving DQ; tAC is measured
        //              from this edge, and DQ is held until N+4.5
        //   N+4.0      the only core clock edge inside that window
        //
        // so the latch has to be three states after S_CMD, not two.  It used to
        // be two and the NA-2 lane's tb_na2_sdram.sv reads back high-Z on every access --
        // the whole 68000 program ROM.  Nothing in a fitter run or in the boot
        // testbench (which swaps in a behavioural memory) can see this.
        // RD_STATES counts these, so with the default of 3 the sequence is
        // S_CL1 -> S_CL2 -> S_CL3(latch).  A fourth would go here.
        //
        // A BURST LATCHES ONE STATE EARLIER, and this is the part to get
        // right.  S_CMD2 pushed the whole chain one clock later relative to
        // the FIRST read command, so that command's data is already on the
        // bus at S_CL2 and the second command's arrives at S_CL3:
        //
        //   single   S_CMD  S_CL1  S_CL2  S_CL3(dout)
        //   burst    S_CMD  S_CMD2 S_CL1  S_CL2(dout)  S_CL3(dout2)
        //
        // Both latch exactly RD_STATES states after the READ command that
        // fetched them, which is the invariant.  CHANGING RD_STATES MOVES
        // BOTH and this comment is the only place that says so.
        //
        // A FOUR-word burst keeps the same invariant, two states further:
        //
        //   burst4   S_CMD  S_CMD2 S_CMD2 S_CMD2  S_CL1(w0) S_CL2(w1) S_CL3(w2) ack(w3)
        //
        // where (wN) names the word IN dq_in during that clock -- words 0 and
        // 1 are copied to b0 / b1 there, word 2 to w1 as a burst-2's first
        // word is, and word 3 is dq_in on the ack clock.
        //
        // "Latch" is `dq_in`, on the edge that ends the state named above; it
        // loads on every edge, so the states below only say which of its
        // values is a word (see "DQ capture" at the top).
        S_CL1: begin
          if (burst4_pending) b0 <= dq_in;
          st <= (RD_STATES <= 1 && !burst_pending) ? S_CL3 : S_CL2;
        end
        S_CL2: begin                       // a burst's first word -> dq_in
          if (burst4_pending) b1 <= dq_in;
          st <= S_CL3;
        end
        S_CL3: begin                       // the (last) word -> dq_in
          if (rmw_rd) begin
            // R half of a byte write: merge (rmw_cap, next clock), then go
            // round again as a write.  No ack -- the caller sees one
            // transaction.
            rmw_cap  <= 1'b1;
            rmw_rd   <= 1'b0;
            rmw_wr   <= 1'b1;
          end else begin
            // A burst's first word is in dq_in DURING this state and is
            // overwritten on the edge that ends it, so it is copied out now.
            if (burst_pending) w1 <= dq_in;
            rd_fresh_b <= burst_pending;
            ack        <= 1'b1;
          end
          // ACTIVE + 2 tRCD + CMD + 3 CL = 7 clocks = 73 ns at 96 MHz, which
          // already clears tRC; one more for the auto-precharge tRP.
          timer <= 16'd2;
          st    <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
