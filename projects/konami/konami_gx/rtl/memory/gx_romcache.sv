//============================================================================
//  gx_romcache -- a small direct-mapped word cache for the sound 68000's ROM
//
//  ---- this is a sibling lane's file -----------------------------------------
//  Copied from projects/vsystem/power_spikes/rtl/memory/ps_romcache.sv, which
//  is on MiSTer hardware.  Root CLAUDE.md section 1.2 shelf 3.  Only the two
//  parameters differ; the FSM is unchanged, and so is the reason it exists.
//
//  WHY IT IS HERE, measured rather than assumed.  docs/DECISIONS.md D10 sizes
//  this board's sound CPU from `tools/gx_sndcpu.lua`, over MAME frames
//  163-660 -- the window in which the main CPU is stuck waiting for it:
//
//      sndrom   12,839,106 reads      25,833 per frame     90.7 % of traffic
//      sndram    1,143,783 + 166,402   2,636 per frame      9.3 %
//
//  The 9 % is on-chip (D10).  The 90 % cannot be: 256 KB is 256 M10K by this
//  design's own measured rate and there are 175 blocks free.  So the sound
//  CPU's program fetch goes to SDRAM as arbiter client 3, and at D2's measured
//  8.96 clocks per word that is ~231,000 of a frame's 1,600,000 system clocks
//  before any cache -- on a bus that is already oversubscribed.
//
//  Power Spikes measured the identical situation on hardware: a 68000 whose
//  work RAM answers in a clock and whose program ROM is one SDRAM transaction
//  per word, waiting 7.19 system clocks per fetch.  It tried 128 entries and
//  got a 44 % hit rate -- the working set of a real handler is larger than 256
//  bytes -- and shipped 1024.  Taking the shipped number rather than re-deriving
//  it is the whole point of shelf 3.
//
//  SIZING HERE.  1024 entries of 16 data + 8 tag is 24,576 bits, about 3 M10K
//  of the 175 free.  `v` is a reset-cleared bit per entry, so it is 1024 flip
//  flops and NOT block RAM -- that is deliberate and is what makes
//  reset-invalidate a single clock.
//
//  COHERENCY is free for the same reason it was there: the sound CPU cannot
//  write this region (`sel_rom` is read-only in gx_sound) and the only thing
//  that changes it is the ROM download, during which `rst` is held.  No
//  snooping to get wrong.
//
//  NOT A PREFETCHER.  Prefetching needs a second outstanding request and a
//  policy for what to do when the CPU branches away; caching what was actually
//  fetched needs neither and cannot issue a transaction the CPU did not ask
//  for.
//============================================================================
`default_nettype none

module gx_romcache #(
    parameter int IDX_BITS = 10,         // 1024 entries -- see the sizing note
    parameter int AW       = 18          // word-address width, [AW:1].
                                         // 256 KB = 128 K words = 18 bits.
) (
    input  wire            clk,
    input  wire            rst,

    // --- CPU side: the same contract gx_sound speaks to every other device --
    input  wire [AW:1]     c_a,
    input  wire            c_rd,         // level, held until c_ack
    output reg             c_ack,        // one-clock pulse, data valid with it
    output reg  [15:0]     c_q,

    // --- memory side: the same contract the arbiter already speaks ----------
    output reg  [AW:1]     m_a,
    output reg             m_rd,
    input  wire            m_ack,
    input  wire [15:0]     m_q
);

  // No hit/miss counters here on purpose.  The thing worth counting is what
  // the CPU actually waits, and that is counted where the waiting happens.
  // A separate hit counter would be a second instrument for the same fact,
  // and this factory has been burned by a counter that agreed with its own
  // assumption more than once (root LESSONS_LEARNED, and this board's L-018).

  localparam int TAG_BITS = AW - IDX_BITS;

  reg                     v   [0:(1<<IDX_BITS)-1];
  reg [TAG_BITS-1:0]      tag [0:(1<<IDX_BITS)-1];
  reg [15:0]              dat [0:(1<<IDX_BITS)-1];

  wire [IDX_BITS-1:0] idx    = c_a[IDX_BITS:1];
  wire [TAG_BITS-1:0] cur_tg = c_a[AW:IDX_BITS+1];

  localparam [1:0] S_IDLE = 2'd0, S_LOOK = 2'd1, S_FILL = 2'd2, S_DONE = 2'd3;
  reg [1:0] st;

  // Registered lookup.  A combinational hit would have to drive c_ack in the
  // same clock the request appears, which puts the tag compare and the data
  // mux in one path; one clock of latency against the SDRAM transaction being
  // removed is not worth that risk on a 96 MHz design that has already spent
  // a session on timing.
  reg                le_v;
  reg [TAG_BITS-1:0] le_tag;
  reg [15:0]         le_dat;
  reg [IDX_BITS-1:0] le_idx;
  reg [TAG_BITS-1:0] le_cur;

  // A fill is written one clock after the SDRAM acknowledges it.  `m_q` is
  // gx_sdram's `dout`, served combinationally from the I/O-cell register
  // `dq_in` on the ack clock (MEASUREMENTS 20, 3d4ea19), so writing it straight
  // into `dat` and `c_q` made a whole clock of fabric route from the pad to
  // this RAM.  MEASURED, build of 12eb5c5 (MEASUREMENTS 37): with M10K at
  // 545 / 553 the fitter put `dat` far from the pads and that path missed
  // 96 MHz by 0.292 ns -- 4 of the 320 dq_in paths within 0.3 ns were this
  // module's.  `fill_q` is the one register on that route; the sound CPU sees
  // its acknowledge one clock later on a miss, against a 12-clock enable.
  reg [15:0]         fill_q;
  reg                fill_d;

  integer i;
  always @(posedge clk) begin
    if (rst) begin
      st         <= S_IDLE;
      c_ack      <= 1'b0;
      c_q        <= 16'h0000;
      m_rd       <= 1'b0;
      m_a        <= {AW{1'b0}};
      fill_d     <= 1'b0;
      for (i = 0; i < (1<<IDX_BITS); i = i + 1) v[i] <= 1'b0;
    end else begin
      c_ack <= 1'b0;

      case (st)
        S_IDLE: begin
          if (c_rd) begin
            le_v   <= v[idx];
            le_tag <= tag[idx];
            le_dat <= dat[idx];
            le_idx <= idx;
            le_cur <= cur_tg;
            m_a    <= c_a;
            st     <= S_LOOK;
          end
        end

        S_LOOK: begin
          if (le_v && le_tag == le_cur) begin
            c_q   <= le_dat;
            c_ack <= 1'b1;
            st    <= S_DONE;
          end else begin
            m_rd <= 1'b1;
            st   <= S_FILL;
          end
        end

        S_FILL: begin
          if (m_ack) begin
            m_rd   <= 1'b0;
            fill_q <= m_q;
            fill_d <= 1'b1;
          end
          if (fill_d) begin
            fill_d      <= 1'b0;
            v  [le_idx] <= 1'b1;
            tag[le_idx] <= le_cur;
            dat[le_idx] <= fill_q;
            c_q         <= fill_q;
            c_ack       <= 1'b1;
            st          <= S_DONE;
          end
        end

        // `c_rd` is a level held until the CPU sees its acknowledge, so wait
        // for it to drop before looking at it again -- otherwise one read is
        // acknowledged twice.
        S_DONE: if (!c_rd) st <= S_IDLE;

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
