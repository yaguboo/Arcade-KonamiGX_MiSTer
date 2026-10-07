//============================================================================
//  Konami System GX -- palette RAM
//
//  konamigx.cpp:1754 for the map, :1755 for the format:
//      map(0xd90000, 0xd97fff).ram().w(palette, write32).share("palette")
//      PALETTE(config, m_palette).set_format(palette_device::xRGB_888, 8192)
//      m_palette->enable_shadows();  m_palette->enable_highlights();
//
//  32 KB = 8192 entries x 32 bits, of which 24 are used.  This is a big flat
//  palette by arcade standards and it is shared by everything: four tilemap
//  layers, sprites and the gradient backdrop all index into it, with the
//  K055555's per-input PALBASE registers deciding which slice each one gets
//  (docs/SOURCE_AUDIT.md section 9).
//
//  Practical consequence, and it is worth knowing before the first wrong
//  colour appears: a stray pixel's palette index tells you which layer drew it.
//  Reverse-index the colour to an entry, compare with the PALBASE registers,
//  and the owner falls out -- before opening any RTL.  That is the factory
//  lesson `layer-identified-by-palette-bank` and this board is the one it was
//  written for.
//
//  ---- memory shape --------------------------------------------------------
//  8192 x 24 bits of real data = 196 608 bits.  Stored as 8192 x 32 to keep the
//  CPU side a plain 32-bit (here: two 16-bit) write with no read-modify-write,
//  which costs 64 Kbit and buys a much simpler write path.
//
//  Root section 7 / power_spikes L24: this must infer as block RAM.  An
//  earlier version of this file said "M10K inference is safe" here.  IT WAS
//  NOT, and finding out cost a 45-minute build that never finished.  The
//  measurement and the shape that does infer are at the memory declaration
//  below.  CHECK THE FITTER'S RAM SUMMARY; the failure mode is silent.
//============================================================================
`default_nettype none

module gx_palette (
    input  wire        clk,

    // --- pixel enable -------------------------------------------------------
    //  ONLY the two video outputs are gated with this.  The read-port rotation
    //  below keeps running on every clock, because it has to: one port is
    //  shared between the winner, the runner-up and the CPU (docs/DECISIONS.md
    //  D7), and stalling it would stall CPU palette reads.
    input  wire        pxl_cen,

    // --- CPU side.  The 68EC020 sees 32-bit entries; TG68K splits them into
    //     two 16-bit accesses, so a[1] selects the half.
    input  wire        cs,
    input  wire [14:1] addr,       // byte address within d90000-d97fff
    input  wire [15:0] din,
    input  wire [1:0]  ds,         // {uds, lds}, active high
    input  wire        we,
    output wire [15:0] dout,
    // The CPU read is served by the same port as the video reads, so it is not
    // ready in one clock.  gx_top folds this into gx_main's `dev_ok`.
    output reg         dout_ok,

    // --- video side ----------------------------------------------------------
    //  TWO lookups per pixel, not one.  The K054338 blends the winning layer
    //  against whatever is behind it, so gx_prio hands down a winner and a
    //  runner-up and both have to become RGB.  docs/REUSE_PLAN.md section 3.
    //
    //  They share one RAM port, alternating every clock.  At a 6 MHz dot rate
    //  on a ~96 MHz system clock there are 16 clocks per pixel and the indices
    //  are stable for all of them, so two reads cost two of the sixteen.
    //
    //  The alternative -- a second read port -- would need the whole 512 Kbit
    //  duplicated, because an M10K has two ports and the CPU already owns one.
    //  Root section 7 and power_spikes L24: the failure mode of asking a block
    //  RAM for something it cannot do is silent, so do not ask.
    input  wire [12:0] index0,          // winner
    input  wire [12:0] index1,          // runner-up
    output wire [23:0] rgb0,
    output wire [23:0] rgb1
);

// entry = addr[14:2], half = addr[1]
wire [12:0] cpu_entry = addr[14:2];
wire        cpu_half  = addr[1];

// xRGB_888 in MAME's palette_device is (x << 24) | (R << 16) | (G << 8) | B,
// and the board writes it big-endian, so the first 16-bit half of a long holds
// x and R and the second holds G and B.
//
//  ---- MEASURED 2026-09-07: a byte-select write does not infer -------------
//  The first Quartus run of this core did not finish.  Its analysis-and-
//  mapping stage ran 45 minutes and reached 12 GB before it was killed, and a run of THIS
//  MODULE ALONE reached 2.2 GB in seven minutes -- for a memory that should be
//  26 M10K blocks.  It was building flip-flops.
//
//  Three shapes were then synthesised at depth 256 so they would finish, and
//  the result is not what would have been guessed:
//
//      16-bit array, byte-select write, two reads      NOT inferred
//      two 8-bit arrays, plain write, two reads        INFERRED
//      16-bit array, byte-select write, ONE read at a
//        different address than the write              NOT inferred
//
//  So it is the BYTE-SELECT WRITE that defeats inference, not the second read.
//  `mem[addr][15:8] <= d` is a read-modify-write of a 16-bit word as far as
//  Quartus 17.0 is concerned, and it will only fold that into an M10K byte
//  enable when the read address is the same expression as the write address --
//  which is why gx_top's work RAM infers and this did not.
//
//  Root section 7 and power_spikes L24 say the failure mode is silent.  It is
//  worse than silent: `ramstyle = "M10K"` is ignored without a word, and the
//  only symptom is that the build never ends.
//
//  So the lanes are split BY HAND below.  One array per byte, every write a
//  whole-array write.
//
//  One lane per byte of the 32-bit entry, named for what it holds rather than
//  for which half of the long it arrives in:
//
//      pal_x   bits 31-24   the unused byte of xRGB_888
//      pal_r   bits 23-16
//      pal_g   bits 15-8
//      pal_b   bits 7-0
//
//  ---- and then MEASURED again: two reads cost TWO COPIES ------------------
//  Splitting the lanes made it infer, and the report then said 458 752 bits
//  for a 262 144-bit palette.  Quartus 17.0 does not implement "port A writes
//  and reads, port B reads" as one true-dual-port M10K -- it DUPLICATES the
//  memory and gives each copy a simple-dual-port block.  Writing the two ports
//  as two separate always blocks, which is the classic template, duplicates it
//  as well; that was tested too.
//
//  Across the board that was the difference between 42 % and 64 % of the
//  5CSEBA6's block RAM, so it is worth removing.
//
//  So there is ONE read port, rotating between three addresses: the winner,
//  the runner-up and the CPU.  At a 6 MHz dot rate on a 96 MHz clock there are
//  16 clocks per pixel and the rotation is 3, so the video side is served five
//  times over and the CPU waits at most four clocks -- which is why `dout_ok`
//  exists.  One write port, one read port, no duplication.
(* ramstyle = "M10K" *) reg [7:0] pal_x [0:8191];
(* ramstyle = "M10K" *) reg [7:0] pal_r [0:8191];
(* ramstyle = "M10K" *) reg [7:0] pal_g [0:8191];
(* ramstyle = "M10K" *) reg [7:0] pal_b [0:8191];

reg [7:0] q_x, q_r, q_g, q_b;      // the one read port's output
reg [15:0] cpu_q;

// The long's upper half carries x and R, the lower half G and B.
wire we_x = cs && we && !cpu_half && ds[1];
wire we_r = cs && we && !cpu_half && ds[0];
wire we_g = cs && we &&  cpu_half && ds[1];
wire we_b = cs && we &&  cpu_half && ds[0];

// The rotation.  `ph` picks the address; `ph_d` says which of the three the
// data now standing at q_* belongs to, because the address is presented one
// clock before the data appears and the tag has to travel with it.
localparam [1:0] PH_W0 = 2'd0, PH_W1 = 2'd1, PH_CPU = 2'd2;

reg [1:0]  ph, ph_d;
reg        cs_at_read, half_at_read;
reg [12:0] entry_at_read;
reg [23:0] rgb0_r, rgb1_r;

wire [12:0] raddr = (ph == PH_W0) ? index0 :
                    (ph == PH_W1) ? index1 : cpu_entry;

always @(posedge clk) begin
    // ---- write port ------------------------------------------------------
    if (we_x) pal_x[cpu_entry] <= din[15:8];
    if (we_r) pal_r[cpu_entry] <= din[ 7:0];
    if (we_g) pal_g[cpu_entry] <= din[15:8];
    if (we_b) pal_b[cpu_entry] <= din[ 7:0];

    // ---- the one read port -----------------------------------------------
    q_x <= pal_x[raddr];
    q_r <= pal_r[raddr];
    q_g <= pal_g[raddr];
    q_b <= pal_b[raddr];

    ph   <= (ph == PH_CPU) ? PH_W0 : ph + 2'd1;
    ph_d <= ph;

    // Sampled at the same edge as the read, so they describe the data that
    // will be standing at q_* on the next edge.  Selecting on the live `cs`
    // or `cpu_half` one clock later returns the wrong pair whenever the CPU
    // moves between halves of a long.
    if (ph == PH_CPU) begin
        cs_at_read    <= cs;
        half_at_read  <= cpu_half;
        entry_at_read <= cpu_entry;
    end

    case (ph_d)
        PH_W0: rgb0_r <= { q_r, q_g, q_b };
        PH_W1: rgb1_r <= { q_r, q_g, q_b };
        default: begin
            cpu_q   <= half_at_read ? { q_g, q_b } : { q_x, q_r };
            dout_ok <= cs_at_read;
        end
    endcase

    // Last, so it always wins over the case above whatever the rotation was
    // doing.
    //
    //  ---- 2026-09-08: `dout_ok` has to carry an ADDRESS -------------------
    //  `if (!cs)` alone was not enough, and the case it missed is the one the
    //  68EC020 uses most here: a LONG read.
    //
    //  That the chip select does NOT drop between the two halves is read out
    //  of the kernel, which is where a "when is it valid" question is answered
    //  (root CLAUDE.md 5.1 -- MAME has no clocks and cannot answer it):
    //
    //      TG68KdotC_Kernel.vhd:1185  a long access loads memmask = "100001"
    //      :447                       memmaskmux = memmask[4:0]&'1' = "000011"
    //      :450                       memmaskmux[3] = 0, so clkena_lw = 0 and
    //                                 the long is NOT finished
    //      :1082                      memmask shifts to "000111" on the next
    //                                 enable, memmaskmux[3] = 1, long done
    //      :1166,:1176                setstate stays "10" across both, so
    //      :442                       busstate stays "10"
    //
    //  and gx_main's `bus_active` is (busstate != 01), so the select stands
    //  through both cycles with the address stepping by two.  `dout_ok` was
    //  therefore still high from the first half when the CPU sampled the
    //  second: gx_main's `stall` was 0 and the second half returned {x, R}
    //  instead of {G, B}.  EVERY long read of the palette was wrong, not one
    //  in three.
    //
    //  sim/tb_gx_palette.sv reproduces it by driving gx_main's real cadence
    //  (address valid one clock after cen, data sampled at the next cen), and
    //  carries a CONTROL that drops cs between the halves: the control passes
    //  on the broken module, which is what says the fault is the handshake
    //  and not the testbench.
    //
    //  The fix is to say WHICH address the standing data belongs to.  Without
    //  that, `dout_ok` means "a CPU phase completed at some point", which is
    //  not a statement about the access being asked for now.  gx_tilemap's
    //  `ram_ok` had the identical defect and carries the identical fix; there
    //  it needed the fetcher to be holding the port at the wrong moment, so it
    //  was intermittent rather than total.
    //
    //  No deadlock: while the CPU is stalled it holds its address, the
    //  rotation never stalls, so the next PH_CPU phase captures the address
    //  being asked for and the handshake comes back one clock later.
    if (!cs || cpu_entry !== entry_at_read || cpu_half !== half_at_read)
        dout_ok <= 1'b0;
end

initial begin
    ph            = 2'd0;
    ph_d          = 2'd0;
    dout_ok       = 1'b0;
    entry_at_read = 13'd0;
    half_at_read  = 1'b0;
end

assign dout = cpu_q;
// ---------------------------------------------------------------------------
//  The video outputs, in the pixel domain
//
//  rgb0_r/rgb1_r are written when the rotation reaches their phase, so they
//  change every THREE clocks.  gx_colmix captures on pxl_cen, every six or
//  more.  docs/DECISIONS.md D9 wrote this trap down before it was hit:
//
//      MEASURED, build J, 2026-09-07 -- the five worst paths in the design:
//          gx_palette|rgb0_r[6] -> gx_colmix|blue[0]     slack -5.257
//
//  and it is tempting to answer it with a multicycle of 3, because the source
//  changes no faster than that.  THAT WOULD BE FALSE.  A setup multicycle is a
//  claim about the interval between a launch and the capture that USES it, and
//  these two enables have no fixed phase relationship -- the destination can
//  capture one clock after the source changed.  A long destination period
//  buys nothing by itself.
//
//  So the answer is the same as everywhere else on this board: make the launch
//  rate match the capture rate, then say so.  One dot's worth of rotation is
//  more than enough to have serviced both video phases -- even at DOTSEL=3,
//  the fastest dot clock, six system clocks give the three-phase rotation two
//  full turns.
//
//  Cost: 48 flip-flops and one more dot of latency, which is a term in the
//  alignment arithmetic D9 says to carry into P4.
reg [23:0] rgb0_p, rgb1_p;
always @(posedge clk) begin
    if (pxl_cen) begin
        rgb0_p <= rgb0_r;
        rgb1_p <= rgb1_r;
    end
end

assign rgb0 = rgb0_p;
assign rgb1 = rgb1_p;

endmodule

`default_nettype wire
