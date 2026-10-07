//============================================================================
//  Konami System GX Type 2 -- board top
//
//  This is the arcade board and nothing else.  No hps_io, no ioctl, no APF, no
//  MRA: those live in targets/, per root CLAUDE.md section 4.  What crosses
//  this boundary is a neutral memory port, neutral inputs, and video out.
//
//  ---- clocking -------------------------------------------------------------
//  clk = 96.000 MHz, and every clock this board needs divides from it exactly:
//
//      68EC020 main       24 MHz   / 4    MASTER_CLOCK, gx.cpp:117
//      68000 sound         8 MHz   / 12   SUB_CLOCK/2,  gx.cpp:1723
//      TMS57002 DSP       12 MHz   / 8    MASTER_CLOCK/2
//      dot clock, DOTSEL   6 MHz   / 16   } control_w bits 17-16 selects one
//                          8 MHz   / 12   } of these four at runtime
//                         12 MHz   / 8    } (gx.cpp:520-527)
//                         16 MHz   / 6    }
//
//  That all four DOTSEL rates AND both CPUs are integer divisors of 96 MHz is
//  what chose the number.  It is a coincidence worth stating, because Power
//  Spikes had to build a fractional enable for its pixel clock and this board
//  does not.
//
//  The two K054539s want 18.432 MHz, which is NOT a divisor of 96 -- that is a
//  P5 problem and it will need the fractional-enable trick.  Noted here so the
//  clock plan is not later mistaken for complete.
//
//  ---- what is implemented --------------------------------------------------
//      68EC020, full address decode, work RAM              yes
//      K053252 CCU, raster, INT1/INT2                      yes
//      control_w / eeprom_w / input readback               yes
//      K056832 tilemap: registers, VRAM, 4-layer 5bpp      yes
//      palette, K055555 priority, K054338 mixer            yes
//      K055673 sprites, solid pens                         yes  -- gx_sprite
//        sprite shadows                                    yes
//      sound: 68000, K056800, two K054539, TMS57002        NO   -- P5
//      EEPROM 93C46                                        yes  -- jt9346
//      object DMA: MAME's vblank snapshot                  yes  -- EMULATION_DERIVED
//        DMA busy bit (the game waits on it)               yes  -- EMULATION_DERIVED, U10
//        IRQ3                                              NO   -- never enabled by gokuparo (U7)
//
//  ---- work RAM is on-chip, and that is a change of plan ---------------------
//  docs/REUSE_PLAN.md section 4 put the 128 KB work RAM in SDRAM.  It is
//  on-chip here instead, for bring-up:
//
//      tile VRAM   128 KB  = 1.05 Mbit
//      work RAM    128 KB  = 1.05 Mbit
//      palette      32 KB  = 0.26 Mbit  (stored 8192 x 32)
//                          ----------
//                            2.36 Mbit  of the 5CSEBA6's 5.66 Mbit -- 42 %
//
//  Root section 7's Stage-0 rule is "if the sum passes half, consider external
//  memory there and then".  42 % does not pass half, and putting the CPU's RAM
//  behind the arbiter would add a write path and a stall case to the one part
//  of this board that cannot be simulated at all (TG68K is VHDL).  So it goes
//  on-chip for the first screen and the decision gets revisited when the
//  sprite line buffers and the PCM path arrive.
//
//  TODO(RESOURCE): move work RAM to SDRAM if the fitter says so -- and read
//  the RAM summary before the ALM count either way (root section 7,
//  power_spikes L24).  The failure mode is silent.
//============================================================================
`default_nettype none

module gx_top #(
    //  1 = the ESC is the chip running its own firmware (rtl/esc, DECISIONS D31);
    //  0 = rtl/gx_esc.sv, the per-family translation (D18), unchanged.
    parameter bit ESC_CORE = 1'b1
) (
    input  wire        clk,            // 96.000 MHz
    input  wire        rst,
    input  wire        mem_rst,        // the memory transport runs during load
    input  wire        pause,

    // --- D5 experiment, switchable from the OSD.  docs/DECISIONS.md D5 ------
    //  0 = tilemap palette index is colour_code * 32 (this board's choice)
    //  1 = colour_code * 16 (MAME's).  One bitstream tests both.
    input  wire        pal_gran16,
    //  D21: 1 = the calibrated 20 MHz CPU rate, 0 = the 24 MHz every build
    //  before 2026-09-21 ran.  See the enable below.
    input  wire        cpu_cal,
    //  OSD "Voice boost": the sound gains MAME's [HACK] gives Dragoon Might
    //  (gx_sound, MEASUREMENTS 162), for every set -- no per-game gate (user,
    //  2026-10-02).
    input  wire        voice_boost,

    // --- neutral memory port ------------------------------------------------
    output wire [24:0] mem_addr,
    output wire [15:0] mem_din,
    input  wire [15:0] mem_dout,
    input  wire [15:0] mem_dout2,      // second word of a burst read
    //  The controller's raw return (gx_sdram raw_q / raw_w1), for the tile
    //  adapter only: the single word or a burst's second word, and a burst's
    //  first word, on the ack clock.  See "the two-tier return" at u_arb.
    input  wire [15:0] mem_raw_q,
    input  wire [15:0] mem_raw_w1,
    //  gx_sdram raw_b0 / raw_b1: a FOUR-word burst's words 0 and 1 on its ack
    //  clock (words 2 and 3 are mem_raw_w1 / mem_raw_q).  Sprite fetch only.
    input  wire [15:0] mem_raw_b0,
    input  wire [15:0] mem_raw_b1,
    output wire        mem_req,
    output wire        mem_we,
    output wire        mem_burst,      // ask for two consecutive words
    output wire        mem_burst4,     // with mem_burst: FOUR (gx_sdram)
    output wire [1:0]  mem_ds,
    input  wire        mem_ack,

    // --- download (the target drives this while loading) --------------------
    input  wire        dl_active,
    input  wire [24:0] dl_addr,
    input  wire [15:0] dl_data,
    input  wire        dl_req,
    output wire        dl_ack,

    // --- EEPROM default image ------------------------------------------------
    //  One 16-bit word per write, straight into the 93C46's storage.  Neutral
    //  in the sense root CLAUDE.md section 4 asks for: no ioctl, no index, no
    //  notion of where the bytes came from.
    input  wire        nvs_wr,
    input  wire [5:0]  nvs_addr,
    input  wire [15:0] nvs_din,

    // --- inputs, all ACTIVE LOW ---------------------------------------------
    input  wire [7:0]  p1, p2, p3, p4, // bit 0 L, 1 R, 2 U, 3 D, 4-6 B1-B3, 7 START
    input  wire [3:0]  coin,
    input  wire [1:0]  service,
    input  wire [7:0]  dip1,           // SW1: bit0 sound output, bit1 flip
    input  wire [7:0]  dip2,           // SW2: unused by gokuparo

    // --- board configuration: .mra ioctl index 1, byte 3 (DECISIONS D18) ----
    //  WHICH MACHINE, not a bag of flags.  MAME gives each set one `special`
    //  case in init_konamigx (:4120-4166) and the cases are mutually exclusive
    //  by construction -- gx_esc and gx_fjdma already share one bus port on
    //  exactly that ground.  So this is a number, and the numbers are chosen
    //  so that every .mra already shipped keeps its meaning byte for byte:
    //
    //      0  gokuparo / fantjoura-less base   byte 3 absent, reads 0
    //      1  sexyparo, sexyparoa   special 4  the .mra has always sent 0x01
    //      2  fantjour, fantjoura   special 9  the .mra has always sent 0x02
    //      3  tbyahhoo, mtwinbee    special 8  NEW 2026-09-22
    //      4  daiskiss              special 5  NEW 2026-09-22
    //      5  salmndr2, salmndr2a   special 6  NEW 2026-09-30
    //      6  dragoonj, dragoona    special 3  NEW 2026-09-30
    //      7  tokkae                special 0  NEW 2026-10-03  konamigx_6bpp machine
    //      8  tkmmpzdm              special 2  NEW 2026-10-03  konamigx_6bpp + ESC
    //
    //      9  crzcross, puzldama    special 0  NEW 2026-10-03  gokuparo machine
    //     10  winspike(a)(j)        special 7  NEW 2026-10-05  8 bpp tiles + LE2 8 bpp
    //                                                         sprites, Xilinx protection
    //
    //  crzcross / puzldama / tokkae are `special 0` in MAME, but their PCBs
    //  carry an ESC and the games LOAD a program into it and poll the packet
    //  (crzcross 205590 / 2055D2) -- MAME's esc_w completes any packet with no
    //  callback, so the sets run there without one.  Here the chip runs, so
    //  each needs its secret (gx_esc056 modes 7, 8) -- and a machine number.
    //
    //  3 and 4 are one mode in this core (the two callbacks are letter for
    //  letter the same call, :370-378) and are kept apart only so a later
    //  difference has somewhere to go.
    input  wire [3:0]  machine,

    // --- video --------------------------------------------------------------
    output wire [7:0]  red,
    output wire [7:0]  green,
    output wire [7:0]  blue,
    output wire        hsync,
    output wire        vsync,
    output wire        hblank,
    output wire        vblank,
    output wire        ce_pix,

    // --- audio: the two K054539s summed per side, a new value every 48 kHz ---
    output wire signed [15:0] audio_l,
    output wire signed [15:0] audio_r,
    output wire [7:0]  dbg_audio,      // the overlay's second row -- see "THE AUDIO BAND"

    // --- the sound half's external memory (gx_sndxm, DECISIONS D16) ----------
    //  One 64-bit word per op, held request, one-clock ack.  Neutral: the
    //  target decides what memory answers it.
    output wire        sxm_req,
    output wire        sxm_we,
    output wire [21:0] sxm_word,
    output wire [63:0] sxm_wdata,
    output wire [7:0]  sxm_be,
    input  wire        sxm_ack,
    input  wire [63:0] sxm_rdata,

    // --- the TMS57002 host port as a log (DEBUG, rtl/sound/gx_sndtrace.sv) ---
    //  A queue of 64-bit words; the target decides where they go (MiSTer:
    //  DDR3).  Off, and costing nothing but its registers, unless trc_en.
    input  wire        trc_en,
    // H1: the sprite group token, selectable so ONE bitstream can be measured
    // at more than one value.  0 = the shipped 3.  See GRP_TOK below.
    input  wire [1:0]  grp_tok_sel,
    output wire        trc_valid,
    output wire [63:0] trc_data,
    input  wire        trc_take,
    output wire [31:0] trc_polls,

    // --- observability ------------------------------------------------------
    // The CPU liveness ladder's DECODE lives here, not in the target.
    //
    //  MEASURED 2026-09-08, and it cost a build.  With the decode in the
    //  target, `dbg_addr` -- gx_main's CPU-rate address register -- fed seven
    //  comparators across the chip into 96 MHz registers with no exception:
    //  core clock -3.913, TNS -97.8, on a build whose other change was worth
    //  a few tenths.  "Observation only" was true of BEHAVIOUR and false of
    //  TIMING, and those are not the same claim.  docs/DECISIONS.md D9's
    //  pattern, self-inflicted while adding an instrument.
    //
    //  Decoded here, the comparators sit beside the register they read and
    //  only seven slow bits cross the boundary.
    //  The eight ladder rungs are ASSEMBLED here as well as decoded here, and
    //  handed over as one bus.  Before this they were five separate ports that
    //  the target concatenated, and that put the rung ORDER in one file and
    //  the rung MEANING in another -- the exact split that produces a
    //  confident, plausible, wrong table (this file's third assignment, and
    //  tools/read_ladder.py's header).  One bus, one comment, one order.
    output wire [7:0]  dbg_rung,
    output reg  [6:0]  dbg_live,
    output wire [23:1] dbg_addr,
    output reg  [23:1] dbg_pc,         // one opcode-fetch address a frame -- "THE PC SAMPLE"
    output wire [1:0]  dbg_busstate,
    output wire        dbg_stalled,
    output wire        dbg_video_en,
    output wire [7:0]  dbg_enable,     // K055555 ENABLE (reg 45), live
    output wire [15:0] dbg_cpu_stall,
    output wire [3:0]  dbg_winner
);

`include "gx_rommap.svh"

// ---------------------------------------------------------------------------
//  the machine, decoded once
//
//  Everything downstream reads these, not `machine`, so a new set adds a case
//  here and touches nothing else.  The three constants below are the whole of
//  what SOURCE_AUDIT section 19 found the A2 shelf needs: MAME's own gate says
//  the sets differ from the two we run in the ESC call, the sprite X offset
//  and their ROMs -- no new memory map, handler, register or clock (19.5).
// ---------------------------------------------------------------------------
localparam [3:0] MACH_BASE     = 4'd0;   // gokuparo
localparam [3:0] MACH_SEXYPARO = 4'd1;   // special 4
localparam [3:0] MACH_FANTJOUR = 4'd2;   // special 9
localparam [3:0] MACH_TBYAHHOO = 4'd3;   // special 8
localparam [3:0] MACH_DAISKISS = 4'd4;   // special 5
localparam [3:0] MACH_SALMNDR2 = 4'd5;   // special 6
localparam [3:0] MACH_DRAGOONJ = 4'd6;   // special 3
localparam [3:0] MACH_TOKKAE   = 4'd7;   // special 0, konamigx_6bpp
localparam [3:0] MACH_TKMMPZDM = 4'd8;   // special 2, konamigx_6bpp
localparam [3:0] MACH_CRZCROSS = 4'd9;   // special 0, gokuparo machine + an ESC MAME does not run
localparam [3:0] MACH_WINSPIKE = 4'd10;  // special 7, konamigx.cpp:2023

wire sexyparo = (machine == MACH_SEXYPARO);
wire fantjour = (machine == MACH_FANTJOUR);

// which ESC program the machine's chip runs (gx_esc's header has the table);
// mtwinbee is MACH_TBYAHHOO -- its image is byte-identical to tbyahhoo's
wire        daiskiss  = (machine == MACH_DAISKISS);
wire        salmndr2  = (machine == MACH_SALMNDR2);
wire        dragoonj  = (machine == MACH_DRAGOONJ);
wire        tkmmpzdm  = (machine == MACH_TKMMPZDM);
// winspike (konamigx.cpp:2023): K056832_BPP_8 tiles, K055673_LAYOUT_LE2 sprites,
// and special 7's type 4 Xilinx protection at cc0000 in place of the ESC
wire        winspike  = (machine == MACH_WINSPIKE);
// the konamigx_6bpp machine (konamigx.cpp:1860): 6 bpp tiles in salmndr2's ROM
// layout, GX 5bpp sprites with an 8 MB 4bpp area (k055673 region 0xa00000)
wire        gx6bpp    = (machine == MACH_TOKKAE) || tkmmpzdm;
wire [3:0]  esc_mode  = sexyparo ? 4'd1 : (machine == MACH_TBYAHHOO) ? 4'd2 : daiskiss ? 4'd3 :
                        salmndr2 ? 4'd4 : dragoonj ? 4'd5 : tkmmpzdm ? 4'd6 :
                        (machine == MACH_CRZCROSS) ? 4'd7 : (machine == MACH_TOKKAE) ? 4'd8 : 4'd0;

// the sprite ROM format (MAME set_config: K055673_LAYOUT_GX / GX6 / RNG) --
// gx_sprfetch, gx_objdraw, gx_sprite and gx_prio read it.  Registered: it is a
// per-set constant and must not sit combinationally in front of the cache's
// index and tag.
reg  [1:0]  obj_fmt;
always @(posedge clk) obj_fmt <= salmndr2 ? 2'd1 : dragoonj ? 2'd2 : winspike ? 2'd3 : 2'd0;
// the tile ROM format (MAME k056832 set_config): 0 5 bpp with a byte-a-row
// plane-4 ROM, 1 salmndr2's 6 bpp with a word-a-row plane-4/5 ROM (charlayout6),
// 2 dragoonj's 5 bpp with no plane-4 ROM at all (ROMREGION_ERASE00: plane 4 is 0)
reg  [1:0]  tile_fmt;
always @(posedge clk) tile_fmt <= (salmndr2 || gx6bpp) ? 2'd1 : dragoonj ? 2'd2 : winspike ? 2'd3 : 2'd0;
// 3: 8 bpp, a row is 8 bytes in tile4 (charlayout8), fetched as two bursts
wire        tile_bpp8 = (tile_fmt == 2'd3);
// the GX sprite layout's 4bpp area is 8 MB (65536 tiles) on the 6bpp machine
reg         spr_big;
always @(posedge clk) spr_big <= gx6bpp;
// the tilemap and sprite X offsets -- from the CCU the game programs, not the
// set's name (u_xoffs below, rtl/video/gx_xoffs.sv, MEASUREMENTS 174/175).
// Same values as the per-set constants they replaced: tile 0 / dragoonj -15,
// sprite HOFFSET 954 GX / 952 salmndr2 / 931 dragoonj.  EMULATION_DERIVED.
wire signed [9:0] tile_dx_adj;
wire [9:0]        spr_hoffset;

// ---------------------------------------------------------------------------
//  clock enables
//
//  The dot enable divides by 16, 12, 8 or 6 under DOTSEL.  A comparator
//  against a variable terminal count rather than a power-of-two mask, because
//  12 and 6 are not powers of two -- and because DOTSEL can change while the
//  raster is running, so the divider has to be able to reload mid-line without
//  producing a runt enable.
//
//  Gokujou Parodius writes DOTSEL = 00 and never changes it
//  (docs/MEASUREMENTS.md section 2), so only the /16 branch is exercised by
//  anything measured.  The other three are implemented from gx.cpp:527's
//  table and are UNVERIFIED -- UPSTREAM_TODO U2.
// ---------------------------------------------------------------------------
wire [1:0] dotsel;

reg [4:0] dot_div;
reg [4:0] dot_cnt;
reg       pxl_cen, cen_cpu;

// ---------------------------------------------------------------------------
//  THE CPU's EFFECTIVE CLOCK (docs/DECISIONS.md D21)
//
//  APPROXIMATION -- calibrated, not the PCB's number.
//  TG68KdotC_Kernel is not cycle-exact against a 68EC020.  MEASUREMENTS 73
//  measured it 20.0 % fast over `0x284548`'s register-only dbra loop and
//  20.9 % fast over the whole boot to the 0xFE command -- two spans sharing
//  almost no instructions, wanting the same correction to within 0.8 %.  At
//  the PCB's 24 MHz this core reaches the sound self-test's deadline in 203
//  frames where MAME's 68EC020 takes 245, and 227 frames of sound work then do
//  not fit (defect B, MEASUREMENTS 68/71).
//
//  So the enable is an accumulator: +24 a clock of 96 is the old behaviour to
//  the clock (24, 48, 72, wrap -- an enable every fourth), and +20 is
//  20.000 MHz, an enable every 4.8 clocks on average.
//
//  TODO(HARDWAREIZE): cycle-correct the CPU core and restore 24 MHz.
//
//  UNCACHED FETCH, 2026-10-01 (DECISIONS D28, MEASUREMENTS 157, 165).
//  APPROXIMATION -- the ratio is the 68020's, the wait states are unknown.
//  Was MAME's init_posthack (konamigx.cpp:4013-4022: the 68020 at 2/3 for 12 s
//  on tbyahhoo/mtwinbee and dragoonj/dragoona only).  Now from the hardware:
//    * WHAT IS TIMED: the POST's delays are single-instruction `dbra D0,*`
//      loops (tbyahhoo 0x292B44, 0x292B92, 0x292C04 -- 99 % of its PC samples
//      during the sound wait).  MC68020 User's Manual, conditional branch
//      table: DBcc (cc false, count not expired) is 6 clocks in the cache
//      case and 9 (0/2/0, two prefetch bus cycles) in the worst case, "the
//      cache is disabled", at no wait states.  6/9 = 2/3 -- MAME's number is
//      the chip's own ratio for this loop.  With W ROM wait states it is
//      6/(9+2W); W is not known, so 2/3 is the PCB's slowdown's lower bound.
//    * WHEN: CACR bit 0 (gx_main cache_on), for EVERY set: all seven sets'
//      POSTs run ~500 frames with the cache off (tools/gx_cacrtap.lua), and
//      the PCB does not know which game it runs.  MAME's per-set list is
//      where the timeout was noticed, not where the PCB is slow.
//    * HOW: the CPU is stopped for 1,024 of every 3,072 clocks (the `pause`
//      gate), which slows anything by 2/3 whatever it is bound by.  Dividing
//      the enable RATE made the fetch-bound loop faster (157, LESSONS L-047).
//  Applied to every instruction while uncached, not per instruction class.
//  TODO(HARDWAREIZE): the PCB's ROM DSACK wait states (W).
wire        cpu_cache_on;           // gx_main: CACR bit 0
reg  [11:0] ph_cnt;
wire        posthack = !cpu_cache_on;
wire        ph_stop  = posthack && (ph_cnt >= 12'd2048);
always @(posedge clk)
    if (rst || ph_cnt == 12'd3071) ph_cnt <= 12'd0;
    else                           ph_cnt <= ph_cnt + 12'd1;

reg  [6:0] cpu_acc;
wire [6:0] cpu_step = cpu_cal ? 7'd20 : 7'd24;
wire [6:0] cpu_next = cpu_acc + cpu_step;
wire       cpu_wrap = cpu_next >= 7'd96;

always @(*) begin
    case (dotsel)
        2'd0: dot_div = 5'd15;      //  6 MHz
        2'd1: dot_div = 5'd11;      //  8 MHz
        2'd2: dot_div = 5'd7;       // 12 MHz
        2'd3: dot_div = 5'd5;       // 16 MHz
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        dot_cnt <= 5'd0;
        cpu_acc <= 7'd0;
        pxl_cen <= 1'b0;
        cen_cpu <= 1'b0;
    end else begin
        // A DOTSEL change that leaves the counter past the new terminal count
        // would otherwise wrap the long way round and drop a line.
        if (dot_cnt >= dot_div) dot_cnt <= 5'd0;
        else                    dot_cnt <= dot_cnt + 5'd1;
        pxl_cen <= (dot_cnt >= dot_div);

        cpu_acc <= cpu_wrap ? (cpu_next - 7'd96) : cpu_next;
        // The CPU does not run while ROMs are loading -- the loader owns the
        // memory bus, and a CPU fetching from half-written SDRAM would execute
        // whatever happened to be there.
        cen_cpu <= cpu_wrap && !pause && !dl_active && !ph_stop;
    end
end

// the sound CPU's enable, registered for the same reason -- see u_sound
reg snd_ce_en = 1'b0;
always @(posedge clk) snd_ce_en <= !pause && !dl_active;

assign ce_pix = pxl_cen;

// ---------------------------------------------------------------------------
//  CPU and address decode
// ---------------------------------------------------------------------------
wire [24:0] cpu_rom_addr;
wire [24:0] snd_rom_addr;
wire        snd_rom_req;
wire        snd_rom_burst;   // the K054539 sample fetcher's blocks
wire [2:0]  snd_audio_ev;    // gx_sound's audio event pulses, for the AUDIO band
wire [15:0] cpu_rom_data;
wire        cpu_rom_req, cpu_rom_ack;

wire [16:1] wram_addr;
wire [15:0] wram_din, wram_dout;
wire [1:0]  wram_we;

wire ccu_cs, k055555_cs, k054338_cs, objram_cs, objset1_cs, objset2_cs;
wire k056800_cs;
wire k056832_reg_cs, k056832_ram_cs, tilebank_cs;
wire pal_cs, eeprom_cs, control_cs, sysdsw_cs, inputs_cs, service_cs;

wire [23:1] cpu_addr;
wire [15:0] cpu_dout;
wire        cpu_we;
wire [1:0]  cpu_ds;

reg  [15:0] dev_din;

// Every device answers in one cycle EXCEPT the palette, whose CPU read shares
// the video read port and takes up to four (rtl/video/gx_palette.sv).  A write
// never waits -- it goes straight into the write port.
wire        pal_ok, tm_ram_ok;
wire        dev_ok = pal_cs         ? (cpu_we | pal_ok)
                   : k056832_ram_cs ? (cpu_we | tm_ram_ok)
                   : objram_cs      ? (cpu_we | objram_ok)   // the sprite DMA shares its read port
                   : 1'b1;

wire int1, int2;
wire irq2_en, irq4_en;          // INT1 is gated by the ARM below, not the level
wire irq1_sync_set;

// ---- INT1 ARM (DECISIONS D26, MEASUREMENTS 164) --------------------------------
//  INFERRED from the programs -- one mechanism where MAME has two.
//  MAME fires IRQ1 at vblank on (enable & 0x81) == 0x81 OR m_gx_syncen bit 0
//  (konamigx.cpp:514-516, :651-653).  tbyahhoo needs the second: at f833 it
//  writes 0x91 on line 2, acks the CCU on lines 2 and 26, and its ESC routine
//  rewrites the byte to 0x80/0x90 on line 27 -- 197 lines before vblank -- so
//  neither "ESC latency" nor "a pending latch" (gx_ccu's INT1 already is one)
//  puts an enabled INT1 at that vblank (tools/gx_irq1tap.lua).
//  The first path is never needed: over 4000 frames of all eight sets every
//  IRQ1 follows an enable write carrying 0x81 since the previous IRQ1, and
//  none fires on the level alone (tools/gx_irq1arm.lua).  So the enable bit
//  is an ARM: set by a write carrying bits 7 and 0, NOT cleared by a later
//  write without them, consumed by the vblank edge it fires on.  The request
//  then holds until the handler acks the CCU, as before.
//  Not PCB-verified; it is the one model all eight programs agree with.
//  TODO(HARDWAREIZE): U9 -- D56001 bit 0 against /IPL on a PCB.
wire int1_g;
gx_int1arm u_int1arm (
    .clk     (clk),
    .rst     (rst),
    .int1    (int1),            // gx_ccu: set at vblank, cleared by the ack
    .arm_set (irq1_sync_set),   // gx_ctrl: an enable write carrying 0x81
    .irq     (int1_g)
);

// ---------------------------------------------------------------------------
//  the ESC -- the chips' sprite-list programs (rtl/gx_esc.sv, MEASUREMENTS 158)
//
//  DECISIONS D18.  gx_main asks for a run when the game writes the long at
//  cc0000, freezes the 68EC020 and lends gx_esc its transaction slice until
//  `done`.  A run starts only while the sprite DMA is quiet; the DMA in turn
//  defers its copy while a run is in progress (gx_sprite `dma_hold`) and never
//  waits for one that has not started, so the two never touch object RAM
//  together and neither waits for the other forever.
//
//  gokuparo never writes cc0000.  With `esc_mode` 0 a run would still
//  complete -- status byte, IRQ4 if enabled -- and build no sprites (gx_esc
//  S_SUB), which is what MAME's esc_w does with no callback.
// ---------------------------------------------------------------------------
wire        esc_start, esc_done_o, esc_irq4_set, esc_b3_clr, esc_busy, esc_boot_hold;
wire        irq3_en, irq3_set;         // IRQ3, object DMA end (see the CTRL section)
wire [31:0] esc_cmd;
wire        esc_req, esc_we, esc_ack;
wire [23:1] esc_addr;
wire [15:0] esc_din, esc_rdata;
wire [1:0]  esc_be;
wire        spr_dma_quiet;      // gx_sprite: no copy running, pending or starting

// the fantjour device, declared here because the ESC's ack is gated by it
wire        fj_start, fj_done, fj_busy;
// winspike's Xilinx protection (gx_xprot), the third master on this seam
wire        xp_start, xp_busy, xp_done, xp_d1c;
wire        xp_req, xp_we;
wire [23:1] xp_addr;
wire [15:0] xp_din;
wire [1:0]  xp_be;
wire        fj_reg_wr;
wire [4:1]  fj_reg_a;
wire [15:0] fj_reg_d;
wire [1:0]  fj_reg_be;
wire        fj_req, fj_we;
wire [23:1] fj_addr;
wire [15:0] fj_din;
wire [1:0]  fj_be;

reg  esc_go = 1'b0;             // a run asked for, waiting for the DMA to be quiet
reg  esc_b3 = 1'b1;             // rdport1_3 bit 3: reset 1 (0xfc, konamigx.cpp:3985)
wire esc_go_now = (esc_go || esc_start) && spr_dma_quiet;

always @(posedge clk) begin
    if (rst) begin
        esc_go <= 1'b0;
        esc_b3 <= 1'b1;
    end else begin
        if (esc_start)  esc_go <= 1'b1;
        if (esc_go_now) esc_go <= 1'b0;
        if (esc_b3_clr) esc_b3 <= 1'b0;     // :446, and nothing sets it again (U40)
    end
end

generate if (ESC_CORE) begin : g_esc056
gx_esc056 u_esc (
    .clk         (clk),
    .rst         (rst),
    .mode        (esc_mode),
    .start       (esc_go_now),
    .cmd         (esc_cmd),
    .irq_en      (irq4_en),
    .busy        (esc_busy),
    .boot_hold   (esc_boot_hold),
    .irq4_set    (esc_irq4_set),
    .b3_clr      (esc_b3_clr),
    .bus_req     (esc_req),
    .bus_we      (esc_we),
    .bus_addr    (esc_addr),
    .bus_din     (esc_din),
    .bus_be      (esc_be),
    .bus_ack     (esc_ack && !fj_busy && !xp_busy),
    .bus_rdata   (esc_rdata),
    .dbg_booted  (),
    .dbg_fault   ()
);
// D31: each word the chip moves is a whole borrow -- its ack hands the slice back.
assign esc_done_o = esc_ack && !fj_busy && !xp_busy;
end else begin : g_esc_xlat
// The translation knows modes 1-5 only; anything else is its 0.
wire [2:0]  esc_xmode = (esc_mode <= 4'd5) ? esc_mode[2:0] : 3'd0;
assign esc_boot_hold = 1'b0;
gx_esc u_esc (
    .clk         (clk),
    .rst         (rst),
    .mode        (esc_xmode),
    .start       (esc_go_now),
    .cmd         (esc_cmd),
    .irq_en      (irq4_en),
    .busy        (esc_busy),
    .done        (esc_done_o),
    .irq4_set    (esc_irq4_set),
    .b3_clr      (esc_b3_clr),
    .bus_req     (esc_req),
    .bus_we      (esc_we),
    .bus_addr    (esc_addr),
    .bus_din     (esc_din),
    .bus_be      (esc_be),
    .bus_ack     (esc_ack && !fj_busy && !xp_busy),
    .bus_rdata   (esc_rdata),
    // Open: nothing reads it yet -- see the list at the end of this file.
    .dbg_sprites ()
);
end endgenerate

// ---------------------------------------------------------------------------
//  the fantjour device -- the fill / XOR copy at 0xdb0000 (rtl/gx_fjdma.sv)
//
//  DECISIONS D19, MEASUREMENTS 57 and 58.  The same seam as the ESC: gx_main
//  recognises the trigger write, freezes the 68EC020 and lends its transaction
//  slice until `done`, and a run waits for the sprite DMA to be quiet exactly
//  as an ESC run does.
//
//  THE TWO MASTERS SHARE ONE BUS PORT because they cannot both exist.  MAME
//  installs the ESC callback for sexyparo (special 4) and this device for
//  fantjour and fantjoura (special 9); sexyparo never writes 0xdb0000,
//  Fantastic Journey never writes 0xcc0000, and gokuparo writes neither
//  (MEASURED, MEASUREMENTS 49 and 57).  So `fj_busy` says which master the
//  slice is carrying, and which one gx_main's single ack belongs to.
// ---------------------------------------------------------------------------
reg  fj_go = 1'b0;              // a run asked for, waiting for the DMA to be quiet
wire fj_go_now = (fj_go || fj_start) && spr_dma_quiet;

always @(posedge clk) begin
    if (rst) begin
        fj_go <= 1'b0;
    end else begin
        if (fj_start)  fj_go <= 1'b1;
        if (fj_go_now) fj_go <= 1'b0;
    end
end

gx_fjdma u_fjdma (
    .clk       (clk),
    .rst       (rst),
    .reg_wr    (fj_reg_wr),
    .reg_a     (fj_reg_a),
    .reg_d     (fj_reg_d),
    .reg_be    (fj_reg_be),
    .start     (fj_go_now),
    .busy      (fj_busy),
    .done      (fj_done),
    .bus_req   (fj_req),
    .bus_we    (fj_we),
    .bus_addr  (fj_addr),
    .bus_din   (fj_din),
    .bus_be    (fj_be),
    .bus_ack   (esc_ack && fj_busy),
    .bus_rdata (esc_rdata),
    // Open: nothing reads it yet -- see the list at the end of this file.
    .dbg_longs ()
);

// winspike's Xilinx protection (special 7): a third master on the same seam.
// It cannot meet the other two -- winspike has neither an ESC program nor the
// fantjour device, and gx_main only triggers it on that machine.
gx_xprot u_xprot (
    .clk       (clk),
    .rst       (rst),
    .start     (xp_start),
    .op_d1c    (xp_d1c),
    .busy      (xp_busy),
    .done      (xp_done),
    .bus_req   (xp_req),
    .bus_we    (xp_we),
    .bus_addr  (xp_addr),
    .bus_din   (xp_din),
    .bus_be    (xp_be),
    .bus_ack   (esc_ack && xp_busy),
    .bus_rdata (esc_rdata)
);

wire        m_req  = xp_busy ? xp_req  : fj_busy ? fj_req  : esc_req;
wire        m_we   = xp_busy ? xp_we   : fj_busy ? fj_we   : esc_we;
wire [23:1] m_addr = xp_busy ? xp_addr : fj_busy ? fj_addr : esc_addr;
wire [15:0] m_din  = xp_busy ? xp_din  : fj_busy ? fj_din  : esc_din;
wire [1:0]  m_be   = xp_busy ? xp_be   : fj_busy ? fj_be   : esc_be;

gx_main u_main (
    .clk            (clk),
    .rst            (rst),
    .cen            (cen_cpu),

    .rom_addr       (cpu_rom_addr),
    .rom_data       (cpu_rom_data),
    .rom_req        (cpu_rom_req),
    .rom_ack        (cpu_rom_ack),

    .wram_addr      (wram_addr),
    .wram_din       (wram_din),
    .wram_dout      (wram_dout),
    .wram_we        (wram_we),

    .ccu_cs         (ccu_cs),
    .k055555_cs     (k055555_cs),
    .k054338_cs     (k054338_cs),
    .k056832_reg_cs (k056832_reg_cs),
    .k056832_ram_cs (k056832_ram_cs),
    .tilebank_cs    (tilebank_cs),
    .pal_cs         (pal_cs),
    .k056800_cs     (k056800_cs),
    // Decoded by gx_decode and not consumed yet -- see the list at the end of
    // this file.  Left open rather than tied to a dead wire.
    .k056832_rom_cs (),
    .objset1_cs     (objset1_cs),
    .objset2_cs     (objset2_cs),
    .objram_cs      (objram_cs),
    .objrom_cs      (),
    .esc_cs         (),
    .eeprom_cs      (eeprom_cs),
    .control_cs     (control_cs),
    .sysdsw_cs      (sysdsw_cs),
    .inputs_cs      (inputs_cs),
    .service_cs     (service_cs),

    .cpu_addr       (cpu_addr),
    .cpu_dout       (cpu_dout),
    .cpu_we         (cpu_we),
    .cpu_ds         (cpu_ds),
    .dev_din        (dev_din),
    .dev_ok         (dev_ok),

    .int1           (int1_g),         // IRQ1 SYNC above
    .int2           (int2 & irq2_en),
    .irq3_set       (irq3_set),       // object DMA end (below)

    .esc_start      (esc_start),
    .esc_cmd        (esc_cmd),
    // Either master's `done` hands the slice back; only one can be running.
    .esc_done       (esc_done_o | fj_done | xp_done),
    .esc_core       (ESC_CORE),
    .esc_want       (ESC_CORE ? esc_req : 1'b0),
    .esc_hold       (esc_boot_hold),
    .esc_irq4_set   (esc_irq4_set),
    .esc_req        (m_req),
    .esc_we         (m_we),
    .esc_addr       (m_addr),
    .esc_din        (m_din),
    .esc_be         (m_be),
    .esc_ack        (esc_ack),
    .esc_rdata      (esc_rdata),

    .fj_en          (fantjour),
    .fj_reg_wr      (fj_reg_wr),
    .fj_reg_a       (fj_reg_a),
    .fj_reg_d       (fj_reg_d),
    .fj_reg_be      (fj_reg_be),
    .fj_start       (fj_start),
    .xp_en          (winspike),
    .xp_start       (xp_start),
    .xp_d1c         (xp_d1c),

    .dbg_addr       (dbg_addr),
    .dbg_busstate   (dbg_busstate),
    .dbg_stalled    (dbg_stalled),
    .cache_on       (cpu_cache_on)
);

// ---------------------------------------------------------------------------
//  work RAM, 128 KB.  See the header for why it is on-chip.
// ---------------------------------------------------------------------------
//  This one DOES infer with a byte-select write, and the measurement in
//  gx_palette.sv's header says why: the read address is the same expression as
//  the write address, which is the one case Quartus 17.0 folds into an M10K
//  byte enable.  Left as it is -- the first Quartus run verified it inferring,
//  and root section 1.3 does not rewrite working code to match a style.
//
//  (Careful: Quartus reads the word S-Y-N-T-H-E-S-I-S followed by another word
//  inside an ordinary // comment as a pragma, and warns that the second word is
//  an unrecognised attribute.  This paragraph tripped it once by naming the
//  tool and a verb.  Spelled out here so this note does not trip it too.)
(* ramstyle = "M10K" *) reg [15:0] wram [0:65535];
reg [15:0] wram_q;

always @(posedge clk) begin
    if (wram_we[1]) wram[wram_addr][15:8] <= wram_din[15:8];
    if (wram_we[0]) wram[wram_addr][ 7:0] <= wram_din[ 7:0];
    wram_q <= wram[wram_addr];
end

assign wram_dout = wram_q;

// ---------------------------------------------------------------------------
//  object RAM, 16 KB at 0xd20000-0xd23fff
//
//  WHY IT IS HERE BEFORE THE SPRITE ENGINE IS.  The sprite chip is P5 and
//  nothing renders from this memory yet, so on the face of it this is dead
//  storage.  It is not: the game's power-on self-test WRITES AND READS IT
//  BACK, and until 2026-09-08 the CPU got 0xffff for every read because
//  `objram_cs` was decoded and then left open.
//
//  MEASURED, tools/gx_readtap.lua against MAME, 700 frames:
//
//      objram 0xd20000-0xd23fff   40,960 reads   frames 105..109
//
//  16 KB is 4,096 longs and 40,960 / 4,096 is exactly 10.0 passes -- a
//  write-then-verify RAM test, ten patterns deep.  A self-test cannot pass
//  against a window that answers 0xffff, and the board sits on the self-test
//  screen (ENABLE = 0x01) for as long as anyone has watched it: 145 seconds,
//  where the golden model is through it in eleven.
//
//  That is a hypothesis about the hang, not a proof, and it is written here
//  rather than in a commit message because the next person to read this line
//  needs to know why a memory with no reader exists.
//
//  Same shape as the work RAM above, deliberately: the read address is the
//  same expression as the write address, which is the one case Quartus 17.0
//  folds a byte-select write into an M10K byte enable (root section 7,
//  power_spikes L24, and the measurement in gx_palette.sv's header).  8,192
//  words of 16 bits is 128 Kbit, about 13 M10K blocks on top of the 362 this
//  design already uses.
//
//  ---- 2026-09-14: the sprite DMA shares the ONE read port ------------------
//  gx_sprite copies the first 256 entries into its own table once a frame,
//  at vblank begin, through this memory's read port -- a second read port
//  would DUPLICATE the memory (gx_tilemap measured that).  While the copy runs
//  (`spr_dma_rd`, a few thousand clocks) a CPU read of this window waits on
//  `objram_ok`, which carries the address it was read at, the same handshake
//  gx_tilemap's VRAM and gx_palette use.  Writes never wait.
//
//  AND THE LANES ARE NOW SPLIT BY HAND.  The paragraph above says a byte-select
//  write infers only because the read address is the write expression; with
//  the DMA's address muxed in, it is not any more, and gx_tilemap measured
//  what happens then: the build never ends.  One 8-bit array per lane, every
//  write a whole-array write.
// ---------------------------------------------------------------------------
(* ramstyle = "M10K" *) reg [7:0] objram_h [0:8191];
(* ramstyle = "M10K" *) reg [7:0] objram_l [0:8191];
reg  [7:0]  objram_qh, objram_ql;
wire [15:0] objram_q = { objram_qh, objram_ql };

wire        spr_dma_rd;
wire [12:0] spr_dma_addr;
wire        spr_dma_busy;       // d5a003 bit 1, gx_sprite's DMA busy flag
wire [21:0] spr_pf_unit;
wire        spr_pf_valid;
wire        spr_dma_done;       // one clock: a DMA copy finished (ladder rung 7)
// The sprite DMA copies from the 4 KB bank WRPOR1 bit 30 (objscan) names.
// sexyparo and daiskiss have their ESC write d20000 / d21000 in turn and set
// objscan to the bank just written after every RUN (MEASURED, PCs 289CCC and
// 287180; MEASUREMENTS 158).  Every set follows the bit -- the board does not
// know which game is plugged in.  It used to be gated to those two machines;
// MEASUREMENTS 180 ran all nineteen sets in MAME for 9,000 frames (attract,
// coin, start, play) and only sexyparo / sexyparoa / daiskiss ever write it 1,
// so the gate changed nothing.  Latched while no copy runs, so one copy reads
// one bank.
reg         spr_bank = 1'b0;
always @(posedge clk)
    if (!spr_dma_rd) spr_bank <= objscan;
wire [12:0] objram_raddr = spr_dma_rd ? (spr_dma_addr | {1'b0, spr_bank, 11'd0}) : cpu_addr[13:1];
reg         objram_cpu_d = 1'b0;
reg  [12:0] objram_addr_d = 13'd0;

always @(posedge clk) begin
    if (objram_cs && cpu_we && cpu_ds[1]) objram_h[cpu_addr[13:1]] <= cpu_dout[15:8];
    if (objram_cs && cpu_we && cpu_ds[0]) objram_l[cpu_addr[13:1]] <= cpu_dout[ 7:0];
    objram_qh <= objram_h[objram_raddr];
    objram_ql <= objram_l[objram_raddr];
    // Sampled with the read, so they describe what is at objram_q next clock.
    objram_cpu_d  <= !spr_dma_rd;
    objram_addr_d <= cpu_addr[13:1];
end

wire objram_ok = objram_cpu_d && (objram_addr_d == cpu_addr[13:1]);

// ---------------------------------------------------------------------------
//  control registers and inputs
// ---------------------------------------------------------------------------
wire [15:0] ctrl_dout;
wire        objscan;
wire        bgc_from_pal;
wire        spri_sel18, spri_sel19;

gx_ctrl u_ctrl (
    .clk          (clk),
    .rst          (rst),
    .eeprom_cs    (eeprom_cs),
    .control_cs   (control_cs),
    .sysdsw_cs    (sysdsw_cs),
    .inputs_cs    (inputs_cs),
    .service_cs   (service_cs),
    .we           (cpu_we),
    .a1           (cpu_addr[1]),
    .din          (cpu_dout),
    .ds           (cpu_ds),
    .dout         (ctrl_dout),

    .dotsel       (dotsel),
    .bgc_from_pal (bgc_from_pal),
    .irq1_en      (),           // the level; INT1 uses the ARM (D26)
    .irq1_sync_set(irq1_sync_set),
    .irq2_en      (irq2_en),
    // Decoded and not consumed yet -- see the list at the end of this file.
    .snd_run      (snd_run),
    .vram_chard   (),
    .objcha       (),
    .spri_sel19   (spri_sel19),
    .spri_sel18   (spri_sel18),
    .gfx_rst_n    (),
    .watchdog     (),
    .objscan      (objscan),
    .coin_ctr     (),
    .ee_clk       (ee_clk),
    .ee_cs        (ee_cs),
    .ee_di        (ee_di),
    .irq3_en      (irq3_en),
    .irq4_en      (irq4_en),          // the ESC, DECISIONS D18

    .p1(p1), .p2(p2), .p3(p3), .p4(p4),
    .coin(coin), .service(service),
    .dip1(dip1), .dip2(dip2),
    // d5e000: bit 3 is PORT_SERVICE_NO_TOGGLE, the rest are IPT_UNKNOWN and
    // read high.  Wired from service[0] so the service switch works before
    // there is a front end for it.
    .svc_port     ({4'hf, service[0], 3'b111}),
    // 93C46, active high and taken straight from the part -- konamigx.cpp:1272
    // reads it with IP_ACTIVE_HIGH through do_read().
    .ee_do        (ee_do),
    // gx_rdport1_3 reset value is 0xfc (gx.cpp:3983); bit 0 of the byte is the
    // EEPROM line, so the seven status bits above it are all 1 = idle -- except
    // bit 1, DMA busy, which gx_sprite drives (gx.cpp:599, :618).  Bit 7,
    // OBJINT-REQ, only falls when IRQ3 fires, and gokuparo never enables IRQ3:
    // MEASURED, eeprom_w bits 23-16 are only ever 00 / 90 / 91 / D1 in 11000
    // frames through gameplay (tools/gx_dmatap.lua).
    // Bit 3 is the ESC's (DECISIONS D18, U40): 1 from reset, cleared at the end
    // of an ESC run with IRQ4 enabled, never set again.  gokuparo reads 1.
    // The port is the byte's bits 7-1, so bit 3 is index 2.
    .rdport1_3    ({objint_b7, 3'b111, esc_b3, 1'b1, spr_dma_busy})
);

// ---------------------------------------------------------------------------
//  IRQ3, object DMA end -- MAME konamigx.cpp dmaend_callback:
//      if ((wrport1_1 & 0x84) == 0x84 || (syncen & 4)) {
//          syncen &= ~4; rdport1_3 &= ~0x80; IRQ3 HOLD_LINE }
//  Dragoon Might's main loop waits for it (2026-09-30: PC stuck at 0x203042,
//  its level-3 handler at 0x2030FE sets the flag the loop polls).  The syncen
//  bit 2 route is not modelled -- only bit 0 is carried out of gx_ctrl, and no
//  set here has been seen to use it.  OBJINT-REQ (the status byte's bit 7)
//  falls with the first IRQ3 and nothing in MAME raises it again.
// ---------------------------------------------------------------------------
//  WHEN: MAME calls dmaend_callback from the DMA-delay timer that EVERY vblank
//  starts (dmastart_callback runs whatever DMAEN says), the same moment it
//  drops the busy flag.  gx_sprite's dma_busy is that flag with that timing,
//  so the edge is its fall -- NOT spr_dma_done, which only a real copy (DMAEN
//  set) produces: 44bbf038 took that one and dragoonj, which never sets DMAEN
//  while it waits, still sat at 0x20303E.
reg  dma_busy_d = 1'b0;
always @(posedge clk) dma_busy_d <= spr_dma_busy;
assign irq3_set = dma_busy_d && !spr_dma_busy && irq3_en;
reg  objint_b7 = 1'b1;
always @(posedge clk)
    if (rst)           objint_b7 <= 1'b1;
    else if (irq3_set) objint_b7 <= 1'b0;

// ---------------------------------------------------------------------------
//  EEPROM -- Microchip 93C46, 64 x 16
//
//  konamigx.cpp:1740 and :2047 configure EEPROM_93C46_16BIT.  That width is
//  the whole reason this is jt9346 and not jt5911: jt5911 is the ER5911 at
//  128 x 8, which is what five other jtcores Konami games use, and it has one
//  MORE address bit on the wire.  docs/REUSE_PLAN.md section 2.1 and
//  sim/tb_jt9346.sv carry the argument and the rung-4 check.
//
//  The three pins come from eeprom_w bits 26-24, decoded in gx_ctrl:
//      bit 2 clk    bit 1 cs    bit 0 di        (konamigx.cpp:485-487)
//  and the read is bit 0 of the d5a000 long, active high.
//
//  ---- the default image ----------------------------------------------------
//  A blank 93C46 does not boot this game.  MAME's own comment on the region is
//  "default eeprom to prevent game booting with error" (konamigx.cpp:2163), so
//  gokuparo.nv is delivered over nvs_* before the CPU is let go.  Carrying
//  stored bytes is PLATFORM_TRANSPORT (root section 6); nothing here computes
//  anything the PCB computed.
//
//  ---- what is NOT here -----------------------------------------------------
//  There is no write-back.  jt9346's dump_flag says the game wrote something
//  and nothing reads it, so settings changed in the service menu are lost at
//  the next load.  Deliberate for P3: a save path needs sd_lba/sd_wr plumbing
//  in the target and the OSD entry to go with it.
//  TODO(P5): NVRAM save -- read dump_flag, upload through ioctl_upload.
// ---------------------------------------------------------------------------
wire ee_clk, ee_cs, ee_di, ee_do;

//  ---- the start bit, 2026-09-30 ---------------------------------------------
//  A 93C46 takes the start bit as the first DI = 1 at an SK rising edge with CS
//  high: a LEVEL.  jt9346 takes a 0 -> 1 TRANSITION of DI between two SK edges,
//  and it does not forget the last sample when CS drops.  gokuparo always clocks
//  one 0 before the start bit (MEASUREMENTS section 8), so that never showed.
//  Dragoon Might's memory check reads all 64 words in one frame with the start
//  bit on the FIRST clock after CS rises, and the previous read's data phase
//  often left DI = 1 -- so every such read was missed and the check printed
//  22D/M BAD (the EEPROM's place on the PCB; MAME's DI/CS/SK trace, frame 299).
//  So here, on every CS rise while SK is low, jt9346 is given one extra SK
//  pulse with DI = 0 (rtl/gx_eestart.sv).  In IDLE it ignores a 0, so the only
//  effect is that its last sample is 0 and the real first 1 is a transition.
//  The vendored module is unchanged.  sim/tb_gx_eestart.sv replays MAME's trace.
wire ee_sclk_j, ee_sdi_j;
gx_eestart u_eestart (.clk(clk), .rst(rst), .cs(ee_cs), .sk(ee_clk), .di(ee_di),
                      .sk_o(ee_sclk_j), .di_o(ee_sdi_j), .enable(1'b1));

jt9346 #(.AW(6), .CW(6), .DW(16)) u_eeprom (
    .rst       (rst),
    .clk       (clk),

    .sclk      (ee_sclk_j),
    .sdi       (ee_sdi_j),
    .sdo       (ee_do),
    .scs       (ee_cs),

    .dump_clk  (clk),
    .dump_addr (nvs_addr),
    .dump_we   (nvs_wr),
    .dump_din  (nvs_din),
    .dump_dout (),
    .dump_clr  (1'b0),
    .dump_flag ()
);

// ---------------------------------------------------------------------------
//  CCU
// ---------------------------------------------------------------------------
wire [9:0] hcnt;
wire [8:0] vcnt;
wire [7:0] ccu_dout;
wire       ccu_hs, ccu_vs, ccu_hblank, ccu_vblank;

wire [9:0] ras_htotal, ras_vtotal, ras_hres, ras_vres;
wire [8:0] ccu_hbp;
wire [9:0] ccu_hsw_dots;

gx_ccu u_ccu (
    .clk     (clk),
    .rst     (rst),
    .pxl_cen (pxl_cen),
    .cs      (ccu_cs),
    .we      (cpu_we),
    .addr    (cpu_addr[4:1]),
    // 8-bit device on lanes 0 and 2 of the long: the byte is always D[15:8].
    .din     (cpu_dout[15:8]),
    .dout    (ccu_dout),
    .hcnt    (hcnt),
    .vcnt    (vcnt),
    .hs      (ccu_hs),
    .vs      (ccu_vs),
    .hblank  (ccu_hblank),
    .vblank  (ccu_vblank),
    // hblank/vblank carry the same information inverted and are what the
    // mixer and the target want, so these two have no consumer.
    .hdisp   (),
    .vdisp   (),
    //  Frame constants the tile fetcher needs to wrap its lead into the
    //  raster and to know which groups are ever displayed.  gx_ccu owns them.
    .htotal_o(ras_htotal),
    .vtotal_o(ras_vtotal),
    .hres_o  (ras_hres),
    .vres_o  (ras_vres),
    .hbp_o     (ccu_hbp),
    .hsw_dots_o(ccu_hsw_dots),
    .int1    (int1),
    .int2    (int2)
);

gx_xoffs u_xoffs (
    .clk         (clk),
    .hbp         (ccu_hbp),
    .hsw_dots    (ccu_hsw_dots),
    .obj_fmt     (obj_fmt),
    .tile_dx_adj (tile_dx_adj),
    .spr_hoffset (spr_hoffset)
);

// ---------------------------------------------------------------------------
//  Video output timing, delayed by the colour pipeline's latency
//
//  The tilemap's pixel for dot h stands while hcnt = h.  gx_prio registers
//  its palette index on the next dot enable, gx_palette its RGB on the one
//  after that, and gx_colmix red/green/blue on the third -- so the colour of
//  dot h leaves this module while hcnt = h + 3.  The CCU's blanks and syncs
//  are combinational from its counter and used to leave undelayed, so every
//  picture sat three dots right of its own raster.
//
//  MEASURED, afa0806 on the board against MAME memory rebuilt into pixels:
//  once the tilemap's sub-tile scroll term is taken out (gx_tilemap, "sub-tile
//  X scroll"), all four layers are a uniform +3 px right.
//
//  So the syncs and blanks leave three dots late, and gx_colmix's blanking
//  input is tapped at two, which is where the colour of dot h is when the
//  mixer captures it.  This is a statement about THIS pipeline and nothing
//  about the PCB.  Add or remove a dot-rate stage on the colour path and this
//  depth changes with it.
// ---------------------------------------------------------------------------
reg [2:0] hb_dly = 3'b111, vb_dly = 3'b111, hs_dly = 3'b000, vs_dly = 3'b000;
always @(posedge clk) begin
    if (pxl_cen) begin
        hb_dly <= {hb_dly[1:0], ccu_hblank};
        vb_dly <= {vb_dly[1:0], ccu_vblank};
        hs_dly <= {hs_dly[1:0], ccu_hs};
        vs_dly <= {vs_dly[1:0], ccu_vs};
    end
end

assign hblank = hb_dly[2];
assign vblank = vb_dly[2];
assign hsync  = hs_dly[2];
assign vsync  = vs_dly[2];

wire mix_hblank = hb_dly[1];
wire mix_vblank = vb_dly[1];

// ---------------------------------------------------------------------------
//  tilemap
// ---------------------------------------------------------------------------
wire [19:0] tm_rom4_addr, tm_rom1_addr;
wire [31:0] tm_rom4_data;
wire [31:0] tm_rom1_data;
wire        tm_rom_req, tm_rom_ok;
wire        tm_group_done, tm_group_busy;
wire [15:0] tm_ram_dout;

wire [7:0]  px_a, px_b, px_c, px_d;
wire [7:0]  col_a, col_b, col_c, col_d;
wire [3:0]  tm_a_hit;       // layer A instrumentation, one-clock pulses
wire        tm_late;        // a group edge arrived mid-fetch (ladder rung 3)
wire        spr_late;       // gx_sprite: a line ran out of time (rungs 4-7)

gx_tilemap #(
    // EMULATION_DERIVED
    // MAME's GX screen is a 288x224 visible crop at (24,16) of the CCU's
    // 384x264 raster.  hcnt/vcnt remain 0-based for output timing; the
    // tilemap gets the crop origin as explicit address-generation offsets.
    // TODO(HARDWAREIZE): replace these once K053252/PCB crop origin is proven.
    // 2026-10-06 (MEASUREMENTS 181), what the 053252 silicon reconstruction
    // says and does not say: in the CCU the first active dot is 8*(HSW+1) +
    // HBP + 1 dots after the H load (sync start) and the first active line is
    // (VSW+1) + (VBP+1) = 23 lines after the V load; gx_ccu now matches it
    // to the dot.  So 16 = VBP+1 + 1 and X 24 - (48 - HBP) = HBP+1 - 25 read as
    // "the K056832 counts from the END of sync, plus a fixed pipeline"; the
    // fixed parts (+1, -25) are inside the K056832, whose netlist is not on
    // disk (jt05415x/doc is the 054156/054157 pair).  Every supported set
    // writes the same V setting, so the Y reading cannot be tested; kept.
    .SCREEN_X_OFFSET (10'd24),
    .SCREEN_Y_OFFSET (9'd16),
    // EMULATION_DERIVED
    // Matches MAME konamigx_v.cpp:1136-1144, set_layer_offs(-2, 0, +2, +3)
    // for every non-flipped Type 2 game ("+ve values move layers to the
    // right") -- required for bring-up.  The game itself is the evidence that
    // the PCB displaces its layers this way: its resting X scrolls are -26,
    // -24, -22, -21 = -24 + (-2, 0, +2, +3), so without these the attract's
    // four layers disagree with each other by up to 5 px.
    // The actual PCB source of the displacement is NOT verified.
    // TODO(HARDWAREIZE): UPSTREAM_TODO U5 -- K056832 per-layer output latency,
    // or the CCU?  jt05415x's behavioural x_sum has no per-layer term.
    // Read 2026-10-06 (MEASUREMENTS 181): the 054157 netlist's per-layer
    // H-offset block (page 7, hofsa/b/c/d) adds each layer's latched fine
    // scroll to hcnt[2:0]; the four latches are clocked at four phases of a
    // 4-dot cycle (B 0, A 1, C 2, D 3), which does not give -2/0/+2/+3 and
    // adds no constant per layer.  It is not the K056832 either.  Not settled.
    .LAYER_DX_A      (-10'sd2),
    .LAYER_DX_B      ( 10'sd0),
    .LAYER_DX_C      ( 10'sd2),
    .LAYER_DX_D      ( 10'sd3)
) u_tilemap (
    .clk          (clk),
    .rst          (rst),
    .reg_cs       (k056832_reg_cs),
    .reg_we       (cpu_we),
    .reg_addr     (cpu_addr[5:1]),
    .reg_din      (cpu_dout),
    .reg_ds       (cpu_ds),
    .ram_cs       (k056832_ram_cs),
    .ram_we       (cpu_we),
    .ram_addr     (cpu_addr[13:1]),
    .ram_din      (cpu_dout),
    .ram_ds       (cpu_ds),
    .ram_dout     (tm_ram_dout),
    .ram_ok       (tm_ram_ok),
    .tilebank_cs  (tilebank_cs),
    .tilebank_we  (cpu_we),
    .tilebank_addr(cpu_addr[3:1]),
    .tilebank_din (cpu_dout),
    .tilebank_ds  (cpu_ds),
    .pxl_cen      (pxl_cen),
    .hcnt         (hcnt),
    .vcnt         (vcnt),
    .htotal       (ras_htotal),
    .vtotal       (ras_vtotal),
    .hres         (ras_hres),
    .vres         (ras_vres),
    .rom4_addr    (tm_rom4_addr),
    .rom4_data    (tm_rom4_data),
    .rom1_addr    (tm_rom1_addr),
    .rom1_data    (tm_rom1_data),
    .bpp8         (tile_bpp8),
    .dx_adj       (tile_dx_adj),
    .rom_req      (tm_rom_req),
    .rom_ok       (tm_rom_ok),
    .group_done   (tm_group_done),
    .group_busy   (tm_group_busy),
    .pxl_a(px_a), .pxl_b(px_b), .pxl_c(px_c), .pxl_d(px_d),
    .col_a(col_a), .col_b(col_b), .col_c(col_c), .col_d(col_d),
    // Per-tile blend codes.  gx_prio recovers the same two bits from the
    // colour byte's bits 5-4 (they are the same field when FBITS=3, which is
    // what this game programs -- see the decode_vmixcolor note in gx_prio),
    // so these are redundant TODAY and will not be if FBITS ever changes.
    .mix_a(), .mix_b(), .mix_c(), .mix_d(),
    .dbg_a_hit(tm_a_hit),
    // Hardware-proven by the thirtieth ladder.  Keep the module/testbench
    // output, but retire its top-level route so settled debug logic drops out.
    .dbg_ref_hit(),
    .dbg_late (tm_late)          // ladder rung 3
);

// ---------------------------------------------------------------------------
//  tile ROM fetch
//
//  gx_tilemap asks for one 32-bit word from the 4bpp region and one byte from
//  the 1bpp region per 8-pixel group, and the arbiter is 16 bits wide.  The
//  32-bit read is ONE BURST-2 transaction and the byte read is one single,
//  sequenced here rather than inside the tilemap: the tilemap's contract is
//  "req, then wait for ok", which is the same shape whatever the bus
//  underneath is.
//
//  IT WAS THREE SINGLES AND THAT DID NOT FIT.  docs/DECISIONS.md D2, and the
//  numbers below are MEASURED by sim/tb_gx_sdram.sv rather than counted:
//
//      three singles per layer, four layers   109.54 clocks   85.5 % of budget
//      burst-2 + single, four layers           77.07 clocks   60.2 % of budget
//
//  The budget is 128 system clocks -- 8 dots at the 6 MHz DOTSEL=0 rate this
//  game programs.  85.5 % is the TILEMAP ALONE, before the CPU, the sprites
//  or the two K054539s ask for anything, so the old arrangement could not
//  have drawn a correct picture and it would have looked like a tilemap
//  fault.
//
//  The 1bpp region is byte-addressed and this bus is not, so the byte read
//  fetches the containing word and picks a half.  A word-wide 1bpp fetch would
//  serve two groups, but only when they are adjacent in ROM -- which four
//  independently scrolling layers do not guarantee.  Caching that is a P8
//  optimisation, not a P3 one.
// ---------------------------------------------------------------------------
//  F_W1 is gone: the two words of the 4bpp fetch now come back from one
//  burst transaction.  The remaining codes keep their values so a waveform
//  from before this change still reads the same.
localparam [2:0] F_IDLE = 3'd0, F_W0 = 3'd1, F_B = 3'd3, F_OK = 3'd4;

reg [2:0]  fst;
reg [31:0] fdata4;
reg [31:0] fdata1;     // 8 bpp: bytes 4-7 of the row
reg        f1_lo;      // the 1bpp byte is the word's low half -- latched for F_B
reg [24:0] farb_addr;
reg        farb_req;
reg        farb_burst;
reg        farb_burst4;   // 8 bpp: the whole row, four words, one transaction

//  Both of these are WORD addresses; gx_rommap.svh's constants are byte
//  addresses and the shift is written out here rather than hidden.
//
//  tm_rom4_addr counts 32-bit words, so its byte address is x4 and its word
//  address is x2.  tm_rom1_addr counts bytes, so its word address is /2 and
//  the low bit picks the half.
//
//  The x2 is also what satisfies gx_sdram's BURST CONTRACT: a_tile4 ends in a
//  hard 1'b0 and GX_TILE4_BASE is 2 MB aligned, so a burst address is even by
//  construction and can never carry out of the SDRAM column field.
wire [24:0] a_tile4  = {1'b0, GX_TILE4_BASE[24:1]} + {4'd0, tm_rom4_addr, 1'b0};
wire [24:0] a_tile1b = GX_TILE1_BASE + {5'd0, tm_rom1_addr};    // byte
// 6 bpp (salmndr2): planes 4 and 5 are one word a row
wire [24:0] a_tile1w = {1'b0, GX_TILE1_BASE[24:1]} + {5'd0, tm_rom1_addr};
wire [24:0] a_tile1  = (tile_fmt == 2'd1) ? a_tile1w : {1'b0, a_tile1b[24:1]};   // word
// 8 bpp (winspike): the row is 8 bytes = 4 words at tile4 + row*8, read as ONE
// four-word burst (a multiple of four words: the burst-4 contract).  MEASURED,
// 57678d63: as two burst-2s the groups ran late (ladder rung 3) and tiles
// showed another position's glyph.
wire [24:0] a_tile8  = {1'b0, GX_TILE4_BASE[24:1]} + {3'd0, tm_rom4_addr, 2'b00};

// arb_ack[0] the ROM loader, [1] the CPU, [2] this fetcher.  Getting these
// the wrong way round makes the tile fetcher advance on somebody else's acks,
// which is the sort of thing that looks like a tilemap bug.
wire [4:0]  arb_ack;
//  [0] ROM loader  [1] main CPU  [2] tile fetcher  [3] SOUND CPU  [4] SPRITE
//  fetcher -- and the index IS the priority, lowest first.  Client 3 being
//  last is why it starved (fifteenth ladder); `urgent` now lifts it, and
//  lifts client 4 the same way, see the instantiation.
wire [4:0]  arb_starved;
wire [15:0] arb_dout, arb_dout2;

//  ---- A PLANE-4 CACHE, 2026-09-15 -------------------------------------------
//  MEASURED (docs/MEASUREMENTS.md 41): half of the tile client's transactions
//  are the plane-4 single word, and a layer asks for the same tile row again
//  and again across a line.  From MAME's own VRAM, in this FSM's order, the
//  eight most recently INSERTED plane-4 bytes of a layer already hold the next
//  request 91.8 % of the time on the battleship, 81.9 % on the boss and
//  87-90 % in the city (tools/gx_tile1census.py, byte-keyed FIFO).  With every
//  plane-4 transaction gone, tb_gx_busmix's longest tile group went from 121
//  to 84 clocks of 128 (+tile1free) -- the tile gate is what the main CPU
//  waits on.
//
//  Eight entries per layer, flip-flops.  The lookup is registered when the
//  4bpp burst is launched; a hit answers plane 4 at the burst's ack and the
//  layer is done in ONE transaction.  A miss fetches the word as before and
//  inserts `fdata1` -- the byte this FSM has already registered -- one clock
//  later, so the cache adds no endpoint to the SDRAM return path (MEASUREMENTS
//  40).  The tile ROM changes only during the download, under `rst`.
//  `t1_lyr` counts the layer inside a group: gx_tilemap fetches layers 0..3 in
//  order and each costs exactly one request.
reg  [1:0]  t1_lyr;
reg         t1_busy_d;
reg  [19:0] t1c_key [0:31];          // {layer, slot}
reg  [15:0] t1c_dat [0:31];          // 6 bpp: both planes
reg  [31:0] t1c_v;
reg         t1_hit;
reg  [15:0] t1_hit_dat;
reg         t1_fill;
integer     t1i;

reg         t1_any;
reg  [15:0] t1_dat_c;
always @(*) begin
    t1_any   = 1'b0;
    t1_dat_c = 16'd0;
    for (t1i = 0; t1i < 8; t1i = t1i + 1)
        if (t1c_v[{t1_lyr, t1i[2:0]}] && t1c_key[{t1_lyr, t1i[2:0]}] == tm_rom1_addr) begin
            t1_any   = 1'b1;
            t1_dat_c = t1c_dat[{t1_lyr, t1i[2:0]}];
        end
end

always @(posedge clk) begin
    if (rst) begin
        fst        <= F_IDLE;
        farb_req   <= 1'b0;
        farb_burst <= 1'b0;
        farb_burst4 <= 1'b0;
        t1_lyr     <= 2'd0;
        t1_busy_d  <= 1'b0;
        t1c_v      <= 32'd0;
        t1_hit     <= 1'b0;
        t1_fill    <= 1'b0;
    end else begin
        t1_busy_d <= tm_group_busy;
        if (tm_group_busy && !t1_busy_d) t1_lyr <= 2'd0;

        // insert the byte a miss fetched, at slot 0 of its layer
        t1_fill <= 1'b0;
        if (t1_fill) begin
            for (t1i = 7; t1i > 0; t1i = t1i - 1) begin
                t1c_key[{t1_lyr, t1i[2:0]}] <= t1c_key[{t1_lyr, t1i[2:0] - 3'd1}];
                t1c_dat[{t1_lyr, t1i[2:0]}] <= t1c_dat[{t1_lyr, t1i[2:0] - 3'd1}];
                t1c_v  [{t1_lyr, t1i[2:0]}] <= t1c_v  [{t1_lyr, t1i[2:0] - 3'd1}];
            end
            t1c_key[{t1_lyr, 3'd0}] <= tm_rom1_addr;
            t1c_dat[{t1_lyr, 3'd0}] <= fdata1[15:0];
            t1c_v  [{t1_lyr, 3'd0}] <= 1'b1;
        end

        case (fst)
            F_IDLE: if (tm_rom_req) begin
                farb_addr  <= tile_bpp8 ? a_tile8 : a_tile4;
                farb_req   <= 1'b1;
                farb_burst <= 1'b1;     // the 4bpp fetch is two words
                farb_burst4 <= tile_bpp8;
                // plane 4 answered from the cache?  dragoonj has no plane-4 ROM:
                // always "answered", with zero, and never a transaction.  8 bpp
                // never uses the cache: its second half is a burst of its own.
                t1_hit     <= (t1_any && !tile_bpp8) || (tile_fmt == 2'd2);
                t1_hit_dat <= (tile_fmt == 2'd2) ? 16'd0 : t1_dat_c;
                fst        <= F_W0;
            end
            // gx_rommap's plane equations, and gx_tilemap's unpack, want
            // d[7:0] = region byte 0.  gx_download stores every SDRAM word
            // big-endian, {byte 2W, byte 2W+1}, so arb_dout = {byte0, byte1}
            // and arb_dout2 = {byte2, byte3}: the bytes have to be laid out
            // explicitly.
            //
            //  ---- MEASURED 2026-09-14: `{arb_dout, arb_dout2}` was wrong -----
            //  It put byte 0 in d[31:24], which permutes the four low planes
            //  (pen bit3<-byte0, bit2<-byte2, bit1<-byte1, bit0<-byte3).
            //  Transparency survives any permutation, so every tile SHAPE was
            //  right and every colour was wrong.  Rebuilding MAME frame 3000
            //  from its VRAM, registers and palette: with that permutation the
            //  board's text layer matched 100.0 % and its landscape 90.7 %;
            //  with MAME's plane order 45 % and 4.5 %.  The "ROM RAM CHECK"
            //  oracle could not see it because its bytes are 00 EE EE EE --
            //  three of four equal.
            //
            //  The layout lives in ONE place, gx_rommap.svh's gx_plane_word,
            //  next to the plane equations it serves; tb_gx_tilemap pins it.
            //  Sprites (GX_SPR4) will need the same function.
            //  ---- RAW RETURN, 2026-09-15 ------------------------------------------
            //  The words come straight from gx_sdram's registers (mem_raw_w1 = a
            //  burst's first word, mem_raw_q = its second, or a single word), not
            //  from arb_dout: this client is the only one that keeps the
            //  controller's ack clock (gx_memarb RESP_REG), because its group is
            //  the hard deadline.  So these captures are the only far endpoints
            //  `dq_in` still has.
            F_W0: if (arb_ack[2] && tile_bpp8) begin
                // the four words: b0 b1 = bytes 0-3, w1 q = bytes 4-7
                fdata4      <= gx_plane_word(mem_raw_b0, mem_raw_b1);
                fdata1      <= gx_plane_word(mem_raw_w1, mem_raw_q);
                farb_req    <= 1'b0;
                farb_burst  <= 1'b0;
                farb_burst4 <= 1'b0;
                fst         <= F_OK;
            end else if (arb_ack[2]) begin
                fdata4     <= gx_plane_word(mem_raw_w1, mem_raw_q);
                if (t1_hit) begin
                    fdata1     <= {16'd0, t1_hit_dat};
                    farb_req   <= 1'b0;
                    farb_burst <= 1'b0;
                    fst        <= F_OK;
                end else begin
                    farb_addr  <= a_tile1;
                    farb_burst <= 1'b0;     // the 1bpp fetch is one word
                    f1_lo      <= a_tile1b[0];
                    fst        <= F_B;
                end
            end
            F_B: if (arb_ack[2]) begin
                // Big-endian: byte 2W is in bits 15-8, so an even byte address
                // takes the high half.  Same convention as gx_download.
                // 6 bpp: the word is {byte 4, byte 5} = {plane 4, plane 5}
                fdata1   <= (tile_fmt == 2'd1) ? {16'd0, mem_raw_q[7:0], mem_raw_q[15:8]}
                          : {24'd0, f1_lo ? mem_raw_q[7:0] : mem_raw_q[15:8]};
                farb_req   <= 1'b0;
                t1_fill    <= 1'b1;
                fst      <= F_OK;
            end
            // Hold `ok` until the tilemap drops its request, so the handshake
            // cannot be missed by a state machine running on a slower enable.
            // The layer counter moves here, once per request.
            F_OK: if (!tm_rom_req) begin
                fst    <= F_IDLE;
                t1_lyr <= t1_lyr + 2'd1;
            end
            default: fst <= F_IDLE;
        endcase
    end
end

assign tm_rom4_data = fdata4;
assign tm_rom1_data = fdata1;
assign tm_rom_ok    = (fst == F_OK);

// ---------------------------------------------------------------------------
//  sound subsystem -- 68000, its ROM and RAM, and the mailbox
// ---------------------------------------------------------------------------
//  This is the thing the whole boot has been waiting on.  STATUS.md 2026-09-08
//  section 18: in the interval where the main CPU sits in its ROM/RAM CHECK
//  screen, the only device it reads is the K056800 at 0xd52010, and with no
//  sound CPU that read returned 0xff forever.
//
//  The K056800 is instantiated HERE rather than inside gx_sound because its
//  two ports are on two different CPUs' buses.  One chip, two buses.
wire       k56_s_cs, k56_s_we, k56_irq;
wire [4:1] k56_s_addr;
wire [7:0] k56_s_din, k56_s_dout, k56_h_dout;
wire [7:0] k56_s2h0;                 // the byte the main CPU polls at 0xd52010
wire [18:0] snd_dbg;
wire       snd_run;

gx_k056800 u_k056800 (
    .clk     (clk),
    .rst     (rst),
    // Host side.  8-bit device on umask32(0xff00ff00), so the byte is D[15:8]
    // of whichever half of the long the CPU addressed -- the same lane the CCU
    // takes, and for the same reason.
    .h_cs    (k056800_cs),
    .h_we    (cpu_we),
    .h_addr  (cpu_addr[4:1]),
    .h_din   (cpu_dout[15:8]),
    .h_dout  (k56_h_dout),
    // Sound side.  16-bit bus, umask16(0x00ff), so the LOW byte.
    .s_cs    (k56_s_cs),
    .s_we    (k56_s_we),
    .s_addr  (k56_s_addr),
    .s_din   (k56_s_din),
    .s_dout  (k56_s_dout),
    .snd_irq (k56_irq),
    .dbg_s2h0(k56_s2h0),
    .dbg_h2s0(),
    .dbg_h2s_int()
);

gx_sound u_sound (
    .clk      (clk),
    .rst      (rst),
    // Same condition the main CPU's enable carries and for the same reason:
    // the loader owns the memory bus, and a CPU fetching from half-written
    // SDRAM executes whatever happened to be there.  A LEVEL, not a pulse --
    // gx_m68k divides it down to 8 MHz itself.
    //
    // REGISTERED since 2026-09-28 (MEASUREMENTS 147).  `dl_active` is
    // combinational from hps_io's ioctl_index compare in gx_download, and fed
    // straight in here it made hps_io|ioctl_index -> fx68k excUnit|aob a
    // single-cycle path: -0.230 ns on the build of 41fe28e2.  The main CPU's
    // `cen_cpu` has always taken it through a register.  One clock later is
    // nothing: the HPS raises ioctl_download many clocks before its first word.
    .ce_en    (snd_ce_en),
    .snd_run  (snd_run),
    .voice_boost(voice_boost),            // every set (user, 2026-10-02): no per-game gate
    .k56_cs   (k56_s_cs),
    .k56_we   (k56_s_we),
    .k56_addr (k56_s_addr),
    .k56_din  (k56_s_din),
    .k56_dout (k56_s_dout),
    .k56_irq  (k56_irq),
    // The chip's OTHER port, for gx_sndtrace only.  Same clock, no crossing;
    // gx_sound drives none of it.  P1: the mailbox timeline is what turns the
    // 245-frame budget and the self-test's duration into measurements.
    .k56h_cs  (k056800_cs),
    .k56h_we  (cpu_we),
    .k56h_addr(cpu_addr[4:1]),
    .k56h_din (cpu_dout[15:8]),
    .k56h_dout(k56_h_dout),
    //  and the frame, so the log's durations are counted rather than converted
    .dbg_vblank(vblank),
    .rom_addr (snd_rom_addr),
    .rom_req  (snd_rom_req),
    .rom_burst(snd_rom_burst),
    .rom_ack  (arb_ack[3]),
    .rom_data (arb_dout),
    .rom_data2(arb_dout2),
    .xm_req   (sxm_req),
    .xm_we    (sxm_we),
    .xm_word  (sxm_word),
    .xm_wdata (sxm_wdata),
    .xm_be    (sxm_be),
    .xm_ack   (sxm_ack),
    .xm_rdata (sxm_rdata),
    .trc_en   (trc_en),
    .trc_valid(trc_valid),
    .trc_data (trc_data),
    .trc_take (trc_take),
    .trc_polls(trc_polls),
    .snd_l    (audio_l),
    .snd_r    (audio_r),
    .dbg_audio_ev (snd_audio_ev),
    .dbg      (snd_dbg),
    .dbg_sram_hi ()          // D30 observation point; 0 in every measured set
);

// ---------------------------------------------------------------------------
//  SDRAM arbiter
//
//  Two clients today: the CPU's program/BIOS fetch and the tile fetcher above.
//  gx_memarb's own header argues the order.
// ---------------------------------------------------------------------------
//  Three clients.  Index 0 is the highest priority, and the flattened vectors
//  are little-endian by index, so client 0's fields are the LOW slice of each
//  concatenation -- which puts it last in the {} list.  That is easy to get
//  backwards and the result is a working core that serves the wrong client
//  first, so the order is spelled out at every port below.
//
//      0  dl    ROM download.  Highest, and free: the CPU is held off the bus
//               while it runs, so it can never stall anything.
//      1  cpu   68EC020 program and BIOS fetch
//      2  tile  K056832 tile fetch
//      3  snd   sound 68000 program fetch, behind gx_romcache
//
//  WHY THE SOUND CPU IS LAST, argued rather than defaulted -- gx_memarb's own
//  header asked for a measurement when this client arrived, and
//  docs/DECISIONS.md D10 is it.  The tilemap's deadline is HARD: its shifter
//  loads whether the data came or not, 128 system clocks per 8 dots, measured
//  at 77.07 used.  A late sound fetch only makes the sound CPU slower, and a
//  slow sound CPU is late, not wrong.
//
//  THAT INVERTS WHEN THE K054539 ARRIVES.  PCM streaming has a real deadline
//  and Power Spikes paid to learn what a missed one sounds like (its
//  DEBUG_LOG O13: fifteen wrong-nibble output steps in one frame against
//  MAME's zero in 2.16 M samples).  The `urgent` port was reserved for exactly
//  that client, and this note said it would be the ONLY one ever to set a bit
//  in it.
//
//  THAT RESERVATION IS SPENT, 2026-09-09, and on measurement rather than on
//  anticipation: the sound CPU's ROM client was starved on hardware, without
//  a PCM engine existing yet.  A deadline you can predict lost to a deadlock
//  you can see.  When the PCM engine does arrive there will be TWO candidates
//  for one bit, and that is a decision to take then -- with a measurement,
//  the way this one was taken -- not a line to write now.
localparam int NARB = 5;

// Sound must not be left behind the continuously active tile client, but a
// level urgent on every sound request starves the tile FSM instead.  Alternate
// at TILE GROUP boundaries, not SDRAM transaction boundaries.  A group is the
// eight transactions that stage 4bpp+1bpp for all four layers.  Hardware
// measured that yielding after each tile ack balanced requests/completions but
// missed visible group deadlines every frame (twenty-eighth ladder), because
// it let sound split the 4bpp burst from its matching 1bpp read.  Sound has a
// soft deadline; the tile shifter's group edge is hard.  A sound completion
// therefore gives tile the bus through `tm_group_done`, then sound gets the
// next opportunity.  When either peer is idle the other remains urgent.
//
// Both sides still have to be urgent in their turn -- leaving tile at normal
// priority lets the higher-priority main CPU consume the yielded slot forever,
// which rowcov29 reproduced as sound and tile starvation.
reg snd_urgent_allow = 1'b1;
always @(posedge clk) begin
    // Unlike the arbiter itself, this policy state belongs to the running
    // cores, not to the download transport.  rst stays asserted while MiSTer
    // loads ROM after the PLL reset has gone away, so it also gives the token
    // a deterministic sound-first value at the boundary where the CPUs and
    // tile fetcher are released.
    if (rst || mem_rst)
        snd_urgent_allow <= 1'b1;
    else if (arb_ack[3])
        snd_urgent_allow <= 1'b0;
    else if (tm_group_done)
        snd_urgent_allow <= 1'b1;
end

// If only one real-time client is asking, serve it without waiting for a turn
// owned by an idle peer.  If both ask, snd_urgent_allow chooses exactly one.
wire tile_arb_urgent = farb_req    && (!snd_urgent_allow || !snd_rom_req);

// Reserving a TURN was not enough.  Between a layer's completed ROM read and
// the next layer's request, gx_tilemap spends five clocks reading VRAM and
// `farb_req` is low.  The arbiter can start a CPU or sound transaction in each
// of those holes, and an in-flight SDRAM access cannot be preempted when the
// next tile request arrives.  Hardware kept missing group deadlines after the
// token moved to tm_group_done, with request/completion counts still balanced.
//
// Hold off NEW soft-deadline ROM transactions while a visible tile group is
// being assembled.  An access already active at group start is allowed to
// finish.  At tm_group_done both clients reopen until the next group starts;
// horizontal/vertical blanking also remains entirely available.  Download is
// never gated, and the tile client itself is unchanged.
wire cpu_la_hit;     // the word asked for is the look-ahead's -- see the CPU ROM return below
wire cpu_cc_miss;    // gx_cpucache looked and does not have it -- same place
wire cpu_rom_sched_req = cpu_rom_req && !cpu_hold && !cpu_used && !cpu_la_hit && cpu_cc_miss &&
                         !tm_group_busy;
wire snd_rom_sched_req = snd_rom_req && !tm_group_busy;
wire snd_sched_urgent  = snd_rom_sched_req &&
                         (snd_urgent_allow || !farb_req);

// The sprite fetcher, client 4.  docs/DECISIONS.md D12.
//
// GATED LIKE THE CPU AND THE SOUND CPU: no new sprite transaction starts
// while a tile group is being staged, so it can never be the thing that makes
// the tilemap miss its hard group edge.  That is the lesson power_spikes' own
// arbiter header records -- a sprite engine given urgency there undid O11 --
// and it is why this is not a free-running override: `farb_req` is only ever
// high inside `tm_group_busy`, so the tile client and this one can never be
// asking in the same clock.
//
// URGENT OUTSIDE THE GROUPS, because its deadline is one line (a window of
// 6,144 clocks at DOTSEL 0) and the main CPU, which outranks it by index,
// asks almost continuously.  Without it the worst MEASURED line -- 40 sprite
// tile-rows, about 1,165 clocks of transactions (COMPUTED from D2's measured
// per-transaction cost) -- would get only the gaps the CPU leaves.  The sound
// client stays ahead of it by index when both are urgent.
// ---------------------------------------------------------------------------
//  THE GROUP TOKEN, 2026-09-22 (MEASUREMENTS 104, DECISIONS D25)
//
//  The gate above was right when a tile group used 121 of its 128 clocks.  The
//  plane-4 byte cache (2026-09-15, `t1_hit` above) cut the group to 101 and
//  THE GATE KEPT REFUSING THE 27 CLOCKS THAT FREED UP.  MEASURED with
//  `+linestat` on sexyparo's character select, which is the screen that tears:
//  of a late line's 6,144 clocks, 3,102 were the sprite holding a request this
//  gate would not pass, against 997 spent on its own SDRAM transactions.
//
//  AND `starve` COULD NEVER HAVE SHOWN IT.  gx_memarb's starvation counter
//  needs `req && !ack` held for 1,024 clocks, and what reaches it as `req[4]`
//  is this already-gated signal -- so when the gate closes, req[4] falls and
//  the counter resets.  Five measurements read `starve 0` off a client blocked
//  for half of every line.
//
//  So: let up to GRP_TOK sprite transactions START inside a group.  The tile
//  client is urgent and lower-indexed, so it wins every clock it asks, and a
//  token can only be spent in the VRAM gaps BETWEEN layers where `farb_req` is
//  low.  A group has three such gaps, and the measurement saturates at three.
//
//  MEASURED, charsel 15903, shipped policy, VRAM loaded (MEASUREMENTS 104.4):
//      K=0  plane 75.657 %   group 101/128      K=2  91.386 %   110
//      K=3  plane 99.290 %   group 112/128      K=4  99.240 %   111
//  and `tm_late` is 0 at every K, as is `over 120`.
//
//  WHY THE ACK AND NOT THE GRANT.  gx_memarb has no grant output, and adding
//  one would touch a hardware-verified module.  The arbiter holds `busy` from
//  grant to `m_ack` and allows ONE outstanding transaction, so starts and
//  completions alternate and counting either gives the same number of starts.
//  That is MEASURED, not argued: the bench was run both ways (`+grptokack`)
//  and K=3 and K=4 give byte-identical late lines, group length and plane.
//
//  SAFETY, from a 302-frame sweep of every sexyparo dump in dist/ with VRAM
//  loaded: `tm_late` 0 on every frame, `over 120` zero on every frame, worst
//  group 119 of 128.  That 119 is NOT this token's doing -- the frames that
//  reach it give the SAME 118/119 at K=0, 1, 2 and 3, because a group already
//  that long has no gaps to spend a token in.  And the attract frames the
//  board is scored on are bit-identical at K=0 and K=3, plane 100.000 %.
// ---------------------------------------------------------------------------
// 2026-09-23: 3 -> 4, user decision on MEASUREMENTS 115-118.  D25 chose 3 on
// the SINGLE-FRAME plane score, which is structurally blind to flicker; the
// frame-to-frame metric (115.1) reads 173 flickering dots at 3 and 7 at 4,
// and 4 is better at all four contention levels measured (117.1).  The user
// confirmed on the board that 4 is the steadiest setting and that the
// portrait changes properly at it (116.1).  Costs no memory and no DSP.
localparam integer GRP_TOK = 4;
// MEASUREMENTS 113: D25 chose 3 on a bench that reads `tm_late 0` at every
// sound rate, and the BOARD lights ladder rung 3 -- a tile group edge arriving
// mid-fetch -- on SELECT PLAYER.  So the board is paying for this gate and the
// bench cannot price it.  `grp_tok_sel` makes the two candidates measurable
// from one bitstream; selector 0 is the shipped value, so a .mra that does not
// set it behaves exactly as before.
//
//   sel 0 -> 4 (SHIPPED)   sel 1 -> 3 (the previous default, for regression)
//   sel 2 -> 5             sel 3 -> 0 (the control: the gate fully closed)
//
// No released `.mra` sets bits 21-22, so remapping the VALUES here re-points
// nothing in the field -- the trap root CLAUDE.md 1.4.5 records is about
// moving the BITS, and they have not moved.
//
// Flicker at each, charsel 15903 at the shipped policy with the measured
// sound traffic (MEASUREMENTS 115.2): 7 / 173 / 45 / 2,384 dots of 64,512.
// ---- 2026-09-28: sel 0 is now the DEADLINE gate (MEASUREMENTS 144) -------
//  The board said 3 and 4 were barely better than before and 5 was the best
//  picture -- no ghosting, no occlusion -- but with the portrait's
//  surroundings "자글자글".  The bench prices 5 at 2-8 late tile groups a
//  frame (groups of 129-136 clocks against 128): that is the shimmer.  A
//  token counts TRANSACTIONS and cannot see the deadline, so every count
//  trades one defect for the other.
//
//  sel 0 lets up to seven sprite transactions start inside a group, but each
//  only if it can finish and leave the tile fetcher the time its remaining
//  layers need:
//
//      clocks into the group + (4 - layers done) x DL_L + DL_S  <=  DL_D
//
//  swept in tb_gx_busmix over the six charsel frames and 10x / 20x sound:
//  124 / 18 / 14 is the point where tm_late is 0 at the measured sound rate on
//  every frame and the sprite margin (late lines) is 5's, not 4's; at 10x and
//  20x sound it beats 4 on both axes.  Eight other sexyparo frames unchanged.
//  `t1_lyr` (the tile adapter's F_OK count) is the layers done.  The verdict
//  is REGISTERED -- one clock stale, which only makes it more conservative --
//  so no adder sits in front of the arbiter's pick.
//
//   sel 0 -> deadline gate, cap 7 (SHIPPED)   sel 1 -> 4 (the 2026-09-23 value)
//   sel 2 -> 5                                sel 3 -> 0 (the gate fully closed)
//
// ---- 2026-09-28, later: the tile QUEUE makes 5 the default (MEASUREMENTS 146)
//  The board still showed late groups at the deadline gate (ladder rung 3),
//  and no constant closed them (145): the fetch had no slack at all.
//  gx_tilemap now fetches into a two-group queue (GQ), and with it tb_gx_busmix
//  reads ZERO late groups for Auto, 5, 6 and 7 at every sound rate measured
//  (1x-20x).  5 has the best sprite margin and is the one the board preferred
//  for sprites, so:
//
//   sel 0 -> 5 (SHIPPED)                      sel 1 -> deadline gate, cap 7
//   sel 2 -> 4                                sel 3 -> 0 (the gate fully closed)
//
// EMULATION_DERIVED is not the right tag -- this is our bus policy, not the
// board's; the PCB has no such arbiter.  DL_* are bench-fitted constants.
localparam integer DL_D = 124, DL_L = 18, DL_S = 14;
wire       grp_dl  = (grp_tok_sel == 2'd1);
wire [2:0] grp_tok_val = (grp_tok_sel == 2'd0) ? 3'd5 :
                         (grp_tok_sel == 2'd1) ? 3'd7 :
                         (grp_tok_sel == 2'd2) ? 3'(GRP_TOK) : 3'd0;

reg       tm_busy_d = 1'b0;
reg [2:0] grp_tok   = 3'd0;
always @(posedge clk) begin
    if (rst) begin
        tm_busy_d <= 1'b0;
        grp_tok   <= 3'd0;
    end else begin
        tm_busy_d <= tm_group_busy;
        if (tm_group_busy && !tm_busy_d)
            grp_tok <= grp_tok_val;
        else if (tm_group_busy && arb_ack[4] && grp_tok != 3'd0)
            grp_tok <= grp_tok - 3'd1;
    end
end

reg  [7:0] grp_clk  = 8'd0;
reg        spr_fits = 1'b0;
wire [7:0] dl_rem   = (t1_lyr == 2'd0) ? 8'(4 * DL_L) :
                      (t1_lyr == 2'd1) ? 8'(3 * DL_L) :
                      (t1_lyr == 2'd2) ? 8'(2 * DL_L) : 8'(DL_L);
always @(posedge clk) begin
    if (rst) begin
        grp_clk  <= 8'd0;
        spr_fits <= 1'b0;
    end else begin
        if (tm_group_busy && !tm_busy_d)                grp_clk <= 8'd1;
        else if (tm_group_busy && grp_clk != 8'hff)     grp_clk <= grp_clk + 8'd1;
        spr_fits <= !grp_dl ||
                    ({1'b0, grp_clk} + {1'b0, dl_rem} + 9'(DL_S) <= 9'(DL_D));
    end
end

wire spr_rom_sched_req = sfarb_req && (!tm_group_busy || (grp_tok != 3'd0 && spr_fits));
wire spr_sched_urgent  = spr_rom_sched_req;

//  ---- THE TWO-TIER RETURN, 2026-09-15 (MEASUREMENTS 40, 45) ------------------
//  Clients 1 (CPU) and 3 (sound) take a response registered beside the
//  arbiter, acked one clock after the controller: one clock more per SDRAM
//  transaction; look-ahead and gx_cpucache hits unchanged.  Clients 2 (tiles)
//  and 4 (sprites) keep the controller's ack clock and read gx_sdram's raw
//  registers themselves.  Client 0 (download) only writes.  Why and how:
//  gx_memarb's RESP_REG header.
//
//  THE SPRITES ARE RAW BY MEASUREMENT, NOT BY THE DESIGN THAT WAS REVIEWED.
//  Codex's two-tier return put them on the registered tier.  tb_gx_busmix,
//  four frames each, the same flags as MEASUREMENTS 41 (MEASUREMENTS 45):
//      sprites registered   boss 15200: late sprite lines in warm frames
//                           0/0/0 -> 14/5/9, sprite plane 100 % -> 99.949 %
//      CPU registered       CPU words 29,758 -> 29,806 (boss), 34,359 -> 34,624
//                           (battleship): nothing lost
//  A sprite line's window is the tight one (D12's ring of eight); a clock per
//  transaction is what it does not have.
gx_memarb #(.N(NARB), .RESP_REG(32'b01010)) u_arb (
    .clk           (clk),
    //  `mem_rst`, NOT `rst`.  MEASURED on hardware 2026-09-08: with `rst` here
    //  the core would not load a ROM at all -- not a 14 MB one and not a
    //  256-byte one.  `load_core` of the bare .rbf worked and an .mra with no
    //  <rom> at all worked, so the FPGA and the .mra were both fine; anything
    //  with a <rom> put the board back at MENU.
    //
    //  The chain: gx_download raises `busy` on the first word and clears it
    //  only on `dl_ack`, which is `arb_ack[0]` from this arbiter.  `busy` is
    //  `ioctl_wait`, which is HPS_BUS[37], which stops sys_top ever raising
    //  io_ack -- and MiSTer's main spins on io_ack for EVERY word.  So an
    //  arbiter held in reset while the download runs wedges the HPS on word
    //  one.  gx_download's own header describes exactly this failure and
    //  exactly this symptom; it just did not own the reset that caused it.
    //
    //  `rst` is rst_sys = RESET | status[0] | buttons[1] | ~pll_locked, which
    //  is the BOARD's reset.  The memory transport has to keep running across
    //  it -- that is what this port's comment says at the top of this file,
    //  and the "Unread inputs" note at the bottom used to rationalise ignoring
    //  it.  projects/dataeast/stadium_hero/rtl/sh_top.sv:340 wires its
    //  arbiter the same way and that core loads ROMs on this board.
    //
    //  Held down by sim/tb_gx_dlpath.sv.
    .rst           (mem_rst),
    //  ---- THE SOUND CPU HAS NOW EARNED ONE, 2026-09-09 -------------------
    //  MEASURED, fifteenth ladder, five identical captures: client 3 was
    //  asking and not being granted for >=1024 CONSECUTIVE clocks while
    //  client 1 was not.  `pick` is the lowest set request and this input was
    //  tied off, so client 3 is granted only when 0, 1 and 2 are ALL idle --
    //  and between them they never leave a gap.  gx_romcache then never
    //  leaves S_FILL, `c_ack` never fires, and a 68000 with no DTACK and no
    //  BERR holds that cycle forever.  The main CPU polls a mailbox the sound
    //  CPU can no longer write, and its own fetches help keep client 3 out:
    //  a deadlock by priority inversion.
    //
    //  This is the SECOND pass of gx_memarb's selection, which existed from
    //  the start and had never been reachable on this board.  The sound CPU
    //  asks rarely -- 139,577 cache misses across an entire self-test against
    //  a tile fetcher that uses 77 of every 128 clocks -- so the cost to the
    //  fetcher should be one SDRAM transaction per sound miss.  A level urgent
    //  on every sound request violated that contract: whenever tile fetch and
    //  sound were both waiting, sound won again immediately and the tile FSM
    //  made only a few scanlines of progress.  A simple `!farb_req` gate went
    //  the other way on hardware: the tile client is active almost
    //  continuously, so the sound CPU starved again.  The transaction-boundary
    //  token above alternates sound and tile instead.
    //
    //  DECISIONS D10 argued a late sound fetch "only makes the sound CPU
    //  slower".  That predates the K054539 timer's deadline, and starvation
    //  is not lateness.
    .urgent        ({spr_sched_urgent, snd_sched_urgent, tile_arb_urgent, 2'b00}),
    //                spr                snd                tile        cpu                dl
    .req           ({ spr_rom_sched_req, snd_rom_sched_req, farb_req,   cpu_rom_sched_req, dl_req   }),
    .addr          ({ sfarb_addr,        snd_rom_addr,      farb_addr,  cpu_rom_addr,      dl_addr  }),
    .din           ({ 16'd0,             16'd0,             16'd0,      16'd0,             dl_data  }),
    .we            ({ 1'b0,              1'b0,              1'b0,       1'b0,              1'b1     }),
    //  The tile and sprite fetchers burst.  The CPU could -- TG68K splits
    //  every long access into two consecutive word accesses -- but answering
    //  the second from a prefetch buffer means new state inside gx_main, and
    //  gx_main is the one module on this board that cannot be simulated at
    //  all (TG68K is VHDL).  Worth roughly half of the CPU's fetch clocks;
    //  not worth adding unsimulatable state to reach a first screen.
    .burst         ({ sfarb_burst,       snd_rom_burst,     farb_burst, !cpu_rom_addr[0],  1'b0     }),
    //  Four words: the sprite client only, and it is a raw client (RESP_REG
    //  bit 4 clear) as gx_memarb requires.
    .burst4        ({ sfarb_burst4,      1'b0,              farb_burst4, 1'b0,              1'b0     }),
    .ds            ({ 2'b11,             2'b11,             2'b11,      2'b11,             2'b11    }),
    .ack           (arb_ack),
    .dout          (arb_dout),
    .dout2         (arb_dout2),
    .m_addr        (mem_addr),
    .m_din         (mem_din),
    .m_dout        (mem_dout),
    .m_dout2       (mem_dout2),
    .m_req         (mem_req),
    .m_we          (mem_we),
    .m_burst       (mem_burst),
    .m_burst4      (mem_burst4),
    .m_ds          (mem_ds),
    .m_ack         (mem_ack),
    .dbg_cpu_stall (dbg_cpu_stall),
    .dbg_starved   (arb_starved)
);

// ---------------------------------------------------------------------------
//  The CPU's read return, registered -- data AND ack on the same edge
//
//  The last failing path in the design was
//
//      gx_sdram|dout[11] -> TG68KdotC_Kernel|opcode[3]     slack -0.365
//
//  the read data walking from the SDRAM controller, which sits at the pins,
//  through the arbiter and gx_main's read mux, into a CPU register deep in the
//  core.  It is a genuine single-cycle requirement -- the destination is
//  clkena-gated but the two enables have no fixed phase relationship, so no
//  exception can honestly relax it -- and the distance is physical.
//
//  A reseed was tried first, because build.sh records this clock varying from
//  -0.044 to +0.720 across builds of the same RTL.  It did not hold: seed 11
//  gave -0.365 and seed 3 gave -0.769.  Two seeds failing is a marginal path,
//  not bad luck, so this is the structural answer.
//
//  BOTH SIGNALS MOVE ON THE SAME EDGE and that is the whole design.  Delaying
//  the data without the ack would hand the CPU a stale word on the clock it
//  finally advances; delaying the ack without the data would do the opposite.
//  Registered together, gx_main sees exactly what it saw before, one clock
//  later, and its stall logic needs no change: it waits for ack either way.
//
//  Cost: one system clock per CPU SDRAM read, on top of the ~9 the transaction
//  already takes, and the CPU advances only every 4 clocks anyway.
// ---------------------------------------------------------------------------
//
//  ---- AND HELD UNTIL THE CPU TAKES IT, 2026-09-14 ---------------------------
//  MEASURED, sim/tb_gx_busmix.sv (real gx_main, gx_memarb, gx_sdram; bus-level
//  CPU model; docs/MEASUREMENTS.md 21): with the ack a one-clock pulse the CPU
//  issued 4.0 SDRAM transactions per ROM word it consumed.  gx_main advances
//  only on `cen`, one clock in four, and only when rom_ack is high on that
//  clock -- so an ack that lands between enables is thrown away, the request
//  is still up, the arbiter grants it again and the same word is read again.
//  A free bus lands on the enable; any wait that is not a multiple of four
//  clocks (a tile group, a sprite transaction, the SDRAM's own post-access
//  wait) does not, and each retry moves the phase by one.
//
//  `cpu_hold` keeps the word and the ack from the arbiter's ack until the
//  enable that consumes it; `cpu_used` then withdraws the request for the two
//  clocks until gx_main's slice (loaded on cen_d2) shows the next access.  The
//  CPU sees exactly the word it saw before on exactly the enable it would have
//  taken it, when that enable was hit; only the re-reads are gone.
//  tb_gx_busmix checks every consumed word against the ROM image.
//
//  ---- AND THE NEXT WORD KEPT, 2026-09-15 -------------------------------------
//  MEASURED (docs/MEASUREMENTS.md 38): in the demo stages the CPU is frozen on
//  ROM reads for 79 % of a frame, two thirds of it waiting for tile groups, and
//  89 % of the words the program reads are the previous word + 1 (MAME trace,
//  tools/gx_romtrace.lua; TG68K also splits every long into two words).  So
//  the CPU's read is a 2-word burst, and the second word is kept: a request for
//  exactly that word is answered from it, with no SDRAM transaction and no
//  tile-group wait, through the same hold/used handshake as above.
//  gx_sdram's BURST CONTRACT allows a burst only from an EVEN word (column
//  0x3ff would carry into the bank field), so odd words are single reads and
//  leave no look-ahead.  tb_gx_busmix +cpula=1: 0.50 SDRAM transactions a
//  word, CPU words +31 % (6260) / +32 % (13300), no tile group late; its first
//  run burst from odd words too and the testbench's word check caught 168
//  wrong words.
//
//  ---- AND A CACHE IN FRONT OF IT ALL, 2026-09-15 ------------------------------
//  MEASURED on the board (docs/MEASUREMENTS.md 41): the look-ahead took the
//  city demo from 61 % to 82 % of MAME's pace and left the battleship at
//  48.5 %, whose work is steady just above what reaches the CPU.  Words that
//  never touch the bus come only from on-chip memory: gx_cpucache, 2^10 words,
//  looked up for every request the look-ahead does not answer.  A hit is
//  served through the same hold/used handshake; a miss asks the arbiter one
//  clock later than before.  It fills from cpu_rd_q and cpu_la_data -- these
//  registers -- and never from arb_dout.  tb_gx_busmix instantiates the same
//  module: +21 % CPU words at the battleship, +15 % in the city (think 8).
wire        cpu_cc_serve;
wire [15:0] cpu_cc_q;

reg [15:0] cpu_rd_q;
reg        cpu_hold;       // acked, not yet consumed at cen
reg        cpu_used;       // consumed; gx_main's slice still shows that access
reg        cen_cpu_d1, cen_cpu_d2;
reg [24:0] cpu_la_addr;    // the burst's second word, and where it came from
reg [15:0] cpu_la_data;
reg        cpu_la_valid;
wire       cpu_burst = !cpu_rom_addr[0];
assign     cpu_la_hit = cpu_la_valid && cpu_rom_req && !cpu_hold && !cpu_used &&
                        (cpu_rom_addr == cpu_la_addr);
always @(posedge clk) begin
    cen_cpu_d1 <= cen_cpu;
    cen_cpu_d2 <= cen_cpu_d1;     // gx_main's cen_d2: the slice loads here
    if (rst) begin
        cpu_rd_q     <= 16'd0;
        cpu_hold     <= 1'b0;
        cpu_used     <= 1'b0;
        cpu_la_valid <= 1'b0;
    end else begin
        if (arb_ack[1]) begin
            cpu_rd_q     <= arb_dout;
            cpu_hold     <= 1'b1;
            cpu_la_addr  <= cpu_rom_addr + 25'd1;
            cpu_la_data  <= arb_dout2;
            cpu_la_valid <= cpu_burst;
        end else if (cpu_la_hit) begin
            cpu_rd_q     <= cpu_la_data;
            cpu_hold     <= 1'b1;
            cpu_la_valid <= 1'b0;
        end else if (cpu_cc_serve) begin
            cpu_rd_q     <= cpu_cc_q;
            cpu_hold     <= 1'b1;
        end
        if (cen_cpu && cpu_hold) begin
            cpu_hold <= 1'b0;
            cpu_used <= 1'b1;
        end
        if (cen_cpu_d2) cpu_used <= 1'b0;
    end
end

gx_cpucache #(.IDX_BITS(10)) u_cpucache (
    .clk      (clk),
    .rst      (rst),
    .look     (cpu_rom_req && !cpu_hold && !cpu_used && !cpu_la_hit),
    .addr     (cpu_rom_addr),
    .serve    (cpu_cc_serve),
    .q        (cpu_cc_q),
    .miss     (cpu_cc_miss),
    .ack      (arb_ack[1]),
    .fill_two (cpu_burst),
    .fill_d1  (cpu_rd_q),
    .fill_d2  (cpu_la_data)
);

assign cpu_rom_ack  = cpu_hold;
assign cpu_rom_data = cpu_rd_q;
assign dl_ack       = arb_ack[0];

// ---------------------------------------------------------------------------
//  palette, priority, mixer
//
//  hpos/vpos are the position inside the ACTIVE area; gx_ccu counts from 0 at
//  the start of active display, so they are hcnt and vcnt directly.  The
//  tilemap and the gradient backdrop are both given the crop origin (24,16)
//  as explicit parameters, so output timing never moves.
//
//  EMULATION_DERIVED -- the backdrop origin.  MAME's fill_backcolor starts
//  pal_ptr at base + cliprect.min_y (k054338.cpp:134), and cliprect is in
//  bitmap coordinates whose visible area begins at y = 16.  MEASURED
//  2026-09-14 on MAME frame 3000: its snapshot's backdrop column matches
//  base + vpos + 16 on 98 % of 164 sampled rows and base + vpos on none.
//  TODO(HARDWAREIZE): the same U4/U5 crop-origin question as the tilemap.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  sprites -- K053246 + K055673
//
//  rtl/video/gx_sprite.sv carries the design; docs/DECISIONS.md D12 the
//  argument for where its ROM comes from and at what priority.
// ---------------------------------------------------------------------------
wire [21:0] spr_unit;
wire        spr_req;
wire        spr_ok;
wire [31:0] spr_data4;
wire [31:0] spr_data1;
wire [7:0]  spr_pen;
wire [15:0] spr_c18;
wire [9:0]  spr_attr;
wire [1:0]  spr_shd;
wire [3:1]  spr_shdm;         // every shadow code at the dot (STACKED SHADOWS)
wire [3:0]  spr_coregshift;
wire        spr_sdsel;
wire        spr_shd_front;

wire [3:1] shd_live;         // gx_colmix -> gx_sprite, SHADOW PRESET LIVE

gx_sprite #(
    // EMULATION_DERIVED
    // GOKUPARO-SPECIFIC
    // Matches MAME gx.cpp:1815, set_config(K055673_LAYOUT_GX, -46, -23) for
    // gokuparo (the base konamigx() machine says -26; sexyparo -42) --
    // required for bring-up.  gx_objscan puts a sprite at w3 - (xoffset -
    // HOFFSET) in line-buffer dots and at vline + w2 + yoffset + VOFFSET in
    // its own rows, and MAME's gxcore puts it at w3 + dx - offx in bitmap X
    // and w2 - dy - offy in bitmap Y, whose visible area starts at (24,16):
    //     HOFFSET = dx - 24  = -70  = 10'd954     gokuparo, dx = -46 (:1815)
    //     HOFFSET = dx - 24  = -66  = 10'd958     sexyparo, dx = -42 (:1824)
    //     HOFFSET = dx - 24  = -50  = 10'd974     A2 shelf, dx = -26 (:1772)
    //     VOFFSET = -dy + 16 =  39                all,      dy = -23
    // HOFFSET has been a port since 2026-09-16, chosen by `sexyparo`
    // (DECISIONS D18); the Y offset is the same for every set.
    //
    // 2026-09-29: ONE value again.  MAME's -42 (sexyparo) and -26 (A2 shelf)
    // compensated for its ESC HLE, which did not add the offset the chips'
    // programs add to x (+4, and +0x10 first for tbyahhoo / daiskiss).  With
    // the programs in gx_esc all three are gokuparo's -46, the value their
    // identical OBJSET1 offsets always implied (U6's caveat above; MEASUREMENTS
    // 159).  The mame-src build with the programs uses -46 for all of them.
    //
    // -26 IS THE BASE MACHINE'S AND NOBODY OVERRIDES IT (SOURCE_AUDIT 19.1,
    // UPSTREAM_TODO U48).  tbyahhoo(), which mtwinbee shares and daiskiss does
    // not use at all, only repeats the BPP_5 config konamigx() already set
    // (:1761, :1805-1808), so all three take K055673_LAYOUT_GX, -26, -23 from
    // the base.  U6's caveat stands for this value as it does for the other
    // two: gokuparo and sexyparo write IDENTICAL OBJSET1 offsets while MAME
    // gives them different dx, so at most one of the three is the board's.
    // tb_gx_sprite checks that correspondence against a forward model of
    // gxcore, whole screen, both screen-flip settings.
    // The actual PCB source of the two numbers is NOT verified.
    // TODO(HARDWAREIZE): UPSTREAM_TODO U6 -- the same CCU / crop-origin
    // question as the tilemap's SCREEN_X_OFFSET and LAYER_DX.
    // 2026-10-06 (MEASUREMENTS 181): MAME's dy = -23 is exactly the silicon
    // 053252's distance from the V load (vsync start) to the first active
    // line, (VSW+1) + (VBP+1) = 8 + 15, so VOFFSET 39 = 16 + 23 reads as "the
    // sprite chip counts lines from the V load".  Consistent, not proven: no
    // K055673 netlist on disk and every set writes the same V setting.
    .VOFFSET (10'd39),
    // Eight line buffers: a line may be drawn up to seven lines ahead of the
    // display.  MEASURED, tb_gx_busmix on the demo stages' consecutive MAME
    // frames (docs/MEASUREMENTS.md 35): two buffers leave 43 / 21 late lines
    // a frame, four and eight none, plane 100 %; a cold row cache (a scene
    // change) leaves 69 / 84 with four and 43 / 59 with eight.  Eight costs
    // +6 M10K against four's +1 (COMPUTED; the fit's RAM summary measures it).
    //
    // SIXTEEN since 2026-09-23 (MEASUREMENTS 140).  With the real four-word
    // burst the row fetch is ~1 clock slower on average than 138's fixture
    // priced, and the character select's broken frames read 94.4-98.6 % at
    // eight; at sixteen all six frames of dist/charsel_dump read 100.000 %,
    // and 16068 holds 100.000 % at 20x the measured sound traffic.  Costs
    // +8 M10K (handoff estimate; the fit's RAM summary is the measurement).
    .LB_AW   (4),
    // gx_objdraw change 11: fetch the next tile's first half during this
    // tile's dots.  MEASUREMENTS 132-139; shipped with gx_sprfetch BURST4
    // below, because neither is worth much alone (138.2: 83.9 -> 85.8 % and
    // 83.9 -> 84.7 %) and together they draw frame 16068 at 100.000 %.
    .PIPE    (1)
) u_sprite (
    .clk            (clk),
    .rst            (rst),
    .pxl_cen        (pxl_cen),
    .hoffset        (spr_hoffset),
    .objset1_cs     (objset1_cs),
    .objset2_cs     (objset2_cs),
    .cpu_we         (cpu_we),
    .cpu_addr       (cpu_addr[3:1]),
    .cpu_din        (cpu_dout),
    .cpu_ds         (cpu_ds),
    .spri_sel18     (spri_sel18),
    .spri_sel19     (spri_sel19),
    .dma_rd         (spr_dma_rd),
    .dma_addr       (spr_dma_addr),
    .objram_q       (objram_q),
    .dma_busy       (spr_dma_busy),
    .dma_hold       (esc_busy | fj_busy), // a run of either master defers the copy (D18, D19)
    .dma_quiet      (spr_dma_quiet),
    .hcnt           (hcnt),
    .vcnt           (vcnt),
    .htotal         (ras_htotal),
    .vtotal         (ras_vtotal),
    .hres           (ras_hres),
    .vres           (ras_vres),
    .fmt            (obj_fmt),
    .spr_big        (spr_big),
    .rom_unit       (spr_unit),
    .rom_req        (spr_req),
    .rom_ok         (spr_ok),
    .rom_data4      (spr_data4),
    .rom_data1      (spr_data1),
    .pix_pen        (spr_pen),
    .pix_c18        (spr_c18),
    .pix_attr       (spr_attr),
    .pix_coregshift (spr_coregshift),
    // The dot's SHADOW pixel -- its own buffer in gx_sprite since 2026-09-14,
    // a stream over whatever wins in gx_prio, not a property of the solid pen.
    .shd_live       (shd_live),
    .pix_shd        (spr_shd),
    .pix_shdm       (spr_shdm),
    .pix_sdsel      (spr_sdsel),
    .pix_shd_front  (spr_shd_front),
    .dbg_late       (spr_late),       // ladder rungs 4-6
    .dbg_dma        (spr_dma_done),   // ladder rung 7
    .pf_unit        (spr_pf_unit),
    .pf_valid       (spr_pf_valid)
);

// ---------------------------------------------------------------------------
//  sprite ROM fetch -- arbiter client 4
//
//  rtl/memory/gx_sprfetch.sv since 2026-09-14, so tb_gx_busmix runs it rather
//  than a copy.  row_prefetch: a tile row's other half is fetched in the same
//  held request as the first, so the main CPU is not granted between a row's
//  halves.  MEASURED in tb_gx_busmix on MAME frames (docs/MEASUREMENTS.md 21):
//  late sprite lines 144 -> 16 on frame 1850 with the CPU idle, and 32 -> 0 on
//  910; the busiest line of the attract (2240) keeps 144 and is not this fix.
// ---------------------------------------------------------------------------
wire [24:0] sfarb_addr;
wire        sfarb_req, sfarb_burst, sfarb_burst4;

gx_sprfetch #(
    .CACHE_AW     (12),
    // One four-word burst per tile row instead of two burst-2s: one SDRAM row
    // activation where there were two (MEASUREMENTS 135.1, 138).  With PIPE
    // on u_sprite.  The ROM image and every .mra are unchanged.
    .BURST4       (1)
) u_sprfetch (
    .clk          (clk),
    .rst          (rst),
    .spec_ok      (1'b0),               // SPEC_PF is 0 here; see MEASUREMENTS 122.8
    .w1_direct    (16'd0),              // W1FREE is 0 here; MEASUREMENTS 135
    .oth_direct   (32'd0),              // OTHFREE is 0 here; MEASUREMENTS 138
    .hint_unit    (spr_pf_unit),
    .hint_valid   (spr_pf_valid),
    .row_prefetch (1'b1),
    .row_cache    (1'b1),
    .fmt          (obj_fmt),
    .unit         (spr_unit),
    .req          (spr_req),
    .ok           (spr_ok),
    .data4        (spr_data4),
    .data1        (spr_data1),
    .arb_addr     (sfarb_addr),
    .arb_req      (sfarb_req),
    .arb_burst    (sfarb_burst),
    .arb_burst4   (sfarb_burst4),
    .arb_ack      (arb_ack[4]),
    //  The raw tier (see u_arb): a burst's first word is raw_w1 and its second
    //  raw_q, a single word is raw_q.  `sfarb_burst` is this client's own
    //  register and still describes the acked transaction on the ack clock.
    .arb_dout     (sfarb_burst ? mem_raw_w1 : mem_raw_q),
    .arb_dout2    (mem_raw_q),
    //  ... and a four-word burst's first two words, which only the raw tier has
    .arb_b0       (mem_raw_b0),
    .arb_b1       (mem_raw_b1)
);

wire [12:0] idx0, idx1;
wire        bg0, bg1;
wire [1:0]  blend, bri, bri1, shadow;
wire [1:0]  blend_e, shadow_e;       // one stage earlier, for gx_colmix's K054338 selection (R1)
wire [3:1]  shadow_m, shadow_me;     // STACKED SHADOWS: every preset that lands (gx_prio -> gx_colmix)
wire [23:0] rgb0, rgb1;
wire [15:0] pal_dout, k338_dout;

gx_prio #(
    .BG_X_OFFSET (10'd24),
    .BG_Y_OFFSET (9'd16)
) u_prio (
    .clk            (clk),
    .rst            (rst),
    .pxl_cen        (pxl_cen),
    .reg_cs         (k055555_cs),
    .reg_we         (cpu_we),
    .reg_addr       (cpu_addr[8:1]),
    .reg_din        (cpu_dout[15:8]),
    .pal_gran16     (pal_gran16),
    .bgc_from_pal   (bgc_from_pal),
    .hpos           (hcnt),
    .vpos           (vcnt),
    .px_a(px_a), .px_b(px_b), .px_c(px_c), .px_d(px_d),
    .col_a(col_a), .col_b(col_b), .col_c(col_c), .col_d(col_d),
    // The sprite chip's one pixel per dot: the smallest zcode that is opaque
    // there, with that entry's c18.  gx_sprite's header says why one is enough.
    .px_o           (spr_pen),
    .obj_c18        (spr_c18),
    .obj_fmt        (obj_fmt),
    .obj_attr       (spr_attr),
    .obj_coregshift (spr_coregshift),
    .obj_shd        (spr_shd),         // sprite shadows, see u_sprite
    .obj_shdm       (spr_shdm),
    .obj_sdsel      (spr_sdsel),
    .obj_shd_front  (spr_shd_front),
    // Type 2 has no ROZ and no sub-layers; ENABLE's top three bits are 0 in
    // the measured trace, so these are doubly disconnected.
    .px_s           (15'd0),
    .col_s          (24'd0),
    .idx0(idx0), .idx1(idx1), .bg0(bg0), .bg1(bg1),
    .blend(blend), .bri(bri), .bri1(bri1), .shadow(shadow),
    .blend_e(blend_e), .shadow_e(shadow_e),
    .shadow_m(shadow_m), .shadow_me(shadow_me),
    .dbg_winner     (dbg_winner),
    .dbg_enable     (dbg_enable)
);

gx_palette u_palette (
    .clk    (clk),
    .pxl_cen(pxl_cen),
    .cs     (pal_cs),
    .addr   (cpu_addr[14:1]),
    .din    (cpu_dout),
    .ds     (cpu_ds),
    .we     (cpu_we),
    .dout   (pal_dout),
    .dout_ok(pal_ok),
    .index0 (idx0),
    .index1 (idx1),
    .rgb0   (rgb0),
    .rgb1   (rgb1)
);

gx_colmix u_colmix (
    .clk      (clk),
    .rst      (rst),
    .pxl_cen  (pxl_cen),
    .reg_cs   (k054338_cs),
    .reg_we   (cpu_we),
    .reg_addr (cpu_addr[4:1]),
    .reg_din  (cpu_dout),
    .reg_ds   (cpu_ds),
    .reg_dout (k338_dout),
    .shd_live (shd_live),
    .bg0(bg0), .bg1(bg1),
    .blend(blend), .bri(bri), .bri1(bri1), .shadow(shadow),
    .blend_e(blend_e), .shadow_e(shadow_e),
    .shadow_m(shadow_m), .shadow_me(shadow_me),
    .rgb0(rgb0), .rgb1(rgb1),
    // Two dots late: where the colour it blanks is.  See "Video output
    // timing" after the CCU.
    .hblank   (mix_hblank),
    .vblank   (mix_vblank),
    .red      (red),
    .green    (green),
    .blue     (blue),
    .dbg_video_en (dbg_video_en)
);

// ---------------------------------------------------------------------------
//  CPU liveness ladder -- the decode half.  The painting half is in the
//  target, which is the right split: this end reads registers that live here,
//  and only the seven result bits travel.
//
//  Ordered the way the game walks it (docs/MEASUREMENTS.md sections 6 and 9),
//  so the first bit still clear is where the CPU stopped:
//
//      0  ran a bus cycle at all
//      1  fetched the program       200000-3FFFFF
//      2  touched work RAM          C00000-C1FFFF
//      3  wrote the K054338         D80000-D8001F   <- 2026-09-08
//      4  wrote the K053252 CCU     D4C000-D4C01F
//      5  wrote the K055555         D50000-D500FF
//      6  wrote the palette         D90000-D97FFF
//
//  MEASURED 2026-09-08: rungs 0-6 of the PREVIOUS assignment were all green,
//  so "does the CPU run" is answered and the ladder has to ask something
//  else.  The BIOS rung is retired (green, and implied by the program rung)
//  and its slot now carries the K054338, because docs/MEASUREMENTS.md 5 has
//  gokuparo writing CONTROL 0x30 -> 0x31 at frame 162 and bit 0 is
//  K338_CTL_KILL, "0 = no video output".  Until that write the screen is
//  black BY DESIGN, and nothing in the old ladder could see it.
//
//  Slot 7 is no longer `dbg_stalled` -- it is `video_en` itself, live, so the
//  two halves of the question are side by side: did the CPU write the chip,
//  and is the chip letting video out.
// ---------------------------------------------------------------------------
wire dbg_bus_active = (dbg_busstate != 2'b01);
wire dbg_bus_write  = (dbg_busstate == 2'b11);

always @(posedge clk) begin
    if (rst) dbg_live <= 7'd0;
    else if (dbg_bus_active) begin
        dbg_live[0] <= 1'b1;
        if (dbg_addr[23:21] == 3'b001)                   dbg_live[1] <= 1'b1;
        if (dbg_addr[23:17] == 7'h60)                    dbg_live[2] <= 1'b1;
        if (dbg_bus_write && dbg_addr[23:5]  == 19'h6C000) dbg_live[3] <= 1'b1;
        //  These three carry the SAME constants as gx_decode.sv's own chip
        //  selects, written in the same form, so a mismatch is visible by eye.
        //  MEASURED 2026-09-08: bit 6 was 9'b110110011 = 9'h1B3, which is
        //  D98000-D9FFFF -- the block ABOVE the palette.  `pal_cs` is 9'h1B2.
        //  So this rung could never light, and a dead rung on a liveness
        //  ladder does not read as "the instrument is broken", it reads as
        //  "the CPU never got here" -- which would have sent the next session
        //  looking for a CPU fault that was not there.
        if (dbg_bus_write && dbg_addr[23:5]  == 19'h6A600) dbg_live[4] <= 1'b1;
        if (dbg_bus_write && dbg_addr[23:8]  == 16'hD500)  dbg_live[5] <= 1'b1;
        if (dbg_bus_write && dbg_addr[23:15] == 9'h1B2)    dbg_live[6] <= 1'b1;
        // BIOS fetch (000000-01FFFF) had its own rung and is retired: it was
        // GREEN on hardware, and rung 1 cannot light without it anyway.
    end
end

// ---------------------------------------------------------------------------
//  WHAT THE EARLIER ASSIGNMENTS SETTLED.  Their rung tables are deliberately
//  NOT repeated here: exactly one table lives in this file at a time, because
//  a second one is how this project produced a confident wrong reading once
//  already, and tools/read_ladder.py reads its names from the live one.
//  The findings survive; the tables do not.
//
//    fourth   layer A's tile code was zero on every frame of three board
//             sessions.  Layer A was innocent -- MAME settled it.  The K055555
//             ENABLE trace does NOT end at 0x01: it goes 0x1F, 0x00, 0x01 at
//             frame 162, 0x00 at 654 and 0x1F at 660 where it STAYS, and 0x01
//             is the SELF-TEST screen, not a POST error screen (root L-018).
//             Every rung that ladder lit is satisfied by frame 162, so nothing
//             it measured required the CPU to execute one instruction past it.
//             And the board is not merely slow: captures at 45, 70, 95, 120
//             and 145 seconds are identical and black where the golden model
//             is through the self-test in 11.
//
//    fifth    the main CPU EXECUTES (a bus cycle completes every frame), is
//             NOT wedged on a bus handshake, writes tile VRAM after ENABLE
//             reaches 0x01, and layer A's tile code is non-zero.  It has
//             never left the self-test.  Stable across two builds.
//
//    sixth    the sound 68000 runs on hardware; see the paragraph below, which
//             is this assignment's starting point rather than history.
// ---------------------------------------------------------------------------
//  HISTORICAL LADDER NOTES THROUGH THE NINETEENTH ASSIGNMENT.
//  SUPERSEDED by the live TWENTIETH assignment immediately above the rung
//  registers below.  The old findings remain useful, but its rung tables must
//  not be used to name a current screenshot.
//
//  THE EIGHTEENTH ANSWERED THE PIXEL QUESTION, five identical captures:
//
//      G R R G G R G R, zero non-black pixels outside the band.
//
//  The ordered chain reached a real non-transparent layer-A pixel, while the
//  live K054338 enable was low at capture time.  More importantly, ENABLE
//  still never reached the golden model's later 0x1F.  The internal mailbox
//  counter is moving, but that does not yet prove the 68EC020 reads the moving
//  value.  This assignment measures the consumer side of that boundary:
//
//      0  SOUND: a bus cycle COMPLETED     this frame   REGRESSION GUARD
//      1  SOUND: a cycle UNACKED >=1024    this frame   REGRESSION GUARD
//      2  ARB:   the SOUND client STARVED  this frame   REGRESSION GUARD
//      3  REPLY: eight internal +1 steps                REGRESSION GUARD
//      4  MAIN:  read D52010 after rung 3                EVER
//      5  MAIN:  read a non-zero reply high nibble       EVER, EXPECT RED
//      6  MAIN:  saw eight changing reply low nibbles    EVER
//      7  MAIN:  ENABLE 0x01 -> later 0x1F               VERDICT
//
//  Rung 4 is exact-address qualified: the K056800 aliases its register number
//  in both halves, but the measured main-CPU poll is specifically D52010.
//  Rung 5 starts only after the internal counter guard is green, so the valid
//  0xC0 self-test phase cannot trip it.  MAME measures the running reply as
//  0x00..0x0F, with the high nibble always zero.  Rung 6 counts changes rather
//  than +1 because the main CPU samples once per frame while the reply advances
//  about 4.24 times per frame; consecutive samples are not expected to differ
//  by one.
//
//  Interpretation: 3 G + 4 R means the main CPU no longer polls the mailbox;
//  4 G + 6 R means the value reaching the CPU is frozen; 5 G means the byte is
//  malformed; 4 G + 5 R + 6 G + 7 R moves the hunt past the mailbox into the
//  main program's next self-test decision.
//
//  ---- the eighteenth assignment, for the record --------------------------
//  THE LADDER, EIGHTEENTH ASSIGNMENT -- 2026-09-12, ninth session
//
//  THE SEVENTEENTH RE-DERIVED THE FIRST HALF, five identical captures:
//
//      G R R G G R G G, zero non-black pixels outside the band.
//
//  The sound guards are healthy, the reply still counts, and the main CPU
//  writes tile VRAM after ENABLE has reached 0x01.  It still never reaches
//  0x1F.  But rung 4 exposed an instrumentation hole: `dbg_ev[2]` was sticky
//  from reset, not qualified by ENABLE 0x01, so its GREEN could have remembered
//  a layer-A code from early init.  It did NOT prove that the self-test screen
//  consumed one.  This assignment closes that temporal hole and follows the
//  active-display path three steps further:
//
//      0  SOUND: a bus cycle COMPLETED     this frame   REGRESSION GUARD
//      1  SOUND: a cycle UNACKED >=1024    this frame   REGRESSION GUARD
//      2  ARB:   the SOUND client STARVED  this frame   REGRESSION GUARD
//      3  VIDEO: layer A code non-zero AFTER 0x01        EVER
//      4  VIDEO: layer A active pixel non-zero AFTER 01  EVER
//      5  VIDEO: layer A won priority AFTER 0x01         EVER
//      6  REPLY: eight +1 INCREMENTS                     REGRESSION GUARD
//      7  MIXER: K054338 video enable                   LIVE CONTROL
//
//  Rungs 0..2 are kept unchanged: 0 GREEN, 1 and 2 RED, or the urgent fix
//  regressed and none of rungs 3..5 describes the machine under test.  Rung 6
//  keeps the sixteenth's exact moving-counter predicate.  Rung 7 is the
//  K054338 KILL output itself, not an inference from an old write; it must be
//  GREEN before the pixel-path chain is interpreted.
//
//  This is an ORDERED chain, not four independent sticky observations:
//  ENABLE 0x01 -> later VRAM write -> later code -> later pixel -> later win.
//  Without the VRAM-write gate, a code read after 0x01 could still be old
//  content written during early init -- the same temporal hole one level
//  later.  Pixel and winner are also restricted to active display because
//  gx_tilemap's shifter holds its last value in blanking.
//  The constituent positive and negative controls are already in
//  tb_gx_tilemap (populated/empty page and non-zero/zero pixel) and tb_gx_prio
//  (ENABLE 0x01 selects A; 0x00 selects backdrop).
//
//  ---- the sixteenth assignment, for the record ----------------------------
//  THE LADDER, SIXTEENTH ASSIGNMENT -- 2026-09-09, eighth session
//
//  THE FIFTEENTH FOUND IT AND ONE LINE FIXED IT.  rung 2 read GREEN -- the
//  sound client asking and not being granted for >=1024 consecutive clocks
//  while the CPU client was not -- and `.urgent({snd_rom_req,3'b000})` flipped
//  rungs 0, 1, 2 and 3 all the right way.  The sound CPU completes bus cycles
//  and touches devices for the first time in eight sessions.
//
//  AND THE SCREEN IS STILL BLACK.  `non-black outside the ladder band` reads
//  0 on all five captures, so the machine is UNSTUCK and not FINISHED, and
//  this assignment exists to keep those two apart.  "The sound CPU touched a
//  device" is not progress -- L-021 is the same sentence about the K054539,
//  where 146 register writes were read as "it got past the loop".
//
//      0  SOUND: a bus cycle COMPLETED     this frame   REGRESSION GUARD
//      1  SOUND: a cycle UNACKED >=1024    this frame   REGRESSION GUARD
//      2  ARB:   the SOUND client STARVED  this frame   REGRESSION GUARD
//      3  SOUND: touched ANY device        this frame
//      4  REPLY: eight +1 INCREMENTS       EVER    <- is it COUNTING
//      5  MAIN:  ENABLE 0x01 -> 0x1F       EVER    <- did the main CPU LEAVE
//      6  MAIN:  pulsed the sound interrupt EVER   CONTROL
//      7  MAIN:  palette write             this frame  CONTROL
//
//  Rungs 0..2 are kept so the fix has to STAY fixed: 0 GREEN, 1 and 2 RED, or
//  nothing above them is about this machine.
//
//  rung 4 counts only +1 steps.  The handshake 0x00->0xC0->0x01 changes the
//  byte too, and MEASUREMENTS 11 is explicit that a machine which answers
//  ONCE looks identical to one that answers CORRECTLY if you only count
//  changes.  Eight is past the handshake and far short of the simulator's 565.
//
//  ---- the fifteenth assignment, for the record -----------------------------
//  THE LADDER, FIFTEENTH ASSIGNMENT -- 2026-09-09, eighth session
//
//  THE FOURTEENTH ANSWERED, and the answer was not on its own list.  Five
//  identical captures of bitstream 547d62c3:
//
//      0 R   1 G   2 R   3 R   4 G   5 R   6 G   7 G
//
//  Nothing COMPLETES (0), a cycle is STUCK (1), it is not HALTED (2), and it
//  is fetching PAGE 0 (4).  Rung 4 green with rung 0 red means the stuck cycle
//  is a ROM *READ* -- the fourteenth's table predicted 4 R for "frozen"
//  because the simulator's freeze was a WRITE, and a write does not set that
//  bit.  The prediction was wrong in the informative direction.
//
//  SO THE SOUND CPU IS FROZEN ON AN UNACKNOWLEDGED ROM READ IN PAGE 0.  Not
//  crashed, not looping, not halted -- and that retro-explains both earlier
//  ladders: the twelfth read "last page fetched = 0" and the thirteenth "last
//  non-zero page = 3", which is exactly what a machine that ran through its
//  main loop and then hung on a page-0 read paints.
//
//  A ROM read only fails to complete if gx_romcache never got its `m_ack`, and
//  gx_memarb is the only thing that hands one out.  gx_memarb picks the LOWEST
//  set request and `urgent` WAS tied to 0 here, so with
//
//      req = { snd_rom_req, farb_req, cpu_rom_req, dl_req }
//
//  the SOUND CPU is client 3, LAST OF FOUR, with no aging and no round-robin.
//  Its wait is unbounded by construction whenever the lower three keep asking.
//  That is a candidate the simulator can never see -- tb_gx_sndboot has no
//  competing clients at all -- and it is what this assignment measures.
//
//      0  SOUND: a bus cycle COMPLETED (acked)   this frame
//      1  SOUND: a cycle UNACKED >= 1024 clocks  this frame
//      2  ARB:   the SOUND client STARVED        this frame  <- THE QUESTION
//      3  SOUND: touched ANY device              this frame
//      4  SOUND: fetched FROM page 0             this frame
//      5  ARB:   the MAIN CPU client STARVED     this frame
//      6  MAIN:  pulsed the sound interrupt      EVER        CONTROL
//      7  MAIN:  palette write                   this frame  CONTROL
//
//  rung 2 GREEN -> starvation, and rung 5 says whether the tile fetcher is
//  jamming everyone or the priority order is starving only the sound CPU.
//  rung 2 RED   -> the client is not even asking, and gx_romcache's own FSM
//  is where to look next.
//
//  ---- the fourteenth assignment, for the record -----------------------------
//  THE LADDER, FOURTEENTH ASSIGNMENT -- 2026-09-09, eighth session
//
//  THE THIRTEENTH ANSWERED ITS QUESTION AND THE ANSWER WAS A DEAD END.  Five
//  captures, all identical: the last 2 KB page other than 0 is page 3.  So the
//  crash site is 0x1800..0x1FFF -- and tools/gx_snddasm.py then disassembled
//  that page and found that NOTHING IN IT CAN FAULT.  No DIVU, no CHK, no
//  TRAPV, no TRAP, no line-A or line-F on the instruction stream; the only
//  fault-capable instructions are two RTEs, and RTE faults only in user mode,
//  which this program never enters.  Worse for the reading: page 3 is the
//  program's MAIN LOOP (0x1972, where the reset path branches), so "it rested
//  at page 3" says only "it was where it always is".  tb_gx_sndboot's control
//  run ends healthy with that same latch reading 3.
//
//  AND THE PREMISE UNDERNEATH IT WAS NEVER TESTED.  Every assignment since the
//  sixth has read rung 5, "SOUND: a bus cycle THIS FRAME", as "the sound CPU
//  is running".  It does not mean that.  It is a per-frame OR of a LEVEL, and
//  a 68000 frozen on an unacknowledged cycle holds that level asserted
//  forever -- so a frozen CPU and a looping CPU paint the SAME square.  That
//  is L-018 rule 5 for the third time on this board.
//
//  There are THREE ways for this machine to stop and the ladder could
//  distinguish none of them:
//
//      looping   executing the vector table at 0, fetching, acking, forever
//      frozen    one bus cycle asserted and never acknowledged
//      halted    fx68k's double-bus-fault halt, AS left asserted
//
//  MEASURED in simulation (+corrupt=2000, a wrong fetched word once in every
//  2000 SDRAM reads): the machine goes FROZEN, not looping and not halted --
//  and the cycle it froze on was a WRITE INTO THE ROM REGION, which
//  gx_sound.sv acknowledged with the cache's read-only c_ack and therefore
//  never acknowledged at all.  That defect is fixed in the same commit as this
//  assignment; this ladder is how the board says whether it was the one.
//
//      0  SOUND: a bus cycle COMPLETED (acked)   this frame  <- the discriminator
//      1  SOUND: a cycle UNACKED >= 1024 clocks  this frame
//      2  SOUND: the CPU HALTED                  EVER
//      3  SOUND: touched ANY device              this frame
//      4  SOUND: fetched FROM page 0             this frame
//      5  SOUND: fetched OUTSIDE page 0          this frame
//      6  MAIN:  pulsed the sound interrupt      EVER        CONTROL
//      7  MAIN:  palette write                   this frame  CONTROL
//
//  HOW TO READ IT.  Rungs 6 and 7 are the controls and must be GREEN; if they
//  are not, nothing below them means anything.
//
//      healthy          0 G  1 R  2 R  3 G  4 ?  5 G
//      looping at 0     0 G  1 R  2 R  3 R  4 G  5 R
//      frozen           0 R  1 G  2 R  3 R  4 R  5 R
//      halted           0 R  1 G  2 G  3 R  4 R  5 R
//      STOPped          0 R  1 R  2 R  3 R  4 R  5 R
//
//  The last one comes free, as the combination none of the others can make,
//  and it is a real destination: 0x18E8 is `MOVE.W #$2700,SR` followed by
//  `STOP #$2700`, this program's own die-here routine.  A STOPped 68000 runs
//  no bus cycles at all, so it is not frozen, not halted and not looping --
//  every sound rung dark.  MEASURED: with the ROM-write acknowledge fixed,
//  the +corrupt=2000 run ends exactly there, `cyc` frozen and `unacked 0`.
//
//  Rung 0 is the one that was missing.  "A cycle happened" cannot separate a
//  running CPU from a stuck one; "a cycle COMPLETED" can, and it is one AND
//  gate away in gx_sound (dbg[16] = cyc && ack).
//
//  ---- the thirteenth assignment, for the record ----------------------------
//  THE LADDER, THIRTEENTH ASSIGNMENT -- 2026-09-09, seventh session
//
//  THE TWELFTH ASSIGNMENT FOUND THE CRASH.  Four captures:
//
//      0 R   1 R   2 R   3 R   4 G   5 G   6 R   7 G
//
//  Page 0.  The sound CPU is looping in bytes 0x0000..0x07FF of its own
//  program, touching no device, while IRQ2 sits asserted.
//
//  AND THE SOUND ROM SAYS WHY.  Its vector table, read straight out of
//  dist/gokuparo.bin at GX_SND_BASE:
//
//      vec 0-1   SSP 0x00103E00   PC 0x00001400
//      vec 2     bus error        0x000018F0
//      vec 3     address error    0x000018F0
//      vec 4-23  0x00000000       <- ALL ZERO
//
//  Illegal instruction, divide-by-zero, CHK, TRAPV, privilege violation,
//  trace, line-A and line-F ALL VECTOR TO ADDRESS ZERO.  A 68000 that takes
//  one of them jumps to 0x000000 and starts EXECUTING ITS OWN VECTOR TABLE --
//  which is in page 0, touches no device, and never returns.  That is exactly
//  the picture, and it means the sound CPU CRASHED.
//
//  It also explains the unserviced IRQ2: whatever the CPU is executing in
//  there is data, and data that lands on the SR does what it likes to the
//  interrupt mask.
//
//  SO THE QUESTION IS NOW: WHERE WAS IT WHEN IT CRASHED?
//
//  Once it is in page 0 it never leaves, so the last page it fetched from
//  that was NOT page 0 is the crash site.  That freezes itself, with no edge
//  detection and nothing to arm -- during normal running it simply follows
//  the CPU, which is what makes it readable in simulation as a control.
//
//      0..3  the last 2 KB page OTHER THAN 0 that was fetched from
//      4     MAIN: pulsed the sound interrupt   EVER
//      5     SOUND: a bus cycle                 this frame   live control
//      6     SOUND: touched ANY device          this frame   the crash control
//      7     the main CPU wrote the palette                  CONTROL
//
//  HOW TO READ IT: page = 8*r3 + 4*r2 + 2*r1 + r0, and the code that crashed
//  is the 2 KB at byte 0x800 * page.  Disassemble it out of dist/sndrom.hex.
//  With rung 6 RED the machine is still in the crash this was built for.
//
//  PREDICTION, in public: page 3 or 4.  Page 3 holds the IRQ handlers
//  (0x18F0, 0x193C) and page 4 the counter code at 0x24B4, and the reply
//  froze at a counter value -- so it was probably in or just after the
//  handler that advances it.
//
//  ---- the twelfth assignment, for the record --------------------------------
//
//  THE ELEVENTH ASSIGNMENT WAS PREDICTED RIGHT AND IT CLOSED THE LAST INPUT:
//
//      0 R   1 G   2 G   3 G   4 G   5 G   6 R   7 G
//
//  low nibble 0xE -- the main CPU sends 0xFE, the same command MAME sends at
//  frame 163 -- and rung 4 says it pulsed the sound interrupt too.  The SAME
//  bitstream then read byte-identically with releases/gokuparo_flat.mra, which
//  has no <interleave> and no map= at all, so the ROM image and the .mra digit
//  order are cleared as well, in a session that cost no build.
//
//  So the sound CPU gets the same program and the same input as the simulator,
//  and the simulator never fails.  Everything else that could differ has been
//  measured and killed: SDRAM latency at 9, 40, 80 and 200 clocks per word;
//  both host commands; the command's TIMING (0xFE arriving late makes the
//  sound CPU re-run its entire self-test, and it completes -- even starved);
//  and two billion clocks of runtime.
//
//  WHAT THE BOARD ACTUALLY LOOKS LIKE, from the ninth and tenth readings:
//
//      IRQ2 asserted and NEVER ACKNOWLEDGED
//      no K054539, no TMS57002, no work RAM write, no SDRAM fetch, for frames
//      bus cycles still running
//      the reply frozen at a COUNTER value (0x08, 0x18, 0x28 or 0x38)
//
//  That is not a device wait.  A 68000 that is executing, ignoring an asserted
//  interrupt, and touching nothing has TAKEN AN EXCEPTION and landed in a halt
//  handler -- the mask is up because the exception raised it.
//
//  THE FIRST VERSION OF THIS ASSIGNMENT WATCHED FOR THE EXCEPTION VECTOR and
//  it was WRONG.  The sound program checksums its own ROM from 0x000000
//  upward, so it reads the vector table AS DATA, and the rung fired 54 million
//  times in a 60-million-clock simulation where the CPU never crashes.  A
//  vector fetch and a data read of the same address are indistinguishable from
//  inside gx_sound.  It was caught by a NEGATIVE CONTROL -- running the
//  instrument where the answer must be zero -- and not by inspection.
//
//  So this asks something that cannot misfire instead: WHERE IS THE LOOP.
//
//      0..3  WHICH 2 KB PAGE of the sound program it last fetched from, live
//      4  MAIN: pulsed the sound interrupt       EVER
//      5  SOUND: a bus cycle                     this frame   live control
//      6  SOUND: touched ANY device              this frame   the halt control
//      7  the main CPU wrote the palette                      CONTROL
//
//  HOW TO READ IT.  Rungs 0..3 are a NUMBER, not a chain:
//
//      page = 8*rung3 + 4*rung2 + 2*rung1 + rung0
//      the loop lives at byte 0x800 * page in the sound program
//
//  With rung 6 RED (no device access) and rung 5 GREEN (still executing),
//  whatever is at that address IS the loop, and dist/sndrom.hex is the ROM to
//  disassemble.  That turns "why did it stop" into "read the code".
//
//  IT CANNOT MISFIRE.  Every instruction fetch updates it, so it always holds
//  a real address the CPU used, and a tight loop pins it to one value.  The
//  only ambiguity is a loop straddling a 2 KB boundary, which would show two
//  values across captures -- itself informative.
//
//  PREDICTION, in public: one stable page, and NOT page 0 -- the reset code
//  and the vector table live there and the machine is long past both.
//
//  ---- the eleventh assignment, for the record -------------------------------
//  THE LADDER, ELEVENTH ASSIGNMENT -- 2026-09-09, seventh session
//
//  THE TENTH ASSIGNMENT ANSWERED ITS QUESTION AND KILLED ITS OWN PREDICTION:
//
//      0 R   1 R   2 R   3 R   4 R   5 G   6 G   7 G
//
//  In a whole frame the sound CPU touches NO K054539 (read or write), NO
//  TMS57002, NO work RAM write and NO SDRAM fetch -- and still runs bus
//  cycles.  The prediction was rung 0 GREEN, a poll on a K054539 register.
//  Wrong, and the truth is narrower: it is a TIGHT LOOP ENTIRELY INSIDE
//  gx_romcache with no data access at all.  That is the fingerprint of a
//  `bra *`, an error trap or a deliberate halt -- not of waiting on a device.
//
//  Put beside the ninth reading, IRQ2 IS ARRIVING AND IS NOT BEING SERVICED,
//  which for a 68000 means the interrupt mask is up: it is inside something it
//  never leaves.
//
//  THE SIMULATOR NEVER DOES THIS.  Two billion clocks, 4,480 counter
//  increments, no halt.  Same RTL, same 256 KB program.  And the inputs are
//  now enumerated down to one: ROM (verified by the vector table and by the
//  program running), the K054539 (echoes in both), the TMS57002 status (0x0001
//  in both) -- and the K056800 HOST SIDE, which on the board is written by a
//  main CPU whose behaviour this project cannot simulate at all.
//
//  Rung 6 of the tenth assignment says the main CPU DID write it.  So the
//  question is finally small enough to be answered by four squares:
//
//      WHAT DID IT SEND?
//
//      0..3  host_to_snd[0], the command byte, LOW NIBBLE, live
//      4     the main CPU wrote host register 7, the SOUND INTERRUPT, EVER
//      5     the sound CPU ran a bus cycle this frame          live control
//      6     the sound CPU touched ANY device this frame       the halt test
//      7     the main CPU wrote the palette                    CONTROL
//
//  WHY THE LOW NIBBLE IS THE RIGHT FOUR BITS.  MEASURED, dist/sndcomm_mame.csv:
//  the only commands MAME's main CPU ever sends are 0xFE at frame 163 and 0xFB
//  at frame 660.  Their low nibbles are 0xE and 0xB, and the reset value is
//  0x0 -- three states, three distinct pictures:
//
//      R R R R   0x_0   nothing has been sent, or a command ending in 0
//      R G G G   0x_E   the boot command 0xFE          MAME sends this at f163
//      G G R G   0x_B   the second command 0xFB        MAME sends this at f660
//
//  Anything else is a command MAME never sends, and THAT would be the finding:
//  the simulator can then be given the same byte and the halt reproduced in a
//  minute instead of a build.
//
//  RUNG 4 MATTERS SEPARATELY FROM THE COMMAND.  k056800.cpp drops the
//  interrupt pulse when the sound CPU has interrupts disabled, so "the host
//  pulsed" and "the sound CPU was interrupted" are different facts.  If the
//  command byte is right and rung 4 is RED, the sound CPU was never told.
//
//  RUNG 6 IS THE TENTH ASSIGNMENT COMPRESSED TO ONE SQUARE, kept as a control
//  on the halt itself: it must stay RED for the reading above to still hold.
//  If it goes GREEN the machine is in a different state than the one this
//  ladder was built for, and the command bits are being read out of a
//  situation they were not chosen for.
//
//  WHEN.  Rungs 0-3 follow a register that is ZERO out of reset and can only
//  change when the main CPU writes -- they cannot lie early.  Rung 4 is sticky
//  from the first pulse, which MAME puts at frame 163.  Rung 5 lights in
//  microseconds and rung 7 at frame 4; both are controls.
//
//  PREDICTION, in public: 0x_E -- the boot command, correctly delivered, with
//  rung 4 GREEN.  Two predictions in a row have been wrong on this board and
//  both times the wrong one was the interesting one, so this is offered
//  cheaply.
//
//  ---- the tenth assignment, for the record ----------------------------------
//
//  THE NINTH ASSIGNMENT'S PREDICTION WAS WRONG, AND USEFULLY SO:
//
//      0 G   1 G   2 R   3 G   4 R   5 G   6 R   7 G
//
//  The gate is OPEN (0 G) and IRQ2 IS ARRIVING (1 G).  The prediction was
//  rung 0 RED -- a latched gate that never re-opens -- and the board says the
//  opposite.  So the interrupt chain is intact and the fault is INSIDE the
//  handler: it is entered and it does not reach the end, because rung 2 says
//  the mailbox is never written while rung 5 says the CPU is executing.
//
//  A handler that is entered, runs, and never finishes is spinning on a read.
//  THAT is a question about WHICH WINDOW, and it is exactly the question
//  sim/tb_gx_sndboot.sv answered in one run for the TMS57002 -- by counting
//  accesses per region and seeing every counter frozen except one.  This
//  ladder is that table, on hardware, one square per region.
//
//      0  SOUND: a K054539 READ        this frame
//      1  SOUND: a K054539 WRITE       this frame
//      2  SOUND: a TMS57002 status read this frame
//      3  SOUND: a work RAM write      this frame
//      4  SOUND: an SDRAM ROM fetch    this frame
//      5  SOUND: a bus cycle           this frame     live control
//      6  MAIN:  wrote the K056800     EVER           what the sim cannot see
//      7  the main CPU wrote the palette              CONTROL
//
//  HOW TO READ IT.  A spinning loop lights very few squares, and WHICH ones
//  name the loop:
//
//      0 G, others mostly R    it is polling a K054539 register.  gx_k054539
//                              echoes, so it is waiting for a value only the
//                              PCM ENGINE could change -- REUSE_PLAN item 4.
//      2 G, others mostly R    it is back in the TMS57002 status loop, and
//                              0x0001 is wrong for whatever it is doing now.
//      3 G and 4 G             it is running real code out of ROM, touching
//                              work RAM -- not spinning on a device at all,
//                              and the fault is a program state we do not
//                              understand yet.
//      4 R                     it is running entirely out of gx_romcache, so
//                              the loop is small and tight.
//
//  RUNG 6 IS STICKY ON PURPOSE.  The ninth assignment asked it per frame and
//  got RED, which only says the main CPU was quiet during those four frames.
//  Whether it EVER sent a command is the question, because that command is the
//  one input to the sound CPU that tb_gx_sndboot has to guess.
//
//  WHEN.  Rungs 0-5 are all per-frame and can light at any time.  Rung 6 is
//  sticky from the first host write, which MAME puts at frame 4.  Rung 7
//  lights at frame 4.  NONE of these requires the boot to proceed -- this
//  assignment has no verdict rung, deliberately: the verdict is known (RED)
//  and the question is why.
//
//  PREDICTION, in public: rung 0 GREEN and rungs 3 and 4 RED -- a tight poll
//  on a K054539 register, out of cache, waiting for the PCM engine.  That is
//  the only device in the sound map whose reads this core answers with
//  something that can never change.
//
//  ---- the ninth assignment, for the record ----------------------------------
//
//  THE EIGHTH ASSIGNMENT'S PREDICTION WAS RIGHT AND IT SPLIT THE QUESTION:
//
//      0 G   1 R   2 R   3 G   4 R   5 G   6 R   7 G
//
//  rung 3 GREEN -- the reply DID count, at least eight +1 steps -- and rung 4
//  RED, so it is not counting now.  IT RAN AND STOPPED.  Rung 2 RED says the
//  sound CPU is no longer writing the mailbox at all, while rung 5 GREEN says
//  it is still executing.  Rungs 0 and 1 put the stuck byte at 0x08, 0x18,
//  0x28 or 0x38, and 0x08 is a value the counter passes through.
//
//  THE SIMULATOR DOES NOT DO THIS.  sim/tb_gx_sndboot.sv, same RTL, same
//  256 KB program, counts for 1.68 BILLION clocks -- 17.5 emulated seconds,
//  3,963 changes -- and never stops.  Three explanations were tested and
//  killed:
//
//    * SDRAM starvation: 9, 40 and 80 clocks per word, byte-identical.
//    * the host's first command: with +nocmd the sim reaches the counter at
//      exactly the same clock.  The sound CPU self-drives.
//    * the host's SECOND command: MAME sends 0xFB and 360 volume writes at
//      frame 660.  Replayed with +cmd2, the counter carries straight on --
//      239, 341, 443, 648 changes at 220, 260, 300 and 380 million clocks.
//      (This one was briefly recorded as CONFIRMED because the transition log
//      caps at 24 lines and looked silent.  The progress marks are the
//      readout; the transition list is a sample.)
//
//  So the difference is between the timer and the handler, and every wire in
//  that chain is inside gx_sound where no rung has ever looked.  Two new debug
//  levels come out of it and this ladder spends four squares on them.
//
//      0  sound_ctrl bit 0 -- the IRQ2 GATE      live
//      1  IRQ2 was asserted at any point this frame
//      2  SOUND: wrote the mailbox this frame
//      3  the reply has INCREMENTED BY ONE >= 8 times, EVER   (kept)
//      4  the reply CHANGED in the last ~62 frames            (kept)
//      5  the sound CPU ran a bus cycle this frame            live control
//      6  MAIN: wrote the K056800 host side this frame
//      7  the main CPU wrote the palette                      CONTROL
//
//  HOW IT SPLITS.  Rungs 0 and 1 are the whole interrupt chain:
//
//      0 R          the GATE IS CLOSED.  The sound program acknowledges IRQ2
//                   by writing sound_ctrl with bit 0 clear and re-opens it by
//                   writing it set.  A gate that never re-opens is a lost
//                   write or a program that stopped acknowledging.
//      0 G, 1 R     the gate is open and NO INTERRUPT ARRIVES.  That is the
//                   K054539 timer: register 0x227, the accumulator, or the
//                   rising-edge detect.
//      0 G, 1 G     interrupts are arriving and the handler is not advancing
//                   the reply -- which, with rung 2 RED, means the handler is
//                   not the code that writes the mailbox any more.
//
//  RUNG 6 IS NEW AND IT IS NOT A CONTROL.  If the main CPU is writing the
//  K056800 every frame it is reacting to something, and what it sends is the
//  one input to the sound CPU this testbench cannot guess.
//
//  WHEN.  Rung 0 follows a register that is ZERO out of reset, so it reads RED
//  until the sound program first opens the gate -- it cannot lie early.  Rung
//  1 needs a real interrupt.  Rung 3 needs eight +1 steps and the handshake
//  cannot fake them.  Rungs 5 and 7 are controls and light in microseconds and
//  at frame 4 respectively.
//
//  PREDICTION, in public: rung 0 RED -- the gate is closed and never re-opens.
//  It is the only part of the chain that is a LATCHED STATE rather than a
//  repeating event, and a repeating event that stops permanently usually has a
//  latch behind it.
//
//  ---- the eighth assignment, for the record ---------------------------------
//
//  THE SEVENTH ASSIGNMENT'S PREDICTION WAS RIGHT, TWICE, AND THEN THE BOARD
//  AND THE SIMULATOR DISAGREED.  That disagreement is what this one is for.
//
//  Reading 1, bitstream ba82928 -> f72d5613 (K054539, no TMS fix):
//
//      0 R   1 R   2 R   3 G   4 R   5 G   6 R   7 G
//
//  reply 0xC0, frozen, sound CPU alive.  Exactly the prediction written into
//  this file before that build, and exactly what sim/tb_gx_sndboot.sv shows
//  with the TMS57002 status word answering zero.  Board and simulator agreed.
//
//  Reading 2, bitstream cf7a6815 (+ the TMS57002 status fix):
//
//      0 R   1 R   2 R   3 R   4 R   5 G   6 R   7 G
//
//  The reply is no longer 0xC0 -- bits 6, 2, 1 and 0 are ALL clear -- it is
//  still frozen, and the sound CPU is still alive.  THE SIMULATOR, RUNNING THE
//  SAME RTL AND THE SAME 256 KB PROGRAM, COUNTS: 0xC0 at clock 971, 0x01 at
//  6,229,307, then +1 every 392,400 clocks for as long as it is run.
//
//  So one of two things is true, and four squares of value bits cannot tell
//  them apart:
//
//      the counter NEVER RAN on hardware     -> the fix did not take, and the
//                                               difference is in the board:
//                                               memory, reset, arbitration
//      the counter RAN AND STOPPED           -> something later kills it, and
//                                               the sim does not model that
//                                               something (the main CPU is the
//                                               obvious candidate -- it is not
//                                               in the testbench at all)
//
//  ELIMINATED ALREADY, so this ladder does not spend a rung on any of them:
//
//    * SDRAM starvation.  D10 makes the sound CPU arbiter client 3 of 4, so a
//      fetch can be very late.  MEASURED in sim at 9, 40 and 80 clocks per
//      word: byte-identical behaviour, because gx_romcache absorbs it -- the
//      whole run takes 3,129 SDRAM fetches.
//    * The host command.  The sound CPU does NOT need it: with +nocmd the sim
//      reaches 0xC0 and then the counter at exactly the same clocks.  An
//      earlier reading of the log had attributed the 0xC0 -> 0x01 step to the
//      command arriving 557,000 clocks earlier; it does not.
//    * The sound ROM image.  A byte-swapped 68000 program does not reach its
//      own first mailbox write, and both readings show 0xC0 or later.
//
//  ---- THE RUNGS, AND WHEN EACH CAN FIRST LIGHT (L-018 rule 5) ---------------
//
//      0  reply bit 3    live, latched at vblank   narrows the stuck value
//      1  reply bit 7    live, latched at vblank   an error code would show
//      2  SOUND: wrote the mailbox THIS frame      is it still answering NOW
//      3  the reply has INCREMENTED BY ONE at     <- DID THE COUNTER EVER RUN
//         least 8 times, EVER
//      4  the reply CHANGED in the last ~62 frames <- IS IT COUNTING NOW
//      5  the sound CPU ran a bus cycle THIS frame <- live control
//      6  MAIN: ENABLE ever 0x1F after being 0x01  <- the verdict
//      7  the main CPU wrote the palette           <- CONTROL
//
//  RUNGS 3 AND 4 ARE THE POINT and they split three ways:
//
//      3 R, 4 R    the counter NEVER RAN.  The TMS fix did not take on
//                  hardware and the divergence is below the program.
//      3 G, 4 R    it RAN AND STOPPED.  The sim runs it for 6 emulated
//                  seconds without stopping, so whatever stops it is not in
//                  the testbench -- and the main CPU is the thing the
//                  testbench does not have.
//      3 G, 4 G    it is counting now, and rung 6 says whether that was
//                  enough.
//
//  WHEN.  Rung 3 needs EIGHT increments, so it cannot be lit by the handshake:
//  0x00 -> 0xC0 is not +1, and 0xC0 -> 0x01 is not +1.  Only the free-running
//  counter steps by one, and eight of them is 33 ms at the measured rate.
//  Rungs 0 and 1 read a register that is zero out of reset, so they cannot lie
//  early.  Rung 2 can first light when the sound CPU answers, microseconds in.
//  Rung 5 lights microseconds after snd_run.  Rung 7 lights at frame 4.
//  ONLY RUNG 6 REQUIRES THE BOOT TO PROCEED.
//
//  DECODE.  Rungs 0 and 1 are bits 3 and 7.  Reading 2 established that bits
//  6, 2, 1 and 0 are all clear, so together they narrow the stuck byte to:
//
//      0 R, 1 R   ->  0x00, 0x10, 0x20 or 0x30
//      0 G, 1 R   ->  0x08, 0x18, 0x28 or 0x38
//      0 R, 1 G   ->  0x80, 0x90, 0xA0 or 0xB0
//      0 G, 1 G   ->  0x88, 0x98, 0xA8 or 0xB8
//
//  PREDICTION, written before the build so it can be wrong in public: rung 3
//  GREEN and rung 4 RED -- the counter ran and stopped.  The reasoning is that
//  the simulator and the board agreed perfectly until the program was allowed
//  to run further, and the only thing the board has that the testbench does
//  not is a main CPU writing to the other side of the K056800.
//
//  TIMING.  All eight rungs are FLIP-FLOPS here.  The only new arithmetic is
//  an 8-bit increment-compare and a 4-bit saturating counter, both between
//  registers.
//
//  ---- THE LIVE LADDER, THIRTY-SECOND ASSIGNMENT -- 2026-09-14 --------------
//
//  The thirty-first asked where the main CPU's POST stopped.  It does not stop
//  any more: the attract runs to the title and the sprites are on the board
//  (STATUS 2026-09-14, night).  The open question is BUS TIME, and until this
//  assignment the board could answer it only with a picture -- a busy curtain
//  missing the right-hand end of its gold frame.  gx_sprite and gx_tilemap
//  have both raised a `dbg_late` since they were written, and nothing read
//  either (DECISIONS D12, "What would make it wrong").  These rungs read them,
//  once a frame, so a capture says "late" directly and says how much.
//
//      0  SOUND:  a bus cycle COMPLETED              frame  guard  expect G
//      1  ARB:    the SOUND client STARVED           held   guard  expect R
//      2  ARB:    the MAIN CPU client STARVED        held          expect R
//      3  TILE:   a group edge arrived mid-fetch     held   guard  expect R
//      4  SPRITE: >=   1 line ran out of time        held
//      5  SPRITE: >=   8 lines                       held
//      6  SPRITE: >=  32 lines                       held
//      7  DMA:    a frame with no copy (lag frame)   held   lit through boot
//
//  4..6 are a thermometer: they can only read as a run of greens from 4
//  upward, and a green above a red is an instrument fault.
//
//  HELD, since 2026-09-14 (the user saw brief flicker the captures never
//  caught): rungs 1..7 stay lit for RUNG_HOLD frames, about two seconds, after
//  any frame that lit them.  A flicker is one frame and captures are a second
//  apart, so a rung that showed only the last frame was almost never
//  photographed lit.  Rung 0 stays per-frame: it is a guard that must be lit
//  every frame and a hold would hide it going out.
//
//  Rung 7 was ">= 128 late lines" and is now "no DMA copy since the last
//  vblank": gx_sprite skips the copy when DMAEN is clear at vblank begin,
//  which after frame 660 happens only in lag frames (MEASUREMENTS 27).  It
//  says whether the board had one around a flicker.
//
//  Rung 2 is not a fault signal on its own.  A policy that holds the CPU off
//  for a busy sprite line can make it wait 1024 clocks by design -- b56f8c0's
//  did -- and the rung is here so a bus change that starves the CPU shows in
//  the same capture that shows the sprites it bought.
//
//  Rung 3 is gx_tilemap's own predicate: at a group edge whose pixels will be
//  displayed, the FSM is still busy with the group before.  It must stay RED
//  under any sprite policy -- the tile deadline is the hard one.
//
//  TIMING.  Flip-flops only: a saturating 8-bit count, four compares and four
//  sticky bits, all latched at vblank.  `tm_late` and `spr_late` are declared
//  beside u_tilemap, which is the first place either is named.

reg         snd_ackd_seen,  snd_ackd_frame;
reg         snd_starv_seen, snd_starv_frame;
reg         cpu_starv_seen, cpu_starv_frame;
reg         tm_late_seen,   tm_late_frame;
reg  [7:0]  spr_late_cnt;
reg  [2:0]  spr_late_frame;
reg         dma_seen,       dma_skip_frame;
reg         vb_d, vb_d2;

localparam [6:0] RUNG_HOLD = 7'd120;           // frames, ~2 s at 59.19 Hz
reg  [6:0]  rung_hold [1:7];
integer     rk;

// 7 DMA skipped, 6..4 SPRITE >=32 / >=8 / >=1, 3 TILE, 2 CPU starved, 1 SOUND starved
wire [7:1]  rung_frame = { dma_skip_frame, spr_late_frame, tm_late_frame,
                           cpu_starv_frame, snd_starv_frame };

always @(posedge clk) begin
    if (rst) begin
        snd_ackd_seen  <= 1'b0;  snd_ackd_frame  <= 1'b0;
        snd_starv_seen <= 1'b0;  snd_starv_frame <= 1'b0;
        cpu_starv_seen <= 1'b0;  cpu_starv_frame <= 1'b0;
        tm_late_seen   <= 1'b0;  tm_late_frame   <= 1'b0;
        spr_late_cnt   <= 8'd0;  spr_late_frame  <= 3'd0;
        dma_seen       <= 1'b0;  dma_skip_frame  <= 1'b0;
        vb_d           <= 1'b0;  vb_d2           <= 1'b0;
        for (rk = 1; rk < 8; rk = rk + 1) rung_hold[rk] <= 7'd0;
    end else begin
        vb_d  <= vblank;
        vb_d2 <= vb_d;

        if (snd_dbg[16])    snd_ackd_seen  <= 1'b1;   // sound cycle COMPLETED
        if (arb_starved[3]) snd_starv_seen <= 1'b1;
        if (arb_starved[1]) cpu_starv_seen <= 1'b1;
        if (tm_late)        tm_late_seen   <= 1'b1;
        if (spr_dma_done)   dma_seen       <= 1'b1;
        // gx_sprite raises dbg_late for one clock when a line goes on display
        // with its scan or draw still running -- at most once a line.
        if (spr_late && spr_late_cnt != 8'hff)
            spr_late_cnt <= spr_late_cnt + 8'd1;

        // Last, so the frame boundary's clears win over the sets above.
        if (vblank && !vb_d) begin
            snd_ackd_frame  <= snd_ackd_seen;   snd_ackd_seen  <= 1'b0;
            snd_starv_frame <= snd_starv_seen;  snd_starv_seen <= 1'b0;
            cpu_starv_frame <= cpu_starv_seen;  cpu_starv_seen <= 1'b0;
            tm_late_frame   <= tm_late_seen;    tm_late_seen   <= 1'b0;
            spr_late_frame  <= { spr_late_cnt >= 8'd32, spr_late_cnt >= 8'd8,
                                 spr_late_cnt != 8'd0 };
            spr_late_cnt    <= 8'd0;
            // The copy of THIS vblank finishes a line after this edge, so
            // dma_seen here covers the vblank before it.
            dma_skip_frame  <= !dma_seen;       dma_seen       <= 1'b0;
        end

        // one clock after the latch: reload or count down each rung's hold
        if (vb_d && !vb_d2)
            for (rk = 1; rk < 8; rk = rk + 1)
                rung_hold[rk] <= rung_frame[rk]          ? RUNG_HOLD :
                                 (rung_hold[rk] != 7'd0) ? rung_hold[rk] - 7'd1 : 7'd0;
    end
end

wire [7:1]  rung_held = { rung_hold[7] != 7'd0, rung_hold[6] != 7'd0, rung_hold[5] != 7'd0,
                          rung_hold[4] != 7'd0, rung_hold[3] != 7'd0, rung_hold[2] != 7'd0,
                          rung_hold[1] != 7'd0 };

assign dbg_rung = { rung_frame | rung_held,   // 7..1, held ~2 s
                    snd_ackd_frame };         // 0 SOUND: bus cycle COMPLETED, per frame

// ---- THE PC SAMPLE, 2026-09-28 -----------------------------------------------
//  Where is the main CPU?  tbyahhoo passes its RAM CHECK on the board and then
//  goes black for good, and nothing on this board could say where the program
//  was when it stopped: the ladder says what the BUS did, never which code.
//  MAME ruled out four candidates (MEASUREMENTS 148); one address tells the
//  rest apart.
//
//  The first opcode fetch (busstate 00) after PC_SMP_AT clocks past vblank
//  begin -- about mid-frame, away from the vblank IRQ's entry -- is latched
//  and held for the frame.  A program spinning in a polling loop shows an
//  address inside that loop in every screenshot.
//
//  Sampled HERE, beside the register it reads: the target decoding dbg_addr
//  cost -3.913 ns once (this file, "observability").  Only the registered
//  sample crosses the boundary.
localparam [20:0] PC_SMP_AT = 21'd800_000;   // ~half of 1,622,013 clocks a frame
reg  [20:0] pc_cnt;
reg         pc_arm, pc_vb_d;

always @(posedge clk) begin
    if (rst) begin
        pc_cnt  <= 21'd0;
        pc_arm  <= 1'b0;
        pc_vb_d <= 1'b0;
        dbg_pc  <= 23'd0;
    end else begin
        pc_vb_d <= vblank;
        if (vblank && !pc_vb_d) begin
            pc_cnt <= 21'd0;
        end else begin
            // wraps rather than saturating: with vblank gone the sample still
            // refreshes every 2^21 clocks (21.8 ms) instead of freezing
            pc_cnt <= pc_cnt + 21'd1;
            if (pc_cnt == PC_SMP_AT) pc_arm <= 1'b1;
        end
        if (pc_arm && dbg_busstate == 2'b00) begin
            dbg_pc <= dbg_addr;
            pc_arm <= 1'b0;
        end
    end
end

// ---- THE AUDIO BAND, 2026-09-15 ----------------------------------------------
//  A second row of eight squares under the ladder (KonamiGX.sv rows 28-43), so
//  a screenshot says what the K054539 engines put out on the BOARD.  Nobody on
//  this bench can listen, and rung 1 (the model comparison, MEASUREMENTS 39)
//  says nothing about the board's fetch under real bus traffic.
//
//      0  AUDIO: a non-zero output sample this frame      frame
//      1  AUDIO: |sample| >=   256                        held
//      2  AUDIO: |sample| >=  1024                        held
//      3  AUDIO: |sample| >=  4096                        held
//      4  AUDIO: |sample| >= 16384                        held
//      5  AUDIO: the mix clipped at 16 bits                held   expect R
//      6  PCM:   an engine missed its sample deadline      held   expect R
//      7  PCM:   an engine waited for a sample-ROM fetch   held
//
//  1..4 are a thermometer of the loudest sample on either side in the last
//  ~2 s (RUNG_HOLD), the same span the model's peaks are quoted over.
reg        au_nz_seen, au_nz_frame;
reg  [7:1] au_seen;
reg  [6:0] au_hold [1:7];
reg        au_vb_d;
integer    ak;

wire au_nz = (audio_l != 16'sd0) || (audio_r != 16'sd0);
wire [7:1] au_now = {
    snd_audio_ev[2],
    snd_audio_ev[0],
    snd_audio_ev[1],
    (audio_l >= 16'sd16384) || (audio_l <= -16'sd16384) || (audio_r >= 16'sd16384) || (audio_r <= -16'sd16384),
    (audio_l >= 16'sd4096)  || (audio_l <= -16'sd4096)  || (audio_r >= 16'sd4096)  || (audio_r <= -16'sd4096),
    (audio_l >= 16'sd1024)  || (audio_l <= -16'sd1024)  || (audio_r >= 16'sd1024)  || (audio_r <= -16'sd1024),
    (audio_l >= 16'sd256)   || (audio_l <= -16'sd256)   || (audio_r >= 16'sd256)   || (audio_r <= -16'sd256)
};

always @(posedge clk) begin
    if (rst) begin
        au_nz_seen  <= 1'b0;
        au_nz_frame <= 1'b0;
        au_seen     <= 7'd0;
        au_vb_d     <= 1'b0;
        for (ak = 1; ak < 8; ak = ak + 1) au_hold[ak] <= 7'd0;
    end else begin
        au_vb_d <= vblank;
        if (au_nz) au_nz_seen <= 1'b1;
        au_seen <= au_seen | au_now;
        if (vblank && !au_vb_d) begin
            au_nz_frame <= au_nz_seen;
            au_nz_seen  <= 1'b0;
            au_seen     <= 7'd0;
            for (ak = 1; ak < 8; ak = ak + 1)
                au_hold[ak] <= au_seen[ak]             ? RUNG_HOLD :
                               (au_hold[ak] != 7'd0)   ? au_hold[ak] - 7'd1 : 7'd0;
        end
    end
end

assign dbg_audio = { au_hold[7] != 7'd0, au_hold[6] != 7'd0, au_hold[5] != 7'd0,
                     au_hold[4] != 7'd0, au_hold[3] != 7'd0, au_hold[2] != 7'd0,
                     au_hold[1] != 7'd0, au_nz_frame };

// ---------------------------------------------------------------------------
//  CPU read mux
//
//  Anything not listed answers 0xffff, which gx_main also does for unmapped
//  space: an unintended fetch should decode as something that traps, not as a
//  plausible NOP.
// ---------------------------------------------------------------------------
always @(*) begin
    if      (pal_cs)         dev_din = pal_dout;
    else if (k056832_ram_cs) dev_din = tm_ram_dout;
    else if (objram_cs)      dev_din = objram_q;
    else if (k054338_cs)     dev_din = k338_dout;
    else if (ccu_cs)         dev_din = {ccu_dout, ccu_dout};
    // Same umask32(0xff00ff00) as the CCU -- the byte is replicated into both
    // halves so either lane of the long reads it.  gx.cpp:1056.
    else if (k056800_cs)     dev_din = {k56_h_dout, k56_h_dout};
    else if (sysdsw_cs | inputs_cs | service_cs) dev_din = ctrl_dout;
    else                     dev_din = 16'hffff;
end

// ---------------------------------------------------------------------------
//  not yet built.  Listed so the gaps are visible from the RTL rather than
//  only from docs/IMPLEMENTATION_TODO.md.
//
//  TODO(P5): K055673 remainder        -- IRQ3 (U7, never enabled by this
//                                        game), the DMAEN trigger (U30), the
//                                        ROM readback windows objrom_cs and
//                                        objcha
//  TODO(SOUND): TMS57002 handshake    -- gx_sound returns 0 for the status
//                                        word; the sound CPU's most frequent
//                                        access in the blocking interval
//  TODO(SOUND): K054539 reverb ring   -- the engines are in gx_sound
//                                        (DECISIONS D13); the ring is not
//  TODO(P5): tile ROM readback window -- k056832_rom_cs, vram_chard
//  TODO(P5): watchdog and coin counters
//  esc_cs is no longer the ESC's only trace (2026-09-16): gx_main recognises
//  the long written to cc0000 itself and lends its bus to gx_esc (DECISIONS
//  D18), so the select stays open below only because nothing here needs it.
// ---------------------------------------------------------------------------
//  Quartus warning 10036, "assigned a value but never read", is a
//  project-owned warning and docs/WARNING_POLICY.md does not accept those.
//  The `wire _unused = &{...}` idiom that keeps Verilator quiet is exactly
//  what produces it, so instead every output with no consumer yet is left
//  UNCONNECTED at its instantiation and listed here.  Verilator does not
//  warn about that, Quartus does not warn about that, and the list below is
//  the documentation the reduction used to be.
//
//  Left open at their instantiations:
//      gx_main   k056832_rom_cs objrom_cs esc_cs
//      gx_esc    dbg_sprites
//      gx_ctrl   vram_chard objcha gfx_rst_n watchdog objscan coin_ctr
//                irq3_en
//      gx_sprite (none since 2026-09-14)
//      jt9346    dump_dout dump_flag
//      gx_ccu    hdisp vdisp
//      gx_tilemap  mix_a mix_b mix_c mix_d
//
//  `mem_rst` IS read, and it took a hardware run to find out that it had to
//  be.  It resets the SDRAM arbiter, and only the arbiter: the memory
//  transport has to keep running across a board reset because the ROM
//  download happens while the board is held in one.  The note that used to
//  sit here said the opposite -- that the board had nothing to do with it --
//  and that sentence was the bug.

endmodule

`default_nettype wire
