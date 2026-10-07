//============================================================================
//  Konami System GX -- main CPU (68EC020) and address decode
//
//  Memory map transcribed in docs/SOURCE_AUDIT.md section 4 from
//  konamigx.cpp:1040 (gx_base_memmap) + :1092 (gx_type2_map).
//
//  ---- the CPU -------------------------------------------------------------
//  TG68KdotC_Kernel with CPU="11" = 68020: extended addressing modes, 32-bit
//  MUL/DIV, bitfield instructions, VBR and extended stack frames.  LGPL-3.0,
//  Tobias Gubener.  docs/REUSE_PLAN.md section 1.
//
//  Two honest deviations from a real 68EC020, recorded so they are not
//  rediscovered as bugs:
//
//    1. NOT CYCLE ACCURATE.  Upstream says so itself: "The core does not value
//       cycle accuracy."  Nothing on this board is known to depend on 68020
//       cycle timing, but nothing has proved it does not either.
//    2. THE DATA BUS IS 16 BITS.  A real 68EC020 has a 32-bit bus with dynamic
//       sizing; TG68K splits every long access into two word accesses.  This
//       is invisible to software and it happens to suit us -- every device on
//       this board is 8 or 16 bits wide anyway, so a 16-bit port is the
//       natural width and no 32-bit assembly is needed.
//
//  The kernel's own interface is synchronous, not the 68000's asynchronous
//  AS/DTACK handshake.  TG68K.vhd wraps it into the async bus for Amiga-style
//  systems; we do not use that wrapper.  Instead:
//
//      busstate  00 fetch code   10 read data   11 write data   01 no access
//
//  and `clkena_in` is held low until the memory answers.  This is simpler and
//  faster than reconstructing AS/DTACK only to consume it again.
//
//  ---- byte lanes ----------------------------------------------------------
//  This is the one place the 32-bit board bus still shows through.  Several
//  devices are 8 bits wide and MAME maps them `umask32(0xff00ff00)`:
//  the CCU, the K056800 and the K055555.  That mask puts the device on bits
//  31-24 and 15-8 of each long, i.e. on the UPPER byte of each 16-bit half.
//
//  So for those three, a 16-bit access at any address carries the register
//  byte on D[15:8], and the register index is simply the word address.
//  Confirmed against the measured trace: docs/MEASUREMENTS.md section 2 shows
//  writes to D4C000 with mask FF000000 and to D4C002 with mask 0000FF00
//  landing on CCU registers 0 and 1 respectively -- exactly `addr[4:1]`.
//
//  The K054338 and K056832 are genuinely 16-bit (`word_w`) and take D[15:0].
//============================================================================
`default_nettype none

module gx_main (
    input  wire        clk,
    input  wire        rst,
    input  wire        cen,             // CPU clock enable (24 MHz domain)

    // --- neutral memory port for ROM and work RAM ----------------------------
    output wire [24:0] rom_addr,
    input  wire [15:0] rom_data,
    output wire        rom_req,
    input  wire        rom_ack,

    output wire [16:1] wram_addr,       // 128 KB
    output wire [15:0] wram_din,
    input  wire [15:0] wram_dout,
    output wire [1:0]  wram_we,

    // --- device selects.  All are word-addressed; see "byte lanes" above. ----
    output wire        ccu_cs,          // d4c000  K053252   8-bit on D[15:8]
    output wire        k056800_cs,      // d52000  K056800   8-bit on D[15:8]
    output wire        k055555_cs,      // d50000  K055555   8-bit on D[15:8]
    output wire        k054338_cs,      // d80000  K054338  16-bit
    output wire        k056832_reg_cs,  // d40000  K056832  16-bit, regs
    output wire        k056832_ram_cs,  // da0000  K056832  16-bit, VRAM window
    output wire        k056832_rom_cs,  // d00000  K056832  tile ROM readback
    output wire        objset1_cs,      // d48000  K053246   8-bit
    output wire        objset2_cs,      // d4a010  K055673  16-bit
    output wire        objram_cs,       // d20000  sprite RAM
    output wire        objrom_cs,       // d4a000  sprite ROM readback
    output wire        tilebank_cs,     // d44000
    output wire        pal_cs,          // d90000  palette RAM
    output wire        esc_cs,          // cc0000  ESC -- unused by gokuparo
    output wire        eeprom_cs,       // d56000
    output wire        control_cs,      // d58000
    output wire        sysdsw_cs,       // d5a000
    output wire        inputs_cs,       // d5c000
    output wire        service_cs,      // d5e000

    output wire [23:1] cpu_addr,
    output wire [15:0] cpu_dout,
    output wire        cpu_we,
    output wire [1:0]  cpu_ds,          // {uds, lds}, active high
    input  wire [15:0] dev_din,         // muxed device readback
    input  wire        dev_ok,          // device data valid (0 = insert wait)

    // --- interrupts ----------------------------------------------------------
    input  wire        int1,            // vblank        -> IPL 1
    input  wire        int2,            // programmable  -> IPL 2
    input  wire        irq3_set,        // object DMA end: set the level-3 latch (one clock)

    // --- the ESC (rtl/gx_esc.sv), DECISIONS D18 ------------------------------
    //  A long written to cc0000 asks for a run; the 68EC020 is frozen from the
    //  next clock until esc_done, and meanwhile the ESC's word bus is loaded
    //  into the transaction slice in the kernel's place.
    output reg         esc_start,       // one clock: the long's second word (cc0002) is written
    output reg  [31:0] esc_cmd,         //   that long, held until the next start
    input  wire        esc_done,        // one clock: the run has finished every effect
    //  ESC_CORE (rtl/esc/gx_esc056.sv, DECISIONS D31): the chip runs on its own
    //  and borrows the slice per WORD.  `esc_core` high: the cc0000 write no
    //  longer freezes the CPU; instead `esc_want` (the chip's held request)
    //  takes the slice at the next CPU cycle boundary, and gx_top's esc_done
    //  (the ack of that word) hands it back -- the same freeze and hand-back as
    //  D18, one word long.  Low: D18 unchanged, bit for bit.
    input  wire        esc_core,
    input  wire        esc_want,
    input  wire        esc_hold,        // D31: the chip is booting -- the kernel does not run
    input  wire        esc_irq4_set,    // one clock: raise IRQ4 (gx_esc gates it by the enable)
    input  wire        esc_req,
    input  wire        esc_we,
    input  wire [23:1] esc_addr,
    input  wire [15:0] esc_din,
    input  wire [1:0]  esc_be,          // {high byte, low byte}, writes
    output wire        esc_ack,         // one clock, with esc_rdata valid
    output wire [15:0] esc_rdata,

    // --- the fantjour DMA device (rtl/gx_fjdma.sv), DECISIONS D19 ------------
    //  MAME installs a write handler at 0xdb0000 for special 9 only
    //  (konamigx.cpp:4054-4055, konamigx_m.cpp:489-538), so the whole path is
    //  gated by the set.  The eight registers live in gx_fjdma; what crosses
    //  here is the slice's word write into them and the trigger, which is a
    //  write touching the TOP BYTE of long 0 (:501).  A run then takes the
    //  slice exactly as an ESC run does -- the same esc_own, the same freeze,
    //  the same hand-back -- and gx_top muxes the two masters onto the esc_*
    //  bus above.  They cannot overlap: a set is either sexyparo or fantjour.
    input  wire        fj_en,           // .mra byte 3 bit 1: the fantjour machine
    output wire        fj_reg_wr,       // one clock: a word write into the file
    output wire [4:1]  fj_reg_a,        // word index, 0..15
    output wire [15:0] fj_reg_d,
    output wire [1:0]  fj_reg_be,       // {high byte, low byte}
    output reg         fj_start,        // one clock: run with that file
    //  winspike's type 4 Xilinx protection (gx_xprot, special 7): the opcode
    //  latched from cc0004, run on the falling edge of cc0000 bit 9 (bit 25
    //  of the long, konamigx.cpp:893).  The freeze is the ESC's.
    input  wire        xp_en,
    output reg         xp_start,
    output reg         xp_d1c,          // with xp_start: opcode 0x0d1c (else 0x057a)

    // --- observability -------------------------------------------------------
    output wire [23:1] dbg_addr,
    output wire [1:0]  dbg_busstate,
    output wire        dbg_stalled,
    output wire        cache_on        // CACR bit 0, the 68020's instruction cache enable (POST HACK)
);

`include "gx_rommap.svh"

// ---------------------------------------------------------------------------
//  TG68K kernel
// ---------------------------------------------------------------------------
// ---- the CPU's raw outputs, and why nothing downstream uses them ---------
// TG68KdotC_Kernel's addr_out is COMBINATIONAL from the ALU.  Measured on the
// 2026-09-07 build: the worst setup path in the whole design started at the
// kernel's register file, ran through the ALU carry chain and two selectors,
// and ended at gx_memarb's latched address --
//
//     regfile(altsyncram)|ram_block1a0~PORT_B_WRITE_ENABLE_REG
//       -> gx_memarb|l_addr[7]      17.159 ns, 13 logic levels, slack -8.355
//
// About 8 ns of that is inside the CPU and about 9 ns is outside it.  The
// design's fabric is 96 MHz (10.4 ns) and THIS CPU'S OWN Fmax with the 68020
// options is 46.81 MHz, also measured.  So a combinational output of a 47 MHz
// core was being consumed by 96 MHz registers -- the arbiter, the work RAM,
// the tile VRAM, the palette and every device register file.
//
// docs/DECISIONS.md D8's multicycle is `-from TG68K* -to TG68K*` and that is
// deliberately right: gx_memarb's l_addr is NOT enable-gated, so it can latch
// on any 96 MHz edge and a one-cycle requirement is honest.  Widening the
// exception to those consumers would be a false promise, invisible until
// hardware.
wire [31:0] cpu_a32_raw;
wire [15:0] cpu_d_out_raw;
wire [1:0]  busstate_raw;
wire        nWr_raw, nUDS_raw, nLDS_raw, skipFetch_raw;
wire        nResetOut;
wire [3:0]  cacr;
wire [2:0]  fc;

reg         clkena;
reg  [15:0] cpu_din;

// IPL is active low and encodes the highest pending level.
// docs/SOURCE_AUDIT.md section 6: IRQ1 vblank, IRQ2 scanline, IRQ3 obj DMA,
// and IRQ4 the ESC's run end (DECISIONS D18).  Higher number = higher priority
// on 68k, so int4 wins.
//
// MAME raises IRQ4 with HOLD_LINE (konamigx.cpp:447): asserted until the CPU
// acknowledges it.  Here it is a latch that the level-4 IACK cycle clears --
// see `iack4` at the ESC section below.
reg        int4 = 1'b0;
// IRQ3 (object DMA end), 2026-09-30: MAME HOLD_LINE too (konamigx.cpp dmaend
// callback), so the same latch shape -- set by gx_top when the DMA ends with
// WRPORT1_1 bits 7 and 2 set, cleared by the level-3 IACK.  Dragoon Might's
// main loop waits on it; no set before this one enabled it (U7, U9).
reg        int3 = 1'b0;
wire [2:0] ipl_level = int4 ? 3'd4 : int3 ? 3'd3 : int2 ? 3'd2 : int1 ? 3'd1 : 3'd0;
wire [2:0] ipl_n     = ~ipl_level;

TG68KdotC_Kernel #(
    .SR_Read        (2),
    .VBR_Stackframe (2),
    .extAddr_Mode   (2),
    .MUL_Mode       (2),
    .DIV_Mode       (2),
    .BitField       (2),
    .BarrelShifter  (2),
    .MUL_Hardware   (1)
) u_cpu (
    .CPU            (2'b11),        // 68020
    .clk            (clk),
    .nReset         (~rst),
    .clkena_in      (clkena),
    .data_in        (cpu_din),
    .IPL            (ipl_n),
    .IPL_autovector (1'b1),         // the board has no vector generator
    .addr_out       (cpu_a32_raw),
    .berr           (1'b0),
    .FC             (fc),
    .data_write     (cpu_d_out_raw),
    .busstate       (busstate_raw),
    .nWr            (nWr_raw),
    .nUDS           (nUDS_raw),
    .nLDS           (nLDS_raw),
    .nResetOut      (nResetOut),
    .CACR_out       (cacr),
    .skipFetch      (skipFetch_raw)
);

// ---------------------------------------------------------------------------
//  TRANSACTION SLICE -- one 96 MHz register between the CPU and everything
//
//  This cuts the path above in half.  Nothing downstream sees a raw kernel
//  output any more, so the CPU's combinational depth ends here and the
//  address translation, the decoder, the arbiter and the local memories all
//  start from a register.
//
//  THE WHOLE BUNDLE MOVES ON ONE EDGE, and that is the point rather than a
//  tidiness preference.  Registering the address alone would let the arbiter
//  see a new request while it samples the previous address on the same edge,
//  and a local write enable could commit twice or to the wrong place.  Address,
//  data, byte strobes, direction and bus state are one transaction and they
//  travel together.
//
//  It costs TWO system clocks, not one CPU cycle, and the CPU cannot notice:
//  `cen` has period 4, so a value captured at edge N+2 is standing before the
//  CPU's next active edge at N+4.  `stall` is computed from the registered
//  copy for the same reason -- at every edge where `cen` is high, the copy
//  describes the access the CPU is actually waiting on.
//
//  ---- this paragraph used to say the opposite, and it was mine to fix -----
//  It read: "NO SDC CHANGE GOES WITH THIS.  Both halves stay honest
//  single-cycle paths ... A multicycle here would be a promise nothing in the
//  design keeps."  That was true while the slice captured at N+1.  It stopped
//  being true on 2026-09-08 when the slice moved to N+2, and a confident wrong
//  sentence sitting beside the thing it describes is worse than no sentence --
//  this project has already paid for one of those in check_pll.py.
//
//  What is true now:
//
//      kernel -> slice      TWO clocks (launch N, capture N+2).  Exception in
//                           KonamiGX.sdc, and it is the reason for the move.
//      slice  -> TG68K      TWO clocks (launch N+2, capture N+4).  Exception,
//                           reduced from three, which is the price.
//      slice  -> everything the decoder, work RAM and device register files
//               else        are NOT enable-gated, so these stay honest
//                           single-cycle paths and get no exception at all.
//
//  The third line is the one to keep intact.  Widening either exception's
//  destination list to cover it would be the false multicycle the old
//  paragraph was warning about, and this board cannot simulate the CPU that
//  would reveal it.
//
//  Nothing on the RESPONSE side is registered -- read data and the acks come
//  back combinationally, as before.  Adding a stage there would change when
//  the CPU samples its data and is a separate decision.
// ---------------------------------------------------------------------------
reg  [31:0] cpu_a32;
reg  [15:0] cpu_d_out;
reg  [1:0]  busstate;
reg         nWr, nUDS, nLDS, skipFetch;

//  ---- captured one clock AFTER cen, and that is not an optimisation -------
//  The kernel's outputs are combinational from registers that move on `cen`,
//  so they settle during the clock after a cen edge and then stand still for
//  the rest of the period.  Sampling every clock therefore stored the SAME
//  value four times, and TimeQuest -- which reasons about the register, not
//  about the data -- had to assume this slice could launch on any clock.
//
//  Sampling on a delayed `cen` stores exactly the same value.  What changes is
//  that the slice becomes a CPU-rate register, so the paths from it back INTO
//  the CPU can be given an honest exception.  The delay is TWO clocks, not
//  one -- see the note beside cen_d2 below for the measurement that moved it.
//
//  MEASURED, build N, 2026-09-07 -- the five worst paths in the design were
//
//      gx_main|cpu_a32[9] -> TG68KdotC_Kernel|TG68K_ALU|Flags[1]   -0.875
//
//  which is this slice feeding the address decoder, the read mux and so the
//  CPU's own data input.
//
//  Launch is at cen+2 and the CPU captures at the next cen, so the interval is
//  TWO clocks -- see KonamiGX.sdc, which says 2 for that reason.
//  Paths from here to anything NOT enable-gated -- the decoder's chip selects,
//  the work RAM, the device register files -- keep the full single-cycle
//  requirement, and the exception is written narrowly so they do.
reg cen_d1, cen_d2;
always @(posedge clk) begin
    cen_d1 <= cen;
    cen_d2 <= cen_d1;
end

//  ---- the slice moved from cen_d1 to cen_d2, and why -----------------------
//  MEASURED across five builds.  The recurring critical path of this design is
//
//      TG68K|regfile ram_block1a0 -> 8 logic levels -> gx_main|skipFetch
//
//  and it read -0.214, then better than +0.237, then -0.469, then -0.613 as
//  unrelated one-wire edits reshuffled placement.  Its detail says why it is
//  so sensitive: 8 levels AND 2.371 ns of clock skew, because the register
//  file's M10K gets a long clock leg wherever the fitter puts it.
//
//  On cen_d1 that path is ONE clock: the kernel launches on `cen` and this
//  slice captured on `cen + 1`.  No exception could relax it -- and a false
//  multicycle here is invisible until hardware, on the one module this
//  factory cannot simulate at all.  Two reseeds failed, which is the .qsf's
//  own criterion for stopping.
//
//  On cen_d2 it is TWO clocks, and so is the slice's path back into the CPU
//  (launch at cen+2, the CPU captures at cen+4).  Both directions then need
//  about 11 ns and have 20.8, which is the first comfortable number this
//  clock has seen.  KonamiGX.sdc carries the matching pair.
//
//  What it costs: the address, the chip selects and the write strobes appear
//  one clock later inside the CPU's four-clock period, so a device has two
//  clocks to answer instead of three.  Checked case by case --
//
//      work RAM   registered; address at cen+2, data at cen+3, the CPU
//                 samples at cen+4.  Still one clock of margin.
//      SDRAM      stalls anyway; one more clock per fetch.
//      palette,   self-timed handshakes (dout_ok / ram_ok), so they stall one
//      tilemap    more cen period at worst.
//      writes     the strobe is 2 clocks wide instead of 3, still >= 1.
//
//  What would make this unsound: shortening the CPU's enable period below 4
//  clocks, or giving any device a fixed-latency read of more than one clock
//  without a handshake.

//  ---- a SECOND copy of the address, for the work RAM alone ----------------
//  MEASURED, seed 7, 2026-09-08.  The two worst paths in the design were
//
//      gx_main|cpu_a32[10]
//        -> altsyncram:wram|ram_block1a2~portb_address_reg9    -0.059
//
//  i.e. this address register driving the work RAM's M10K address port.  That
//  destination is NOT enable-gated -- an M10K latches its address on every
//  clock whether the CPU is looking or not -- so it keeps the full 96 MHz
//  requirement and no exception can touch it (KonamiGX.sdc says exactly this
//  about the slice's non-CPU destinations).
//
//  `cpu_a32` fans out to the whole address decoder, every device register
//  file, the read mux AND these M10Ks, and one placement cannot be near all of
//  them.  So the work RAM gets its own copy, loaded from the same source on
//  the same enable: identical value, identical timing, and the fitter is free
//  to put it beside the memory.
//
//  This is register duplication done in RTL because Quartus 17.0 cannot be
//  asked to do it -- PHYSICAL_SYNTHESIS_REGISTER_DUPLICATION is OFF in the
//  .qsf because its fitter dies in sta_find_duplicates_of_deleted_net_name
//  after that pass runs.
//
//  `preserve` is not decoration: two registers with identical inputs are
//  exactly what the synthesiser merges, and merging them puts the fanout back.
(* preserve *) reg [16:1] cpu_a32_wram;

//  ---- the ESC in the kernel's place (DECISIONS D18) ------------------------
//  While `esc_own` is set the kernel is frozen (see `clkena`) and this slice
//  loads the ESC's pending word request instead of the kernel's outputs.  The
//  mux is in FRONT of the slice on purpose: everything behind it -- the
//  decoder, the ROM hold/used handshake, the work RAM's one address register
//  above and its byte-enable inference -- sees an ordinary bus cycle.  The
//  kernel's FC is loaded into fc_r with the rest, for the IACK test; an ESC
//  cycle carries supervisor data (5).
//
//  `esc_fin` is set by the run's done, and the next cen_d2 loads the KERNEL
//  again while clearing esc_own on the same edge.  Handing back on any other
//  edge would let a cen judge the ESC's last cycle with the kernel running.
reg  [2:0]  fc_r     = 3'd0;
reg         esc_own  = 1'b0;   // the ESC owns the slice; the kernel is frozen
reg         esc_fin  = 1'b0;   // the run has ended: hand back on the next cen_d2
reg         esc_live = 1'b0;   // the slice holds an ESC request not yet acked
wire        esc_load = esc_own && !esc_fin;

always @(posedge clk) begin
    if (rst) begin
        cpu_a32   <= 32'd0;
        cpu_a32_wram <= 16'd0;
        cpu_d_out <= 16'd0;
        busstate  <= 2'b01;        // 01 = no access, so nothing is selected
        nWr       <= 1'b1;
        nUDS      <= 1'b1;
        nLDS      <= 1'b1;
        skipFetch <= 1'b0;
        fc_r      <= 3'd0;
    end else if (cen_d2) begin
        if (esc_load) begin
            cpu_a32      <= {8'h00, esc_addr, 1'b0};
            cpu_a32_wram <= esc_addr[16:1];
            cpu_d_out    <= esc_din;
            busstate     <= !esc_req ? 2'b01 : esc_we ? 2'b11 : 2'b10;
            nWr          <= !(esc_req && esc_we);
            nUDS         <= !(esc_req && (!esc_we || esc_be[1]));
            nLDS         <= !(esc_req && (!esc_we || esc_be[0]));
            skipFetch    <= 1'b0;
            fc_r         <= 3'd5;
        end else begin
            cpu_a32   <= cpu_a32_raw;
            cpu_a32_wram <= cpu_a32_raw[16:1];
            cpu_d_out <= cpu_d_out_raw;
            busstate  <= busstate_raw;
            nWr       <= nWr_raw;
            nUDS      <= nUDS_raw;
            nLDS      <= nLDS_raw;
            skipFetch <= skipFetch_raw;
            fc_r      <= fc;
        end
    end
end

// 68EC020: 24 address bits.  Anything above is not decoded and must not alias
// into a real region -- a stray long fetch should read open bus, not RAM.
wire [23:1] a = cpu_a32[23:1];

assign cpu_addr = a;
assign cpu_dout = cpu_d_out;
assign cpu_we   = ~nWr;
assign cpu_ds   = {~nUDS, ~nLDS};

assign dbg_addr     = a;
assign dbg_busstate = busstate;
assign cache_on     = cacr[0];

// busstate 01 = no memory access this cycle
wire bus_active = (busstate != 2'b01) && !skipFetch;
wire is_write   = (busstate == 2'b11);

// ---------------------------------------------------------------------------
//  address decode
//
//  Lives in gx_decode.sv so it can be swept exhaustively without a CPU --
//  which matters here because this factory cannot simulate the main CPU at all
//  (TG68K is VHDL).  tb_gx_decode walks all 2^23 word addresses and checks both
//  that every select matches an independently written reference model and that
//  no two selects ever assert together.
//
//  Note that `da0000-da1fff` and `da2000-da3fff` go to the SAME handler -- the
//  K056832's CPU window is bank-selected by its own register 0x32, not by
//  address -- so one 16 KB select covers both.
// ---------------------------------------------------------------------------
wire sel_bios, sel_prg, sel_dat, sel_wram;
wire fjdma_cs;                      // db0000, the fantjour device (D19)

//  ---- the selects are REGISTERED, on the same edge as the address --------
//  MEASURED, 2026-09-30/10-01.  Two seeds of two netlists failed on the
//  decode between the slice and a 96 MHz consumer:
//
//      80f5c9b8 seed 4  -0.610  gx_main|cpu_a32 -> gx_tilemap vram we
//      0f1eddff seed 8  -0.196  gx_main|cpu_a32 -> gx_memarb|l_addr
//
//  and 9c70d73 had measured the decode at 4.7 ns of such a path.  The .qsf's
//  rule for a `cpu_a32 -> vram we` return is a register stage on the decode.
//
//  It is NOT a pipeline stage.  The decoder here reads the value the slice is
//  ABOUT to load (`dec_a`, `dec_active` -- the same muxes the slice uses), and
//  `dsel_r` loads on the same cen_d2 edge.  The address and busstate change on
//  no other edge (reset aside), so dsel_r equals gx_decode(cpu_a32,
//  bus_active) on every clock: the same selects, on the same clock, as the
//  combinational decode it replaces.  What moves is only where the 4.7 ns is
//  paid -- on the kernel -> slice side, which already has two clocks
//  (KonamiGX.sdc lists dsel_r with the slice for that reason).
//
//  Reset value = gx_decode(0, inactive): sel_bios alone.
wire [23:0] dec_a      = esc_load ? {esc_addr, 1'b0} : cpu_a32_raw[23:0];
wire [1:0]  dec_bs     = esc_load ? (!esc_req ? 2'b01 : esc_we ? 2'b11 : 2'b10)
                                  : busstate_raw;
wire        dec_active = (dec_bs != 2'b01) && !(esc_load ? 1'b0 : skipFetch_raw);

wire [24:0] dsel_n;
reg  [24:0] dsel_r = 25'd1 << 24;

gx_decode u_decode (
    .a              (dec_a),
    .active         (dec_active),
    .sel_bios       (dsel_n[24]),
    .sel_prg        (dsel_n[23]),
    .sel_dat        (dsel_n[22]),
    .sel_wram       (dsel_n[21]),
    .esc_cs         (dsel_n[20]),
    .k056832_rom_cs (dsel_n[19]),
    .objram_cs      (dsel_n[18]),
    .k056832_reg_cs (dsel_n[17]),
    .tilebank_cs    (dsel_n[16]),
    .objset1_cs     (dsel_n[15]),
    .objrom_cs      (dsel_n[14]),
    .objset2_cs     (dsel_n[13]),
    .ccu_cs         (dsel_n[12]),
    .ccu2_cs        (dsel_n[11]),
    .k055555_cs     (dsel_n[10]),
    .k056800_cs     (dsel_n[9]),
    .eeprom_cs      (dsel_n[8]),
    .control_cs     (dsel_n[7]),
    .sysdsw_cs      (dsel_n[6]),
    .inputs_cs      (dsel_n[5]),
    .service_cs     (dsel_n[4]),
    .k054338_cs     (dsel_n[3]),
    .pal_cs         (dsel_n[2]),
    .k056832_ram_cs (dsel_n[1]),
    .fjdma_cs       (dsel_n[0])
);

always @(posedge clk) begin
    if (rst)         dsel_r <= 25'd1 << 24;
    else if (cen_d2) dsel_r <= dsel_n;
end

//  dsel_r[11] is ccu2_cs, which nothing reads (see below) -- it was an
//  unconnected instance output before, and a named wire here would be a 10036.
assign {sel_bios, sel_prg, sel_dat, sel_wram, esc_cs, k056832_rom_cs,
        objram_cs, k056832_reg_cs, tilebank_cs, objset1_cs, objrom_cs,
        objset2_cs, ccu_cs} = dsel_r[24:12];
assign {k055555_cs, k056800_cs, eeprom_cs,
        control_cs, sysdsw_cs, inputs_cs, service_cs, k054338_cs, pal_cs,
        k056832_ram_cs, fjdma_cs} = dsel_r[10:0];

// ccu2 (d4e000) is `nopw` on type 2 -- the second CCU only exists on the dual
// screen type 3/4 boards.  Decoded so a write there is absorbed rather than
// falling through to open bus, which is what MAME does.
// sel_dat likewise: the 400000-7fffff window is real on the board but gokuparo
// loads nothing into it (docs/SOURCE_AUDIT.md section 13), so it deliberately
// reads open bus instead of aliasing the program ROM.

// ---------------------------------------------------------------------------
//  SDRAM fetch for the two ROM windows
//
//  The BIOS is a separate 128 KB image at CPU 0x000000; the program ROM is a
//  2 MB window at 0x200000 of which gokuparo fills 1 MB.  The data ROM window
//  at 0x400000 is decoded but gokuparo loads nothing there
//  (docs/SOURCE_AUDIT.md section 13), so it reads as open bus rather than
//  aliasing the program -- a stray fetch should look wrong, not plausible.
// ---------------------------------------------------------------------------
// The data ROM socket answers 0x400000-0x5fffff on every set (MAME maps
// 0x400000-0x7fffff .rom() in the one Type 2 map, konamigx.cpp:1044).
// dragoonj / dragoona load 417a04/417a05 there (ROM_LOAD32_WORD_SWAP); the
// other sets' .mra leave the region zero-filled, which is what an empty
// socket reads as here.  Until 2026-10-06 the window was gated to dragoonj by
// machine number; MEASUREMENTS 180: in 9,000 frames of MAME (attract, coin,
// start, play) no other set reads 0x400000-0x7fffff even once, so the gate
// changed nothing.  0x600000-0x7fffff stays open bus in all of them.
wire sel_dat_rom = sel_dat && !cpu_a32[21];
wire rom_sel = sel_bios | sel_prg | sel_dat_rom;

//  `rom_addr` is a WORD address -- see the note in gx_rommap.svh.  The CPU
//  gives a word index already (cpu_a32[n:1]), so the region base is the only
//  thing that needs halving, and it is a constant.
//
//  This used to add the base as a byte address to a word index, which is a
//  factor of two and would have fetched the wrong half of the ROM everywhere.
//  Nothing caught it because the two conventions were never written down;
//  they are now.
assign rom_addr = sel_bios    ? ({1'b0, GX_BIOS_BASE[24:1]} + {9'd0, cpu_a32[16:1]})
                : sel_dat_rom ? ({1'b0, GX_DATA_BASE[24:1]} + {5'd0, cpu_a32[20:1]})
                              : ({1'b0, GX_MAIN_BASE[24:1]} + {6'd0, cpu_a32[19:1]});
assign rom_req  = bus_active && rom_sel && !is_write;

// ---------------------------------------------------------------------------
//  work RAM
// ---------------------------------------------------------------------------
// The duplicate, not cpu_a32 -- see its declaration.  Same value on the same
// edge, so this is a placement change and nothing else.
assign wram_addr = cpu_a32_wram;
assign wram_din  = cpu_d_out;
assign wram_we   = (bus_active && sel_wram && is_write) ? {~nUDS, ~nLDS} : 2'b00;

// ---------------------------------------------------------------------------
//  read mux and wait-state generation
//
//  Work RAM needs NO wait state, and that is a consequence of `cen` below.
//  The CPU advances only on a 24 MHz enable, so it holds an address for four
//  system clocks; the RAM is registered and answers on the first of them.
//
//  There used to be a `wram_pend` flip-flop here that inserted one cycle.  It
//  had to go: it toggles with period 2 and `cen` has period 4, so once the CPU
//  was correctly gated the flag would have read the SAME value on every cen
//  edge -- and if that value was 0, the CPU would have stalled on work RAM
//  forever.  A wait state that is unnecessary is not harmless.
//
//  The SDRAM fetch still stalls, and so do the palette and the tilemap window,
//  whose CPU reads share a port with the video side and are not ready in a
//  fixed number of clocks.
// ---------------------------------------------------------------------------

wire dev_sel = esc_cs | k056832_rom_cs | objram_cs | k056832_reg_cs | tilebank_cs
             | objset1_cs | objrom_cs | objset2_cs | ccu_cs | k055555_cs
             | k056800_cs | eeprom_cs | control_cs | sysdsw_cs | inputs_cs
             | service_cs | k054338_cs | pal_cs | k056832_ram_cs;

wire stall = bus_active &&
             (  (rom_sel && !rom_ack)
              | (dev_sel && !dev_ok) );

assign dbg_stalled = stall;

// ---------------------------------------------------------------------------
//  THE CPU CLOCK ENABLE
//
//  `cen` was declared on this module's port list and never used, so the
//  68EC020 ran at the full system clock -- 96 MHz instead of 24, four times
//  its own speed.  The first Quartus build is what exposed it, and not as a
//  speed bug: TimeQuest reported Fmax 46.81 MHz against a 96 MHz constraint,
//  every one of the five worst paths inside TG68K, because the tool was being
//  asked to close the CPU's whole datapath in one 10.4 ns clock.
//
//  With this line the CPU advances on one clock in four, which is both the
//  right speed and what makes the multicycle constraint in
//  targets/mister/KonamiGX.sdc legitimate.  See docs/DECISIONS.md D8 for the
//  argument that the constraint is sound, and for exactly what would make it
//  unsound again.
//
//  `!esc_own`: frozen for an ESC run (DECISIONS D18).  KonamiGX.sdc gives
//  esc_own the same two-clock exception as the slice, and says why it is true.
always @(*) begin
    clkena = cen && !stall && !esc_own && !esc_hold;
end

always @(*) begin
    // sel_dat_rom (dragoonj's data window) is the LAST branch, not part of
    // the first: the windows are disjoint, so the order changes nothing, and
    // putting it in `rom_sel` there lengthened every other read's select --
    // build f5baea90: jt054338 regs -> cpu_din -> TG68K ALU flags, -3.35 ns.
    if      (sel_bios || sel_prg) cpu_din = rom_data;
    else if (sel_wram)    cpu_din = wram_dout;
    else if (dev_sel)     cpu_din = dev_din;
    else if (sel_dat_rom) cpu_din = rom_data;
    // Unmapped, including the empty data-ROM window.  MAME returns the bus
    // value; returning 0xffff here makes an unintended fetch decode as a
    // trap-generating opcode rather than as a plausible NOP.
    else               cpu_din = 16'hffff;
end

// ---------------------------------------------------------------------------
//  the ESC's run -- ask, freeze, hand back -- and IRQ4 (DECISIONS D18)
//
//  TG68K splits the long written to cc0000 into two word writes, cc0000 then
//  cc0002 (header, deviation 2).  The first word is latched; the edge that
//  completes the second asks gx_top for a run and sets esc_own, which holds
//  clkena low from the next clock.  MAME calls esc_w once for the long
//  (konamigx.cpp:380), and every write the game makes there is a long from one
//  PC (dist/esctap_b/esc_calls.txt), so a lone word to cc0002 is not modelled.
//
//  The ack is this module's own `cen && !stall` for a request the slice holds:
//  the edge on which the kernel would have consumed the cycle, so gx_top's ROM
//  hold/used handshake is consumed exactly as it is for the CPU.  `esc_live`
//  keeps an acked request from being acked again while the slice still shows
//  it; gx_esc drops its request on the ack, so the next cen_d2 cannot reload
//  the old one.
//
//  IRQ4 clears on the interrupt-acknowledge cycle.  TG68K in autovector mode
//  still runs one: a data read with FC = 7 at 0xFFFFFFF8 for level 4
//  (TG68KdotC_Kernel.vhd:1563-1567, 1157-1159, 937-938; the data is ignored,
//  :1127-1134).  The kernel has latched the level by the time that cycle
//  appears, so clearing on it is safe; the address reads unmapped, no stall.
// ---------------------------------------------------------------------------
reg  [15:0] esc_hi = 16'd0;    // the long's first word, from cc0000

wire cpu_step    = cen && !stall;        // the slice's cycle completes on this edge
wire esc_word_wr = cpu_step && !esc_own && bus_active && is_write && esc_cs && !a[2] && !xp_en;
wire xp_word_wr  = cpu_step && !esc_own && bus_active && is_write && esc_cs && xp_en;
reg  [15:0] xp_op  = 16'hFFFF;   // MAME m_last_prot_op starts at -1
reg         xp_clk = 1'b0;
wire        xp_trig = xp_word_wr && (a[2:1] == 2'b00) && !cpu_d_out[9] && xp_clk
                      && (xp_op == 16'h057A || xp_op == 16'h0D1C);
wire iack4       = cpu_step && !esc_own && (busstate == 2'b10) && (fc_r == 3'd7)
                   && (cpu_a32[31:1] == 31'h7FFF_FFFC);
wire iack3       = cpu_step && !esc_own && (busstate == 2'b10) && (fc_r == 3'd7)
                   && (cpu_a32[31:1] == 31'h7FFF_FFFB);

assign esc_ack   = esc_own && esc_live && cpu_step;
assign esc_rdata = cpu_din;

//  ---- the fantjour device's register file and trigger (DECISIONS D19) ------
//  The slice's word writes into 0xdb0000-0xdb001f go to gx_fjdma, which keeps
//  the eight longs.  A run is asked for by a write touching the TOP BYTE of
//  long 0 (konamigx_m.cpp:501, `!offset && ACCESSING_BITS_24_31`), which on
//  this 16-bit bus is the high byte of word 0.  `mode` and `sz2` are both in
//  that word, so the trigger carries everything the run reads out of long 0;
//  `sz1`, in the long's low word, is unused in MAME as well (:511).
//
//  `fj_en` is the set.  With it low nothing here happens and 0xdb0000 stays
//  unmapped, which is what MAME's map is for every set but these two.
wire fj_word_wr = cpu_step && !esc_own && bus_active && is_write && fjdma_cs && fj_en;

//  REGISTERED on the way out, all five of them.  MEASURED, build fd6e145:
//  with the write forwarded combinationally the core clock missed at
//  -0.133 ns and every failing path was `cpu_a32[8] -> gx_fjdma|bus_addr[*]`
//  -- the slice's address bits reaching the other module inside one 96 MHz
//  clock through the select and the file's write enable.  A register here
//  starts that path fresh and costs 22 flops.
//
//  `fj_start` is delayed by the same clock so the ORDER survives: the trigger
//  word lands in gx_fjdma's file on the edge `fj_trig` is high, and `start`
//  arrives the edge after.  The CPU is already frozen from the trigger, so the
//  extra clock costs the game nothing.
reg        fj_trig   = 1'b0;
reg        fj_wr_r   = 1'b0;
reg [4:1]  fj_a_r    = 4'd0;
reg [15:0] fj_d_r    = 16'd0;
reg [1:0]  fj_be_r   = 2'b00;

assign fj_reg_wr = fj_wr_r;
assign fj_reg_a  = fj_a_r;
assign fj_reg_d  = fj_d_r;
assign fj_reg_be = fj_be_r;

always @(posedge clk) begin
    if (rst) begin
        fj_trig <= 1'b0;
        fj_wr_r <= 1'b0;
    end else begin
        fj_wr_r <= fj_word_wr;
        fj_a_r  <= a[4:1];
        fj_d_r  <= cpu_d_out;
        fj_be_r <= {~nUDS, ~nLDS};
        fj_trig <= fj_word_wr && (a[4:1] == 4'd0) && ~nUDS;
    end
end

always @(posedge clk) begin
    esc_start <= 1'b0;
    fj_start  <= 1'b0;
    xp_start  <= 1'b0;
    if (rst) begin
        esc_hi   <= 16'd0;
        xp_op    <= 16'hFFFF;
        xp_clk   <= 1'b0;
        esc_cmd  <= 32'd0;
        esc_own  <= 1'b0;
        esc_fin  <= 1'b0;
        esc_live <= 1'b0;
        int4     <= 1'b0;
        int3     <= 1'b0;
    end else begin
        if (esc_word_wr && !a[1]) esc_hi <= cpu_d_out;
        if (esc_word_wr &&  a[1]) begin
            esc_cmd   <= {esc_hi, cpu_d_out};
            esc_start <= 1'b1;
            if (!esc_core) esc_own <= 1'b1;
        end
        //  D31: the chip's word, at a CPU cycle boundary (cpu_step is the edge
        //  the kernel's own cycle completes on, as for the cc0002 write above).
        //  Not while a hand-back is pending (esc_fin), so a release and a take
        //  never meet on one edge.
        if (esc_core && esc_want && cpu_step && !esc_own) esc_own <= 1'b1;

        //  The same seam, the other master.  The freeze happens AT the trigger,
        //  like the ESC's; the run is asked for one clock later, when the
        //  registered write has reached gx_fjdma's file (see fj_trig above).
        //  gx_top then starts it when the sprite DMA is quiet, and the
        //  hand-back is esc_done below.
        if (fj_word_wr && (a[4:1] == 4'd0) && ~nUDS) esc_own  <= 1'b1;
        if (fj_trig)                                 fj_start <= 1'b1;
        if (xp_word_wr && (a[2:1] == 2'b10))         xp_op    <= cpu_d_out;
        if (xp_word_wr && (a[2:1] == 2'b00))         xp_clk   <= cpu_d_out[9];
        if (xp_trig) begin
            esc_own  <= 1'b1;
            xp_start <= 1'b1;
            xp_d1c   <= (xp_op == 16'h0D1C);
        end

        if (cen_d2 && esc_load) esc_live <= esc_req;
        else if (esc_ack)       esc_live <= 1'b0;

        if (esc_done)           esc_fin  <= 1'b1;
        if (cen_d2 && esc_own && esc_fin) begin
            esc_own <= 1'b0;
            esc_fin <= 1'b0;
        end

        if (esc_irq4_set)       int4 <= 1'b1;
        else if (iack4)         int4 <= 1'b0;

        if (irq3_set)           int3 <= 1'b1;
        else if (iack3)         int3 <= 1'b0;
    end
end

endmodule

`default_nettype wire
