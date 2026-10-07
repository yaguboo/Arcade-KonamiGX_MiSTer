//============================================================================
//  Konami 056832 (+054156) -- four tilemap layers, 5 bpp
//
//  Sources, in the order root section 1.2 puts them:
//
//    MAME     k054156_k054157_k056832.cpp -- register map, page grid, tile
//             encoding, linescroll.  Transcribed in SOURCE_AUDIT section 8.
//    SILICON  jtcores modules/jt05415x, reconstructed from Furrtek's reverse
//             engineering of the K054156/K054157.  Its doc/register_map.md
//             corrects MAME in four places and is cited below where it does.
//    MEASURED docs/MEASUREMENTS.md section 4 -- what gokuparo actually writes.
//
//  jt05415x itself could not be reused: its register file is generated at
//  build time by the `jtframe mmr` Go tool and is .gitignore'd upstream, and
//  the module is explicitly partial (no tile ROM fetch, no colour path).
//  docs/REUSE_PLAN.md section 1.
//
//  ---- one MAME comment is wrong and it matters -----------------------------
//  MAME's header says the layer grid registers are at byte offsets "10-13"
//  and "14-17", one byte per layer.  They are not.  MAME's own CODE uses
//  `offset >= 0x10/2 && offset <= 0x16/2` -- word-spaced registers at byte
//  0x10, 0x12, 0x14, 0x16 and 0x18, 0x1a, 0x1c, 0x1e.  The silicon map agrees
//  with the code, and so does the measured trace: gokuparo writes the low byte
//  of each of those eight words (bytes 0x11, 0x13, ... 0x1f).
//
//  Three independent sources against one comment.  The comment loses.
//
//  ---- what the measurement says the geometry actually is -------------------
//  docs/MEASUREMENTS.md section 4.  gokuparo writes:
//
//      word 0x10 = 0x00   layer A  y=0 h=0        word 0x18 = 0x00  x=0 w=0
//      word 0x12 = 0x00   layer B  y=0 h=0        word 0x1a = 0x08  x=1 w=0
//      word 0x14 = 0x08   layer C  y=1 h=0        word 0x1c = 0x00  x=0 w=0
//      word 0x16 = 0x08   layer D  y=1 h=0        word 0x1e = 0x08  x=1 w=0
//
//  so AT BOOT all four layers are ONE 64x32 page each, at grid positions
//  A=(0,0) B=(1,0) C=(0,1) D=(1,1), i.e. pages 0, 1, 4 and 5.
//
//  That is boot only.  From the attract onward every layer spans 2x2 pages
//  (MEASURED, tools/gx_layerdump.lua frames 680-5400: grid words
//  01 01 11 10 / 01 11 01 11), so the span-2 path is the one in use.  Spans
//  of 3 and 4 are implemented because that is what the registers mean.
//
//  ---- page addressing ------------------------------------------------------
//  MAME: page_idx = (((rowstart + r) & 3) << 2) + ((colstart + c) & 3), a 4x4
//  grid of 64x32-tile pages, each 512x256 pixels.
//
//  Silicon agrees on the shape and adds the wiring: VA[10:0] is the intra-page
//  tile address, VA[13:11] is the page X field and VA[16:14] the page Y field,
//  with the low bit of each forced to zero -- so the four usable page
//  coordinates are 0, 2, 4, 6 in the raw field, which is the 2-bit grid index
//  shifted left by one.  That is why the register fields are 3 bits wide while
//  only 2 bits are meaningful.
//
//  ---- tile encoding --------------------------------------------------------
//  4 bytes per tile, 0x2000-byte page banks (confirmed by the CPU window being
//  0x2000 and by all 16 pages being walked at boot):
//
//      word0 = attr:  ---- ---- pppp --yx
//      word1 = code:  cccc cccc cccc cccc
//
//  The palette/flip split is not fixed: `fbits = (regs[3] >> 6) & 3` selects
//  one of four shift/mask sets (k056832_shiftmasks).  All four are implemented.
//
//  The GX driver then remaps the code through an 8-entry external bank table
//  (konamigx_tilebank_w at 0xd44000):
//      code = (tilebank[code[15:13]] << 13) | code[12:0]
//  and the colour through the K055555's per-layer decode, which happens
//  downstream in gx_mixer, not here.
//============================================================================
`default_nettype none

module gx_tilemap #(
    // The CCU's active window is a 288x224 crop at (24,16) in the GX
    // 384x264 raster (SOURCE_AUDIT §3).  Unit benches exercise the chip at
    // raster origin, so the board top supplies these two real visible-area
    // offsets explicitly instead of hiding them in the fetch arithmetic.
    parameter [9:0] SCREEN_X_OFFSET = 10'd0,
    parameter [8:0] SCREEN_Y_OFFSET = 9'd0,
    // Per-layer horizontal displacement in MAME's set_layer_offs convention:
    // positive moves the layer RIGHT, so vx = x + SCREEN_X_OFFSET + scroll -
    // LAYER_DX.  Zero here; gx_top supplies the GX values and says where they
    // come from.
    parameter signed [9:0] LAYER_DX_A = 10'sd0,
    parameter signed [9:0] LAYER_DX_B = 10'sd0,
    parameter signed [9:0] LAYER_DX_C = 10'sd0,
    parameter signed [9:0] LAYER_DX_D = 10'sd0,
    // GQ: a TWO-group staging queue (MEASUREMENTS 146, Codex's design).  Each
    // group is fetched two edges ahead into one of two stage banks, and a
    // fetch that finds the FSM still busy at its edge is held and launched the
    // moment it is free -- so two consecutive fetches share 256 clocks instead
    // of each owning exactly 128.  0 is the design before 2026-09-28 exactly.
    parameter bit GQ = 1'b1
) (
    input  wire        clk,
    input  wire        rst,

    // --- CPU: register bank 1 (0xd40000-0xd4003f, 16-bit) --------------------
    input  wire        reg_cs,
    input  wire        reg_we,
    input  wire [5:1]  reg_addr,
    input  wire [15:0] reg_din,
    input  wire [1:0]  reg_ds,

    // --- CPU: VRAM window (0xda0000-0xda3fff, 16-bit) -----------------------
    //  8 KB visible at a time; which of the 16 pages is bank-selected by
    //  register 0x32 (word index 0x19).  MAME maps 16 KB to the same handler
    //  because the window is mirrored, not because two pages are visible.
    input  wire        ram_cs,
    input  wire        ram_we,
    input  wire [13:1] ram_addr,
    input  wire [15:0] ram_din,
    input  wire [1:0]  ram_ds,
    output wire [15:0] ram_dout,
    // The CPU read shares the one VRAM read port with the tile fetcher, so it
    // is not ready in one clock.  gx_top folds this into gx_main's `dev_ok`.
    output reg         ram_ok,

    // --- CPU: external tile bank table (0xd44000, GX-specific) ---------------
    input  wire        tilebank_cs,
    input  wire        tilebank_we,
    input  wire [3:1]  tilebank_addr,
    input  wire [15:0] tilebank_din,
    input  wire [1:0]  tilebank_ds,

    // --- raster --------------------------------------------------------------
    input  wire        pxl_cen,
    input  wire [9:0]  hcnt,
    input  wire [8:0]  vcnt,
    //  Raster geometry from the CCU.  Needed because this module fetches AHEAD
    //  of the beam and the lead has to wrap into the raster; the exact group
    //  destination is registered below before address generation.
    input  wire [9:0]  htotal,
    input  wire [9:0]  vtotal,
    input  wire [9:0]  hres,
    input  wire [9:0]  vres,

    // --- tile ROM, 4bpp planes + separate 5th plane.  gx_rommap.svh explains
    //     why the PCB's split is kept instead of packing to 5 bpp.
    //     20-bit row index since 2026-09-30: salmndr2's 4bpp ROM is 3 MB.
    output reg  [19:0] rom4_addr,      // 32-bit word address into GX_TILE4
    input  wire [31:0] rom4_data,
    output reg  [19:0] rom1_addr,      // the row index into GX_TILE1 (a byte, or a word at 6 bpp)
    input  wire [31:0] rom1_data,      // [7:0] plane 4, [15:8] plane 5 (6 bpp); the fetcher zeroes what a set lacks;
                                       // 8 bpp: bytes 4-7 of the row (gx_plane_word order)
    input  wire        bpp8,           // K056832_BPP_8 (winspike): charlayout8, rom4 = bytes 0-3, rom1 = 4-7
    // Per-set horizontal adjustment folded into every layer's displacement
    // (MAME set_layer_offs and the k053252 crop origin): dragoonj's layers sit
    // one dot further right (VIDEO_START dragoonj) and its visible area starts
    // 16 dots later (set_offsets(24+16,16)), so -15.  0 for every other set.
    // It meets the scroll in the REGISTERED subtraction below, not the fetch.
    input  wire signed [9:0] dx_adj,
    output reg         rom_req,
    input  wire        rom_ok,

    // One-cycle pulse after all four layers for one 8-pixel group have been
    // staged.  The SDRAM policy uses this boundary to yield to soft-deadline
    // clients without interrupting the tilemap's hard-deadline group.
    output reg         group_done,
    output wire        group_busy,

    // --- per-layer pixel output, in raster order -----------------------------
    //  Colour assembly (PALBASE, the VINMIX split) is NOT done here -- that is
    //  the K055555's job and it needs the raw 8-bit tile colour.
    output wire [7:0]  pxl_a, pxl_b, pxl_c, pxl_d,     // 6 bits: salmndr2 (K056832_BPP_6); 8: winspike
    output wire [7:0]  col_a, col_b, col_c, col_d,
    output wire [1:0]  mix_a, mix_b, mix_c, mix_d,  // attr bits for per-tile blend

    // --- LAYER A instrumentation, 2026-09-08 fourth session -----------------
    //  Four one-clock PULSES.  They live here rather than in gx_top because
    //  every one of them needs `lyr`: the fetch loop visits four layers
    //  through ONE ROM port and ONE VRAM port, so a ROM word seen from
    //  outside belongs to whichever layer happened to be current, and gx_top
    //  cannot tell which that was.  The previous build's "SDRAM answered
    //  non-zero" rung had exactly that hole.
    //
    //      [0]  layer A's 4bpp fetch came back non-zero
    //      [1]  layer A's 1bpp fetch came back non-zero
    //      [2]  layer A's tile CODE, straight out of VRAM, was non-zero
    //      [3]  a layer A grid / scroll register was written
    //
    //  gx_top decides which of these are per-frame and which are sticky, so
    //  that policy lives in one place.
    output reg  [3:0]  dbg_a_hit,

    // Direct board check for the first 'R' in "ROM RAM CHECK".  Bit 0 pulses
    // when layer A fetches code 0x52 row 7 (ROM index 0x297); bit 1 pulses
    // when the 40 returned ROM bits do NOT match the local ROM/MAME oracle.
    output reg  [1:0]  dbg_ref_hit,

    // One-clock pulse when a visible 8-pixel group reaches its launch edge
    // while the previous group is still in flight.  The shifters load at the
    // edge whether fresh data arrived or not, so this is the hard deadline.
    output reg         dbg_late
);

// ---------------------------------------------------------------------------
//  register file
// ---------------------------------------------------------------------------
reg [15:0] regs [0:31];

integer ri;
always @(posedge clk) begin
    if (rst) begin
        for (ri = 0; ri < 32; ri = ri + 1) regs[ri] <= 16'd0;
    end else if (reg_cs && reg_we) begin
        if (reg_ds[1]) regs[reg_addr][15:8] <= reg_din[15:8];
        if (reg_ds[0]) regs[reg_addr][ 7:0] <= reg_din[ 7:0];
    end
end

// Named accessors, word indices exactly as MAME indexes m_regs.  The 68EC020
// writes this chip one 16-bit word per register; the real-ROM trace puts
// 0x0104 in word 0, 0x00FF in word 1 and 0x00D0 in word 3.
//
//  ---- 2026-09-14: 73b582c's byte reading is withdrawn ---------------------
//  73b582c moved global flip to bits 12/13 and the tile-flip enable to
//  regs[0][7:0] to stop every glyph rotating 180 degrees in place.  The
//  rotation was real; the register map was not its cause.  The .mra DIP
//  default was FD -- Flip Screen ON -- so the game wrote regs[0] = 0x0134
//  (reproduced by a MAME run with Flip Screen forced On) and this module's
//  in-place flip rotated each tile.  With the DIP at FE the game writes
//  0x0104, and MAME's code is the map (k054156_k054157_k056832.cpp):
//
//      global flip X / Y   m_regs[0] & 0x10 / 0x20          :1080-1083
//      ext linescroll      m_regs[0] & 0x02                 :666
//      tile flip enable    m_regs[1] >> (layer*2) & 3       :618
//      FBITS               m_regs[3] >> 6 & 3               :617
//
//  The global flip below is still only the in-place pixel/row flip.  A real
//  screen flip also mirrors tile positions.  TODO(P5).
wire        glob_hflip = regs[5'h00][4];
wire        glob_vflip = regs[5'h00][5];
// Bit 1 points the CPU window at page 16, the external linescroll RAM.  The
// renderer does not read that page: MAME draws from `scrollbank` unless a game
// sets m_use_ext_linescroll (:1333), and konamigx never does.
wire        ext_lnscr  = regs[5'h00][1];
wire [1:0]  fbits      = regs[5'h03][7:6];
// byte 0x0a, two bits per layer -- register 0x05: 0 = per-line, 1 = unknown
// (U23; MAME draws it as XY, :1392), 2 = per-8-lines, 3 = xy scroll.  Modes 0
// and 2 are rendered since 2026-09-15 -- "LINE SCROLL" below.
wire [7:0]  cpu_bank   = regs[5'h19][7:0];      // byte 0x32
// byte 0x38 -- register 0x1c, the chip's own 4-entry bank LUT.  Same reason
// as lnscr_ctl above; TODO(P6) at the end of this file.
//    wire [15:0] tile_lut = regs[5'h1c];

// per-layer grid and scroll
wire [1:0] lyr_y  [0:3];
wire [1:0] lyr_h  [0:3];
wire [1:0] lyr_x  [0:3];
wire [1:0] lyr_w  [0:3];
wire [15:0] scr_y [0:3];
wire [15:0] scr_x [0:3];

genvar g;
generate
for (g = 0; g < 4; g = g + 1) begin : gv
    // words 0x08..0x0b = bytes 0x10..0x16 : vertical grid
    assign lyr_y[g] = regs[5'h08 + g][4:3];
    assign lyr_h[g] = regs[5'h08 + g][1:0];
    // words 0x0c..0x0f = bytes 0x18..0x1e : horizontal grid
    assign lyr_x[g] = regs[5'h0c + g][4:3];
    assign lyr_w[g] = regs[5'h0c + g][1:0];
    // words 0x10..0x13 = bytes 0x20..0x26 : Y scroll
    assign scr_y[g] = regs[5'h10 + g];
    // words 0x14..0x17 = bytes 0x28..0x2e : X scroll
    assign scr_x[g] = regs[5'h14 + g];
end
endgenerate

// X scroll with the per-layer displacement already taken off, REGISTERED.
// The fetcher reads this and not scr_x, so the subtraction costs the
// hcnt -> fetch_addr path nothing; a CPU scroll write reaches the fetch one
// system clock later, which no dot can see.
wire signed [9:0] layer_dx [0:3];
assign layer_dx[0] = LAYER_DX_A;
assign layer_dx[1] = LAYER_DX_B;
assign layer_dx[2] = LAYER_DX_C;
assign layer_dx[3] = LAYER_DX_D;

reg [15:0] scr_xo [0:3];
integer si;
always @(posedge clk)
    for (si = 0; si < 4; si = si + 1)
        scr_xo[si] <= scr_x[si] - {{6{layer_dx[si][9]}}, layer_dx[si]} - {{6{dx_adj[9]}}, dx_adj};

// ---------------------------------------------------------------------------
//  external tile bank table (GX-specific, 0xd44000)
//  gxv.cpp:1600 -- eight 16-bit entries, packed two per long.
// ---------------------------------------------------------------------------
reg [7:0] tilebank [0:7];
integer tb;
always @(posedge clk) begin
    if (rst) begin
        for (tb = 0; tb < 8; tb = tb + 1) tilebank[tb] <= 8'd0;
    end else if (tilebank_cs && tilebank_we && !tilebank_addr[3]) begin
        // konamigx_tilebank_w (gxv.cpp:1600) indexes the table by BYTE:
        // m_gx_tilebanks[offset*4 + lane].  Eight entries live at
        // 0xd44000-0xd44007, so the word address contributes bits [2:1] and
        // the byte lane contributes bit 0.
        //
        // MAME maps 16 bytes here and would index up to 15 into an 8-entry
        // array; addresses with bit 3 set are ignored rather than wrapped,
        // because a wrap would corrupt a live bank.  The measured trace only
        // ever writes 0xd44000 and 0xd44004 (docs/MEASUREMENTS.md), with the
        // identity values 0..7 -- so at boot the bank table is a no-op and a
        // tile code passes through unchanged.
        // The 68EC020 writes this table one BYTE at a time.  Updating both
        // entries on every 16-bit bus cycle turns the intended identity table
        // 0,1,2,3,... into duplicated odd bytes and remaps every low-bank tile
        // into unrelated graphics.  Honour the same byte strobes as the other
        // 16-bit devices on this bus.
        if (tilebank_ds[1])
            tilebank[{tilebank_addr[2:1], 1'b0}] <= tilebank_din[15:8];
        if (tilebank_ds[0])
            tilebank[{tilebank_addr[2:1], 1'b1}] <= tilebank_din[ 7:0];
    end
end

// The FSM's state and its encoding are declared HERE, above the VRAM, because
// the read-port arbitration below has to name two of the states.  Verilog
// wants a declaration before use and Verilator was willing to look ahead;
// nothing else is guaranteed to be.
localparam S_IDLE  = 3'd0, S_ADDR  = 3'd1, S_ATTR1 = 3'd2, S_ATTR2 = 3'd3,
           S_CODE1 = 3'd4, S_CODE2 = 3'd5, S_ROM   = 3'd6, S_DONE  = 3'd7;
reg [2:0] st;
// The per-line scroll read ("LINE SCROLL", above the fetch FSM) shares the one
// VRAM read port and `fetch_addr` with the fetcher, so its state is named here
// for the same reason.
localparam [1:0] LS_IDLE = 2'd0, LS_ADDR = 2'd1, LS_HOLD = 2'd2, LS_SAMP = 2'd3;
reg [1:0] ls_st;

// ---------------------------------------------------------------------------
//  VRAM -- 16 tile pages + one external linescroll page, 4096 words each.
//  The K056832 exposes page 16 when regs[0].bit1 is set.  Keeping that page
//  in the same byte-lane memories preserves the CPU readback contract.
//
//  This is the largest on-chip memory on the board (docs/MEASUREMENTS.md
//  section 4 measured all sixteen pages in use) and it is the one to watch in
//  the fitter's RAM summary.  Root section 7 / power_spikes L24: a description
//  block RAM cannot implement becomes flip-flops with no warning.  Two ports,
//  independent addresses, no same-cycle read-and-clear -- this shape infers.
// ---------------------------------------------------------------------------
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
//  This is the biggest memory on the board -- 1.05 Mbit, 103 M10K blocks --
//  so it is the one that made the build never end.
//  ---- and then MEASURED again: two reads cost TWO COPIES ------------------
//  Splitting the lanes made it infer, and the report then said 2 097 152 bits
//  for a 1 048 576-bit VRAM.  Quartus 17.0 does not implement "port A writes
//  and reads, port B reads" as one true-dual-port M10K -- it DUPLICATES the
//  memory.  Writing the two ports as two separate always blocks, which is the
//  classic template, duplicates it as well; that was tested too.  On the
//  biggest memory on the board that is 1 Mbit of the 5CSEBA6's 5.66.
//
//  So there is ONE read port and the CPU and the fetcher share it.  The
//  fetcher gets absolute priority during the two states where its read must
//  land; everywhere else -- which is most of the 128 clocks in an 8-pixel
//  group -- the CPU has it.
// 126.2 / MEASUREMENTS 127: sixteen pages, not seventeen.  The seventeenth is
// the K056832 external linescroll page; MAME allocates PAGE_COUNT+1 for it and
// konamigx never sets m_use_ext_linescroll, and **553 VRAM dumps across every
// dump directory in this lane have it entirely zero**.  Writes to it are
// dropped and reads return 0, which is what those dumps show, and the array
// loses 4 M10K per lane -- 8 of the 20 the row cache needs (126.2).
(* ramstyle = "M10K" *) reg [7:0] vram_h [0:65535];
(* ramstyle = "M10K" *) reg [7:0] vram_l [0:65535];

wire [16:0] cpu_vram_addr = ext_lnscr
                          ? { 5'b1_0000, ram_addr[12:1] }
                          : { 1'b0, cpu_bank[4:3], cpu_bank[1:0], ram_addr[12:1] };

reg  [15:0] fetch_addr;
// 128.1: the array read must be UNCONDITIONAL.  `vq_h <= raddr[16] ? 0 :
// vram_h[...]` put a mux between the array and the register, Quartus 17.0
// called it "asynchronous read logic" (Info 276007), refused to infer M10K,
// and then FAILED Analysis & Synthesis trying to build 1 Mbit of registers
// (Error 276003).  The page-16 zero moves AFTER the read instead, on a
// registered copy of the page bit, which is the same cycle and the same value.
reg  [7:0]  vq_h_r, vq_l_r;
reg         vq_hi_d;
reg  [15:0] cpu_vram_q;
reg         grant_d, cs_at_read;
reg  [16:0] addr_at_read;

// The fetcher owns the port only while a read of ITS address has to be in
// flight -- see the FSM below, where S_ATTR1 and S_CODE1 exist for exactly
// this.  Everywhere else the CPU may take it.
wire        fetch_busy = (st == S_ATTR1) || (st == S_CODE1) || (ls_st == LS_HOLD);
wire        cpu_grant  = ram_cs && !ram_we && !fetch_busy;
// The address mux is selected by `fetch_busy` alone, not by `cpu_grant`.
// The fetcher only consumes reads made in its three busy states (S_ATTR2,
// S_CODE2 and LS_SAMP take `fetch_q`), and the CPU handshake below still keys
// off `cpu_grant`, so a read of the CPU address outside both is never used.
// Measured 2026-09-15, build 9c70d73: with `cpu_grant` as the select the path
// gx_main cpu_a32 -> address decode -> ram_cs -> this mux -> M10K address was
// -1.054 ns; the decode is 4.7 ns of it.
wire [16:0] vram_raddr = fetch_busy ? {1'b0, fetch_addr} : cpu_vram_addr;

// The page-16 zero belongs on the CPU READBACK ONLY.  `vram_raddr[16]` can
// only be set by `cpu_vram_addr`, because the fetcher's address is
// {1'b0, fetch_addr} -- so the fetch path can never see page 16 and must not
// pay for the mux.  MEASURED: putting it on `fetch_q` cost the core clock
// -0.059 ns on the VRAM -> lscr_xo path, the one the .qsf already had two
// failed seeds on; taking it off is the whole fix.
wire [15:0] fetch_q = { vq_h_r, vq_l_r };

always @(posedge clk) begin
    if (ram_cs && ram_we && ram_ds[1] && !cpu_vram_addr[16]) vram_h[cpu_vram_addr[15:0]] <= ram_din[15:8];
    if (ram_cs && ram_we && ram_ds[0] && !cpu_vram_addr[16]) vram_l[cpu_vram_addr[15:0]] <= ram_din[ 7:0];

    vq_h_r  <= vram_h[vram_raddr[15:0]];
    vq_l_r  <= vram_l[vram_raddr[15:0]];
    vq_hi_d <= vram_raddr[16];

    // Sampled at the same edge as the read, so they describe the data that
    // will be standing at vq_* on the next edge.
    grant_d    <= cpu_grant;
    cs_at_read <= ram_cs;
    if (cpu_grant) addr_at_read <= cpu_vram_addr;

    if (grant_d) begin
        cpu_vram_q <= vq_hi_d ? 16'd0 : { vq_h_r, vq_l_r };
        ram_ok     <= cs_at_read;
    end

    // Last, so it always wins over the case above.
    //
    //  ---- 2026-09-08: the handshake has to carry an ADDRESS ---------------
    //  Identical defect to rtl/video/gx_palette.sv, found there first because
    //  that module got a testbench first.  `ram_ok` said "a granted CPU read
    //  completed at some point", which is not a statement about the access
    //  being asked for NOW -- and the 68EC020 asks two in a row without ever
    //  dropping the select, because TG68K holds busstate = "10" across both
    //  halves of a long (TG68KdotC_Kernel.vhd:1185, :447, :450, :1082, :1166;
    //  the derivation is written out in gx_palette.sv).
    //
    //  This one is INTERMITTENT where the palette's was total, and that is
    //  worse rather than better.  `cpu_grant` is high on most clocks, so the
    //  read usually re-issues at the new address in time; it only goes wrong
    //  when `fetch_busy` holds the port during the one or two clocks after the
    //  address changes -- which is exactly when the picture is being drawn.
    //  It would have looked like a rare CPU read of the wrong VRAM word.
    //
    //  No deadlock: fetch_busy is two transient states of the FSM, so a held
    //  address always gets a granted read, and the handshake follows one clock
    //  later.
    if (!ram_cs || cpu_vram_addr !== addr_at_read) ram_ok <= 1'b0;
end

initial begin
    grant_d      = 1'b0;
    ram_ok       = 1'b0;
    addr_at_read = 17'd0;
end

assign ram_dout = cpu_vram_q;

// ---------------------------------------------------------------------------
//  address generation, one layer at a time
//
//  vx = scroll_x + screen_x, vy = scroll_y + screen_y, wrapped to the layer's
//  virtual map of (w+1) x 512 by (h+1) x 256 pixels.
//
//  The page column is (vx >> 9) modulo the span.  Spans of 1, 2 and 4 are
//  masks; a span of 3 needs a real modulo, which is why `mod3` exists.  It is
//  eight bits wide, so it costs almost nothing, and implementing it properly
//  avoids a special case that would otherwise silently mis-render a
//  three-page layer.  gokuparo uses span 1 everywhere in the measured trace,
//  so this path is unexercised -- but it is correct rather than absent.
// ---------------------------------------------------------------------------
//  v % 3 WITHOUT A DIVIDER.
//
//  It used to be `2'(v % 8'd3)`, and Quartus builds that as a remainder
//  circuit.  MEASURED, build I, 2026-09-07 -- the five worst setup paths in
//  the whole design ran through it:
//
//      gx_tilemap|lyr[1] -> gx_tilemap|fetch_addr[13]     slack -5.873
//
//  and it is on BOTH page_col and page_row.  The sting is that gokuparo uses
//  span 1 everywhere in the measured trace, so this is a divider on the
//  critical path of a branch this game never takes.
//
//  4 is congruent to 1 mod 3, so a number is congruent to the sum of its
//  base-4 digits.  Two reductions take 8 bits down to 0..5 and a case
//  finishes it: three small adders instead of a divider.
//
//  The zero-extensions are for the reader, not for correctness -- and that
//  sentence used to say the opposite.  Verilog sizes an addition by its
//  ASSIGNMENT CONTEXT, so `s1 = v[7:6] + v[5:4] + ...` with `s1` four bits
//  wide is already evaluated in four bits and does not truncate.  The claim
//  that it did was written here and was wrong; it was caught by trying to
//  break the function that way and finding the check below stayed silent,
//  because there was nothing to catch.
//
//  Checked exhaustively against the operator it replaces, in the module
//  itself rather than in a copy -- see the `initial` block below.  That check
//  is armed: mapping case 1/4 to the wrong residue makes it fail 85 of 256
//  and abort the simulation.
function automatic [1:0] mod3(input [7:0] v);
    reg [3:0] s1;
    reg [2:0] s2;
begin
    s1 = {2'd0, v[7:6]} + {2'd0, v[5:4]} + {2'd0, v[3:2]} + {2'd0, v[1:0]};
    s2 = {1'd0, s1[3:2]} + {1'd0, s1[1:0]};
    case (s2)
        3'd0, 3'd3, 3'd6: mod3 = 2'd0;
        3'd1, 3'd4:       mod3 = 2'd1;
        default:          mod3 = 2'd2;   // 2, 5
    endcase
end
endfunction

`ifndef SYNTHESIS
// Exhaustive, on the real function, in every simulation that elaborates this
// module.  256 inputs is nothing to check and the alternative is trusting a
// hand-rolled congruence on a branch no testbench exercises.
initial begin : mod3_exhaustive
    integer v;
    integer bad;
    bad = 0;
    for (v = 0; v < 256; v = v + 1)
        if (mod3(v[7:0]) !== 2'(v % 3)) bad = bad + 1;
    if (bad != 0) $fatal(1, "gx_tilemap: mod3 wrong for %0d of 256 inputs", bad);
end
`endif

function automatic [1:0] mod_span(input [7:0] v, input [1:0] span_m1);
begin
    case (span_m1)
        2'd0: mod_span = 2'd0;              // span 1
        2'd1: mod_span = {1'b0, v[0]};      // span 2
        2'd3: mod_span = v[1:0];            // span 4
        default: mod_span = mod3(v);        // span 3
    endcase
end
endfunction

// ---------------------------------------------------------------------------
//  fetch pipeline
//
//  One 8-pixel group per layer is fetched ahead of the beam.  At a 6 MHz dot
//  clock and a ~96 MHz system clock there are 16 system cycles per pixel, so
//  128 per group -- and the work is 4 layers x (1 VRAM read + 1 ROM read),
//  which fits with an order of magnitude to spare.  Sequencing them rather
//  than building four parallel fetchers keeps one ROM port and one VRAM port.
//
//  Sequence per group, repeated for layer 0..3:
//     S_ADDR  compute vx/vy, present the VRAM address for the attribute word
//     S_ATTR  latch attr, present the address for the code word
//     S_CODE  latch code, apply the external bank table, issue the ROM read
//     S_ROM   wait for rom_ok, unpack 8 pixels into the layer's shift register
// ---------------------------------------------------------------------------
//  ---- the read pipeline was one state short, and it was a real bug --------
//  As written before 2026-09-07 this FSM presented an address and sampled
//  `fetch_q` on the NEXT state -- but `fetch_q` is registered from the address
//  that was standing during the state BEFORE that, so `attr_w` was loaded with
//  whatever the previous fetch left behind and `code_w` was loaded with the
//  attribute word.  Every tile would have been wrong, and it would have looked
//  like a VRAM addressing fault.
//
//  Nothing caught it because there is no testbench on this path: the module
//  was lint-only.  So each read now takes two states -- one to hold the
//  address while the RAM is read, one to sample -- and those are also exactly
//  the states in which the fetcher must own the read port.

reg [1:0] lyr;

reg [15:0] attr_w, code_w;

// per-layer 8-pixel staging.
//
// TWO sets, and the reason is not tidiness.  The four layers are fetched
// SEQUENTIALLY, so layer D's group lands several cycles after layer A's.  If
// each shift register were loaded as its own fetch completed, the four layers
// would be shifted out with different phases and the picture would tear
// between layers by up to a few pixels -- a defect that looks like a scroll
// bug and would be hunted in the scroll registers.
//
// So fetches land in `stage_*` and all four are transferred to `shift`/`cur_*`
// together, at the group boundary.
//
// GQ: TWO banks of it, indexed {bank, layer}.  The fetch in progress writes
// bank `wb`; the group edge moves bank `eb` (the edge parity) out.  A group is
// assigned the bank of the edge it is launched for and read out two edges
// later, which has the same parity.  With GQ = 0 both are always bank 0.
reg [63:0] stage    [0:7];   // 8 pixels x 8 bits
reg [7:0]  stage_col[0:7];
reg [1:0]  stage_mix[0:7];
reg [2:0]  stage_f  [0:7];   // vx[2:0] of the fetch: the sub-tile scroll
reg        wb;               // bank the running fetch writes
reg        eb;               // bank the next group edge moves out
reg  [1:0] b_need;           // a group was assigned to this bank
reg  [1:0] b_done;           // ... and its fetch has completed
reg        lp_v;             // GQ: a group launch is waiting for the FSM
reg        lp_b;
reg  [9:0] lp_x;
reg  [8:0] lp_y;
wire [2:0] wsl = {wb, lyr};  // stage slot of the layer being fetched

//  ---- sub-tile X scroll: a two-group window per layer, 2026-09-14 ---------
//  Until afa0806 a group was ONE tile's eight pixels shifted out against the
//  8-dot group grid, and vx[2:0] -- the part of the X scroll below one tile
//  -- was never read.  Every layer snapped to that grid and sat
//  (SCREEN_X_OFFSET + scroll) mod 8 dots to the right of where it belongs.
//
//  MEASURED, afa0806 on the board against MAME's memory rebuilt into pixels:
//  layers right by A 9, B 3, C 5, D 6.  The game's resting scrolls are -26,
//  -24, -22, -21, and (24 + scroll) mod 8 is 6, 0, 2, 3 -- those four
//  numbers less a uniform 3, which is the colour pipeline's latency (gx_top,
//  "Video output timing").  tb_gx_tilemap section 11 on the old RTL: marker
//  dots 168/160/160/160 where 162/160/158/157 are correct.
//
//  B, C and D had LOOKED like MAME because the game's scrolls are -24 plus
//  MAME's per-layer offsets (-2, 0, +2, +3): the dropped bits happened to
//  cancel those offsets for the three layers where they are non-negative.
//
//  A dot can now need either of two tiles, and the fetch budget has room for
//  one per layer per group.  So the fetcher leads one group further (+17)
//  and the output keeps the last TWO groups.  With f = vx[2:0] and j the dot
//  within the group:
//
//      vx = 8*T + f + j   ->  tile T,   pixel f+j,    while f+j < 8   (prv)
//                             tile T+1, pixel f+j-8,  otherwise      (cur)
//
//  The FSM, the 128-clock group budget and the all-layers-together transfer
//  are unchanged.  One more group per line is fetched: 37, c = 0 .. 288.
reg [63:0] cur_px [0:3];     // 8 pixels x 8 bits, the group loaded last edge
reg [7:0]  cur_col[0:3];
reg [1:0]  cur_mix[0:3];
reg [2:0]  cur_f  [0:3];
reg [63:0] prv_px [0:3];     // the group before it
reg [7:0]  prv_col[0:3];
reg [1:0]  prv_mix[0:3];
reg [2:0]  prv_f  [0:3];
reg [3:0]  cur_sel[0:3];     // f + j for the dot standing now: <8 prv, else cur

// ---------------------------------------------------------------------------
//  pixel position of the group being fetched: one group ahead of the beam
//
//  The destination is prepared continuously with lead +9 and snapshotted on
//  the group edge.  This is exactly the same coordinate the old S_ADDR
//  expression obtained one clock later as `hcnt + 8`: hcnt advances by one
//  dot on that group edge.
//  Holding the coordinate for all four sequential layer fetches also states
//  the intended contract directly instead of relying on hcnt not reaching its
//  next pixel enable while the FSM is working.
//
//  THE WRAP IS A BUG FIX, 2026-09-08.  This used to be a plain `hcnt + 8`.
//  For every group but one that is right.  The exception is the group that
//  serves dots 0..7 of a line: it is started at hcnt = htotal-9 and sampled at
//  htotal-8, where the old sum ran to `htotal` instead of wrapping to 0 --
//  addressing tile 48 of the PREVIOUS row for the leftmost eight pixels of
//  every single line.  Nobody could see it because the screen has been black
//  since before the fetcher existed.
//
//  It has to be fixed HERE and not merely worked around, because the blanking
//  gate below asks "is this group's destination visible" and the answer for
//  that group is yes.  A gate that trusted the unwrapped sum would have turned
//  eight wrong pixels into eight missing ones.
//
//  THE REGISTER IS A TIMING FIX, 2026-09-12.  Two different diagnostic
//  netlists failed at -0.208 and -0.201 ns on the same path family:
//
//      gx_ccu|h -> gx_tilemap|fetch_addr
//
//  The old path crossed the module boundary, added/wrapped h, added scroll,
//  decoded page span and finally drove an M10K address register in one cycle.
//  Snapshotting the group coordinate split that path and produced a passing
//  +0.783 ns build, but the next diagnostic netlist exposed the remaining
//  predecode path at -0.490 ns:
//
//      gx_ccu|h -> gx_tilemap|st.S_ADDR / fetch_x_q
//
//  `hcnt` changes only on `pxl_cen`, once every 16 system clocks.  The
//  sum, wrap and visibility predicate are therefore prepared in three small
//  registered stages during those otherwise idle clocks and are stable long
//  before an eight-pixel group edge consumes them.  This adds no FSM state and
//  changes neither the group cadence nor the 128-clock group budget.  Check 6
//  in tb_gx_tilemap distinguishes the wrapped left-edge address from the old
//  column-48 failure; check 7 keeps the exact 32,256-request frame census.
// ---------------------------------------------------------------------------
reg  [10:0] gx_raw_q;
reg  [9:0]  fy_next_q;
reg  [9:0]  grp_x_q;
reg  [8:0]  grp_y_q;
reg         group_needed_q;

reg  [9:0]  fetch_x_q;
reg  [8:0]  fetch_y_q;

localparam [9:0] RASTER_X_OFFSET = SCREEN_X_OFFSET;
localparam [8:0] RASTER_Y_OFFSET = SCREEN_Y_OFFSET;

// ---------------------------------------------------------------------------
//  LINE SCROLL, 2026-09-15 (MEASUREMENTS 46, R3)
//
//  MEASURED: the sea stage (MAME frames 14220-15680) writes word 5 = 0x00CF,
//  layer C in mode 0, and its table on scrollbank page 15 holds six different
//  X scrolls down the screen -- the horizon and water band.  Until this block
//  every layer drew with the register scroll, and the board's band matched the
//  model WITHOUT line scroll.  No other layer and no other mode appears in any
//  dumped attract frame (dist/demodump, dist/objdump, dist/gamedump).
//
//  WHAT MAME DOES, k054156_k054157_k056832.cpp tilemap_draw_common:
//    scrollbank = ((word 0x18 >> 1) & 0xc) | (word 0x18 & 3)          :1330
//    mode       = word 5 >> (2 * layer) & 3                            :1331
//    table      = VRAM page scrollbank, word offset layer << 10        :1381, :377
//    entry n    = words 2n (high) and 2n+1 (low); dx = entry + corr    :1579
//                 and corr = -set_layer_offs x, as for the register    :1370
//    n for screen line y (bitmap y, the visible row + 16):
//                 (y + scroll_y) & 0x1ff            mode 0 (per line)  :1499, :1516-1517, :1570
//                 (y + scroll_y) & 0x1f8            mode 2 (per 8)     :1454-1458, :1511-1512
//    modes 1 and 3 draw the register scroll                            :1392
//  "Source-oriented" (the header, :154): the entry follows the tilemap row,
//  not the screen row.  The table's high word only reaches bits above 15 of dx
//  and a layer is at most 2,048 dots wide, so only the low word is read.
//  (For a 3-page-tall layer MAME's `(unsigned)dy % 768` differs from this for a
//  negative scroll; gokuparo uses 2 pages, and span 3 is the same approximation
//  the tile address already makes -- mod3 above.)  jt05415x's silicon netlist
//  has the line-scroll latch control (jt054156.v:782-813) but no data path to
//  borrow, so MAME's arithmetic is the source.  EMULATION_DERIVED.
//
//  WHEN IT IS READ.  A mode-0/2 layer's scroll changes only between lines, and
//  the first group of line N+1 is fetched at hcnt = htotal - 17 of line N.  So
//  once per line, at hcnt = hres + 32 -- after the last group of the line has
//  started (hcnt = hres - 17) and 47 dots before the next line's first one --
//  one VRAM word per such layer is read for line N+1 (`fy_next_q`) through the
//  fetcher's own port: address, hold (the CPU is locked out, `fetch_busy`),
//  sample.  At most 12 clocks a line, no group can start meanwhile, none is in
//  flight when it starts, and the SDRAM is not touched -- the tile group budget
//  (DECISIONS D2) does not change.  No new memory: VRAM is read through the
//  port it already has.
//
//  THE FETCH ADDRESS PATH GAINS NOTHING: `scr_eff` is a register that holds the
//  register scroll or the line's table scroll per layer, and vx_raw reads it
//  where it read `scr_xo`.
// ---------------------------------------------------------------------------
wire [1:0]  ls_mode [0:3];
assign ls_mode[0] = regs[5'h05][1:0];
assign ls_mode[1] = regs[5'h05][3:2];
assign ls_mode[2] = regs[5'h05][5:4];
assign ls_mode[3] = regs[5'h05][7:6];

reg  [1:0]  ls_l;
reg         ls_pend;
reg  [15:0] lscr_xo [0:3];
reg  [15:0] scr_eff [0:3];
// The line-scroll word, registered straight off the VRAM before the layer_dx
// subtraction (MEASUREMENTS 143).  `ls_wr` / `ls_wl` say which lscr_xo it
// becomes, one clock after LS_SAMP.
reg  [15:0] ls_raw;
reg  [1:0]  ls_wl;
reg         ls_wr;

wire        ls_on   = (ls_mode[ls_l] == 2'd0) || (ls_mode[ls_l] == 2'd2);
wire [3:0]  ls_bank = {regs[5'h18][4:3], regs[5'h18][1:0]};
wire [9:0]  ls_sum  = fy_next_q + {1'b0, RASTER_Y_OFFSET} + {1'b0, scr_y[ls_l][8:0]};
wire [8:0]  ls_line = (ls_mode[ls_l] == 2'd2) ? {ls_sum[8:3], 3'b000} : ls_sum[8:0];
wire [15:0] ls_addr = {ls_bank, ls_l, ls_line, 1'b1};

integer sj;
always @(posedge clk)
    for (sj = 0; sj < 4; sj = sj + 1)
        scr_eff[sj] <= ((ls_mode[sj] == 2'd0) || (ls_mode[sj] == 2'd2)) ? lscr_xo[sj] : scr_xo[sj];

wire [16:0] vy_raw = {1'b0, scr_y[lyr]} + {8'd0, fetch_y_q};

wire [16:0] vx_raw = {1'b0, scr_eff[lyr]} + {7'd0, fetch_x_q};

wire [1:0] page_col = mod_span(vx_raw[16:9], lyr_w[lyr]);
wire [1:0] page_row = mod_span(vy_raw[15:8], lyr_h[lyr]);

wire [1:0] page_x = lyr_x[lyr] + page_col;
wire [1:0] page_y = lyr_y[lyr] + page_row;
wire [3:0] page   = {page_y, page_x};

wire [5:0] tile_x = vx_raw[8:3];
wire [4:0] tile_y = vy_raw[7:3];
wire [2:0] sub_y  = vy_raw[2:0];

wire [15:0] vram_attr_addr = {page, tile_y, tile_x, 1'b0};
wire [15:0] vram_code_addr = {page, tile_y, tile_x, 1'b1};

// attribute decode.  k056832_shiftmasks, one set per `fbits`.
reg [5:0] pal_m1;   reg [2:0] pal_s2;   reg [5:0] pal_m2;   reg [2:0] flip_s;
always @(*) begin
    case (fbits)
        2'd0: begin flip_s = 3'd6; pal_m1 = 6'h3f; pal_s2 = 3'd0; pal_m2 = 6'h00; end
        2'd1: begin flip_s = 3'd4; pal_m1 = 6'h0f; pal_s2 = 3'd2; pal_m2 = 6'h30; end
        2'd2: begin flip_s = 3'd2; pal_m1 = 6'h03; pal_s2 = 3'd2; pal_m2 = 6'h3c; end
        2'd3: begin flip_s = 3'd0; pal_m1 = 6'h00; pal_s2 = 3'd2; pal_m2 = 6'h3f; end
    endcase
end

wire [15:0] attr_flip_sh = attr_w >> flip_s;
wire [15:0] attr_pal_sh  = attr_w >> pal_s2;
// Word 1 (MAME m_regs[1], :618, "tile-flip override ... REG2") holds two
// tile-flip enable bits per layer.  A tile attribute can flip an axis only
// when the corresponding layer bit is enabled.  gokuparo writes 0x00FF.
wire [7:0]  flip_allow_q = regs[5'h01][7:0] >> {lyr, 1'b0};
wire [1:0]  tile_flip    = attr_flip_sh[1:0] & flip_allow_q[1:0];
wire [5:0]  tile_pal     = (attr_w[5:0] & pal_m1) | (attr_pal_sh[5:0] & pal_m2);

// GX external bank table, gxv.cpp:1036
//
//  ---- THE SAME OFF-BY-ONE-STATE BUG, ONE STATE FURTHER DOWN ---------------
//  The header above describes a read pipeline that was one state short and was
//  fixed on 2026-09-07.  It was fixed for `attr_w` and `code_w`.  It was NOT
//  fixed here: `rom4_addr <= tile_index` runs in S_CODE2, the same clock as
//  `code_w <= fetch_q`, so a `tile_index` built from the REGISTER carried the
//  code of the PREVIOUS fetch -- which, because the fetcher walks A, B, C, D
//  through one port, is the previous layer's tile, and for layer A the
//  previous group's layer D.  Every layer drew somebody else's tile.
//
//  Nothing caught it for the same reason nothing caught the first half: the
//  testbench filled every tile in the row with the SAME code, so a one-fetch
//  lag is invisible.  It was found on 2026-09-08 by refilling the page with
//  `code = 0x1000 + tile_x` -- a fill in which each tile NAMES ITS OWN COLUMN,
//  so the ROM address says which tile was addressed instead of only whether
//  it matched.  sim/tb_gx_tilemap.sv check 6.
//
//  So the address is built from `fetch_q`, the word arriving on this clock,
//  and `code_w` keeps its job of recording what was latched -- which is what
//  `dbg_a_hit[2]` reads one state later.
wire [15:0] code_now  = fetch_q;
wire [7:0]  bank_sel  = tilebank[code_now[15:13]];
wire [20:0] tile_code = {bank_sel, code_now[12:0]};

// row within the tile, honouring per-tile and global V flip
wire [2:0] row = (tile_flip[1] ^ (glob_vflip)) ? ~sub_y : sub_y;

// tile ROM index: 8x8 tiles, one 8-pixel row per index.  gx_rommap.svh.
wire [23:0] tile_index = {tile_code, row};

// ---------------------------------------------------------------------------
//  Do not fetch tiles for dots that are never displayed
//
//  MEASURED, docs/DEBUG_LOG and STATUS 2026-09-08 section 11: the fetcher used
//  to start a group every 8 dots unconditionally, across the whole 384x264
//  raster, while only 288x224 of it is displayed.
//
//      displayed   288 x 224 =  64,512 dots     36 groups x 224 lines
//      raster      384 x 264 = 101,376 dots     48 groups x 264 lines
//
//  36.4 % of this module's SDRAM bandwidth -- about 355,000 system clocks a
//  frame -- was spent on pixels that leave through the blanking.  That is
//  larger than the entire sound CPU that is about to become arbiter client 3
//  (docs/DECISIONS.md D10), which is why it is recovered first: otherwise the
//  first sound build would be measured against a bus already wasting more
//  than the thing being added.
//
//  THE PREDICATE IS ON THE GROUP'S DESTINATION, NOT ON THE BEAM.  Gating on
//  `hblank` would be wrong in both directions -- it would kill the two groups
//  inside horizontal blanking that feed the start of the next line, and it
//  would keep fetching on blanked LINES, which is 40 of 264.  So the question
//  asked is the only one that matters: when these eight pixels reach the
//  shifter, will anything be looking?
//
//  The +9 destination is prepared ahead of the group edge and then copied to
//  the fetch snapshot.  This is the same coordinate the former combinational
//  +8 path saw after `hcnt` advanced by one dot.
// ---------------------------------------------------------------------------
integer lsi;
always @(posedge clk) begin
    if (rst) begin
        st             <= S_IDLE;
        ls_st          <= LS_IDLE;
        ls_l           <= 2'd0;
        ls_pend        <= 1'b0;
        ls_wr          <= 1'b0;
        for (lsi = 0; lsi < 4; lsi = lsi + 1) lscr_xo[lsi] <= 16'd0;
        lyr            <= 2'd0;
        rom_req        <= 1'b0;
        gx_raw_q       <= 11'd0;
        fy_next_q      <= 10'd0;
        grp_x_q        <= 10'd0;
        grp_y_q        <= 9'd0;
        group_needed_q <= 1'b0;
        fetch_x_q      <= 10'd0;
        fetch_y_q      <= 9'd0;
        group_done     <= 1'b0;
        dbg_late       <= 1'b0;
        wb             <= 1'b0;
        eb             <= 1'b0;
        b_need         <= 2'b00;
        b_done         <= 2'b00;
        lp_v           <= 1'b0;
    end else begin
        group_done <= 1'b0;
        // GQ: late means the bank this edge moves out was assigned a group
        // whose fetch has not completed -- not merely "the FSM is busy", which
        // the queue now absorbs.
        dbg_late <= GQ ? (pxl_cen && hcnt[2:0] == 3'd7 && b_need[eb] && !b_done[eb])
                       : (pxl_cen && hcnt[2:0] == 3'd7 && group_needed_q &&
                          st != S_IDLE);

        // Three-stage raster predecode.  hcnt/vcnt are stable for 16 clocks
        // between pixel enables, so these continuously running stages settle
        // well before the next group edge uses them.
        //
        // +17, not +9, since 2026-09-14: a group fetched now is shown across
        // the NEXT two groups, as the right and then the left half of the
        // output window ("sub-tile X scroll", at the staging registers).  At
        // the group edge hcnt = c - 17, so the destination is c.
        //
        // +25 with GQ: one group further again, because the queue shows a
        // group two edges after it is launched instead of one.
        gx_raw_q  <= {1'b0, hcnt} + (GQ ? 11'd25 : 11'd17);
        fy_next_q <= (({1'b0, vcnt} + 10'd1) == vtotal) ? 10'd0
                                                        : ({1'b0, vcnt} + 10'd1);

        if (gx_raw_q >= {1'b0, htotal}) begin
            grp_x_q <= gx_raw_q[9:0] - htotal;
            grp_y_q <= fy_next_q[8:0];
        end else begin
            grp_x_q <= gx_raw_q[9:0];
            grp_y_q <= vcnt;
        end

        // `<=`: 37 groups a line, c = 0 .. hres.  With a non-zero sub-tile
        // scroll the last visible dots show the first pixels of the tile at
        // c = hres, so that group is displayed too.
        group_needed_q <= (grp_x_q <= hres) && ({1'b0, grp_y_q} < vres);

        // ---- the per-line scroll read, "LINE SCROLL" above ------------------
        //  Starts only while no group is in flight and never on a group edge,
        //  and a group cannot start while it runs, so `fetch_addr` has one
        //  owner at a time.  A layer not in mode 0/2 is skipped in one clock.
        case (ls_st)
            LS_IDLE: if (ls_pend && st == S_IDLE && !lp_v && !(pxl_cen && hcnt[2:0] == 3'd7)) begin
                ls_pend <= 1'b0;
                ls_l    <= 2'd0;
                ls_st   <= LS_ADDR;
            end
            LS_ADDR: if (ls_on) begin
                fetch_addr <= ls_addr;
                ls_st      <= LS_HOLD;
            end else if (ls_l == 2'd3)
                ls_st <= LS_IDLE;
            else
                ls_l  <= ls_l + 2'd1;
            // Hold the address while the RAM reads it (`fetch_busy`).
            LS_HOLD: ls_st <= LS_SAMP;
            //  ---- one register between the VRAM and the subtraction -------
            //  MEASURED, build of 747b7eb7: vram_h ... PORT_B_WRITE_ENABLE_REG
            //  -> lscr_xo[0][15], slack -0.037 ns at 96 MHz -- the THIRD fit
            //  on which a gx_tilemap VRAM path has failed (seeds 12 and 1
            //  before it, KonamiGX.qsf).  VRAM is 136 M10K spread over the die
            //  and this path added a 16-bit subtract after the far read.  The
            //  .qsf says to stop reseeding and add a stage.  lscr_xo is filled
            //  in horizontal blanking and read on the next line, so one clock
            //  later changes nothing it feeds; tb_gx_lscroll 15560 identical.
            LS_SAMP: begin
                ls_raw <= fetch_q;
                ls_wl  <= ls_l;
                if (ls_l == 2'd3)
                    ls_st <= LS_IDLE;
                else begin
                    ls_l  <= ls_l + 2'd1;
                    ls_st <= LS_ADDR;
                end
            end
            default: ls_st <= LS_IDLE;
        endcase
        ls_wr <= (ls_st == LS_SAMP);
        if (ls_wr)
            lscr_xo[ls_wl] <= ls_raw - {{6{layer_dx[ls_wl][9]}}, layer_dx[ls_wl]} - {{6{dx_adj[9]}}, dx_adj};
        // After the case, so a request for the next line is never lost to a
        // start in the same clock.
        if (pxl_cen && hcnt == hres + 10'd32) ls_pend <= 1'b1;

        // ---- GQ: bank bookkeeping and the held launch ------------------------
        //  At every group edge the bank `eb` is moved out (below, at the
        //  staging transfer) and reassigned to the group this edge launches.
        //  A launch that finds the FSM busy is held in lp_* and started as
        //  soon as S_IDLE comes round; a second edge while one is still held
        //  overwrites it -- that group was going to be late anyway, and its
        //  bank reads as not done.
        if (GQ) begin
            if (st == S_DONE && lyr == 2'd3) b_done[wb] <= 1'b1;
            if (pxl_cen && hcnt[2:0] == 3'd7) begin
                eb         <= ~eb;
                b_need[eb] <= group_needed_q;
                b_done[eb] <= 1'b0;
                if (group_needed_q &&
                    (lp_v || st != S_IDLE || ls_st != LS_IDLE)) begin
                    lp_v <= 1'b1;
                    lp_b <= eb;
                    lp_x <= grp_x_q;
                    lp_y <= grp_y_q;
                end
            end
        end

        case (st)
            S_IDLE: begin
                // Start a group every 8 pixels -- but only if the eight pixels
                // it would produce are ever displayed.  `group_needed_q` above.
                // Not while the line-scroll read owns the VRAM port.
                if (GQ && lp_v && ls_st == LS_IDLE) begin
                    // a held launch, with the coordinate and bank of its edge;
                    // an edge on this same clock re-fills lp_* above
                    if (!(pxl_cen && hcnt[2:0] == 3'd7 && group_needed_q))
                        lp_v <= 1'b0;
                    fetch_x_q <= lp_x + RASTER_X_OFFSET;
                    fetch_y_q <= lp_y + RASTER_Y_OFFSET;
                    wb        <= lp_b;
                    lyr       <= 2'd0;
                    st        <= S_ADDR;
                end else if (pxl_cen && hcnt[2:0] == 3'd7 && group_needed_q && ls_st == LS_IDLE) begin
                    wb        <= GQ ? eb : 1'b0;
                    // Capture the SAME destination that passed group_needed_q.
                    // S_ADDR sees these registered values one clock later.
                    // The beam counters are the 0-based active-window
                    // coordinates used for output timing.  Tile addresses,
                    // however, are in the full CCU raster; gokuparo's
                    // visible window begins at (24,16), so map the fetched
                    // group into that coordinate system here.
                    fetch_x_q <= grp_x_q + RASTER_X_OFFSET;
                    fetch_y_q <= grp_y_q + RASTER_Y_OFFSET;
                    lyr       <= 2'd0;
                    st        <= S_ADDR;
                end
            end

            S_ADDR: begin
                fetch_addr   <= vram_attr_addr;
                // The group coordinate is a multiple of 8, so vx[2:0] is this
                // layer's sub-tile scroll, the same for every dot of the group.
                stage_f[wsl] <= vx_raw[2:0];
                st           <= S_ATTR1;
            end

            // Hold the address while the RAM reads it.  The fetcher owns the
            // port in this state (see `fetch_busy`).
            S_ATTR1: st <= S_ATTR2;

            S_ATTR2: begin
                attr_w     <= fetch_q;
                fetch_addr <= vram_code_addr;
                st         <= S_CODE1;
            end

            S_CODE1: st <= S_CODE2;

            S_CODE2: begin
                code_w    <= fetch_q;
                rom4_addr <= tile_index[19:0];
                rom1_addr <= tile_index[19:0];
                rom_req   <= 1'b1;
                st        <= S_ROM;
            end

            S_ROM: begin
                if (rom_ok) begin
                    rom_req <= 1'b0;
                    st      <= S_DONE;
                end
            end

            S_DONE: begin
                stage_col[wsl] <= {2'b00, tile_pal};
                stage_mix[wsl] <= attr_w[7:6]; // alpha_tile_callback, gxv.cpp:1059
                if (lyr == 2'd3) begin
                    group_done <= 1'b1;
                    st         <= S_IDLE;
                end
                else begin
                    lyr <= lyr + 2'd1;
                    st  <= S_ADDR;
                end
            end

            default: st <= S_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------------
//  pixel unpack and output
//
//  Plane order for tiles is charlayout5 { 32, 24, 8, 16, 0 } -- note planes 2
//  and 3 are swapped relative to sprites.  gx_rommap.svh derives this and
//  provides the macro; it is spelled out here rather than included so that a
//  reader of this file can see it.
//
//      px[4] = rom1[7-n]      px[3] = rom4[31-n]   px[2] = rom4[15-n]
//      px[1] = rom4[23-n]     px[0] = rom4[7-n]
//
//  Bit 7 of each plane byte is the leftmost pixel (MAME's readbit is
//  MSB-first), so no swizzle is needed.
// ---------------------------------------------------------------------------
wire hflip_eff = tile_flip[0] ^ glob_hflip;
// GQ: a held launch counts as busy, so no soft client starts in the one
// clock between two chained fetches (gx_top's gates key off this).
assign group_busy = (st != S_IDLE) || lp_v;

integer px, L;
wire group_edge = pxl_cen && (hcnt[2:0] == 3'd7);

always @(posedge clk) begin
    // a completed fetch lands in the staging register for its own layer
    if (st == S_DONE) begin
        for (px = 0; px < 8; px = px + 1) begin
            // charlayout6 {40,32,24,8,16,0} is charlayout5 with byte 5 on top.
            // charlayout8 {56,24,40,8,48,16,32,0} (k054156_k054157_k056832.cpp:234):
            // pen bits 7..0 from bytes 7,3,5,1,6,2,4,0; bytes 4-7 are rom1's.
            if (bpp8)
                stage[wsl][px*8 +: 8] <= hflip_eff
                    ? { rom1_data[24+px],    rom4_data[24+px],    rom1_data[8+px],
                        rom4_data[8+px],     rom1_data[16+px],    rom4_data[16+px],
                        rom1_data[px],       rom4_data[px] }
                    : { rom1_data[31-px],    rom4_data[31-px],    rom1_data[15-px],
                        rom4_data[15-px],    rom1_data[23-px],    rom4_data[23-px],
                        rom1_data[7-px],     rom4_data[7-px] };
            else
                stage[wsl][px*8 +: 8] <= hflip_eff
                    ? { 2'b00,
                        rom1_data[8+px],     rom1_data[px],       rom4_data[24+px],
                        rom4_data[8+px],     rom4_data[16+px],    rom4_data[px] }
                    : { 2'b00,
                        rom1_data[15-px],    rom1_data[7-px],     rom4_data[31-px],
                        rom4_data[15-px],    rom4_data[23-px],    rom4_data[7-px] };
        end
    end

    if (group_edge) begin
        // all four layers move their window on the same dot
        for (L = 0; L < 4; L = L + 1) begin
            prv_px[L]  <= cur_px[L];
            prv_col[L] <= cur_col[L];
            prv_mix[L] <= cur_mix[L];
            prv_f[L]   <= cur_f[L];
            cur_px[L]  <= stage[{eb, L[1:0]}];
            cur_col[L] <= stage_col[{eb, L[1:0]}];
            cur_mix[L] <= stage_mix[{eb, L[1:0]}];
            cur_f[L]   <= stage_f[{eb, L[1:0]}];
        end
    end

    // The select for the dot that begins at this enable, so the output mux
    // reads only pxl_cen-loaded registers (KonamiGX.sdc relies on that).  f
    // is the LEFT group's -- after a group edge that is today's cur -- and
    // the dot index wraps with hcnt: the group edge is hcnt[2:0] == 7.
    if (pxl_cen)
        for (L = 0; L < 4; L = L + 1)
            cur_sel[L] <= {1'b0, group_edge ? cur_f[L] : prv_f[L]}
                        + {1'b0, hcnt[2:0] + 3'd1};
end

wire [7:0] win_px [0:3];
wire [7:0] win_col[0:3];
wire [1:0] win_mix[0:3];
genvar w;
generate
for (w = 0; w < 4; w = w + 1) begin : win
    assign win_px[w]  = cur_sel[w][3] ? cur_px[w][cur_sel[w][2:0]*8 +: 8]
                                      : prv_px[w][cur_sel[w][2:0]*8 +: 8];
    assign win_col[w] = cur_sel[w][3] ? cur_col[w] : prv_col[w];
    assign win_mix[w] = cur_sel[w][3] ? cur_mix[w] : prv_mix[w];
end
endgenerate

assign pxl_a = win_px[0];
assign pxl_b = win_px[1];
assign pxl_c = win_px[2];
assign pxl_d = win_px[3];

assign col_a = win_col[0];  assign col_b = win_col[1];
assign col_c = win_col[2];  assign col_d = win_col[3];
assign mix_a = win_mix[0];  assign mix_b = win_mix[1];
assign mix_c = win_mix[2];  assign mix_d = win_mix[3];

// ---------------------------------------------------------------------------
//  LAYER A instrumentation -- see the port comment for why it is in here
//
//  WHY 4bpp AND 1bpp ARE ASKED SEPARATELY.  A tile pixel is five bits: four
//  planes out of GX_TILE4 and a fifth out of GX_TILE1 (the unpack directly
//  above).  Either half being zero still leaves a NON-transparent pixel;
//  `pxl_a` is zero only when BOTH are.  The previous build ORed the two
//  buses into one rung, which cannot distinguish them -- and they are fetched
//  from two different ROM regions with two different address units, so they
//  are two independent ways to be wrong.
//
//  Sampled at S_DONE, the same state in which the unpack above reads the same
//  two buses, so "non-zero" is a statement about the data the pixels were
//  actually built from and not about some other clock.
//
//  TIMING.  The wide compare is a reduction of `rom4_data`, which is already
//  routed into this module and already read combinationally by the unpack, so
//  it adds no new long path; its result goes straight into a flip-flop.  This
//  project has spent two builds on a diagnostic that became the critical
//  path, so that sentence is load-bearing rather than decorative.
// ---------------------------------------------------------------------------
//  Layer A's own geometry and scroll registers, from the named accessors
//  above: word 0x08 vertical grid, 0x0c horizontal grid, 0x10 Y scroll,
//  0x14 X scroll.  Asked as "was it WRITTEN", not "is it non-zero": the
//  measured trace writes 0x00 to both of layer A's grid words, so a value
//  test would read the correct programming as an absence of it.
wire lyr_a_regw = reg_cs && reg_we &&
                  ((reg_addr == 5'h08) || (reg_addr == 5'h0c) ||
                   (reg_addr == 5'h10) || (reg_addr == 5'h14));

always @(posedge clk) begin
    if (rst) dbg_a_hit <= 4'd0;
    else begin
        dbg_a_hit[2:0] <= 3'd0;
        if (st == S_DONE && lyr == 2'd0) begin
            dbg_a_hit[0] <= (rom4_data != 32'd0);
            dbg_a_hit[1] <= (rom1_data != 32'd0);
            dbg_a_hit[2] <= (code_w    != 16'd0);
        end
        dbg_a_hit[3] <= lyr_a_regw;
    end
end

// MAME's frame-300 text, the MAME VRAM tap and dist/gokuparo.bin agree on a
// deterministic end-to-end oracle.  Page-0 tile (16,3) contains code 0x0052,
// the first 'R' in "ROM RAM CHECK".  For its final row the address is
// 0x52*8+7 = 0x297 and the raw region bytes are 00 EE EE EE / EE.  Rebuilding
// the whole 13-character mask with this module's address and plane equations
// matches MAME 561/561 pixels, while the board currently draws only 311 and
// many wrong colours.  This pulse therefore separates a wrong VRAM/address
// walk from wrong SDRAM return data without inferring from final RGB.
always @(posedge clk) begin
    if (rst) begin
        dbg_ref_hit <= 2'b00;
    end else begin
        dbg_ref_hit <= 2'b00;
        if (st == S_DONE && lyr == 2'd0 && rom4_addr == 20'h00297) begin
            dbg_ref_hit[0] <= 1'b1;
            // Region bytes 00 EE EE EE with d[7:0] = byte 0 (gx_rommap),
            // then the 1bpp byte EE.
            dbg_ref_hit[1] <= ({rom4_data, rom1_data[7:0]} != 40'hEEEEEE00_EE);
        end
    end
end

// ---------------------------------------------------------------------------
//  NOT YET IMPLEMENTED -- listed so the gaps are visible from the RTL, not
//  only from docs/IMPLEMENTATION_TODO.md
//
//  DONE 2026-09-15: linescroll modes 0 and 2 ("LINE SCROLL").  Still open:
//            mode 1 (UPSTREAM_TODO U23, drawn as XY as MAME does), the flip-Y
//            table walk (:1400, :1576-1577) -- global flip is only the in-place
//            flip here anyway, see the flip TODO below.
//  TODO(P5): the A>B>C>D page-ownership rule.  When two layers claim the same
//            page the higher letter wins and the lower is disabled entirely;
//            some games use that deliberately to hide a layer.
//  TODO(P5): per-tile priority for planes A and B (K055555 regs 8/9/11/12).
//            The game writes them and MAME reads neither -- U15.
//  TODO(P6): global flip corrections, regs 0x3a / 0x3c.
//  TODO(P6): tile_lut (reg 0x38) -- the chip's own 4-entry bank LUT, distinct
//            from the GX-specific external table at 0xd44000.  Both exist and
//            they are not the same mechanism; do not conflate them.
// ---------------------------------------------------------------------------

endmodule

`default_nettype wire
