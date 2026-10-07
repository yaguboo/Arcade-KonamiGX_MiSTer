//============================================================================
//  Konami System GX -- SDRAM ROM map  (set: gokuparo)
//
//  One place that the loader, the fetch logic and the MRA all agree on.
//
//  ---- the packing decision ------------------------------------------------
//  Tiles and sprites are both 5 bpp, and 5 is an awkward number.  The obvious
//  move is to pad each 8-pixel row out to 64 bits so one aligned read yields
//  eight pixels.  That costs 60% on 7.5 MB of graphics and it was NOT taken.
//
//  Instead this map keeps the split the PCB already has.  Both graphics
//  regions are physically two ROM groups: a 4bpp group and a separate 1bpp
//  ROM carrying the top plane (docs/SOURCE_AUDIT.md section 12).  MAME merges
//  them into a 5-byte-per-row region at load time; we do not.
//
//      tiles    321b14.17h  2 MB    4 planes   ->  one 32-bit read = 8 px
//               321b12.13g  512 KB  1 plane    ->  one  8-bit read = 8 px
//      sprites  321b11+b10  4 MB    4 planes   ->  one 32-bit read = 8 px
//               321b09.30g  1 MB    1 plane    ->  one  8-bit read = 8 px
//
//  Two consequences, both good:
//
//    * zero padding.  7.5 MB of graphics stays 7.5 MB.
//    * the ROMs go in raw.  MAME's interleave arithmetic cancels exactly --
//      see "why no repacking is needed" below.  The MRA concatenates files
//      and does one 32-bit word interleave for the sprites, nothing more.
//
//  Cost: two fetches per 8-pixel group instead of one.  The second is 8 bits
//  and lands in a different region, so it can be issued in parallel with the
//  first on a banked controller.
//
//  ---- why no repacking is needed ------------------------------------------
//  MAME builds its tile region with
//      TILE_WORD_ROM_LOAD  = ROM_GROUPDWORD | ROM_SKIP(1)   4 bytes, skip 1
//      TILE_BYTE_ROM_LOAD  = ROM_GROUPBYTE  | ROM_SKIP(4)   1 byte,  skip 4
//  so region byte 5*i+0..3 comes from word-ROM byte 4*i+0..3, and region byte
//  5*i+4 comes from byte-ROM byte i.  charlayout5 puts one 8-pixel row at
//  region offset 5*(tile*8 + row).
//
//  Substituting: the four low planes of (tile,row) sit at word-ROM offset
//  4*(tile*8+row), and the top plane at byte-ROM offset (tile*8+row).  Both
//  are the natural index.  The interleave and the layout cancel.
//
//  The same holds for sprites: 16x16x5bpp is 160 bytes = 32 five-byte units,
//  two units per row (left 8 px, right 8 px), so
//      unit = tile*32 + row*2 + half
//  indexes the 4bpp ROM at unit*4 and the 1bpp ROM at unit.
//
//  This is BUILD_TIME_ONLY under root CLAUDE.md section 6 -- and barely even
//  that, since nothing is reordered.
//
//  ---- plane order ---------------------------------------------------------
//  MAME planeoffset arrays, from docs/SOURCE_AUDIT.md section 12.  Note the
//  two are NOT the same: tiles swap planes 2 and 3 relative to sprites.
//
//      tiles    charlayout5   { 32, 24,  8, 16, 0 }
//      sprites  spritelayout  { 32, 24, 16,  8, 0 }
//
//  planeoffset[0] is the MOST significant pixel bit.  Offset 32 is byte 4,
//  which is the separate 1bpp ROM in both cases.  So, with d[31:0] the 32-bit
//  word from the 4bpp region (d[7:0] = region byte 0) and p the byte from the
//  1bpp region, the pixel bits are:
//
//      tiles     px[4]=p   px[3]=d[31:24]  px[2]=d[15:8]  px[1]=d[23:16]  px[0]=d[7:0]
//      sprites   px[4]=p   px[3]=d[31:24]  px[2]=d[23:16] px[1]=d[15:8]   px[0]=d[7:0]
//
//  Within every plane byte, MAME bit offset 0 is the MSB (its readbit() is
//  `(src[n/8] >> (7 - n%8)) & 1`), and xoffset is {0,1,...,7}.  So bit 7 of
//  each plane byte is the LEFTMOST pixel -- the factory's usual packed_msb
//  convention, no swizzle needed.
//
//  ---- sizes ---------------------------------------------------------------
//  Actual ROM content, no padding:
//
//      bios       128 KB      300a01.34k          shared by every GX set
//      main         1 MB      321jad02 + 321jad04 32-bit, word-interleaved
//      sound      256 KB      321b06 + 321b07     16-bit, byte-interleaved
//      tile4      2.0 MB      321b14.17h
//      tile1      512 KB      321b12.13g
//      spr4       4.0 MB      321b11.25g + 321b10.28g
//      spr1       1.0 MB      321b09.30g
//      pcm        4.0 MB      321b17.9g + 321b18.7g
//                 -------
//                12.875 MB
//
//  Regions are aligned so that every fetcher can mask rather than add.
//  (2026-09-30: the map below now takes the whole 32 MB -- see there.)
//
//  NOTE: everything except bios and main is currently MISSING from this
//  machine -- fantjour.zip is absent (docs/SOURCE_AUDIT.md section 13).  The
//  map is written in full anyway so the loader and MRA do not have to change
//  when the archive turns up.
//============================================================================
//  ---- no include guard, on purpose ----------------------------------------
//  Everything below is a module-scope `localparam`, so each module that wants
//  these constants has to have its own copy emitted inside it.  A
//  `ifndef/`define guard makes the FIRST include win for the whole
//  compilation unit and every later one expand to nothing -- and the symptom
//  is not "already defined", it is "Can't find definition of GX_BIOS_BASE" in
//  whichever module happens to be compiled second.  Include this once per
//  module and never twice.

// ---- CPU-visible ROM windows (68EC020 address space) -----------------------
// docs/SOURCE_AUDIT.md section 4.  The 68EC020 has 24 address bits.
localparam [23:0] GX_CPU_BIOS_LO = 24'h00_0000;   // 128 KB
localparam [23:0] GX_CPU_BIOS_HI = 24'h01_ffff;
localparam [23:0] GX_CPU_PRG_LO  = 24'h20_0000;   // 2 MB window, 1 MB present
localparam [23:0] GX_CPU_PRG_HI  = 24'h3f_ffff;
localparam [23:0] GX_CPU_DAT_LO  = 24'h40_0000;   // data ROM window
localparam [23:0] GX_CPU_DAT_HI  = 24'h7f_ffff;   // gokuparo loads nothing here

// ---- SDRAM byte addresses --------------------------------------------------
//
//  THESE ARE BYTE ADDRESSES.  The neutral `mem_addr` port at the board's edge
//  is a WORD address, because that is what a 16-bit SDRAM controller wants and
//  what the sibling lanes' controllers take.  The two conventions meet at
//  exactly two places -- gx_main's ROM fetch and gx_top's tile fetch -- and
//  both write the shift out in full rather than hiding it.
//
//  Byte addresses are kept here because everything OUTSIDE the FPGA counts in
//  bytes: the MRA, the ROM image, the sizes below and MAME's own region
//  offsets.  Halving them here would make this table impossible to check
//  against the ROM_START block it came from.
//  ---- 2026-09-30: one map for every set, the whole 32 MB --------------------
//  Dragoon Might's sprites are 16 MB (LAYOUT_RNG, 8 x 2 MB) and Salamander 2's
//  tiles and sprites are 6 bpp (a 3 MB 4bpp tile ROM, a 2-byte-a-row top-plane
//  ROM, a 2 MB plane-4/5 sprite ROM), so the 14 MB gokuparo map no longer
//  holds every set.  The bases stay constants -- a per-set base mux would put
//  a LUT on address paths that meet timing by +0.1 ns -- and the price is a
//  larger download for the sets that leave most of it empty.
//
//      region  base       size   5bpp sets        salmndr2           dragoonj
//      tile4   0x0200000  4 MB   2 MB             3 MB (a09+a11)     2 MB
//      tile1   0x0600000  2 MB   512 KB, 1 B/row  2 MB, 2 B/row      --  (plane 4 = 0)
//      pcm     0x0800000  4 MB   4 MB             3 MB               2 MB
//      spr1    0x0C00000  2 MB   1 MB, 1 B/unit   2 MB, 2 B/unit     --  (plane 4 = 0)
//      data    0x0E00000  2 MB   --               --                 2 MB (CPU 0x400000)
//      spr4    0x1000000  16 MB  4 MB             4 MB (a08+a07)     16 MB
localparam [24:0] GX_BIOS_BASE  = 25'h000_0000;   // 128 KB
localparam [24:0] GX_BIOS_SIZE  = 25'h002_0000;

localparam [24:0] GX_MAIN_BASE  = 25'h002_0000;   // 1 MB, 32-bit big-endian
localparam [24:0] GX_MAIN_SIZE  = 25'h010_0000;

localparam [24:0] GX_SND_BASE   = 25'h012_0000;   // 256 KB, 16-bit big-endian
localparam [24:0] GX_SND_SIZE   = 25'h004_0000;

localparam [24:0] GX_TILE4_BASE = 25'h020_0000;   // 4 MB   planes 3..0
localparam [24:0] GX_TILE4_SIZE = 25'h040_0000;

localparam [24:0] GX_TILE1_BASE = 25'h060_0000;   // 2 MB   plane 4 (5bpp) / planes 4,5 (6bpp)
localparam [24:0] GX_TILE1_SIZE = 25'h020_0000;

localparam [24:0] GX_PCM_BASE   = 25'h080_0000;   // 4 MB
localparam [24:0] GX_PCM_SIZE   = 25'h040_0000;

localparam [24:0] GX_SPR1_BASE  = 25'h0C0_0000;   // 2 MB   plane 4 (5bpp) / planes 4,5 (6bpp)
localparam [24:0] GX_SPR1_SIZE  = 25'h020_0000;

localparam [24:0] GX_DATA_BASE  = 25'h0E0_0000;   // 2 MB   CPU data ROM, 0x400000- (dragoonj)
localparam [24:0] GX_DATA_SIZE  = 25'h020_0000;

localparam [24:0] GX_SPR4_BASE  = 25'h100_0000;   // 16 MB  planes 3..0
localparam [24:0] GX_SPR4_SIZE  = 25'h100_0000;

localparam [24:0] GX_ROM_END    = 25'h1FF_FFFF;   // the last byte: 32 MB

// ---- address arithmetic used by the fetchers -------------------------------
// tiles:   8x8, 5 bpp.  index = tile*8 + row      (13-bit code + 3-bit row)
//          4bpp word address = GX_TILE4_BASE + index*4
//          1bpp byte address = GX_TILE1_BASE + index
//
// sprites: 16x16, 5 bpp.  unit = tile*32 + row*2 + half
//          4bpp word address = GX_SPR4_BASE  + unit*4
//          1bpp byte address = GX_SPR1_BASE  + unit
//
// Both are shifts, not multiplies.

// ---- one plane word from its two SDRAM words ------------------------------
// Every fetcher reads a 4bpp word as two 16-bit SDRAM words, and gx_download
// stores those big-endian: the word at byte address 2W is {byte 2W, byte 2W+1}.
// The macros below want d[7:0] = region byte 0, so the four bytes are laid out
// explicitly, here and nowhere else.
//
// MEASURED 2026-09-14: the tile fetch had {w_at, w_next}, which puts byte 0 in
// d[31:24].  Transparency survives that permutation, so every tile SHAPE was
// right and every colour was wrong.  tb_gx_tilemap pins this function.
function automatic [31:0] gx_plane_word(input [15:0] w_at, input [15:0] w_next);
    gx_plane_word = { w_next[7:0], w_next[15:8], w_at[7:0], w_at[15:8] };
endfunction

// ---- pixel assembly --------------------------------------------------------
// d = 32-bit word from the 4bpp region, p = byte from the 1bpp region,
// n = pixel index 0..7 left to right.  See "plane order" above.
`define GX_TILE_PX(d, p, n) \
    { (p)[7-(n)], (d)[31-(n)], (d)[15-(n)], (d)[23-(n)], (d)[7-(n)] }

`define GX_SPR_PX(d, p, n) \
    { (p)[7-(n)], (d)[31-(n)], (d)[23-(n)], (d)[15-(n)], (d)[7-(n)] }
