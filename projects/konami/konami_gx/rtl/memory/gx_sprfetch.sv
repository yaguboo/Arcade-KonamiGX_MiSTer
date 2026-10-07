//============================================================================
//  gx_sprfetch -- the sprite ROM adapter: gx_objdraw's req/ok on one side,
//                 gx_memarb's client 4 on the other
//
//  Moved out of gx_top on 2026-09-14 so that sim/tb_gx_busmix.sv runs the
//  adapter itself and not a copy of it.  With `row_prefetch` and `row_cache`
//  low it is the 9bf82a3 adapter, state for state.
//
//  One 16-dot row of a tile is two 8-dot halves.  Each half is one 32-bit word
//  of planes 0-3, fetched as ONE BURST-2 exactly like the tilemap's, and one
//  byte of plane 4 -- and both halves' bytes of plane 4 sit in the SAME 16-bit
//  SDRAM word (unit = tile*32 + row*2 + half, so the pair differs only in the
//  byte's low bit).  So the second half of a row finds that word in `w1_*` and
//  costs one burst, not a burst and a single.
//
//  The handshake with gx_objdraw is req / one-clock ok, and the requester drops
//  req on the ok edge -- so SF_OK waits exactly one clock.
//
//  ---- row_prefetch, 2026-09-14 ---------------------------------------------
//  MEASURED (docs/MEASUREMENTS.md 19): on busy curtain pictures sprite lines
//  run out of time because the main CPU, which asks almost continuously, is
//  granted in the gaps BETWEEN the sprite client's requests, and a transaction
//  in flight cannot be preempted.  One of those gaps is inside a tile row:
//  gx_objdraw drops req for a clock after the first half's ok and then asks
//  for the other half, and SF_OK -> SF_IDLE adds more.  So every row paid for
//  two CPU transactions, one between tiles and one between its own halves.
//
//  With row_prefetch the adapter answers the first half and then fetches the
//  OTHER half of the same row without dropping its arbiter request -- the
//  sprite client is urgent outside tile groups (gx_top), so a request held
//  across the chain is not interleaved with CPU transactions -- and serves the
//  second request from that copy.  Same number of SDRAM transactions per row;
//  one CPU wait per row instead of two; gx_objdraw's contract untouched.
//
//  A row copy is invalidated by starting another row.  A prefetch whose row
//  was aborted (gx_sprite's line window closed) simply goes unused.
//
//  ---- row_cache, 2026-09-14 -------------------------------------------------
//  MEASURED (docs/MEASUREMENTS.md 29, 30): with the prefetch in, sprite lines
//  still run late on the curtains, the white flash and the demo stages (96-113
//  late lines on the battleship and the underwater boss in tb_gx_busmix), and
//  the tile groups hold the bus for ~60 % of every late window.  The same
//  frames cost the main CPU half its words.  What is short is BUS TIME, and
//  the sprite ROM is ROM: a row read once is the same row forever.
//
//  So a completed row (both halves and the plane-4 word, i.e. what the row copy
//  holds after SF_PF) is also written to block RAM, and a request that misses
//  the row copy looks there first -- one clock (SF_LOOK) -- before asking the
//  arbiter.  A hit costs no SDRAM transaction and does not wait for the tile
//  group gate.  The SDRAM stays the backing store (DECISIONS D12).
//
//  TWO TABLES, skewed (MEASUREMENTS 31).  One 2^12-row table left the demo
//  stages capacity-limited (up to ~7,000 rows a frame; hit rates 37-46 %) and
//  one 2^13-row table would need ~88 M10K against 79 free.  So:
//
//      A  2^CACHE_AW rows, 88 bits  { valid, tile[14:AW-4], w1, d4_1, d4_0 }
//         index { tile[AW-5:0], row }           tag = the tile's high bits
//      B  2^CACHE_BW rows, 96 bits  { valid, tile[14:0],    w1, d4_1, d4_0 }
//         index { fold(tile), row }             tag = the whole tile
//
//  Two tiles sharing their low bits meet in A and part in B.  Both are read on
//  the same edge; a hit in either serves.  A completed row goes into whichever
//  slot was empty at its own missed lookup (valid bits kept from SF_LOOK), and
//  when both were full the tables alternate.
//
//  Block RAM shape (root 7, power_spikes L24): each table has one registered
//  read port addressed from `unit`, one write port driven from registers,
//  whole-word writes.  A lookup never starts on an edge that writes either
//  table, so no port pair meets on one address in one clock.  The valid bits
//  are CLEARED by a sweep after reset -- a generation tag would wrap (memory
//  `generation-tag-cannot-replace-clearing`) -- and the cache is off meanwhile.
//  The cache needs row_prefetch: that is what completes a row.
//
//  ---- fmt, 2026-09-30: three sprite ROM formats, one cache ---------------------
//  0  GX   5bpp  gokuparo .. daiskiss   everything above, unchanged
//  1  GX6  6bpp  salmndr2               planes 4 and 5 are one 16-bit word per
//     HALF (spr1 word GX_SPR1_BASE/2 + unit), fetched for both halves as one
//     even-aligned burst-2.  A row is 96 bits, so the cache holds HALF rows:
//     key {tile[13:0], row, half} where GX holds {tile[14:0], row}, payload
//     {ext, 16'd0, w45, d4}, and a whole row is two writes (the second from
//     registers latched with the first, so a fetch starting in between cannot
//     change it).  Capacity in rows is halved for this set only.
//  2  RNG  4bpp  dragoonj               no plane 4: no spr1 transaction and
//     data1 = 0.  The code is 17 bits (unit 22), and the two extra tag bits
//     live in the payload's otherwise empty w1 field.
//  3  LE2  8bpp  winspike (2026-10-05)   spritelayout3: a half is 8 bytes,
//     one per plane, from four ROM_LOAD64_WORD files.  The image keeps them as
//     two ROM_LOAD32_WORD pairs (a 64-bit .mra interleave is not one the board
//     loader has been seen to take): files 0+1 in the first 8 MB of spr4,
//     files 2+3 in the second.  So a half is two burst-2s at the same offset
//     in each: data4 = bytes 0-3, data1 = bytes 4-7.  No spr1.  The row cache, the live row and
//     the slot are bypassed: every half is a transaction (the cache's 80-bit
//     payload would hold one half, as GX6's does -- a later measurement).
//  The key, the extra tag and the payload are chosen by `fmt`, a per-set
//  constant.  With fmt = 0 every table index, tag and payload is the one the
//  shipped adapter used, bit for bit.  Advisor (Opus, standing in for Codex at
//  its limit) recommended this shape over widening (+11 M10K) or bypassing.
//============================================================================
`default_nettype none

module gx_sprfetch #(
    parameter integer CACHE_AW = 12,         // table A: 2^CACHE_AW rows, 88 bits
    parameter integer CACHE_BW = 11,         // table B: 2^CACHE_BW rows, 96 bits
    parameter bit     HASH_A   = 1'b0,        // table A's index: 0 = direct, 1 = XOR the tag in
    parameter bit     SPEC_PF  = 1'b0,        // fetch the NEXT row while idle (MEASUREMENTS 122)
    // Placement when BOTH ways hold a row.  MEASUREMENTS 128.
    //   0  alternate          the policy that shipped until 2026-09-23
    //   1  always evict A     SHIPPED NOW
    //   2  always evict B
    // Costs NO memory and no DSP -- it is a mux on a one-bit signal -- and it
    // is the only change in this lane that reaches the stability a +20 M10K
    // cache would have bought.  Table A is direct-mapped and table B is
    // hashed, so B is the one that spreads conflicts; keeping B and always
    // retiring A is what the numbers say.
    parameter integer EVICT    = 1,
    // 136: fetch plane 4 as a BURST-2 and keep both words.  unit =
    // tile*32 + row*2 + half, so one 16-bit word holds the plane-4 bytes of
    // BOTH halves of a row and the NEXT word holds the NEXT row's -- so an
    // even-aligned burst-2 serves TWO rows for one transaction.  135.1: a
    // miss is two SDRAM row activations and this halves how often the second
    // one happens, without touching the ROM image or the burst length.
    parameter integer W1BURST  = 0,
    // 138, MEASUREMENT ONLY: fill the row's OTHER half without its own
    // transaction.  A row's two halves of planes 0-3 are FOUR CONSECUTIVE
    // WORDS, so one burst-4 would fetch both -- this prices that, and unlike
    // 135.4's ROM repack it would touch no image and no .mra.
    parameter integer OTHFREE  = 0,
    // 138 MADE REAL, 2026-09-23: fetch a whole row -- both halves of planes
    // 0-3, four consecutive words -- as ONE gx_sdram four-word burst, and drop
    // the SF_PF chain.  OTHFREE priced it with the ROM image read behind the
    // controller; this is the controller doing it, two clocks longer than the
    // fixture per transaction (the two extra READ commands).  Needs
    // row_prefetch, which is what wants the other half at all.  The plane-4
    // word and the ROM image are untouched.
    parameter integer BURST4   = 0,
    // 135, MEASUREMENT ONLY: answer the plane-4 word from `w1_direct` instead
    // of a second bus transaction.  A miss costs TWO SDRAM row activations --
    // planes 0-3 as a burst and plane 4 as a single, in a different region --
    // and this prices what ONE would be worth.  gx_top ties it to 0.
    parameter integer W1FREE   = 0
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        row_prefetch,
    // 122.8: speculate ONLY while this is high.  MEASURED: of the clocks this
    // fetcher is not asking, the bus is busy for 56 % and IDLE for 44 % (about
    // 1,211 of a late line).  122.5/122.6 failed because they speculated in
    // both, queueing behind real traffic in the busy half.  The caller drives
    // this from the arbiter: no transaction in flight and nobody asking.
    input  wire        spec_ok,
    input  wire [15:0] w1_direct,   // W1FREE only
    input  wire [31:0] oth_direct,  // OTHFREE only: the other half's 32 bits
    // 123.3: gx_objscan's pending tile, {dr_code, ysubf, 1'b0}.  MEASURED to be
    // the NEXT request 37 % of the time; rc_row + 1 was 0 of 80,933.
    input  wire [21:0] hint_unit,
    input  wire        hint_valid,
    input  wire        row_cache,

    // --- gx_objdraw, through gx_sprite --------------------------------------
    input  wire [1:0]  fmt,          // 0 GX 5bpp, 1 GX6 6bpp, 2 RNG 4bpp, 3 LE2 8bpp (see the header)
    input  wire [21:0] unit,
    input  wire        req,
    output reg         ok,
    output reg  [31:0] data4,
    output wire [31:0] data1,        // [7:0] plane 4, [15:8] plane 5 (GX6); 0 for RNG;
                                     // LE2: bytes 4-7 of the half, gx_plane_word order

    // --- gx_memarb client ------------------------------------------------------
    output reg  [24:0] arb_addr,
    output reg         arb_req,
    output reg         arb_burst,
    output reg         arb_burst4,    // BURST4: this burst is four words
    input  wire        arb_ack,
    input  wire [15:0] arb_dout,
    input  wire [15:0] arb_dout2,
    //  BURST4 only: a four-word burst's words 0 and 1 (gx_sdram raw_b0 /
    //  raw_b1).  `arb_dout` / `arb_dout2` are then its words 2 and 3, which is
    //  what the raw tier already hands a burst's two words as.
    input  wire [15:0] arb_b0,
    input  wire [15:0] arb_b1
);

`include "gx_rommap.svh"

localparam [2:0] SF_IDLE = 3'd0, SF_W4 = 3'd1, SF_W1 = 3'd2, SF_OK = 3'd3,
                 SF_PF   = 3'd4, SF_LOOK = 3'd5;

reg  [2:0]  st;
reg  [21:0] sf_unit;
// The format flags, REGISTERED here: fmt is a per-set constant (it changes only
// while the core is held for a download), and decoding it next to the logic it
// steers keeps gx_top's obj_fmt register out of every path.  MEASURED,
// 5d6da0d6: obj_fmt -> d1q was the core clock's worst path (-0.152).
reg         f6  = 1'b0;
reg         frn = 1'b0;
reg         fle = 1'b0;              // LE2 8bpp
always @(posedge clk) begin
    f6  <= (fmt == 2'd1);
    frn <= (fmt == 2'd2);
    fle <= (fmt == 2'd3);
end
// data1: the 16-bit register every other format has always used, and LE2's
// 32 bits in a register of their own, chosen at the port by the per-set fmt.
// MEASURED, e8266ee6: with LE2's word written into the same register the
// cache -> data1 path (SF_LOOK) missed the core clock by -0.193.
reg  [15:0] d1q;
reg  [31:0] le_d1;
assign data1 = fle ? le_d1 : {16'd0, d1q};
reg  [24:0] w1_addr;            // W1BURST=0: the one word held
reg  [15:0] w1_data;
reg         w1_valid;
// 136: the burst-2 pair.  `w1p_tag` is the word-address pair index, so a hit
// covers both words and therefore both rows.
reg  [23:0] w1p_tag;
reg  [15:0] w1p_d0, w1p_d1;
reg         w1p_valid;

// the row copy
reg  [20:0] rc_row;
reg  [1:0]  rc_have;
reg  [31:0] rc_d4_0, rc_d4_1;
reg  [15:0] rc_w1;
reg  [15:0] rc_w45_0, rc_w45_1;     // GX6: planes 4/5 of each half
reg         rc_w1_ok;

//  Word addresses, as the tile fetch's are: the 4bpp word of `unit` is at
//  byte GX_SPR4_BASE + unit*4, word GX_SPR4_BASE/2 + unit*2 -- even by
//  construction, which is gx_sdram's BURST CONTRACT.
wire [24:0] a_spr4   = {1'b0, GX_SPR4_BASE[24:1]} + {2'd0, unit, 1'b0};
// LE2: bytes 0-3 of a half are at a_spr4 (files 0+1); bytes 4-7 at the same
// offset in the second 8 MB (files 2+3, 0x400000 words on).  Even, burst-2.
wire [24:0] a_le_hi  = {1'b0, GX_SPR4_BASE[24:1]} + 25'h0400000 + {2'd0, sf_unit, 1'b0};
wire [24:0] a1b      = GX_SPR1_BASE + {3'd0, sf_unit};              // byte
// GX6: one word a half; the even-aligned pair holds both halves of the row
wire [24:0] a16      = {1'b0, GX_SPR1_BASE[24:1]} + {3'd0, sf_unit[21:1], 1'b0};
wire [24:0] a1       = f6 ? a16 : {1'b0, a1b[24:1]};                // word
// 136: the even-aligned pair this word belongs to, and the pair cache's answer
wire [24:0] a1e      = {a1[24:1], 1'b0};
wire        w1p_hit  = (W1BURST != 0) && w1p_valid && (w1p_tag == a1[24:1]);
wire [15:0] w1p_q    = a1[0] ? w1p_d1 : w1p_d0;
// Either cache can answer; the pair one is checked first because it is the
// one that spans two rows.
// RNG has no plane 4: every row is "hit" with a zero word.  GX6 always fetches.
wire        w1_hit   = frn || (!f6 && (w1p_hit || (w1_valid && w1_addr == a1)));
wire [15:0] w1_q     = frn ? 16'd0 : w1p_hit ? w1p_q : w1_data;
wire [21:0] other    = {sf_unit[21:1], ~sf_unit[0]};
wire [24:0] a_other4 = {1'b0, GX_SPR4_BASE[24:1]} + {2'd0, other, 1'b0};

// ---- SPEC_PF: the speculative next row, 2026-09-23 (MEASUREMENTS 122) -----
//  MEASURED: on a late line this fetcher spends 46 % of the line in SF_IDLE
//  with no `req` -- gx_objscan is walking the table (1,103 clocks) or
//  gx_objdraw is working between its own fetches (1,577).  Its own SDRAM
//  transactions are 2,019 of ~6,144.  A sprite is drawn as CONSECUTIVE tile
//  rows, so the row after the last one served is the likely next request, and
//  fetching it while idle turns dead clocks into cache.
//
//  It is speculative, so it must not answer anything: `ok` is suppressed and
//  the row lands in the cache exactly as a real fetch's does.
//
//  THE RISK, and it is why this is a parameter: a real request arriving during
//  a speculative fetch WAITS for it.  Whether the dead clocks bought more than
//  that costs is a measurement, not an argument -- 122.3's arithmetic says the
//  headroom is 2.4x and the shortfall is 38 %, and neither is a result.
// 122.5 REJECTED the first version for two reasons and this fixes both:
//   * it overwrote the LIVE row copy, so the draw's other half then missed.
//     rc_* is not touched at all now -- the slot below holds it, so the
//     speculative row reaches the cache without the live one moving
//   * a real request waited for the whole speculative chain.  The chain is
//     checked at its one safe point -- SF_W4, where the burst's ack has landed
//     and nothing is outstanding -- and abandons there, so a real `req` is
//     delayed by at most ONE transaction
// The DEDICATED prefetch slot (123.4).  One row in its own registers,
// consulted BEFORE the cache and never written into it, so a wrong guess
// costs only the idle bus it was fetched on -- which is what 122.5, 122.6 and
// 123.1 could not say.  rc_* is not touched, so the live row the draw is
// using stays exactly where it was.
reg [20:0]  pfs_row;
reg [31:0]  pfs_d4_0, pfs_d4_1;
reg [15:0]  pfs_w1;
reg [1:0]   pfs_have;
reg         pfs_valid;
wire [21:0] spec_unit = hint_unit;
// SPEC_PF is GX-only (it is 0 as shipped): the slot keeps a GX row
wire        pfs_hit   = !f6 && !frn && pfs_valid && (pfs_row == unit[21:1]) && pfs_have[unit[0]];
wire [24:0] a_spec4   = {1'b0, GX_SPR4_BASE[24:1]} + {2'd0, spec_unit, 1'b0};
wire [24:0] pfs_ub    = GX_SPR1_BASE + {3'd0, unit};
reg         spec;

// Serving `unit` from the row copy.  The plane-4 byte's half comes from the
// byte address of `unit` itself, as a1b's does for sf_unit.
wire [24:0] ub       = GX_SPR1_BASE + {3'd0, unit};
wire        row_hit  = !fle && row_prefetch && rc_w1_ok && rc_row == unit[21:1] &&
                       rc_have[unit[0]];
// what one half of the live row / a cache payload answers as data1
wire [15:0] rc_d1    = frn ? 16'd0
                     : f6  ? (unit[0] ? {rc_w45_1[7:0], rc_w45_1[15:8]} : {rc_w45_0[7:0], rc_w45_0[15:8]})
                           : {8'd0, ub[0] ? rc_w1[7:0] : rc_w1[15:8]};

// BURST4: the burst starts at the row's half 0, so words 0-1 are half 0 and
// words 2-3 half 1 -- whichever half was asked for.  a_row4 is a_spr4 with
// unit[0] cleared: a multiple of four words, gx_sdram's four-word contract.
wire        b4       = (BURST4 != 0) && row_prefetch;
wire        oth_en   = (OTHFREE != 0) || b4;
wire [24:0] a_row4   = {1'b0, GX_SPR4_BASE[24:1]} + {2'd0, unit[21:1], 2'b00};
wire [24:0] a_srow4  = {1'b0, GX_SPR4_BASE[24:1]} + {2'd0, spec_unit[21:1], 2'b00};
wire [31:0] w4_h0    = gx_plane_word(arb_b0, arb_b1);
wire [31:0] w4_h1    = gx_plane_word(arb_dout, arb_dout2);
wire [31:0] w4_now   = !b4 ? gx_plane_word(arb_dout, arb_dout2)
                           : (sf_unit[0] ? w4_h1 : w4_h0);
// The row's OTHER half: from this burst on its ack clock, and from `oth_q`
// after it (SF_W1 comes a transaction later).  OTHFREE's fixture otherwise.
wire [31:0] w4_oth   = sf_unit[0] ? w4_h0 : w4_h1;
reg  [31:0] oth_q;
wire [31:0] oth_w4   = b4 ? w4_oth : oth_direct;
wire [31:0] oth_w1   = b4 ? oth_q  : oth_direct;
// 136: SF_W1's answer.  With the burst it is whichever half of the pair this
// request asked for; without it the single word that came back.
wire [15:0] w1_sel   = (W1BURST != 0) ? (a1[0] ? arb_dout2 : arb_dout)
                                      : arb_dout;

// ---- the row cache: two tables ---------------------------------------------------
localparam integer TAGA = 15 - (CACHE_AW - 4);     // A's tag: the tile's high bits
// Table A's word is { valid, tag[TAGA-1:0], w1, d4_1, d4_0 } -- one valid bit,
// the tag, and 80 bits of payload.  CWA was the CONSTANT 88, which is right
// only at CACHE_AW = 12 (TAGA = 7, 1 + 7 + 80 = 88).  At any other size the
// write `{1'b1, rc_row[18:CACHE_AW], rc_all}` is narrower than the register,
// zero-extends on the LEFT, and the valid bit lands below cqa[CWA-1] -- so
// `a_v` reads 0 forever and table A never hits.  That is the "13/14 give hit
// exactly 0" the 2026-09-22 handoff recorded, and it made the row cache the
// one lever in the fetch path that could not be evaluated (MEASUREMENTS
// 117.3).  At CACHE_AW = 12 this expression is 88, so the SHIPPED core is
// unchanged -- checked by re-running the suite and the flicker metric.
// 2026-10-03, tokkae / tkmmpzdm: a GX 5bpp set with an 8 MB 4bpp sprite
// area has 65536 tiles, so tile bit 15 (unit[20]) is live and the 15-bit
// key above would alias tile t and t + 32768.  One more tag bit, `t15`,
// right under the valid bit; 0 for every 4 MB GX set (their tile bit 15 is
// always 0, gx_sprite dr_code), and 0 for GX6 / RNG, whose ext field already
// carries the high bits.  Width only: 89 and 97 bits still fill the same
// 2K x 5 M10K slices as 88 and 96.
localparam integer CWA  = 82 + TAGA, CWB = 97;

// +hasha=1 (candidate, 2026-09-23, MEASUREMENTS 118): XOR the index with the
// TAG bits.  A direct-mapped `r[CACHE_AW-1:0]` gives every tile whose low
// AW-4 bits agree the same eight rows, and a sprite sheet's tiles are
// consecutive, so a screen that walks two sheets far apart in the ROM
// collides on every row.  Mixing the high bits in costs NO MEMORY: the tag is
// still r[18:CACHE_AW], and (index, tag) still determines the address because
// low = index ^ f(tag).  The alternative -- CACHE_AW 13 -- takes the flicker
// to zero and costs +43 M10K against the four P8 measured free.
function automatic [CACHE_AW-1:0] cidx_a(input [18:0] r);
    reg [CACHE_AW-1:0] lo, hi;
begin
    lo     = r[CACHE_AW-1:0];                      // { tile[AW-5:0], row }
    hi     = {{(2*CACHE_AW-19){1'b0}}, r[18:CACHE_AW]};
    cidx_a = HASH_A ? (lo ^ hi) : lo;
end
endfunction

function automatic [CACHE_BW-1:0] cidx_b(input [18:0] r);
    reg [14:0] t, f;
begin
    t      = r[18:4];
    f      = t ^ (t >> (CACHE_BW - 4));
    cidx_b = {f[CACHE_BW-5:0], r[3:0]};
end
endfunction

//  max_depth 2048 (2026-10-02, docs/BRAM_AUDIT.md 3): the same RAM, built from
//  2K x 5 M10K slices instead of 4K x 2 -- 36 blocks for 4096 x 88, not 44.
//  No port, address or timing of the RAM changes; only how it is packed.
(* ramstyle = "M10K", max_depth = 2048 *) reg [CWA-1:0] cmem_a [0:(1 << CACHE_AW) - 1];
(* ramstyle = "M10K" *) reg [CWB-1:0] cmem_b [0:(1 << CACHE_BW) - 1];
reg  [CWA-1:0]      cqa;
reg  [CWB-1:0]      cqb;
reg                 c_we_a, c_we_b;
reg  [CACHE_AW-1:0] c_wa_a;
reg  [CACHE_BW-1:0] c_wa_b;
reg  [CWA-1:0]      c_wd_a;
reg  [CWB-1:0]      c_wd_b;
// The 19-bit key the index and tag are cut from, and the extra tag bits kept
// in the payload's w1 field (not for GX, whose w1 field is plane 4 and whose
// codes are 15 bits, so it has none).
wire [18:0]         key_now = f6 ? unit[18:0] : unit[19:1];
wire [15:0]         ext_now = f6 ? {13'd0, unit[21:19]} : {14'd0, unit[21:20]};
wire [18:0]         key_rc0 = f6 ? {rc_row[17:0], 1'b0} : rc_row[18:0];
wire [18:0]         key_rc1 = {rc_row[17:0], 1'b1};
wire [15:0]         ext_rc  = f6 ? {13'd0, rc_row[20:18]} : {14'd0, rc_row[20:19]};
wire [CACHE_AW-1:0] c_ra_a = cidx_a(key_now);
wire [CACHE_BW-1:0] c_ra_b = cidx_b(key_now);

always @(posedge clk) begin
    cqa <= cmem_a[c_ra_a];
    if (c_we_a) cmem_a[c_wa_a] <= c_wd_a;
end

always @(posedge clk) begin
    cqb <= cmem_b[c_ra_b];
    if (c_we_b) cmem_b[c_wa_b] <= c_wd_b;
end

reg                 clearing;
reg  [CACHE_AW-1:0] clr_a;
reg                 cw_pend;
reg                 cw2_pend;        // GX6: the row's second half, from cw2_*
reg  [CACHE_AW-1:0] cw2_a;
reg  [CACHE_BW-1:0] cw2_b;
reg  [CWA-1:0]      cw2_da;
reg  [CWB-1:0]      cw2_db;
reg                 wr_to_b;         // chosen at the row's miss, used when it is whole
reg                 evict_tgl;

wire            c_on   = row_cache && row_prefetch && !clearing && !fle;
wire            a_v    = cqa[CWA-1];
wire            a_t15  = cqa[CWA-2];
wire [TAGA-1:0] a_tag  = cqa[CWA-3 -: TAGA];
wire            gx5    = !f6 && !frn;
wire            t15_now = gx5 && unit[20];
wire            t15_rc  = gx5 && rc_row[19];
wire            hit_a  = a_v && a_t15 == t15_now && a_tag == key_now[18:CACHE_AW] && (gx5 || cqa[79:64] == ext_now);
wire            b_v    = cqb[CWB-1];
wire            b_t15  = cqb[CWB-2];
wire [14:0]     b_tag  = cqb[CWB-3 -: 15];
wire            hit_b  = b_v && b_t15 == t15_now && b_tag == key_now[18:4] && (gx5 || cqb[79:64] == ext_now);
wire            c_hit  = hit_a || hit_b;
wire [79:0]     c_row  = hit_a ? cqa[79:0] : cqb[79:0];   // { w1, d4_1, d4_0 }
wire [15:0]     c_w1   = c_row[79:64];
wire [31:0]     c_d4_1 = c_row[63:32];
wire [31:0]     c_d4_0 = c_row[31:0];
// GX6 half entry: { ext, 16'd0, w45, d4 }
wire [15:0]     c_w45  = c_row[47:32];
// the payload written: GX {w1, d4_1, d4_0}; RNG {ext, d4_1, d4_0}; GX6 per half
wire [79:0]     rc_all = {frn ? ext_rc : rc_w1, rc_d4_1, rc_d4_0};
wire [79:0]     rc_h0  = {ext_rc, 16'd0, rc_w45_0, rc_d4_0};
wire [79:0]     rc_h1  = {ext_rc, 16'd0, rc_w45_1, rc_d4_1};
wire [15:0]     c_d1   = frn ? 16'd0
                       : f6  ? {c_w45[7:0], c_w45[15:8]}
                             : {8'd0, ub[0] ? c_w1[7:0] : c_w1[15:8]};

always @(posedge clk) begin
    ok     <= 1'b0;
    c_we_a <= 1'b0;
    c_we_b <= 1'b0;
    if (rst) begin
        st        <= SF_IDLE;
        arb_req   <= 1'b0;
        arb_burst <= 1'b0;
        arb_burst4 <= 1'b0;
        w1_valid  <= 1'b0;
        w1p_valid <= 1'b0;
        // Driven here so that a parameter which leaves their only other
        // assignment dead (W1BURST=0, SPEC_PF=0 -- both as shipped) does not
        // leave them undriven: Quartus 10030 x3, map of 102378e0.  Each is
        // qualified by a valid bit that is 0 here, so nothing reads the value.
        w1p_d0    <= 16'd0;
        w1p_d1    <= 16'd0;
        pfs_row   <= 21'd0;
        rc_have   <= 2'b00;
        rc_w1_ok  <= 1'b0;
        spec      <= 1'b0;
        pfs_valid <= 1'b0;
        pfs_have  <= 2'b00;
        clearing  <= 1'b1;
        clr_a     <= {CACHE_AW{1'b0}};
        cw_pend   <= 1'b0;
        cw2_pend  <= 1'b0;
        wr_to_b   <= 1'b0;
        evict_tgl <= 1'b0;
    end else begin
        // the sweep, then the row writes -- never both, c_on is low while sweeping
        if (clearing) begin
            c_we_a <= 1'b1;
            c_wa_a <= clr_a;
            c_wd_a <= {CWA{1'b0}};
            c_we_b <= 1'b1;
            c_wa_b <= clr_a[CACHE_BW-1:0];
            c_wd_b <= {CWB{1'b0}};
            clr_a  <= clr_a + 1'b1;
            if (&clr_a) clearing <= 1'b0;
        end else if (cw_pend) begin
            // rc_* hold the whole row this clock (SF_PF's ack landed on the edge
            // before); a new fetch starting now changes them only after it
            if (c_on && rc_have == 2'b11 && rc_w1_ok) begin
                if (wr_to_b) begin
                    c_we_b <= 1'b1;
                    c_wa_b <= cidx_b(key_rc0);
                    c_wd_b <= {1'b1, t15_rc, key_rc0[18:4], f6 ? rc_h0 : rc_all};
                end else begin
                    c_we_a <= 1'b1;
                    c_wa_a <= cidx_a(key_rc0);
                    c_wd_a <= {1'b1, t15_rc, key_rc0[18:CACHE_AW], f6 ? rc_h0 : rc_all};
                end
                // GX6: the other half next clock, from registers taken now
                cw2_pend <= f6;
                cw2_a    <= cidx_a(key_rc1);
                cw2_b    <= cidx_b(key_rc1);
                cw2_da   <= {1'b1, 1'b0, key_rc1[18:CACHE_AW], rc_h1};
                cw2_db   <= {1'b1, 1'b0, key_rc1[18:4], rc_h1};
            end
        end else if (cw2_pend) begin
            if (wr_to_b) begin c_we_b <= 1'b1; c_wa_b <= cw2_b; c_wd_b <= cw2_db; end
            else         begin c_we_a <= 1'b1; c_wa_a <= cw2_a; c_wd_a <= cw2_da; end
        end
        cw_pend  <= 1'b0;
        if (!cw_pend) cw2_pend <= 1'b0;

        case (st)
            SF_IDLE: if (req) begin
                if (fle) begin
                    // LE2: bytes 0-3, then (SF_W4) bytes 4-7
                    sf_unit    <= unit;
                    arb_addr   <= a_spr4;
                    arb_burst  <= 1'b1;
                    arb_burst4 <= 1'b0;
                    arb_req    <= 1'b1;
                    spec       <= 1'b0;
                    st         <= SF_W4;
                end else if (pfs_hit) begin
                    // the speculation was right: one clock, no bus at all
                    data4     <= unit[0] ? pfs_d4_1 : pfs_d4_0;
                    d1q       <= {8'd0, pfs_ub[0] ? pfs_w1[7:0] : pfs_w1[15:8]};
                    ok        <= 1'b1;
                    st        <= SF_OK;
                    pfs_valid <= 1'b0;
                end else if (row_hit) begin
                    data4 <= unit[0] ? rc_d4_1 : rc_d4_0;
                    // Big-endian, as the tile fetch: byte 2W is bits 15-8.
                    d1q   <= rc_d1;
                    ok    <= 1'b1;
                    st    <= SF_OK;
                end else if (c_on) begin
                    // cqa / cqb hold this unit's slots after this edge; not on a write
                    // edge, nor the edge before GX6's second write
                    if (!c_we_a && !c_we_b && !cw2_pend) st <= SF_LOOK;
                end else begin
                    sf_unit   <= unit;
                    arb_addr  <= b4 ? a_row4 : a_spr4;
                    arb_burst <= 1'b1;
                    arb_burst4 <= b4;
                    arb_req   <= 1'b1;
                    st        <= SF_W4;
                    rc_row    <= unit[21:1];
                    rc_have   <= 2'b00;
                    rc_w1_ok  <= 1'b0;
                    spec      <= 1'b0;
                end
            end else if (SPEC_PF && !f6 && !frn && spec_ok && hint_valid && !cw_pend &&
                         !(pfs_valid && pfs_row == hint_unit[21:1]) &&
                         !(rc_w1_ok  && rc_row  == hint_unit[21:1])) begin
                // idle bus, and the scan has a tile neither the slot nor the
                // live row holds.  Fetch it into the SLOT.
                sf_unit   <= spec_unit;
                arb_addr  <= b4 ? a_srow4 : a_spec4;
                arb_burst <= 1'b1;
                arb_burst4 <= b4;
                arb_req   <= 1'b1;
                st        <= SF_W4;
                pfs_row   <= spec_unit[21:1];
                pfs_have  <= 2'b00;
                pfs_valid <= 1'b0;
                spec      <= 1'b1;
            end
            SF_LOOK: if (c_hit) begin
                // a GX6 entry is one half, in the d4_0 / w45 slots
                data4    <= f6 ? c_d4_0 : unit[0] ? c_d4_1 : c_d4_0;
                d1q      <= c_d1;
                ok       <= 1'b1;
                st       <= SF_OK;
                // the row copy takes the whole row, so the other half is a row_hit
                // (GX6: only the half the entry holds)
                rc_row   <= unit[21:1];
                rc_have  <= f6 ? (unit[0] ? 2'b10 : 2'b01) : 2'b11;
                if (!f6 || !unit[0]) rc_d4_0 <= c_d4_0;
                if (!f6)             rc_d4_1 <= c_d4_1;
                else if (unit[0])    rc_d4_1 <= c_d4_0;
                if (f6) begin
                    if (unit[0]) rc_w45_1 <= c_w45; else rc_w45_0 <= c_w45;
                end
                rc_w1    <= c_w1;
                rc_w1_ok <= 1'b1;
            end else begin
                // where this row will be cached once it is whole
                wr_to_b   <= (EVICT == 1) ? (a_v && !b_v)
                           : (EVICT == 2) ? (a_v || b_v ? 1'b1 : 1'b0)
                           :                (a_v && (!b_v || evict_tgl));
                if (a_v && b_v) evict_tgl <= ~evict_tgl;
                sf_unit   <= unit;
                arb_addr  <= b4 ? a_row4 : a_spr4;
                arb_burst <= 1'b1;
                arb_burst4 <= b4;
                arb_req   <= 1'b1;
                st        <= SF_W4;
                rc_row    <= unit[21:1];
                rc_have   <= 2'b00;
                rc_w1_ok  <= 1'b0;
            end
            SF_W4: if (arb_ack && spec && req) begin
                // a real request arrived: abandon the speculation HERE, where
                // the burst's ack has landed and nothing is outstanding.
                arb_req   <= 1'b0;
                arb_burst <= 1'b0;
                arb_burst4 <= 1'b0;
                spec      <= 1'b0;
                st        <= SF_IDLE;
                pfs_have  <= 2'b00;       // the partial slot is simply dropped
            end else if (arb_ack && fle) begin
                // bytes 0-3; the request stays up for bytes 4-7 (SF_W1)
                data4      <= gx_plane_word(arb_dout, arb_dout2);
                arb_addr   <= a_le_hi;
                st         <= SF_W1;
            end else if (arb_ack) begin
                if (!spec) data4 <= w4_now;
                if (spec) begin
                    if (sf_unit[0]) pfs_d4_1 <= w4_now; else pfs_d4_0 <= w4_now;
                    pfs_have[sf_unit[0]] <= 1'b1;
                end else begin
                    if (sf_unit[0]) rc_d4_1 <= w4_now; else rc_d4_0 <= w4_now;
                    rc_have[sf_unit[0]] <= 1'b1;
                end
                arb_burst <= 1'b0;
                arb_burst4 <= 1'b0;
                oth_q     <= w4_oth;
                if (w1_hit) begin
                    if (!spec) begin
                        d1q      <= frn ? 16'd0 : {8'd0, a1b[0] ? w1_q[7:0] : w1_q[15:8]};
                        rc_w1    <= w1_q;
                        rc_w1_ok <= 1'b1;
                    end else pfs_w1 <= w1_q;
                    ok <= !spec;                // a speculative row answers nothing
                    if (row_prefetch && oth_en) begin
                        // 138: the other half comes with the same burst, so the row is
                        // whole here and the chained transaction is not needed.
                        if (spec) begin
                            if (other[0]) pfs_d4_1 <= oth_w4; else pfs_d4_0 <= oth_w4;
                            pfs_have[other[0]] <= 1'b1;
                            pfs_valid <= 1'b1;
                        end else begin
                            if (other[0]) rc_d4_1 <= oth_w4; else rc_d4_0 <= oth_w4;
                            rc_have[other[0]] <= 1'b1;
                            cw_pend <= c_on;
                        end
                        arb_req <= 1'b0;
                        st      <= SF_OK;
                    end else if (row_prefetch) begin
                        arb_addr  <= a_other4;     // req stays up: the chain
                        arb_burst <= 1'b1;
                        st        <= SF_PF;
                    end else begin
                        arb_req <= 1'b0;
                        st      <= SF_OK;
                    end
                end else if (W1FREE != 0) begin
                    // 135: the plane-4 word arrives with no second transaction
                    w1_addr  <= a1;
                    w1_data  <= w1_direct;
                    w1_valid <= 1'b1;
                    if (!spec) begin
                        d1q      <= {8'd0, a1b[0] ? w1_direct[7:0] : w1_direct[15:8]};
                        rc_w1    <= w1_direct;
                        rc_w1_ok <= 1'b1;
                    end else pfs_w1 <= w1_direct;
                    ok <= !spec;
                    if (row_prefetch && oth_en) begin
                        // 138: the other half comes with the same burst, so the row is
                        // whole here and the chained transaction is not needed.
                        if (spec) begin
                            if (other[0]) pfs_d4_1 <= oth_w4; else pfs_d4_0 <= oth_w4;
                            pfs_have[other[0]] <= 1'b1;
                            pfs_valid <= 1'b1;
                        end else begin
                            if (other[0]) rc_d4_1 <= oth_w4; else rc_d4_0 <= oth_w4;
                            rc_have[other[0]] <= 1'b1;
                            cw_pend <= c_on;
                        end
                        arb_req <= 1'b0;
                        st      <= SF_OK;
                    end else if (row_prefetch) begin
                        arb_addr  <= a_other4;
                        arb_burst <= 1'b1;
                        st        <= SF_PF;
                    end else begin
                        arb_req <= 1'b0;
                        st      <= SF_OK;
                    end
                end else begin
                    // 136: an even-aligned burst-2 brings this row's plane-4
                    // word and the NEXT row's, for one transaction.
                    // GX6: the burst-2 is the row's two halves' plane-4/5 words.
                    arb_addr  <= (W1BURST != 0) ? a1e : a1;
                    arb_burst <= (W1BURST != 0 || f6) ? 1'b1 : 1'b0;
                    st        <= SF_W1;
                end
            end
            SF_W1: if (arb_ack && fle) begin
                le_d1     <= gx_plane_word(arb_dout, arb_dout2);
                ok        <= 1'b1;
                arb_req   <= 1'b0;
                arb_burst <= 1'b0;
                st        <= SF_OK;
            end else if (arb_ack) begin
                w1_addr  <= a1;
                w1_data  <= w1_sel;
                w1_valid <= !f6;
                if (f6) begin
                    // the pair: half 0's word first (arb_dout), half 1's second
                    rc_w45_0  <= arb_dout;
                    rc_w45_1  <= arb_dout2;
                    arb_burst <= 1'b0;
                end
                if (W1BURST != 0) begin
                    w1p_tag   <= a1[24:1];
                    w1p_d0    <= arb_dout;      // the pair's even word
                    w1p_d1    <= arb_dout2;     // ... and its odd word
                    w1p_valid <= 1'b1;
                    arb_burst <= 1'b0;
                end
                if (!spec) begin
                    d1q      <= f6 ? (sf_unit[0] ? {arb_dout2[7:0], arb_dout2[15:8]} : {arb_dout[7:0], arb_dout[15:8]})
                                   : {8'd0, a1b[0] ? w1_sel[7:0] : w1_sel[15:8]};
                    rc_w1    <= w1_sel;
                    rc_w1_ok <= 1'b1;
                end else pfs_w1 <= w1_sel;
                ok <= !spec;                    // a speculative row answers nothing
                if (row_prefetch && oth_en) begin
                    // 138: the other half comes with the same burst, so the row is
                    // whole here and the chained transaction is not needed.
                    if (spec) begin
                        if (other[0]) pfs_d4_1 <= oth_w1; else pfs_d4_0 <= oth_w1;
                        pfs_have[other[0]] <= 1'b1;
                        pfs_valid <= 1'b1;
                    end else begin
                        if (other[0]) rc_d4_1 <= oth_w1; else rc_d4_0 <= oth_w1;
                        rc_have[other[0]] <= 1'b1;
                        cw_pend <= c_on;
                    end
                    arb_req <= 1'b0;
                    st      <= SF_OK;
                end else if (row_prefetch) begin
                    arb_addr  <= a_other4;
                    arb_burst <= 1'b1;
                    st        <= SF_PF;
                end else begin
                    arb_req <= 1'b0;
                    st      <= SF_OK;
                end
            end
            // The other half.  gx_objdraw's next request is not looked at
            // until this lands, and the ok above was a different half, so
            // there is no stale request to skip here.
            SF_PF: if (arb_ack) begin
                if (spec) begin
                    if (other[0]) pfs_d4_1 <= w4_now; else pfs_d4_0 <= w4_now;
                    pfs_have[other[0]] <= 1'b1;
                    pfs_valid <= 1'b1;             // the slot holds a whole row
                end else begin
                    if (other[0]) rc_d4_1 <= w4_now; else rc_d4_0 <= w4_now;
                    rc_have[other[0]] <= 1'b1;
                    cw_pend <= c_on;               // the row is whole: cache it
                end
                arb_req   <= 1'b0;
                arb_burst <= 1'b0;
                st        <= SF_IDLE;
                spec      <= 1'b0;
            end
            default: st <= SF_IDLE;              // SF_OK: one clock
        endcase
    end
end

endmodule

`default_nettype wire
