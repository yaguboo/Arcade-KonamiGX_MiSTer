/* SPDX-FileCopyrightText: 2026 Jose Tejada Gomez
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Derived from jtcores modules/jtframe/hdl/video/jtframe_draw.v (Date:
 * 18-12-2022) and the LATCH=1 input stage of jtframe_objdraw_gate.v, jtcores
 * commit 62cacc840340ad1fe7d6be481d0f9bbebe835e7d.  Modified for Konami
 * System GX, 2026-09-14 -- changes listed below and in
 * docs/LICENSE_USAGE_REPORT.md.  The zoom stepping (hz_cnt, readon/moveon,
 * HZONE) and the dot counter are upstream's.
 */
//============================================================================
//  One 16-dot row of one 16x16 System GX sprite tile into the line buffer
//
//  ---- what changed from jtframe_draw, and why ----------------------------
//  1. FIVE planes, not four.  spritelayout (k053246_k053247_k055673.cpp:621)
//     is 5 bpp and the fifth plane is a separate ROM region; the fetch
//     returns both.  Plane order is gx_rommap.svh's GX_SPR_PX:
//         pen = { p[7-n], d[31-n], d[23-n], d[15-n], d[7-n] }
//     MSB first, bit 7 of every plane byte is the leftmost dot.  Upstream's
//     ROM word is LSB first with the halves swapped (SWAPH), so neither its
//     bit order nor its half select carries over.
//  2. The ROM port is a req/ok handshake: `rom_req` is held with a stable
//     `rom_unit` until a one-clock `rom_ok`, and is low for at least one
//     clock before the next request.  An ok therefore always belongs to the
//     address standing while it was asked for.
//  3. A half that arrives while the dots of the half before it are still
//     going out waits in `nxt`.  Upstream latches straight into the shifter
//     and relies on its SDRAM being slower than eight dots; on this board
//     the fetch time depends on who else is on the arbiter.
//  4. Transparent and shadow pens are decided here.  Pen 0 is never written.
//     SHADOW pens go out on their own port, `buf_shd_we` / `buf_shd`, and not
//     into the solid buffer: pen 31 when the sprite has a shadow code (MAME
//     drawmode 1 draws the solid pens, drawmode 4 the shadow pen,
//     zdrawgfxzoom32GP, shdpen = granularity - 1), and every opaque pen when
//     the code is 1 and OBJSET1 bit 5 is clear (drawmode 5, the whole sprite
//     is a shadow, konamigx_v.cpp:544; preset 1 either way).  2026-09-14.
//  5. `abort`: the line this tile belongs to has run out of time.  Writes
//     stop at once, no further half is requested, and a request already on
//     the wire is waited out so the fetcher's handshake stays whole.
//  6. The inputs are latched on `draw` here (upstream's LATCH=1 stage)
//     rather than in a wrapper.
//  7. ZOOM IS MAME'S, ONE TILE AT A TIME (2026-09-14).  Upstream stepped a
//     zoom counter (hz_cnt, readon/moveon) across a whole sprite, so tile
//     edges fell between dots.  MAME rounds each tile's start and width on
//     its own (draw_yxloop_gx) and picks the source column of dot x as
//     (x * ((16 << 19) / width)) >> 19 (zdrawgfxzoom32GP).  gx_objscan now
//     hands over the tile's first dot and width (`tzw`); this module keeps
//     BOTH halves (change 3's single waiting half is gone) and reads the
//     column it needs, waiting at the dot if that half has not arrived.
//     MEASURED before: MAME frame 4480 95.2 %, gameplay 3700 96.96 %.
//============================================================================
`default_nettype none

module gx_objdraw #(
    // change 11 (MEASUREMENTS 132): overlap the NEXT tile's first fetch with
    // THIS tile's dots.  0 is the serial behaviour this module shipped with.
    parameter integer PIPE = 0
) (
    input  wire          rst,
    input  wire          clk,

    input  wire          draw,
    input  wire          abort,
    output wire          busy,
    // change 11: `busy` answers "can you take another tile?" and with PIPE it
    // drops while this one is still drawing.  `active` answers "is anything
    // still in flight?" and is what the line machinery must wait for -- using
    // `busy` for both closed the line early and threw the remaining dots away.
    output wire          active,
    input  wire [1:0]    fmt,         // 0 GX 5bpp, 1 GX6 6bpp, 2 RNG 4bpp, 3 LE2 8bpp (gx_sprfetch header)
    input  wire [16:0]   code,        // tile number, sub-tile bits already added (17 bits: RNG)
    input  wire [9:0]    xpos,        // this tile's first dot on screen
    input  wire [3:0]    ysub,
    input  wire [7:0]    tzw,         // change 7: this tile's width on screen
    input  wire          hflip,
    input  wire          vflip,
    input  wire [9:0]    attr,
    input  wire [1:0]    shd,
    // change 11: the line-buffer index this tile belongs to.  With PIPE the
    // scan may move on to the NEXT line while this tile's dots are still going
    // out, and gx_sprite's write address used to come from the scan's CURRENT
    // line -- so those dots would land in the wrong buffer.  The tile carries
    // its own index instead, and `dline_q` is what gx_sprite must write with.
    input  wire [7:0]    dline,
    output wire [7:0]    dline_q,
    input  wire          full_shadow_off, // OBJSET1 bit 5

    // --- ROM: unit = tile*32 + row*2 + half, gx_rommap.svh ----------------
    output wire [21:0]   rom_unit,
    output reg           rom_req,
    input  wire          rom_ok,
    input  wire [31:0]   rom_data4,   // gx_plane_word order: d[7:0] = region byte 0
    input  wire [31:0]   rom_data1,   // [7:0] plane 4, [15:8] plane 5 (GX6); 0 for RNG; LE2 planes 4-7

    // --- line buffer -----------------------------------------------------------
    output reg  [9:0]    buf_addr,
    output wire          buf_we,
    output wire [19:0]   buf_din,     // { shd, attr, pen8 }
    output wire          buf_shd_we,  // change 4: a shadow pixel at buf_addr
    output wire [1:0]    buf_shd      //           its shadow code (preset)
);

`include "gx_zoomtab.svh"

// ---- the LATCH=1 input stage ------------------------------------------------
reg  [16:0]   dr_code;
reg  [9:0]    dr_xpos, dr_attr;
reg  [3:0]    dr_ysub;
reg  [7:0]    dr_tzw;
reg           dr_hflip, dr_vflip, dr_draw;
reg  [1:0]    dr_shd;
reg  [7:0]    dr_dline;
reg           pre_bsy;

// ---- change 11: the staged tile --------------------------------------------
//  132.3 measured the cost of being serial: a tile is `latency + dots`, about
//  12 + 18 clocks, and the twelve are a stall with the line buffer idle.  The
//  ROM port is free during those eighteen dots, so the NEXT tile's first half
//  is fetched into `nx_hd` there and is already in hand when the tile starts.
//  Only the FIRST half is staged: the second is whatever the row cache kept
//  from the same burst, so it was never the expensive one.
reg  [16:0]   nx_code;
reg  [9:0]    nx_xpos, nx_attr;
reg  [3:0]    nx_ysub;
reg  [7:0]    nx_tzw;
reg           nx_hflip, nx_vflip;
reg  [1:0]    nx_shd;
reg  [63:0]   nx_hd;          // the staged tile's first half
reg           nx_live;        // a tile is staged
reg           nx_v;           // ... and its first half has arrived
reg           nx_arm;         // staged, waiting for hflip/ysub to settle
reg           nx_req;         // the outstanding ROM request belongs to it
reg  [7:0]    nx_dline;

always @(posedge clk) begin
    // 139: a `draw` that STAGING took must not also reach the serial start
    // branch.  It used to: staging fires, the tile also ends on that clock,
    // `pre_bsy` drops (the end condition reads the registered `nx_live`, still
    // 0), the input latch does NOT fire (`pre_bsy` was still 1) -- and one
    // clock later the leftover `dr_draw` starts a tile from `dr_*` that were
    // never latched, so its first half and `buf_addr` come from the tile
    // BEFORE.  618 times in one frame; 1,023 wrong dots.  It also closes a
    // zero-width staged tile drawing the previous tile a second time.
    dr_draw <= draw && !abort && !stage_now;
    // The dr_* input latch is in the draw block below (MEASUREMENTS 141).
end
assign dline_q = dr_dline;
// The scan may hand over the next tile once this one's own fetches are done
// and nothing is staged yet.  Its dots keep going out meanwhile; only the ROM
// port changes hands, and the line buffer still has exactly one writer.
wire can_stage = (PIPE != 0) && pre_bsy && !nx_live && !nx_arm && !aborting
                 && !abort && v0 && v1 && !rom_req && !second;
// change 11 / 139: the clock a tile is actually taken INTO the staging slot.
// It has to be visible to the two places below that would otherwise act on
// the registered `nx_live`, which is still 0 on this very clock.
wire stage_now = can_stage && draw && !abort;
assign busy   = (pre_bsy & ~can_stage) | dr_draw;
assign active = pre_bsy | dr_draw | nx_live | nx_arm;

// ---- the draw ---------------------------------------------------------------
reg  [63:0]   h0, h1;          // both halves, { p7 .. p0 } bytes (p7/p6 LE2 only)
reg           v0, v1;          // ... and whether each has arrived
reg           half;            // the half being fetched
reg           second;          // the second half has not been requested yet
reg           aborting;
reg           all_shadow;
reg  [7:0]    dcnt;            // dots of this tile written so far
reg  [23:0]   sacc;            // source column, 19-bit fraction
reg  [23:0]   stx;             // source stride per dot

wire [3:0]  src   = sacc[22:19];
wire [3:0]  srcf  = dr_hflip ? ~src : src;        // MAME flips by 15 - x
wire        have  = srcf[3] ? v1 : v0;
wire [63:0] hp    = srcf[3] ? h1 : h0;
wire [2:0]  ncol  = ~srcf[2:0];                   // bit 7 of a plane byte is dot 0
// Byte first, then the bit: Quartus 17.0 flags a concatenated index into the
// 40-bit vector (10027) though it addresses 32..39 exactly.
wire [7:0]  hp7   = hp[63:56];
wire [7:0]  hp6   = hp[55:48];
wire [7:0]  hp5   = hp[47:40];
wire [7:0]  hp4   = hp[39:32];
wire [7:0]  hp3   = hp[31:24];
wire [7:0]  hp2   = hp[23:16];
wire [7:0]  hp1   = hp[15: 8];
wire [7:0]  hp0   = hp[ 7: 0];
// LE2 (spritelayout3 { 56, 48, .. 0 }): plane k is byte k, the MSB byte 7
wire [7:0]  pen   = { hp7[ncol], hp6[ncol], hp5[ncol], hp4[ncol], hp3[ncol], hp2[ncol], hp1[ncol], hp0[ncol] };
// the shadow pen is the format's last pen: MAME shdpen = granularity - 1
// (k053246_k053247_k055673.cpp), 31 / 63 / 15 / 255.  Planes the format lacks are 0.
// As a REGISTERED mask: pen == shpen is &(pen | ~mask), because the planes a
// format lacks are always 0.  MEASURED, 63624628: the compare against a
// four-way mux of fmt sat on the core clock's worst path (sacc -> sbuf we,
// -0.254) once pens were 8 bits.  fmt is a per-set constant.
reg  [7:0]  shmask = 8'h1F;
always @(posedge clk)
    shmask <= (fmt == 2'd1) ? 8'h3F : (fmt == 2'd2) ? 8'h0F : (fmt == 2'd3) ? 8'hFF : 8'h1F;
wire        is_shpen = &(pen | ~shmask);
wire        dots  = (dcnt != dr_tzw);
wire        dot   = pre_bsy & ~aborting & ~abort & dots & have;

wire [3:0]  ysubf    = dr_ysub ^ {4{dr_vflip}};
wire [3:0]  nx_ysubf = nx_ysub ^ {4{nx_vflip}};
// change 11: while `nx_req` is up the port is fetching for the STAGED tile,
// whose first half is the one `half <= dr_hflip` would pick for it.
assign rom_unit = nx_req ? { nx_code, nx_ysubf, nx_hflip }
                         : { dr_code, ysubf,    half     };

// change 4
wire opaque       = (pen != 8'd0) && !all_shadow
                    && !((dr_shd != 2'd0) && is_shpen);
assign buf_we     = dot & opaque;
assign buf_din    = { dr_shd, dr_attr, pen };
wire shadow_px    = (pen != 8'd0) && (all_shadow || ((dr_shd != 2'd0) && is_shpen));
assign buf_shd_we = dot & shadow_px;
assign buf_shd    = dr_shd;       // all_shadow implies code 1

always @(posedge clk) begin
    // ---- the input latch, and it must live in THIS block (MEASUREMENTS 141) --
    //  The staged tile's promotion further down writes dr_* too.  The two are
    //  exclusive -- this fires on !pre_bsy, promotion only under pre_bsy -- so
    //  simulation was right, but two always blocks driving one register is not
    //  a circuit: Quartus map stopped on it (10028) the first time PIPE=1 was
    //  built, because with PIPE=0 the promotion folds away.  The SAME defect as
    //  137's nx_live / nx_v.  Placed before `rst` so it latches during reset
    //  exactly as it did in its own block.
    if( !pre_bsy ) begin
        dr_code  <= code;
        dr_xpos  <= xpos;
        dr_ysub  <= ysub;
        dr_tzw   <= tzw;
        dr_hflip <= hflip;
        dr_vflip <= vflip;
        dr_attr  <= attr;
        dr_shd   <= shd;
        dr_dline <= dline;
    end
    if( rst ) begin
        rom_req    <= 0;
        buf_addr   <= 0;
        pre_bsy    <= 0;
        second     <= 0;
        half       <= 0;
        v0         <= 0;
        v1         <= 0;
        aborting   <= 0;
        all_shadow <= 0;
        dcnt       <= 0;
        sacc       <= 0;
        stx        <= 0;
        h0         <= 64'd0;
        h1         <= 64'd0;
        nx_live    <= 0;
        nx_v       <= 0;
        nx_arm     <= 0;
        nx_req     <= 0;
        nx_hd      <= 64'd0;
    end else begin
    // ---- change 11, and it must live in THIS block -------------------------
    //  `nx_live` and `nx_v` were assigned here AND in the input-latch block
    //  above.  Two always blocks driving one register is last-writer-wins in
    //  simulation and not a circuit at all in synthesis; it cost 1,023 dots
    //  (MEASUREMENTS 137).  The staging latch is here now, first, so the abort
    //  and promotion branches below still take priority over it.
    //
    //  gx_objscan drives `hflip` and `ysub` as COMBINATIONAL outputs
    //  (gx_objscan.sv:72,75) off `hhalf`, which moves the clock AFTER a tile
    //  is handed over, so this closes in the clock `draw` is asserted.
    //  TIMING, and it is the whole of 137.  `hflip` and `ysub` are
    //  COMBINATIONAL scan outputs off `hhalf`, and the scan updates `hhalf`
    //  in the SAME clock it hands a tile over -- so they are only right from
    //  the clock AFTER `draw` rises.  The serial path latches while
    //  `!pre_bsy`, across both clocks, and therefore always ends up with the
    //  later value.  Staging in one clock took the earlier one and fetched
    //  the WRONG HALF FIRST on every mirrored tile.
    //
    //  So it is two stages: the registered fields on `draw`, and the two
    //  combinational ones a clock later, where they are settled.  Arming for
    //  one clock also keeps `busy` up, so the scan cannot hand over twice.
    if( stage_now ) begin
        nx_code  <= code;
        nx_xpos  <= xpos;
        nx_ysub  <= ysub;
        nx_tzw   <= tzw;
        nx_hflip <= hflip;
        nx_vflip <= vflip;
        nx_attr  <= attr;
        nx_shd   <= shd;
        nx_dline <= dline;
        nx_live  <= tzw != 8'd0;
        nx_v     <= 1'b0;
        nx_arm   <= 1'b0;
    end
    if( !pre_bsy ) begin
        aborting <= 0;
        // A tile rounded to no width draws nothing and fetches nothing, as in
        // MAME (a zero-width destination returns before any pixel).
        if( dr_draw && !abort && dr_tzw != 8'd0 ) begin
            half       <= dr_hflip;          // flipped: the right half is dot 0
            rom_req    <= 1;
            second     <= 1;
            v0         <= 0;
            v1         <= 0;
            nx_live    <= 0;
            nx_v       <= 0;
            pre_bsy    <= 1;
            dcnt       <= 0;
            sacc       <= 0;
            stx        <= gx_zstride(dr_tzw);
            buf_addr   <= dr_xpos;
            all_shadow <= (dr_shd == 2'd1) && !full_shadow_off;
        end
    end else if( aborting || abort ) begin
        // change 5
        aborting <= 1;
        second   <= 0;
        nx_live  <= 0;
        nx_v     <= 0;
        nx_arm   <= 0;
        if( rom_req && rom_ok ) begin rom_req <= 0; nx_req <= 0; end
        if( !rom_req || rom_ok ) pre_bsy <= 0;
    end else begin
        // change 2: one ok per request; the second is raised the clock after
        // the first is answered, so its address is new before req rises.
        if( rom_req && rom_ok ) begin
            rom_req <= 0;
            if( nx_req ) begin
                nx_req <= 0;
                nx_hd  <= { rom_data1, rom_data4 };
                nx_v   <= 1;
            end else if( half ) begin h1 <= { rom_data1, rom_data4 }; v1 <= 1; end
            else                begin h0 <= { rom_data1, rom_data4 }; v0 <= 1; end
        end else if( !rom_req && second ) begin
            second  <= 0;
            half    <= ~half;
            rom_req <= 1;
        end else if( (PIPE != 0) && !rom_req && !second && nx_live && !nx_v
                     && !nx_req && v0 && v1 ) begin
            // change 11: this tile is fully fetched and still drawing, so the
            // port is idle -- spend it on the staged tile's first half.  A
            // zero-width staged tile fetches nothing, as MAME draws nothing.
            if( nx_tzw != 8'd0 ) begin
                nx_req  <= 1;
                rom_req <= 1;
            end else nx_v <= 1;
        end
        if( dot ) begin
            buf_addr <= buf_addr + 1'd1;
            dcnt     <= dcnt + 1'd1;
            sacc     <= sacc + stx;
        end
        // The tile ends when its dots are out and no fetch is left on the wire.
        // change 11: a staged tile takes over in the same clock, already
        // holding its first half, so it begins at the dots instead of at a
        // twelve-clock stall.
        // change 11: a STAGED tile must never be dropped here.  The scan has
        // already handed it over and will not offer it again, so if its first
        // half has not arrived yet this tile stays busy and waits -- the
        // prefetch branch above starts or finishes it on one of these clocks.
        // Dropping it instead cost 2.3 % of the plane at a latency where the
        // serial path is perfect, which is how the bug was found.
        if( !dots && !rom_req && !second
            && !((PIPE != 0) && ((nx_live && !nx_v) || stage_now)) ) begin
            if( (PIPE != 0) && nx_live && nx_v ) begin
                dr_code  <= nx_code;
                dr_xpos  <= nx_xpos;
                dr_ysub  <= nx_ysub;
                dr_tzw   <= nx_tzw;
                dr_hflip <= nx_hflip;
                dr_vflip <= nx_vflip;
                dr_attr  <= nx_attr;
                dr_shd   <= nx_shd;
                dr_dline <= nx_dline;
                if( PIPE == 2 ) begin       // 137: isolate -- discard the prefetch
                    v0 <= 0; v1 <= 0;
                    half   <= nx_hflip;
                    second <= 1;
                end else begin
                    if( nx_hflip ) begin h1 <= nx_hd; v1 <= 1; v0 <= 0; end
                    else           begin h0 <= nx_hd; v0 <= 1; v1 <= 0; end
                    half   <= ~nx_hflip;    // the other half is still to come
                    second <= 0;
                end
                rom_req    <= nx_tzw != 8'd0;
                dcnt       <= 0;
                sacc       <= 0;
                stx        <= gx_zstride(nx_tzw);
                buf_addr   <= nx_xpos;
                all_shadow <= (nx_shd == 2'd1) && !full_shadow_off;
                nx_live    <= 0;
                nx_v       <= 0;
                pre_bsy    <= nx_tzw != 8'd0;
            end else pre_bsy <= 0;
        end
    end
    end
end

endmodule

`default_nettype wire
