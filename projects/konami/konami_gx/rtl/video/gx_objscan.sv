/* SPDX-FileCopyrightText: 2026 Jose Tejada Gomez
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Derived from jtcores cores/simson/hdl/jt053246_scan.sv (Date: 23-9-2024),
 * jtcores commit 62cacc840340ad1fe7d6be481d0f9bbebe835e7d.  Modified for
 * Konami System GX, 2026-09-14 -- every change is listed below and in
 * docs/LICENSE_USAGE_REPORT.md.  The table walk, the zoom tables, the
 * inzone/hsum/vsum arithmetic and the draw handshake are upstream's.
 */
//============================================================================
//  K053246/K055673 per-line sprite table scan -- System GX
//
//  One line at a time: walk the zcode-sorted table gx_sprite's DMA built,
//  decide which entries cover the line, and hand each covered 16x16 tile row
//  to gx_objdraw.  Upstream ships this in Run'n Gun and the Simpsons family;
//  root CLAUDE.md 1.2 shelf 2, docs/REUSE_PLAN.md section 1.
//
//  ---- what changed from jt053246_scan, and why --------------------------
//  1. SCAN_START is 0, not 0x40.  Upstream starts K55673 scans at slot 0x40
//     ("K55673 seems to have fewer objects ... second screen on Run'n Gun?").
//     MEASURED on gokuparo, tools/gx_objstat.py over 535 attract frames: up
//     to 47 on-screen entries a frame have zcode < 0x40.  They would never be
//     drawn.
//  2. The negedge-toggled `cen2` is a posedge toggle.  Same cadence -- the
//     FSM steps on every other clock while the arithmetic below it registers
//     every clock -- without a half-cycle path at 96 MHz.
//  3. The line is started by `start` with the line to draw in `vline`,
//     instead of an HS edge gated on Run'n Gun's vdump range, and `vline` is
//     used unsigned (upstream sign-extends a vdump that runs F8..1FF).
//  4. No left_wrap / HADJ.  Upstream's hdump starts at 0x20; gx_sprite's line
//     buffer is addressed from dot 0 with a 10-bit x, so a sprite left of the
//     screen wraps into invisible addresses by itself.
//  5. MIRROR FOLLOWS MAME, k053246_k053247_k055673.h:164 and :180-181:
//        mirror x forces flipx = 0, and screen flip X does not toggle it;
//        mirror y leaves flipy alone, and screen flip Y does not toggle it.
//     Upstream XORs the global flips in regardless.  That is invisible on a
//     board that never sets them and wrong on this one: gokuparo runs with
//     OBJSET1 bit 1 SET (0x22/0x32, MEASURED), so every Y-mirrored sprite
//     would draw its two halves the wrong way round.
//  6. $display bookkeeping and the debug bus are gone.
//  7. The Y-mirror half is decided at step 4 (see `vflip_c`).
//  8. The zoom tables were case functions; change 10 retired them.
//  9. Empty slots are SKIPPED, not walked (see `skip_next`).
// 10. ZOOM ROUNDS EACH TILE, as MAME's draw_yxloop_gx does, not the whole
//     sprite as one stream (see `recx`).  2026-09-14.
//
//  ---- coordinates -------------------------------------------------------
//  x: w3 - (xoffset - hoffset), 10 bits, is gx_sprite's line-buffer dot.
//  y: vline + w2 (or -w2 under screen flip Y) + yoffset + VOFFSET, 10 bits,
//     is the row inside the sprite, centred by `ymove`.  gx_sprite passes the
//     constants and says where they come from.
//============================================================================
`default_nettype none

module gx_objscan #(
    parameter [9:0] VOFFSET = 10'd0
) (
    input  wire        rst,
    input  wire        clk,
    // HOFFSET was a parameter until 2026-09-16; it is per set now (gokuparo
    // 954, sexyparo 958 -- gx_top, DECISIONS D18) and it is only ever used in
    // one registered subtraction, so an input costs nothing.
    input  wire [9:0]  hoffset,

    input  wire        start,          // one clock: begin the table for `vline`
    input  wire [8:0]  vline,
    output reg         done,

    // --- to gx_objdraw ------------------------------------------------------
    output reg  [15:0] code,
    output reg  [9:0]  attr,           // word 6 bits 9-0
    output wire        hflip,
    output reg         vflip,
    output reg  [9:0]  hpos,
    output wire [3:0]  ysub,
    output reg  [7:0]  tzw,            // change 10: this tile's width on screen
    output reg  [1:0]  shd,
    output reg         dr_start,
    input  wire        dr_busy,

    // --- the sorted table -----------------------------------------------------
    input  wire [15:0] scan_even,      // words 0 2 4 6 of the slot
    input  wire [15:0] scan_odd,       // words 1 3 5 7
    output wire [9:0]  scan_addr,      // {slot, word/2}

    // --- OBJSET1 --------------------------------------------------------------
    input  wire [9:0]  xoffset,
    input  wire [9:0]  yoffset,
    input  wire        ghf, gvf,

    // --- change 9: which of the 256 sorted slots the DMA filled ---------------
    input  wire [255:0] act
);

localparam [11:0] MAX_ZOOMIN = 12'd6; // "a value below 3 will break the pass scene in run&gun"

reg  [11:0] vzoom, hzoom;
reg  [ 9:0] y, y2, x, ydiff, ydiff_b, xadj, yadj, x2, xstart;
reg  [ 8:0] vlatch;
reg  [ 7:0] scan_obj;
reg  [ 3:0] size;
reg  [ 2:0] hstep, hcode, hsum, vsum;
reg  [ 1:0] scan_sub;
reg         inzone, hdone,
            vmir, hmir, sq, pre_vf, pre_hf, indr,
            hmir_eff, vmir_eff, hhalf;
reg         start_pend;

wire [ 1:0] nx_mir, hsz, vsz;

// Change 5: under X mirror MAME forces flipx = 0 and the screen flip does not
// reach it, so the flip of each tile is exactly "is this the right half".
assign hflip     = hmir ? hmir_eff : (ghf ^ pre_hf);
assign scan_addr = { scan_obj, scan_sub };
assign ysub      = ydiff[3:0];
assign nx_mir    = scan_even[15:14];
assign {vsz,hsz} = size;

//  ---- change 9: skip empty slots ------------------------------------------
//  MEASURED, sim/tb_gx_busmix.sv +linestat=1 (docs/MEASUREMENTS.md 23): on the
//  busiest attract lines the walk spent 554 clocks of a 6,144-clock line window
//  stepping through slots the DMA left empty -- one FSM step, two clocks, for
//  each of up to 256 -- while the line ran out of time.  An upper-bound model
//  that jumped straight to the next occupied slot took that to 104 and drew
//  frame 1850 with no late line.
//
//  `act` is gx_sprite's mirror of every slot's active bit.  The next occupied
//  slot strictly after `scan_obj` is found in two registered stages, which fit
//  because this FSM moves only on cen2 -- every other clock -- and `scan_obj`
//  changes only on those edges: stage A registers on the clock after a change,
//  stage B is read on the next cen2 edge.
//      byte_any / fb   per 8-slot byte: any slot filled, and the lowest one
//      A               the lowest filled slot above scan_obj in its own byte,
//                      and the lowest later byte with any
//  Where upstream walked slot + 1 and ended at slot 255, every walk step now
//  jumps to `skip_next` and ends when there is none.
reg  [31:0] byte_any;
reg  [2:0]  fb [0:31];
integer     sk_j, sk_b;
always @(posedge clk) begin
    for (sk_j = 0; sk_j < 32; sk_j = sk_j + 1) begin
        byte_any[sk_j] <= |act[sk_j*8 +: 8];
        fb[sk_j] <= 3'd0;
        for (sk_b = 7; sk_b >= 0; sk_b = sk_b - 1)
            if (act[sk_j*8 + sk_b]) fb[sk_j] <= sk_b[2:0];
    end
end

wire [4:0] cur_byte = scan_obj[7:3];
wire [8:0] above_m  = ~((9'd2 << scan_obj[2:0]) - 9'd1);    // bits above cur
wire [7:0] cur_rest = act[cur_byte*8 +: 8] & above_m[7:0];
reg        a_ib_any, a_nb_any;
reg  [2:0] a_ib_bit;
reg  [4:0] a_nb;
integer    sa_j, sa_b;
always @(posedge clk) begin
    a_ib_any <= |cur_rest;
    a_ib_bit <= 3'd0;
    for (sa_b = 7; sa_b >= 0; sa_b = sa_b - 1)
        if (cur_rest[sa_b]) a_ib_bit <= sa_b[2:0];
    a_nb_any <= 1'b0;
    a_nb     <= 5'd0;
    for (sa_j = 31; sa_j >= 0; sa_j = sa_j - 1)
        if (sa_j > cur_byte && byte_any[sa_j]) begin
            a_nb_any <= 1'b1;
            a_nb     <= sa_j[4:0];
        end
end
wire [8:0] skip_next = a_ib_any ? {1'b0, cur_byte, a_ib_bit}
                     : a_nb_any ? {1'b0, a_nb, fb[a_nb]}
                     :            9'h100;                   // none left

reg cen2;
always @(posedge clk) cen2 <= rst ? 1'b0 : ~cen2;

`include "gx_zoomtab.svh"

//  ---- change 10: MAME's per-tile zoom ---------------------------------------
//  k053246_k053247_k055673.h:146-157 and :316-322, zdrawgfxzoom32GP :464:
//      zoom      = (0x400000 + scale/2) / scale         (gx_zrecip)
//      centre    = (zoom * size) >> 13
//      tile k at (zoom * k + 0x800) >> 12, size = next tile's start - its own
//      source    = (offset * ((16 << 19) / size)) >> 19 (gx_zstride)
//  Upstream multiplied the whole sprite's row by one zoom and cut tiles out of
//  the result, so tile edges fell between dots: MAME frame 4480 95.2 %, the
//  gameplay start (3700) 96.96 %.
//
//  Every register below loads on every clock; the FSM samples on cen2 steps.
//      step 2 edge   vzoom / hzoom
//      +1            recx / recy
//      +2 (step 3)   py1..py8 (row tops), ydiff_b (the line from the top)
//      +3            trow, tin
//      +4 (step 4)   tsub, tzh
//      +5            srow                 -> read at step 5
reg  [23:0] recx, recy;
reg  [14:0] py1, py2, py3, py4, py5, py6, py7, py8;
reg  [2:0]  trow;
reg         tin;
reg  [7:0]  tsub, tzh;
reg  [3:0]  srow;
reg  [26:0] xacc;
reg  [10:0] tile_pos, tile_end;

function automatic [14:0] ztop(input [23:0] r, input [3:0] k);
    reg [27:0] m;
begin
    m    = {4'd0, r} * {24'd0, k} + 28'h800;
    ztop = m[26:12];
end
endfunction

function automatic [14:0] pyrow(input [2:0] k, input [14:0] a1, input [14:0] a2,
                                input [14:0] a3, input [14:0] a4, input [14:0] a5,
                                input [14:0] a6, input [14:0] a7);
begin
    case (k)
        3'd0: pyrow = 15'd0;  3'd1: pyrow = a1;  3'd2: pyrow = a2;  3'd3: pyrow = a3;
        3'd4: pyrow = a4;     3'd5: pyrow = a5;  3'd6: pyrow = a6;  3'd7: pyrow = a7;
    endcase
end
endfunction

wire [26:0] hx_s   = {3'd0, recx} << hsz;
wire [26:0] hy_s   = {3'd0, recy} << vsz;
wire [9:0]  half_x = hx_s[22:13];
wire [9:0]  half_y = hy_s[22:13];
wire [14:0] dl     = {5'd0, ydiff_b};
wire [14:0] py_h   = (vsz == 2'd0) ? py1 : (vsz == 2'd1) ? py2 : (vsz == 2'd2) ? py4 : py8;
wire [14:0] row_t  = pyrow(trow, py1, py2, py3, py4, py5, py6, py7);
wire [14:0] row_n  = (trow == 3'd7) ? py8 : pyrow(trow + 3'd1, py1, py2, py3, py4, py5, py6, py7);
wire [14:0] tsub_w = dl - row_t;
wire [14:0] tzh_w  = row_n - row_t;
wire [31:0] srow_m = {24'd0, tsub} * {8'd0, gx_zstride(tzh)};
wire [27:0] xpos_r = {1'b0, xacc} + 28'h800;
wire [27:0] xend_r = {1'b0, xacc} + {4'd0, recx} + 28'h800;
wire [10:0] tile_w_11 = tile_end - tile_pos;
wire [7:0]  tile_w = tile_w_11[7:0];

reg  [2:0]  zr;
always @(*) begin
    zr = 3'd0;
    if ((vsz != 2'd0) && (dl >= py1)) zr = 3'd1;
    if ((vsz >= 2'd2) && (dl >= py2)) zr = 3'd2;
    if ((vsz >= 2'd2) && (dl >= py3)) zr = 3'd3;
    if ((vsz == 2'd3) && (dl >= py4)) zr = 3'd4;
    if ((vsz == 2'd3) && (dl >= py5)) zr = 3'd5;
    if ((vsz == 2'd3) && (dl >= py6)) zr = 3'd6;
    if ((vsz == 2'd3) && (dl >= py7)) zr = 3'd7;
end

always @(posedge clk) begin
    xadj <= xoffset - hoffset;
    yadj <= yoffset + VOFFSET;
    recx <= gx_zrecip(hzoom[9:0]);
    recy <= gx_zrecip(vzoom[9:0]);
    ydiff_b <= y2 + { 1'b0, vlatch };
    py1 <= ztop(recy, 4'd1);  py2 <= ztop(recy, 4'd2);  py3 <= ztop(recy, 4'd3);
    py4 <= ztop(recy, 4'd4);  py5 <= ztop(recy, 4'd5);  py6 <= ztop(recy, 4'd6);
    py7 <= ztop(recy, 4'd7);  py8 <= ztop(recy, 4'd8);
    trow <= zr;
    tin  <= !ydiff_b[9] && (dl < py_h);
    tsub <= tsub_w[7:0];
    // a row taller than 255 dots (vertical scale below 4) saturates
    tzh  <= (tzh_w > 15'd255) ? 8'd255 : tzh_w[7:0];
    srow <= srow_m[22:19];
    tile_pos <= xpos_r[22:12];
    tile_end <= xend_r[22:12];
end

//  ---- change 7: the Y-mirror half is decided at step 4, not step 3 ---------
//  Upstream registers `vflip` at step 3 from `vmir_eff`, which reads `ydiff`.
//  But `ydiff` is yz_add, two registers downstream of the `y` and `vzoom`
//  that step 2 loads: at step 3 it still holds the row of the PREVIOUS entry
//  in the table.  MEASURED, tb_gx_sprite case "2x2 mirror y": every one of
//  the sprite's 1024 dots drawn from the wrong half.  At step 4 -- where
//  upstream already reads `inzone` and `vsum` from the same `ydiff` -- it is
//  this entry's row.  By then `scan_even` has moved on to the next word, so
//  the mirror bit comes from `vmir`, registered at step 3.
reg vflip_c;

always @* begin : B
    y2        = y + half_y;                        // change 10
    ydiff     = { 3'd0, trow, srow };
    x2        = x - half_x;
    case( vsz )
        0: vmir_eff = vmir && !ydiff[3];
        1: vmir_eff = vmir && !ydiff[4];
        2: vmir_eff = vmir && !ydiff[5];
        3: vmir_eff = vmir && !ydiff[6];
    endcase
    // change 5: MAME keeps flipy under Y mirror and does not apply the screen
    // flip to it (h:180-181).
    vflip_c = vmir ? (pre_vf ^ vmir_eff) : (pre_vf ^ gvf);
    hmir_eff = hmir & hhalf;
    inzone = tin;                                  // change 10
    case( hsz )
        0: hdone = 1;
        1: hdone = hstep==1;
        2: hdone = hstep==3;
        3: hdone = hstep==7;
    endcase
    case( hsz )
        0: hsum = 0;
        1: hsum = hmir ? 3'd0                           : {2'd0,hstep[0]^hflip};
        2: hsum = hmir ? {2'd0,hstep[0]^hflip}          : {1'd0,hstep[1:0]^{2{hflip}}};
        3: hsum = hmir ? ({1'b0,hstep[1:0]^{2{hflip}}}) : hstep[2:0]^{3{hflip}};
    endcase
    case( vsz )
        0: vsum = 0;
        1: vsum = { 2'd0, ydiff[4]^vflip_c   };
        2: vsum = { 1'd0, ydiff[5:4]^{2{vflip_c}} };
        3: vsum = ydiff[6:4]^{3{vflip_c}};
    endcase
end

// Table scan
always @(posedge clk) begin : A
    if( rst ) begin
        scan_obj   <= 0;
        scan_sub   <= 0;
        hstep      <= 0;
        code       <= 0;
        attr       <= 0;
        pre_vf     <= 0;
        pre_hf     <= 0;
        vflip      <= 0;
        vzoom      <= 0;
        hzoom      <= 0;
        tzw        <= 0;
        indr       <= 0;
        hhalf      <= 0;
        shd        <= 0;
        done       <= 1;
        dr_start   <= 0;
        start_pend <= 0;
        vlatch     <= 0;
        hmir       <= 0;
        vmir       <= 0;
    end else begin
        // `start` is one system clock; the FSM below only moves on cen2.
        if( start ) start_pend <= 1;
        if( cen2 ) begin
            dr_start <= 0;
            if( start_pend ) begin
                start_pend <= 0;
                done       <= 0;
                scan_obj   <= 0;               // change 1
                scan_sub   <= 0;
                indr       <= 0;
                hmir       <= 0;
                vlatch     <= vline;           // change 3
            end else if( !done ) begin
                {indr, scan_sub} <= {indr, scan_sub} + 1'd1;
                case( {indr, scan_sub} )
                    0: begin
                        hhalf <= 0;
                        hmir  <= 0;
                        { sq, pre_vf, pre_hf, size } <= scan_even[14:8];
                        code    <= scan_odd;
                        hstep   <= 0;
                        if( !scan_even[15] ) begin
                            scan_sub <= 0;
                            scan_obj <= skip_next[7:0];
                            if( skip_next[8] ) done <= 1;
                        end
                    end
                    1: begin
                        y <= gvf ? -scan_even[9:0] : scan_even[9:0];
                        x <= ghf ? -scan_odd[ 9:0] : scan_odd[ 9:0];
                        hcode <= {code[4],code[2],code[0]};
                        hstep <= 0;
                    end
                    2: begin
                        x <= x-xadj;
                        y <= y+yadj;
                        vzoom <= {2'b0, scan_even[9:0]};
                        hzoom <= sq ? {2'b0, scan_even[9:0]} : {2'b0, scan_odd[9:0]};
                    end
                    3: begin
                        { vmir, hmir } <= nx_mir;
                        { shd, attr } <= scan_even[11:0];   // bits 13-12 "reserved", never read
                        if( hzoom < MAX_ZOOMIN ) begin
                            { indr, scan_sub } <= 0;
                            scan_obj <= skip_next[7:0];
                            if( skip_next[8] ) done <= 1;
                        end
                    end
                    // change 10: the per-tile row arithmetic is two clocks deeper
                    // than upstream's one multiply, so one step waits for it.
                    4: ;
                    5: begin
                        xacc <= 27'd0;
                        // Add the vertical offset to the code, must wait for zoom
                        // calculations, so it cannot be done at step 3
                        {code[5],code[3],code[1]} <= {code[5],code[3],code[1]} + vsum;
                        vflip  <= vflip_c;                     // change 7
                        xstart <= x2;
                        if( ~inzone ) begin
                            { indr, scan_sub } <= 0;
                            scan_obj <= skip_next[7:0];
                            if( skip_next[8] ) done <= 1;
                        end
                    end
                    default: begin // in draw state
                        case( hsz )
                            1: if(hstep>=1) hhalf <= 1;
                            2: if(hstep>=2) hhalf <= 1;
                            3: if(hstep>=4) hhalf <= 1;
                            default: ;
                        endcase
                        {indr, scan_sub} <= 3'd6; // stay here
                        if( (!dr_start && !dr_busy) || !inzone ) begin
                            {code[4],code[2],code[0]} <= hcode + hsum;
                            // change 10: this tile's first dot and width, rounded
                            // on their own as MAME rounds them
                            hpos <= xstart + tile_pos[9:0];
                            tzw  <= tile_w;
                            xacc <= xacc + {3'd0, recx};
                            hstep <= hstep + 1'd1;
                            dr_start <= inzone;
                            if( hdone || !inzone ) begin
                                { indr, scan_sub } <= 0;
                                scan_obj <= skip_next[7:0];
                                indr     <= 0;
                                if( skip_next[8] ) done <= 1;
                            end
                        end
                    end
                endcase
            end
        end
    end
end

endmodule

`default_nettype wire
