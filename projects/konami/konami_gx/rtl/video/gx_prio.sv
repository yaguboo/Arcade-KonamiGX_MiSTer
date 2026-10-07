//============================================================================
//  Konami 055555 "5^5" -- 8-input per-pixel priority encoder
//
//  This is the design centrepiece of the board and the one chip that exists
//  nowhere else, so the reasoning is written out rather than assumed.
//
//  ---- why this is not a port of MAME ---------------------------------------
//  MAME's k055555.cpp is 160 lines of which none are priority logic: a
//  128-byte register file, a byte write decode and two accessors.  It says so
//  itself.  All the behaviour lives in konamigx_v.cpp as a std::vector of up
//  to 518 objects, std::stable_sort'ed on a packed 32-bit key and drawn
//  back-to-front through a z-buffer.
//
//  That structure is a workaround for software not having eight simultaneous
//  pixel streams.  The chip's own header (k055555.cpp:5-22) describes what it
//  actually is:
//
//      "5-bit-per-pixel Priority encoder... has 8 inputs: A B C D intended for
//       a 156/157 type tilemap chip, OBJ intended for a '246 type sprite chip,
//       and SUB1-SUB3... each input can be chosen to participate in shadow/
//       highlight operations, R/G/B alpha blending, and R/G/B brightness"
//
//  In an FPGA the eight streams exist by construction.  So this is a
//  comparator, and the sort is not ported.  docs/REUSE_PLAN.md section 3, and
//  the standing rule in docs/IMPLEMENTATION_TODO.md: a screen that looks right
//  because layers were reordered by hand is a defect.
//
//  What IS ported literally is the colour-index assembly --
//  K055555GX_decode_vmixcolor (gxv.cpp:165), K055555GX_decode_osmixcolor
//  (:194), K055555GX_decode_objcolor (:78) and K055555GX_decode_inpri (:90).
//  Those are combinational descriptions of the chip, not workarounds.
//
//  ---- which way is "in front" ---------------------------------------------
//  LOWER priority value is in front.  Derivation, since getting this backwards
//  inverts the whole screen:
//
//    * konamigx_v.cpp:610 sorts the object pool DESCENDING by a key whose top
//      byte is the priority, and draws in that order.  Drawn first = furthest
//      back.  So the largest priority value is the backmost.
//    * gokuparo's measured registers agree with that reading of the screen:
//      A=0xFF (backmost), D=0xC0, C=0x80, B=0x40, OBJ=0x00 (frontmost).
//      docs/MEASUREMENTS.md section 3.
//
//  K55_CTL_FLIPPRI (control bit 2) reverses it.  MAME has that line commented
//  out with "not used by any GX game?" -- implemented here because it is one
//  XOR and leaving it out would be a silent difference.  UNVERIFIED.
//
//  ---- ties ----------------------------------------------------------------
//  Equal priorities need a fixed order.  For the four tilemap planes MAME's
//  pre-sort (gxv.cpp:437) uses `if (layerpri[j] <= layerpri[i]) swap`, so on a
//  tie the LATER letter ends up drawn first, i.e. behind.  That gives A in
//  front of B in front of C in front of D, and this module reproduces it by
//  making the input index the low bits of the comparison key.
//
//  Where OBJ and SUB1-3 sit in that order is NOT established by any source we
//  have -- in MAME sprites are separate objects whose tie-break is decided by
//  emulator bookkeeping (insertion order and a std::reverse), which is not
//  evidence about silicon.  The order used here is the chip's own input order
//  A B C D OBJ S1 S2 S3.  TODO(HARDWAREIZE): the Konami System GX manual would
//  settle it -- docs/UPSTREAM_TODO_AUDIT.md section F.
//
//  ---- two outputs, not one ------------------------------------------------
//  The '338 blends the winner against what is behind it, so a runner-up must
//  be carried.  MAME gets this for free by drawing into a bitmap in order; we
//  have to be explicit.  docs/REUSE_PLAN.md section 3.
//
//  ---- the palette index ---------------------------------------------------
//  MAME overrides the K056832 tile granularity to 16, but the decoded tile
//  pen is still five bits wide:
//
//      index = colour_code * 16 + pixel[4:0]
//
//  The ADDITION matters.  pixel[4] is a carry into the next 16-entry colour
//  block; it is not discarded.  Concatenating {colour_code,pixel[3:0]} aliases
//  pens 0..15 with pens 16..31 and collapses the colours while leaving the
//  tile geometry intact.  That exact failure reached hardware on 2026-09-13.
//
//  Hardware already settled the scale itself: x32 produced a non-zero winner
//  but no final RGB, while x16 produced a stable full-width tile scene.  The
//  remaining bug was that the experimental x16 path had been implemented as
//  truncation instead of MAME's arithmetic.  docs/DECISIONS.md D5.
//
//  ---- COLSET is decoded and deliberately unused ---------------------------
//  Registers 2-5 are "colour depth select" for the input pairs (A,B) (C,D)
//  (OBJ,S1) (S2,S3), one nibble each.  Gokujou Parodius writes 0x11 to all
//  four, i.e. every input gets code 1, and MAME never reads them at all.
//  One observed value cannot establish a decode table, and inventing one would
//  put a guess in the colour path where a wrong answer is invisible until
//  something looks slightly off.  So the registers are stored, the width is a
//  parameter, and this is logged.
//  TODO(HARDWAREIZE): what does the COLSET nibble mean?
//============================================================================
`default_nettype none

module gx_prio #(
    // Bits per pixel of every input.  Everything on System GX Type 2 is 5 bpp:
    // charlayout5 for tiles and spritelayout for sprites, both in
    // docs/SOURCE_AUDIT.md section 12.  See the COLSET note above.
    // PXBITS is the WIDTH of a pixel out of the tilemap, and it is still 5:
    // the data really is 5 bpp (docs/MEASUREMENTS.md section 10 --
    // 321b12.13g is non-zero on 45.7% of its rows).  What is switchable below
    // is the colour-code stride.  All five pixel bits still participate in
    // the addition when that stride is 16; the top bit becomes a carry.
    parameter integer PXBITS = 5,
    // Gradient backdrop origin.  fill_backcolor indexes base + the BITMAP
    // coordinate (k054338.cpp:134-150), whose visible area starts at (24,16),
    // while hpos/vpos here start at 0.  The board top passes the crop origin;
    // unit benches keep 0.  EMULATION_DERIVED -- see the gx_top instance.
    parameter [9:0] BG_X_OFFSET = 10'd0,
    parameter [8:0] BG_Y_OFFSET = 9'd0
) (
    input  wire        clk,
    input  wire        rst,

    // --- pixel enable -------------------------------------------------------
    //  THE OUTPUT REGISTERS BELOW ARE GATED WITH THIS, and that is a timing
    //  decision as much as a functional one.
    //
    //  Every data input of the comparator changes only on `pxl_cen`: the
    //  tilemap's `shift` registers and its `cur_col`/`cur_mix` latches are
    //  loaded on it, and hpos/vpos come from counters that advance on it.  The
    //  outputs used to be clocked on every 96 MHz edge, so the whole 8-input
    //  comparator had to settle in one 10.4 ns clock while its inputs moved
    //  once every 16.
    //
    //  MEASURED, build E, 2026-09-07 -- all five worst paths in the design:
    //      gx_tilemap|shift[1][4] -> gx_prio|idx1[7]      slack -7.001
    //
    //  Gating makes the capture rate match the launch rate, which is what lets
    //  targets/mister/KonamiGX.sdc relax those paths honestly.
    //
    //  COST: ONE DOT OF LATENCY.  idx0/idx1 now update on the pxl_cen edge
    //  AFTER their inputs change, not on the system clock after.  The picture
    //  therefore sits one pixel later relative to the CCU's blanking than it
    //  did.  That is the same axis as UPSTREAM_TODO U5/U6 -- MAME covers the
    //  GX raster with hardcoded offsets (-2,0,2,3) that this core deliberately
    //  does not port -- so the alignment is an open P4 question either way and
    //  this is one more known term in it.  Nothing has been on hardware yet,
    //  so no previously-correct alignment is being broken.
    input  wire        pxl_cen,

    // --- CPU port -----------------------------------------------------------
    //  d50000-d500ff.  K055555_long_w (k055555.cpp:79) puts register 2n in bits
    //  31-24 of long n and register 2n+1 in bits 15-8, so on the 16-bit port
    //  the register index IS the word address and the data is always D[15:8].
    //  Same lane convention as the CCU and the K056800.
    input  wire        reg_cs,
    input  wire        reg_we,
    input  wire [7:0]  reg_addr,       // cpu_addr[8:1]
    input  wire [7:0]  reg_din,        // cpu_dout[15:8]

    // --- D5: how the tilemap palette index is scaled -------------------------
    //  0 = colour_code * 32 + pixel[4:0]   what this board has always done
    //  1 = colour_code * 16 + pixel[4:0]   what MAME does
    //
    //  Switchable at RUNTIME, from the OSD, because it is settled by looking
    //  at one screen and a rebuild costs the better part of an hour.  Same
    //  reasoning as the video-source bisection in KonamiGX.sv.
    //
    //  It applies to the four tilemaps and the three sub-layers ONLY.  The
    //  sprite input is granularity 32 under both readings -- its colour comes
    //  from decode_objcolor, which already divides by coregshift -- so
    //  switching it too would make the experiment test two things at once.
    //
    //  docs/DECISIONS.md D5.  Hardware settled the scale at 16 on 2026-09-13;
    //  the switch remains only as the negative control used by that test.
    input  wire        pal_gran16,

    // --- backdrop source select ---------------------------------------------
    //  eeprom_w bit 29 = m_gx_wrport1_0 & 0x20 (gxv.cpp:345):
    //    0 -> the K054338's own solid background colour registers
    //    1 -> the unified 338/5^5 fill, which indexes the palette
    //  Gokujou Parodius writes 0xA0 here, so 1.  docs/MEASUREMENTS.md sec. 6.
    input  wire        bgc_from_pal,

    // --- raster, for the gradient backdrop ----------------------------------
    input  wire [9:0]  hpos,           // column within the active area
    input  wire [8:0]  vpos,           // line within the active area

    // --- tilemap inputs, straight from gx_tilemap ---------------------------
    // 6 bits since 2026-09-30: salmndr2's tiles are 6 bpp (K056832_BPP_6);
    // the 5 bpp sets leave bit 5 at 0
    input  wire [7:0]  px_a, px_b, px_c, px_d,     // 8 bits since 2026-10-05: winspike (K056832_BPP_8)
    input  wire [7:0]  col_a, col_b, col_c, col_d,

    // --- sprite input -------------------------------------------------------
    //  c18 is the K053247 side of the split (K053247GX_combine_c18, gxv.cpp:66,
    //  "see p.46"); the sprite chip owns that arithmetic because it needs OPSET
    //  and OBJSET2.  What arrives here is c18 and the shift, and this module
    //  does the two 5^5 halves: decode_objcolor and decode_inpri.
    input  wire [7:0]  px_o,
    // sprite ROM format (gx_sprfetch): 0 GX 5bpp, 1 GX6 6bpp, 2 RNG 4bpp, 3 LE2 8bpp.  It
    // picks MAME's colour granularity (1 << bpp: index = colour % (8192 >> bpp)
    // * granularity + pen, konami_helper.cpp / k053246_k053247_k055673.cpp) and
    // the set's sprite priority callback:
    //   0 type2_sprite_callback     decode_inpri(c18)
    //   1 salmndr2_sprite_callback  pri = attr >> 4 & 0x3f
    //   2 dragoonj_sprite_callback  pri = attr & 0x200 ? 4 : attr >> 4 & 0xf
    // (konamigx_v.cpp), each then (pri & ~OINPRI_ON) | (OBJ_PRI & OINPRI_ON).
    input  wire [1:0]  obj_fmt,
    input  wire [9:0]  obj_attr,       // the sprite's raw attribute word, bits 9-0
    input  wire [15:0] obj_c18,
    input  wire [3:0]  obj_coregshift, // 4..8, from OPSET & 7 clamped to 4
    input  wire [1:0]  obj_shd,        // sprite shadow code, 0 = none (the lowest of obj_shdm)
    input  wire [3:1]  obj_shdm,       // every shadow code present at the dot (STACKED SHADOWS)
    input  wire        obj_sdsel,      // OPSET bit 5 ("see p.51 OPSET SDSEL")
    input  wire        obj_shd_front,  // the shadow is nearer in z than the sprite pixel (gx_sprite)

    // --- sub inputs.  Type 2 has no ROZ, so gx_top ties these off; the ports
    //     exist because the chip has eight inputs and a five-input comparator
    //     would have to be rebuilt for the first Type 1 or Type 3 board.
    input  wire [14:0] px_s,           // {S3, S2, S1}, 5 bits each
    input  wire [23:0] col_s,          // {S3, S2, S1}, 8 bits each

    // --- results ------------------------------------------------------------
    output reg  [12:0] idx0,           // winner palette index
    output reg  [12:0] idx1,           // runner-up, needed for blending
    output reg         bg0,            // winner is the '338 solid backdrop
    output reg         bg1,            // runner-up is the '338 solid backdrop
    output reg  [1:0]  blend,          // winner's alpha preset, 0 = opaque
    output reg  [1:0]  bri,            // winner's brightness select, 0 = none
    output reg  [1:0]  bri1,           // runner-up's, for the blend's back (MEASUREMENTS 161)
    output reg  [1:0]  shadow,         // shadow preset over the winner, 0 = none
    //  The same two codes ONE STAGE EARLIER, for gx_colmix's K054338 register
    //  selection: jt054338 picks the level from the code combinationally and
    //  gx_colmix registers that level on pxl_cen, so it must see the code one
    //  dot before `blend` / `shadow` do (MEASUREMENTS 46, R1).
    output wire [1:0]  blend_e,
    output wire [1:0]  shadow_e,
    //  STACKED SHADOWS: every preset that lands, on `shadow`'s dot, and one
    //  stage earlier for gx_colmix's summed delta.  `shadow` is the lowest.
    output reg  [3:1]  shadow_m,
    output wire [3:1]  shadow_me,
    output reg  [3:0]  dbg_winner,     // which input won, 8 = backdrop
    // The ENABLE register itself (reg 45), live, in the pixel domain.
    // MEASURED 2026-09-08: the comparator never let anything beat the
    // backdrop while the tilemap WAS producing pixels, and `opaque` is
    // `(px != 0) && enable[gi]`, so this is the half that has to be read
    // out rather than reasoned about.
    output wire [7:0]  dbg_enable
);

// ---------------------------------------------------------------------------
//  register file
//
//  The chip decodes 128 bytes; 46 are named (k055555.cpp:57).  Storing 64
//  covers every named register with room to spare and costs half the flops of
//  the full 128.  Writes above 63 are dropped rather than wrapped -- a wrap
//  would silently corrupt a live priority register.
// ---------------------------------------------------------------------------
localparam [5:0] R_BGC_CBLK    =  6'd0;
localparam [5:0] R_CONTROL     =  6'd1;
localparam [5:0] R_PRI_A       =  6'd7;
localparam [5:0] R_PRI_B       =  6'd10;
localparam [5:0] R_PRI_C       =  6'd13;
localparam [5:0] R_PRI_D       =  6'd14;
localparam [5:0] R_PRI_OBJ     =  6'd15;
localparam [5:0] R_PRI_S1      =  6'd16;
localparam [5:0] R_OINPRI_ON   =  6'd19;
localparam [5:0] R_PALBASE_A   =  6'd23;
localparam [5:0] R_PALBASE_OBJ =  6'd27;
localparam [5:0] R_VINMIX      =  6'd33;
localparam [5:0] R_VINMIX_ON   =  6'd34;
localparam [5:0] R_OSINMIX     =  6'd35;
localparam [5:0] R_OSINMIX_ON  =  6'd36;
localparam [5:0] R_SHDPRI1     =  6'd37;
localparam [5:0] R_SHD_ON      =  6'd40;
localparam [5:0] R_SHDPRISEL   =  6'd41;
localparam [5:0] R_VBRI        =  6'd42;
localparam [5:0] R_OSBRI       =  6'd43;
localparam [5:0] R_OSBRI_ON    =  6'd44;
localparam [5:0] R_ENABLE      =  6'd45;

reg [7:0] regs [0:63];

// ---------------------------------------------------------------------------
//  The configuration, brought into the pixel domain
//
//  `regs` is written by the CPU on ITS enable, so it can change on any 96 MHz
//  edge relative to a dot.  Every read below goes through this copy instead,
//  which changes only on `pxl_cen`.
//
//  MEASURED, build F, 2026-09-07 -- after the tilemap-to-comparator paths were
//  relaxed, the five worst paths in the design became:
//
//      gx_prio|regs[45][0] -> gx_prio|idx1[7]      slack -8.726
//
//  i.e. the configuration feeding the same 8-input comparator.  That path was
//  deliberately NOT covered by the exception added with the previous commit,
//  because a multicycle from a CPU-written register would have been a false
//  promise -- the CPU can write one clock before a dot edge.  The fix is to
//  make the statement TRUE rather than to widen the constraint until it is
//  convenient: with the comparator reading only `pregs`, every one of its
//  inputs now moves at the dot rate.
//
//  The two halves are both honest.  regs -> pregs is a register-to-register
//  copy with no logic between, so it closes in one clock easily; pregs -> idx
//  is the comparator and it gets the dot-rate exception.
//
//  Cost: 512 flip-flops, and a configuration write takes effect on the next
//  dot instead of the next system clock.  A one-dot delay on a register the
//  CPU writes during blanking is not observable.
//  `bgc_from_pal` is deliberately NOT copied.  It is CPU-written too, but it
//  reaches only bg0/bg1 through a single AND, so its path is nowhere near
//  critical and a shadow register would be a flip-flop spent on nothing.
reg [7:0] pregs [0:63];
integer   pi;
always @(posedge clk) begin
    if (rst)
        for (pi = 0; pi < 64; pi = pi + 1) pregs[pi] <= 8'd0;
    else if (pxl_cen)
        for (pi = 0; pi < 64; pi = pi + 1) pregs[pi] <= regs[pi];
end

// obj_fmt in the pixel domain, like pregs: a per-set constant, but as a plain
// input it is a 96 MHz source into the comparator and build f5baea90 failed
// on exactly that (obj_fmt -> idx1, -7.659 ns, the 400 worst paths).  The
// name keeps `pregs` so KonamiGX.sdc's *gx_prio*pregs* multicycle covers it.
reg [1:0] pregs_fmt = 2'd0;
always @(posedge clk) if (pxl_cen) pregs_fmt <= obj_fmt;

integer ri;
always @(posedge clk) begin
    if (rst) begin
        for (ri = 0; ri < 64; ri = ri + 1) regs[ri] <= 8'd0;
    end else if (reg_cs && reg_we && (reg_addr[7:6] == 2'b00)) begin
        regs[reg_addr[5:0]] <= reg_din;
    end
end

wire [7:0] enable     = pregs[R_ENABLE];       // (MSB) S3 S2 S1 OB VD VC VB VA
assign     dbg_enable = enable;
wire [7:0] control    = pregs[R_CONTROL];
wire       graddir    = control[0];           // 0 vertical, 1 horizontal
wire       gradenable = control[1];
wire       flippri    = control[2];

wire [7:0] vinmix     = pregs[R_VINMIX];
wire [7:0] vmixon     = pregs[R_VINMIX_ON];
wire [7:0] osinmix    = pregs[R_OSINMIX];
wire [7:0] osmixon    = pregs[R_OSINMIX_ON];
wire [7:0] vbri       = pregs[R_VBRI];
wire [7:0] osbri      = pregs[R_OSBRI];
wire [7:0] osbrion    = pregs[R_OSBRI_ON];
wire [7:0] shd_on     = pregs[R_SHD_ON];
wire [7:0] shdprisel  = pregs[R_SHDPRISEL];
wire [7:0] oinprion   = pregs[R_OINPRI_ON];

// ---------------------------------------------------------------------------
//  colour-index assembly -- K055555GX_decode_vmixcolor, gxv.cpp:165
//  ("see p.62 7.2.6 and p.27 3.3")
//
//      vcb  = PALBASE[n] << 6
//      von  = VINMIX_ON >> 2n & 3      which of colour bits 4-5 are palette
//      vmx  = VINMIX    >> 2n & 3
//      pl45 = colour >> 4 & 3
//      colour_code = (colour & 0xf) | ((pl45 & von) << 4) | vcb
//      mix         = (pl45 & ~von) | (vmx & von)
//
//  Read plainly: VINMIX_ON chooses, one bit at a time, whether colour bits 4
//  and 5 are palette bits (and the blend bit then comes from the VINMIX
//  register) or blend bits (and the palette loses them).
//
//  MAME reaches the same place by two different routes -- decode_vmixcolor
//  above, and gx_draw_basic_tilemaps (gxv.cpp:695) which splits internal
//  (VINMIX & VINMIX_ON) from external (tile attribute bits 6-7 & ~VINMIX_ON).
//  Those two agree here because gokuparo runs with FBITS=3, where the k056832
//  shift/mask set puts attribute bits 6-7 on colour bits 4-5.  Checked against
//  the measured trace: k056832 word 3 = 0x00D0, so FBITS = 3 (m_regs[3] >> 6;
//  word 1, which 73b582c read for it, is the tile-flip enable).
//
//  gokuparo writes VINMIX_ON = 0xFF and VINMIX = 0x00, so on this game every
//  layer takes all six colour bits into the palette and blends nothing.
// ---------------------------------------------------------------------------
//  ---- PALBASE IS THREE BITS, 2026-09-15 (MEASUREMENTS 46, R2) ---------------
//  This took palbase[1:0].  vcb = PALBASE << 6 (gxv.cpp:169) is the colour
//  code's bits 8:6 into an 8,192-entry palette (konamigx.cpp:1755), so PALBASE
//  bit 2 is palette bit 12.  MEASURED: during the explosion flash the game sets
//  PALBASE_A = 0x07; MAME indexes 0x1C1E (black), the board 0x0C1E (the blue
//  double line at y 8-12, shot 6302: board == this function's old output on
//  every pixel of the box).  OBJ already used OBJ_PALBASE[2:0].
function automatic [8:0] vmix_colour(input [7:0] pal, input [7:0] palbase, input [1:0] von);
begin
    vmix_colour = { palbase[2:0], pal[5:4] & von, pal[3:0] };
end
endfunction

function automatic [1:0] vmix_blend(input [7:0] pal, input [1:0] von, input [1:0] vmx);
begin
    vmix_blend = (pal[5:4] & ~von) | (vmx & von);
end
endfunction

// ---------------------------------------------------------------------------
//  sprite colour and priority -- the 5^5 half of the split
//
//  K055555GX_decode_objcolor (gxv.cpp:78, "see p.59 7.2.2"):
//      opon = (OINPRI_ON << 8) | 0xff
//      objcolor = ((((OBJ_PALBASE & 7) << 10) & ~opon) | (c18 & opon)) >> shift
//
//  K055555GX_decode_inpri (gxv.cpp:90):
//      inpri = ((c18 >> 8) & ~OINPRI_ON) | (OBJ_PRI & OINPRI_ON)
//
//  So OINPRI_ON is a per-bit selector between the sprite's own attribute byte
//  and the OBJ PRI register -- the sprite carries its priority in the high byte
//  of c18 for the bits the register does not claim.  gokuparo writes
//  OINPRI_ON = 0x03, i.e. the bottom two priority bits come from the register
//  (which is 0x00) and the top six from the sprite.
// ---------------------------------------------------------------------------
wire [15:0] opon            = { oinprion, 8'hff };
wire [15:0] obj_ocb         = { 3'd0, pregs[R_PALBASE_OBJ][2:0], 10'd0 };
wire [15:0] obj_mixed       = (obj_ocb & ~opon) | (obj_c18 & opon);
wire [15:0] obj_colour_full = obj_mixed >> obj_coregshift;
wire [8:0]  obj_colour      = obj_colour_full[8:0];
wire [7:0]  obj_pri_src     = (pregs_fmt == 2'd1) ? { 2'b00, obj_attr[9:4] }
                            : (pregs_fmt == 2'd2) ? (obj_attr[9] ? 8'd4 : { 4'd0, obj_attr[7:4] })
                            :                     obj_c18[15:8];
wire [7:0]  obj_inpri       = (obj_pri_src & ~oinprion) | (pregs[R_PRI_OBJ] & oinprion);

// ---------------------------------------------------------------------------
//  the eight inputs
//
//  Order is the chip's own: 0 A, 1 B, 2 C, 3 D, 4 OBJ, 5 S1, 6 S2, 7 S3.
//  That order is also the ENABLE register's bit order and the tie-break.
// ---------------------------------------------------------------------------
wire [7:0] px  [0:7];
wire [8:0] cc  [0:7];      // colour code: tiles x16 -> index bits 12:4; OBJ [7:0] x32 -> 12:5
wire [7:0] pri [0:7];
wire [1:0] mix [0:7];
wire [1:0] brs [0:7];      // brightness select

assign px[0] = px_a;  assign px[1] = px_b;  assign px[2] = px_c;  assign px[3] = px_d;
assign px[4] = px_o;
assign px[5] = {3'b0, px_s[4:0]};  assign px[6] = {3'b0, px_s[9:5]};  assign px[7] = {3'b0, px_s[14:10]};

assign cc[0] = vmix_colour(col_a, pregs[R_PALBASE_A +  6'd0], vmixon[1:0]);
assign cc[1] = vmix_colour(col_b, pregs[R_PALBASE_A +  6'd1], vmixon[3:2]);
assign cc[2] = vmix_colour(col_c, pregs[R_PALBASE_A +  6'd2], vmixon[5:4]);
assign cc[3] = vmix_colour(col_d, pregs[R_PALBASE_A +  6'd3], vmixon[7:6]);
assign cc[4] = obj_colour;
// SUB1-3 take K055555GX_decode_osmixcolor's layer!=0 branch, which is the same
// shape as the tilemap one but with OS INMIX ON and PALBASE_SUB1..3.
assign cc[5] = vmix_colour(col_s[ 7: 0], pregs[R_PALBASE_A +  6'd5], osmixon[3:2]);
assign cc[6] = vmix_colour(col_s[15: 8], pregs[R_PALBASE_A +  6'd6], osmixon[5:4]);
assign cc[7] = vmix_colour(col_s[23:16], pregs[R_PALBASE_A +  6'd7], osmixon[7:6]);

assign pri[0] = pregs[R_PRI_A];
assign pri[1] = pregs[R_PRI_B];
assign pri[2] = pregs[R_PRI_C];
assign pri[3] = pregs[R_PRI_D];
assign pri[4] = obj_inpri;
assign pri[5] = pregs[R_PRI_S1 +  6'd0];
assign pri[6] = pregs[R_PRI_S1 +  6'd1];
assign pri[7] = pregs[R_PRI_S1 +  6'd2];

assign mix[0] = vmix_blend(col_a, vmixon[1:0], vinmix[1:0]);
assign mix[1] = vmix_blend(col_b, vmixon[3:2], vinmix[3:2]);
assign mix[2] = vmix_blend(col_c, vmixon[5:4], vinmix[5:4]);
assign mix[3] = vmix_blend(col_d, vmixon[7:6], vinmix[7:6]);
// Sprites take the layer==0 branch of decode_osmixcolor (gxv.cpp:218), which
// has no external bits at all: "layer 0 is the sprite layer with different
// attributes decode; detail on p.49 (missing)" and MAME hardcodes emx = 0.
assign mix[4] = osinmix[1:0] & osmixon[1:0];
assign mix[5] = vmix_blend(col_s[ 7: 0], osmixon[3:2], osinmix[3:2]);
assign mix[6] = vmix_blend(col_s[15: 8], osmixon[5:4], osinmix[5:4]);
assign mix[7] = vmix_blend(col_s[23:16], osmixon[7:6], osinmix[7:6]);

// Brightness select.  V BRI is two bits per tilemap plane (set_brightness,
// gxv.cpp:253).  OS INBRI / OS INBRI ON are the obj/sub equivalent; MAME
// caches them and never reads them, so the gating below is this port's reading
// of the register names, not a transcription.
//
// TODO(HARDWAREIZE): what codes 1/2/3 select.  MAME uses them to pick one of
// the '338's three brightness registers and applies it as a SCALAR contrast to
// every channel, which cannot be right -- those three registers are the R, G
// and B of one triple (k054338.h:9, "11-12: brightness R/G/B").
assign brs[0] = vbri[1:0];
assign brs[1] = vbri[3:2];
assign brs[2] = vbri[5:4];
assign brs[3] = vbri[7:6];
assign brs[4] = osbri[1:0] & osbrion[1:0];
assign brs[5] = osbri[3:2] & osbrion[3:2];
assign brs[6] = osbri[5:4] & osbrion[5:4];
assign brs[7] = osbri[7:6] & osbrion[7:6];

// ---------------------------------------------------------------------------
//  the comparator
//
//  Key = { 1'b0, priority, input index }.  The index in the low bits makes
//  every enabled input's key unique, so "less than" is a total order and the
//  tie-break is the chip's input order without a second comparison.
//
//      12'hFFF   this input is transparent or disabled
//      12'hFFE   the backdrop, which is always opaque and always last
// ---------------------------------------------------------------------------
localparam [11:0] KEY_NONE = 12'hFFF;
localparam [11:0] KEY_BG   = 12'hFFE;

wire [11:0] key [0:8];

genvar gi;
generate
for (gi = 0; gi < 8; gi = gi + 1) begin : gkey
    wire opaque = (px[gi] != 8'd0) && enable[gi];
    assign key[gi] = opaque ? { 1'b0, flippri ? ~pri[gi] : pri[gi], gi[2:0] }
                            : KEY_NONE;
end
endgenerate
assign key[8] = KEY_BG;

reg [3:0] w0, w1;
integer j;
always @(*) begin
    w0 = 4'd8;
    for (j = 0; j < 8; j = j + 1)
        if (key[j] < key[w0]) w0 = j[3:0];
    w1 = 4'd8;
    for (j = 0; j < 8; j = j + 1)
        if ((j[3:0] != w0) && (key[j] < key[w1])) w1 = j[3:0];
end

// ---------------------------------------------------------------------------
//  backdrop
//
//  fill_backcolor (k054338.cpp:118) is a joint 338/5^5 function, "Unified
//  k054338/K055555 BG color fill (see p.67)".  When eeprom_w bit 29 selects it:
//
//      base  = K055555 reg 0 << 9
//      mode  = K055555 reg 1
//      bit 1 = 0  solid, one palette entry
//      bit 1 = 1  gradient; bit 0 = 0 vertical (index by Y), 1 horizontal (X)
//
//  MEASURED 2026-09-08 against the COMPLETE ROM set, docs/MEASUREMENTS.md
//  section 9: Gokujou Parodius writes **reg 0 = 0x0B and reg 1 = 0x02**, i.e.
//  GRADENABLE set, GRADDIR clear -- a VERTICAL gradient based at palette
//  0x0B << 9 = 0x1600, on 1126 of 1127 writes.
//
//  An earlier version of this comment said both registers were zero and that
//  the backdrop was therefore solid.  That was measured on a run with nine
//  ROMs zero-filled, which stopped at the POST error screen and never reached
//  the code that programs this.  So the gradient path is not a spare limb: it
//  is what the game actually uses, and a solid backdrop would be visibly
//  wrong.
//
//  MAME advances pal_ptr from `base + cliprect.min_y` once per line
//  (k054338.cpp:134-143), and cliprect.min_y is a BITMAP coordinate: the
//  visible area starts at y = 16 (x = 24 for the horizontal mode).  gx_ccu
//  counts from 0 at the start of active video, so the crop origin has to be
//  added back.  This comment used to say `vpos` alone was the right index.
//  MEASURED 2026-09-14 against MAME frame 3000: base + vpos + 16 matched its
//  backdrop and base + vpos matched none of it.  BG_X/Y_OFFSET, from gx_top.
// ---------------------------------------------------------------------------
wire [12:0] bgc_base = { pregs[R_BGC_CBLK][3:0], 9'd0 };
wire [12:0] bgc_ofs  = gradenable ? (graddir ? {3'd0, hpos} + {3'd0, BG_X_OFFSET}
                                             : {4'd0, vpos} + {4'd0, BG_Y_OFFSET})
                                  : 13'd0;
wire [12:0] bgc_idx  = bgc_base + bgc_ofs;

// ---------------------------------------------------------------------------
//  shadow
//
//  A sprite's shadow is a separate stream: it carries no colour, it darkens
//  whatever is behind it.  Its priority is its own if OPSET bit 5 (SDSEL) is
//  set, otherwise SHDPRI[code] -- gxv.cpp:566, "see p.51 OPSET SDSEL".
//
//  SHD ON (reg 40) "specifies layers on which shadows can be projected (see
//  detail on p.65 7.2.8)", so it is indexed by the WINNING input, not by the
//  shadow.  SHD PRI SEL (reg 41) enables the three shadows two bits at a time.
//
//  MAME implements neither: it forces all three shadowon[] to 0 and then turns
//  them back on only if the '338's RGB delta exceeds +/-7 (gxv.cpp:420).  That
//  threshold is an emulator optimisation -- a delta of +/-3 is a real, if
//  subtle, shadow -- so it is not reproduced.  U13 stays open for what the
//  three-way SHDPRISEL comparison actually selects.
//
//  2026-09-14, from the full ROM set (dist/objdump, dist/gamedump): gokuparo
//  writes SHD ON = 0x1F and SHD PRI SEL = 0x36 / 0x33 / 0x34 once the attract
//  runs -- the zeros this comment used to quote were the POST trace -- and
//  every curtain change is a shadow sprite with an animated '338 preset.  The
//  stream now comes from gx_sprite's shadow buffer.
//
//  WHERE A SHADOW LANDS ON A TIE, from MAME's sort (gxv.cpp:470-610): objects
//  are drawn in descending key order and a shadow darkens what is already in
//  the bitmap.  A layer's key is `pri << 24` exactly; a shadow's is
//  `spri << 24 | zcode << 16 | ...`, never smaller.  So at equal priority the
//  shadow is drawn FIRST and the layer covers it: strictly-in-front for a layer
//  winner.  Against a sprite winner the tie is decided by zcode: the shadow
//  lands only when it is nearer than the sprite pixel, which gx_sprite reports
//  as `obj_shd_front`.  Until 2026-09-14 every tie landed (<=): the framed
//  picture went dark with the attract's curtains, and the demo's explosion
//  flash blacked out every sprite on screen (MEASUREMENTS 37).  The backdrop
//  is filled before any object, so a shadow always reaches it -- SHD ON has no
//  bit for it.
//
//  ---- A SHADOW WHOSE SHD PRI SEL FIELD IS 2 LANDS AT EQUAL PRIORITY ---------
//  UPSTREAM_TODO U13, MEASUREMENTS 51 and 69, DECISIONS D17 and D20.
//  MAME tests each 2-bit field only for non-zero ("filters shadows by different
//  priority comparison methods (UNIMPLEMENTED, see detail on p.66)",
//  gxv.cpp:405).  The game writes three different values, 0x36 = 2 / 1 / 3, and
//  the value 2 appears only in the curtain scenes (every dist/objdump, demodump
//  and gamedump state).  There the shadow is code 1 over the whole screen at
//  SHD PRI 0x0C.  tools/gx_shdsel_probe.py on dist/curtaindump 1146 renders the
//  four readings:
//
//      behind (MAME's rule)   C 33,885 px   the CURTAINS go black
//      equal priority         O 15,906      the FRAMED PICTURE goes black
//      in front               O 14,366+355  the gold frame goes black
//      no landing             0             nothing darkens
//
//  **The PCB darkens the framed picture.**  The user, from the PCB footage:
//  "커텐 액자는 고정이고 그림만 어두워졌다가 밝아지면서 바껴" -- the curtain
//  and the gold frame are fixed, and only the picture darkens and brightens as
//  it changes, going FULLY black (asked and answered, 2026-09-21).  So field 2
//  is "equal priority".
//
//  **This corrects D17, which read "no landing" from the same footage.**  What
//  made the earlier reading wrong was a word: the user's "그림바뀔때 어두워지는거
//  전혀없음" was taken as "nothing darkens" when it meant "the curtains do not
//  darken".  Both readings leave the curtains lit and only the render of the
//  picture tells them apart -- which is why the correction cost a build rather
//  than a session (D20, and the lesson is in the DECISIONS entry).
//
//  Fields 1 and 3 keep the rule above: no scene we have tells them apart.
//  EMULATION_DERIVED for fields 1 and 3.  Field 2 is INFERRED from one PCB
//  scene, now on a rendered comparison rather than on a negative.
//  TODO(HARDWAREIZE): the p.66 table.
// ---------------------------------------------------------------------------
wire [7:0] win_pri    = (w0 == 4'd8) ? 8'hff : key[w0][10:3];
wire       shd_layer  = enable[4] && ((w0 == 4'd8) || shd_on[w0[2:0]]);

//  The rule above, for one shadow code.  Written out per code with explicit
//  wires: Quartus 17 does not see a function reading module signals and warned
//  them unread (10036) -- a warning this factory does not leave standing.
wire [7:0] shd_p1 = obj_sdsel ? obj_inpri : pregs[R_SHDPRI1 + 6'd0];
wire [7:0] shd_p2 = obj_sdsel ? obj_inpri : pregs[R_SHDPRI1 + 6'd1];
wire [7:0] shd_p3 = obj_sdsel ? obj_inpri : pregs[R_SHDPRI1 + 6'd2];
wire [7:0] shd_k1 = flippri ? ~shd_p1 : shd_p1;
wire [7:0] shd_k2 = flippri ? ~shd_p2 : shd_p2;
wire [7:0] shd_k3 = flippri ? ~shd_p3 : shd_p3;
//  Fields 1 and 3: on a winner BEHIND the shadow, a sprite tie decided by zcode.
//  Field 2: at EQUAL priority, and only there.  Compared on the KEYS, so a
//  flipped priority compares like for like.
function automatic shd_lands(input [1:0] fld, input [7:0] k, input [7:0] wp,
                             input obj_won, input front_z);
    reg behind;
begin
    behind    = obj_won ? ((k < wp) || ((k == wp) && front_z)) : (k < wp);
    shd_lands = (fld != 2'd0) && ((fld == 2'd2) ? (k == wp) : behind);
end
endfunction
wire       shd_objw = (w0 == 4'd4);
wire       shd_h1 = obj_shdm[1] && shd_layer && shd_lands(shdprisel[1:0], shd_k1, win_pri, shd_objw, obj_shd_front);
wire       shd_h2 = obj_shdm[2] && shd_layer && shd_lands(shdprisel[3:2], shd_k2, win_pri, shd_objw, obj_shd_front);
wire       shd_h3 = obj_shdm[3] && shd_layer && shd_lands(shdprisel[5:4], shd_k3, win_pri, shd_objw, obj_shd_front);
//  STACKED SHADOWS, 2026-09-29 (MEASUREMENTS 155).  MAME draws a later shadow
//  over an earlier one only at a strictly LOWER priority
//  (k053246_k053247_k055673.cpp:570), so codes that land at the same key count
//  once -- the lowest code is kept.
//  APPROXIMATION: MAME also requires the later shadow to be nearer-or-equal in
//  z, and which of two equal-key shadows survives depends on z too; the per-code
//  buffers do not carry z.  TODO(HARDWAREIZE): U13/U14, how the chip stacks.
wire [3:1] shd_hitm = { shd_h3 && !(shd_h1 && shd_k3 == shd_k1) && !(shd_h2 && shd_k3 == shd_k2),
                        shd_h2 && !(shd_h1 && shd_k2 == shd_k1),
                        shd_h1 };
wire [1:0] shd_low  = shd_hitm[1] ? 2'd1 : shd_hitm[2] ? 2'd2 : shd_hitm[3] ? 2'd3 : 2'd0;

// ---------------------------------------------------------------------------
//  outputs
//
//  Registered once.  The inputs are stable for a whole pixel (16 system clocks
//  at a 6 MHz dot rate on a ~96 MHz clock), so a cycle of latency costs nothing
//  and it keeps the comparator out of the palette RAM's address path.
// ---------------------------------------------------------------------------
//  Input 4 is OBJ and keeps granularity 32 whatever pal_gran16 says -- see
//  the port comment.  For granularity 16 this is an ADD, not a concatenation:
//  the five-bit pen can carry into the next 16-entry colour block.  That is
//  how MAME's five-bit gfx element behaves after set_granularity(16), and it
//  is deliberately written as two 13-bit addends so pixel[4] cannot vanish in
//  an implicit-width conversion.
//  The x16 sum is 13 bits and wraps at 8,192, as MAME's palette does.  The
//  legacy x32 path (D5's diagnostic) and OBJ keep the 8-bit code.
function automatic [12:0] pal_index(input [3:0] w, input [8:0] c, input [7:0] pix);
begin
    if (w == 4'd4)
        // OBJ: MAME's granularity is 1 << bpp
        pal_index = (pregs_fmt == 2'd1) ? { c[6:0], pix[5:0] }
                  : (pregs_fmt == 2'd2) ? { c[8:0], pix[3:0] }
                  : (pregs_fmt == 2'd3) ? { c[4:0], pix[7:0] }
                  :                     { c[7:0], pix[4:0] };
    else if (pal_gran16)
        // tiles: set_granularity(16) for every K056832 bpp -- the 6th bit carries too
        pal_index = { c, 4'd0 } + { 5'd0, pix };
    else
        pal_index = { c[7:0], pix[PXBITS-1:0] };
end
endfunction

wire [12:0] idx_of_w0 = (w0 == 4'd8) ? bgc_idx
                                     : pal_index(w0, cc[w0[2:0]], px[w0[2:0]]);
wire [12:0] idx_of_w1 = (w1 == 4'd8) ? bgc_idx
                                     : pal_index(w1, cc[w1[2:0]], px[w1[2:0]]);

//  Gated on pxl_cen.  See the port comment: the inputs move once per dot, so
//  capturing once per dot is both correct and what makes the SDC exception
//  true.  A reset still clears asynchronously to the enable.
//
//  ---- the side band leaves ONE DOT LATER than idx0/idx1, 2026-09-14 --------
//  idx0/idx1 go through gx_palette, whose rgb*_p copy is one more pxl_cen
//  stage, before gx_colmix meets them.  bg0/bg1/blend/bri/shadow went straight
//  to gx_colmix, so the mixer combined the colour of dot h with the side band
//  of dot h+1.  docs/DECISIONS.md D9 addendum counted it and left it as
//  TODO(P5) because every one of these was 0 on the tile path; sprites and
//  their shadows are what makes it visible, so it is paid with them.
//
//  The extra stage is INSIDE this module on purpose: every output is still a
//  pxl_cen register of gx_prio, so KonamiGX.sdc's `-from gx_prio -to
//  gx_colmix` exception stays exactly as true as it was.  Remove the palette's
//  output stage and this one has to go with it.
reg        bg0_s, bg1_s;
reg [1:0]  blend_s, bri_s, bri1_s, shadow_s;
reg [3:1]  shadow_ms;
assign blend_e   = blend_s;
assign shadow_e  = shadow_s;
assign shadow_me = shadow_ms;

always @(posedge clk) begin
    if (rst) begin
        idx0       <= 13'd0;
        idx1       <= 13'd0;
        bg0_s      <= 1'b0;
        bg1_s      <= 1'b0;
        blend_s    <= 2'd0;
        bri_s      <= 2'd0;
        bri1_s     <= 2'd0;
        shadow_s   <= 2'd0;
        shadow_ms  <= 3'd0;
        shadow_m   <= 3'd0;
        bg0        <= 1'b0;
        bg1        <= 1'b0;
        blend      <= 2'd0;
        bri        <= 2'd0;
        bri1       <= 2'd0;
        shadow     <= 2'd0;
        dbg_winner <= 4'd8;
    end else if (pxl_cen) begin
        idx0 <= idx_of_w0;
        idx1 <= idx_of_w1;
        // The backdrop only bypasses the palette when eeprom_w bit 29 selects
        // the '338's own solid colour registers.
        bg0_s      <= (w0 == 4'd8) && !bgc_from_pal;
        bg1_s      <= (w1 == 4'd8) && !bgc_from_pal;
        blend_s    <= (w0 == 4'd8) ? 2'd0 : mix[w0[2:0]];
        bri_s      <= (w0 == 4'd8) ? 2'd0 : brs[w0[2:0]];
        bri1_s     <= (w1 == 4'd8) ? 2'd0 : brs[w1[2:0]];
        shadow_s   <= shd_low;
        shadow_ms  <= shd_hitm;
        bg0        <= bg0_s;
        bg1        <= bg1_s;
        blend      <= blend_s;
        bri        <= bri_s;
        bri1       <= bri1_s;
        shadow     <= shadow_s;
        shadow_m   <= shadow_ms;
        dbg_winner <= w0;
    end
end

endmodule

`default_nettype wire
