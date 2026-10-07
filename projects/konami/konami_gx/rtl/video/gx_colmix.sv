//============================================================================
//  Konami System GX -- final colour combiner
//
//  The K054338 half of the mixer.  gx_prio (the K055555) has already decided
//  which of the eight inputs wins each pixel and which is behind it; the
//  palette has turned both into RGB.  What is left is arithmetic:
//
//      brightness  ->  shadow / highlight  ->  alpha or additive blend
//
//  ---- the register file is not ours ---------------------------------------
//  third_party/video/jt054338/jt054338.v, GPL-3.0-or-later, jotego, shipping
//  in the released Cowboys of Moo Mesa core.  Root CLAUDE.md section 1.2 shelf
//  2: MAME's k054338.cpp says of itself "this is just a register-handling
//  shell", so there is no behaviour to port from it -- only a map, and that map
//  already exists in a core people play.
//
//  It was checked field by field against k054338.cpp before adoption and they
//  agree everywhere, including the one selection nobody would guess:
//
//      pblend 1 -> word 13 bits 7-0
//      pblend 2 -> word 14 bits 15-8
//      pblend 3 -> word 14 bits 7-0
//
//  MAME writes that as `regs[13 + (pblend>>1 & 1)] >> (~pblend << 3 & 8)`.
//  Two independent implementations landing on the same three bytes makes it a
//  hardware fact rather than one source's reading (root section 12.0).
//
//  ---- one upstream defect, worked around rather than patched ---------------
//  jt054338 substitutes mixset = 0x1f when pblend == 0.  With ALPHA_INV = 0
//  that is 31/31 = opaque and correct.  System GX sets alpha_invert(1)
//  (konamigx.cpp:1768, and again at gxv.cpp:319), which turns 0x1f into 0 --
//  fully transparent.  MAME's set_alpha_level returns 255 for pblend == 0
//  before it reads any register at all.
//
//  Moo Mesa never hits it because jtmoo_colmix hardwires pblend = 1.  The fix
//  is here, not in the vendored file: `blend == 0` forces opaque.  Keeping the
//  third-party file byte-identical to upstream is worth more than saving one
//  mux, and this way the next person diffing it against jtcores sees nothing.
//  Recorded in third_party/video/jt054338/UPSTREAM.yml under
//  known_upstream_defect.
//
//  ---- what MAME did not implement -----------------------------------------
//  Additive blending.  k054338.cpp:150 says "addition blending unimplemented
//  (requires major changes to drawgfx and tilemap.cpp)" and gxv.cpp:730 papers
//  over it: "FIXME: implement mixpri and additive -- hack: mask out mixpri bit.
//  if additive bit set, mask it out and invert alpha."
//
//  An FPGA has no such constraint, so it is implemented -- but the arithmetic
//  itself is a reading of the word "additive", not a transcription of anything.
//  UNVERIFIED, and marked below.
//
//  MIXPRI (control bit 1) is decoded and unused.  MAME masks it out and says
//  so; we do not know what it selects either.  U28.
//
//  ---- video enable --------------------------------------------------------
//  K338_CTL_KILL, control bit 0, "0 = no video output, 1 = enable".  Taken
//  literally: output is black when it is clear.
//
//  MAME appears to disagree -- konamigx_mixer fills the backdrop BEFORE
//  testing KILL and returns after, leaving the backdrop visible.  That is
//  incidental ordering in a function that also fills the backdrop before
//  testing the input-enable register, not a statement about the chip.
//  jt054338's own consumer takes the literal reading (`!video_en -> 24'd0`),
//  and the register's name is KILL.  Two against one, and the register text is
//  unambiguous.
//
//  It matters less than it looks on this game: gokuparo runs with CONTROL =
//  0x30 (KILL clear) until frame 162, and its backdrop at that point is
//  palette entry 0.  docs/MEASUREMENTS.md section 5.
//============================================================================
`default_nettype none

module gx_colmix (
    input  wire        clk,
    input  wire        rst,
    input  wire        pxl_cen,

    // --- CPU: K054338 registers, d80000-d8001f, genuinely 16-bit -----------
    input  wire        reg_cs,
    input  wire        reg_we,
    input  wire [4:1]  reg_addr,
    input  wire [15:0] reg_din,
    input  wire [1:0]  reg_ds,          // {uds, lds}, active high
    output wire [15:0] reg_dout,
    output reg  [3:1]  shd_live,        // shadow preset N has a delta beyond +/-7 (gx_sprite)

    // --- from gx_prio -------------------------------------------------------
    input  wire        bg0,             // winner is the '338's own solid colour
    input  wire        bg1,             // runner-up is
    input  wire [1:0]  blend,           // alpha preset, 0 = opaque
    input  wire [1:0]  bri,             // brightness select, 0 = none
    input  wire [1:0]  bri1,            // the runner-up's, aligned with bri
    input  wire [1:0]  shadow,          // shadow preset, 0 = none
    //  blend / shadow one stage earlier (gx_prio blend_e / shadow_e): what the
    //  K054338 register selection below is driven by.  See "R1" at u_k338.
    input  wire [1:0]  blend_e,
    input  wire [1:0]  shadow_e,
    //  STACKED SHADOWS (gx_prio): every preset that lands, aligned with
    //  `shadow`, and one stage earlier for the summed delta below.
    input  wire [3:1]  shadow_m,
    input  wire [3:1]  shadow_me,

    // --- from gx_palette ----------------------------------------------------
    input  wire [23:0] rgb0,            // winner    {R,G,B}
    input  wire [23:0] rgb1,            // runner-up {R,G,B}

    // --- raster -------------------------------------------------------------
    input  wire        hblank,
    input  wire        vblank,

    // --- output -------------------------------------------------------------
    output reg  [7:0]  red,
    output reg  [7:0]  green,
    output reg  [7:0]  blue,

    // --- observability ------------------------------------------------------
    //  K338_CTL_KILL, live.  This module blanks the output when it is clear
    //  (see the note at the top), and gokuparo runs with it CLEAR until frame
    //  162 -- so "black screen" and "the game has not switched video on yet"
    //  are the same picture and nothing else in the core can tell them apart.
    //  Read out rather than inferred.
    output wire        dbg_video_en
);

// ---------------------------------------------------------------------------
//  the vendored register file
// ---------------------------------------------------------------------------
wire [23:0] k338_bg;
wire [7:0]  alpha_level;
wire        alpha_add, video_en, clipsl;

jt054338 #(.ALPHA_INV(1)) u_k338 (
    .rst         (rst),
    .clk         (clk),

    .cs          (reg_cs),
    .we          (reg_we),
    .addr        (reg_addr),
    .din         (reg_din),
    .dsn         (~reg_ds),
    .dout        (reg_dout),

    //  ---- R1, 2026-09-15 (MEASUREMENTS 46) -----------------------------------
    //  These were `blend` / `shadow`.  jt054338 selects alpha_level, alpha_add
    //  and shadow_r/g/b from the code combinationally, and the p_* copy below
    //  registers them on pxl_cen -- so the levels reached the arithmetic one dot
    //  AFTER the code they belong to, and the first dot of every shadow or blend
    //  run took the previous dot's level (delta 0 / alpha 0).  MEASURED on the
    //  board's own screenshots against a model of that lag: 100.00 % of the
    //  differing pixels at 5039 (HUD box), 1824 (x=0 column), 9244 (curtain)
    //  and 10825 (title); 99.4 / 99.1 % on the sea-stage blend bands.  gx_prio's
    //  earlier stage puts the level in p_* on the same edge `blend` / `shadow`
    //  change.  jt054338 stays byte-identical (third party).
    .pblend      (blend_e),
    .shadow      (shadow_e),

    .bg_rgb      (k338_bg),
    .alpha_level (alpha_level),
    .alpha_add   (alpha_add),
    .video_en    (video_en),
    // Three control bits nothing consumes yet.  MIXPRI is U28 -- MAME masks
    // it out and says so, and we do not know what it selects either.  SHDPRI
    // is MAME's own "demote shadows by one layer when this bit is set??? (see
    // p.73 8.6)", question marks included.  Left OPEN rather than tied to a
    // dead wire: docs/WARNING_POLICY.md does not accept a project-owned
    // "assigned but never read".
    .mixpri      (),
    .shdpri      (),
    .brtpri      (),
    .clipsl      (clipsl),
    .dump_mmr    (),

    // the single-preset shadow level: superseded by the summed delta
    // (STACKED SHADOWS) and left unconnected
    .shadow_r    (),
    .shadow_g    (),
    .shadow_b    ()
);

// ---------------------------------------------------------------------------
//  brightness registers -- snooped, not read back
//
//  The '338's three brightness PRESETS live in words 11 and 12 (not R/G/B --
//  see "BRIGHTNESS IS THREE PRESETS" below; bri_r/g/b keep their old names):
//
//      word 11 bits 7-0    preset 1   (bri_r)     konamigx_v.cpp:59-61, which
//      word 12 bits 15-8   preset 2   (bri_g)     reads exactly those three
//      word 12 bits 7-0    preset 3   (bri_b)     bytes as m_brightness[0..2]
//
//  jt054338 exposes word 11 only, and only inside its jtframe debug bundle.
//  Rather than modify a vendored file for three bytes, the same CPU writes are
//  watched here.  Duplicating three bytes of register state is cheaper than a
//  local patch and it cannot drift: it is the same bus, the same cycle.
//
//  gokuparo writes all three as 0xFF -- full brightness, i.e. a no-op.  What
//  the two-bit select CODE means is a different question and it is open; see
//  the TODO in gx_prio.sv.  Here, non-zero simply means "apply the triple".
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  SHADOW PRESET LIVE, 2026-09-28
//
//  EMULATION_DERIVED
//  Matches MAME konamigx_v.cpp:421-426: a shadow code whose preset (regs 2-4,
//  5-7, 8-10, each a sign-extended 9-bit delta) is within +/-7 on all three
//  channels is not drawn at all -- not even as an invisible shadow.
//  Required because gx_sprite's shadow buffer holds ONE shadow a dot: tbyahhoo's
//  demo lays a full-screen code-2 shadow (preset 2 = 0) at the front zcode, and
//  it displaced the code-3 (-64) shadow of the 1P panel everywhere, so the
//  panel never darkened (MEASUREMENTS 152).  MAME drops the code-2 object and
//  the code-3 one lands.
//  This comment used to call the threshold an emulator optimisation, and it may
//  be: a real K055555 probably keeps more than one shadow a dot.  The actual PCB
//  behaviour is NOT verified.
//  TODO(HARDWAREIZE): do stacked shadows exist on the chip (U13, U14)?
// ---------------------------------------------------------------------------
reg [8:0] shd9 [2:10];
integer   si;
function automatic big7(input [8:0] d);
    // sign-extended 9-bit: outside [-7, 7]
    big7 = d[8] ? (d < 9'h1F9) : (d > 9'd7);
endfunction
always @(posedge clk) begin
    if (rst) begin
        for (si = 2; si <= 10; si = si + 1) shd9[si] <= 9'd0;
    end else if (reg_cs && reg_we && reg_addr >= 4'd2 && reg_addr <= 4'd10) begin
        if (reg_ds[1]) shd9[reg_addr][8]   <= reg_din[8];
        if (reg_ds[0]) shd9[reg_addr][7:0] <= reg_din[7:0];
    end
end
always @(posedge clk) begin
    shd_live[1] <= big7(shd9[2]) | big7(shd9[3]) | big7(shd9[4]);
    shd_live[2] <= big7(shd9[5]) | big7(shd9[6]) | big7(shd9[7]);
    shd_live[3] <= big7(shd9[8]) | big7(shd9[9]) | big7(shd9[10]);
end

reg [7:0] bri_r, bri_g, bri_b;
always @(posedge clk) begin
    if (rst) begin
        bri_r <= 8'hff;
        bri_g <= 8'hff;
        bri_b <= 8'hff;
    end else if (reg_cs && reg_we) begin
        if (reg_addr == 4'd11 && reg_ds[0]) bri_r <= reg_din[7:0];
        if (reg_addr == 4'd12) begin
            if (reg_ds[1]) bri_g <= reg_din[15:8];
            if (reg_ds[0]) bri_b <= reg_din[7:0];
        end
    end
end

// ---------------------------------------------------------------------------
//  arithmetic helpers
//
//  scale8: c * (k+1) >> 8.  Exact at both ends -- k = 0xff gives c back and
//  k = 0 gives 0 -- which a plain (c*k)>>8 does not.
//
//  blend8: b + ((a - b) * (alpha+1) >>> 8).  One signed multiplier per channel
//  instead of the two an (a*al + b*(256-al)) form needs, and exact at both
//  ends for the same reason.
//
//  add_clip: the signed 9-bit shadow delta, clamped.  MAME's
//  update_all_shadows sign-extends regs[SHAD*] & 0x1ff the same way
//  (k054338.cpp:82) and jt054338 hands it over already sign-extended.
// ---------------------------------------------------------------------------
function automatic [7:0] scale8(input [7:0] c, input [7:0] k);
    reg [8:0]  k1;
    reg [16:0] p;
begin
    k1 = {1'b0, k} + 9'd1;
    p  = {9'd0, c} * {8'd0, k1};
    scale8 = p[15:8];
end
endfunction

function automatic [7:0] blend8(input [7:0] a, input [7:0] b, input [7:0] al);
    reg [8:0]         a1;
    reg signed [19:0] d, m, p;
begin
    a1 = {1'b0, al} + 9'd1;                     // 1 .. 256
    d  = $signed({12'd0, a}) - $signed({12'd0, b});
    m  = $signed({11'd0, a1});
    p  = d * m;
    // p >>> 8 is exact and lands in [-255, 255]; b + it is in [0, 255] by
    // construction, so the low byte is the answer.
    blend8 = b + p[15:8];
end
endfunction

function automatic [7:0] addsat8(input [7:0] a, input [7:0] b);
    reg [8:0] s;
begin
    s = {1'b0, a} + {1'b0, b};
    addsat8 = s[8] ? 8'hff : s[7:0];
end
endfunction

// clipsl is K338_CTL_CLIPSL, "no-clip for shadow arithmetic".  gokuparo has it
// set throughout, but every shadow delta it writes is zero, so which branch is
// right is not decided by anything measured.  Clamping is the documented
// behaviour ("The hardware clamps at black or white as necessary: see the
// Graphics Test in many System GX games", k054338.cpp:16); CLIPSL turning that
// off is this port's reading of the bit's name.
// TODO(VERIFY): the Graphics Test in the service menu is a golden reference
// for exactly this, reachable once the game boots.
function automatic [7:0] shade8(input [7:0] c, input signed [9:0] d, input noclip);
    reg signed [10:0] s, cc, dd;
begin
    cc = $signed({3'b000, c});
    dd = $signed({d[9], d});
    s  = cc + dd;
    if (noclip)            shade8 = s[7:0];
    else if (s < 11'sd0)   shade8 = 8'h00;
    else if (s > 11'sd255) shade8 = 8'hff;
    else                   shade8 = s[7:0];
end
endfunction

// ---------------------------------------------------------------------------
//  the pixel path
//
//  Order: brightness, then shadow, then blend against what is behind.
//
//  Brightness and shadow act on the winning layer, so they come first; the
//  blend is by definition the last step because it needs the finished winner.
//  MAME reaches the same order for a different reason -- it applies both as
//  palette adjustments (set_pen_contrast, set_shadow_dRGB32) and blends when
//  drawing.  The '338 is one chip doing all three and the real internal order
//  is not documented anywhere we have.
//  TODO(HARDWAREIZE): confirm against the Graphics Test.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  Everything the mixer reads, brought into the pixel domain
//
//  The K054338's register file and the three brightness bytes below are
//  written by the CPU on ITS enable, so they can change on any 96 MHz edge
//  relative to a dot -- while the mixer's outputs (red/green/blue) capture on
//  pxl_cen.  That is the same slow-source / fast-capture shape as
//  docs/DECISIONS.md D9, and it was the fifth place this board made it.
//
//  MEASURED, build H, 2026-09-07 -- the five worst paths in the design:
//
//      gx_colmix|bri_g[2] -> gx_colmix|green[2]     slack -5.689
//
//  jt054338 is vendored byte-identical and must not be touched, so the copy
//  lives here, on its outputs.  With it, every input to the blend arithmetic
//  moves at the dot rate and the SDC exception below is true for all of them.
//
//  NOT copied, deliberately: `blend`, `shadow`, `bg0`, `bg1`, `bri`, `rgb0`,
//  `rgb1` are already dot-rate (they come from gx_prio and gx_palette), and
//  `hblank`/`vblank` come from gx_ccu's pxl_cen-gated counters.
//
//  Cost: 89 flip-flops, and a K054338 register write lands on the next dot.
//  The game writes these during blanking, so a one-dot delay is not
//  observable -- and it is one more term in the alignment arithmetic D9 says
//  to carry into P4 rather than rediscover.
// ---------------------------------------------------------------------------
reg [23:0]        p_k338_bg;
reg [7:0]         p_alpha_level;
reg               p_alpha_add, p_video_en, p_clipsl;
reg [7:0]         p_bri_r, p_bri_g, p_bri_b;

// ---- STACKED SHADOWS, 2026-09-29 (MEASUREMENTS 155) ------------------------
//  The delta for every preset that lands, summed and registered one stage
//  early -- the way jt054338's single level is taken from shadow_e -- so the
//  pixel path still does ONE add-and-clamp.  MAME applies the presets one
//  after another, each clamped; for shadows (all deltas <= 0) and clamping
//  the sum is the same answer.  A highlight (> 0) stacked with a shadow is
//  not the same.  APPROXIMATION there.
function automatic signed [10:0] sx9(input [8:0] d);
    sx9 = {{2{d[8]}}, d};
endfunction
wire signed [10:0] sum_r = (shadow_me[1] ? sx9(shd9[2]) : 11'sd0) + (shadow_me[2] ? sx9(shd9[5]) : 11'sd0)
                         + (shadow_me[3] ? sx9(shd9[8]) : 11'sd0);
wire signed [10:0] sum_g = (shadow_me[1] ? sx9(shd9[3]) : 11'sd0) + (shadow_me[2] ? sx9(shd9[6]) : 11'sd0)
                         + (shadow_me[3] ? sx9(shd9[9]) : 11'sd0);
wire signed [10:0] sum_b = (shadow_me[1] ? sx9(shd9[4]) : 11'sd0) + (shadow_me[2] ? sx9(shd9[7]) : 11'sd0)
                         + (shadow_me[3] ? sx9(shd9[10]) : 11'sd0);
function automatic signed [9:0] sat10(input signed [10:0] v);
    sat10 = (v > 11'sd511) ? 10'sd511 : (v < -11'sd512) ? 10'h200 : v[9:0];   // 10'h200 = -512
endfunction
reg signed [9:0]  p_sum_r, p_sum_g, p_sum_b;
always @(posedge clk) begin
    if (rst) begin
        p_sum_r <= 10'sd0;
        p_sum_g <= 10'sd0;
        p_sum_b <= 10'sd0;
    end else if (pxl_cen) begin
        p_sum_r <= sat10(sum_r);
        p_sum_g <= sat10(sum_g);
        p_sum_b <= sat10(sum_b);
    end
end

always @(posedge clk) begin
    if (rst) begin
        p_k338_bg     <= 24'd0;
        p_alpha_level <= 8'hff;
        p_alpha_add   <= 1'b0;
        p_video_en    <= 1'b0;
        p_clipsl      <= 1'b0;
        p_bri_r       <= 8'hff;
        p_bri_g       <= 8'hff;
        p_bri_b       <= 8'hff;
    end else if (pxl_cen) begin
        p_k338_bg     <= k338_bg;
        p_alpha_level <= alpha_level;
        p_alpha_add   <= alpha_add;
        p_video_en    <= video_en;
        p_clipsl      <= clipsl;
        p_bri_r       <= bri_r;
        p_bri_g       <= bri_g;
        p_bri_b       <= bri_b;
    end
end
wire [23:0] win  = bg0 ? p_k338_bg : rgb0;
wire [23:0] back_raw = bg1 ? p_k338_bg : rgb1;

// The upstream-defect workaround: pblend 0 is opaque (MAME set_alpha_level).
wire [7:0] alpha = (blend == 2'd0) ? 8'hff : p_alpha_level;

// ---------------------------------------------------------------------------
//  BRIGHTNESS IS THREE PRESETS, NOT AN R/G/B TRIPLE, 2026-09-30
//
//  The three bytes (word 11 low, word 12 high, word 12 low) are brightness
//  presets 1, 2, 3; the layer's two-bit select (K055555 VBRI / OSBRI) picks
//  one and it scales R, G and B alike.  MAME konamigx_v.cpp:59-61 and :251-263
//  (set_brightness: m_brightness[bri_mode - 1] -> set_pen_contrast).  This
//  module used to read k054338.cpp:21's "11-12: brightness R/G/B" literally,
//  but the next line calls 13-14 "alpha blend R/G/B" too, and 13-14 are three
//  presets picked by a two-bit code in jt054338 and in MAME alike.
//  MEASURED (MEASUREMENTS 161): dragoonj's ranking screen, VBRI 0x50, presets
//  A0/FF/FF -- the board dimmed red alone (teal picture, 91 % of the backdrop
//  pixels = D x (0.627, 1, 1)), MAME dims all three.
//
//  The RUNNER-UP gets its own preset too (bri1), before it meets the blend:
//  MAME dims every layer as it draws it (set_brightness per layer), so the
//  back of a blend is already dimmed.  MEASURED on the 5716aa56 board run: with
//  the winner alone dimmed, the ranking screen's difference sat exactly on the
//  pixels where layer C blends over layer D, and nowhere else.
//  EMULATION_DERIVED -- TODO(HARDWAREIZE): whether the K054338 applies
//  brightness to its second input at all is not known; MAME's frame-buffer
//  drawing makes it so.
// ---------------------------------------------------------------------------
wire [7:0] bri_k  = (bri  == 2'd1) ? p_bri_r : (bri  == 2'd2) ? p_bri_g : p_bri_b;
wire [7:0] bri1_k = (bri1 == 2'd1) ? p_bri_r : (bri1 == 2'd2) ? p_bri_g : p_bri_b;
wire [23:0] back = (bri1 == 2'd0) ? back_raw
                 : { scale8(back_raw[23:16], bri1_k), scale8(back_raw[15:8], bri1_k),
                     scale8(back_raw[7:0], bri1_k) };
wire [7:0] b_r = (bri == 2'd0) ? win[23:16] : scale8(win[23:16], bri_k);
wire [7:0] b_g = (bri == 2'd0) ? win[15: 8] : scale8(win[15: 8], bri_k);
wire [7:0] b_b = (bri == 2'd0) ? win[ 7: 0] : scale8(win[ 7: 0], bri_k);

//  One preset: p_sum_* is exactly jt054338's level for that code (same
//  registers, the same stage).
wire [7:0] s_r = (shadow_m == 3'd0) ? b_r : shade8(b_r, p_sum_r, p_clipsl);
wire [7:0] s_g = (shadow_m == 3'd0) ? b_g : shade8(b_g, p_sum_g, p_clipsl);
wire [7:0] s_b = (shadow_m == 3'd0) ? b_b : shade8(b_b, p_sum_b, p_clipsl);

// ---------------------------------------------------------------------------
//  ADDITIVE: the level is NOT inverted, 2026-09-22
//
//  MEASURED on the board (MEASUREMENTS 95, 96): on sexyparo's demo stage B the
//  whole of layer B disappears and the runner-up shows through untouched --
//  17,615 pixels, 99.5 % of that frame's board-vs-MAME difference.  Following
//  the arithmetic all the way gives exactly that picture, with no wiring fault
//  anywhere:
//
//      tile mix code 2   K054338 PBLEND word 14 high byte = 0x3F
//      ALPHA_INV         mixlv = ~0x1F = 0            ->  alpha_level 0
//      alpha_add         mixset[5] = 1                ->  additive
//      addsat8(back, scale8(win, 0)) = back                 the runner-up
//
//  So the level said "invisible" and additive-at-invisible adds nothing.  The
//  game plainly means that layer to be seen, which puts the polarity of the
//  ADDITIVE level in question -- and nothing upstream can settle it:
//
//      k054338.cpp:150   "addition blending unimplemented"
//      gxv.cpp:730       when the additive bit is set MAME forces alpha to 0
//                        and DOES NOT DRAW THE PASS.  So MAME never reads the
//                        additive level at all and is no authority on it
//      jt054338          carries ALPHA_INV as a parameter and applies it to
//                        every code alike; Mystic Warriors, which it was
//                        written for, calls invert_alpha(0) (mystwarr_v.cpp:267)
//                        where System GX calls invert_alpha(1) (gxv.cpp:320)
//
//  So: ALPHA_INV applies to the BLEND level, which MAME does use and which the
//  verified band agrees on, and NOT to the ADDITIVE level.  Un-inverted here
//  rather than in the vendored file, the same way the pblend-0 defect above is
//  worked around, so jt054338 stays byte-identical.
//
//  `alpha_level` is {mixlv, mixlv[4:2]} over mixlv = ~mixset[4:0], so the
//  register's own five bits come back as ~alpha_level[7:3] -- no second
//  instance and no new port.
//
//  THIS TOUCHES ONLY PIXELS WITH `p_alpha_add` SET.  Every other pixel is
//  bit-identical, which is why the sixteen-of-sixteen band cannot move: there
//  every K054338 blend register is 0x0000 and every tile's code is 0
//  (MEASUREMENTS 93.2).
//
//  STILL UNVERIFIED -- TODO(HARDWAREIZE): that the additive SUM is
//  saturating-add of the scaled winner, and the level's polarity itself.  What
//  changed is which of two readings we run, on measured grounds; neither is a
//  transcription of silicon.
// ---------------------------------------------------------------------------
//  FROM THE REGISTERED COPY, not from jt054338's combinational output.  The
//  first version of this read `alpha_level` directly and that was wrong twice:
//  it put jt054338's `regs` in a combinational path to `red`/`green`/`blue`
//  (the fit came back at -0.077 ns on the core clock, and the five worst paths
//  were all regs[13]/regs[14] -> red[0]), and it undid MEASUREMENTS 46 R1,
//  which added this pipeline stage precisely because the level was arriving one
//  dot before the code it belongs to.  `alpha` below has always used p_*.
wire [4:0] add_lv5 = ~p_alpha_level[7:3];           // the register's own bits
wire [7:0] alpha_add_lv = (blend == 2'd0) ? 8'hff : {add_lv5, add_lv5[4:2]};

wire [7:0] mix_r = p_alpha_add ? addsat8(back[23:16], scale8(s_r, alpha_add_lv))
                             : blend8(s_r, back[23:16], alpha);
wire [7:0] mix_g = p_alpha_add ? addsat8(back[15: 8], scale8(s_g, alpha_add_lv))
                             : blend8(s_g, back[15: 8], alpha);
wire [7:0] mix_b = p_alpha_add ? addsat8(back[ 7: 0], scale8(s_b, alpha_add_lv))
                             : blend8(s_b, back[ 7: 0], alpha);

wire blanked = hblank | vblank | ~p_video_en;
assign dbg_video_en = video_en;

always @(posedge clk) begin
    if (rst) begin
        red   <= 8'd0;
        green <= 8'd0;
        blue  <= 8'd0;
    end else if (pxl_cen) begin
        red   <= blanked ? 8'd0 : mix_r;
        green <= blanked ? 8'd0 : mix_g;
        blue  <= blanked ? 8'd0 : mix_b;
    end
end

endmodule

`default_nettype wire
