//============================================================================
//  Konami 053252 "CCU" -- video timing and interrupt controller
//
//  Register map from references/upstream/mame/src/devices/machine/k053252.cpp
//  (LGPL-2.1+, Angelo Salese, from notes by Olivier Galibert), transcribed in
//  docs/SOURCE_AUDIT.md section 3.
//
//  ---- why this is written rather than reused -------------------------------
//  jtcores has a silicon-derived K053252 (cores/rungun/hdl/jtk053252.v, GPL-3),
//  and root section 1.2 says prefer it.  It cannot be used as-is: its register
//  file is `jtk053252_mmr`, generated at build time by the `jtframe mmr` Go
//  tool from cfg/mmr.yaml, and is .gitignore'd upstream.  We do not have that
//  tool.
//
//  So this is an independent implementation from the documented register map.
//  jtk053252.v was read for structure and is cited where it settled something
//  MAME's comments left open.  Under docs/THIRD_PARTY_POLICY.md that is
//  REFERENCE_FACTS_ONLY: no code copied, and both licences are GPL-compatible
//  anyway.
//
//  ---- checked against the silicon, 2026-10-06 (MEASUREMENTS 181) ----------
//  sim/tb_gx_ccu_053252.v runs Furrtek's silicon reconstruction of the 053252
//  (jtcores cores/rungun/doc/053252.v, read from references/, not copied)
//  beside this module, programmed with the two settings the nineteen sets
//  write (GX: HC 17F HFP 10 HBP 30 VC 107 VFP 11 VBP 0E SW 73; dragoonj /
//  winspike: HC 1FF HFP 19 HBP 3F SW 74).  Per line and per frame the active,
//  blank, sync and porch lengths are IDENTICAL -- including k053252.cpp's
//  "+1 on VBP only" (the silicon's V back porch is VBP+1 lines; its H back
//  porch is HBP+1 too, but its front porch is HFP-1, so the H active width is
//  the same formula).  The one H difference was the sync start, one dot early
//  in silicon; fixed below.  So the formulas in this file are the silicon's,
//  not just MAME's.
//
//  KNOWN, NOT CHANGED: the silicon increments its V counter (and so raises
//  VBLANK and INT1, and ticks INT2) at the START of horizontal blank; this
//  module does it at the end of the line, one horizontal blank (96 / 128
//  dots) later.  Both are inside blanking, so no pixel moves, but INT1 reaches
//  the 68EC020 ~16 us later than on the PCB.  Moving it changes CPU timing on
//  sets with measured timing margins (sexyparo's TMS57002 test, ESC runs), so
//  it is left for its own board regression.  No set reads VCT (regs e/f;
//  MAME read tap, 11 sets, 3,000 frames: zero reads).
//  TODO(HARDWAREIZE): the V-edge phase (see above).
//
//  One deliberate structural difference: jtk053252 models the chip's actual
//  down-counters loaded with ~value, because it is reconstructed from silicon.
//  This uses plain up-counters against comparators.  Same raster, easier to
//  read against the MAME formulas, and the counters are not observable from
//  outside the chip.  If a discrepancy ever shows up, jtk053252 is the more
//  faithful of the two and wins.
//
//  ---- the numbers are measured, not assumed --------------------------------
//  docs/MEASUREMENTS.md section 2.  Gokujou Parodius writes, once, at boot:
//
//      HC 0x017F  HFP 0x0010  HBP 0x0030      -> htotal 384, hvis 288
//      VC 0x0107  VFP 0x11    VBP 0x0E        -> vtotal 264, vvis 224
//      reg c 0x73 -> VSW field 7, HSW field 3
//      DOTSEL = 00 -> 6 MHz
//
//      HSync 6 000 000 / 384 = 15 625.0 Hz
//      VSync 15 625 / 264    = 59.1856 Hz
//
//  This CONTRADICTS konamigx.cpp:1745, which claims 8 MHz and htotal 512 under
//  the comment "These parameters are actual value written to the CCU".  It
//  agrees with the set_visarea() that MAME immediately applies on top.  See
//  docs/UPSTREAM_TODO_AUDIT.md U3.
//
//  Nothing below is hardcoded to those values -- they are what the game writes
//  into the registers, and the RESET_* parameters only pre-load them so that a
//  core with a stalled CPU still produces sync.
//============================================================================
`default_nettype none

module gx_ccu (
    input  wire        clk,
    input  wire        rst,
    input  wire        pxl_cen,        // one pulse per dot (6 MHz for gokuparo)

    // --- CPU port.  8-bit device; the board puts it on lanes 0 and 2 of a
    //     32-bit long (umask32(0xff00ff00)), which the decoder handles.
    input  wire        cs,
    input  wire        we,
    input  wire [3:0]  addr,
    input  wire [7:0]  din,
    output wire [7:0]  dout,

    // --- raster ---------------------------------------------------------------
    output wire [9:0]  hcnt,           // 0 .. htotal-1
    output wire [8:0]  vcnt,           // 0 .. vtotal-1
    output wire        hs,
    output wire        vs,
    output wire        hblank,         // active high
    output wire        vblank,         // active high
    output wire        hdisp,          // active display, = ~hblank
    output wire        vdisp,

    // --- raster geometry, for engines that fetch AHEAD of the beam ------------
    //  A fetcher that runs N dots in front of the beam has to wrap that lead
    //  into the raster itself, and it cannot do that without the totals.  The
    //  tilemap used to add its lead to `hcnt` and let the sum run past
    //  `htotal`, which put the group that serves the FIRST EIGHT DOTS of every
    //  line at tile 48 of the PREVIOUS row.  These are the four numbers that
    //  make the wrap expressible; they are frame constants, already registered
    //  here, and this is the only place that owns them.
    output wire [9:0]  htotal_o,
    output wire [9:0]  vtotal_o,
    output wire [9:0]  hres_o,
    output wire [9:0]  vres_o,
    //  the back porch and the hsync width (dots), registered.  gx_top derives
    //  the per-set tile / sprite X offsets from them (MEASUREMENTS 174).
    output wire [8:0]  hbp_o,
    output wire [9:0]  hsw_dots_o,

    // --- interrupts -----------------------------------------------------------
    //  int1 = vblank        -> 68EC020 IRQ 1
    //  int2 = programmable  -> 68EC020 IRQ 2
    //  Both are cleared by a CPU write to registers 0x0e / 0x0f.  That is the
    //  real acknowledge path on this board: konamigx.cpp:1732 wires the CCU's
    //  ack callbacks to CLEAR_LINE on the CPU, so the game acknowledges the
    //  interrupt by writing the CCU, not by touching the CPU.
    output reg         int1,
    output reg         int2
);

// Reset values.  Gokujou Parodius' measured programming, so a core whose CPU
// has not run yet still emits a valid 15.625 kHz / 59.19 Hz signal instead of
// a dead screen.  docs/MEASUREMENTS.md section 2.
parameter [9:0] RESET_HC  = 10'h17F;
parameter [8:0] RESET_HFP =  9'h010;
parameter [8:0] RESET_HBP =  9'h030;
parameter [8:0] RESET_VC  =  9'h107;
parameter [7:0] RESET_VFP =  8'h11;
parameter [7:0] RESET_VBP =  8'h0E;
parameter [7:0] RESET_SW  =  8'h73;   // VSW in 7:4, HSW in 3:0

// ---------------------------------------------------------------------------
//  register file
// ---------------------------------------------------------------------------
reg [7:0] regs [0:15];

// Only e/f are readable, and they return VCT -- the current vertical count.
// k053252.cpp: "Read-only: e-f: bits 8-0: VCT", and it notes that viostorm and
// dbz read the port purely as a side effect of acknowledging the interrupt.
assign dout = (addr == 4'he) ? {7'd0, vcnt[8]} : vcnt[7:0];

wire [9:0] hc  = { regs[0][1:0], regs[1] };
wire [8:0] hfp = { regs[2][0],   regs[3] };
wire [8:0] hbp = { regs[4][0],   regs[5] };
wire [8:0] vc  = { regs[8][0],   regs[9] };
wire [7:0] vfp =   regs[10];
wire [7:0] vbp =   regs[11];
wire [3:0] vsw =   regs[12][7:4];
wire [3:0] hsw =   regs[12][3:0];

wire [7:0] int_time = regs[13];

// ---------------------------------------------------------------------------
//  derived raster geometry
//
//  k053252.cpp states the visible sizes as
//      Hres ~ (HC+1) - HFP - HBP - 8*(HSW+1)
//      Vres ~ (VC+1) - VFP - (VBP+1) - (VSW+1)
//  and notes "according to p.14-15 both HBP and VBP have +1 added, but to get
//  correct visible areas you have to add it only to VBP".
//
//  With gokuparo's measured values those give exactly 288 and 224, including
//  that asymmetry, so it is reproduced here rather than tidied up.
//  docs/MEASUREMENTS.md section 2 -- U4.
//
//  Layout of one line, in dots, starting from count 0 at the start of the
//  active display:
//      [0, hres)              active
//      [hres, +hfp)           front porch
//      [.., +8*(hsw+1))       hsync
//      [.., +hbp)             back porch   -> wraps at htotal
// ---------------------------------------------------------------------------
//  ---- and it is REGISTERED, in two stages ---------------------------------
//  These are FRAME CONSTANTS: they change only when the CPU reprograms the
//  CCU, which this game does once at boot.  Left combinational they were a
//  chain of six 10-bit carries hanging off CPU-written registers, and every
//  consumer inherited it -- the counters, the blanking, and the sync outputs
//  that leave for the MiSTer framework.
//
//  MEASURED, build K, 2026-09-07 -- the five worst paths in the design:
//
//      gx_ccu|regs[1][1] -> emu|arcade_video|VS     slack -2.302
//
//  Two stages rather than one because the chain is long enough that halving it
//  is nearly free: totals first, then the derived starts and ends.  Nothing
//  downstream can tell, because a geometry write takes effect two clocks later
//  instead of on the next comparison, and the counters are 16 system clocks
//  apart at this game's dot rate.
//
//  NOT gated on pxl_cen, deliberately.  The point here is to SHORTEN the path
//  from the CPU's registers, not to change when the raster moves; gating would
//  have made these dot-rate sources and left the arithmetic just as deep.
reg [9:0] htotal, vtotal, hsw_dots;
reg [8:0] hbp_r;
reg [9:0] hres, vres;
reg [9:0] hs_start, hs_end, vs_start, vs_end;

wire [9:0] htotal_c   = hc + 10'd1;
wire [9:0] vtotal_c   = { 1'b0, vc } + 10'd1;
wire [9:0] hsw_dots_c = { 3'd0, hsw, 3'd0 } + 10'd8;    // 8 * (HSW + 1)

always @(posedge clk) begin
    // stage 1 -- totals, straight off the register file
    htotal   <= htotal_c;
    vtotal   <= vtotal_c;
    hsw_dots <= hsw_dots_c;
    hbp_r    <= hbp;

    // stage 2 -- everything derived from them.  Reads the STAGE 1 registers,
    // which is what keeps this half short.
    hres     <= htotal - {1'b0, hfp} - {1'b0, hbp} - hsw_dots;
    vres     <= vtotal - {2'd0, vfp} - ({2'd0, vbp} + 10'd1) - ({6'd0, vsw} + 10'd1);
    // One dot before the end of the front porch: the silicon starts HSYNC at
    // the H counter's load, while its blank starts one registered dot after
    // the HFP compare (Furrtek 053252: E78 -> HBK_START flop; nNHSY set by
    // ~H29 directly).  So the front porch is HFP-1 dots and the back porch
    // HBP+1, active unchanged.  MEASUREMENTS 181.
    hs_start <= hres + {1'b0, hfp} - 10'd1;
    hs_end   <= hs_start + hsw_dots;
    vs_start <= vres + {2'd0, vfp};
    vs_end   <= vs_start + {6'd0, vsw} + 10'd1;
end

// ---------------------------------------------------------------------------
//  counters
// ---------------------------------------------------------------------------
reg [9:0] h;
reg [9:0] v;

assign hcnt = h;
assign vcnt = v[8:0];

assign htotal_o = htotal;
assign vtotal_o = vtotal;
assign hres_o   = hres;
assign hbp_o      = hbp_r;
assign hsw_dots_o = hsw_dots;
assign vres_o   = vres;

wire h_last = (h == htotal - 10'd1);
wire v_last = (v == vtotal - 10'd1);

assign hdisp  = (h < hres);
assign vdisp  = (v < vres);
assign hblank = ~hdisp;
assign vblank = ~vdisp;
assign hs     = (h >= hs_start) && (h < hs_end);
assign vs     = (v >= vs_start) && (v < vs_end);

// vblank edge, used for INT1
wire vb_start = h_last && (v == vres - 10'd1);

always @(posedge clk) begin
    if (rst) begin
        h <= 10'd0;
        v <= 10'd0;
    end else if (pxl_cen) begin
        if (h_last) begin
            h <= 10'd0;
            v <= v_last ? 10'd0 : v + 10'd1;
        end else begin
            h <= h + 10'd1;
        end
    end
end

// ---------------------------------------------------------------------------
//  interrupts
//
//  INT1 fires once per frame at the start of vertical blanking.  It has no
//  internal enable: the silicon-derived jtk053252 drives its INT1 edge latch
//  directly from VB and clears it with register 0x0e.  GX supplies the enable
//  outside the CCU in D56001 (`irq1_en` in gx_ctrl).  MAME's old register-map
//  names for 6/7 as INT1EN/INT2EN are not the silicon semantics; its GX driver
//  also leaves those callbacks unbound and gates IRQ1 only with D56001.
//
//  INT2 is the programmable one.  In jtk053252, writing register 0x0d both
//  loads INT-TIME and permanently starts the line counter until chip reset.
//  MAME does not model that timer; its GX driver substitutes scanline 48.
//  Gokujou Parodius never writes 0x0d, so INT2 remains inactive for this game.
// ---------------------------------------------------------------------------
reg [7:0] int2_cnt;
reg       int2en;

wire int1_ack = cs && we && (addr == 4'he);
wire int2_ack = cs && we && (addr == 4'hf);

always @(posedge clk) begin
    if (rst) begin
        int1     <= 1'b0;
        int2     <= 1'b0;
        int2_cnt <= 8'd0;
        int2en   <= 1'b0;
    end else begin
        // jtk053252's set_int2en is the rising edge of the register-0x0d
        // write event and has no clear other than reset.
        if (cs && we && (addr == 4'hd)) int2en <= 1'b1;

        if (pxl_cen) begin
            if (vb_start) int1 <= 1'b1;

            // one tick per line, at the end of the line
            if (h_last) begin
                if (!int2en) begin
                    int2_cnt <= int_time;
                end else if (int2_cnt == 8'd0) begin
                    int2_cnt <= int_time;
                    int2     <= 1'b1;
                end else begin
                    int2_cnt <= int2_cnt - 8'd1;
                end
            end
        end

        // Acknowledge wins over assert in the same cycle: the CPU writing the
        // ack register must always be able to clear the line, or a fast game
        // can wedge.  MAME's equivalent (vblank_irq_ack_w) is unconditional.
        if (int1_ack) int1 <= 1'b0;
        if (int2_ack) int2 <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
//  CPU writes
// ---------------------------------------------------------------------------
integer i;
always @(posedge clk) begin
    if (rst) begin
        for (i = 0; i < 16; i = i + 1) regs[i] <= 8'd0;
        regs[0]  <= {6'd0, RESET_HC[9:8]};
        regs[1]  <= RESET_HC[7:0];
        regs[2]  <= {7'd0, RESET_HFP[8]};
        regs[3]  <= RESET_HFP[7:0];
        regs[4]  <= {7'd0, RESET_HBP[8]};
        regs[5]  <= RESET_HBP[7:0];
        regs[8]  <= {7'd0, RESET_VC[8]};
        regs[9]  <= RESET_VC[7:0];
        regs[10] <= RESET_VFP;
        regs[11] <= RESET_VBP;
        regs[12] <= RESET_SW;
    end else if (cs && we) begin
        // e and f are acknowledge strobes, not storage.  Writing them must not
        // disturb the readback path.
        if (addr != 4'he && addr != 4'hf) regs[addr] <= din;
    end
end

endmodule

`default_nettype wire
