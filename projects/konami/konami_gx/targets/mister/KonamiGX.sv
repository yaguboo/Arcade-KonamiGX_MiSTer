//============================================================================
//  Konami System GX (Type 2) for MiSTer -- board wrapper
//
//  Wires the board-independent core in rtl/gx_top.sv to the MiSTer framework:
//  clocks, HPS I/O, ROM download, SDRAM and video.  Root CLAUDE.md section 4
//  keeps every hps_io / ioctl_ / MISTER_ name on this side of the line.
//
//  ============ STATE OF PLAY -- READ BEFORE BELIEVING ANYTHING ============
//
//  THIS BUILD IS NOT EXPECTED TO SHOW THE GAME.  It exists to produce a fit,
//  a timing result and -- the reason it is being built at all -- the fitter's
//  RAM summary.  Root CLAUDE.md section 7 and power_spikes L24: a memory that
//  cannot infer as block RAM becomes flip-flops with no warning, and the ALM
//  count does not say why.  Three memories here must infer: 128 KB tile VRAM,
//  128 KB work RAM, 32 KB palette.
//
//  What is in it:   68EC020, address decode, work RAM, control registers,
//                   CCU raster, K056832 tilemap, palette, K055555 priority,
//                   K054338 mixer.
//  What is not:     sprites, sound of any kind, object DMA.
//
//  Two things are known to be wrong before it is switched on, and one that
//  was, all written up in docs/DECISIONS.md so that time is not spent
//  rediscovering them:
//
//    D2  FIXED 2026-09-07.  The SDRAM controller was too slow for the tile
//        fetch -- 9 M word accesses/s wanted against a MEASURED 10.70 M/s
//        ceiling, which is 85.5 % of the per-group budget for the tilemap
//        alone.  The 4bpp fetch is now one burst-2 transaction instead of two
//        singles: 60.2 % of budget, measured by sim/tb_gx_sdram.sv.
//    D3  its read timing at 96 MHz is unverified and the arithmetic says the
//        data window falls between two core clock edges.  Settle it with a
//        self-test, not with more arithmetic.  NOTE that a green
//        tb_gx_sdram is NOT evidence here: it models the datasheet's cycle,
//        not this board's picoseconds.
//    --  fantjour.zip is missing from the build machine, so there are no
//        graphics ROMs at all.  Tiles will fetch zeros.
//
//  So: sync, a raster and a backdrop would be a good result.  A picture of
//  the game would be a surprise.
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 3 of the License, or (at your option)
//  any later version.
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Ports this core does not use /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

// No framebuffer: Gokujou Parodius is a horizontal game (MAME rotate=0), so
// nothing rotates and the .qsf does not define MISTER_FB.  The FB_* ports
// therefore DO NOT EXIST -- sys/emu_ports.vh declares them inside
// `ifdef MISTER_FB.  Assigning them anyway creates implicit nets that go
// nowhere and the synthesiser warns about every one.
// DDRAM_* is driven by the AUDIO RECORDER below (debug, OSD P1 "Audio
// recorder", default off); nothing in the core reads DDR3.

// Aspect ratio.  Nothing drives these by default and an undriven output is
// only a warning, so the picture would come out the wrong shape and nothing
// would say so.  288x224 on a 4:3 monitor is the original.
wire [1:0] ar = status[122:121];
assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

assign VGA_F1        = 0;
assign VGA_SCALER    = 0;
assign VGA_DISABLE   = 0;
assign HDMI_FREEZE   = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

// The board is genuinely stereo -- SOURCE_AUDIT section 14 has the two K054539s
// routed left and right -- so AUDIO_MIX stays 0 and mixing them would be a
// change, not a fix.  gx_sound sums and clips; nothing else happens here.
wire [15:0] snd_l, snd_r;
assign AUDIO_S   = 1;
assign AUDIO_L   = snd_l;
assign AUDIO_R   = snd_r;
assign AUDIO_MIX = 0;

// LED_DISK is {override enable, value}.  pll_alive toggles only if the PLL's
// third output is actually running, so this is a liveness lamp AND the real
// load that keeps Quartus from deleting the counter -- see the note at
// aux_cnt.  A dead disk LED on a first bring-up means the PLL did not come up
// the way the .qsf asked for, which is the single most expensive thing this
// factory has had go wrong silently.
assign LED_DISK  = {1'b1, pll_alive};
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;
assign BUTTONS   = 0;

//////////////////////////////////////////////////////////////////

`include "build_id.v"
localparam CONF_STR = {
	"KonamiGX;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[4:2],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	// The .mra carries the <switches> block.  Without this line the framework
	// loads those defaults at ioctl index 254 -- the board really does get
	// them -- but no menu ever shows them.
	"DIP;",
	"-;",
	// docs/OSD_POLICY.md 2.1: `On` first, so status = 0 -- what MiSTer starts a
	// fresh core with -- is On.  Bit 15, as power_spikes.
	"O[15],Pause when OSD open,On,Off;",
	// Voice boost: chip 2's channels 0-3 x0.8, 4-7 x2.0 -- the gains MAME's
	// [HACK] gives Dragoon Might (MEASUREMENTS 162), offered for EVERY set
	// (user, 2026-10-02: no per-game hardcoding).  MAME applies them to
	// dragoonj / dragoona only (and another pattern to tkmmpzdm), so On is
	// MAME's sound for those two and not for the rest.  Default Off = the chip
	// model.  The status bit stays 23 (root CLAUDE.md 1.4.5).
	"O[23],Voice boost,Off,On;",
	"-;",
	// Video source.  This is the bisection the sibling lane needed when its
	// first hardware bring-up came up blank: a test pattern generated here,
	// from clk_sys, with NO RESET AT ALL.  Bars visible means arcade_video,
	// the scaler and this file are fine and the fault is inside gx_top.
	// Default is the test pattern, because on a first build that is the more
	// informative of the two.
	"P1O[11],Video source,Test pattern,Core;",
	"P1O[14],Pause CPU,Off,On;",
	// docs/DECISIONS.md D5, settled on hardware 2026-09-13.  The same RBF
	// produced no RGB at x32 and real RGB in five of five x16 captures.  Keep
	// the losing setting as a diagnostic control; the .mra defaults to x16.
	"P1O[12],Palette scale,x32 (legacy),x16 (measured);",
	// The CPU liveness ladder.  Eight squares along the top; the first RED one
	// is where the CPU stopped.  Painted over whatever is underneath, so it is
	// default OFF and never corrupts a picture that is worth looking at.
	"P1O[13],CPU ladder,Off,On;",
	"P1O[16],Audio recorder (DDR3),Off,On;",
	"P1O[17],DDR3 probe,Off,On;",
	"P1O[18],Sound trace (DDR3),Off,On;",
	// The byte enable this core sends DDR3 on a READ of the sound half's
	// external memory.  Default is what every build so far has sent -- the
	// access's own partial mask -- so gokuparo is unchanged unless this is set.
	"P1O[19],DSP read byte enable,As written,All lanes;",
	// D21: the CPU's effective clock.  Default (status 0) is the CALIBRATED
	// rate; the legacy 24 MHz stays reachable so one bitstream can be measured
	// both ways rather than two builds compared across a netlist change.
	"P1O[20],CPU clock,20MHz calibrated,24MHz legacy;",
	// H1, MEASUREMENTS 113: the sprite group token D25 shipped at 3.  The
	// board lights ladder rung 3 -- a tile group edge arriving mid-fetch -- on
	// SELECT PLAYER while tb_gx_busmix reads tm_late 0 at every sound rate, so
	// the board is paying for that gate and the bench cannot price it.
	// Default (status 0) is the shipped 3; nothing changes unless this is set.
	// 2026-09-28 (MEASUREMENTS 144): the default is the deadline gate; the
	// bits have not moved, only what the values select.  3 is gone -- the
	// board rated it no better than 4.
	// Later the same day: gx_tilemap's two-group queue removes the late
	// tile groups, and 5 is the default (MEASUREMENTS 146).
	"P1O[22:21],Sprite group token,5 (shipped),Auto,4,0;",
	"P1,Debug;",
	"-;",
	"R[0],Reset;",
	// Gokujou Parodius is a two-button game (SOURCE_AUDIT section 7 gives the
	// port layout as L R U D B1 B2 B3 START per player, bit 0 first); the third
	// button is wired because the board reads it.  docs/OSD_POLICY.md 2.2: the
	// last J1 entry is always Pause, R on the pad.  releases/gokuparo.mra's
	// <buttons> must list these in this order, or the .mra maps by position
	// onto the wrong bits.
	// 2026-09-30: D, E, F for Dragoon Might (six buttons, konamigx.cpp
	// INPUT_PORTS dragoonj).  The three-button sets' .mra name them "-".
	"J1,Shot,Missile,C,D,E,F,Start,Coin,Pause;",
	"jn,A,B,X,Y,L,-,Start,Select,R;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire [21:0] gamma_bus;
wire [127:0] status;
wire  [1:0] buttons;

wire        ioctl_download;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] ioctl_index;
wire        ioctl_wait;

wire [31:0] joystick_0, joystick_1;

hps_io #(.CONF_STR(CONF_STR), .WIDE(0)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_menumask(16'd0),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_2(),
	.joystick_3(),

	// No keyboard input on this board.  Left open rather than tied to a wire
	// nothing reads -- docs/WARNING_POLICY.md does not accept a project-owned
	// "assigned a value but never read".
	.ps2_key(),

	// hps_io declares these inputs with no default, so leaving them open makes
	// them float.  NA-1/NA-2 lost a session to exactly that: an open
	// ioctl_wait goes onto HPS_BUS[37] as "core not ready" and hung the HPS
	// partway through the ROM load.  Tie every one of them off explicitly.
	.joystick_0_rumble(16'd0),
	.joystick_1_rumble(16'd0),
	.joystick_2_rumble(16'd0),
	.joystick_3_rumble(16'd0),
	.joystick_4_rumble(16'd0),
	.joystick_5_rumble(16'd0),
	.ps2_kbd_clk_in(1'b0),
	.ps2_kbd_data_in(1'b0),
	.ps2_kbd_led_status(3'd0),
	.ps2_kbd_led_use(3'd0),
	.ps2_mouse_clk_in(1'b0),
	.ps2_mouse_data_in(1'b0),
	.video_rotated(1'b0),
	.new_vmode(1'b0),
	// OSD defaults come from the .mra, not from a rebuild.  Bit 0 (Reset) is
	// masked off: seeding it would hold the core in reset from the moment the
	// .mra finished loading, which is a baffling way to find an .mra typo.
	.status_in({104'd0, mra_status[23:1], 1'b0}),
	.status_set(mra_status_set),
	.info_req(1'b0),
	.info(8'd0),
	.sd_lba('{default:32'd0}),
	.sd_blk_cnt('{default:6'd0}),
	.sd_rd(1'b0),
	.sd_wr(1'b0),
	.sd_buff_din('{default:8'd0}),
	.ioctl_upload(),
	.ioctl_upload_req(1'b0),
	.ioctl_upload_index(8'd0),
	.ioctl_din(8'd0)
);

///////////////////   .MRA-SUPPLIED OSD DEFAULTS   ///////////////////////
//
// MiSTer clears every status bit on a fresh arcade load, so the DEFAULT is
// what a session actually starts with.  Rebuilding the bitstream to flip a
// debug option costs the better part of an hour; editing three bytes in the
// .mra costs seconds.  Mechanism inherited from NA-1/NA-2 via power_spikes.
//
//   <rom index="1"><part>hh mm ll</part></rom>
//     hh = status[23:16]   mm = status[15:8]   ll = status[7:0]
reg [23:0] mra_status      = 24'd0;
reg        mra_status_seen = 1'b0;
reg        ioctl_dl_d      = 1'b0;
reg        mra_status_done = 1'b0;
reg        mra_status_set  = 1'b0;

// Byte 3 is board configuration (DECISIONS D18).  Its low nibble names WHICH
// MACHINE MAME's init_konamigx builds, and gx_top's port list carries the
// table.  It was two independent bits until 2026-09-22; the A2 shelf needs a
// third machine and the sprite X offset a third value, and a bit each does not
// scale.
//
// THE NUMBERS ARE CHOSEN SO NO .mra ALREADY SHIPPED CHANGES MEANING: sexyparo
// has always sent 0x01 and fantjour 0x02, which are now machines 1 and 2, and
// gokuparo has always sent no byte 3 at all, which reads 0.  Nothing ever sent
// 0x03, so the old "both bits" encoding names no set and loses nothing.
// Moving these would silently re-point every .mra in the field -- the same
// trap root CLAUDE.md 1.4.5 records for OSD status bits.
//
// gokuparo's .mra sends three bytes, so the field is cleared when byte 0
// arrives and a set loaded after sexyparo does not inherit it.
reg  [3:0] cfg_machine     = 4'd0;

always @(posedge clk_sys) begin
	// !ioctl_addr[26:2] so only the first FOUR bytes can land here.
	if (ioctl_wr && (ioctl_index == 16'd1) && !ioctl_addr[26:2]) begin
		case (ioctl_addr[1:0])
			2'd0: begin mra_status[23:16] <= ioctl_dout;
			            cfg_machine <= 4'd0; end
			2'd1: mra_status[15:8]  <= ioctl_dout;
			2'd2: begin mra_status[7:0] <= ioctl_dout; mra_status_seen <= 1'b1; end
			2'd3: cfg_machine <= ioctl_dout[3:0];
		endcase
	end
end

always @(posedge clk_sys) begin
	ioctl_dl_d     <= ioctl_download;
	mra_status_set <= 1'b0;
	if (ioctl_dl_d && !ioctl_download && mra_status_seen && !mra_status_done) begin
		// Only when NO saved settings were loaded.  MiSTer loads <setname>.CFG
		// into status BEFORE the ROM download, and status_set makes Main replace
		// all 128 bits with status_in (Main_MiSTer user_io.cpp
		// check_status_change) -- so pushing the .mra defaults unconditionally
		// overwrote every saved OSD setting on every load ("settings do not
		// save", reported 2026-10-07).  A saved file has some bit of [127:1]
		// set; a fresh load has none.  [0] is the reset bit Main pulses.
		mra_status_set  <= ~|status[127:1];
		mra_status_done <= 1'b1;
	end
end

///////////////////////   CLOCKS   ///////////////////////////////
//
// 96.000 MHz.  docs/DECISIONS.md D1: every clock this board needs is an
// integer divisor of it -- 24 MHz for the 68EC020, 8 for the sound 68000, 12
// for the DSP, and all four DOTSEL dot clocks (6, 8, 12, 16).  Unlike Power
// Spikes, no fractional enable is needed anywhere.
//
// 96 from a 50 MHz reference is 48/25, VCO 2400 MHz.  NA-1/NA-2's worst bug
// was a PLL that Quartus accepted, TimeQuest blessed and the fitter programmed
// with an out-of-range VCO, so the core clock never ran on hardware and
// nothing said a word.  build.sh runs tools/check_pll.py against the Fitter's
// PLL Usage Summary, which reports the counters as PROGRAMMED rather than as
// requested.  DO NOT REMOVE THAT CHECK.
//
// outclk_1 is the same frequency shifted half a period for SDRAM_CLK
// (-5208 ps at 96 MHz).  docs/DECISIONS.md D3 says that phase is very likely
// wrong at this frequency and explains what to do about it; it is left at the
// sibling's value so that the first hardware measurement has a known starting
// point rather than a second guess layered on the first.

wire clk_sys, clk_sdram, clk_aux, pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_sdram),
	.outclk_2(clk_aux),
	.locked(pll_locked)
);

assign SDRAM_CLK = clk_sdram;

// clk_aux is the third output, and it MUST HAVE A REAL LOAD.  Left
// unconnected, Quartus deletes the counter, re-solves the PLL with two outputs
// and can land on a different VCO -- which is exactly what happened to the
// sibling lane, and tools/check_pll.py caught it with "the fitter programmed 2
// outputs, 3 were expected".
//
// So it drives a counter whose top bit is observable.  That is a genuine
// instrument as well as a load: if pll_alive never toggles, that clock is not
// running.
reg [23:0] aux_cnt = 24'd0;
always @(posedge clk_aux) aux_cnt <= aux_cnt + 24'd1;

reg [2:0] aux_sync = 3'd0;
always @(posedge clk_sys) aux_sync <= {aux_sync[1:0], aux_cnt[23]};
wire pll_alive = aux_sync[2] ^ aux_sync[1];   // toggling => that clock runs

wire rst_sys = RESET | status[0] | buttons[1] | ~pll_locked;

// The core is held in reset while ROMs load.  The memory bus must NOT be: it
// is the thing doing the loading.
wire rst_mem = ~pll_locked;

///////////////////////   INPUT   ////////////////////////////////
//
// SOURCE_AUDIT section 7.  The INPUTS port at d5c000 is four players wide:
// P1 in bits 31-24, P2 23-16, P3 15-8, P4 7-0, all ACTIVE LOW.  Within each
// byte, konamigx.cpp:1255-1262 (P1):
//     bit 0 LEFT  1 RIGHT  2 UP  3 DOWN  4 BUTTON1  5 BUTTON2  6 BUTTON3  7 START
// gx_ctrl takes the four bytes already in that order and inverts nothing, so
// the inversion is here.
//
// Until 2026-09-14 this read the list MSB first -- LEFT on bit 7, START on
// bit 0 -- which the attract never pressed and so never showed.
//
// joystick bit order from the J1 line above:
//   0 right  1 left  2 down  3 up  4 Shot  5 Missile  6 C  7 D  8 E  9 F
//   10 Start  11 Coin  12 Pause    -- per pad, so P2 uses its own pad's Start/Select

wire [31:0] j1 = joystick_0;
wire [31:0] j2 = joystick_1;

//             bit 7   6       5       4       3 DOWN  2 UP    1 RIGHT 0 LEFT
wire [7:0] p1 = ~{ j1[10], j1[6], j1[5], j1[4], j1[2], j1[3], j1[0], j1[1] };
wire [7:0] p2 = ~{ j2[10], j2[6], j2[5], j2[4], j2[2], j2[3], j2[0], j2[1] };

// --- Pause ------------------------------------------------------------------
// docs/OSD_POLICY.md 2: the pad's Pause toggles, opening the OSD pauses unless
// "Pause when OSD open" is Off, and the P1 page's Pause CPU still forces it.
// The toggle is released on every download: a pause left over from before a
// load looks exactly like a hang.  power_spikes' wiring, same bits.
wire pause_btn = j1[12] | j2[12];
reg  pause_btn_d = 1'b0, pause_latch = 1'b0;
always @(posedge clk_sys) begin
	pause_btn_d <= pause_btn;
	if (ioctl_download)                pause_latch <= 1'b0;
	else if (~pause_btn_d & pause_btn) pause_latch <= ~pause_latch;
end
wire pause_core = status[14] | pause_latch | (OSD_STATUS & ~status[15]);
// The P3 byte (INPUTS bits 15-8) is on the board for every set.  Dragoon
// Might's harness puts buttons 4-6 there (konamigx.cpp INPUT_PORTS dragoonj):
// P1's at 14-12, P2's at 10-8, and this wrapper wires the pad that way for
// every set -- no machine-number gate (it was machine 6 only until
// 2026-10-06).  The other sets' .mra map no button to D/E/F ("-"), so for
// them j1/j2[9:7] stay 0 and the byte stays 0xFF, as before.
wire [7:0] p3 = ~{ 1'b0, j1[9], j1[8], j1[7], 1'b0, j2[9], j2[8], j2[7] };
wire [7:0] p4 = 8'hff;

wire [3:0] coin    = ~{ 2'b00, j2[11], j1[11] };
wire [1:0] service = 2'b11;

// DIP switches arrive from the .mra <switches> block at ioctl index 254.
// SW1 is the low byte, SW2 the high byte.  SOURCE_AUDIT section 7: SW1:1
// Sound Output defaults to 0 (Stereo) and SW1:2 Flip Screen to 1 (Off).
// SW1:1 is bit 0 and SW1:2 is bit 1, so the default byte is 1111_1110 = FE.
//
// It was FD until 2026-09-14: the right sentence with the two bits swapped,
// which booted every session MONO and FLIP SCREEN ON.  The game then wrote
// K056832 reg 0 = 0x0134 and an upside-down backdrop gradient -- measured by
// rebuilding the board's frame from a MAME run with Flip Screen forced On.
reg [15:0] dsw = 16'hFFFE;
always @(posedge clk_sys) begin
	if (ioctl_wr && (ioctl_index == 16'h00FE) && !ioctl_addr[26:1]) begin
		if (!ioctl_addr[0]) dsw[7:0]  <= ioctl_dout;
		else                dsw[15:8] <= ioctl_dout;
	end
end

///////////////////////   CORE   /////////////////////////////////

wire [24:0] mem_addr;
wire [15:0] mem_dout, mem_dout2, mem_din;
wire [15:0] mem_raw_q, mem_raw_w1, mem_raw_b0, mem_raw_b1;

// the sound half's external memory (gx_top sxm_*), served from DDR3 below
wire        sxm_req, sxm_we;
wire [21:0] sxm_word;
wire [63:0] sxm_wdata;
wire  [7:0] sxm_be;
reg         sxm_ack   = 1'b0;
reg  [63:0] sxm_rdata = 64'd0;
// the sound CPU's TMS57002 host-port log (rtl/sound/gx_sndtrace.sv), drained
// into DDR3 below.  status[18] = OSD "Sound trace", exclusive with the audio
// recorder (16) and the DDR3 probe (17), which share the same write path.
wire        trc_en = status[18] & ~status[16] & ~status[17];
// A READ's byte enable does not change memory and gx_sndxm picks its own lanes
// out of the word it gets back (gx_sndxm.sv d_take), so all-ones on a read is
// behaviourally neutral HERE -- what it changes is what the bridge is asked
// for.  It is a switch and not a fix because nothing in this repository says
// what the HPS f2h bridge does with byteenable != FF on a read, and this core
// is the only master on the platform that has ever sent one: sys/ddr_svc.sv
// and sys/ascal.vhd both hold it at all-ones.  MEASUREMENTS 65 proved the
// partial enables stick on WRITES by reading the memory back; the read side
// has never been tested on hardware or in simulation (the testbench model
// returned all eight lanes whatever the enable said, until +xmrd).
wire        rdbe_full = status[19];
wire        trc_valid;
wire [63:0] trc_data;
reg         trc_take = 1'b0;
wire        mem_req, mem_we, mem_burst, mem_burst4, mem_ack;
wire  [1:0] mem_ds;

wire [24:0] dl_addr;
wire [15:0] dl_data;
wire        dl_req, dl_ack, dl_active;

wire        nvs_wr;
wire  [5:0] nvs_addr;
wire [15:0] nvs_din;

wire [7:0]  vid_r, vid_g, vid_b;
wire        ce_pix, hblank, vblank, hsync, vsync;

gx_download u_download
(
	.clk(clk_sys),
	.rst(rst_mem),
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),
	.dl_addr(dl_addr),
	.dl_data(dl_data),
	.dl_req(dl_req),
	.dl_ack(dl_ack),
	.dl_active(dl_active),
	.nvs_wr(nvs_wr),
	.nvs_addr(nvs_addr),
	.nvs_din(nvs_din)
);

gx_top gx
(
	.clk(clk_sys),
	.rst(rst_sys),
	.grp_tok_sel(status[22:21]),
	.mem_rst(rst_mem),
	.pause(pause_core),
	.pal_gran16(status[12]),
	.cpu_cal(~status[20]),
	.voice_boost(status[23]),
	.machine(cfg_machine),

	.mem_addr(mem_addr),
	.mem_din(mem_din),
	.mem_dout(mem_dout),
	.mem_dout2(mem_dout2),
	.mem_raw_q(mem_raw_q),
	.mem_raw_w1(mem_raw_w1),
	.mem_raw_b0(mem_raw_b0),
	.mem_raw_b1(mem_raw_b1),
	.mem_req(mem_req),
	.mem_we(mem_we),
	.mem_burst(mem_burst),
	.mem_burst4(mem_burst4),
	.mem_ds(mem_ds),
	.mem_ack(mem_ack),

	.dl_active(dl_active),
	.dl_addr(dl_addr),
	.dl_data(dl_data),
	.dl_req(dl_req),
	.dl_ack(dl_ack),

	.nvs_wr(nvs_wr),
	.nvs_addr(nvs_addr),
	.nvs_din(nvs_din),

	.p1(p1), .p2(p2), .p3(p3), .p4(p4),
	.coin(coin),
	.service(service),
	.dip1(dsw[7:0]),
	.dip2(dsw[15:8]),

	.red(vid_r),
	.green(vid_g),
	.blue(vid_b),
	.hsync(hsync),
	.vsync(vsync),
	.hblank(hblank),
	.vblank(vblank),
	.ce_pix(ce_pix),
	.audio_l(snd_l),
	.audio_r(snd_r),

	.sxm_req(sxm_req),
	.sxm_we(sxm_we),
	.sxm_word(sxm_word),
	.sxm_wdata(sxm_wdata),
	.sxm_be(sxm_be),
	.sxm_ack(sxm_ack),
	.sxm_rdata(sxm_rdata),
	.trc_en(trc_en),
	.trc_valid(trc_valid),
	.trc_data(trc_data),
	.trc_take(trc_take),
	.trc_polls(),

	// A debug overlay like the sibling lane's is the obvious next instrument
	// and these are what it would read.  Left OPEN until it exists: a wire
	// nothing reads is a project-owned Quartus warning, and the logic behind
	// these ports is diagnostic, so letting the synthesiser drop it for now is
	// the honest outcome rather than a loss.
	// gx_top now ASSEMBLES the ladder as well as decoding it, so exactly one
	// debug bus crosses into the target and the rung order lives in the same
	// comment as the rung meaning.  Everything else is left OPEN the way this
	// file's header asks: a wire that is assigned and never read is Quartus
	// warning 10036, which this factory counts as project-owned.
	.dbg_rung(dbg_rung),
	.dbg_audio(dbg_audio),
	.dbg_live(),
	.dbg_addr(),
	.dbg_pc(dbg_pc),
	.dbg_video_en(),
	.dbg_enable(),
	.dbg_busstate(),
	.dbg_stalled(),
	.dbg_cpu_stall(),
	.dbg_winner()
);

///////////////////   AUDIO RECORDER (DEBUG)   //////////////////
//
// WHY.  The K054539 engines match MAME's model sample for sample in simulation
// and reach the model's loudness on the board (AUDIO band, MEASUREMENTS 42,
// 43) -- but nobody on this bench can listen, and loudness is not a waveform.
// With OSD P1 "Audio recorder" on, every 48 kHz output sample from frame
// REC_FROM on is written into the DE10-Nano's DDR3, so the HPS can read it back
// from /dev/mem and tools/gx_audrec.py can correlate it with
// tools/gx_k054539_model.py's output for the same frames.
//
//   byte base 0x30000000 (DDRAM_ADDR counts 64-bit words), 2^22 words = 32 MB,
//   about 87 s from REC_FROM, then it stops
//   word = {frame[19:0], sample-in-frame[11:0], AUDIO_L[15:0], AUDIO_R[15:0]}
//   frame = vblank rising edges since reset
//
// The write pattern is sys/arcade_video.v screen_rotate's: one word per write,
// burst 1, on the core's own clock.  Since the sound half's external memory
// arrived (DECISIONS D16) the recorder shares DDR3 with it and yields: rec_we
// is a request that the DDR3 adapter below completes (rec_done).
// DEBUG TRANSPORT ONLY: nothing in the core reads it, and it is off by default.
localparam [19:0] REC_FROM = 20'd2300;
wire       rec_en = status[16];
reg [10:0] rec_div   = 11'd0;
reg        rec_tick  = 1'b0;
reg        rec_vb_d  = 1'b0;
reg [19:0] rec_frame = 20'd0;
reg [11:0] rec_sub   = 12'd0;
reg [21:0] rec_idx   = 22'd0;
reg        rec_full  = 1'b0;
reg        rec_we    = 1'b0;
reg [28:0] rec_addr  = 29'd0;
reg [63:0] rec_din   = 64'd0;
always @(posedge clk_sys) begin
	rec_div  <= (rec_div == 11'd1999) ? 11'd0 : rec_div + 11'd1;
	rec_tick <= (rec_div == 11'd1999);
	rec_vb_d <= vblank;
	if (rst_sys) begin
		rec_frame <= 20'd0;
		rec_sub   <= 12'd0;
		rec_idx   <= 22'd0;
		rec_full  <= 1'b0;
		rec_we    <= 1'b0;
	end else begin
		if (vblank && !rec_vb_d) begin
			rec_frame <= rec_frame + 20'd1;
			rec_sub   <= 12'd0;
		end else if (rec_tick)
			rec_sub <= rec_sub + 12'd1;
		if (rec_we && rec_done)
			rec_we <= 1'b0;
		if (rec_tick && rec_en && !rec_full && !rec_we && rec_frame >= REC_FROM) begin
			rec_we   <= 1'b1;
			rec_addr <= 29'h600_0000 + {7'd0, rec_idx};
			rec_din  <= {rec_frame, rec_sub, snd_l, snd_r};
			rec_idx  <= rec_idx + 22'd1;
			if (rec_idx == 22'h3f_ffff) rec_full <= 1'b1;
		end
	end
end
///////////////////   DDR3 PROBE (DEBUG)   //////////////////////
//
// WHY.  The K054539 reverb ring (0x2000 x 16 bits a chip) and the TMS57002's
// delay memory (65,536 x 24-bit words, MEASURED 16 reads and 9 writes a sample,
// MEASUREMENTS 45) cannot be block RAM on this device (550 / 553 M10K), and
// SDRAM time is what the frame-drop work was paid for.  DDR3 is the one memory
// left -- and this lane has only ever WRITTEN it (the recorder above).  No
// MiSTer core in references/ reads it at run time (D14's survey).  So before an
// audio design is built on its read latency, the board measures it.
//
// With OSD P1 "DDR3 probe" on (and the recorder off), the probe runs back to
// back: write a pattern word into a scratch word, read it back, compare.  Every
// 1,024 pairs it logs one word:
//
//   scratch words at DDRAM_ADDR 0x6200000 (byte 0x31000000), 4,096 of them
//   log     words at DDRAM_ADDR 0x6300000 (byte 0x31800000), 2^16 of them
//   log word = {max read latency [63:48], max write latency [47:32],
//               sum of read latencies >> 10 [31:16], mismatches [15:8], 8'h5A}
//   latency = clocks from asserting RD / WE to DOUT_READY / !BUSY at 96 MHz
//
// tools/board_audrec.sh's mmap dump covers both regions.  Saturated on purpose:
// a real audio client asks for far fewer transactions (~60 a sample at worst).
// DEBUG TRANSPORT ONLY, off by default, nothing in the core reads it.
localparam [28:0] PRB_SCR = 29'h620_0000;
localparam [28:0] PRB_LOG = 29'h630_0000;
wire       prb_en = status[17] & ~status[16];
reg  [2:0] prb_st   = 3'd0;
reg        prb_we   = 1'b0, prb_rd = 1'b0;
reg [28:0] prb_addr = 29'd0;
reg [63:0] prb_din  = 64'd0, prb_pat = 64'd0;
reg [11:0] prb_k    = 12'd0;
reg [15:0] prb_lat  = 16'd0, prb_maxr = 16'd0, prb_maxw = 16'd0;
reg [25:0] prb_sumr = 26'd0;
reg  [7:0] prb_err  = 8'd0;
reg  [9:0] prb_n    = 10'd0;
reg [15:0] prb_log  = 16'd0;
reg [31:0] prb_seed = 32'h1234_5678;
always @(posedge clk_sys) begin
	if (!prb_en || rst_sys) begin          // RESET QUIESCES DDR3, see the sound master
		prb_st <= 3'd0; prb_we <= 1'b0; prb_rd <= 1'b0;
	end else begin
		if (prb_lat != 16'hFFFF) prb_lat <= prb_lat + 16'd1;
		case (prb_st)
			3'd0: begin                                        // write a pattern
				prb_seed <= {prb_seed[30:0], prb_seed[31] ^ prb_seed[21] ^ prb_seed[1] ^ prb_seed[0]};
				prb_pat  <= {prb_seed, ~prb_seed[15:0], 4'hA, prb_k};
				prb_din  <= {prb_seed, ~prb_seed[15:0], 4'hA, prb_k};
				prb_addr <= PRB_SCR + {17'd0, prb_k};
				prb_we   <= 1'b1;
				prb_lat  <= 16'd0;
				prb_st   <= 3'd1;
			end
			3'd1: if (!DDRAM_BUSY) begin                       // accepted this clock
				prb_we <= 1'b0;
				if (prb_lat > prb_maxw) prb_maxw <= prb_lat;
				prb_rd  <= 1'b1;
				prb_lat <= 16'd0;
				prb_st  <= 3'd2;
			end
			3'd2: begin                                        // read it back
				if (!DDRAM_BUSY) prb_rd <= 1'b0;
				if (DDRAM_DOUT_READY) begin
					prb_rd <= 1'b0;
					if (prb_lat > prb_maxr) prb_maxr <= prb_lat;
					prb_sumr <= prb_sumr + {10'd0, prb_lat};
					if (DDRAM_DOUT != prb_pat && prb_err != 8'hFF) prb_err <= prb_err + 8'd1;
					prb_k <= prb_k + 12'd1;
					prb_n <= prb_n + 10'd1;
					prb_st <= (prb_n == 10'd1023) ? 3'd3 : 3'd0;
				end
			end
			3'd3: begin                                        // log 1,024 pairs
				prb_addr <= PRB_LOG + {13'd0, prb_log};
				prb_din  <= {prb_maxr, prb_maxw, prb_sumr[25:10], prb_err, 8'h5A};
				prb_we   <= 1'b1;
				prb_st   <= 3'd4;
			end
			3'd4: if (!DDRAM_BUSY) begin
				prb_we   <= 1'b0;
				prb_log  <= prb_log + 16'd1;
				prb_maxr <= 16'd0; prb_maxw <= 16'd0; prb_sumr <= 26'd0; prb_err <= 8'd0;
				prb_st   <= 3'd0;
			end
			default: prb_st <= 3'd0;
		endcase
	end
end

///////////////////   SOUND EXTERNAL MEMORY -> DDR3   //////////////////
//
// gx_top's sxm port -- gx_sndxm: both K054539 reverb rings and the TMS57002's
// data RAM, one 64-bit word per op (DECISIONS D16) -- is DDR3 at DDRAM_ADDR
// 0x6800000 (byte 0x34000000), 0x9000 words, clear of the recorder (0x6000000,
// 2^22 words) and the probe (0x6200000 / 0x6300000).  One op at a time; the
// recorder's write waits behind a pending sound op.  With the probe on, the
// probe owns DDR3 and the sound memory waits (debug only).
//
// Handshake as ddr_svc and the probe: RD / WE held until !BUSY accepts them; a
// read's word arrives with DOUT_READY afterwards.
localparam [28:0] SXM_BASE = 29'h680_0000;
// The sound trace: one 64-bit word per event at DDRAM_ADDR 0x6400000 (byte
// 0x32000000), 2^16 words, clear of the recorder (0x6000000 + 2^22 words ends
// at 0x63FFFFF) and of the sound memory (0x6800000).  It reuses the recorder's
// write path: a word is copied into trc_din and trc_we is a request that
// dd_st completes with rec_done.  The log stops when full rather than wrapping,
// so what survives is the BEGINNING of the run -- the RAM CHECK.
localparam [28:0] TRC_BASE = 29'h640_0000;
reg        trc_we   = 1'b0;
reg [28:0] trc_addr = 29'd0;
reg [63:0] trc_din  = 64'd0;
reg [15:0] trc_idx  = 16'd0;
reg        trc_full = 1'b0;
localparam [1:0]  DD_IDLE = 2'd0, DD_XM = 2'd1, DD_REC = 2'd2, DD_GAP = 2'd3;
reg  [1:0] dd_st   = DD_IDLE;
reg        dd_we   = 1'b0, dd_rd = 1'b0;
reg [28:0] dd_addr = 29'd0;
reg [63:0] dd_din  = 64'd0;
reg  [7:0] dd_be   = 8'hFF;
reg        rec_done = 1'b0;
// RESET QUIESCES DDR3, 2026-09-29.  Until now this master had no reset at all,
// and it is the only core-side DDR3 master in the factory that runs during
// play (the DSP's memory, every sample).  Four boards hung coming back from
// `reboot` -- every one with this core running, none in any other lane -- and
// an HPS warm reboot resets the DDR3 controller under a master that may still
// be holding RD or WE up.  UNVERIFIED as the cause; it is the one difference.
// On rst_sys (which carries the HPS reset, sysmem reset_out) nothing new is
// started and RD / WE drop at once.  A read the controller had already taken
// still owes a DOUT_READY: it is swallowed (dd_drain) so it cannot answer the
// next read, with a bound in case the controller that owed it was reset too.
reg        dd_drain = 1'b0;
reg  [9:0] dd_drain_t = 10'd0;
always @(posedge clk_sys) begin
	sxm_ack  <= 1'b0;
	rec_done <= 1'b0;
	if (dd_drain) begin
		dd_drain_t <= dd_drain_t + 10'd1;
		if (DDRAM_DOUT_READY || &dd_drain_t) dd_drain <= 1'b0;
	end
	if (rst_sys) begin
		// a read accepted (RD already dropped) and still unanswered
		if (dd_st == DD_XM && !dd_we && !dd_rd) begin
			dd_drain   <= 1'b1;
			dd_drain_t <= 10'd0;
		end
		dd_st <= DD_IDLE;
		dd_we <= 1'b0;
		dd_rd <= 1'b0;
	end else if (prb_en || dd_drain) begin
		dd_st <= DD_IDLE;
		dd_we <= 1'b0;
		dd_rd <= 1'b0;
	end else begin
		case (dd_st)
			DD_IDLE:
				if (sxm_req) begin
					dd_addr <= SXM_BASE + {7'd0, sxm_word};
					dd_din  <= sxm_wdata;
					dd_be   <= (sxm_we || !rdbe_full) ? sxm_be : 8'hFF;
					dd_we   <= sxm_we;
					dd_rd   <= ~sxm_we;
					dd_st   <= DD_XM;
				end else if (rec_we || trc_we) begin
					dd_addr <= trc_we ? trc_addr : rec_addr;
					dd_din  <= trc_we ? trc_din  : rec_din;
					dd_be   <= 8'hFF;
					dd_we   <= 1'b1;
					dd_st   <= DD_REC;
				end
			DD_XM:
				if (dd_we) begin
					if (!DDRAM_BUSY) begin
						dd_we   <= 1'b0;
						sxm_ack <= 1'b1;
						dd_st   <= DD_GAP;
					end
				end else begin
					if (dd_rd && !DDRAM_BUSY) dd_rd <= 1'b0;
					if (DDRAM_DOUT_READY) begin
						dd_rd     <= 1'b0;
						sxm_rdata <= DDRAM_DOUT;
						sxm_ack   <= 1'b1;
						dd_st     <= DD_GAP;
					end
				end
			DD_REC:
				if (!DDRAM_BUSY) begin
					dd_we    <= 1'b0;
					rec_done <= 1'b1;
					dd_st    <= DD_GAP;
				end
			default: dd_st <= DD_IDLE;           // DD_GAP: the served side lets go
		endcase
	end
end

always @(posedge clk_sys) begin
	trc_take <= 1'b0;
	if (!trc_en) begin
		trc_we   <= 1'b0;
		trc_idx  <= 16'd0;
		trc_full <= 1'b0;
	end else begin
		if (trc_we && rec_done) trc_we <= 1'b0;
		if (trc_valid && !trc_take && !trc_we && !trc_full) begin
			trc_addr <= TRC_BASE + {13'd0, trc_idx};
			trc_din  <= trc_data;
			trc_we   <= 1'b1;
			trc_take <= 1'b1;
			trc_idx  <= trc_idx + 16'd1;
			if (trc_idx == 16'hFFFF) trc_full <= 1'b1;
		end
	end
end

assign DDRAM_CLK      = clk_sys;
assign DDRAM_BURSTCNT = 8'd1;
assign DDRAM_ADDR     = prb_en ? prb_addr : dd_addr;
assign DDRAM_DIN      = prb_en ? prb_din  : dd_din;
assign DDRAM_BE       = prb_en ? 8'hFF    : dd_be;
assign DDRAM_WE       = prb_en ? prb_we   : dd_we;
assign DDRAM_RD       = prb_en ? prb_rd   : dd_rd;

///////////////////   CPU LIVENESS LADDER   //////////////////////
//
// Eight squares along the top of the picture, painted only when the OSD bit
// is set.  It exists because the first core-video run came back PURE BLACK,
// and black is exactly what an unprogrammed board looks like: every register
// at its reset value means the backdrop wins, the backdrop is palette entry
// 0, and that entry is zero.  So a black screen cannot tell "the CPU is dead"
// from "the CPU has not got there yet" -- and the answer decides whether to
// look at gx_main or at the video path.
//
// It changes no BEHAVIOUR.  It does not follow that it changes no TIMING, and
// the first version of this ladder proved the difference the expensive way:
// the decode was up here, comparing gx_main's CPU-rate `dbg_addr` across the
// chip into seven 96 MHz registers with no exception, and the build came back
// at -3.913 with TNS -97.8 -- against a few tenths for everything else in it.
//
// So the decode moved into gx_top, beside the register it reads, and only
// seven slow bits cross the boundary.  What is left here is painting.
//
// Read it left to right.  The ASSIGNMENT, the rung table and the reasoning
// behind them live in rtl/gx_top.sv, beside the registers that produce them,
// and are NOT repeated here -- two copies of a rung table is how this project
// produced a confident wrong reading once already.  What is here is painting.
//
// The per-frame rungs are latched at vblank and held for the whole of the
// NEXT frame, so they are stable while the overlay is painted at rows 8-23.
// tools/read_ladder.py prints "~" for a square that changes during the scan
// and nothing here should print one.
//
// Default OFF, so it can never corrupt a picture worth looking at.
wire [7:0] dbg_rung;
wire [7:0] ladder = dbg_rung;
wire [7:0] dbg_audio;          // second row, rows 28-43: gx_top's AUDIO band

// The core does not hand out its raster counters, so the overlay derives its
// own from the blanking it does hand out.  That is deliberate: counting the
// signal that will actually be displayed means the squares cannot drift away
// from the picture they are drawn on.
reg  [9:0] ov_x = 10'd0;
reg  [8:0] ov_y = 9'd0;
reg        hb_d = 1'b0, vb_d = 1'b0;
always @(posedge clk_sys) begin
	if (ce_pix) begin
		hb_d <= hblank;
		vb_d <= vblank;
		if (hblank) begin
			ov_x <= 10'd0;
			if (!hb_d) ov_y <= vblank ? 9'd0 : ov_y + 9'd1;   // one per line
		end else begin
			ov_x <= ov_x + 10'd1;
		end
		if (vblank && !vb_d) ov_y <= 9'd0;
	end
end

// Geometry: eight 16-wide squares on a 32 px pitch, 16 px in, 16 rows tall.
//
// EVERY number here is a power of two, and that is not tidiness.  The first
// version used a 24 px pitch and wrote the obvious thing --
//
//     (((dbg_x - 8) % 24) < 16)      and      (dbg_x - 8) / 24
//
// -- which is a modulo and a divide by a non-power-of-two, ten bits wide,
// combinationally into the video mux.  MEASURED: it became the critical path
// of the whole design at -4.302, `ov_x[5] -> arcade_video|RGB_fix[22]`, and
// it did that in TWO consecutive builds, hiding the cen_d2 change both times.
// STATUS already had "a DIVIDER on the critical path" in its timing list from
// an earlier hunt; this is the same mistake in a diagnostic.
//
// Registered as well, on ce_pix, so nothing here reaches RGB_fix through
// combinational logic at all.  It costs the overlay one pixel of offset,
// which for eight coloured squares is nothing.
wire       dbg_on = status[13];
wire [9:0] dbg_rx = ov_x - 10'd16;
wire [2:0] dbg_idx = dbg_rx[7:5];

// THE PC SAMPLE (gx_top): the main CPU's opcode-fetch address, one a frame,
// as three more rows at 48-63 / 80-95 / 112-127 -- bits 23-16, 15-8, 7-0,
// MOST significant bit LEFT (the other two rows are bit 0 left).  Green = 1.
// The row test is ov_y[8:4], so it costs no comparator.
wire [22:0] dbg_pc;
wire [23:0] dbg_pc_b = {dbg_pc, 1'b0};
wire [3:0]  dbg_row  = ov_y[7:4];
wire        dbg_pcrow = !ov_y[8] && ((dbg_row == 4'd3) || (dbg_row == 4'd5) || (dbg_row == 4'd7));
wire [7:0]  dbg_pcbyte = (dbg_row == 4'd3) ? dbg_pc_b[23:16] :
                         (dbg_row == 4'd5) ? dbg_pc_b[15:8]  : dbg_pc_b[7:0];

reg        dbg_band = 1'b0;
reg [23:0] dbg_rgb  = 24'd0;
always @(posedge clk_sys) if (ce_pix) begin
	// Two rows: the ladder at 8-23 and the AUDIO band at 28-43 (gx_top says
	// what its squares mean).  Same pitch, same colours, both registered.
	dbg_band <= dbg_on && (((ov_y >= 9'd8) && (ov_y < 9'd24)) || ((ov_y >= 9'd28) && (ov_y < 9'd44)) || dbg_pcrow)
	                   && (ov_x >= 10'd16) && (ov_x < 10'd16 + 10'd256)
	                   && (dbg_rx[4] == 1'b0);          // 16 on, 16 off
	dbg_rgb  <= (dbg_pcrow           ? dbg_pcbyte[3'd7 - dbg_idx] :
	             (ov_y >= 9'd28)     ? dbg_audio[dbg_idx] : ladder[dbg_idx]) ? 24'h00C000 : 24'hC00000;
end

///////////////////////   SDRAM   ////////////////////////////////

reg  [3:0] sdram_init_cnt = 0;
wire       sdram_init = ~sdram_init_cnt[3];
always @(posedge clk_sys) begin
	if (!pll_locked)     sdram_init_cnt <= 0;
	else if (sdram_init) sdram_init_cnt <= sdram_init_cnt + 1'd1;
end

// HOLD_DOUT 0: nothing reads the words after the ack clock any more (the
// two-tier return, gx_memarb RESP_REG), so the hold registers are not built.
gx_sdram #(.CLK_HZ(96_000_000), .REFRESH_CLK(700), .HOLD_DOUT(1'b0)) u_sdram
(
	.ref_sync   (1'b0),       // free-running; MEASUREMENTS 120 has the candidate
	.clk        (clk_sys),
	.init       (sdram_init),
	.addr       (mem_addr),
	.din        (mem_din),
	.dout       (mem_dout),
	.dout2      (mem_dout2),
	.raw_q      (mem_raw_q),
	.raw_w1     (mem_raw_w1),
	.raw_b0     (mem_raw_b0),
	.raw_b1     (mem_raw_b1),
	.req        (mem_req),
	.we         (mem_we),
	.burst      (mem_burst),
	.burst4     (mem_burst4),
	.ds         (mem_ds),
	.ack        (mem_ack),
	.SDRAM_A    (SDRAM_A),
	.SDRAM_BA   (SDRAM_BA),
	.SDRAM_DQ   (SDRAM_DQ),
	.SDRAM_DQML (SDRAM_DQML),
	.SDRAM_DQMH (SDRAM_DQMH),
	.SDRAM_nCS  (SDRAM_nCS),
	.SDRAM_nWE  (SDRAM_nWE),
	.SDRAM_nRAS (SDRAM_nRAS),
	.SDRAM_nCAS (SDRAM_nCAS),
	.SDRAM_CKE  (SDRAM_CKE)
);

///////////////////////   VIDEO   ////////////////////////////////

wire [2:0] fx = status[4:2];

// VIDEO BISECTION.  A test pattern generated here from clk_sys with NO RESET
// AT ALL -- deliberately, because a stuck rst_sys is one of the two candidate
// causes of a blank first bring-up and a pattern sharing the suspect reset
// would prove nothing.  The sibling lane needed exactly this.
//
//   bars visible  -> arcade_video, the scaler and this file are fine, and the
//                    fault is inside gx_top
//   still blank   -> the fault is at the target level or in the clock itself
//
// The raster below is this board's own, measured: 384 x 264 total, 288 x 224
// visible, sync where the CCU puts it (docs/MEASUREMENTS.md section 2).  So a
// pattern and the core produce the same shape and the scaler does not have to
// be re-taught between them.
reg [3:0] tp_div = 4'd0;
reg       tp_ce  = 1'b0;
reg [9:0] tp_h   = 10'd0;
reg [8:0] tp_v   = 9'd0;

always @(posedge clk_sys) begin
	tp_div <= tp_div + 4'd1;                 // 96 MHz / 16 = 6 MHz
	tp_ce  <= (tp_div == 4'd15);
	if (tp_ce) begin
		if (tp_h == 10'd383) begin
			tp_h <= 10'd0;
			tp_v <= (tp_v == 9'd263) ? 9'd0 : tp_v + 9'd1;
		end else begin
			tp_h <= tp_h + 10'd1;
		end
	end
end

wire       tp_hb = (tp_h >= 10'd288);
wire       tp_vb = (tp_v >= 9'd224);
wire       tp_hs = (tp_h >= 10'd304) && (tp_h < 10'd336);
wire       tp_vs = (tp_v >= 9'd241) && (tp_v < 9'd249);
// Coarse vertical bars plus a horizontal ramp: any sweep at all is obvious,
// and a frozen counter shows as a flat colour.
wire [7:0] tp_r = {8{tp_h[5]}};
wire [7:0] tp_g = {8{tp_v[5]}};
wire [7:0] tp_b = tp_h[7:0];

wire use_core = status[11];

wire        v_ce  = use_core ? ce_pix : tp_ce;
wire        v_hb  = use_core ? hblank : tp_hb;
wire        v_vb  = use_core ? vblank : tp_vb;
wire        v_hs  = use_core ? hsync  : tp_hs;
wire        v_vs  = use_core ? vsync  : tp_vs;
wire [23:0] v_rgb_sel = use_core ? {vid_r, vid_g, vid_b} : {tp_r, tp_g, tp_b};
// The ladder paints over whatever is underneath, core or pattern, and only
// when its OSD bit is on.
wire [23:0] v_rgb = dbg_band ? dbg_rgb : v_rgb_sel;

// 288 visible pixels -- measured, docs/MEASUREMENTS.md section 2, NOT the 384
// that konamigx.cpp:1745's set_raw claims.  WIDTH is what arcade_video uses
// for the "Original" aspect ratio; getting it wrong stretches the picture and
// nothing errors.
arcade_video #(.WIDTH(288), .DW(24)) arcade_video
(
	.*,
	.clk_video(clk_sys),
	.ce_pix(v_ce),
	.RGB_in(v_rgb),
	.HBlank(v_hb),
	.VBlank(v_vb),
	.HSync(v_hs),
	.VSync(v_vs),
	.fx(fx)
);

endmodule
