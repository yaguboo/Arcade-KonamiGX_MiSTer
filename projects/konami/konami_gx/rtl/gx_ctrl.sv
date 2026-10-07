//============================================================================
//  Konami System GX -- board control registers and input readback
//
//  Four small things that all live on the same few addresses:
//
//      d56000  eeprom_w    EEPROM pins, coin counters, backdrop select,
//                          watchdog, and the IRQ enable byte
//      d58000  control_w   DOTSEL, sound reset, OBJCHA, sprite priority
//                          source select, graphics chip reset
//      d5a000  read        DIP switches, coin/service, EEPROM data out
//      d5c000  read        four players' controls
//      d5e000  read        service port
//
//  Transcribed from docs/SOURCE_AUDIT.md section 7, which is itself
//  konamigx.cpp:472, :498, :520 and the PORT_START blocks at :1219 and :1264.
//
//  ---- byte lanes, which is the whole difficulty here -----------------------
//  Both write registers are 32-bit writes whose useful bits live in the upper
//  half of the long.  TG68K splits every long into two word accesses, so on
//  our 16-bit bus the useful half is the word at offset 0 and:
//
//      d56000 word 0, D[15:8]   = eeprom_w bits 31-24  (wrport1_0)
//      d56000 word 0, D[ 7:0]   = eeprom_w bits 23-16  (wrport1_1, IRQ enables)
//      d58000 word 0, D[ 7:0]   = control_w bits 23-16 (wrport2)
//
//  Bit 23 of control_w is therefore D[7] and DOTSEL0 is D[0].  Getting this
//  off by eight bits would make the dot clock 12 MHz and the raster wrong, and
//  it would look like a CCU bug.  Held down by tb_gx_ctrl.
//
//  Reads are 32-bit too, so each port answers two word addresses:
//
//      d5a000 word 0 = { DIP SW1, DIP SW2 }
//      d5a002 word 1 = { coin/service, EEPROM DO + status }
//      d5c000 word 0 = { P1, P2 }
//      d5c002 word 1 = { P3, P4 }
//
//  Everything except the EEPROM data line is ACTIVE LOW (konamigx.cpp:1229
//  onwards).  The DIP defaults are the second argument of each PORT_DIPNAME:
//  Sound Output = Stereo = 0, Flip Screen = Off = 1, everything else 1.
//  Sound Output is SW1 bit 0 and Flip Screen bit 1, so SW1 = 1111_1110 = FE.
//============================================================================
`default_nettype none

module gx_ctrl (
    input  wire        clk,
    input  wire        rst,

    // --- CPU ----------------------------------------------------------------
    input  wire        eeprom_cs,
    input  wire        control_cs,
    input  wire        sysdsw_cs,
    input  wire        inputs_cs,
    input  wire        service_cs,
    input  wire        we,
    input  wire        a1,             // cpu_addr[1]: which word of the long
    input  wire [15:0] din,
    input  wire [1:0]  ds,             // {uds, lds}, active high
    output reg  [15:0] dout,

    // --- control_w outputs --------------------------------------------------
    output wire [1:0]  dotsel,         // 0=6 1=8 2=12 3=16 MHz
    output wire        snd_run,        // bit 22: 0 = hold the 68000 in reset
    output wire        vram_chard,     // bit 21: 0 = VRAM, 1 = tile ROM window
    output wire        objcha,         // bit 20: sprite ROM readback enable
    output wire        spri_sel19,     // bit 19
    output wire        spri_sel18,     // bit 18
    // Bit 23.  MAME's comment calls it "reset graphics chips" and MAME's CODE
    // never reads it -- konamigx.cpp:537 resets the K056832 from bit 22, the
    // SOUND reset, instead.  So the bit's polarity has no source at all, and
    // the measured trace shows the game ending at wrport2 = 0xC8, i.e. bit 23
    // SET.  If it were an active-high reset the graphics chips would be held
    // in reset for the whole game.
    //
    // Decoded, exposed, and NOT wired to anything.  Named _n because bit 22
    // one place along is documented as "0 to halt, 1 to let it run", so this
    // bank reads as active-low releases -- but that is an inference and the
    // cost of acting on a wrong one is a black screen hunted in the video path.
    // TODO(HARDWAREIZE): what bit 23 actually resets, and in which sense.
    output wire        gfx_rst_n,      // bit 23

    // --- eeprom_w outputs ---------------------------------------------------
    output wire        watchdog,       // bit 31 "afr"
    output wire        objscan,        // bit 30
    output wire        bgc_from_pal,   // bit 29: 0 = '338 solid, 1 = 5^5 fill
    output wire [1:0]  coin_ctr,       // bits 28-27
    output wire        ee_clk,         // bit 26
    output wire        ee_cs,          // bit 25
    output wire        ee_di,          // bit 24
    output wire        irq1_en,        // vblank
    output wire        irq2_en,        // programmable scanline
    output wire        irq3_en,        // object DMA end
    output wire        irq4_en,        // ESC run end -- bit 4 ALONE, see below
    output reg         irq1_sync_set,  // one clock: an enable write carrying 0x81 (gx_top, IRQ1 SYNC)

    // --- board inputs, all ACTIVE LOW except ee_do --------------------------
    input  wire [7:0]  p1, p2, p3, p4, // bit 0 L, 1 R, 2 U, 3 D, 4-6 B1-B3, 7 START
    input  wire [3:0]  coin,
    input  wire [1:0]  service,
    input  wire [7:0]  dip1,
    input  wire [7:0]  dip2,
    input  wire [7:0]  svc_port,       // d5e000 bits 31-24, bit 3 = service
    input  wire        ee_do,
    input  wire [6:0]  rdport1_3       // objdma / int status bits
);

// ---------------------------------------------------------------------------
//  write registers
//
//  Reset values.  wrport1_1 = 0 means every interrupt masked, which is where
//  the board starts: the measured trace never writes anything else
//  (docs/MEASUREMENTS.md section 6).  wrport2 = 0 means DOTSEL 6 MHz, sound
//  CPU held in reset and OBJCHA low -- all the safe ends.
// ---------------------------------------------------------------------------
reg [7:0] wrport1_0;    // eeprom_w bits 31-24
reg [7:0] wrport1_1;    // eeprom_w bits 23-16
reg [7:0] wrport2;      // control_w bits 23-16

always @(posedge clk) begin
    if (rst) begin
        wrport1_0 <= 8'h00;
        wrport1_1 <= 8'h00;
        wrport2   <= 8'h00;
        irq1_sync_set <= 1'b0;
    end else begin
        // konamigx.cpp:515-516: a write with the master bit ORs the source bits
        // into m_gx_syncen.  Only bit 0 is carried out (gx_top, IRQ1 SYNC).
        irq1_sync_set <= eeprom_cs && we && !a1 && ds[0] && din[7] && din[0];
        if (eeprom_cs && we && !a1) begin
            if (ds[1]) wrport1_0 <= din[15:8];
            if (ds[0]) wrport1_1 <= din[ 7:0];
        end
        if (control_cs && we && !a1) begin
            if (ds[0]) wrport2 <= din[7:0];
        end
    end
end

assign dotsel       = wrport2[1:0];
assign spri_sel18   = wrport2[2];
assign spri_sel19   = wrport2[3];
assign objcha       = wrport2[4];
assign vram_chard   = wrport2[5];
assign snd_run      = wrport2[6];
assign gfx_rst_n    = wrport2[7];

assign ee_di        = wrport1_0[0];
assign ee_cs        = wrport1_0[1];
assign ee_clk       = wrport1_0[2];
assign coin_ctr     = wrport1_0[4:3];
assign bgc_from_pal = wrport1_0[5];
assign objscan      = wrport1_0[6];
assign watchdog     = wrport1_0[7];

// "The enable test is (m_gx_wrport1_1 & 0x8N) == 0x8N -- master bit AND the
//  per-source bit."  konamigx.cpp:498, SOURCE_AUDIT section 6.
assign irq1_en = wrport1_1[7] & wrport1_1[0];
assign irq2_en = wrport1_1[7] & wrport1_1[1];
assign irq3_en = wrport1_1[7] & wrport1_1[2];

// IRQ4 is the exception, and it is not a slip from the pattern above: esc_w
// tests `m_gx_wrport1_1 & 0x10` with NO master bit (konamigx.cpp:444) and
// clears rdport1_3 bit 3 inside the same test (:446).  sexyparo writes 0x91.
// DECISIONS D18.
assign irq4_en = wrport1_1[4];

// ---------------------------------------------------------------------------
//  readback
//
//  m_gx_syncen: bit 0 IS modelled now (gx_top, IRQ1 SYNC) because tbyahhoo
//  deadlocks without it (MEASUREMENTS 149).  Bits 1 and 2 are not: IRQ2 is
//  never enabled by any set here and IRQ3 is not generated.  UPSTREAM_TODO U9
//  reopened.
// ---------------------------------------------------------------------------
always @(*) begin
    dout = 16'hffff;
    if (sysdsw_cs)
        // The EEPROM data line is bit 0 of the long, NOT the top of the byte:
        // konamigx.cpp:1272 gives it mask 0x00000001 and hands bits 7-1 to
        // gx_rdport1_3_r, which returns `m_gx_rdport1_3 >> 1` (:469).  So the
        // status byte is shifted up by one and the EEPROM sits underneath it.
        // tb_gx_ctrl caught this the wrong way round on its first run.
        dout = a1 ? { coin_svc, rdport1_3, ee_do }   // d5a002: bits 15-0
                  : { dip1, dip2 };                  // d5a000: bits 31-16
    else if (inputs_cs)
        dout = a1 ? { p3, p4 }                       // d5c002
                  : { p1, p2 };                      // d5c000
    else if (service_cs)
        dout = a1 ? 16'hffff
                  : { svc_port, 8'hff };             // d5e000: bits 31-24
end

// d5a002 bits 15-8: coin 4-1 and service 4-1, the layout at konamigx.cpp:1274.
// Bits 10, 11, 14 and 15 are IPT_UNKNOWN and read high.
wire [7:0] coin_svc = { 2'b11, service[1:0], 2'b11, coin[1:0] };

// coin[3:2] are decoded by MAME's port map but Gokujou Parodius is a two-slot
// game, so this module reads only coin[1:0].  The port stays four bits wide so
// that a four-slot set does not need the interface changed.  An unread INPUT
// does not raise Quartus warning 10036 -- only an assigned-and-unread object
// does -- so there is nothing to tie off here.

endmodule

`default_nettype wire
