//============================================================================
//  Konami K054539 -- the part of it the BOOT depends on
//
//  This is ONE chip.  The board has two, sharing one 16-bit window with chip 1
//  on the high byte and chip 2 on the low (gx.cpp:1188-1189), so the lane
//  selection belongs to the bus and this module takes 8-bit data -- the same
//  split gx_k056800 uses and for the same reason.
//
//  Sources, in the order root section 1.2 puts them:
//
//    MAME     src/devices/sound/k054539.cpp.  BSD-3-Clause, so structure may
//             be followed and is cited line by line below.
//    MEASURED docs/MEASUREMENTS.md sections 11 and 12, 2026-09-08 --
//             tools/gx_sndreply.lua, tools/gx_539io.lua, tools/gx_noirq2.lua.
//
//  docs/REUSE_PLAN.md shelf 2 is EMPTY for this chip: jtcores' jt539 is an
//  empty submodule stub and references/upstream/jt539 is a broken clone.  New
//  RTL, deliberately.
//
//  ---- WHAT THIS IS AND, JUST AS IMPORTANTLY, WHAT IT IS NOT ----------------
//
//  IT IS:   a register file that ECHOES what was written to it, and the
//           programmable timer whose output drives the sound CPU's IRQ2.
//
//  IT IS NOT: the PCM engine.  No channels, no ROM fetch, no interpolation,
//           no reverb buffer, no audio output.  That is REUSE_PLAN item 4 and
//           it is a much larger job.
//
//  The split is not arbitrary and it is not a guess.  MEASURED, twice, and
//  each one alone hangs the boot (docs/MEASUREMENTS.md section 12):
//
//    * the sound self-test walks 00/FF/AA/55 through 256 registers on EACH
//      chip and READS THEM BACK -- 2560 reads inside the window where the
//      main CPU is stuck.  k054539.cpp's read handler is `return
//      m_regs[offset]`, so the chip echoes.  This core answered ZERO.
//    * the sound program's IRQ2 handler advances the byte the main CPU polls,
//      and IRQ2 is the RISING EDGE of this chip's timer output, gated by
//      sound_ctrl bit 0 (konamigx.cpp:1202).  This core tied irq2 to 1'b0.
//
//  Forcing EITHER of those in MAME -- everything else left real -- reproduces
//  this board's symptom exactly.  So both are here, and nothing else is.
//
//  ---- THE TIMER RATE, DERIVED RATHER THAN COPIED ---------------------------
//  k054539.cpp:427, on a write to register 0x227:
//
//      period = attotime::from_hz((38 + data) * (clock()/384.0/14400.0)) / 2.0
//
//  and each expiry TOGGLES the output.  konamigx.cpp:1789 clocks the part at
//  XTAL(18'432'000).  Rearranged into this board's 96 MHz domain the divisor
//  collapses to something with no chip clock in it at all:
//
//      clocks per toggle = 96e6 * 384 * 7200 / (18.432e6 * (38 + data))
//                        = 14,400,000 / (38 + data)
//
//  MEASURED: gokuparo writes 0x6D at frame 163, so 14,400,000/147 = 97,959.18
//  clocks per toggle -- 980 Hz of toggles, 490 Hz of rising edges, 490 IRQ2/s.
//  NOT AN INTEGER, so a plain compare-and-reload would drift.  An accumulator
//  that adds (38 + data) every clock and subtracts 14,400,000 when it reaches
//  it has the exact average rate and no divider, which is the same trick
//  gx_tilemap's mod3 uses for the same reason (that one was worth 5.8 ns).
//
//  ACCURACY MARKING, root section 1.6.  The RATE is EMULATION_DERIVED: 384 and
//  14400 are MAME's factors and 14,400,000/(38+data) is not an integer number
//  of the real chip's own clocks either, so whatever the silicon divides, it
//  is not this.  It is PURE_RTL on the purity axis -- the arithmetic is in the
//  FPGA -- so it does not block release.
//  TODO(HARDWAREIZE): what does the real K054539 divide, and by what?
//
//  ---- WHAT IS DELIBERATELY ABSENT, WITH ITS CONDITION ----------------------
//
//  * The KEYON POSITION LATCH.  k054539.cpp:370 does not store writes to a
//    voice's position registers (offset < 0x100, (offset & 0x1f) - 0xc in
//    0..2) when `m_flags & UPDATE_AT_KEYON` and `m_regs[0x22f] & 1`; it puts
//    them in m_posreg_latch instead.  device_start sets UPDATE_AT_KEYON
//    unconditionally (k054539.cpp:319), so the condition reduces to bit 0 of
//    register 0x22f.  Those latched values only ever reach a CHANNEL, and
//    there are no channels here, so this module echoes every write.  That is
//    identical to MAME whenever 0x22f bit 0 is clear.
//    TODO(SOUND): implement with the channels, in REUSE_PLAN item 4.
//
//  * Register 0x22d, the ROM/RAM read port.  MAME returns data from the PCM
//    ROM or the sample RAM, and returns ZERO when `m_regs[0x22f] & 0x10` is
//    clear.  There is no PCM path here, so it reads zero unconditionally --
//    "absent", which is honest, rather than an echo, which would be a lie the
//    RAM test could believe.
//    TODO(SOUND): with item 4.
//============================================================================
`default_nettype none

module gx_k054539 (
    input  wire        clk,
    input  wire        rst,

    // --- register port, 8-bit: ONE lane of the shared 16-bit window --------
    input  wire        cs,
    input  wire        we,
    input  wire [9:0]  addr,        // register number, gx.cpp maps reg*2
    input  wire [7:0]  din,
    output wire [7:0]  dout,

    // --- the timer output.  A LEVEL that TOGGLES, exactly as MAME's
    //     m_timer_state does, because konamigx.cpp:1202 wants its RISING
    //     EDGE and an edge is not something this module should presume to
    //     have already taken.
    output reg         timer_out
);

// ---------------------------------------------------------------------------
//  the register file
// ---------------------------------------------------------------------------
//  m_regs is 0x230 bytes (k054539.h:91) but the WINDOW is 0x500 bytes = 0x280
//  registers, so MAME would index past its own array for the top of it.  1024
//  entries covers the whole window with no undefined region, and costs the
//  same: this design's measured rate is 1 M10K per 1024 bytes (D10), so one
//  block per chip either way.
//
//  Written exactly like gx_sound's work RAM -- read address is the SAME
//  EXPRESSION as the write address -- because that is the shape Quartus 17.0
//  infers as block RAM.  Root section 7 and power_spikes L24: read the
//  fitter's RAM summary before the ALM count.
(* ramstyle = "M10K" *) reg [7:0] regs [0:1023];
reg  [7:0] regs_q;
reg  [9:0] addr_q;

always @(posedge clk) begin
    if (cs && we) regs[addr] <= din;
    regs_q <= regs[addr];
    addr_q <= addr;
end

//  k054539.cpp:read -- everything is `return m_regs[offset]` except 0x22d.
//  0x22c is a `break` in that switch, which FALLS THROUGH to the same return,
//  so it echoes; it is named here rather than left to the default because the
//  two look alike in the source and only one of them is special.
assign dout = (addr_q == 10'h22d) ? 8'h00 : regs_q;

// ---------------------------------------------------------------------------
//  the timer
// ---------------------------------------------------------------------------
//  ACC_LIMIT is 14,400,000 -- see the header derivation.  The accumulator adds
//  (38 + data) per clock and toggles when it crosses, so the average period is
//  exact even though the per-toggle count is not an integer.
localparam [23:0] ACC_LIMIT = 24'd14_400_000;

reg [23:0] acc;
reg [8:0]  rate;      // 38 + data, so 38..293: nine bits
reg        run;

always @(posedge clk) begin
    if (rst) begin
        acc       <= 24'd0;
        rate      <= 9'd0;
        run       <= 1'b0;
        timer_out <= 1'b0;
    end else begin
        // ---- register 0x227: set the period, k054539.cpp:427 -------------
        //  MAME restarts the timer AND clears the output on this write:
        //      m_timer->adjust(period, 0, period);
        //      m_timer_state = 0;  m_timer_handler(m_timer_state);
        //  so the phase is reset here too, not just the rate.
        if (cs && we && addr == 10'h227) begin
            rate      <= 9'd38 + {1'b0, din};
            acc       <= 24'd0;
            run       <= 1'b1;
            timer_out <= 1'b0;
        end
        // ---- register 0x22f bit 5: "Disable timer output?" ----------------
        //  k054539.cpp:445.  MAME clears the state and calls the handler; it
        //  does NOT stop the timer, so `run` is left alone and only the level
        //  is forced low.  Transcribed rather than tidied: a chip that keeps
        //  counting while its output is held is a different chip from one
        //  that stops, and which it is decides what happens on re-enable.
        else if (cs && we && addr == 10'h22f && !din[5]) begin
            timer_out <= 1'b0;
        end
        else if (run) begin
            if (acc >= ACC_LIMIT) begin
                acc       <= acc - ACC_LIMIT + {15'd0, rate};
                timer_out <= ~timer_out;
            end else begin
                acc <= acc + {15'd0, rate};
            end
        end
    end
end

endmodule

`default_nettype wire
