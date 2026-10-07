//============================================================================
//  Konami K056800 "MIRAC" -- main CPU <-> sound CPU mailbox
//
//  Sources, in the order root section 1.2 puts them:
//
//    MAME     src/devices/sound/k056800.cpp -- the whole chip is 150 lines and
//             every branch of it is transcribed below.  BSD-3-Clause, so
//             structure may be followed and is cited line by line.
//    MEASURED tools/gx_sndtap.lua and tools/gx_sndcpu.lua, 2026-09-08 -- the
//             actual boot handshake, byte lanes and offsets, taken off the
//             running game rather than read off the map.
//
//  docs/REUSE_PLAN.md assigns this to shelf 5 (new RTL): jtcores' jt054321 is
//  the K054321, a different chip, and is a shape reference only.
//
//  ---- why this chip is on the critical path --------------------------------
//  It is a mailbox, and mailboxes look like decoration until something waits
//  on one.  MEASURED 2026-09-08: the main CPU writes 0xFE to host register 0
//  at frame 163, pulses the sound-interrupt register, and then polls
//  snd_to_host[0] once a frame.  It gets 0xC0 -- "the sound CPU is still
//  running its own self-test" -- until frame 654, and only then does the game
//  leave its ROM/RAM CHECK screen.
//
//  With no sound CPU this core answered 0xFF forever and the main CPU sat in
//  the self-test for at least 145 seconds, executing and looping the whole
//  time.  That is the entire "layer A draws nothing" defect, two levels up.
//
//  ---- byte lanes, which are NOT the same on the two sides ------------------
//  konamigx.cpp:1056   host  d52000-d5201f  umask32(0xff00ff00)
//  konamigx.cpp:1191   sound 400000-40001f  umask16(0x00ff)
//
//  The host is a 32-bit bus and the chip sits on lanes 3 and 1, i.e. the HIGH
//  byte of each 16-bit half.  The sound bus is 16-bit and the chip sits on the
//  LOW byte.  So the two ports take their data from different halves of the
//  word, and that is why this module takes 8-bit data rather than 16 -- the
//  lane selection belongs to the bus, not to the chip.
//
//  Confirmed against the measured trace rather than derived:
//
//      W D52000 mask FF000000  ->  offset 0, 0xFE     host_to_snd[0]
//      W D52000 mask 0000FF00  ->  offset 1, 0x00     host_to_snd[1]
//      W D5200C mask 0000FF00  ->  offset 7           the interrupt pulse
//      R D52010 mask FF000000  ->  offset 8 & 7 = 0   snd_to_host[0]
//      R 400010 (sound side)   ->  offset 8 & 7 = 0   host_to_snd[0]
//
//  The window is 0x20 bytes = 16 offsets and the chip decodes `offset & 7`,
//  so the top half aliases onto the bottom.  That aliasing is what makes
//  0xd52010 read snd_to_host[0], and it is the chip's behaviour, not a
//  simplification: k056800.cpp does `r = offset & 7` in all four handlers.
//============================================================================
`default_nettype none

module gx_k056800 (
    input  wire        clk,
    input  wire        rst,

    // --- host side: main CPU, HIGH byte lane of each 16-bit half ------------
    input  wire        h_cs,
    input  wire        h_we,
    input  wire [4:1]  h_addr,
    input  wire [7:0]  h_din,
    output reg  [7:0]  h_dout,

    // --- sound side: sound CPU, LOW byte lane -------------------------------
    input  wire        s_cs,
    input  wire        s_we,
    input  wire [4:1]  s_addr,
    input  wire [7:0]  s_din,
    output reg  [7:0]  s_dout,

    // Level, not a pulse: konamigx.cpp:1787 wires int_callback to
    // M68K_IRQ_1 with set_inputline, and MAME asserts and clears it as a
    // level.  The sound CPU acknowledges by writing sound register 4.
    output wire        snd_irq,

    // --- observability ------------------------------------------------------
    //  snd_to_host[0] -- the one byte the whole boot waits on, the byte the
    //  main CPU polls at 0xd52010.  A LEVEL straight off the register, so the
    //  consumer decides what to make of it; gx_top does the framing, the same
    //  split gx_sound's `dbg` uses.
    output wire [7:0]  dbg_s2h0,

    //  host_to_snd[0] -- the COMMAND, the one input to the sound CPU that
    //  sim/tb_gx_sndboot.sv has to guess.  The board's sound CPU halts in a
    //  tight cache-resident loop and the simulator's does not, and the only
    //  difference between them is what arrives here.
    output wire [7:0]  dbg_h2s0,
    //  A write to register 7 is the SOUND INTERRUPT pulse, and k056800.cpp
    //  DROPS it when the sound CPU has interrupts disabled.  Whether it ever
    //  arrived is therefore a different fact from whether it was delivered.
    output wire        dbg_h2s_int
);

// k056800.h:38-41
reg [7:0] h2s [0:3];        // m_host_to_snd_regs
reg [7:0] s2h [0:1];        // m_snd_to_host_regs
reg       int_enabled;      // m_int_enabled
reg       int_pending;      // m_int_pending

wire [2:0] h_r = h_addr[3:1];   // `r = offset & 7`
wire [2:0] s_r = s_addr[3:1];

integer i;
always @(posedge clk) begin
    if (rst) begin
        for (i = 0; i < 4; i = i + 1) h2s[i] <= 8'd0;
        s2h[0] <= 8'd0;
        s2h[1] <= 8'd0;
        int_enabled <= 1'b0;
        int_pending <= 1'b0;
    end else begin
        // ---- host_w, k056800.cpp:84 -------------------------------------
        if (h_cs && h_we) begin
            case (h_r)
                3'd0, 3'd1, 3'd2, 3'd3: h2s[h_r[1:0]] <= h_din;
                // 4 front volume, 5 rear volume, 6 mute -- the chip accepts
                // them and MAME implements none.  Writing them must not
                // disturb anything, which is what "do nothing" means here.
                3'd7: begin
                    // "Sound interrupt".  Note the guard: MAME only sets
                    // pending when the SOUND CPU has enabled interrupts, so a
                    // pulse arriving while they are off is LOST, not queued.
                    if (int_enabled) int_pending <= 1'b1;
                end
                default: ;
            endcase
        end

        // ---- sound_w, k056800.cpp:139 -----------------------------------
        if (s_cs && s_we) begin
            case (s_r)
                3'd0, 3'd1: s2h[s_r[0]] <= s_din;
                // 2, 3 and 5 are "TODO: Unknown" upstream.  Accepted and
                // dropped, which is what the device does today; if the sound
                // program turns out to depend on one, that is a finding and
                // not a place to guess.
                3'd4: begin
                    int_enabled <= s_din[0];
                    // Disabling both acknowledges and clears.  Enabling does
                    // not clear, so a pulse that arrived while enabled stays
                    // pending -- which is how the line re-asserts.
                    if (!s_din[0]) int_pending <= 1'b0;
                end
                default: ;
            endcase
        end
    end
end

// ---- host_r, k056800.cpp:62 -----------------------------------------------
always @(*) begin
    case (h_r)
        3'd0:    h_dout = s2h[0];
        3'd1:    h_dout = s2h[1];
        // bit0 front volume busy, bit1 rear volume busy.  No volume hardware
        // is modelled, so neither is ever busy -- and "always idle" is the
        // answer that lets a poll on it terminate.
        3'd2:    h_dout = 8'h00;
        default: h_dout = 8'h00;
    endcase
end

// ---- sound_r, k056800.cpp:120 ---------------------------------------------
always @(*) begin
    case (s_r)
        3'd0, 3'd1, 3'd2, 3'd3: s_dout = h2s[s_r[1:0]];
        default:                s_dout = 8'h00;
    endcase
end

// MAME drives the line from two separate events; the state it reaches is
// always exactly this conjunction, so the line is expressed as the state
// rather than as a sequence of edge effects.
assign snd_irq = int_enabled & int_pending;

assign dbg_s2h0 = s2h[0];
assign dbg_h2s0 = h2s[0];
assign dbg_h2s_int = h_cs && h_we && (h_r == 3'd7);

endmodule

`default_nettype wire
