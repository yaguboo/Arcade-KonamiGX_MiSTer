//============================================================================
//  Konami System GX -- INT1 (vblank) arm
//
//  DECISIONS D26, MEASUREMENTS 164.  INFERRED from the programs: one
//  mechanism where MAME has two.
//
//  MAME fires IRQ1 at vblank on (enable & 0x81) == 0x81 OR m_gx_syncen bit 0
//  (konamigx.cpp:514-516, :651-653).  tbyahhoo needs the second: at f833 it
//  writes 0x91 on line 2, acks the CCU on lines 2 and 26, and its ESC routine
//  rewrites the byte to 0x80/0x90 on line 27 -- 197 lines before vblank -- so
//  neither "ESC latency" nor "a pending latch" (gx_ccu's INT1 already is one)
//  puts an enabled INT1 at that vblank (tools/gx_irq1tap.lua).
//
//  The first path is never needed: over 4000 frames of all eight sets every
//  IRQ1 follows an enable write carrying 0x81 since the previous IRQ1, and
//  none fires on the level alone (tools/gx_irq1arm.lua).  So the enable bit
//  is an ARM: set by a write carrying bits 7 and 0, NOT cleared by a later
//  write without them, consumed by the vblank edge it fires on.  The request
//  then holds until the handler acks the CCU.
//
//  Not PCB-verified; it is the one model all eight programs agree with.
//  TODO(HARDWAREIZE): U9 -- D56001 bit 0 against /IPL on a PCB.
//============================================================================
module gx_int1arm (
    input  wire clk,
    input  wire rst,
    input  wire int1,       // gx_ccu: set at vblank start, cleared by the ack
    input  wire arm_set,    // one clock: an enable write carrying 0x81
    output reg  irq         // to the 68EC020's level 1
);

reg arm    = 1'b0;
reg int1_d = 1'b0;
wire rise  = int1 && !int1_d;

always @(posedge clk) begin
    if (rst) begin
        arm    <= 1'b0;
        irq    <= 1'b0;
        int1_d <= 1'b0;
    end else begin
        int1_d <= int1;
        if (!int1)     irq <= 1'b0;             // the CCU ack
        else if (rise) irq <= arm | arm_set;

        // an arm written on the edge's own clock fired above, so the edge
        // consumes it too
        if (rise)         arm <= 1'b0;
        else if (arm_set) arm <= 1'b1;
    end
end

endmodule
