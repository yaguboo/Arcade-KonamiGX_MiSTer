//============================================================================
//  gx_xoffs -- the tilemap and sprite X offsets, from the CCU the game programs
//
//  These used to be per-set constants in gx_top, chosen by the set's name:
//      tile_dx_adj   0 for the GX sets, -15 dragoonj
//      spr_hoffset   954 GX / 952 salmndr2 / 931 dragoonj
//  -- MAME's per-machine k053252 set_offsets (crop), K055673 set_config dx and
//  set_layer_offs.  MEASUREMENTS 174 measured the CCU values 16 sets write
//  (gx_lane/gx_vidregs.lua) and found each of MAME's offsets is one formula
//  of the back porch and the hsync width:
//
//      tile_dx_adj = 48 - HBP
//      spr_hoffset = 1034 - (8*(HSW+1) + HBP) - (GX6 sprite ROM ? 2 : 0)
//
//  (1034 is 10 modulo the 10-bit sprite X.)  For the six supported sets the
//  values equal the old constants: GX HBP 48 / HSW 32 dots -> 0, 954;
//  salmndr2 the same CCU and GX6 -> 0, 952; dragoonj HBP 63 / HSW 40 -> -15,
//  931.  sim/tb_gx_xoffs.sv proves it through the real gx_ccu.
//
//  The -2 goes with the sprite ROM layout (obj_fmt 1, K055673_LAYOUT_GX6),
//  not with the game.  Type 3 / 4 boards (soccerss, rungun2) do not fit the
//  formula -- other video hardware, outside what this core supports.
//
//  EMULATION_DERIVED
//  Fitted to MAME's tuned per-set offsets (k053252.cpp TODO: "the offset x/y
//  hack").  The constants 48 and 1034 are pipeline offsets that make MAME's
//  numbers come out; the PCB's source of them is NOT verified.
//  TODO(HARDWAREIZE): where the PCB's object / layer origin comes from relative
//  to the CCU's back porch.
//
//  WHAT THE SILICON SAYS (2026-10-06, MEASUREMENTS 181).  The 053252
//  reconstruction (jtcores rungun/doc/053252.v; gx_ccu matches it to the dot,
//  sim/tb_gx_ccu_053252.v) starts each line at the H counter's load with
//  HSYNC, 8*(HSW+1) dots, then a back porch of HBP+1, then active.  So the
//  first active dot is 8*(HSW+1) + HBP + 1 dots after the H load and HBP + 1
//  after the end of sync.  The two formulas are exactly those two distances:
//      spr_hoffset = 11 - (8*(HSW+1) + HBP + 1)  -> the sprite chip counts
//                                                   from the H load (NHLD)
//      tile_dx_adj = 49 - (HBP + 1)              -> the tile chip counts
//                                                   from the end of HSYNC
//  so the register dependence is the silicon's.  The constants (11, 49, and
//  the GX6 -2) are inside the K055673 / K056832, whose netlists are not on
//  disk -- they stay EMULATION_DERIVED.
//
//  Registered: the CCU values are frame constants written at boot (the reset
//  values are the GX setting), and the sum must not sit combinationally in
//  front of the sprite and tilemap X arithmetic.
//============================================================================
`default_nettype none

module gx_xoffs (
    input  wire              clk,
    input  wire [8:0]        hbp,         // gx_ccu hbp_o
    input  wire [9:0]        hsw_dots,    // gx_ccu hsw_dots_o, 8*(HSW+1)
    input  wire [1:0]        obj_fmt,     // 1 = GX6 sprite ROM
    output reg  signed [9:0] tile_dx_adj,
    output reg  [9:0]        spr_hoffset
);

always @(posedge clk) begin
    tile_dx_adj <= 10'sd48 - $signed({1'b0, hbp});
    spr_hoffset <= 10'd10 - hsw_dots - {1'b0, hbp} - ((obj_fmt == 2'd1) ? 10'd2 : 10'd0);
end

endmodule

`default_nettype wire
