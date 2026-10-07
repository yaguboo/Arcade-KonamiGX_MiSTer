//============================================================================
//  gx_eestart -- a 93C46's LEVEL start bit in front of jt9346's TRANSITION one
//
//  A 93C46 takes the start bit as the first DI = 1 at an SK rising edge with CS
//  high.  jt9346 takes a 0 -> 1 transition of DI between two SK edges and keeps
//  its last sample across CS low.  So on every CS rise while SK is low this
//  hands jt9346 one extra SK pulse with DI = 0; in IDLE it ignores a 0, and its
//  last sample is then 0, so the real first 1 is a transition.  gx_top's
//  EEPROM section has the history (Dragoon Might, 22D/M BAD).  PURE_RTL.
//============================================================================
`default_nettype none
module gx_eestart (
    input  wire clk,
    input  wire rst,
    input  wire cs,
    input  wire sk,
    input  wire di,
    output wire sk_o,
    output wire di_o,
    input  wire enable          // 0: pass-through (the control)
);
reg  [2:0] pre = 3'd0;          // 0 idle; 1..4 the injected pulse
reg        cs_d = 1'b0;
always @(posedge clk) begin
    cs_d <= cs;
    if (rst)                                  pre <= 3'd0;
    else if (enable && cs && !cs_d && !sk)    pre <= 3'd1;
    else if (pre != 3'd0 && pre != 3'd4)      pre <= pre + 3'd1;
    else                                      pre <= 3'd0;
end
assign sk_o = (pre == 3'd2 || pre == 3'd3) ? 1'b1 : sk;
assign di_o = (pre != 3'd0) ? 1'b0 : di;
endmodule
`default_nettype wire
