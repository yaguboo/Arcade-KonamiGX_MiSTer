//============================================================================
//  esc_host -- the ESC core's byte-addressed host port onto the 16-bit bus
//
//  esc_cpu asks for 1, 2 or 4 bytes, big-endian, at ANY byte address (packets
//  sit unaligned in work RAM; escemu reads byte by byte).  The bus gx_main
//  lends is the 68EC020's slice: word address, byte enables, one word per
//  transaction, req / we / addr / din / be held to a one-clock ack, rdata with
//  the ack (gx_esc's contract, unchanged).  One host access becomes one to
//  three word transactions, lowest address first; the core sees one ack.
//============================================================================
`default_nettype none

module esc_host (
    input  wire         clk,
    input  wire         rst,

    input  wire         c_req,
    input  wire         c_we,
    input  wire [23:0]  c_addr,
    input  wire [2:0]   c_size,
    input  wire [31:0]  c_wdata,                // right-aligned
    output reg          c_ack,
    output reg  [31:0]  c_rdata,                // right-aligned

    output reg          bus_req,
    output reg          bus_we,
    output reg  [23:1]  bus_addr,
    output reg  [15:0]  bus_din,
    output reg  [1:0]   bus_be,                 // {even byte, odd byte}
    input  wire         bus_ack,
    input  wire [15:0]  bus_rdata
);

reg         busy;
reg  [23:0] a;                                  // next byte address
reg  [2:0]  left;                               // bytes still to move
reg  [31:0] wd;                                 // write bytes, next one in [31:24]
reg  [31:0] acc;                                // read bytes so far, right-aligned
reg         we;
reg         two;                                // this transaction moves two bytes

always @(posedge clk) begin
    c_ack <= 1'b0;
    if (rst) begin
        busy <= 1'b0; bus_req <= 1'b0;
    end else if (!busy) begin
        if (c_req && !c_ack) begin
            busy <= 1'b1;
            a    <= c_addr;
            left <= c_size;
            we   <= c_we;
            acc  <= 32'd0;
            wd   <= c_wdata << (8 * (3'd4 - c_size));
        end
    end else if (!bus_req) begin
        if (left == 3'd0) begin
            busy <= 1'b0; c_ack <= 1'b1; c_rdata <= acc;
        end else begin
            // an even address with two or more bytes left moves a whole word
            two      <= !a[0] && (left >= 3'd2);
            bus_req  <= 1'b1;
            bus_we   <= we;
            bus_addr <= a[23:1];
            if (a[0]) begin                         // the odd byte only
                bus_be  <= 2'b01;
                bus_din <= {8'd0, wd[31:24]};
            end else if (left >= 3'd2) begin
                bus_be  <= 2'b11;
                bus_din <= wd[31:16];
            end else begin                          // the even byte only
                bus_be  <= 2'b10;
                bus_din <= {wd[31:24], 8'd0};
            end
        end
    end else if (bus_ack) begin
        bus_req <= 1'b0;
        if (two) begin
            acc  <= {acc[15:0], bus_rdata};
            wd   <= {wd[15:0], 16'd0};
            a    <= a + 24'd2;
            left <= left - 3'd2;
        end else begin
            acc  <= {acc[23:0], a[0] ? bus_rdata[7:0] : bus_rdata[15:8]};
            wd   <= {wd[23:0], 8'd0};
            a    <= a + 24'd1;
            left <= left - 3'd1;
        end
    end
end

endmodule

`default_nettype wire
