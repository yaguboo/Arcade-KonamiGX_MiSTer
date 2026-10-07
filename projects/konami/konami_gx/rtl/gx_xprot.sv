//============================================================================
//  gx_xprot -- the type 4 Xilinx protection on Winning Spike (special 7)
//
//  EMULATION_DERIVED
//  Matches MAME konamigx.cpp:878-930 (`type4_prot_w`), installed at
//  0xcc0000-0xcc0007 for winspike by init_konamigx special 7 (:4155) in place
//  of the ESC.  Required for bring-up.
//  The actual PCB device is a Xilinx part whose logic is NOT known.
//  TODO(HARDWAREIZE): what the FPGA on the type 4 / winspike board computes.
//
//  What MAME does, and so what this does: the long at cc0004 latches an
//  opcode (bits 31-16); the long at cc0000 carries a clock in bit 25 (bit 9
//  of its high word), and on that clock's FALLING edge the latched opcode
//  runs.  Winning Spike uses two opcodes (MAME's table, :899-904):
//
//      0x057a  copy 2 longs  c00f10 -> c10f00    player 1 input buffer
//              copy 2 longs  c00f20 -> c10f20    player 2 input buffer
//              copy 2 longs  c00f30 -> c0fe00
//      0x0d1c  copy 0x400 bytes c01000 -> c01400  "startup check for type 4
//              games" -- MISSED in 57678d63, whose board run looped in POST
//
//  gx_main recognises the edge, freezes the 68EC020 on that write and lends
//  this module the slice through the ESC / fjdma seam (D18, D19), so the copy
//  is done before the game's next instruction -- MAME's handler runs inside
//  the write.  A word read and a word write per word: 12 for 057a, 512 for
//  0d1c.  The other type 4
//  opcodes (rungun2, rushhero, vsnet) are not winspike's and are not here.
//============================================================================
`default_nettype none

module gx_xprot (
    input  wire         clk,
    input  wire         rst,

    input  wire         start,           // one clock: run
    input  wire         op_d1c,          // with start: 0x0d1c, else 0x057a
    output wire         busy,
    output reg          done,            // one clock: every word written

    output reg          bus_req,
    output reg          bus_we,
    output reg  [23:1]  bus_addr,
    output reg  [15:0]  bus_din,
    output wire [1:0]   bus_be,
    input  wire         bus_ack,
    input  wire [15:0]  bus_rdata
);

assign bus_be = 2'b11;

// the twelve word moves: source and destination word addresses
function automatic [23:1] src_of(input [3:0] i);
    case (i[3:2])
        2'd0:    src_of = 23'h600788 + {21'd0, i[1:0]};   // c00f10
        2'd1:    src_of = 23'h600790 + {21'd0, i[1:0]};   // c00f20
        default: src_of = 23'h600798 + {21'd0, i[1:0]};   // c00f30
    endcase
endfunction
function automatic [23:1] dst_of(input [3:0] i);
    case (i[3:2])
        2'd0:    dst_of = 23'h608780 + {21'd0, i[1:0]};   // c10f00
        2'd1:    dst_of = 23'h608790 + {21'd0, i[1:0]};   // c10f20
        default: dst_of = 23'h607F00 + {21'd0, i[1:0]};   // c0fe00
    endcase
endfunction

localparam [1:0] S_IDLE = 2'd0, S_RD = 2'd1, S_WR = 2'd2, S_FIN = 2'd3;
reg  [1:0]  st;
reg  [8:0]  n;         // word index: 0..11 (057a), 0..511 (0d1c)
reg         d1c;
assign busy = (st != S_IDLE);
wire [8:0]  last = d1c ? 9'd511 : 9'd11;
// 0d1c: c01000 -> c01400, word addresses 600800 / 600A00
wire [23:1] dst_n = d1c ? 23'h600A00 + {14'd0, n} : dst_of(n[3:0]);
wire [8:0]  n1    = n + 9'd1;
wire [23:1] src_1 = d1c ? 23'h600800 + {14'd0, n1} : src_of(n1[3:0]);

always @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
        st       <= S_IDLE;
        n        <= 9'd0;
        d1c      <= 1'b0;
        bus_req  <= 1'b0;
        bus_we   <= 1'b0;
        bus_addr <= 23'd0;
        bus_din  <= 16'd0;
    end else begin
        case (st)
        S_IDLE: if (start) begin
            n        <= 9'd0;
            d1c      <= op_d1c;
            bus_req  <= 1'b1;
            bus_we   <= 1'b0;
            bus_addr <= op_d1c ? 23'h600800 : src_of(4'd0);
            st       <= S_RD;
        end
        S_RD: if (bus_ack) begin
            bus_we   <= 1'b1;
            bus_addr <= dst_n;
            bus_din  <= bus_rdata;
            st       <= S_WR;
        end
        S_WR: if (bus_ack) begin
            if (n == last) begin
                bus_req <= 1'b0;
                bus_we  <= 1'b0;
                st      <= S_FIN;
            end else begin
                n        <= n1;
                bus_we   <= 1'b0;
                bus_addr <= src_1;
                st       <= S_RD;
            end
        end
        S_FIN: begin
            done <= 1'b1;
            st   <= S_IDLE;
        end
        endcase
    end
end

endmodule

`default_nettype wire
