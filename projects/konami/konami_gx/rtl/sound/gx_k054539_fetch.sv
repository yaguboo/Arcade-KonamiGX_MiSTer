//============================================================================
//  gx_k054539_fetch -- sample bytes for both K054539 engines, out of SDRAM
//
//  The 4 MB sample region lives in SDRAM with the rest of the ROM
//  (docs/REUSE_PLAN.md section 4; the board has 8 M10K left).  An engine asks
//  for one byte at a time and cannot wait long: a sample is due every 2,000
//  clocks and one engine may need sixteen bytes for it, while one transaction
//  through the arbiter costs tens of clocks with the bus free and more when the
//  tile fetcher holds it (tm_group_busy).
//
//  MEASURED, docs/MEASUREMENTS.md 39: the program asks for about 800 sample
//  bytes a frame, and they are sequential -- a channel reads forward (or
//  backward, 0x200 + 2ch bit 5) from its key-on position, one or two bytes a
//  step.  So each of the sixteen channels keeps TWO 4-byte blocks, one 2-word
//  burst each: the block it is reading and the block after it in playing order.
//  A read from either is served from registers in four clocks.  A read from a
//  block whose successor is not held marks the channel, and the successor is
//  fetched in the background while the engine carries on.  Only a jump waits
//  for a transaction: key-on, a loop back, the first read of a reverse sample.
//
//  Layout facts this depends on, each from its owner:
//    * the image is big-endian, word = {byte 2W, byte 2W+1}   gx_download.sv
//    * a burst starts only at an EVEN word address             gx_sdram BURST CONTRACT
//      -- a block is byte address & ~3, so its word address is always even
//    * the arbiter's data is captured into `cap` and nothing else, the shape
//      gx_romcache's fill_q has (8a8cb21): one register straight off m_q.
//
//  PURE_RTL, PLATFORM_TRANSPORT only: this is where the bytes come from, not
//  what the chip does with them.
//============================================================================
`default_nettype none

module gx_k054539_fetch #(
    parameter [24:0] BASE_W = 25'h040_0000        // GX_PCM_BASE / 2, the region's first WORD
) (
    input  wire         clk,
    input  wire         rst,

    // --- the two engines, index 0 = chip 1 -------------------------------------
    input  wire [1:0]   e_req,        // held until e_ack
    input  wire [43:0]  e_addr,       // {chip 2, chip 1}, 22 bits each
    input  wire [5:0]   e_ch,         // {chip 2, chip 1}
    input  wire [1:0]   e_rev,        // that channel plays backwards
    output reg  [1:0]   e_ack,        // one clock
    output reg  [7:0]   e_data,       // with e_ack

    // --- one arbiter client, 2-word bursts ------------------------------------
    output reg  [24:0]  m_addr,
    output reg          m_req,        // held until m_ack
    input  wire         m_ack,
    input  wire [15:0]  m_q,
    input  wire [15:0]  m_q2,

    output reg  [15:0]  dbg_demand,   // transactions an engine waited for (wrapping)
    output reg  [15:0]  dbg_ahead     // transactions fetched ahead (wrapping)
);

// ---- the blocks: index {chip, channel, slot} ---------------------------------
(* ramstyle = "logic" *) reg [19:0] sb [0:31];   // block number, byte address >> 2
(* ramstyle = "logic" *) reg [31:0] sd [0:31];   // {byte 0, 1, 2, 3}
reg [31:0] sv;                                   // block holds data
reg [15:0] last;                                 // slot of the channel's last read
reg [15:0] rv;                                   // the channel's last read was reverse
reg [15:0] want;                                 // the block after `last` is not held

localparam [2:0] S_IDLE = 3'd0, S_PICK = 3'd1, S_LOOK = 3'd2,
                 S_REQ  = 3'd3, S_WAIT = 3'd4, S_STORE = 3'd5, S_AHEAD = 3'd6;
//  S_IDLE picks the channel to read ahead for and registers its block;
//  S_AHEAD adds or subtracts one.  Together they were last[scan] -> the 32:1
//  select of sb -> a 20-bit add/sub -> f_b, at -0.111 ns in build c4c694b.
reg [19:0] a_blk;
reg        a_rev;
reg [2:0]  st;

reg        r_e, r_rev;
reg [3:0]  r_w;
reg [21:0] r_a;
reg        p_v0, p_v1;
reg [19:0] p_b0, p_b1;
reg [31:0] p_d0, p_d1;

reg        f_demand, f_s;
reg [3:0]  f_w;
reg [19:0] f_b;
reg [31:0] cap;
reg [3:0]  scan;

wire [19:0] r_blk  = r_a[21:2];
wire [19:0] r_next = r_rev ? r_blk - 20'd1 : r_blk + 20'd1;
wire        hit0   = p_v0 && (p_b0 == r_blk);
wire        hit1   = p_v1 && (p_b1 == r_blk);
wire        have   = hit0 ? (p_v1 && (p_b1 == r_next)) : (p_v0 && (p_b0 == r_next));
wire [31:0] hd     = hit0 ? p_d0 : p_d1;
wire [4:0]  s_last = {scan, last[scan]};

always @(posedge clk) begin
    e_ack <= 2'b00;
    if (rst) begin
        st         <= S_IDLE;
        m_req      <= 1'b0;
        sv         <= 32'd0;
        last       <= 16'd0;
        rv         <= 16'd0;
        want       <= 16'd0;
        scan       <= 4'd0;
        dbg_demand <= 16'd0;
        dbg_ahead  <= 16'd0;
    end else case (st)
        S_IDLE:
            if (e_req[0] && !e_ack[0]) begin
                r_e <= 1'b0; r_w <= {1'b0, e_ch[2:0]}; r_a <= e_addr[21:0];  r_rev <= e_rev[0];
                st  <= S_PICK;
            end else if (e_req[1] && !e_ack[1]) begin
                r_e <= 1'b1; r_w <= {1'b1, e_ch[5:3]}; r_a <= e_addr[43:22]; r_rev <= e_rev[1];
                st  <= S_PICK;
            end else if (want[scan]) begin                       // read ahead
                want[scan] <= 1'b0;
                f_demand   <= 1'b0;
                f_w        <= scan;
                f_s        <= ~last[scan];
                a_blk      <= sb[s_last];
                a_rev      <= rv[scan];
                st         <= S_AHEAD;
            end else
                scan <= scan + 4'd1;

        S_PICK: begin
            p_v0 <= sv[{r_w, 1'b0}];  p_v1 <= sv[{r_w, 1'b1}];
            p_b0 <= sb[{r_w, 1'b0}];  p_b1 <= sb[{r_w, 1'b1}];
            p_d0 <= sd[{r_w, 1'b0}];  p_d1 <= sd[{r_w, 1'b1}];
            st   <= S_LOOK;
        end

        S_LOOK:
            if (hit0 || hit1) begin
                case (r_a[1:0])
                    2'd0: e_data <= hd[31:24];
                    2'd1: e_data <= hd[23:16];
                    2'd2: e_data <= hd[15:8];
                    default: e_data <= hd[7:0];
                endcase
                e_ack[r_e] <= 1'b1;
                last[r_w]  <= !hit0;
                rv[r_w]    <= r_rev;
                want[r_w]  <= !have;
                st         <= S_IDLE;
            end else begin                                       // a jump: this engine waits
                f_demand <= 1'b1;
                f_w      <= r_w;
                f_s      <= ~last[r_w];
                f_b      <= r_blk;
                st       <= S_REQ;
            end

        S_AHEAD: begin
            f_b <= a_rev ? a_blk - 20'd1 : a_blk + 20'd1;
            st  <= S_REQ;
        end

        S_REQ: begin
            m_addr <= BASE_W + {4'd0, f_b, 1'b0};
            m_req  <= 1'b1;
            st     <= S_WAIT;
        end

        S_WAIT: if (m_ack) begin
            m_req <= 1'b0;
            cap   <= {m_q, m_q2};
            st    <= S_STORE;
        end

        S_STORE: begin
            sd[{f_w, f_s}] <= cap;
            sb[{f_w, f_s}] <= f_b;
            sv[{f_w, f_s}] <= 1'b1;
            if (f_demand) begin
                last[f_w]  <= f_s;
                dbg_demand <= dbg_demand + 16'd1;
                st         <= S_PICK;
            end else begin
                dbg_ahead  <= dbg_ahead + 16'd1;
                st         <= S_IDLE;
            end
        end

        default: st <= S_IDLE;
    endcase
end

endmodule

`default_nettype wire
