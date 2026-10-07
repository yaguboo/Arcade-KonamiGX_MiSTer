//============================================================================
//  gx_cpucache -- a direct-mapped cache of the main CPU's ROM words
//
//  WHY.  MEASURED on the board (docs/MEASUREMENTS.md 41): with the look-ahead
//  the battleship demo runs at 48.5 % of MAME's pace, and a frame-quantised
//  model of MAME's own ROM reads says that stage needs ~29 k words a frame
//  against ~23.5 k delivered.  The CPU is stalled ~80 % of a frame and most of
//  that waiting for tile groups (section 38), so the words have to come
//  WITHOUT the bus.  tb_gx_busmix with the program's own read order
//  (+cputrace) and this cache at 2^10: +21 % CPU words at the battleship,
//  +15 % in the city, think 8; no tile group late; every consumed word checked
//  against the ROM image.
//
//  SHAPE, and what each choice avoids:
//    * Registered lookup.  A request the look-ahead does not answer waits one
//      clock for the compare; a hit is then served, a miss is let through to
//      the arbiter one clock late.  A combinational hit would put the RAM read,
//      the tag compare and gx_main's handshake in one 96 MHz clock.
//    * Filled from the glue's REGISTERS, not from the SDRAM's data.  The fill
//      writes `fill_d1` (gx_top's cpu_rd_q) the clock after the ack and
//      `fill_d2` (cpu_la_data) the clock after that.  The shared SDRAM return
//      is the path class that has failed timing four builds running
//      (MEASUREMENTS 38, 40); this module adds no endpoint to it.
//    * A lookup that lands on the entry being written, or about to be, is a
//      MISS.  `no_rw_check` then tells the truth: the RAM never has to resolve
//      a read of the address it is writing.
//    * Only the BIOS and main program are cached -- SDRAM words below 2^20
//      (GX_MAIN_BASE + GX_MAIN_SIZE is byte 0x120000, word 0x90000).
//
//  COHERENCY.  The CPU cannot write its ROM and nothing else changes it but
//  the ROM download, during which `rst` is held; `v` clears in that reset.
//
//  PLATFORM-NEUTRAL, PURE_RTL.  Size 2^IDX_BITS words of 16 data + (20 -
//  IDX_BITS) tag bits in block RAM; `v` is flip-flops.
//============================================================================
`default_nettype none

module gx_cpucache #(
    parameter int IDX_BITS = 10
) (
    input  wire         clk,
    input  wire         rst,

    // --- the request, as gx_top's glue sees it ------------------------------
    input  wire         look,        // a ROM read the look-ahead does not answer, not held/used
    input  wire [24:0]  addr,        // SDRAM word address
    output reg          serve,       // one clock: `q` is the word at `addr`
    output reg  [15:0]  q,
    output reg          miss,        // level: this request goes to the arbiter

    // --- the CPU's own SDRAM transaction completing --------------------------
    input  wire         ack,         // arb_ack for the CPU client
    input  wire         fill_two,    // it was a burst (both words are good)
    input  wire [15:0]  fill_d1,     // the glue's registered first word, valid the clock after ack
    input  wire [15:0]  fill_d2      // its registered second word, same clock
);

localparam int TAG_BITS = 20 - IDX_BITS;
localparam int N        = 1 << IDX_BITS;

(* ramstyle = "M10K, no_rw_check" *) reg [TAG_BITS+15:0] mem [0:N-1];
reg [N-1:0] v;

// ---- the fill: two writes, one and two clocks after the ack --------------------
reg [24:0]         f_addr;
reg                f_pend1, f_pend2;
reg                we;
reg [IDX_BITS-1:0] wa;
reg [TAG_BITS+15:0] wd;

wire [IDX_BITS-1:0] l_idx  = addr[IDX_BITS-1:0];
wire                l_ok   = (addr[24:20] == 5'd0);
wire [24:0]         f_addr2 = f_addr + 25'd1;

// an entry being written this clock or queued for the next two
wire l_busy = (we && wa == l_idx) ||
              (f_pend1 && f_addr[IDX_BITS-1:0]  == l_idx) ||
              (f_pend2 && f_addr2[IDX_BITS-1:0] == l_idx) ||
              (ack);

reg                 looked;
reg                 lk_hit_ok;
reg [24:0]          lk_addr;
reg [TAG_BITS+15:0] lk_q;
reg                 lk_v;

always @(posedge clk) begin
    if (we) mem[wa] <= wd;
    lk_q <= mem[l_idx];
end

always @(posedge clk) begin
    serve <= 1'b0;
    we    <= 1'b0;
    if (rst) begin
        v       <= {N{1'b0}};
        f_pend1 <= 1'b0;
        f_pend2 <= 1'b0;
        looked  <= 1'b0;
        miss    <= 1'b0;
    end else begin
        // ---- fill ------------------------------------------------------------
        if (ack) begin
            miss    <= 1'b0;
            f_addr  <= addr;
            f_pend1 <= (addr[24:20] == 5'd0);
            f_pend2 <= fill_two;
        end else if (f_pend1) begin
            f_pend1       <= 1'b0;
            we            <= 1'b1;
            wa            <= f_addr[IDX_BITS-1:0];
            wd            <= {f_addr[19:IDX_BITS], fill_d1};
            v[f_addr[IDX_BITS-1:0]] <= 1'b1;
        end else if (f_pend2) begin
            f_pend2       <= 1'b0;
            if (f_addr2[24:20] == 5'd0) begin
                we        <= 1'b1;
                wa        <= f_addr2[IDX_BITS-1:0];
                wd        <= {f_addr2[19:IDX_BITS], fill_d2};
                v[f_addr2[IDX_BITS-1:0]] <= 1'b1;
            end
        end

        // ---- lookup ------------------------------------------------------------
        if (!look) begin
            looked <= 1'b0;
        end else if (!looked && !miss) begin
            looked    <= 1'b1;
            lk_addr   <= addr;
            lk_v      <= v[l_idx];
            lk_hit_ok <= l_ok && !l_busy;
        end else if (looked) begin
            looked <= 1'b0;
            if (lk_hit_ok && lk_v && lk_addr == addr &&
                lk_q[TAG_BITS+15:16] == lk_addr[19:IDX_BITS]) begin
                q     <= lk_q[15:0];
                serve <= 1'b1;
            end else
                miss  <= 1'b1;
        end
    end
end

endmodule

`default_nettype wire
