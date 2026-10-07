//============================================================================
//  gx_esc056 -- the 056734 (ESC) as a chip: esc_cpu running the game's own
//  boot object, kernel and program, on gx_main's borrowed bus (DECISIONS D31)
//
//  Replaces rtl/gx_esc.sv (the per-family translation) when gx_top's ESC_CORE
//  is 1.  Same seam toward gx_top / gx_main, one difference in how it is used:
//
//    gx_esc    the run owns the 68EC020's slice from the cc0000 write to done
//              -- the CPU is frozen for the whole run (D18)
//    here      the chip computes in its own local memory while the CPU runs;
//              `bus_req` asks for the slice per WORD and gx_main lends it at
//              the next CPU cycle boundary and takes it back after the ack.
//              The game's ESC call returns without waiting (tbyahhoo 296F7A,
//              dragoonj 250444: move.l to cc0000, rts), so freezing the CPU
//              for a run of 1-2 ms would take frame time the PCB's CPU has.
//
//  The chip's secret is per title: the fetch XOR, the lane permutation and
//  s10/s11 (tools/esc/README.md; the clones share their parent's chip).
//  A set without a known secret (gokuparo, fantjour: neither writes cc0000)
//  holds the core in reset.
//
//  `busy` (gx_sprite dma_hold, D18) is from the mailbox delivery of a non-zero
//  long until the kernel says it is waiting again; IRQ4 is the kernel's own
//  s6 pulse, gated by WRPOR1 bit 4 as gx_esc gated its end of run.
//============================================================================
`default_nettype none

module gx_esc056 (
    input  wire         clk,
    input  wire         rst,
    input  wire [3:0]   mode,            // gx_top esc_mode: 1 sexyparo 2 tbyahhoo 3 daiskiss 4 salmndr2 5 dragoonj
                                         //                  6 tkmmpzdm 7 crzcross/puzldama 8 tokkae

    input  wire         start,           // one clock: the game's long to cc0000 may be delivered
    input  wire [31:0]  cmd,
    input  wire         irq_en,          // wrport1_1 bit 4

    output reg          busy,
    output wire         boot_hold,       // the 68EC020 stays off the bus until the chip's kernel is ready (D31)
    output reg          irq4_set,
    output reg          b3_clr,

    output wire         bus_req,
    output wire         bus_we,
    output wire [23:1]  bus_addr,
    output wire [15:0]  bus_din,
    output wire [1:0]   bus_be,
    input  wire         bus_ack,
    input  wire [15:0]  bus_rdata,

    output wire         dbg_booted,
    output wire         dbg_fault
);

// ---- the chips' secrets ------------------------------------------------------
reg  [31:0] K, s10, s11;
reg  [63:0] perm;
always @(*) begin
    case (mode)
        4'd1: begin K = 32'h1886AE1D; perm = 64'hFEDCBA9876543210; s10 = 32'h0000896A; s11 = 32'h0E200100; end   // sexyparo(a)
        4'd2: begin K = 32'hDE78B8AE; perm = 64'h97E60A851BD32C4F; s10 = 32'h00000424; s11 = 32'h0A200100; end   // tbyahhoo / mtwinbee
        4'd3: begin K = 32'h39556CC0; perm = 64'hB854D2E7F3A601C9; s10 = 32'h000089EE; s11 = 32'h0D200100; end   // daiskiss
        4'd4: begin K = 32'hB3F135B3; perm = 64'hBD53C0196FEA4287; s10 = 32'h00001EC6; s11 = 32'h0A200100; end   // salmndr2(a)
        4'd5: begin K = 32'h049DB8E5; perm = 64'hA2CB1F05D68E3974; s10 = 32'h00005963; s11 = 32'h0B200100; end   // dragoonj(a)
        4'd6: begin K = 32'h88600EF4; perm = 64'h4CA873FB602D15E9; s10 = 32'h00004924; s11 = 32'h0C200100; end   // tkmmpzdm (2026-10-03, tools/esc/title_tkmmpzdm.log)
        4'd7: begin K = 32'h91C2C6FB; perm = 64'hF10ED73A65249C8B; s10 = 32'h00005D32; s11 = 32'h03200100; end   // crzcross / puzldama (title_crzcross.log)
        4'd8: begin K = 32'h6BFBB5B9; perm = 64'h17FE43B8A50D962C; s10 = 32'h00009E8E; s11 = 32'h0B200100; end   // tokkae (title_tokkae.log)
        default: begin K = 32'd0; perm = 64'hFEDCBA9876543210; s10 = 32'd0; s11 = 32'h00200100; end
    endcase
end
wire chip_en = (mode != 4'd0) && (mode <= 4'd8);

// ---- the core --------------------------------------------------------------
wire        h_req, h_we, h_ack;
wire [23:0] h_addr;
wire [2:0]  h_size;
wire [31:0] h_wdata, h_rdata;
wire        irq_pulse, ready_pulse;

esc_cpu #(.LAW(13)) u_cpu (
    .clk        (clk),
    .rst        (rst),
    .en         (chip_en),
    .hold       (1'b0),
    .K          (K),
    .perm       (perm),
    .s10_init   (s10),
    .s11_init   (s11),
    .mbox_we    (start),
    .mbox_d     (cmd),
    .host_req   (h_req),
    .host_we    (h_we),
    .host_addr  (h_addr),
    .host_size  (h_size),
    .host_wdata (h_wdata),
    .host_ack   (h_ack),
    .host_rdata (h_rdata),
    .irq_pulse  (irq_pulse),
    .ready_pulse(ready_pulse),
    .pc_o       (),
    .s7_o       (),
    .insn_done  (),
    .booted     (dbg_booted),
    .fault      (dbg_fault)
);

esc_host u_host (
    .clk       (clk),
    .rst       (rst),
    .c_req     (h_req),
    .c_we      (h_we),
    .c_addr    (h_addr),
    .c_size    (h_size),
    .c_wdata   (h_wdata),
    .c_ack     (h_ack),
    .c_rdata   (h_rdata),
    .bus_req   (bus_req),
    .bus_we    (bus_we),
    .bus_addr  (bus_addr),
    .bus_din   (bus_din),
    .bus_be    (bus_be),
    .bus_ack   (bus_ack),
    .bus_rdata (bus_rdata)
);

//  boot_hold (INFERRED, D31): the boot code kicks the watchdog itself -- byte
//  writes of 00 / 80 to D56000 (wrport1_0 bit 7, afr) every ~430,000 steps,
//  six times in 1.83 M -- which only makes sense if the 68020 is not running
//  to kick it.  So the CPU is held until the kernel first says `ready`.
//  Measured without it (905d2ffd): sexyparo's POST stopped on the TMS57002
//  lines in 2 of 7 boots, 0 of 6 on the build before the core.
reg kernel_ready = 1'b0;
assign boot_hold = chip_en && !kernel_ready;

always @(posedge clk) begin
    irq4_set <= 1'b0;
    b3_clr   <= 1'b0;
    if (rst) begin
        busy <= 1'b0; kernel_ready <= 1'b0;
    end else begin
        if (ready_pulse) kernel_ready <= 1'b1;
        if (start && cmd != 32'd0 && chip_en) busy <= 1'b1;
        else if (ready_pulse)                 busy <= 1'b0;
        if (irq_pulse && irq_en) begin irq4_set <= 1'b1; b3_clr <= 1'b1; end
    end
end

endmodule

`default_nettype wire
