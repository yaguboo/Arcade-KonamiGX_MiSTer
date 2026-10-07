//============================================================================
//  Konami System GX -- the Fantastic Journey DMA device at 0xdb0000
//
//  EMULATION_DERIVED
//  Matches MAME konamigx_m.cpp:489-538 (`fantjour_dma_install`,
//  `fantjour_dma_w`), installed for fantjour and fantjoura by `special = 9`
//  (konamigx.cpp:4054-4055, :4162) -- required for those two sets to draw
//  anything at all.  The actual PCB part that performs this is NOT verified;
//  MAME's own name for it is a "dma" and nothing upstream says which chip it
//  is.
//  TODO(HARDWAREIZE): UPSTREAM_TODO U34.
//
//  ---- why it is not optional (docs/MEASUREMENTS.md 57, 58) --------------------
//  MEASURED in MAME, 14,000 frames: fantjour and fantjoura write this register
//  file 200,196 times and trigger 16,683 runs, of which 15,397 change memory --
//  13,246 fills of object RAM 0xd21000-0xd213ff and 3,437 keyed copies into the
//  live palette 0xd90000-0xd94fff.  gokuparo writes the range zero times.  With
//  the writes dropped, which is what this core did until now, MAME's own
//  picture goes black at the same frames the board did (tools/gx_fjdrop.lua).
//
//  ---- what it does, in MAME's order ------------------------------------------
//  Eight longs at 0xdb0000-0xdb001f, written with byte enables (COMBINE_DATA).
//  A write that touches the TOP BYTE of long 0 (`!offset && ACCESSING_BITS_24_31`)
//  runs, in the 68EC020's address space:
//
//      mode = long0[31:24]
//      sz2  = long0[23:16]        sz1 (long0[15:8]) is unused, as in MAME
//      sa   = long1               da = {long3[15:0], long4[31:16]}
//      db   = long5               x  = long6
//
//      mode 0x93   for i1 in 0..sz2: for (i2 = 0; i2 < db; i2 += 4)
//                      *da = *sa ^ x;  da += 4;  sa += 4
//      mode 0x8F   for i1 in 0..sz2: for (i2 = 0; i2 < db; i2 += 4)
//                      *da = x;        da += 4
//      anything else   nothing
//
//  sa and da are NOT reset between rows -- MAME's loops carry them -- so a run
//  is one flat sequence of (sz2 + 1) * ceil(db / 4) longs.  Each long is two
//  word accesses on this bus, high word first (big-endian), so the XOR splits
//  as x[31:16] over the word at `a` and x[15:0] over the word at `a + 2`.
//
//  ---- the bus, and who owns it -----------------------------------------------
//  Word access in the 68EC020's space, the same contract gx_esc uses and for
//  the same reason (DECISIONS D18, D19): req / we / addr / din / be hold until
//  the one-clock ack, rdata is valid with the ack, and gx_top lends this module
//  gx_main's transaction slice with the CPU frozen.  The two masters never run
//  together: a set is either sexyparo (the ESC, and it never writes 0xdb0000)
//  or fantjour (this, and it never writes 0xcc0000), and gokuparo writes
//  neither.
//
//  ---- a bound MAME does not have ---------------------------------------------
//  db is a 32-bit byte count and sz2 a byte, so a corrupt register file could
//  ask for 2^30 longs with the CPU frozen for all of them -- a hang with a
//  still picture, the worst failure to diagnose.  After LONG_LIMIT longs the
//  run ends and the CPU is handed back.  The measured worst case is
//  (14 + 1) * 0x80 / 4 = 480 longs (MEASUREMENTS 57), so the bound is 8x the
//  largest run this game has ever asked for.
//============================================================================
`default_nettype none

module gx_fjdma #(
    parameter [31:0] LONG_LIMIT = 32'd4096
) (
    input  wire        clk,
    input  wire        rst,

    // --- the register file, from gx_main's slice: word writes at 0xdb0000 ----
    input  wire        reg_wr,          // one clock: a word write into the file
    input  wire [4:1]  reg_a,           // word index, 0..15
    input  wire [15:0] reg_d,
    input  wire [1:0]  reg_be,          // {high byte, low byte}

    input  wire        start,           // one clock: run with the current file
    output wire        busy,
    output reg         done,            // one clock: every effect has happened

    output reg         bus_req,
    output reg         bus_we,
    output reg  [23:1] bus_addr,
    output reg  [15:0] bus_din,
    output reg  [1:0]  bus_be,
    input  wire        bus_ack,
    input  wire [15:0] bus_rdata,

    output reg  [15:0] dbg_longs        // longs the last run transferred
);

localparam [7:0] MODE_COPY = 8'h93;     // konamigx_m.cpp:518
localparam [7:0] MODE_FILL = 8'h8F;     // :528

reg [15:0] rf [0:15];

//  `start` NEVER arrives on the same clock as the trigger write, and that is a
//  CONTRACT of the integration, not an accident: gx_main registers the write
//  and the trigger together and raises `start` one clock later still, and
//  gx_top holds it further until the sprite DMA is quiet.  So the register file
//  already carries the new mode and sz2 by the time a run begins.
//
//  MEASURED, and this is why it is written down: the first version read word 0
//  through a same-clock bypass instead, and the build missed timing at
//  -1.202 ns on five paths that were all gx_main's cpu_a32 into this module's
//  state register -- the address bits reaching the next-state decision through
//  that bypass in one 96 MHz clock.  Removing it removes the path.
wire [7:0]  mode_eff = rf[0][15:8];
wire [7:0]  sz2_eff  = rf[0][7:0];
wire [31:0] sa_eff   = {rf[2],  rf[3]};
wire [31:0] da_eff   = {rf[7],  rf[8]};
wire [31:0] db_eff   = {rf[10], rf[11]};
wire [31:0] x_eff    = {rf[12], rf[13]};

typedef enum logic [2:0] {
    S_IDLE, S_ROW, S_RD_HI, S_RD_LO, S_WR_HI, S_WR_LO, S_NEXT, S_FIN
} st_t;

st_t        st;
reg  [7:0]  mode_r, sz2_r;
reg  [8:0]  i1;                 // 0..sz2, so nine bits
reg  [31:0] i2, sa_r, da_r, db_r, x_r, nlong;
reg  [15:0] hi_r, lo_r;

assign busy = (st != S_IDLE);

integer k;

always @(posedge clk) begin
    done <= 1'b0;

    if (rst) begin
        st        <= S_IDLE;
        bus_req   <= 1'b0;
        bus_we    <= 1'b0;
        bus_addr  <= 23'd0;
        bus_din   <= 16'd0;
        bus_be    <= 2'b00;
        dbg_longs <= 16'd0;
        mode_r    <= 8'd0;
        sz2_r     <= 8'd0;
        i1        <= 9'd0;
        i2        <= 32'd0;
        sa_r      <= 32'd0;
        da_r      <= 32'd0;
        db_r      <= 32'd0;
        x_r       <= 32'd0;
        nlong     <= 32'd0;
        hi_r      <= 16'd0;
        lo_r      <= 16'd0;
        for (k = 0; k < 16; k = k + 1) rf[k] <= 16'd0;
    end else begin
        // ---- the register file (COMBINE_DATA, :500) -------------------------
        if (reg_wr) begin
            if (reg_be[1]) rf[reg_a][15:8] <= reg_d[15:8];
            if (reg_be[0]) rf[reg_a][7:0]  <= reg_d[7:0];
        end

        case (st)
        S_IDLE:
            if (start) begin
                mode_r <= mode_eff;
                sz2_r  <= sz2_eff;
                sa_r   <= sa_eff;
                da_r   <= da_eff;
                db_r   <= db_eff;
                x_r    <= x_eff;
                i1     <= 9'd0;
                i2     <= 32'd0;
                nlong  <= 32'd0;
                st     <= (mode_eff == MODE_COPY || mode_eff == MODE_FILL)
                        ? S_ROW : S_FIN;
            end

        // MAME's two loops.  The inner one is `i2 < db` stepping by 4; the
        // outer runs sz2 + 1 times and does NOT reset sa or da.
        S_ROW:
            if (nlong == LONG_LIMIT)          st <= S_FIN;
            else if (i2 >= db_r) begin
                if (i1 == {1'b0, sz2_r})      st <= S_FIN;
                else begin
                    i1 <= i1 + 9'd1;
                    i2 <= 32'd0;
                end
            end else begin
                bus_req  <= 1'b1;
                bus_be   <= 2'b11;
                if (mode_r == MODE_COPY) begin
                    bus_we   <= 1'b0;
                    bus_addr <= sa_r[23:1];
                    st       <= S_RD_HI;
                end else begin
                    bus_we   <= 1'b1;
                    bus_addr <= da_r[23:1];
                    bus_din  <= x_r[31:16];
                    st       <= S_WR_HI;
                end
            end

        S_RD_HI:
            if (bus_ack) begin
                hi_r     <= bus_rdata ^ x_r[31:16];
                bus_addr <= sa_r[23:1] + 23'd1;      // + 2 bytes
                st       <= S_RD_LO;
            end

        S_RD_LO:
            if (bus_ack) begin
                lo_r     <= bus_rdata ^ x_r[15:0];
                bus_we   <= 1'b1;
                bus_addr <= da_r[23:1];
                bus_din  <= hi_r;
                st       <= S_WR_HI;
            end

        S_WR_HI:
            if (bus_ack) begin
                bus_addr <= da_r[23:1] + 23'd1;
                bus_din  <= (mode_r == MODE_COPY) ? lo_r : x_r[15:0];
                st       <= S_WR_LO;
            end

        S_WR_LO:
            if (bus_ack) begin
                bus_req <= 1'b0;
                bus_we  <= 1'b0;
                st      <= S_NEXT;
            end

        S_NEXT: begin
            da_r  <= da_r + 32'd4;
            if (mode_r == MODE_COPY) sa_r <= sa_r + 32'd4;
            i2    <= i2 + 32'd4;
            nlong <= nlong + 32'd1;
            st    <= S_ROW;
        end

        S_FIN: begin
            dbg_longs <= nlong[15:0];
            done      <= 1'b1;
            bus_req   <= 1'b0;
            bus_we    <= 1'b0;
            st        <= S_IDLE;
        end

        default: st <= S_IDLE;
        endcase
    end
end

endmodule

`default_nettype wire
