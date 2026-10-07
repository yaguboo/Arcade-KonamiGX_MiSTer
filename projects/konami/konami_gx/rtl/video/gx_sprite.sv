//============================================================================
//  Konami 053246 + 055673 -- System GX sprites
//
//  Sources, in root CLAUDE.md 1.2 order:
//
//    MAME     k053246_k053247_k055673.cpp/.h -- registers, sprite RAM format,
//             the GX draw (gxcore, yxloop_gx, zdrawgfxzoom32GP);
//             konamigx_v.cpp:23-110 and :491-610 -- which entries are drawn,
//             the z rule, c18.  Transcribed in docs/SOURCE_AUDIT.md 11.
//    SHELF 2  jtcores jt053246_scan and jtframe_draw, shipping in Run'n Gun
//             and the Simpsons family, as gx_objscan and gx_objdraw -- each
//             says what it changed.  docs/REUSE_PLAN.md section 1.
//    MEASURED tools/gx_objstat.py over 535 attract frames of MAME sprite RAM
//             (docs/MEASUREMENTS.md 18).
//
//  ---- the structure ------------------------------------------------------
//     object RAM (gx_top) --DMA at vblank--> zcode-sorted table (256 slots)
//       --one line at a time--> gx_objscan --> gx_objdraw --> line buffer
//       --one dot at a time--> pix_* --> gx_prio's OBJ input
//
//  ---- WHY ONE SPRITE PIXEL PER DOT IS ENOUGH FOR THE K055555 --------------
//  MAME draws sprites and layers back to front in (priority, zcode) order,
//  but the sprites also share a z-buffer that the layers never touch, and a
//  sprite pixel is drawn only where the buffer holds a zcode >= its own
//  (k053246_...cpp:523).  So among the sprites covering a dot, the one with
//  the SMALLEST zcode ends up in the frame, and every layer then competes
//  with THAT sprite's priority alone -- which is what a K055555 with one OBJ
//  input does.  The line buffer below holds exactly that pixel: the table is
//  walked from the back (largest zcode) to the front and each opaque pixel
//  overwrites.  Worked through case by case in docs/DECISIONS.md D12.
//
//  Equal zcodes are where the two differ.  Upstream (jt053246_dma) lets the
//  second entry land in the first one's slot and lose it, and that was
//  accepted here on a gokuparo measurement -- zero duplicate on-screen zcodes
//  in 535 frames.  **SEXY PARODIUS IS NOT LIKE THAT.**  Every one of its
//  eighteen active entries carries zcode 0 (word 0 = 0xAB00, 0xA000, 0xA400,
//  ... -- the low byte is 00 in all of them), so all eighteen wanted one slot
//  and seventeen were lost, which is the whole letterbox mask of its attract
//  (docs/MEASUREMENTS.md 77, 78).
//
//  MAME loses none: its key is `pri << 24 | zcode << 16 | offs << 5 | mode << 4`
//  (konamigx_v.cpp:598), so position breaks the tie, and the pool is drawn
//  back to front in descending order -- at equal zcode the HIGHER offset is
//  drawn first and ends up BEHIND.
//
//  So nothing is dropped here either: the capture is a COUNTING SORT by
//  zcode.  The table has 256 slots and there are at most 256 active entries,
//  so a stable sort always fits, and it reproduces MAME's order exactly
//  rather than approximately.
//
//      pass 1   histogram: how many active entries land on each slot_of
//      pass 2   prefix sum -> a per-bucket pointer at the bucket's LAST slot
//      pass 3   capture, walking entries in ASCENDING offset and taking the
//               pointer DOWNWARD, so the lowest offset of a group gets the
//               highest slot and is drawn last -- in front, as MAME has it
//
//  About 2,000 extra clocks at vblank against roughly 245,000 available.
//
//  This replaces the downward PROBE that shipped first (D22).  The probe was
//  exact whenever one zcode was in play and wrong when a group ran into
//  another group's slots: 247 dots on one frame in twelve, measured rather
//  than assumed (docs/MEASUREMENTS.md 83).  docs/DECISIONS.md D22.
//
//  ---- the draw order comes from OPSET, not from a parameter ----------------
//  MAME inverts the zcode when OPSET bit 4 is set (konamigx_v.cpp:503, "see
//  p.51 OPSET PRI"), and jt053246's K55673_DESC_SORT parameter is the same bit
//  frozen per game ("programmed on register 12, bit 4, but it is never
//  changed").  Two independent readings of one bit, so it is read live here:
//      OPSET[4] = 0   smaller zcode in front   slot = ~zcode
//      OPSET[4] = 1   larger zcode in front    slot =  zcode
//  and slots are drawn in ascending order.  gokuparo: OPSET = 0x0001.
//============================================================================
`default_nettype none

module gx_sprite #(
    parameter [9:0] VOFFSET = 10'd0,
    parameter integer LB_AW = 1,       // 2**LB_AW line buffers, see "lines drawn ahead"
    parameter integer PIPE  = 0        // gx_objdraw change 11, MEASUREMENTS 132
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        pxl_cen,
    // HOFFSET was a parameter until 2026-09-16; it is per set now (gx_top has
    // the derivation: gokuparo 954, sexyparo 958 -- DECISIONS D18).
    input  wire [9:0]  hoffset,

    // --- CPU --------------------------------------------------------------------
    //  OBJSET1 d48000-d48007: k053246_w is a BYTE handler, so each byte lane of
    //  the 16-bit port is its own register (reg 2n in D[15:8], 2n+1 in D[7:0]).
    //  OBJSET2 d4a010-d4a01f: k055673_reg_word_w, eight 16-bit words.
    input  wire        objset1_cs,
    input  wire        objset2_cs,
    input  wire        cpu_we,
    input  wire [3:1]  cpu_addr,
    input  wire [15:0] cpu_din,
    input  wire [1:0]  cpu_ds,
    input  wire        spri_sel18,     // control_w bit 18 (gx.cpp:533)
    input  wire        spri_sel19,     // control_w bit 19

    // --- object RAM, owned by gx_top: this module borrows its read port -------
    output reg         dma_rd,
    output reg  [12:0] dma_addr,       // word index
    input  wire [15:0] objram_q,       // registered one clock after dma_addr
    output reg         dma_busy,       // d5a003 bit 1, the DMA busy flag
    input  wire        dma_hold,       // an ESC run is writing object RAM: defer the copy (D18)
    output wire        dma_quiet,      // no copy running, pending or starting this clock (D18)

    // --- raster -----------------------------------------------------------------
    input  wire [9:0]  hcnt,
    input  wire [8:0]  vcnt,
    input  wire [9:0]  htotal,
    input  wire [9:0]  vtotal,
    input  wire [9:0]  hres,
    input  wire [9:0]  vres,

    // --- ROM, gx_objdraw's handshake passed through ---------------------------
    input  wire [1:0]  fmt,            // sprite ROM format: 0 GX 5bpp, 1 GX6 6bpp, 2 RNG 4bpp, 3 LE2 8bpp
    input  wire        spr_big,        // GX 5bpp with an 8 MB 4bpp area (tokkae, tkmmpzdm): 65536 tiles
    output wire [21:0] rom_unit,
    output wire        rom_req,
    input  wire        rom_ok,
    input  wire [31:0] rom_data4,
    input  wire [31:0] rom_data1,

    // --- to gx_prio, all loaded on pxl_cen --------------------------------------
    output reg  [7:0]  pix_pen,
    output reg  [15:0] pix_c18,
    output reg  [9:0]  pix_attr,       // the raw attribute (salmndr2 / dragoonj priority callbacks)
    output reg  [3:0]  pix_coregshift,
    input  wire [3:1]  shd_live,       // gx_colmix: shadow preset N is drawn at all (MEASUREMENTS 152)
    output reg  [1:0]  pix_shd,        // the lowest shadow code present at the dot, 0 = none
    output reg  [3:1]  pix_shdm,       // every shadow code present at the dot (STACKED SHADOWS)
    output reg         pix_sdsel,
    output reg         pix_shd_front,  // the dot's shadow was drawn after its solid pixel: nearer in z

    // --- observability ----------------------------------------------------------
    output reg         dbg_late,       // one clock: a line ran out of time
    output reg         dbg_dma,        // one clock: a DMA copy finished
    // 123.3/123.4: the tile gx_objscan currently holds, as a prefetch hint for
    // gx_sprfetch.  MEASURED to be the fetcher next request 37 % of the time.
    output wire [21:0] pf_unit,
    output wire        pf_valid
);

// ---------------------------------------------------------------------------
//  registers
// ---------------------------------------------------------------------------
reg [7:0]  r46 [0:7];
reg [15:0] r47 [0:7];

integer ri;
always @(posedge clk) begin
    if (rst) begin
        for (ri = 0; ri < 8; ri = ri + 1) begin
            r46[ri] <= 8'd0;
            r47[ri] <= 16'd0;
        end
    end else if (cpu_we) begin
        if (objset1_cs) begin
            if (cpu_ds[1]) r46[{cpu_addr[2:1], 1'b0}] <= cpu_din[15:8];
            if (cpu_ds[0]) r46[{cpu_addr[2:1], 1'b1}] <= cpu_din[ 7:0];
        end
        if (objset2_cs) begin
            if (cpu_ds[1]) r47[cpu_addr[3:1]][15:8] <= cpu_din[15:8];
            if (cpu_ds[0]) r47[cpu_addr[3:1]][ 7:0] <= cpu_din[ 7:0];
        end
    end
end

// OBJSET1 (k053246_...cpp:19-33).  MAME reads the offsets as 16-bit signed;
// the scan works modulo 1024, which is the wrap MAME applies when OPSET bit 6
// is clear (h:193, gokuparo OPSET = 0x0001).  UNVERIFIED for OPSET bit 6 set.
wire [9:0] xoffset  = { r46[0][1:0], r46[1] };
wire [9:0] yoffset  = { r46[2][1:0], r46[3] };
wire       ghf      = r46[5][0];
wire       gvf      = r46[5][1];
wire       shd_bit5 = r46[5][5];
wire       dmaen    = r46[5][4];          // DMA enable, sampled at vblank begin

// OBJSET2 (konamigx_v.cpp:30-45)
wire [15:0] opset = r47[6];
wire [2:0]  cs_i  = (opset[2:0] > 3'd4) ? 3'd4 : opset[2:0];
reg  [3:0]  coregshift;
reg  [3:0]  coregmask;
always @(*) begin
    case (cs_i)
        3'd0:    begin coregshift = 4'd4; coregmask = 4'hf; end
        3'd1:    begin coregshift = 4'd5; coregmask = 4'he; end
        3'd2:    begin coregshift = 4'd6; coregmask = 4'hc; end
        3'd3:    begin coregshift = 4'd7; coregmask = 4'h8; end
        default: begin coregshift = 4'd8; coregmask = 4'h0; end
    endcase
end
wire [15:0] coreg = { opset[11:8] & coregmask, 12'd0 };

// ---------------------------------------------------------------------------
//  object DMA -- a copy of the table into zcode order, at vblank begin
//
//  EMULATION_DERIVED
//  Matches MAME konamigx.cpp:620-625, "begin transfer if DMAEN(bit4 of
//  OBJSET1) is set (see p.48)" -- MAME's code quoting the manual, evaluated
//  at vblank begin.  MAME itself never copies for GX (U30:
//  konamigx_mixer_init(screen, 0)); its mixer reads the live sprite RAM at
//  vblank END (VIDEO_UPDATE_AFTER_VBLANK, konamigx.cpp:1744).
//  The actual PCB trigger is NOT verified.  TODO(HARDWAREIZE): U30.
//
//  MEASURED (tools/gx_dmatap.lua -> dist/dmatap_11000.txt, 11000 frames
//  through gameplay): DMAEN is set at vblank begin in 10258 frames and clear
//  in 82 -- frame 660 and exactly the lag frames, the ones in which the
//  program never reaches its busy poll.  The program clears DMAEN at line 230
//  and rewrites the sprite list after it, so in a lag frame the list is still
//  being written: the gate keeps the previous frame's sprites where MAME draws
//  the half-written RAM.  That difference is deliberate and documented.
//
//  An earlier reading of dist/regwrites_2900.csv ruled this gate out because
//  DMAEN rose "after vblank" from frame 2700.  That csv's line 0 is vblank END
//  -- MAME's frame notifier moves there with VIDEO_UPDATE_AFTER_VBLANK -- so
//  vblank begins at its line 224 and those rises at lines 1-60 precede it.
//
//  IRQ3 is not generated (gokuparo never enables it); the busy bit is, below.
//
//  Word 7 is not copied: nothing in the GX draw reads it ("game dependent").
// ---------------------------------------------------------------------------
localparam [3:0] D_IDLE = 4'd0,  D_CLR  = 4'd1,  D_WAIT = 4'd2,  D_CAP  = 4'd3,
                 D_HCLR = 4'd4,  D_HW1  = 4'd5,  D_HW2  = 4'd6,  D_HRD  = 4'd7,
                 D_HINC = 4'd8,  D_PS1  = 4'd9,  D_PS2  = 4'd10, D_PS3  = 4'd11,
                 D_TRD  = 4'd12, D_TGET = 4'd13;
reg  [3:0]  dst;
reg  [7:0]  d_ent;
reg  [2:0]  d_word;
reg  [7:0]  d_slot;
// The counting sort's bucket array: one 9-bit count per slot_of, which the
// prefix pass turns into that bucket's DESCENDING write pointer.  An UNPACKED
// array with one registered read, so Quartus is welcome to make a memory of
// it -- unlike `act`, nothing reads this 256 wide.
reg  [8:0]  hist [0:255];
reg  [7:0]  h_idx;
reg  [8:0]  h_q;
reg  [8:0]  h_run;             // running total during the prefix pass
reg         vb_prev;
reg         d_pend;             // the vblank asked for a copy and an ESC run held it

reg         lut_we_e, lut_we_o;
reg  [9:0]  lut_wa;
reg  [15:0] lut_wd;

// Every slot's active bit: gx_objscan's skip (its change 9) reads it, and so
// does the DMA's own collision probe below.  Word 0 is the only word written
// with lut_wa[1:0] = 0 -- the clear pass writes it zero and the capture
// writes the entry's word 0 -- so this is exactly bit 15 of every slot's
// word 0 as the table holds it, which is what "is this slot taken" means.
//
// THIS IS A REGISTER FILE, NOT A MEMORY.  gx_objscan reads all 256 bits at
// once (32 byte-slices for `byte_any`, one more for `cur_rest`), which an
// altsyncram cannot serve.  It has a variable WRITE index; give it a variable
// READ index as well and Quartus infers one anyway -- a 276020 on `act[0]`
// and +1 M10K, measured twice on 2026-09-22 (docs/WARNING_BASELINE.md), and
// `ramstyle = "logic"` does not stop it, because this is a packed vector and
// the attribute only holds unpacked arrays.  The counting sort needs no such
// read, which is a second reason to prefer it to the probe it replaced.
reg [255:0] act = 256'd0;
always @(posedge clk)
    if (lut_we_e && lut_wa[1:0] == 2'b00)
        act[lut_wa[9:2]] <= lut_wd[15];

// One registered read port; the FSM below owns the writes.  h_idx is set one
// state before the value is used, which is the same discipline the object RAM
// read already follows.
always @(posedge clk) h_q <= hist[h_idx];

wire        vb_now = ({1'b0, vcnt} == vres);
wire [7:0]  slot_of = opset[4] ? objram_q[7:0] : ~objram_q[7:0];


//  ---- the ESC (DECISIONS D18) ----------------------------------------------
//  Sexy Parodius's ESC writes the sprite list into object RAM while the CPU is
//  frozen, and a copy taken during a run would tear.  So a trigger that
//  arrives while `dma_hold` is up is remembered and the copy starts when it
//  drops; with no run in progress the copy starts on the same clock as it
//  always did.  gx_top starts a run only while `dma_quiet`.  The CPU cannot
//  change DMAEN during a run, so the deferred copy is the one vblank asked for.
wire        d_trig    = vb_now && !vb_prev && dmaen;
assign      dma_quiet = (dst == D_IDLE) && !d_pend && !d_trig;

always @(posedge clk) begin
    lut_we_e <= 1'b0;
    lut_we_o <= 1'b0;
    dbg_dma  <= 1'b0;
    if (rst) begin
        dst     <= D_IDLE;
        dma_rd  <= 1'b0;
        vb_prev <= 1'b0;
        d_pend  <= 1'b0;
    end else begin
        vb_prev <= vb_now;
        case (dst)
            D_IDLE: if (d_trig || d_pend) begin
                if (dma_hold) begin
                    d_pend <= 1'b1;
                end else begin
                    d_pend <= 1'b0;
                    d_ent  <= 8'd0;
                    dst    <= D_CLR;
                end
            end
            // Only word 0 of each slot has to be cleared: it carries the
            // active bit, and nothing reads the other words of an idle slot.
            // Only word 0 of each slot has to be cleared: it carries the
            // active bit, and nothing reads the other words of an idle slot.
            D_CLR: begin
                lut_we_e <= 1'b1;
                lut_wa   <= { d_ent, 2'd0 };
                lut_wd   <= 16'd0;
                d_ent    <= d_ent + 8'd1;
                if (d_ent == 8'hff) begin
                    d_ent <= 8'd0;
                    dst   <= D_HCLR;
                end
            end

            // ---- pass 1a: empty the buckets -----------------------------
            D_HCLR: begin
                hist[d_ent] <= 9'd0;
                d_ent       <= d_ent + 8'd1;
                if (d_ent == 8'hff) begin
                    d_ent    <= 8'd0;
                    dma_rd   <= 1'b1;
                    dma_addr <= 13'd0;
                    dst      <= D_HW1;
                end
            end

            // ---- pass 1b: count the active entries per slot_of -----------
            // Word 0 only, so the walk steps by eight.
            D_HW1: dst <= D_HW2;
            D_HW2: begin
                if (objram_q[15]) begin
                    h_idx <= slot_of;
                    dst   <= D_HRD;
                end else begin
                    if (d_ent == 8'hff) begin
                        d_ent <= 8'd0;
                        dst   <= D_PS1;
                    end else begin
                        d_ent    <= d_ent + 8'd1;
                        dma_addr <= { 2'b00, d_ent + 8'd1, 3'd0 };
                        dst      <= D_HW1;
                    end
                end
            end
            D_HRD: dst <= D_HINC;             // h_q settles
            D_HINC: begin
                hist[h_idx] <= h_q + 9'd1;
                if (d_ent == 8'hff) begin
                    d_ent <= 8'd0;
                    dst   <= D_PS1;
                end else begin
                    d_ent    <= d_ent + 8'd1;
                    dma_addr <= { 2'b00, d_ent + 8'd1, 3'd0 };
                    dst      <= D_HW1;
                end
            end

            // ---- pass 2: prefix sum -> each bucket's LAST slot -----------
            // hist[k] becomes the slot the NEXT entry of bucket k takes, and
            // the capture walks it downward.  A bucket with no entries gets a
            // meaningless pointer that nothing ever reads.
            D_PS1: begin
                h_idx <= d_ent;
                if (d_ent == 8'd0) h_run <= 9'd0;
                dst   <= D_PS2;
            end
            D_PS2: dst <= D_PS3;              // h_q settles
            D_PS3: begin
                hist[h_idx] <= h_run + h_q - 9'd1;
                h_run       <= h_run + h_q;
                if (d_ent == 8'hff) begin
                    d_ent    <= 8'd0;
                    d_word   <= 3'd0;
                    dma_addr <= 13'd0;
                    dst      <= D_WAIT;
                end else begin
                    d_ent <= d_ent + 8'd1;
                    dst   <= D_PS1;
                end
            end

            // ---- pass 3: the capture, unchanged except where a slot comes
            // from.  The read port is registered: an address set on this edge
            // is readable two edges later.
            D_WAIT: dst <= D_CAP;
            D_CAP: begin
                if (d_word == 3'd0) begin
                    if (objram_q[15]) begin
                        // Take this bucket's pointer.  dma_addr does not move,
                        // so objram_q still holds word 0 in D_TGET.
                        h_idx <= slot_of;
                        dst   <= D_TRD;
                    end else begin
                        d_word <= 3'd7;        // inactive: skip the entry
                    end
                end else begin
                    lut_we_e <= !d_word[0];
                    lut_we_o <=  d_word[0];
                    lut_wa   <= { d_slot, d_word[2:1] };
                    lut_wd   <= objram_q;
                    d_word   <= d_word + 3'd1;
                end
                // next address.  Word 0 of an ACTIVE entry is handled by
                // D_TGET, which advances the address itself when it commits.
                if (d_word == 3'd0 && objram_q[15]) begin
                    // nothing: dst is D_TRD
                end else if ((d_word == 3'd6) || (d_word == 3'd0 && !objram_q[15])) begin
                    if (d_ent == 8'hff) begin
                        dma_rd  <= 1'b0;
                        dbg_dma <= 1'b1;
                        dst     <= D_IDLE;
                    end else begin
                        d_ent    <= d_ent + 8'd1;
                        d_word   <= 3'd0;
                        dma_addr <= { 2'b00, d_ent + 8'd1, 3'd0 };
                        dst      <= D_WAIT;
                    end
                end else begin
                    dma_addr <= { 2'b00, d_ent, (d_word == 3'd0) ? 3'd1 : d_word + 3'd1 };
                    dst      <= D_WAIT;
                end
            end
            D_TRD: dst <= D_TGET;             // h_q settles
            D_TGET: begin
                d_slot      <= h_q[7:0];
                lut_we_e    <= 1'b1;
                lut_wa      <= { h_q[7:0], 2'd0 };
                lut_wd      <= objram_q;
                hist[h_idx] <= h_q - 9'd1;
                d_word      <= 3'd1;
                dma_addr    <= { 2'b00, d_ent, 3'd1 };
                dst         <= D_WAIT;
            end
            default: dst <= D_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------------
//  DMA busy -- d5a003 bit 1
//
//  EMULATION_DERIVED
//  Matches MAME konamigx.cpp:612-633 -- raised at vblank begin whatever DMAEN
//  says, dropped 342+42 us later (256+32 when control_w bit 16 selects 8 MHz).
//  The timing table above it (:581-587) reads 256 dots of clear plus 2048 of
//  transfer at both 6 and 8 MHz, so the length is counted here in dots: 2304.
//  (At 12 / 16 MHz the table is not dot-shaped; gokuparo runs at 6.)
//  Required because the game waits on it: MEASURED with tools/gx_dmatap.lua,
//  11000 frames through gameplay, PC 2A0A9C reads it every frame from vblank
//  begin until it clears, and clears DMAEN right after.  With the bit stuck
//  at 0 the program ran on ~6 lines a frame early.
//  The actual PCB source is NOT verified.
//  TODO(HARDWAREIZE): U10 -- does the real flag rise with DMAEN clear?
// ---------------------------------------------------------------------------
reg  [11:0] busy_cnt;

always @(posedge clk) begin
    if (rst) begin
        dma_busy <= 1'b0;
        busy_cnt <= 12'd0;
    end else if (vb_now && !vb_prev) begin
        dma_busy <= 1'b1;
        busy_cnt <= 12'd0;
    end else if (dma_busy && pxl_cen) begin
        busy_cnt <= busy_cnt + 12'd1;
        if (busy_cnt == 12'd2303) dma_busy <= 1'b0;
    end
end

// The table.  One write port and one read port each, whole-word writes, and
// the read address is never the write expression -- the shape Quartus 17.0
// infers as M10K (gx_tilemap's measurement).  256 x 4 words, even and odd.
(* ramstyle = "M10K" *) reg [15:0] lut_e [0:1023];
(* ramstyle = "M10K" *) reg [15:0] lut_o [0:1023];
reg  [15:0] scan_even, scan_odd;
wire [9:0]  scan_addr;

always @(posedge clk) begin
    if (lut_we_e) lut_e[lut_wa] <= lut_wd;
    if (lut_we_o) lut_o[lut_wa] <= lut_wd;
    scan_even <= lut_e[scan_addr];
    scan_odd  <= lut_o[scan_addr];
end


// ---------------------------------------------------------------------------
//  lines drawn ahead, into a ring of line buffers
//
//  2**LB_AW buffers, and line L is drawn into buffer L mod N.  Line L is on
//  display from the dot where hcnt becomes hres on line L-1 (the start of
//  horizontal blanking, after line L-1's last visible dot has been read) to
//  the same dot of line L.
//
//  One line in hand at a time, in line order: CLEAR its buffer (hres writes,
//  a few hundred clocks), then scan and draw.  Line L is taken up once the
//  line before it is finished and L is 1..N-1 lines ahead of the line on
//  display, modulo the frame -- by then L-N, the last line in its buffer, has
//  left the display.  A line still in hand when it goes on display is stopped
//  there.  It always had at least one whole line, because it was taken up no
//  later than the edge that stopped the line before it.
//
//  LB_AW = 1 is the two-half buffer this replaced, two clocks late: L is taken
//  up just after the edge that puts L-1 on display, where its window used to
//  open, and stopped just after the next, where the window used to close (the
//  decisions are registered, see below).  A larger ring
//  lets a busy line use time that light lines before it left over, and the
//  demo stages run 15-30 busy lines in a row (docs/MEASUREMENTS.md 34).
//
//  Needs vres to be a multiple of N and N well under vtotal - vres, so that a
//  frame's first N lines start only in vertical blanking and never share a
//  buffer with a line still to be shown (gokuparo: 224 and 40).  No line is
//  taken up on the DMA's line or while the DMA copies.
//
//  No generation tag: the factory memory
//  `generation-tag-cannot-replace-clearing` is about exactly this buffer.
//  And no read-and-clear of one address in one clock, which is the shape
//  power_spikes L24 measured turning into 7,673 ALM of flip-flops.
// ---------------------------------------------------------------------------
localparam integer LB_N   = 1 << LB_AW;
localparam [9:0]   LB_N10 = 10'd1 << LB_AW;

reg  [9:0]  hcnt_d;
reg  [8:0]  disp_line;          // the line on display
reg  [8:0]  next_line;          // the line to take up next
reg  [8:0]  draw_line;          // the line in hand
reg         job_busy;
reg         clr_busy;
reg  [8:0]  clr_x;
reg         window_live;        // its buffer is clear and its scan started
reg         scan_seen;          // ... and the scan has left its last `done`
reg         scan_start;
wire        scan_done;

wire        dr_busy, dr_active, dr_we, dr_shd_we, sc_draw;
wire [9:0]  dr_x;
wire [19:0] dr_din;
wire [1:0]  dr_shd;

wire        line_edge = (hcnt == hres) && (hcnt_d != hres);
wire [9:0]  v_plus1   = {1'b0, vcnt} + 10'd1;
wire [9:0]  v_wrap    = (v_plus1 >= vtotal) ? (v_plus1 - vtotal) : v_plus1;

// Every decision below reads registers only.  The first cut decided within
// the clock of the edge -- raster adder, compare, close, next line, lead
// adder, compare, take up -- and missed the 96 MHz clock by 3.693 ns through
// fifteen levels (build of ad95db0, gx_ccu|v -> clr_x).  Registered, a line is
// stopped one clock after the edge that puts it on display (its first visible
// dot is 1,536 clocks later), and taken up two clocks after it was released or
// the line before it closed.
reg         edge_d;             // the clock after line_edge: disp_line is new
reg         close_d;            // the clock after a close: next_line is new
reg         released;           // next_line is 1..N-1 lines ahead of disp_line
reg         table_ok;           // neither the DMA's line nor the one before it

// The scan hands over its last tile on the clock its `done` rises, and the
// draw is busy with it a clock later -- so `sc_draw` is part of "finished".
wire        job_done  = window_live && scan_seen && scan_done && !sc_draw && !dr_active;
wire        stop_now  = job_busy && edge_d && (disp_line == draw_line);
wire        closing   = job_busy && (job_done || stop_now);
wire        take_up   = !job_busy && !close_d && released && table_ok;
wire [9:0]  line_inc  = {1'b0, draw_line} + 10'd1;
wire [9:0]  ahead_r   = {1'b0, next_line} - {1'b0, disp_line};
wire [9:0]  ahead     = ahead_r[9] ? ahead_r + vtotal : ahead_r;

always @(posedge clk) begin
    scan_start <= 1'b0;
    dbg_late   <= 1'b0;
    hcnt_d     <= hcnt;
    edge_d     <= line_edge;
    close_d    <= closing;
    released   <= (ahead != 10'd0) && (ahead < LB_N10);
    table_ok   <= (dst == D_IDLE) && !vb_now && (v_plus1 != vres);
    if (rst) begin
        disp_line   <= 9'd0;
        next_line   <= 9'd0;
        draw_line   <= 9'd0;
        job_busy    <= 1'b0;
        clr_busy    <= 1'b0;
        clr_x       <= 9'd0;
        window_live <= 1'b0;
        scan_seen   <= 1'b0;
    end else begin
        if (line_edge)
            disp_line <= v_wrap[8:0];
        if (window_live && !scan_done)
            scan_seen <= 1'b1;
        if (closing) begin
            dbg_late    <= !job_done;
            job_busy    <= 1'b0;
            window_live <= 1'b0;
            clr_busy    <= 1'b0;
            next_line   <= (line_inc == vres) ? 9'd0 : line_inc[8:0];
        end
        if (take_up) begin
            job_busy    <= 1'b1;
            draw_line   <= next_line;
            clr_busy    <= 1'b1;
            clr_x       <= 9'd0;
            window_live <= 1'b0;
            scan_seen   <= 1'b0;
        end else if (clr_busy && !closing) begin
            clr_x <= clr_x + 9'd1;
            if ({1'b0, clr_x} == hres - 10'd1) begin
                clr_busy    <= 1'b0;
                scan_start  <= 1'b1;
                window_live <= 1'b1;
            end
        end
    end
end

// Anything the draw still has in flight when its line goes on display belongs
// to that line, so it is stopped here.
wire abort = stop_now;

// { attr10, pen8 }: pen bit 5 is salmndr2's (GX6), bits 7-6 winspike's (LE2);
// 0 for the others.  18 bits since 2026-10-05 (was 16, pen6).
(* ramstyle = "M10K" *) reg [17:0] lbuf [0:LB_N*512-1];
reg  [17:0] lb_q;

wire        dr_visible = !dr_x[9] && (dr_x[8:0] < hres[8:0]);
wire        lb_we      = clr_busy || (dr_we && dr_visible && window_live);
// change 11: the CLEAR belongs to the scan's current line, but a DRAW belongs
// to the line its tile was issued for -- which with PIPE may no longer be the
// current one.  gx_objdraw carries it back in `dr_dline`.
wire [LB_AW+8:0] lb_wa = clr_busy ? { draw_line[LB_AW-1:0], clr_x }
                                  : { dr_dline[LB_AW-1:0], dr_x[8:0] };
wire [7:0]  dr_dline;
wire [17:0] lb_wd      = clr_busy ? 18'd0 : dr_din[17:0];
// Read one dot AHEAD so the pxl_cen stage below holds dot h while hcnt = h,
// the same moment gx_tilemap's pixel for dot h stands.
wire [9:0]  hnext      = ({hcnt} + 10'd1 == htotal) ? 10'd0 : hcnt + 10'd1;
wire [LB_AW+8:0] lb_ra = { disp_line[LB_AW-1:0], hnext[8:0] };

always @(posedge clk) begin
    if (lb_we) lbuf[lb_wa] <= lb_wd;
    lb_q <= lbuf[lb_ra];
end

// ---- the shadow line buffer, 2026-09-14 -------------------------------------
//  MAME draws a sprite's shadow as its own object with its own z-buffer
//  (gx_shdzbuf) and priority (SHDPRI, or the sprite's own under OPSET SDSEL),
//  so a shadow pixel must not overwrite the solid pixel a sprite behind it
//  left in `lbuf`.  It gets a buffer of its own, same halves, same clear, same
//  back-to-front walk -- so it holds the smallest zcode's shadow, as `lbuf`
//  holds the smallest zcode's solid pen.  Two shadows at one dot do not stack
//  here; MAME stacks them only when their priorities differ (UNVERIFIED on
//  this game, which puts one shadow-coded entry up at a time in 535 attract
//  frames and at most two in its gameplay dump).
//  MEASURED use (dist/objdump, dist/gamedump): every curtain change and the
//  white flash at 2240-2360 are shadow sprites; 52 % of gameplay frames carry
//  shadow-coded entries.
//
//  STACKED SHADOWS, 2026-09-29 (MEASUREMENTS 155).  The buffer used to hold ONE
//  code a dot, the nearest.  tbyahhoo's demo explanation screens darken the
//  whole screen with a code-2 shadow AND the 1P panel with a code-3 one, and
//  MAME draws both (k053246_k053247_k055673.cpp:570: a later shadow lands when
//  it is nearer-or-equal in z AND strictly lower in priority).  So each code
//  now has a one-bit buffer of its own -- three independent writes, no
//  read-modify-write -- and gx_prio decides which of the present codes land.
//  APPROXIMATION: the z order BETWEEN shadows is not kept; see gx_prio.
//
//  A shadow whose '338 preset is within +/-7 is not drawn (gx_colmix,
//  SHADOW PRESET LIVE).
(* ramstyle = "M10K" *) reg sbuf1 [0:LB_N*512-1];
(* ramstyle = "M10K" *) reg sbuf2 [0:LB_N*512-1];
(* ramstyle = "M10K" *) reg sbuf3 [0:LB_N*512-1];
reg  [3:1]  sb_q;
wire [3:0]  shd_live4 = {shd_live, 1'b0};    // code 0 is never a shadow pixel
wire        dr_shd_ok = dr_shd_we && shd_live4[dr_shd];
wire        sb_draw   = dr_shd_ok && dr_visible && window_live;
always @(posedge clk) begin
    if (clr_busy || (sb_draw && dr_shd == 2'd1)) sbuf1[lb_wa] <= !clr_busy;
    if (clr_busy || (sb_draw && dr_shd == 2'd2)) sbuf2[lb_wa] <= !clr_busy;
    if (clr_busy || (sb_draw && dr_shd == 2'd3)) sbuf3[lb_wa] <= !clr_busy;
    sb_q <= {sbuf3[lb_ra], sbuf2[lb_ra], sbuf1[lb_ra]};
end

// ---- which of the two was drawn last at each dot, 2026-09-14 --------------
//  MAME sorts a shadow object and a solid sprite by `pri << 24 | zcode << 16`
//  (konamigx_v.cpp:600, :607), and a shadow darkens only what is already in
//  the bitmap.  So at EQUAL priority zcode decides: a shadow nearer than the
//  sprite lands on it, one further away is covered by it.  gx_prio sees only
//  priorities, so the zcode half comes from here.  The walk is back to front,
//  so the later write at a dot is the nearer one: this bit is 1 where the last
//  write was the shadow and 0 where it was a solid pixel.
//  MEASURED on MAME memory (docs/MEASUREMENTS.md 37): the attract's curtain
//  flash is a full-screen shadow (pri 12, z 15-16) behind the framed picture
//  (pri 12, z 14-15) and MAME darkens none of the picture's 15,906 dots; the
//  demo's explosion flash (preset -255) darkens 296 sprite dots in MAME
//  against 54,542 when every tie landed.
(* ramstyle = "M10K" *) reg zbuf [0:LB_N*512-1];
reg         zb_q;
wire        zb_we = clr_busy || ((dr_we || dr_shd_ok) && dr_visible && window_live);
wire        zb_wd = !clr_busy && dr_shd_ok;
always @(posedge clk) begin
    if (zb_we) zbuf[lb_wa] <= zb_wd;
    zb_q <= zbuf[lb_ra];
end
always @(posedge clk)
    if (rst)          pix_shd_front <= 1'b0;
    else if (pxl_cen) pix_shd_front <= zb_q;

// ---------------------------------------------------------------------------
//  scan and draw
// ---------------------------------------------------------------------------
wire [15:0] sc_code;
wire [9:0]  sc_attr, sc_hpos;
wire        sc_hflip, sc_vflip;
wire [3:0]  sc_ysub;
wire [7:0]  sc_tzw;
wire [1:0]  sc_shd;

gx_objscan #(
    .VOFFSET (VOFFSET)
) u_scan (
    .rst       (rst),
    .clk       (clk),
    .hoffset   (hoffset),
    .start     (scan_start),
    .vline     (draw_line),
    .done      (scan_done),
    .code      (sc_code),
    .attr      (sc_attr),
    .hflip     (sc_hflip),
    .vflip     (sc_vflip),
    .hpos      (sc_hpos),
    .ysub      (sc_ysub),
    .tzw       (sc_tzw),
    .shd       (sc_shd),
    .dr_start  (sc_draw),
    .dr_busy   (dr_busy),
    .scan_even (scan_even),
    .scan_odd  (scan_odd),
    .scan_addr (scan_addr),
    .xoffset   (xoffset),
    .yoffset   (yoffset),
    .ghf       (ghf),
    .gvf       (gvf),
    .act       (act)
);

// type2_sprite_callback (konamigx_v.cpp:106): the top two code bits select a
// 4-bit bank from OBJSET2 words 4 and 5.  The draw then takes the tile number
// modulo the gfx element count, 0x400000 / 128 = 32768 (k053246_...cpp:703),
// so of the bank only bit 14 survives.  gokuparo's banks are the identity
// (words 0100 / 0302, MEASURED), like its tile bank.
reg  [3:0]  vrcbk;
always @(*) begin
    case (sc_code[15:14])
        2'd0: vrcbk = r47[4][ 3:0];
        2'd1: vrcbk = r47[4][11:8];
        2'd2: vrcbk = r47[5][ 3:0];
        2'd3: vrcbk = r47[5][11:8];
    endcase
end
// RNG (dragoonj, 16 MB): 131072 tiles, so three bank bits survive the
// modulo; GX and GX6 hold 32768 (4 MB / 128, 6 MB / 192), one bit.  A GX set
// with an 8 MB 4bpp area (tokkae, tkmmpzdm: k055673 region 0xa00000, the GX
// layout's 4bpp part (region / 5) * 4, k053246_k053247_k055673.cpp:685)
// holds 65536, two bits.
// LE2 (winspike, 16 MB / 256 bytes a tile) also holds 65536.
wire [16:0] dr_code = (fmt == 2'd2) ? { vrcbk[2:0], sc_code[13:0] }
                    : (spr_big || fmt == 2'd3) ? { 1'b0, vrcbk[1:0], sc_code[13:0] }
                                    : { 2'b00, vrcbk[0], sc_code[13:0] };
assign pf_unit  = { dr_code, sc_ysub ^ {4{sc_vflip}}, 1'b0 };
assign pf_valid = window_live;

gx_objdraw #(.PIPE(PIPE)) u_draw (
    .rst             (rst),
    .clk             (clk),
    .draw            (sc_draw),
    .abort           (abort),
    .busy            (dr_busy),
    .active          (dr_active),
    .fmt             (fmt),
    .code            (dr_code),
    .xpos            (sc_hpos),
    .ysub            (sc_ysub),
    .tzw             (sc_tzw),
    .hflip           (sc_hflip),
    .vflip           (sc_vflip),
    .attr            (sc_attr),
    .shd             (sc_shd),
    .full_shadow_off (shd_bit5),
    .rom_unit        (rom_unit),
    .rom_req         (rom_req),
    .rom_ok          (rom_ok),
    .rom_data4       (rom_data4),
    .rom_data1       (rom_data1),
    .dline           ({4'd0, draw_line[3:0]}),
    .dline_q         (dr_dline),
    .buf_addr        (dr_x),
    .buf_we          (dr_we),
    .buf_din         (dr_din),
    .buf_shd_we      (dr_shd_we),
    .buf_shd         (dr_shd)
);

// ---------------------------------------------------------------------------
//  the dot-rate output: c18, K053247GX_combine_c18 (konamigx_v.cpp:66, p.46)
//
//      c18 = (attr & 0xff) << coregshift | coreg
//      control_w bit 18 set    -> c18 &= 0x3fff
//      else bit 19 clear       -> c18 bits 15-14 = attr bits 9-8
//
//  Every one of these registers is loaded on pxl_cen, so KonamiGX.sdc can
//  name them as dot-rate sources into gx_prio honestly.  The CPU-written
//  registers above reach them through a single-cycle path, as they must.
// ---------------------------------------------------------------------------
wire [9:0]  q_attr = lb_q[17:8];
wire [15:0] c18_sh = { 8'd0, q_attr[7:0] } << coregshift;
wire [15:0] c18_or = c18_sh | coreg;
wire [15:0] c18    = spri_sel18  ? { 2'b00, c18_or[13:0] }
                   : !spri_sel19 ? { q_attr[9:8], c18_or[13:0] }
                   :               c18_or;

always @(posedge clk) begin
    if (rst) begin
        pix_pen        <= 8'd0;
        pix_c18        <= 16'd0;
        pix_attr       <= 10'd0;
        pix_coregshift <= 4'd4;
        pix_shd        <= 2'd0;
        pix_shdm       <= 3'd0;
        pix_sdsel      <= 1'b0;
    end else if (pxl_cen) begin
        pix_pen        <= lb_q[7:0];
        pix_c18        <= c18;
        pix_attr       <= q_attr;
        pix_coregshift <= coregshift;
        pix_shd        <= sb_q[1] ? 2'd1 : sb_q[2] ? 2'd2 : sb_q[3] ? 2'd3 : 2'd0;
        pix_shdm       <= sb_q;
        pix_sdsel      <= opset[5];
    end
end

endmodule

`default_nettype wire
