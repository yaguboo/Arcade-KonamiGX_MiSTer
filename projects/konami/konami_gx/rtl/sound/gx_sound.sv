//============================================================================
//  Konami System GX -- the sound subsystem
//
//  A 68000 at 8 MHz with its own ROM, its own work RAM, two K054539 PCM chips,
//  a TMS57002 DSP and the K056800 mailbox back to the main CPU.
//
//  ---- WHY THIS IS NOT A LATE-STAGE FEATURE ---------------------------------
//  STATUS.md 2026-09-08 section 18, and root LESSONS_LEARNED L-018.  The main
//  CPU does not leave its ROM/RAM CHECK screen until the sound CPU answers,
//  and `tools/gx_readtap.lua` ranked every device window in that interval:
//  with the two controls removed, the ONLY device the main CPU reads is the
//  K056800, 245 times, at 0xd52010 = snd_to_host[0].  Two sessions were spent
//  looking for a defect in layer A that was not there.
//
//  ---- memory map, gxsndmap, gx.cpp:1184 -------------------------------------
//  Transcribed in docs/SOURCE_AUDIT.md section 5.  All of it is decoded here,
//  including the parts that are not implemented yet, because a 68000 that
//  never gets DTACK does not fault -- it stops, with the bus frozen, which
//  looks exactly like a dead clock.
//
//      000000-03ffff  R   program ROM, 256 KB          SDRAM via gx_romcache
//      100000-10ffff  RW  work RAM, 64 KB              on-chip, D10
//      200000-2004ff  RW  k054539_1 umask16(0xff00)    REGISTERS + TIMER + PCM
//                         k054539_2 umask16(0x00ff)    two chips, one window
//                                                      reverb rings in external memory (gx_sndxm)
//      300001         RW  tms57002 data                gx_tms57002 (2026-09-15)
//      400000-40001f  RW  k056800, umask16(0x00ff)     DONE
//      500000-500001  R   tms57002 status              gx_tms57002
//                     W   tms57002 control             pload / cload / reset + IRQ2 gate
//
//  (The paragraphs below describe the NOT IMPLEMENTED stage and are kept as
//  history; nothing in this map is unimplemented any more except NRES.)
//      580000-580001  W   'NRES' -- K056602 reset, nopw in MAME
//
//  ---- what NOT IMPLEMENTED means here, exactly ------------------------------
//  Reads return zero, writes are absorbed, and BOTH are acknowledged.  That is
//  a marked temporary state, not a model of the chip -- and it was not a free
//  one: answering a window with zero is itself a behaviour, and this board
//  spent two sessions stuck behind one of them.
//
//    * The TMS57002 status word is the sound CPU's MOST FREQUENT access in the
//      blocking interval -- 321,703 reads against the K054539's 2,560
//      (`tools/gx_sndcpu.lua`).  98.7 % of the values read are 0x0005.  That
//      is NOT a licence to return 0x0005.  The STATUS handoff says in as many
//      words that whether the download loop waits on `pc0` has not been
//      established, and that reasoning from a dominant value to a constant is
//      the shape of this project's last three wrong answers.  So this returns
//      ZERO, which is honestly "absent", and the next step is to MEASURE which
//      bit the loop branches on.  docs/REUSE_PLAN.md, dependency order item 3.
//
//    * The K054539 is no longer in that list.  MEASURED 2026-09-08
//      (docs/MEASUREMENTS.md 12): the sound self-test walks 00/FF/AA/55
//      through 256 registers on EACH chip and reads them back, and answering
//      that with zero hangs the boot ON ITS OWN -- forcing it in MAME, with
//      the timer left working, reproduces this board's symptom.  So the
//      register file and the timer are built (gx_k054539.sv) and the PCM
//      engine is not.  That module's header draws the line.
//
//  `dbg` below is what turns "it stopped somewhere" into "it stopped here".
//============================================================================
`default_nettype none

module gx_sound #(
    parameter integer SRAM_AW = 13      // work-RAM word-address bits: 13 = 16 KB (D30), 15 = the PCB's 64 KB
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        ce_en,        // global enable (pause / ROM download)
    input  wire        snd_run,      // control bit 22: 0 = hold in reset
    input  wire        voice_boost,  // chip 2 ch 0-3 x0.8, 4-7 x2.0: MAME's dragoonj [HACK] gains, any set (OSD; MEASUREMENTS 162)

    // --- K056800 sound side -------------------------------------------------
    //  The chip itself lives in gx_top, because its other port is on the MAIN
    //  CPU's bus.  One instance, two buses; this is the sound one.
    output wire        k56_cs,
    output wire        k56_we,
    output wire [4:1]  k56_addr,
    output wire [7:0]  k56_din,
    input  wire [7:0]  k56_dout,
    input  wire        k56_irq,      // level -> IPL 1

    // --- K056800 HOST side, for the log and nothing else (DEBUG) ------------
    //  The other port of the same chip, on the MAIN CPU's bus.  Nothing here
    //  drives it or answers it -- these are inputs because gx_sndtrace wants
    //  BOTH ends of the mailbox in one timeline, and the log lives on this
    //  side of the hierarchy.  Same clock domain as everything else here
    //  (gx_top instantiates gx_k056800 and gx_sound on `clk`), so this is a
    //  wire and not a crossing.
    input  wire        k56h_cs,
    input  wire        k56h_we,
    input  wire [4:1]  k56h_addr,
    input  wire [7:0]  k56h_din,     // the main CPU's write data, D[15:8]
    input  wire [7:0]  k56h_dout,    // what the chip is returning to it

    //  The video side's vblank, for the log's frame ticks and nothing else.
    //  A LEVEL; gx_sndtrace takes the rising edge.
    input  wire        dbg_vblank,

    // --- SDRAM client: program ROM fetch AND K054539 samples, arbiter index 3 -
    output wire [24:0] rom_addr,     // WORD address, the arbiter's unit
    output wire        rom_req,
    output wire        rom_burst,    // two words -- the sample fetcher's 4-byte blocks
    input  wire        rom_ack,
    input  wire [15:0] rom_data,
    input  wire [15:0] rom_data2,    // a burst's second word

    // --- external memory: the reverb rings and the TMS57002's RAM ------------
    //  One 64-bit word per op, held request, one-clock ack.  gx_sndxm's header
    //  has the layout; the target maps it (DECISIONS D16).
    output wire        xm_req,
    output wire        xm_we,
    output wire [21:0] xm_word,
    output wire [63:0] xm_wdata,
    output wire [7:0]  xm_be,
    input  wire        xm_ack,
    input  wire [63:0] xm_rdata,

    // --- the TMS57002 host port as a log (DEBUG, gx_sndtrace.sv) --------------
    //  Off unless trc_en; the target drains trc_data into DDR3.  trc_polls is
    //  every status read since trc_en, for a testbench to compare with MAME's.
    input  wire        trc_en,
    output wire        trc_valid,
    output wire [63:0] trc_data,
    input  wire        trc_take,
    output wire [31:0] trc_polls,

    // --- audio: both K054539s, one step per 48 kHz sample --------------------
    output reg  signed [15:0] snd_l,
    output reg  signed [15:0] snd_r,
    output reg  [2:0]  dbg_audio_ev, // pulses: [2] an engine waited for a sample fetch,
                                     // [1] the mix clipped, [0] an engine missed a sample

    // --- observability ------------------------------------------------------
    //  Bits 0..6 are ONE-CLOCK PULSES; bit 7 is a level.  gx_top decides which
    //  are latched per frame and which are sticky, the same way it does for
    //  gx_tilemap's `dbg_a_hit`, so that policy lives in one place.
    //
    //      [0] the sound CPU asserted a bus cycle          it is running
    //      [1] a program fetch was served from SDRAM       ROM path works
    //      [2] work RAM was written                        RAM path works
    //      [3] the sound CPU WROTE the K056800             it answered the host
    //      [4] the TMS57002 status word was read           reached the DSP loop
    //      [5] a K054539 register was written
    //      [6] the TMS57002 data port was written          DSP download running
    //      [7] LEVEL: a bus cycle unacknowledged for 1024 clocks
    //      [8] LEVEL: sound_ctrl bit 0 -- the K054539 timer's IRQ2 GATE
    //      [9] LEVEL: IRQ2 is asserted
    //     [10] a K054539 register was READ
    //  [14:11] the last 2 KB page OTHER THAN PAGE 0 that the CPU fetched from
    //
    //  2 KB, not 16 KB, and the size is MEASURED rather than chosen: in
    //  sim/tb_gx_sndboot.sv the sound CPU's instruction fetches span
    //  0x000000..0x00713C, about 29 KB of the 256 KB region.  Sixteen 16 KB
    //  pages would have put every interesting address -- the reset at 0x1400,
    //  the IRQ handlers at 0x18F0 and 0x193C, the counter code at 0x24B4 --
    //  in page 0 and resolved nothing.  Sixteen 2 KB pages cover 0x0000..0x7FFF
    //  exactly, which is the measured reach, and separate all four.
    //
    //  The board's sound CPU ends up in a tight loop inside gx_romcache with
    //  NO data access at all while IRQ2 sits asserted and unserviced (ninth
    //  and tenth ladder readings).  Knowing WHERE that loop is turns the
    //  question from "why" into "read the code": four bits name one of the
    //  sixteen 16 KB pages of the 256 KB program, and that page can be pulled
    //  out of dist/sndrom.hex and disassembled.
    //
    //  An earlier version of this instrument watched for 68000 EXCEPTION
    //  VECTOR fetches instead, and it was WRONG: the sound program checksums
    //  its own ROM from 0x000000 upward, so it reads the vector table AS DATA
    //  and the rung fired 54 million times in a 60-million-clock run.  A
    //  vector fetch and a data read of the same address are indistinguishable
    //  from here -- both are supervisor reads of low ROM.  Caught by a
    //  negative control in sim/tb_gx_sndboot.sv, where the sound CPU never
    //  crashes and the count should therefore have been zero.
    output reg [18:0]  dbg,
    //  a write above the kept work RAM (SRAM_AW): sticky, measured 0 everywhere
    output reg         dbg_sram_hi
);

`include "gx_rommap.svh"

// ---------------------------------------------------------------------------
//  the CPU
// ---------------------------------------------------------------------------
wire [23:1] a;
wire [15:0] cpu_dout;
reg  [15:0] cpu_din;
wire        rd, wr, uds_n, lds_n;
wire        cpu_halted_n;
reg         ack;

// IPL.  K056800 -> IRQ1 (gx.cpp:1786).  The K054539 timer -> IRQ2
// (gx.cpp:1202, gated by sound_ctrl bit 0) is dependency item 4 and is not
// here.  Written as an encoder rather than a wire so that adding IRQ2 is a
// value change and not a structural one -- the alternative is a one-line
// assign that has to remember to become a priority later.
wire       irq2 = irq2_r;            // k054539_1's timer, below
wire [2:0] ipl  = irq2    ? 3'd2 :
                  k56_irq ? 3'd1 : 3'd0;

gx_m68k #(.CLK_DIV(12)) u_cpu (
    .clk      (clk),
    .rst      (rst),
    .cpu_rst  (~snd_run),
    .ce_en    (ce_en),
    .addr     (a),
    .dout     (cpu_dout),
    .din      (cpu_din),
    .rd       (rd),
    .wr       (wr),
    .uds_n    (uds_n),
    .lds_n    (lds_n),
    .ack      (ack),
    .ipl_n    (~ipl),
    //  fx68k observability with no consumer yet, left UNCONNECTED rather than
    //  reduced into a `_unused` wire.  gx_top.sv's tail explains why that
    //  idiom is banned on this board: it is exactly what produces Quartus
    //  warning 10036, "assigned a value but never read", which is a
    //  project-owned warning and docs/WARNING_POLICY.md does not accept those.
    .fc       (),
    .iack_stb (),
    .dbg_d7   (),
    .as_n     (),
    //  NO LONGER UNCONNECTED.  fx68k HALTS on a double bus fault -- a group 0
    //  exception taken while processing one (fx68k.sv:34 HALT1_NMA, :384
    //  oHALTEDn) -- and a halted CPU and a CPU frozen on an unacknowledged
    //  cycle look identical from outside: AS asserted, no progress.  Two
    //  different faults, two different investigations, and for thirteen
    //  ladder assignments this board could tell neither from a healthy CPU,
    //  because rung 5 asked "a bus cycle THIS FRAME" and a stuck cycle
    //  answers yes forever.
    .halted_n (cpu_halted_n)
);

// ---------------------------------------------------------------------------
//  decode
// ---------------------------------------------------------------------------
//  Every region sits at a distinct multiple of 0x10000 except the ROM, which
//  is the whole first four, so decoding on a[23:16] keeps each line readable
//  against the transcription in the header.
wire [7:0] page = a[23:16];

wire sel_rom  = (a[23:18] == 6'd0);        // 000000-03ffff
wire sel_wram = (page == 8'h10);           // 100000-10ffff
wire sel_539  = (page == 8'h20);           // 200000-2004ff  (window is 0x500)
wire sel_tmsd = (page == 8'h30);           // 300001
wire sel_800  = (page == 8'h40);           // 400000-40001f
wire sel_tmsc = (page == 8'h50);           // 500000-500001
wire sel_nres = (page == 8'h58);           // 580000-580001

wire cyc = rd | wr;

// ---------------------------------------------------------------------------
//  work RAM: the PCB's window is 64 KB (100000-10ffff, D10), the program's
//  is 16 KB.  docs/DECISIONS.md D30, docs/BRAM_AUDIT.md.
// ---------------------------------------------------------------------------
//  MEASURED (gx_lane/gx_ramuse.lua, 13 GX sets x 20,000 frames, attract and
//  play): every READ and WRITE of every set falls in 100000-103fff, except
//  Rushing Heroes, which READS 104000-104dff and never writes there.  STATIC
//  (gx_sndscan.lua, 36 sets' sound ROMs): reset SSP 0x00103E00 everywhere,
//  and no absolute or immediate operand names 104000-10ffff (control: the
//  same checker finds 214-1065 per set in 100000-103fff).
//
//  So SRAM_AW word-address bits are kept (13 = 16 KB, 16 blocks instead of
//  64).  Above them: a read returns 0 -- what MAME's never-written RAM reads,
//  so Rushing Heroes sees the same value -- a write is dropped and
//  `dbg_sram_hi` latches.  SRAM_AW = 15 is the old 64 KB RAM, bit for bit.
//
//  Written exactly like gx_top's `wram`, for the reason its header gives: the
//  read address is the SAME EXPRESSION as the write address, which is the one
//  case Quartus 17.0 folds a byte-select write into an M10K byte enable.
//  Measured cost on this design: 1 M10K per 1024 bytes.
//  Root section 7 -- read the fitter's RAM summary before the ALM count.
(* ramstyle = "M10K" *) reg [15:0] sram [0:(1 << SRAM_AW) - 1];
reg [15:0] sram_q;
wire       sram_hi = (SRAM_AW < 15) && (a[15:1] >> SRAM_AW) != 15'd0;
wire       sram_we = sel_wram && wr && !sram_hi;

always @(posedge clk) begin
    if (sram_we && !uds_n) sram[a[SRAM_AW:1]][15:8] <= cpu_dout[15:8];
    if (sram_we && !lds_n) sram[a[SRAM_AW:1]][ 7:0] <= cpu_dout[ 7:0];
    sram_q <= sram[a[SRAM_AW:1]];
end

always @(posedge clk)
    if (rst)                           dbg_sram_hi <= 1'b0;
    else if (sel_wram && wr && sram_hi) dbg_sram_hi <= 1'b1;

// ---------------------------------------------------------------------------
//  program ROM, through the cache, through the arbiter
// ---------------------------------------------------------------------------
reg         own_pcm;           // arbiter client 3 belongs to the sample fetcher -- see below
wire        rc_ack;
wire [15:0] rc_q;
wire [18:1] rc_m_a;
wire        rc_m_rd;

gx_romcache #(.IDX_BITS(10), .AW(18)) u_cache (
    .clk   (clk),
    .rst   (rst),
    .c_a   (a[18:1]),
    .c_rd  (rd && sel_rom),
    .c_ack (rc_ack),
    .c_q   (rc_q),
    .m_a   (rc_m_a),
    .m_rd  (rc_m_rd),
    .m_ack (rom_ack && !own_pcm),
    .m_q   (rom_data)
);

// GX_SND_BASE is a BYTE address, because everything outside the FPGA counts in
// bytes and gx_rommap.svh says so at length.  The arbiter counts words.
wire [24:0] rc_addr = {1'b0, GX_SND_BASE[24:1]} + {7'd0, rc_m_a};

// ---------------------------------------------------------------------------
//  K056800 sound side -- LOW byte lane, umask16(0x00ff), gx.cpp:1191
// ---------------------------------------------------------------------------
//  Gated on LDS, not just on the select.  umask16(0x00ff) means the chip is
//  wired to D7-D0 only, so a word or upper-byte access does not reach it --
//  and `k56_din` takes cpu_dout[7:0], which is only the right half when the
//  low lane is the one being driven.
assign k56_cs   = sel_800 && cyc && !lds_n;
assign k56_we   = sel_800 && wr  && !lds_n;
assign k56_addr = a[4:1];
assign k56_din  = cpu_dout[7:0];

// ---------------------------------------------------------------------------
//  K054539 x2 -- register file and timer (gx_k054539), PCM engine
//  (gx_k054539_pcm) and the sample fetch (gx_k054539_fetch).  Each header says
//  what is and is not in there.
// ---------------------------------------------------------------------------
//  ONE WINDOW, TWO CHIPS, TWO LANES.  gx.cpp:1188-1189 maps the same
//  0x200000-0x2004ff at both, chip 1 umask16(0xff00) and chip 2
//  umask16(0x00ff).  So a word access reaches BOTH chips with different data,
//  and a byte access reaches one.  Gated on the strobes rather than on the
//  select alone, the same way the K056800 is: umask means the chip is wired to
//  those eight data lines and nothing else.
//
//  The register number is the byte offset halved -- a[10:1] -- because every
//  register sits two bytes apart in this window.
wire [9:0] k539_reg = a[10:1];

wire [7:0] k539_1_dout, k539_2_dout;
wire       k539_1_timer;

gx_k054539 u_k054539_1 (          // high byte, umask16(0xff00)
    .clk       (clk),
    .rst       (rst),
    .cs        (sel_539 && cyc && !uds_n),
    .we        (sel_539 && wr  && !uds_n),
    .addr      (k539_reg),
    .din       (cpu_dout[15:8]),
    .dout      (k539_1_dout),
    .timer_out (k539_1_timer)
);

//  Chip 2's timer output is left UNCONNECTED, not tied off, and that is a
//  transcription and not an oversight: konamigx.cpp:1791 sets
//  timer_handler() on m_k054539_1 ONLY.  The second chip's timer goes
//  nowhere on this board.  gx_top.sv's tail explains why an unused output is
//  left open here rather than reduced into a `_unused` wire.
gx_k054539 u_k054539_2 (          // low byte, umask16(0x00ff)
    .clk       (clk),
    .rst       (rst),
    .cs        (sel_539 && cyc && !lds_n),
    .we        (sel_539 && wr  && !lds_n),
    .addr      (k539_reg),
    .din       (cpu_dout[7:0]),
    .dout      (k539_2_dout),
    .timer_out ()
);

// ---- the PCM engines ----------------------------------------------------------
//  One sample every 2,000 clocks: k054539.cpp device_start allocates the stream
//  at clock() / 384, konamigx.cpp clocks both chips at 18.432 MHz, so 48,000 Hz,
//  and 96,000,000 / 48,000 is exactly 2,000.  Held by ce_en with the sound CPU:
//  a paused machine is silent at its last sample, not running on.
reg [10:0] smp_cnt;
reg        smp_tick;
always @(posedge clk) begin
    smp_tick <= 1'b0;
    if (rst)
        smp_cnt <= 11'd0;
    else if (ce_en) begin
        if (smp_cnt == 11'd1999) begin
            smp_cnt  <= 11'd0;
            smp_tick <= 1'b1;
        end else
            smp_cnt <= smp_cnt + 11'd1;
    end
end

//  ONE write per CPU write.  The register file above takes `we` as a level and
//  rewriting the same byte is harmless there; to an engine a repeated 0x214
//  reloads the key-on position a second time, after it may already have
//  stepped.  So the engines see the first clock of the strobe only.
wire k539_wr1 = sel_539 && wr && !uds_n;
wire k539_wr2 = sel_539 && wr && !lds_n;
reg  k539_wr1_d, k539_wr2_d;
always @(posedge clk) begin
    k539_wr1_d <= k539_wr1;
    k539_wr2_d <= k539_wr2;
end

//  ---- AND REGISTERED BEFORE THE ENGINES, 2026-09-15 ----------------------------
//  Build of 06b7ea9: fx68k rFC -> gx_m68k's strobes -> this decode -> an engine's
//  key-on copy into c_rpos missed 96 MHz by 0.955 ns (and the same bus fed the
//  TMS57002's host port at -1.665).  A 68000 bus cycle here lasts dozens of
//  clocks and MAME applies a write between two samples, so one clock more costs
//  nothing; the register file above keeps the live bus for its read-back.
reg        k539_w1_q, k539_w2_q;
reg [9:0]  k539_reg_q;
reg [15:0] k539_dout_q;
always @(posedge clk) begin
    k539_w1_q   <= k539_wr1 && !k539_wr1_d;
    k539_w2_q   <= k539_wr2 && !k539_wr2_d;
    k539_reg_q  <= k539_reg;
    k539_dout_q <= cpu_dout;
end

wire [21:0] pcm_addr1, pcm_addr2;
wire [2:0]  pcm_ch1, pcm_ch2;
wire        pcm_req1, pcm_req2, pcm_rev1, pcm_rev2;
wire [1:0]  pcm_ack;
wire [7:0]  pcm_byte;
wire [7:0]  pcm1_rd, pcm2_rd;
wire        pcm1_own, pcm2_own;
wire signed [17:0] pcm1_l, pcm1_r, pcm2_l, pcm2_r;
wire        pcm1_ovr, pcm2_ovr;
wire [1:0]  rv_req, rv_we, rv_ack;
wire [12:0] rv_slot1, rv_slot2;
wire [15:0] rv_wdata1, rv_wdata2, rv_rdata;
wire [23:0] tms_so0, tms_so1, tms_so2, tms_so3;     // the TMS57002, below
wire        tms_ovr;
wire        tms_xm_req, tms_xm_we, tms_xm_ack;
wire [19:0] tms_xm_adr;
wire [2:0]  tms_xm_n;
wire [47:0] tms_xm_wdata, tms_xm_rdata;

// ---- the external memory: both rings and the TMS57002's RAM (DECISIONS D16) ----
gx_sndxm u_xm (
    .clk          (clk),
    .rst          (rst),
    .rv_req       (rv_req),
    .rv_we        (rv_we),
    .rv_slot      ({rv_slot2, rv_slot1}),
    .rv_wdata     ({rv_wdata2, rv_wdata1}),
    .rv_ack       (rv_ack),
    .rv_rdata     (rv_rdata),
    .d_req        (tms_xm_req),
    .d_we         (tms_xm_we),
    .d_adr        (tms_xm_adr),
    .d_n          (tms_xm_n),
    .d_wdata      (tms_xm_wdata),
    .d_ack        (tms_xm_ack),
    .d_rdata      (tms_xm_rdata),
    .xm_req       (xm_req),
    .xm_we        (xm_we),
    .xm_word      (xm_word),
    .xm_wdata     (xm_wdata),
    .xm_be        (xm_be),
    .xm_ack       (xm_ack),
    .xm_rdata     (xm_rdata),
    .dbg_clearing ()
);

gx_k054539_pcm u_pcm_1 (          // high byte
    .clk         (clk),
    .rst         (rst),
    .tick        (smp_tick),
    .wr          (k539_w1_q),
    .wr_addr     (k539_reg_q),
    .wr_data     (k539_dout_q[15:8]),
    .rd_addr     (k539_reg),
    .rd_data     (pcm1_rd),
    .rd_own      (pcm1_own),
    .rom_req     (pcm_req1),
    .rom_addr    (pcm_addr1),
    .rom_ch      (pcm_ch1),
    .rom_rev     (pcm_rev1),
    .rom_ack     (pcm_ack[0]),
    .rom_data    (pcm_byte),
    .rv_req      (rv_req[0]),
    .rv_we       (rv_we[0]),
    .rv_slot     (rv_slot1),
    .rv_wdata    (rv_wdata1),
    .rv_ack      (rv_ack[0]),
    .rv_rdata    (rv_rdata),
    .out_l       (pcm1_l),
    .out_r       (pcm1_r),
    .dbg_overrun (pcm1_ovr),
    .ch_gain     (16'h0000)
);

gx_k054539_pcm u_pcm_2 (          // low byte
    .clk         (clk),
    .rst         (rst),
    .tick        (smp_tick),
    .wr          (k539_w2_q),
    .wr_addr     (k539_reg_q),
    .wr_data     (k539_dout_q[7:0]),
    .rd_addr     (k539_reg),
    .rd_data     (pcm2_rd),
    .rd_own      (pcm2_own),
    .rom_req     (pcm_req2),
    .rom_addr    (pcm_addr2),
    .rom_ch      (pcm_ch2),
    .rom_rev     (pcm_rev2),
    .rom_ack     (pcm_ack[1]),
    .rom_data    (pcm_byte),
    .rv_req      (rv_req[1]),
    .rv_we       (rv_we[1]),
    .rv_slot     (rv_slot2),
    .rv_wdata    (rv_wdata2),
    .rv_ack      (rv_ack[1]),
    .rv_rdata    (rv_rdata),
    .out_l       (pcm2_l),
    .out_r       (pcm2_r),
    .dbg_overrun (pcm2_ovr),
    //  konamigx.cpp:4584-4591: chip 2 channels 0-3 x0.8, 4-7 x2.0.  EMULATION_DERIVED
    //  (MAME: "[HACK] This shouldn't be necessary"); the user reports the PCB's
    //  effects louder than MAME's model, which this core reproduces (162).
    .ch_gain     (voice_boost ? 16'hAA55 : 16'h0000)
);

wire [24:0] pf_addr;
wire        pf_req;
wire [15:0] pf_demand;

gx_k054539_fetch #(.BASE_W({1'b0, GX_PCM_BASE[24:1]})) u_pcm_fetch (
    .clk        (clk),
    .rst        (rst),
    .e_req      ({pcm_req2,  pcm_req1}),
    .e_addr     ({pcm_addr2, pcm_addr1}),
    .e_ch       ({pcm_ch2,   pcm_ch1}),
    .e_rev      ({pcm_rev2,  pcm_rev1}),
    .e_ack      (pcm_ack),
    .e_data     (pcm_byte),
    .m_addr     (pf_addr),
    .m_req      (pf_req),
    .m_ack      (rom_ack && own_pcm),
    .m_q        (rom_data),
    .m_q2       (rom_data2),
    .dbg_demand (pf_demand),
    .dbg_ahead  ()
);

//  ONE ARBITER CLIENT, TWO USERS.  The program cache and the sample fetcher
//  share index 3 rather than adding a sixth client: gx_top's scheduling of
//  index 3 -- the tile-group gate, the urgency token -- was argued and measured
//  for exactly this kind of soft-deadline reader, and a sixth index would need
//  that argument again.  The program cache asks rarely once the sound CPU is
//  running (gx_top.sv, "asks rarely").
//
//  `own_pcm` changes only in a clock where the current owner is NOT asking, so
//  a request is never handed from one user to the other while the arbiter may
//  be serving it; both hold their request until their ack and drop it after.
always @(posedge clk) begin
    if (rst)
        own_pcm <= 1'b0;
    else if (!(own_pcm ? pf_req : rc_m_rd) && (own_pcm ? rc_m_rd : pf_req))
        own_pcm <= ~own_pcm;
end

assign rom_addr  = own_pcm ? pf_addr : rc_addr;
assign rom_req   = own_pcm ? pf_req  : rc_m_rd;
assign rom_burst = own_pcm;

//  THE MIX.  konamigx.cpp routes each chip's output 0 to the left speaker and 1
//  to the right at 1.0 (docs/SOURCE_AUDIT.md 14), and the TMS57002's outputs 0
//  and 2 to the left, 1 and 3 to the right, at 0.3 (:1781-1784).  A DSP output
//  reaches the speaker as s32(so << 8) / 2^31 (tms57002.cpp:933), i.e. so / 2^23
//  of full scale, so in this module's 32768 scale it is 0.3 * so / 256.
//  0.3 is 19661 / 65536 here (0.30000305).  Clipped where a 16-bit output clips.
wire signed [24:0] tms_sl = $signed({tms_so0[23], tms_so0}) + $signed({tms_so2[23], tms_so2});
wire signed [24:0] tms_sr = $signed({tms_so1[23], tms_so1}) + $signed({tms_so3[23], tms_so3});
reg  signed [41:0] tms_ml, tms_mr;
always @(posedge clk) begin
    tms_ml <= tms_sl * $signed(17'd19661);
    tms_mr <= tms_sr * $signed(17'd19661);
end
wire signed [16:0] tms_l = 17'(tms_ml >>> 24);
wire signed [16:0] tms_r = 17'(tms_mr >>> 24);
wire signed [19:0] mix_l = $signed({{2{pcm1_l[17]}}, pcm1_l}) + $signed({{2{pcm2_l[17]}}, pcm2_l}) +
                           $signed({{3{tms_l[16]}}, tms_l});
wire signed [19:0] mix_r = $signed({{2{pcm1_r[17]}}, pcm1_r}) + $signed({{2{pcm2_r[17]}}, pcm2_r}) +
                           $signed({{3{tms_r[16]}}, tms_r});

//  dbg_audio_ev feeds gx_top's AUDIO band on the board's overlay, because nobody
//  on the bench can listen and the rung-1 comparison says nothing about the
//  board.  Registered pulses; gx_top latches them a frame at a time.
reg [15:0] pf_demand_d;

//  ONE INSTANT PER SAMPLE.  At a tick the DSP's `so` becomes MAME's output for
//  the sample the engines' output registers still hold (they change later in
//  the sample period, at ST_OUT); tms_ml is registered one clock after that.
//  So the speaker takes both two clocks after the tick, and holds them for the
//  sample (gx_tms57002 header, tools/tmsgold's per-sample order).
reg smp_d1, smp_d2;
always @(posedge clk) begin
    smp_d1 <= smp_tick;
    smp_d2 <= smp_d1;
end

always @(posedge clk) begin
    if (smp_d2) begin
        snd_l <= (mix_l > 20'sd32767) ? 16'sd32767 : (mix_l < -20'sd32768) ? $signed(16'h8000) : mix_l[15:0];
        snd_r <= (mix_r > 20'sd32767) ? 16'sd32767 : (mix_r < -20'sd32768) ? $signed(16'h8000) : mix_r[15:0];
    end
    pf_demand_d  <= pf_demand;
    dbg_audio_ev <= { pf_demand != pf_demand_d,
                      (mix_l > 20'sd32767) || (mix_l < -20'sd32768) ||
                      (mix_r > 20'sd32767) || (mix_r < -20'sd32768),
                      pcm1_ovr || pcm2_ovr || tms_ovr };
end

// ---------------------------------------------------------------------------
//  sound_ctrl (0x500000) and IRQ2
// ---------------------------------------------------------------------------
//  konamigx.cpp:1168 tms57002_control_word_w, ACCESSING_BITS_0_7 -- the low
//  byte, so this is gated on LDS.  Only bit 0 is read here; bits 2, 3 and 4
//  are the TMS57002's pload, cload and reset and arrive with REUSE_PLAN item
//  3b.  The whole byte is STORED anyway, so that adding them is a read and
//  not a re-wiring.
//
//  IRQ2 is a SET/CLEAR pair, transcribed from two different places in MAME
//  rather than invented as one expression:
//
//      konamigx.cpp:1171  a write with bit 0 CLEAR  ->  CLEAR_LINE
//      konamigx.cpp:1202  k054539_irq_gen: if (m_sound_ctrl & 1) and the
//                         timer output goes 0 -> 1, ASSERT_LINE
//
//  Note which way round the gate works: the ENABLE is sampled at the edge, so
//  an edge arriving while bit 0 is clear is LOST rather than queued -- the
//  same shape as the K056800's interrupt guard, and worth stating because the
//  two chips' guards look alike and only one of them latches.
//
//  MEASURED (docs/MEASUREMENTS.md 11): the sound program acknowledges by
//  writing 0xFE and re-enables with 0xFF, 2397 and 2693 times in 760 frames,
//  against 2399 entries to the IRQ2 handler.  One acknowledge per entry.
reg  [7:0] sound_ctrl;
reg        irq2_r;
reg        timer_d;

always @(posedge clk) begin
    if (rst) begin
        sound_ctrl <= 8'd0;
        irq2_r     <= 1'b0;
        timer_d    <= 1'b0;
    end else begin
        timer_d <= k539_1_timer;

        if (sel_tmsc && wr && !lds_n) begin
            sound_ctrl <= cpu_dout[7:0];
            if (!cpu_dout[0]) irq2_r <= 1'b0;
        end

        // Rising edge, gated.  Placed after the write so that an acknowledge
        // and an edge landing on the same clock leave the line ASSERTED --
        // MAME's two callbacks cannot collide, ours can, and dropping the
        // edge would lose an interrupt the program is counting on.
        if (sound_ctrl[0] && k539_1_timer && !timer_d) irq2_r <= 1'b1;

        // konamigx.cpp:548-550: the main CPU's control bit 22 low zeroes
        // m_sound_ctrl as it halts the sound CPU and resets the DSP.  The sound
        // CPU is held in reset meanwhile, so nothing can write it back.
        if (!snd_run) sound_ctrl <= 8'd0;
    end
end

// ---------------------------------------------------------------------------
//  the TMS57002 (DECISIONS D16, MEASUREMENTS 45)
// ---------------------------------------------------------------------------
//  Host side, konamigx.cpp:1161-1192: 0x300001 is the data port (low byte),
//  0x500000's low byte is pload (bit 2), cload (bit 3) and reset (bit 4, 1 =
//  run).  RESET is also asserted while control bit 22 holds the sound CPU
//  (:537-557): bit 22 releasing does not release the DSP unless bit 4 is set,
//  and bit 22 low zeroes the byte (above), so the line is simply
//  !(snd_run && sound_ctrl[4]).
//
//  Inputs, :1789-1802 and tms57002.cpp:927-931: each K054539 output reaches the
//  DSP at 0.5 and is scaled by 32768 * 256 (ST0 SIM set by this game's
//  program), so the 24-bit input is exactly trunc(lval) * 128 masked to 24 bits
//  -- lval[16:0] followed by seven zeros.  The engines' outputs are already
//  truncated (gx_k054539_pcm).  in 0/1 chip 1 left/right, in 2/3 chip 2.
wire [7:0]  tms_dout;
wire [2:0]  tms_status;
//  P2 observability (MEASUREMENTS 72): what the DSP's host port did, and
//  whether it did it inside the coefficient window.  Debug only.
wire        tms_s_host, tms_in_cload;
wire [2:0]  tms_hidx;
wire        tms_rst_line = !(snd_run && sound_ctrl[4]);
wire        tmsd_wr = sel_tmsd && wr && !lds_n;
wire        tmsd_rd = sel_tmsd && rd && !lds_n;
reg         tmsd_wr_d, tmsd_rd_d;
//  The strobes and the byte go to the DSP one clock later, registered -- the
//  same fx68k-bus timing reason as the K054539 writes above (pc / ca / cm_wd
//  at -1.665 / -0.999 / -0.842 ns in 06b7ea9's build).  A data READ therefore
//  latches its byte one clock later too, and its acknowledge waits for it (the
//  read mux below).
reg         tms_dwr_q, tms_drd_q;
reg  [7:0]  tms_din_q;
always @(posedge clk) begin
    tmsd_wr_d <= tmsd_wr;
    tmsd_rd_d <= tmsd_rd;
    tms_dwr_q <= tmsd_wr && !tmsd_wr_d;
    tms_drd_q <= tmsd_rd && !tmsd_rd_d;
    tms_din_q <= cpu_dout[7:0];
end

gx_tms57002 #(.CYCLES(250), .START_DELAY(1)) u_tms (
    .clk         (clk),
    .rst         (rst),
    .tick        (smp_tick),
    .si0         ({pcm1_l[16:0], 7'd0}),
    .si1         ({pcm1_r[16:0], 7'd0}),
    .si2         ({pcm2_l[16:0], 7'd0}),
    .si3         ({pcm2_r[16:0], 7'd0}),
    .so0         (tms_so0),
    .so1         (tms_so1),
    .so2         (tms_so2),
    .so3         (tms_so3),
    .data_wr     (tms_dwr_q),                 // one op per CPU write, as the engines
    .data_in     (tms_din_q),
    .data_rd     (tms_drd_q),                 // data_r's side effects once per read
    .data_out    (tms_dout),
    .ctrl_pload  (sound_ctrl[2]),
    .ctrl_cload  (sound_ctrl[3]),
    .reset_line  (tms_rst_line),
    .status      (tms_status),
    .xm_req      (tms_xm_req),
    .xm_we       (tms_xm_we),
    .xm_adr      (tms_xm_adr),
    .xm_n        (tms_xm_n),
    .xm_wdata    (tms_xm_wdata),
    .xm_ack      (tms_xm_ack),
    .xm_rdata    (tms_xm_rdata),
    .dbg_overrun (tms_ovr),
    .dbg_s_host   (tms_s_host),
    .dbg_hidx     (tms_hidx),
    .dbg_in_cload (tms_in_cload)
);

// ---------------------------------------------------------------------------
//  read mux and acknowledge
// ---------------------------------------------------------------------------
//  EVERY address acknowledges -- the three unimplemented devices, and also the
//  UNDECODED ones.  A stray access must not be able to wedge the machine while
//  the map is still being filled in, and `dbg[7]` is what says a cycle went
//  unanswered anyway.
//
//  ---- and for two sessions ONE address class did not, 2026-09-09 ----------
//  A WRITE to the ROM region.  `ack` routed the whole of `sel_rom` to the
//  cache, and the cache is wired `.c_rd (rd && sel_rom)` -- reads only.  So a
//  write anywhere in 0x000000..0x03FFFF asked the cache for an acknowledge it
//  has no way to give, `c_ack` never fired, and the 68000 sat on that cycle
//  FOREVER: AS asserted, no fault, no progress.  256 KB, the largest region in
//  the map, and the exact hole the paragraph above says must not exist.
//
//  MEASURED, sim/tb_gx_sndboot.sv with +corrupt=2000 -- a wrong fetched word
//  once in every 2000 SDRAM reads:
//
//      [209576307] STUCK: addr 00714e rd 0 wr 1 sel_rom 1 ack 0
//                  romcache st 0 (S_IDLE)  c_rd 0  c_ack 0
//                  halted_n 1        <- NOT a double bus fault.  Just stuck.
//
//  and the run then held `cyc` high for 390 million clocks with dbg[7] latched
//  and not one further fetch.
//
//  WHY IT MATTERS BEYOND BEING WRONG.  Executing from address 0 -- which is
//  where every zero vector in this program leads -- the FIRST instruction is
//  `ORI.B #$0,(A0)`, a read-modify-WRITE through whatever A0 happens to hold.
//  If A0 points into ROM space the machine freezes on the spot instead of
//  sliding on through the vector table; if it does not, it loops.  That is one
//  coin flip, taken once per crash, and the board has shown BOTH faces of it
//  -- the same bitstream resting at page 0 on one run and page 3 on the next
//  (STATUS 16, reading 10).
//
//  A real PCB does not behave this way: DTACK there comes from the address
//  decoder, which does not look at R/W, so writing at a ROM chip is absorbed
//  and the cycle completes.  Absorbing it here is the hardware-accurate
//  answer as well as the safe one.
reg dev_ack;
wire rom_rd = sel_rom && rd;          // the cache answers READS, and only reads
//  A TMS57002 data read acknowledges two clocks later than any other device:
//  its strobe is registered, the DSP latches the byte on it, and the byte is on
//  data_out the clock after that (see "the TMS57002" above).  cyc_age counts the
//  clocks of the current cycle.
reg [1:0] cyc_age;
always @(posedge clk) begin
    if (rst || !cyc)            cyc_age <= 2'd0;
    else if (cyc_age != 2'd3)   cyc_age <= cyc_age + 2'd1;
end
always @(posedge clk) begin
    if (rst) dev_ack <= 1'b0;
    else     dev_ack <= cyc && !rom_rd && !dev_ack && (!tmsd_rd || cyc_age >= 2'd2);
end

// ---------------------------------------------------------------------------
//  the TMS57002 host port, logged (DEBUG)
// ---------------------------------------------------------------------------
//  A read is logged on the FIRST dev_ack of its bus cycle.  `arm` is set while
//  no cycle is running and cleared by the first acknowledge, so a second pulse
//  in the same cycle -- dev_ack re-arms itself every other clock while `cyc`
//  stays high -- cannot count a poll twice.  Writes are edge-detected the same
//  way the DSP's own strobes are (tms_dwr_q above, and sound_ctrl's below).
reg trc_arm, trc_cw_d;
always @(posedge clk) begin
    if (!cyc)        trc_arm <= 1'b1;
    else if (dev_ack) trc_arm <= 1'b0;
    trc_cw_d <= sel_tmsc && wr && !lds_n;
end
wire trc_first = cyc && dev_ack && trc_arm;

// ---- the K056800, both ports (P1, docs/NEXT_SESSION_PROMPT.md) --------------
//  SOUND side: the same first-acknowledge rule as everything else on this bus,
//  and for the same reason -- dev_ack re-arms while `cyc` stays high, so a
//  mailbox access counted twice would make the frame count wrong by a factor
//  nobody could see.  Both directions come through here: a write is
//  acknowledged by the same dev_ack a read is.
wire       trc_k56s     = trc_first && sel_800 && !lds_n;
wire [7:0] trc_k56s_val = wr ? cpu_dout[7:0] : k56_dout;

//  HOST side: this module does not own that bus and gets no acknowledge from
//  it, so the access has to be found from the select alone.  It is logged when
//  it ENDS -- the select falls, or the address or direction changes while it
//  stays high, which is what catches two accesses the 68EC020 runs back to back
//  without dropping `bus_active`.
//
//  THE END AND NOT THE START, and the delayed copies and not the live signals.
//  gx_k056800 does `h2s[h_r] <= h_din` on EVERY clock the select and the write
//  strobe are up, so the byte the chip KEEPS is the one present on the last of
//  them -- which is what these registers hold when the access ends.  Logging
//  the start instead would report whatever the bus happened to be showing
//  during the address phase.
//
//  This was wrong once and the simulator caught it before the board did: the
//  first version emitted the word two clocks after the select rose, which is
//  safe for the 68EC020's twelve-clock device cycle but samples the NEXT write
//  in sim/tb_gx_sndboot, whose host task drives the port for one clock.  It
//  logged the 0xFE command as 0x00.
//  And the STROBE leaves this module registered.  The select is
//  `bus_active && a[23:5] == ...` in gx_main, so anything combinational from it
//  drags the main CPU's address across the die into the log's write logic --
//  which is exactly what failed timing at -0.351 ns on 2026-09-21, every worst
//  path sourced at cpu_a32.  Here the address ends at a flop; gx_sndtrace's own
//  input stage catches the rest.
reg        k56h_cs_d, k56h_we_d, k56h_we_d2;
reg  [4:1] k56h_addr_d, k56h_addr_d2;
reg  [7:0] k56h_din_d, k56h_dout_d;
reg        k56h_ev;
reg  [7:0] k56h_val_q;
always @(posedge clk) begin
    k56h_cs_d    <= k56h_cs;
    k56h_we_d    <= k56h_we;
    k56h_addr_d  <= k56h_addr;
    k56h_din_d   <= k56h_din;
    k56h_dout_d  <= k56h_dout;
    k56h_we_d2   <= k56h_we_d;
    k56h_addr_d2 <= k56h_addr_d;
    //  the access ENDED between the previous clock and this one
    k56h_ev      <= k56h_cs_d && (!k56h_cs || k56h_addr != k56h_addr_d
                                            || k56h_we   != k56h_we_d);
    //  the byte the chip kept, from the same clock the strobe refers to
    k56h_val_q   <= k56h_we_d ? k56h_din_d : k56h_dout_d;
end

gx_sndtrace u_trace (
    .clk       (clk),
    .rst       (rst),
    .en        (trc_en),
    .ev_st     (trc_first && sel_tmsc && rd),
    .st_val    (tms_status),
    .ev_dr     (trc_first && sel_tmsd && rd && !lds_n),
    .dr_val    (tms_dout),
    .ev_dw     (tms_dwr_q),
    .dw_val    (tms_din_q),
    .ev_cw     (sel_tmsc && wr && !lds_n && !trc_cw_d),
    .cw_val    (cpu_dout[7:0]),
    .smp_tick  (smp_tick),
    .ovr       (tms_ovr),
    .snd_run   (snd_run),
    .ev_k56s   (trc_k56s),
    .k56s_val  (trc_k56s_val),
    .k56s_addr (a[4:1]),
    .k56s_we   (wr),
    .ev_k56h   (k56h_ev),
    .k56h_val  (k56h_val_q),
    .k56h_addr (k56h_addr_d2),
    .k56h_we   (k56h_we_d2),
    .vbl       (dbg_vblank),
    .dsp_s_host   (tms_s_host),
    .dsp_hidx     (tms_hidx),
    .dsp_in_cload (tms_in_cload),
    .trc_valid (trc_valid),
    .trc_data  (trc_data),
    .trc_take  (trc_take),
    .dbg_polls (trc_polls)
);

always @(*) begin
    ack = rom_rd ? rc_ack : dev_ack;
    if      (sel_rom)  cpu_din = rc_q;
    else if (sel_wram) cpu_din = sram_hi ? 16'd0 : sram_q;
    else if (sel_800)  cpu_din = {8'd0, k56_dout};
    //  0x22c and the channel positions are the ENGINE's: it keys channels off
    //  and moves positions, and k054539.cpp's read returns those same m_regs.
    else if (sel_539)  cpu_din = {pcm1_own ? pcm1_rd : k539_1_dout,
                                  pcm2_own ? pcm2_rd : k539_2_dout};
    //  The three unimplemented devices and the NRES port are named rather than
    //  folded into the default.  It is not decoration: naming them is what
    //  READS the four select wires, and an assigned-but-never-read wire is
    //  Quartus warning 10036 (gx_top.sv's tail).  It also means the day one of
    //  them becomes real, the line to change is already here.
    //  ---- the TMS57002 status word, MEASURED rather than guessed -----------
    //  konamigx.cpp:1161 assembles it from three DSP flags:
    //
    //      bit 2  dready   the DSP has a result waiting to be read
    //      bit 1  pc0      the DSP's program counter is at zero
    //      bit 0  empty    the DSP has consumed what was written to it
    //
    //  This core has no TMS57002.  What it returns here decides whether the
    //  sound program's DSP download loop terminates, and THAT decides the
    //  whole boot -- so it is worth saying how the value below was arrived at,
    //  because two earlier sessions got this wrong in opposite directions.
    //
    //  WRONG ONCE: `tools/gx_sndcpu.lua` found that 98.7 % of the values MAME
    //  returns in the blocking window are 0x0005 (dready | empty).  Reasoning
    //  from a dominant value to a constant is the shape of this project's
    //  worst answers, and this file's header said so and refused to do it.
    //  It was right to refuse.  MEASURED in sim/tb_gx_sndboot.sv: returning
    //  0x0005 does NOT terminate the loop -- 85,636 status reads and climbing,
    //  with every other counter frozen.
    //
    //  WRONG TWICE, and the more expensive one: the sixth session read a
    //  STICKY hardware rung saying "the sound CPU wrote a K054539" and
    //  concluded the download loop had been passed.  It had not.  The sound
    //  CPU writes 146 K054539 registers, THEN enters this loop and never
    //  leaves.  A sticky rung says "at least once", never "and then it
    //  continued" -- root docs/LESSONS_LEARNED.md L-018, which was written
    //  about this exact mistake one session earlier.
    //
    //  MEASURED, sim/tb_gx_sndboot.sv running the REAL 256 KB sound program
    //  against the real gx_sound, at 40 million clocks:
    //
    //      status   TMS reads   mailbox writes   int enabled   verdict
    //      0x0000     462,406         4              no        SPINS
    //      0x0002     168,288         4              no        SPINS
    //      0x0004      85,636         4              no        SPINS
    //      0x0005      85,636         4              no        SPINS
    //      0x0007      85,636         4              no        SPINS
    //      0x0001         198       406             YES        PROCEEDS
    //
    //  So the loop branches on BIT 0, and setting `dready` actively blocks it:
    //  the program takes the "there is a result to read" path and then waits
    //  on something that never comes.  One bit, and the popular value has it
    //  wrong in company.
    //
    //  WHAT 0x0001 MEANS PHYSICALLY, because "the value that works" is not a
    //  reason.  It is the status of an ABSENT DSP modelled as a sink: always
    //  `empty`, so it has consumed whatever was written and is ready for more;
    //  never `dready`, so it has nothing to hand back; `pc0` low, so it is not
    //  claiming to be at the start of a program it does not have.  That is a
    //  coherent description of a device that is not there, which is exactly
    //  what this core has, and it is why this is honest rather than tuned.
    //
    //  ---- 2026-09-15: THE DSP IS THERE NOW, and it answers for itself ------
    //  The constant 0x0001 above was an absent DSP modelled as a sink, and the
    //  sound program's download loop passed on it.  gx_tms57002 now drives all
    //  three bits from its own state, as konamigx.cpp:1161 reads MAME's:
    //  dready = !S_HOST, pc0 = (pc != 0), empty = (update queue empty).
    //  tb_gx_sndboot is the check that the loop still passes on a real one.
    else if (sel_tmsc) cpu_din = {13'd0, tms_status};
    //  The data port returns the byte data_r returned, latched on the strobe
    //  that applied its side effects.  NRES stays at zero (nopw in MAME).
    else if (sel_tmsd) cpu_din = {8'd0, tms_dout};
    else if (sel_nres) cpu_din = 16'd0;
    else               cpu_din = 16'd0;   // undecoded
end

// ---------------------------------------------------------------------------
//  observability
// ---------------------------------------------------------------------------
//  L-018 rule 5: say WHEN each rung can FIRST light, because a rung that
//  lights before the interesting moment proves the instrument and not the
//  program.  This board has already shipped three green squares that meant
//  nothing for exactly that reason.
//
//      [0] first bus cycle after reset            microseconds -- a control
//      [1] first cache miss completing            first instruction fetch
//      [2] first work RAM write                   early; the CPU clears RAM
//      [3] first K056800 write BY THE SOUND CPU   <- the event the host polls
//      [4] first TMS57002 status read             after the mailbox is up
//      [5] first K054539 write                    during the self-test
//      [6] first TMS57002 data write              the DSP program download
//
//  [0], [1] and [2] are controls: they say the CPU is alive and they light
//  long before anything interesting.  [3] is the one that matters -- it is the
//  same event the main CPU has been polling at 0xd52010 for 145 seconds.
reg [10:0] stall_cnt;


always @(posedge clk) begin
    if (rst) begin
        dbg       <= 19'd0;
        stall_cnt <= 11'd0;
    end else begin
        dbg[6:0] <= 7'd0;
        dbg[10]  <= 1'b0;
        dbg[16]  <= 1'b0;
        dbg[8]   <= sound_ctrl[0];
        dbg[9]   <= irq2_r;
        if (cyc)                dbg[0] <= 1'b1;
        if (rc_m_rd && rom_ack && !own_pcm) dbg[1] <= 1'b1;
        if (sel_wram && wr)     dbg[2] <= 1'b1;
        if (sel_800  && wr)     dbg[3] <= 1'b1;
        if (sel_tmsc && rd)     dbg[4] <= 1'b1;
        if (sel_539  && wr)     dbg[5] <= 1'b1;
        //  The READ side of the same window.  gx_k054539 echoes, so a program
        //  spinning on a read is waiting for a value only the PCM engine could
        //  change -- and that is the difference between "the register file is
        //  wrong" and "the chip is missing".
        if (sel_539  && rd)     dbg[10] <= 1'b1;

        //  ---- exception vectors, 68000 ----------------------------------
        //  The last ROM page the CPU fetched from.  a[17:14] is the 16 KB
        //  page inside the 256 KB program.  A LEVEL, updated on every fetch,
        //  so what it holds at any moment is where the CPU is -- which for a
        //  tight loop is a constant, and that constant is the answer.
        //  THE LAST 2 KB PAGE THAT WAS NOT PAGE 0.
        //
        //  MEASURED on the board (twelfth ladder): the sound CPU ends up
        //  looping in page 0, bytes 0x0000..0x07FF.  The sound program's
        //  vector table is at 0x000..0x0FF and its exception vectors 4..23 are
        //  ALL ZERO -- so an illegal instruction, a line-F, a privilege
        //  violation or a trace jumps to address 0 and the CPU then EXECUTES
        //  ITS OWN VECTOR TABLE.  That is the loop, and it is a crash.
        //
        //  Once crashed it never leaves page 0, so the last page it fetched
        //  from BEFORE that is the crash site -- frozen for free, with no
        //  edge detection and nothing to arm.  During normal running this just
        //  follows the CPU around, which is what makes it readable in
        //  simulation as a control.
        if (sel_rom && rd && (a[14:11] != 4'd0)) dbg[14:11] <= a[14:11];
        if (sel_tmsd && wr)     dbg[6] <= 1'b1;

        //  ---- the two rungs that separate the three ways to stop ---------
        //  [15] the CPU is HALTED.  A LEVEL: fx68k leaves it asserted.
        //  [16] a bus cycle COMPLETED this clock.  Not "a cycle happened" --
        //       COMPLETED.  A frozen cycle and a running CPU both assert
        //       `cyc` forever; only one of them ever gets an acknowledge, and
        //       that is the whole difference L-018 rule 5 keeps asking for.
        dbg[15] <= ~cpu_halted_n;
        dbg[16] <= cyc && ack;

        //  ---- and WHERE it is fetching, RIGHT NOW ------------------------
        //  NOT from dbg[14:11].  That latch is written only when the page is
        //  NOT zero, so it can never read 0 after the first fetch outside
        //  page 0 -- which happens at reset, since the reset PC is 0x1400.
        //  A rung built on `dbg[14:11] == 0` would be RED for the life of the
        //  machine and would mean nothing, which is the same defect as
        //  a5bc82c ("rung 6 checked D98000, not the palette -- it could never
        //  light").  Caught here before it reached a board.
        //
        //  Page 0 is bytes 0x0000..0x07FF of the 256 KB region, so the test
        //  is a[17:11], not a[14:11]: after a crash the PC can be anywhere in
        //  the region, and a[14:11] alone would call 0x8000 "page 0" too.
        dbg[17] <= sel_rom && rd && (a[17:11] == 7'd0);
        dbg[18] <= sel_rom && rd && (a[17:11] != 7'd0);

        // A LEVEL, not a pulse: a cycle up for 1024 clocks with no
        // acknowledge.  1024 clocks is ~10.7 us = 85 sound-CPU clocks -- far
        // longer than the slowest legitimate access (a cache miss queued
        // behind three higher-priority arbiter clients) and far shorter than
        // anything a person could see.
        if (!cyc || ack)         stall_cnt <= 11'd0;
        else if (!stall_cnt[10]) stall_cnt <= stall_cnt + 11'd1;
        dbg[7] <= stall_cnt[10];
    end
end

endmodule

`default_nettype wire
