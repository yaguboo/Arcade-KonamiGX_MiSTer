//============================================================================
//  Konami System GX -- 68000 wrapper around fx68k, for the SOUND CPU
//
//  ---- this is a sibling lane's file -----------------------------------------
//  Copied from projects/vsystem/power_spikes/rtl/cpu/ps_m68k.sv, which is on
//  MiSTer hardware and which itself came from na1_na2/rtl/cpu/na2_m68k.sv.
//  Root CLAUDE.md section 1.2 shelf 3, and the factory memory
//  `check-sibling-lane-first`: this board's 68000 needs the same two things
//  the other two boards needed and got wrong first.
//
//    * fx68k's external data bus is captured on a LATER enabled Phi2 edge than
//      the clock in which `ack` was returned, so the acknowledged word has to
//      be HELD.  `din_hold` does that.
//    * an interrupt acknowledge cycle is FC = 111, and it must not be turned
//      into a bus request -- `cyc` excludes it and `vpa_n` autovectors it.
//
//  What changed for this board, and it is one number:
//
//      CLK_DIV = 12        96 MHz / 12 = 8.000 MHz
//
//  8 MHz is SUB_CLOCK/2, gx.cpp:1723, transcribed in docs/SOURCE_AUDIT.md
//  section 3.  Power Spikes divided 40 by 4 for 10 MHz; NA-1/NA-2 divided
//  differently again.  The divider is the whole difference.
//
//  ---- autovector, and the source that says so -------------------------------
//  gx.cpp wires both of the sound CPU's interrupts with `set_input_line`
//  (K056800's int_callback -> M68K_IRQ_1 at gx.cpp:1786, and k054539_1's timer
//  -> IRQ2 at gx.cpp:1202) and supplies NO vector with either.  That is
//  autovector, so VPAn is asserted for every interrupt acknowledge and the
//  board needs no vector generator.  Same conclusion Power Spikes reached from
//  `irq1_line_hold`.
//
//  fx68k (third_party/cpu/fx68k/) is (c) 2018,2021 Jorge Cwik, GPLv3.  This board
//  vendored it for the sound CPU only -- the MAIN CPU is a 68EC020 and is
//  TG68K, which is VHDL and cannot be simulated here (project CLAUDE.md 2.3).
//  So this is the ONE CPU on this board that a testbench can reach.
//============================================================================
`default_nettype none

module gx_m68k #(
    // clk_sys ticks per 68000 clock.  96 MHz / 12 = 8.000 MHz, the sound
    // CPU's rate at gx.cpp:1723.  MUST be even: the two phase enables are
    // half a CPU clock apart.
    parameter int CLK_DIV = 12
) (
    input  wire        clk,
    input  wire        rst,          // synchronous, active high
    input  wire        cpu_rst,      // hold the 68000 in reset (download, NRES)
    input  wire        ce_en,        // global clock enable (pause / download)

    // --- simple synchronous bus -------------------------------------------
    output wire [23:1] addr,
    output wire [15:0] dout,         // 68000 -> system
    input  wire [15:0] din,          // system -> 68000
    output wire        rd,
    output wire        wr,
    output wire        uds_n,
    output wire        lds_n,
    input  wire        ack,          // one-cycle pulse: din valid / write taken

    // --- interrupts --------------------------------------------------------
    input  wire [2:0]  ipl_n,

    // --- observability -----------------------------------------------------
    output wire [2:0]  fc,
    output wire        iack_stb,   // interrupt acknowledge cycle
    output wire [31:0] dbg_d7,
    output wire        as_n,
    output wire        halted_n
);

  // ---------------------------------------------------------------------
  // Two-phase clock enables: one enPhi1 and one enPhi2 per 68000 clock,
  // half a clock apart.
  // ---------------------------------------------------------------------
  //  The three constants are SIZED.  ps_m68k compares the counter against
  //  unsized integers, which is correct but lints as a 32-bit-versus-4-bit
  //  compare, and this factory's bar is zero project-owned warnings.  The
  //  divergence is here and not in the sibling: root section 14 -- a rule
  //  change audits an existing core, it does not silently repair it.
  localparam int HALF = CLK_DIV / 2;
  localparam int DIVW = $clog2(CLK_DIV);

  localparam [DIVW-1:0] D_ZERO = {DIVW{1'b0}};
  localparam [DIVW-1:0] D_HALF = DIVW'(HALF);
  localparam [DIVW-1:0] D_LAST = DIVW'(CLK_DIV - 1);

  reg [DIVW-1:0] div;
  wire en_phi1 = ce_en && (div == D_ZERO);
  wire en_phi2 = ce_en && (div == D_HALF);

  always @(posedge clk) begin
    if (rst)        div <= D_ZERO;
    else if (ce_en) div <= (div == D_LAST) ? D_ZERO : div + 1'b1;
  end

  // ---------------------------------------------------------------------
  // fx68k
  // ---------------------------------------------------------------------
  wire        cpu_as_n, cpu_lds_n, cpu_uds_n, cpu_rw_n;
  wire [15:0] cpu_dout;
  wire [23:1] cpu_addr;
  wire        fc0, fc1, fc2;
  wire        vma_n, e_clk;

  reg  dtack_n;
  reg  [15:0] din_hold;

  wire iack  = (fc2 & fc1 & fc0);
  wire vpa_n = ~(iack & ~cpu_as_n);

  fx68k u_fx68k (
      .clk       (clk),
      .HALTn     (1'b1),
      .extReset  (rst | cpu_rst),
      .pwrUp     (rst),
      .enPhi1    (en_phi1),
      .enPhi2    (en_phi2),

      .dbg_d7    (dbg_d7),
      .eRWn      (cpu_rw_n),
      .ASn       (cpu_as_n),
      .LDSn      (cpu_lds_n),
      .UDSn      (cpu_uds_n),
      .E         (e_clk),
      .VMAn      (vma_n),

      .FC0       (fc0),
      .FC1       (fc1),
      .FC2       (fc2),
      .BGn       (),
      .oRESETn   (),
      .oHALTEDn  (halted_n),

      .DTACKn    (dtack_n),
      .VPAn      (vpa_n),
      .BERRn     (1'b1),
      .BRn       (1'b1),
      .BGACKn    (1'b1),

      .IPL0n     (ipl_n[0]),
      .IPL1n     (ipl_n[1]),
      .IPL2n     (ipl_n[2]),

      .iEdb      (din_hold),
      .oEdb      (cpu_dout),
      .eab       (cpu_addr)
  );

  // ---------------------------------------------------------------------
  // Bus cycle -> request/ack handshake.
  // ---------------------------------------------------------------------
  wire cyc = ~cpu_as_n & ~(cpu_lds_n & cpu_uds_n) & ~iack;
  reg  done;

  // The system-side contract only guarantees din in the clock where ack is
  // high.  fx68k captures its external data bus later, on an enabled Phi2
  // edge, so keep the acknowledged word stable until the next read completes.
  always @(posedge clk) begin
    if (rst | cpu_rst)       din_hold <= 16'hFFFF;
    else if (ack && rd)      din_hold <= din;
  end

  always @(posedge clk) begin
    if (rst | cpu_rst) begin
      done    <= 1'b0;
      dtack_n <= 1'b1;
    end else if (!cyc) begin
      done    <= 1'b0;
      dtack_n <= 1'b1;
    end else if (ack) begin
      done    <= 1'b1;
      dtack_n <= 1'b0;
    end
  end

  assign rd    = cyc & ~done &  cpu_rw_n;
  assign wr    = cyc & ~done & ~cpu_rw_n;
  assign addr  = cpu_addr;
  assign dout  = cpu_dout;
  assign uds_n = cpu_uds_n;
  assign lds_n = cpu_lds_n;
  assign fc    = {fc2, fc1, fc0};

  assign iack_stb = iack & ~cpu_as_n;
  assign as_n  = cpu_as_n;

endmodule

`default_nettype wire
