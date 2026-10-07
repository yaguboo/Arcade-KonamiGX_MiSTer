derive_pll_clocks
derive_clock_uncertainty

# ---------------------------------------------------------------------------
# SDRAM
#
# There are deliberately NO set_input_delay / set_output_delay constraints on
# the SDRAM pins.  The sibling lane put some in once, invented from memory, and
# the fitter duly failed them by 3.4 ns -- a made-up requirement missed by a
# made-up margin, which tells you nothing about the board.
#
# Established MiSTer practice (MiSTer-devel/NES_MiSTer NES.sdc,
# MiSTer-devel/Genesis_MiSTer Genesis.sdc) is to leave the SDRAM pins
# unconstrained and set the interface timing physically, with the phase of the
# clock driven out on SDRAM_CLK.
#
# Ours is 180 degrees (rtl/pll/pll_0002.v, phase_shift1 = -5208 ps, half of the
# 10417 ps period).  At 96 MHz that leaves:
#   output path   5.2 ns for FPGA clock-to-out + board delay + SDRAM setup
#                 -- against roughly 5 ns needed.  MARGINAL.
#   input path    5.2 ns for SDRAM tAC + board delay + FPGA setup
#                 -- against roughly 7 ns needed.  DOES NOT CLOSE.
#
# **That is docs/DECISIONS.md D3 and it is expected to be wrong.**  The
# sibling has 12.5 ns on each side at 40 MHz, which is why it works.  This
# constraint file cannot fix it -- the phase is a PLL parameter and the fix is
# a hardware measurement, not an SDC line.  Written down here anyway, because
# this is the file somebody will open when reads come back corrupted.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Framework video paths.  The scaler and the HQ2x filter are deeply pipelined
# and do not need to settle in a single core clock; the same relaxations appear
# in the reference cores above.
# ---------------------------------------------------------------------------
set_multicycle_path -to {*Hq2x*} -setup 4
set_multicycle_path -to {*Hq2x*} -hold 3

set_multicycle_path -from [get_clocks {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}] -to {ascal|*} -setup 4
set_multicycle_path -from [get_clocks {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}] -to {ascal|*} -hold 3

# ---------------------------------------------------------------------------
# clk_aux -> clk_sys is a DELIBERATE asynchronous crossing.
#
# The PLL's third output (168 MHz) exists to keep the solver from merging
# counters and to give the liveness counter a clock; it must have a real load
# or Quartus deletes it and re-solves the PLL, which is how the sibling lane
# once ended up with a core clock that never ran on hardware.  Its load is a
# free-running counter whose top bit is sampled into clk_sys through a two-flop
# synchroniser.
#
# Left unconstrained, TimeQuest times that crossing as a real 192-to-96 MHz
# path and reports negative slack on a path that is a synchroniser.  It must
# not be timed.
# ---------------------------------------------------------------------------
set_false_path   -from [get_clocks {*|pll|pll_inst|altera_pll_i|general[2].*|divclk}]   -to   [get_clocks {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}]

# ---------------------------------------------------------------------------
# TG68K runs on a clock enable, and TimeQuest has to be told.
#
# The first build closed at Fmax 46.81 MHz against a 96 MHz constraint, and all
# five worst paths were inside the CPU -- exec decode into the register file's
# write port.  That is not a marginal path to be re-rolled with a seed; it is
# TG68K's own combinational depth with the 68020 options on (BarrelShifter,
# BitField, 32-bit MUL/DIV, extAddr_Mode), and no placement makes it fit in
# 10.4 ns.
#
# It does not have to.  gx_main drives `clkena_in` from a 24 MHz enable, so the
# CPU advances on ONE CLOCK IN FOUR and its logic has 41.7 ns to settle.
#
# WHY THIS IS SOUND, and what would make it unsound:
#
#   * every state-bearing register in TG68KdotC_Kernel.vhd changes only when
#     `clkena_in` is high, or when `clkena_lw` is -- and line 450 defines
#     clkena_lw as `clkena_in AND memmaskmux(3)`, a strict subset.  Checked
#     process by process, 2026-09-07.  The two exceptions are
#     `use_VBR_Stackframe`, which is a pure function of the generics and is
#     constant after elaboration, and the synchronous reset loads, which are
#     constants held for many cycles.
#   * `cen` comes from a free-running 2-bit counter in gx_top (`cpu_cnt == 3`),
#     so consecutive active edges are EXACTLY four clocks apart -- never three.
#     If that divider is ever changed, this number changes with it.
#   * the multicycle is confined to paths INSIDE the CPU.  Paths crossing into
#     or out of it keep the full 96 MHz requirement, because the logic on the
#     other side is not enable-gated and may change on any clock.
#
# docs/DECISIONS.md D8.
# ---------------------------------------------------------------------------
# ONE VALUE PER STATEMENT.  `-setup` and `-hold` are FLAGS; the multiplier is
# the single positional <value> the command takes.  Written as
#     set_multicycle_path -setup 4 -hold 3 -from ... -to ...
# it passes TWO positionals and Quartus rejects the whole line with
#     Error (332000): More than 1 positional argument specified: 4, 3
# The constraint is then simply absent, which is indistinguishable from D8 not
# working -- and worse, quartus_fit aborts the run with a misleading
# "Error (11802): Can't fit design in device" while every resource sits under
# 35 %.  That cost a full build on 2026-09-07.  The Hq2x and ascal lines above
# had the right shape all along; this one did not.
set_multicycle_path -setup 4 -from [get_registers {*TG68KdotC_Kernel*}] -to [get_registers {*TG68KdotC_Kernel*}]
set_multicycle_path -hold  3 -from [get_registers {*TG68KdotC_Kernel*}] -to [get_registers {*TG68KdotC_Kernel*}]

# The CPU bus slice, BOTH directions.
#
# 2026-09-08: gx_main's slice moved from cen_d1 to cen_d2, so BOTH directions
# are now two clocks and both get an exception.  The reason is measured: the
# recurring critical path of this design is TG68K's register file into
# gx_main|skipFetch -- 8 logic levels plus 2.371 ns of clock skew -- and on
# cen_d1 that direction was a genuine SINGLE cycle that no exception could
# honestly relax.  It read -0.214, then better than +0.237, then -0.469, then
# -0.613 as unrelated one-wire edits reshuffled placement.  Two reseeds failed.
#
#     TG68K launches on cen, the slice captures at cen+2   -> 2 clocks
#     the slice launches at cen+2, TG68K captures at cen+4 -> 2 clocks
#
# The second used to be 3 and is now 2, which is the price of the first.  Both
# need roughly 11 ns and now have 20.8.
#
# If the CPU's enable period ever stops being exactly four clocks, BOTH of
# these numbers change.  gx_top's cpu_cnt is what sets it.
#
# gx_main's slice registers (cpu_a32, cpu_d_out, busstate, nWr, nUDS, nLDS,
# skipFetch) capture on cen_d2 -- two clocks after the CPU's own enable, which
# is when the kernel's combinational outputs have settled.  They therefore
# launch exactly once per cen period.
#
# The CPU captures on cen.  Launch at cen+2, capture at the NEXT cen, so the
# interval is TWO clocks and this says 2, not 4.  Claiming 4 would be a promise
# the design does not keep.
#
# (This paragraph said cen_d1 and THREE until 2026-09-21.  It was written for
# the arrangement the block header above describes as superseded on 2026-09-08
# and was never updated; the constraints below have said 2 since that day.
# Found by an external review of the timing failures of 2026-09-21.)
#
# MEASURED, build N -- the five worst paths were this slice feeding the address
# decoder, the read mux and so the CPU's own data input:
#     gx_main|cpu_a32[9] -> TG68KdotC_Kernel|TG68K_ALU|Flags[1]   -0.875
#
# The destination list is TG68K ONLY.  The same registers also drive the chip
# selects, the work RAM and every device register file, none of which are
# enable-gated, and those paths keep the full single-cycle requirement.
#
# 2026-09-21: two destinations ARE enable-gated and get their own exception at
# the end of this section -- gx_esc and gx_fjdma, the slice's borrowed masters.
#
# 2026-09-16, DECISIONS D18: `esc_own` joins this list.  It holds the kernel's
# clkena low for an ESC run, and it changes on a cen edge (the write that asks
# for a run) or on a cen_d2 edge (the hand-back); the kernel captures on cen,
# so the interval is four clocks or two, never one.
set_multicycle_path -setup 2 -from [get_registers {*gx_main*|cpu_a32*  *gx_main*|cpu_d_out*  *gx_main*|busstate*  *gx_main*|nWr  *gx_main*|nUDS  *gx_main*|nLDS  *gx_main*|skipFetch  *gx_main*|esc_own  *gx_main*|dsel_r*}] -to [get_registers {*TG68KdotC_Kernel*}]
set_multicycle_path -hold  1 -from [get_registers {*gx_main*|cpu_a32*  *gx_main*|cpu_d_out*  *gx_main*|busstate*  *gx_main*|nWr  *gx_main*|nUDS  *gx_main*|nLDS  *gx_main*|skipFetch  *gx_main*|esc_own  *gx_main*|dsel_r*}] -to [get_registers {*TG68KdotC_Kernel*}]

# And the other direction, which is the one this change exists for.  The
# destination list is the slice ONLY: the same kernel outputs also feed things
# that are not enable-gated, and those keep the full single-cycle requirement.
#
# 2026-09-16, DECISIONS D18: `fc_r` is part of the slice now (the IACK cycle is
# recognised from it) and loads from the kernel's FC on cen_d2 like the rest.
set_multicycle_path -setup 2 -from [get_registers {*TG68KdotC_Kernel*}] -to [get_registers {*gx_main*|cpu_a32*  *gx_main*|cpu_d_out*  *gx_main*|busstate*  *gx_main*|nWr  *gx_main*|nUDS  *gx_main*|nLDS  *gx_main*|skipFetch  *gx_main*|fc_r*  *gx_main*|dsel_r*}]
set_multicycle_path -hold  1 -from [get_registers {*TG68KdotC_Kernel*}] -to [get_registers {*gx_main*|cpu_a32*  *gx_main*|cpu_d_out*  *gx_main*|busstate*  *gx_main*|nWr  *gx_main*|nUDS  *gx_main*|nLDS  *gx_main*|skipFetch  *gx_main*|fc_r*  *gx_main*|dsel_r*}]

# ---------------------------------------------------------------------------
# THE SLICE'S TWO BORROWED MASTERS -- gx_esc (D18) and gx_fjdma (D19)
#
# MEASURED, 2026-09-21, three consecutive builds that missed timing on changes
# that have nothing to do with this bus (a debug log, a ramstyle attribute, one
# comparator in gx_prio):
#
#     3da75ac  seed  9   -0.951   cpu_a32[19,21,22] -> gx_fjdma|hi_r, lo_r (all five)
#     6e1d4d1  seed  9   -0.165   cpu_a32[19]       -> gx_fjdma|hi_r[10], lo_r[10]
#     89e5977  seed  5   -0.230   cpu_a32[20]       -> gx_esc|rv[3], rv[0], st.S_G_X
#
# The endpoint moved with the seed; the FAMILY did not.  The same design read
# +0.097 one netlist earlier, which is this clock behaving as the .qsf's seed
# notes describe (-0.044 to +0.720 on identical RTL).  A path that is one edit
# away from failing in three different places is a path that is constrained
# wrongly, not a path that is unlucky.
#
# WHY TWO CLOCKS IS TRUE HERE.  Both modules are bus masters that borrow the
# slice while the kernel is frozen.  Everything they take from it arrives
# through `esc_rdata`/`bus_rdata` (gx_main: `assign esc_rdata = cpu_din;`) or
# through the acknowledge, and the acknowledge is
#
#     esc_ack = esc_own && esc_live && cpu_step,   cpu_step = cen && !stall
#
# so it can only be true on a `cen` edge -- one clock in four (gx_top's
# cpu_cnt).  The slice launches at cen+2; the next cen is two clocks later, and
# a stall only pushes it further out.
#
# AND A SPURIOUS CAPTURE IS IMPOSSIBLE, which is the part that makes this an
# exception rather than a waiver.  `cen` is a register and it is ANDed into the
# acknowledge, so on every non-cen edge the acknowledge is 0 no matter what the
# cpu_a32 cone is doing.  Registers like `st` and `bus_req` are written in
# several branches and so are captured on every edge in TimeQuest's view, but
# the only slice-originating cone into them is through that acknowledge, and it
# cannot be 1 at an intermediate edge.  A late cpu_a32 therefore cannot move
# them early.  Checked by reading every assignment in both modules, 2026-09-21.
#
# THE LIST IS EXPLICIT ON PURPOSE.  `-to [get_registers {*gx_esc* *gx_fjdma*}]`
# would silently cover a future register in those modules that takes something
# from the slice on an UNGATED edge, and that would be a false exception on a
# board with no way to simulate the main CPU.  Add to this list by name, after
# checking the same thing.
#
# An external review (Codex, 2026-09-21) proposed this scope; the argument
# above and the reading of both modules were done here before it was taken.
set gx_borrow_src [get_registers {*gx_main*|cpu_a32*  *gx_main*|cpu_d_out*  *gx_main*|busstate*  *gx_main*|nWr  *gx_main*|nUDS  *gx_main*|nLDS  *gx_main*|skipFetch  *gx_main*|dsel_r*}]
#
# THE SEAM'S OWN REGISTERS COUNT TOO.  The first build with this exception
# (2026-09-21) moved the worst path off gx_esc and gx_fjdma and onto
# gx_main|esc_cmd -- the same shape one level up.  gx_main holds the registers
# that RECEIVE the game's writes to the two devices, and every one of them is
# written under `esc_word_wr` or `fj_word_wr`, both of which are
# `cpu_step && ...`, or under `cen_d2` / `esc_ack` / `iack4`, all cen-rate:
#
#     esc_hi esc_cmd esc_start esc_own     esc_word_wr  (gx_main.sv:665-669)
#     esc_own                              fj_word_wr   (:677)
#     esc_live                             cen_d2 / esc_ack (:679)
#     esc_fin                              esc_done / cen_d2 (:682)
#     int4                                 esc_irq4_set / iack4 (:689, :601)
#     fj_trig fj_wr_r                      fj_word_wr   (:646, :650)
#
# NOT in this list: fj_a_r, fj_d_r, fj_be_r.  They sample the slice on EVERY
# edge with no enable at all (:647-649).  They are safe in fact -- the slice
# holds its value for a whole cen period and they are re-sampled before
# fj_reg_wr consumes them -- but "safe in fact" is a longer argument than the
# one above, and they are flop-to-flop copies with no logic between, so they
# have never been near the critical path.  Leave them alone until they are.
# D31 (2026-10-02): rtl/esc/esc_host, the ESC chip core's word port, is the
# same kind of borrowed master.  Every one of its assignments that reads the
# slice (bus_rdata, bus_ack) is in its `else if (bus_ack)` branch, and gx_top
# gives it bus_ack = esc_ack && !fj_busy -- cen-rate as above.  Its other
# branches read only the core's registered request.  The patterns above that
# say *gx_esc* also match gx_esc056's names; inside it, esc_cpu takes nothing
# from the slice (esc_host's c_ack / c_rdata are registers), so those matches
# constrain no path.  Checked by reading esc_host.sv and gx_esc056.sv.
set gx_borrow_dst [get_registers {*gx_esc*|rv*  *gx_esc*|rd_hi*  *gx_esc*|st*  *gx_esc*|bus_req  *gx_esc*|bus_we  *gx_fjdma*|hi_r*  *gx_fjdma*|lo_r*  *gx_fjdma*|st*  *gx_fjdma*|bus_addr*  *gx_fjdma*|bus_din*  *gx_fjdma*|bus_req  *gx_fjdma*|bus_we  *gx_main*|esc_hi*  *gx_main*|esc_cmd*  *gx_main*|esc_start  *gx_main*|esc_own  *gx_main*|esc_live  *gx_main*|esc_fin  *gx_main*|int4  *gx_main*|fj_trig  *gx_main*|fj_wr_r  *esc_host*|acc*  *esc_host*|wd*  *esc_host*|a[*]  *esc_host*|left*  *esc_host*|two  *esc_host*|busy}]
set_multicycle_path -setup 2 -from $gx_borrow_src -to $gx_borrow_dst
set_multicycle_path -hold  1 -from $gx_borrow_src -to $gx_borrow_dst

# ---------------------------------------------------------------------------
# The video path: the tilemap's shifters feed the K055555 comparator, and both
# ends move once per DOT, not once per system clock.
#
# MEASURED, build E, 2026-09-07 -- the five worst setup paths in the design:
#     gx_tilemap|shift[1][4] -> gx_prio|idx1[7]     slack -7.001
#
# Same shape as D8's problem one domain over.  gx_tilemap's `shift` registers
# and its `cur_col`/`cur_mix` latches are loaded only on `pxl_cen`, and
# gx_prio's output registers are now gated on `pxl_cen` too (they used to
# capture on every 96 MHz edge, so the whole 8-input comparator was being asked
# to settle in 10.4 ns while its inputs changed once every 16).
#
# WHY 6 AND NOT 16.  `pxl_cen` divides by dot_div+1 and DOTSEL picks dot_div
# from {15, 11, 7, 5} at RUNTIME (rtl/gx_top.sv), so the period is 16, 12, 8 or
# 6 and the constraint has to be true for the fastest.  Gokujou Parodius
# programs DOTSEL=00 and never changes it, but the RTL implements all four and
# a constraint that is only true for this game is the kind of promise that is
# invisible until hardware.
#
# There is no runt pulse to worry about: dot_cnt is reset to 0 whenever it
# reaches or passes the terminal count, so even a DOTSEL change mid-line
# produces one pulse and the next one is a full period later.
#
# THE SOURCE LIST IS DELIBERATELY NARROW.  It names the pxl_cen-gated registers
# and nothing else.  gx_prio's own configuration registers are written by the
# CPU on its own enable and can change at any time relative to a dot, so paths
# from THOSE into the comparator keep the full single-cycle requirement -- as
# they must.  Widening this to `-from [get_registers {*gx_prio*}]` would cover
# them and would be false.
# ---------------------------------------------------------------------------
#
# 2026-09-14: `shift` is gone.  The tilemap now honours the sub-tile X scroll
# with a two-group window per layer -- `prv_*` and `cur_*`, plus the per-dot
# select `cur_sel` -- and every one of them is loaded only on `pxl_cen`, the
# select included (it is precomputed for the dot that starts at the enable).
# So the same two patterns name the whole pxl_* / col_* output cone.
set_multicycle_path -setup 6 -from [get_registers {*gx_tilemap*prv_*}]  -to [get_registers {*gx_prio*}]
set_multicycle_path -hold  5 -from [get_registers {*gx_tilemap*prv_*}]  -to [get_registers {*gx_prio*}]
set_multicycle_path -setup 6 -from [get_registers {*gx_tilemap*cur_*}]  -to [get_registers {*gx_prio*}]
set_multicycle_path -hold  5 -from [get_registers {*gx_tilemap*cur_*}]  -to [get_registers {*gx_prio*}]

# The K055555's own configuration, once it is in the pixel domain.
#
# `regs` is CPU-written and can change on any 96 MHz edge, so it is NOT a legal
# source here -- and build F proved the point rather than the theory: with the
# tilemap paths relaxed, the five worst paths in the design became
#
#     gx_prio|regs[45][0] -> gx_prio|idx1[7]     slack -8.726
#
# The answer was to make the statement true, not to widen the pattern until it
# covered the problem.  gx_prio now reads only `pregs`, a copy loaded on
# pxl_cen, so every input to the comparator moves at the dot rate.  The
# regs -> pregs leg is a register-to-register copy with no logic in it and
# keeps the full single-cycle requirement, as it should.
#
# `*pregs*` does not match `regs`, so the CPU-written array is still excluded.
set_multicycle_path -setup 6 -from [get_registers {*gx_prio*pregs*}] -to [get_registers {*gx_prio*}]
set_multicycle_path -hold  5 -from [get_registers {*gx_prio*pregs*}] -to [get_registers {*gx_prio*}]

# The sprite chip's OBJ input to the same comparator, 2026-09-14.
#
# gx_sprite's `pix_*` registers -- pen, c18, coregshift, shadow code, SDSEL --
# are loaded only on pxl_cen, from the line buffer's read register and the
# CPU-written OBJSET registers.  Those two sources feed `pix_*` through an
# ordinary single-cycle path, which is right: they can change on any clock.
# From `pix_*` onward everything moves at the dot rate, exactly as the
# tilemap's `prv_*` / `cur_*` do above, so it gets the same six.
#
# The pattern needs `pix_` right after the hierarchy separator, so neither the
# line buffer nor the OBJSET register files can match it.
set_multicycle_path -setup 6 -from [get_registers {*gx_sprite*|pix_*}] -to [get_registers {*gx_prio*}]
set_multicycle_path -hold  5 -from [get_registers {*gx_sprite*|pix_*}] -to [get_registers {*gx_prio*}]

# gx_prio -> gx_colmix.  Both ends already move at the dot rate; only the
# constraint was missing.
#
# MEASURED, build G -- with the comparator's own two problems fixed, the five
# worst paths in the design became
#
#     gx_prio|bg0 -> gx_colmix|green[0]     slack -6.991
#
# gx_colmix's red/green/blue have been gated on pxl_cen since the module was
# written, and EVERY output of gx_prio is a register in the pxl_cen-gated block
# -- idx0, idx1, bg0, bg1, blend, bri, shadow, dbg_winner.  Nothing leaves
# gx_prio combinationally.
#
# That last sentence is what makes the wide `-from` pattern safe here where it
# was not safe two commits ago: gx_prio's CPU-written `regs` array has no path
# to gx_colmix at all, because its only route out of the module is through
# those output registers, and a register breaks a path.
set_multicycle_path -setup 6 -from [get_registers {*gx_prio*}] -to [get_registers {*gx_colmix*}]
set_multicycle_path -hold  5 -from [get_registers {*gx_prio*}] -to [get_registers {*gx_colmix*}]

# The K054338's control registers, once they are in the pixel domain.
#
# MEASURED, build H -- with everything upstream relaxed, the five worst paths
# in the design became
#
#     gx_colmix|bri_g[2] -> gx_colmix|green[2]     slack -5.689
#
# i.e. the brightness bytes and the rest of the '338 register file feeding the
# blend arithmetic.  All CPU-written, so they are not legal sources; gx_colmix
# now reads them through `p_*` copies loaded on pxl_cen.  jt054338 is vendored
# byte-identical, so the copy lives on its outputs rather than inside it.
#
# The pattern requires `p_` immediately after the hierarchy separator, so it
# cannot pick up an unrelated name inside the vendored module.
set_multicycle_path -setup 6 -from [get_registers {*gx_colmix*|p_*}] -to [get_registers {*gx_colmix*}]
set_multicycle_path -hold  5 -from [get_registers {*gx_colmix*|p_*}] -to [get_registers {*gx_colmix*}]

# The palette's video outputs, once they are in the pixel domain.
#
# rgb0_r/rgb1_r change every THREE clocks -- the read-port rotation writes them
# when it reaches their phase -- so they are NOT legal sources even though
# gx_colmix captures every six or more.  A setup multicycle is a claim about
# the interval between a launch and the capture that uses it, and these two
# enables have no fixed phase relationship.  gx_palette now carries rgb0_p /
# rgb1_p, loaded on pxl_cen, and those are the legal sources.
#
# The rotation itself still runs every clock and is deliberately NOT relaxed:
# gx_prio's index -> the palette's read address is a real single-cycle path,
# because the RAM is read on every clock to service the CPU as well.
set_multicycle_path -setup 6 -from [get_registers {*gx_palette*|rgb?_p*}] -to [get_registers {*gx_colmix*}]
set_multicycle_path -hold  5 -from [get_registers {*gx_palette*|rgb?_p*}] -to [get_registers {*gx_colmix*}]

# ---------------------------------------------------------------------------
#  fx68k's instruction-decode PLA, which runs at 8 MHz
# ---------------------------------------------------------------------------
#  MEASURED, the first build with the sound CPU in it (2026-09-08): setup
#  -0.753 on the core clock, TNS -2.225, and THE TWENTY WORST PATHS ARE ALL ONE
#  CLASS:
#
#      gx_sound|gx_m68k|fx68k|Ir[0]  ->  gx_sound|gx_m68k|fx68k|nanoAddr[4]
#
#  That is the instruction register through `uaddrPla` into the microcode
#  address -- fx68k's own decode depth, not anything this board wrote.  It is
#  the same situation D8 found for the main CPU and it has the same answer, but
#  the arithmetic is different and has to be redone rather than copied.
#
#  WHY POWER SPIKES NEVER NEEDED THIS with the same fx68k: its system clock is
#  40 MHz, so one period is 25 ns and this path fits with room to spare.  Ours
#  is 96 MHz -- 10.42 ns -- and the path needs about 11.2.  The CPU is not
#  faster here; the clock it is being MEASURED against is.
#
#  WHY THE EXCEPTION IS SOUND.  A multicycle is a promise about how often the
#  DESTINATION can latch, and a false promise is invisible until hardware.
#  Checked in the vendored source, 2026-09-08:
#
#    * `nanoAddr` and `microAddr` are written in exactly one place,
#      fx68k.sv:262-272, under `if (enT1)` -- and under `if (Clks.pwrUp)`,
#      which loads a CONSTANT and is a level held for many clocks.
#
#    * fx68k.sv:180-183 defines all four T-state enables as a conjunction with
#      a phase enable:
#          enT1 = Clks.enPhi1 & (tState == T4) & ~wClk
#          enT2 = Clks.enPhi2 & (tState == T1)
#          enT3 = Clks.enPhi1 & (tState == T2)
#          enT4 = Clks.enPhi2 & ((tState == T0) | (tState == T3))
#
#    * gx_m68k generates enPhi1 at div == 0 and enPhi2 at div == 6 of a
#      free-running CLK_DIV = 12 counter, so consecutive PHASE enables are
#      exactly 6 system clocks apart and consecutive enT1s are 12.
#
#  So the shortest possible interval from any launch inside fx68k to a capture
#  in `nanoAddr` is SIX clocks, and for this path class it is twelve.
#
#  THIS CLAIMS TWO.  Two periods is 20.83 ns against the ~11.2 the path needs,
#  which closes it with 9 ns to spare, and leaving three quarters of the real
#  margin unclaimed is deliberate: it is the distance by which the analysis
#  above can be wrong and the constraint still be true.
#
#  IF `CLK_DIV` IN gx_m68k EVER CHANGES, re-derive the six and the twelve.  At
#  CLK_DIV = 2 the phases are one clock apart and this exception becomes false.
#
#  Destination-only, deliberately.  Naming the source would have to match
#  `Ir[2]~DUPLICATE` as well as `Ir[2]` -- the fitter duplicates that register --
#  and a pattern that silently misses the duplicate would leave the worst path
#  unconstrained while appearing to work.  Constraining only the destination
#  cannot miss a source, and every source reaching these two registers is
#  inside fx68k and enable-gated.
set_multicycle_path -setup 2 -to [get_registers {*fx68k*|nanoAddr[*]}]
set_multicycle_path -hold  1 -to [get_registers {*fx68k*|nanoAddr[*]}]
set_multicycle_path -setup 2 -to [get_registers {*fx68k*|microAddr[*]}]
set_multicycle_path -hold  1 -to [get_registers {*fx68k*|microAddr[*]}]

# ---------------------------------------------------------------------------
# cfg_machine is a load-time constant.  2026-10-01, e8b96d3a (= 79cd7f61's
# RTL) seed 10: -0.501 on emu|cfg_machine[1] -> gx_memarb|l_addr[8] -- the
# set's decode (gx_top `dragoonj`, `salmndr2`, ...) steering a client address
# into the arbiter's mux.  Not a path the D26 / D28 change touched; the netlist
# moved and exposed it.
#
# Why two clocks are true: KonamiGX.sv writes cfg_machine only on ioctl_wr of
# ioctl index 1 (the .mra's four header bytes, sent between ROM index 0 and
# index 2), so it changes only while dl_active is high -- gx_top holds cen_cpu
# and snd_ce_en low for all of that -- and its two writes (byte 0 clears it,
# byte 3 sets it) are three ioctl writes apart, hundreds of clocks.  Every
# consumer has the final value long before anything that runs on it starts.
# Source-only on purpose: the register is the fact, its consumers are many.
set_multicycle_path -setup 2 -from [get_registers {*|cfg_machine[*]}]
set_multicycle_path -hold  1 -from [get_registers {*|cfg_machine[*]}]
