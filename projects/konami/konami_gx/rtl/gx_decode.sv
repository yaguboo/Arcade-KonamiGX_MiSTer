//============================================================================
//  Konami System GX -- 68EC020 address decode
//
//  Split out of gx_main.sv so it can be tested exhaustively without a CPU.
//  That is not a stylistic preference: this factory cannot simulate the main
//  CPU at all (TG68K is VHDL, docs/REUSE_PLAN.md section 6), so anything that
//  stays welded to it can only be checked on hardware.  The decode is both the
//  most mechanical part of the board and the easiest to get quietly wrong --
//  one constant here was already wrong by 0x100 before tb_gx_decode caught it.
//
//  Every range below is a line-for-line transcription of gx_type2_map
//  (konamigx.cpp:1092) plus gx_base_memmap (:1040), listed in the same order
//  as docs/SOURCE_AUDIT.md section 4 so the two can be diffed by eye.
//
//  Addresses are 24-bit: this is a 68EC020, not a 68020.
//============================================================================
`default_nettype none

module gx_decode (
    input  wire [23:0] a,            // byte address, a[0] unused
    input  wire        active,       // qualify with a real bus cycle

    // ROM / RAM regions
    output wire        sel_bios,     // 000000-01ffff   128 KB
    output wire        sel_prg,      // 200000-3fffff   program ROM window
    output wire        sel_dat,      // 400000-7fffff   data ROM window (empty)
    output wire        sel_wram,     // c00000-c1ffff   128 KB work RAM

    // devices
    output wire        esc_cs,           // cc0000-cc0007 (ESC long at 0, winspike's Xilinx at 0 and 4)
    output wire        k056832_rom_cs,   // d00000-d01fff
    output wire        objram_cs,        // d20000-d23fff
    output wire        k056832_reg_cs,   // d40000-d4003f
    output wire        tilebank_cs,      // d44000-d4400f
    output wire        objset1_cs,       // d48000-d48007
    output wire        objrom_cs,        // d4a000-d4a00f
    output wire        objset2_cs,       // d4a010-d4a01f
    output wire        ccu_cs,           // d4c000-d4c01f
    output wire        ccu2_cs,          // d4e000-d4e01f  (type 3/4 only, nopw)
    output wire        k055555_cs,       // d50000-d500ff
    output wire        k056800_cs,       // d52000-d5201f
    output wire        eeprom_cs,        // d56000-d56003
    output wire        control_cs,       // d58000-d58003
    output wire        sysdsw_cs,        // d5a000-d5a003
    output wire        inputs_cs,        // d5c000-d5c003
    output wire        service_cs,       // d5e000-d5e003
    output wire        k054338_cs,       // d80000-d8001f
    output wire        pal_cs,           // d90000-d97fff
    output wire        k056832_ram_cs,   // da0000-da3fff
    // Not in gx_type2_map: MAME installs a handler here for fantjour and
    // fantjoura only (special 9, konamigx.cpp:4054-4055, :4162 ->
    // konamigx_m.cpp:489-538).  DECISIONS D19, MEASUREMENTS 57.  gx_main gates
    // it by the set, so for every other set this select does nothing and the
    // range stays unmapped, as it is in MAME's map.
    output wire        fjdma_cs          // db0000-db001f
);

// ---------------------------------------------------------------------------
//  Memory regions.  These are NOT qualified by `active` -- the SDRAM address
//  generator wants them combinationally, ahead of the request.
// ---------------------------------------------------------------------------
assign sel_bios = (a[23:17] == 7'b0000000);
assign sel_prg  = (a[23:21] == 3'b001);
assign sel_dat  = (a[23:22] == 2'b01);
assign sel_wram = (a[23:17] == 7'b1100000);

// ---------------------------------------------------------------------------
//  Devices.
//
//  Each comparison is `a[23:N] == base >> N` where 2^N is the region size.
//  Written that way rather than as a range compare so the constant can be
//  checked against the map by dividing, which is what tb_gx_decode does.
// ---------------------------------------------------------------------------
assign esc_cs         = active && (a[23:3]  == 21'h198000);
assign k056832_rom_cs = active && (a[23:13] == 11'h680);
assign objram_cs      = active && (a[23:14] == 10'h348);
assign k056832_reg_cs = active && (a[23:6]  == 18'h35000);
assign tilebank_cs    = active && (a[23:4]  == 20'hD4400);
assign objset1_cs     = active && (a[23:3]  == 21'h1A9000);
assign objrom_cs      = active && (a[23:4]  == 20'hD4A00);
assign objset2_cs     = active && (a[23:4]  == 20'hD4A01);
assign ccu_cs         = active && (a[23:5]  == 19'h6A600);
assign ccu2_cs        = active && (a[23:5]  == 19'h6A700);
assign k055555_cs     = active && (a[23:8]  == 16'hD500);
assign k056800_cs     = active && (a[23:5]  == 19'h6A900);
assign eeprom_cs      = active && (a[23:2]  == 22'h355800);
assign control_cs     = active && (a[23:2]  == 22'h356000);
assign sysdsw_cs      = active && (a[23:2]  == 22'h356800);
assign inputs_cs      = active && (a[23:2]  == 22'h357000);
assign service_cs     = active && (a[23:2]  == 22'h357800);
assign k054338_cs     = active && (a[23:5]  == 19'h6C000);
assign pal_cs         = active && (a[23:15] ==  9'h1B2);
assign k056832_ram_cs = active && (a[23:14] == 10'h368);
// 0xdb0000 >> 5 = 0x6D800; the window is 0x20 bytes (konamigx_m.cpp:493).
assign fjdma_cs       = active && (a[23:5]  == 19'h6D800);

endmodule

`default_nettype wire
