//============================================================================
//  Konami System GX -- ioctl byte stream -> SDRAM words
//
//  Adopted from projects/vsystem/power_spikes/integration/ps_download.sv,
//  unchanged except for the name and this header.  Root CLAUDE.md section 1.2
//  shelf 3.  It carries one thing that was expensive to learn and is repeated
//  below in full: the power-up value of `busy` is not optional.
//
//  This is platform transport, so it lives in integration/ and not in rtl/:
//  root CLAUDE.md section 4 keeps ioctl_* out of the board.  The board sees
//  only dl_addr / dl_data / dl_req / dl_ack.
//
//  The MRA tool streams the download image one byte at a time, ascending from
//  0.  SDRAM is 16 bits wide and rtl/gx_rommap.svh fixes the convention:
//
//      word W  =  { byte 2W , byte 2W+1 }      byte 2W in bits [15:8]
//
//  which is big-endian, so the 68EC020 reads program words the right way round
//  with no further swapping anywhere in the core.  The MRA does the 32-bit
//  word interleave for the main program (MAME ROM_LOAD32_WORD_SWAP) and the
//  16-bit one for the sound program; the two conventions meet exactly here.
//
//  Check the reset vector if this is ever in doubt.  The BIOS 300a01.34k is
//  at SDRAM word 0 and the 68EC020 fetches its SSP and PC from the first two
//  longs.
//
//  ---- the second stream: the EEPROM's default contents ---------------------
//  `<rom index="2">` in the .mra carries gokuparo.nv, the 128-byte default
//  93C46 image MAME calls "default eeprom to prevent game booting with error"
//  (konamigx.cpp:2163).  It does NOT go to SDRAM: it goes straight into the
//  EEPROM's storage over the neutral nvs_* port, so the board never learns
//  where the bytes came from.
//
//  This is PLATFORM_TRANSPORT under root CLAUDE.md section 6 -- the same
//  delivery of stored bytes that ROM loading is.  Nothing computes anything.
//
//  The pairing is the SAME big-endian rule as the ROM stream above, and that
//  is not a coincidence: MAME's region is ROM_REGION16_BE, so file byte 2W is
//  the high half of EEPROM word W.
//============================================================================
`default_nettype none

module gx_download (
    input  wire        clk,
    input  wire        rst,

    // --- from the platform --------------------------------------------------
    input  wire        ioctl_download,
    input  wire        ioctl_wr,
    input  wire [26:0] ioctl_addr,
    input  wire [7:0]  ioctl_dout,
    input  wire [15:0] ioctl_index,
    output wire        ioctl_wait,

    // --- to the board's memory port ----------------------------------------
    output reg  [24:0] dl_addr,
    output reg  [15:0] dl_data,
    output reg         dl_req = 1'b0,   // see the note on `busy` below
    input  wire        dl_ack,
    output wire        dl_active,

    // --- to the board's EEPROM, one write per 16-bit word -------------------
    output reg         nvs_wr = 1'b0,
    output reg  [5:0]  nvs_addr,
    output reg  [15:0] nvs_din
);

  // The MRA's <rom index="0"> is the ROM image.  Match all 16 bits: index
  // 0x00FE is the later <switches> transfer and must not reach ROM space.
  wire is_rom = (ioctl_index == 16'd0);

  // <rom index="2"> is the EEPROM default image.  Index 1 is the OSD status
  // defaults and is consumed in the target.  The address guard is not
  // decoration: the region is 128 bytes and nvs_addr is six bits, so without
  // it a longer file would wrap and overwrite word 0.
  wire is_nv_idx = (ioctl_index == 16'd2);
  wire is_nvs    = is_nv_idx && !ioctl_addr[26:7];

  reg [7:0] hold;      // the even byte, waiting for its odd partner

  // THE POWER-UP VALUE HERE IS NOT OPTIONAL.
  //
  // ioctl_wait goes onto HPS_BUS[37], which sys_top.v calls io_wait, and
  // io_wait high stops sys_top from ever raising io_ack.  MiSTer's main
  // spins on io_ack for EVERY word it sends the core, so a `busy` that comes
  // out of configuration set wedges the HPS before the core has done
  // anything at all.  The .qsf sets ALLOW_POWER_UP_DONT_CARE, which makes an
  // uninitialised register genuinely free to power up either way.
  //
  // Inherited verbatim from NA-1/NA-2's na2_membus, which carries the same
  // comment on the same signal for the same reason.  Nothing else in this
  // core has that reach.
  reg       busy = 1'b0;

  assign ioctl_wait = busy;
  // Keep write ownership through the acknowledgement of the final word.  The
  // host may lower ioctl_download immediately after presenting its last byte.
  //
  // The EEPROM stream counts too, and that is not tidiness: gx_top gates the
  // 68EC020's clock enable on this signal and nothing else holds the CPU
  // during a load.  <rom index="2"> arrives AFTER index 0, so without this the
  // CPU would be executing while its EEPROM was still being filled.  The race
  // is one the CPU would almost certainly win -- 128 bytes go by in a few
  // microseconds -- but "almost certainly" is not a thing to leave in a path
  // that cannot be simulated.
  assign dl_active = (ioctl_download & (is_rom | is_nv_idx)) | busy;

  reg [7:0] nv_hold;   // the even byte of an EEPROM word

  always @(posedge clk) begin
    if (rst) begin
      dl_req <= 1'b0;
      busy   <= 1'b0;
      hold   <= 8'd0;
      nvs_wr <= 1'b0;
    end else begin
      nvs_wr <= 1'b0;
      if (ioctl_wr && is_nvs) begin
        if (!ioctl_addr[0]) begin
          nv_hold <= ioctl_dout;
        end else begin
          nvs_addr <= ioctl_addr[6:1];
          nvs_din  <= {nv_hold, ioctl_dout};
          nvs_wr   <= 1'b1;
        end
      end

      if (dl_req && dl_ack) begin
        dl_req <= 1'b0;
        busy   <= 1'b0;
      end

      if (ioctl_wr && is_rom) begin
        if (!ioctl_addr[0]) begin
          // even byte: high half of the word, held until the odd byte
          hold <= ioctl_dout;
        end else begin
          dl_addr <= ioctl_addr[25:1];
          dl_data <= {hold, ioctl_dout};
          dl_req  <= 1'b1;
          busy    <= 1'b1;
        end
      end
    end
  end

endmodule

`default_nettype wire
