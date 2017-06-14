//****************************************************************************
//*
//*   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
//*   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
//*
//* Redistribution and use in source and binary forms, with or without
//* modification, are permitted provided that the following conditions
//* are met:
//*
//* 1. Redistributions of source code must retain the above copyright
//*    notice, this list of conditions and the following disclaimer.
//* 2. Redistributions in binary form must reproduce the above copyright
//*    notice, this list of conditions and the following disclaimer in
//*    the documentation and/or other materials provided with the
//*    distribution.
//*
//* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//* POSSIBILITY OF SUCH DAMAGE.
//*
//*
//****************************************************************************/

`include "config.v"

// need to provide correct clock for cpu == ethernet and freq meter

module clock_provider
(
    input wire                  clk_i,
    input wire                  rmii_clk_to_PHY_i,

    output wire                 sys_clk_o,
    output wire                 rmii_logick_clk_o,
    output wire                 clk_ref_o
);

`ifdef CLOCK_USE_PLL

// reference clock gen clk_ref = clk_i * `FREF_PLL_MULTIPLYER / `FREF_CLOCK_DEVIDER
DCM_CLKGEN #(
   .CLKFXDV_DIVIDE(2),       // CLKFXDV divide value (2, 4, 8, 16, 32)
   .CLKFX_DIVIDE(`CLOCK_FREF_CLOCK_DEVIDER),         // Divide value - D - (1-256)
   .CLKFX_MD_MAX(`CLOCK_FREF_PLL_MULTIPLYER * 2 / `CLOCK_FREF_CLOCK_DEVIDER),       // Specify maximum M/D ratio for timing anlysis
   .CLKFX_MULTIPLY(`CLOCK_FREF_PLL_MULTIPLYER * 2),       // Multiply value - M - (2-256)
   .CLKIN_PERIOD(`INPUT_CLOCK_PERIOD_NS_F),       // Input clock period specified in nS
   .SPREAD_SPECTRUM("NONE"), // Spread Spectrum mode "NONE", "CENTER_LOW_SPREAD", "CENTER_HIGH_SPREAD",
                             // "VIDEO_LINK_M0", "VIDEO_LINK_M1" or "VIDEO_LINK_M2"
   .STARTUP_WAIT("TRUE")    // Delay config DONE until DCM_CLKGEN LOCKED (TRUE/FALSE)
)
DCM_CLKGEN_f_ref (
   .CLKFX(/* open */),         // 1-bit output: Generated clock output
   .CLKFX180(/* open */),   // 1-bit output: Generated clock output 180 degree out of phase from CLKFX.
   .CLKFXDV(clk_ref_o),     // 1-bit output: Divided clock output
   .LOCKED(/* open */),       // 1-bit output: Locked output
   .PROGDONE(/* open */),   // 1-bit output: Active high output to indicate the successful re-programming
   .STATUS(/* open */),       // 2-bit output: DCM_CLKGEN status
   .CLKIN(clk_i),         // 1-bit input: Input clock
   .FREEZEDCM(1'b0), // 1-bit input: Prevents frequency adjustments to input clock
   .PROGCLK(1'b0),     // 1-bit input: Clock input for M/D reconfiguration
   .PROGDATA(1'b0),   // 1-bit input: Serial data input for M/D reconfiguration
   .PROGEN(1'b0),       // 1-bit input: Active high program enable
   .RST(1'b0)              // 1-bit input: Reset input pin
);

// CPU clock gen clk = clk_i * `CPU_PLL_MULTIPLYER / `CPU_CLOCK_DEVIDER
DCM_CLKGEN #(
   .CLKFXDV_DIVIDE(2),       // CLKFXDV divide value (2, 4, 8, 16, 32)
   .CLKFX_DIVIDE(`CLOCK_CPU_CLOCK_DEVIDER),         // Divide value - D - (1-256)
   .CLKFX_MD_MAX(`CLOCK_CPU_PLL_MULTIPLYER * 2 / `CLOCK_CPU_CLOCK_DEVIDER),       // Specify maximum M/D ratio for timing anlysis
   .CLKFX_MULTIPLY(`CLOCK_CPU_PLL_MULTIPLYER * 2),       // Multiply value - M - (2-256)
   .CLKIN_PERIOD(`INPUT_CLOCK_PERIOD_NS_F),       // Input clock period specified in nS
   .SPREAD_SPECTRUM("NONE"), // Spread Spectrum mode "NONE", "CENTER_LOW_SPREAD", "CENTER_HIGH_SPREAD",
                             // "VIDEO_LINK_M0", "VIDEO_LINK_M1" or "VIDEO_LINK_M2"
   .STARTUP_WAIT("TRUE")    // Delay config DONE until DCM_CLKGEN LOCKED (TRUE/FALSE)
)
DCM_CLKGEN_f_rmii (
   .CLKFX(/* open */),         // 1-bit output: Generated clock output
   .CLKFX180(/* open */),   // 1-bit output: Generated clock output 180 degree out of phase from CLKFX.
   .CLKFXDV(sys_clk_o),     // 1-bit output: Divided clock output
   .LOCKED(/* open */),       // 1-bit output: Locked output
   .PROGDONE(/* open */),   // 1-bit output: Active high output to indicate the successful re-programming
   .STATUS(/* open */),       // 2-bit output: DCM_CLKGEN status
   .CLKIN(clk_i),         // 1-bit input: Input clock
   .FREEZEDCM(1'b0), // 1-bit input: Prevents frequency adjustments to input clock
   .PROGCLK(1'b0),     // 1-bit input: Clock input for M/D reconfiguration
   .PROGDATA(1'b0),   // 1-bit input: Serial data input for M/D reconfiguration
   .PROGEN(1'b0),       // 1-bit input: Active high program enable
   .RST(1'b0)              // 1-bit input: Reset input pin
);
`else

assign sys_clk_o = clk_i;
assign clk_ref_o = clk_i;

`endif
//-----------------------------------------------------------------------------
`ifdef DEVICE_BOARD_NAME_ZR_TECH_V200 // board ZR-Tech_v2.00

assign rmii_logick_clk_o = rmii_clk_to_PHY_i;

`define CONFIG_OK
//-----------------------------------------------------------------------------
`endif
`ifdef DEVICE_BOARD_NAME_RCH2_V20 // board RCH_2.0

assign rmii_logick_clk_o = clk_i;

`define CONFIG_OK
`endif
//-----------------------------------------------------------------------------
`ifndef CONFIG_OK // no valid board specified
`error_DEVICE_BOARD_NAME_NOT_SET
`endif

endmodule
