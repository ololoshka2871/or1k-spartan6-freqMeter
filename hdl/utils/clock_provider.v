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

    output wire                 sys_clk_o,
    output wire                 rmii_clk_to_PHY_o,
    output wire                 rmii_logick_clk_o,
    output wire                 clk_ref
);

// reference clock gen clk_ref = clk_i * `FREF_PLL_MULTIPLYER / `FREF_CLOCK_DEVIDER
DCM_CLKGEN #(
   .CLKFXDV_DIVIDE(2),       // CLKFXDV divide value (2, 4, 8, 16, 32)
   .CLKFX_DIVIDE(`FREF_PLL_MULTIPLYER),         // Divide value - D - (1-256)
   .CLKFX_MD_MAX(`FREF_PLL_MULTIPLYER * 2 / `FREF_PLL_MULTIPLYER),       // Specify maximum M/D ratio for timing anlysis
   .CLKFX_MULTIPLY(`FREF_PLL_MULTIPLYER * 2),       // Multiply value - M - (2-256)
   .CLKIN_PERIOD(`INPUT_CLOCK_PERIOD_NS_F),       // Input clock period specified in nS
   .SPREAD_SPECTRUM("NONE"), // Spread Spectrum mode "NONE", "CENTER_LOW_SPREAD", "CENTER_HIGH_SPREAD",
                             // "VIDEO_LINK_M0", "VIDEO_LINK_M1" or "VIDEO_LINK_M2"
   .STARTUP_WAIT("TRUE")    // Delay config DONE until DCM_CLKGEN LOCKED (TRUE/FALSE)
)
DCM_CLKGEN_f_ref (
   .CLKFX(/* open */),         // 1-bit output: Generated clock output
   .CLKFX180(/* open */),   // 1-bit output: Generated clock output 180 degree out of phase from CLKFX.
   .CLKFXDV(clk_ref),     // 1-bit output: Divided clock output
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

// CPU and RMII 50MHz clock gen clk = clk_i * `CPU_PLL_MULTIPLYER / `CPU_CLOCK_DEVIDER
DCM_CLKGEN #(
   .CLKFXDV_DIVIDE(2),       // CLKFXDV divide value (2, 4, 8, 16, 32)
   .CLKFX_DIVIDE(`CPU_CLOCK_DEVIDER),         // Divide value - D - (1-256)
   .CLKFX_MD_MAX(`CPU_PLL_MULTIPLYER * 2 / `CPU_CLOCK_DEVIDER),       // Specify maximum M/D ratio for timing anlysis
   .CLKFX_MULTIPLY(`CPU_PLL_MULTIPLYER * 2),       // Multiply value - M - (2-256)
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

ODDR2
#(
   .DDR_ALIGNMENT("NONE"), // Sets output alignment to "NONE", "C0" or "C1"
   .INIT(1'b0),    // Sets initial state of the Q output to 1'b0 or 1'b1
   .SRTYPE("SYNC") // Specifies "SYNC" or "ASYNC" set/reset
) RMII_clock_provider (
   .Q(rmii_clk_to_PHY_o),   // 1-bit DDR output data
   .C0(~sys_clk_o),   // 1-bit clock input
   .C1(sys_clk_o),   // 1-bit clock input
   .CE(1'b1), // 1-bit clock enable input
   .D0(1'b1), // 1-bit data input (associated with C0)
   .D1(1'b0), // 1-bit data input (associated with C1)
   .R(1'b0),   // 1-bit reset input
   .S(1'b0)    // 1-bit set input
);

assign rmii_logick_clk_o = sys_clk_o;

endmodule
