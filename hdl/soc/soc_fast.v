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
//* 3. Neither the name NuttX nor the names of its contributors may be
//*    used to endorse or promote products derived from this software
//*    without specific prior written permission.
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

module soc_fast
#(
    parameter   INPUTS_COUNT            = 24, // количество входов (1 - 32)
    parameter   MASER_FREQ_COUNTER_LEN  = 30, // длина регистра, считающего опорную частоту
    parameter   INPUT_FREQ_COUNTER_LEN  = 24  // длина региста, считающего входную частоту
) (
    // WISHBONE bus slave interface
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)
    input  wire				cyc_i,         // cycle
    input  wire				stb_i,         // strobe
    input  wire [8:0]                   adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [`WB_DATA_WIDTH - 1:0]	dat_i,         // data input
    output wire [`WB_DATA_WIDTH - 1:0]	dat_o,         // data output
    output reg 				ack_o,         // normal bus termination

    // Freqmeters
    input  wire                         F_master,      // частота образцовая
    input  wire [INPUTS_COUNT - 1:0]    F_in,          // входы для частоты
    output wire [MASER_FREQ_COUNTER_LEN-1:0] devided_clocks // clk_i деленная на 2, 4, ... 2^MASER_FREQ_COUNTER_LEN

    // MII interface
    input  wire                         phy_tx_clk,
    output wire [3:0]                   phy_tx_data,
    output wire                         phy_tx_en,
    output wire                         phy_tx_er,
    input  wire                         phy_rx_clk,
    input  wire [3:0]                   phy_rx_data,
    input  wire                         phy_dv,
    input  wire                         phy_rx_er,
    input  wire                         phy_col,
    input  wire                         phy_crs,
    output wire                         phy_mii_clk,
    inout  wire                         phy_mii_data
);

// Data Memory 0 (0x10000000 - 0x10FFFFFF)
output [31:0]       dmem0_addr_o,
output [31:0]       dmem0_data_o,
input [31:0]        dmem0_data_i,
output [3:0]        dmem0_sel_o,
output              dmem0_we_o,
output              dmem0_stb_o,
output              dmem0_cyc_o,
output [2:0]        dmem0_cti_o,
input               dmem0_ack_i,
input               dmem0_stall_i,

// Data Memory 1 (0x11000000 - 0x11FFFFFF)
output [31:0]       dmem1_addr_o,
output [31:0]       dmem1_data_o,
input [31:0]        dmem1_data_i,
output [3:0]        dmem1_sel_o,
output              dmem1_we_o,
output              dmem1_stb_o,
output              dmem1_cyc_o,
output [2:0]        dmem1_cti_o,
input               dmem1_ack_i,
input               dmem1_stall_i,

// Data Memory 2 (0x12000000 - 0x12FFFFFF)
output [31:0]       dmem2_addr_o,
output [31:0]       dmem2_data_o,
input [31:0]        dmem2_data_i,
output [3:0]        dmem2_sel_o,
output              dmem2_we_o,
output              dmem2_stb_o,
output              dmem2_cyc_o,
output [2:0]        dmem2_cti_o,
input               dmem2_ack_i,
input               dmem2_stall_i,

// muxer
dmem_mux3
#(
    .ADDR_MUX_START()
) u_dmux (

)

// Freq meter
freqmeters
#(
    .INPUTS_COUNT(`F_INPUTS_COUNT),
    .MASER_FREQ_COUNTER_LEN(`MASER_FREQ_COUNTER_LEN),
    .INPUT_FREQ_COUNTER_LEN(`INPUT_FREQ_COUNTER_LEN)
) fm (
    .clk_i(clk),
    .rst_i(reset),
    .cyc_i(freqmeter_cyc),
    .stb_i(freqmeter_stb),
    .adr_i(freqmeter_addr[8:0]),
    .we_i(freqmeter_we),
    .dat_i(freqmeter_data_w),
    .dat_o(freqmeter_data_r),
    .ack_o(freqmeter_ack),
    .inta_o(freqmeter_inta),

    .F_master(clk_ref),
    .F_in(Fin_inv_pars),

    .devided_clocks(devided_clocks)
);

endmodule
