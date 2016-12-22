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
    input  wire [31:0]                  adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [31:0]                  dat_i,         // data input
    output wire [31:0]                  dat_o,         // data output
    output wire				ack_o,         // normal bus termination
    output wire                         stall_o,       // stall
    input  wire [3:0]                   sel_i,         // byte sellect
    input  wire [2:0]                   cti_i,

    // Freqmeters
    input  wire                         F_master,      // частота образцовая
    input  wire [INPUTS_COUNT - 1:0]    F_in,          // входы для частоты
    output wire [MASER_FREQ_COUNTER_LEN-1:0] devided_clocks, // clk_i деленная на 2, 4, ... 2^MASER_FREQ_COUNTER_LEN

`ifdef ETHERNET_ENABLED
    // RMII interface
    input  wire                         phy_rmii_clk,   // 50 MHZ input
    input  wire                         phy_rmii_crs,   // Ressiver ressiving data
    output wire [1:0]                   phy_rmii_tx_data,// transmit data bis
    input  wire [1:0]                   phy_rmii_rx_data,// ressive data bus
    output wire                         phy_tx_en,      // transmitter enable
`endif

    output wire [2:0]                   interrupts_o
);

//------------------------------------------------------------------------------

// ethernet frame size <= 1530 bytes
// need place to 4 frames
parameter ETHERNET_FRAME_SIZE = 1530;
parameter MEMORY_BLOCK_SIZE = `MEMORY_UNIT_SIZE / 8;
parameter MEMORY_SIZE_BLOCKS = $rtoi($ceil(ETHERNET_FRAME_SIZE * $itor(4) / MEMORY_BLOCK_SIZE));
parameter MEMORY_SIZE_BYTES = MEMORY_SIZE_BLOCKS * MEMORY_BLOCK_SIZE;
parameter MEMORY_ADDR_WIDTH = $clog2(MEMORY_SIZE_BYTES);

//------------------------------------------------------------------------------

// Data Memory 0 (0x11000000 - 0x110FFFFF)
wire [31:0]         freqmeter_addr;
wire [31:0]         freqmeter_data_r;
wire [31:0]         freqmeter_data_w;
wire                freqmeter_we;
wire                freqmeter_stb;
wire                freqmeter_cyc;
wire                freqmeter_ack;

// Data Memory 1 (0x11100000 - 0x111FFFFF)
wire [31:0]         ethernet_ctl_addr;
wire [31:0]         ethernet_ctl_data_r;
wire [31:0]         ethernet_ctl_data_w;
wire                ethernet_ctl_we;
wire                ethernet_ctl_ack;
wire                ethernet_ctl_stb;
wire                ethernet_ctl_cyc;

// Data Memory 2 (0x11200000 - 0x112FFFFF)
wire [31:0]         ethernet_txbuf_addr;
wire [31:0]         ethernet_txbuf_data_r;
wire [31:0]         ethernet_txbuf_data_w;
wire [3:0]          ethernet_txbuf_sel;
wire                ethernet_txbuf_we;
wire                ethernet_txbuf_stb;
wire                ethernet_txbuf_cyc;
wire                ethernet_txbuf_ack;
wire                ethernet_txbuf_stall;

// Data Memory 3 (0x11300000 - 0x113FFFFF)
wire [31:0]         ethernet_rxbuf_addr;
wire [31:0]         ethernet_rxbuf_data_r;
wire [31:0]         ethernet_rxbuf_data_w;
wire [3:0]          ethernet_rxbuf_sel;
wire                ethernet_rxbuf_we;
wire                ethernet_rxbuf_stb;
wire                ethernet_rxbuf_cyc;
wire                ethernet_rxbuf_ack;
wire                ethernet_rxbuf_stall;

//------------------------------------------------------------------------------

wire                freqmeter_inta;
wire                ethernat_rx_int;
wire                ethernat_tx_int;

//------------------------------------------------------------------------------

assign interrupts_o = {ethernat_rx_int, ethernat_tx_int, freqmeter_inta};

// muxer
dmem_mux4
#(
    .ADDR_MUX_START(20)
) u_dmux (
    // Outputs
    // 0x11000000 - 0x110FFFFF
    .out0_addr_o(freqmeter_addr),
    .out0_data_o(freqmeter_data_w),
    .out0_data_i(freqmeter_data_r),
    .out0_sel_o(/* open */),
    .out0_we_o(freqmeter_we),
    .out0_stb_o(freqmeter_stb),
    .out0_cyc_o(freqmeter_cyc),
    .out0_cti_o(/* open */),
    .out0_ack_i(freqmeter_ack),
    .out0_stall_i(1'b0),

    // 0x11100000 - 0x111FFFFF
    .out1_addr_o(ethernet_ctl_addr),
    .out1_data_o(ethernet_ctl_data_w),
    .out1_data_i(ethernet_ctl_data_r),
    .out1_sel_o(/*open*/),
    .out1_we_o(ethernet_ctl_we),
    .out1_stb_o(ethernet_ctl_stb),
    .out1_cyc_o(ethernet_ctl_cyc),
    .out1_cti_o(/*open*/),
    .out1_ack_i(ethernet_ctl_ack),
    .out1_stall_i(1'b0),

    // 0x11200000 - 0x112FFFFF
    .out2_addr_o(ethernet_txbuf_addr),
    .out2_data_o(ethernet_txbuf_data_w),
    .out2_data_i(ethernet_txbuf_data_r),
    .out2_sel_o(ethernet_txbuf_sel),
    .out2_we_o(ethernet_txbuf_we),
    .out2_stb_o(ethernet_txbuf_stb),
    .out2_cyc_o(ethernet_txbuf_cyc),
    .out2_cti_o(/*open*/),
    .out2_ack_i(ethernet_txbuf_ack),
    .out2_stall_i(ethernet_txbuf_stall),

    // 0x11300000 - 0x113FFFFF
    .out3_addr_o(ethernet_rxbuf_addr),
    .out3_data_o(ethernet_rxbuf_data_w),
    .out3_data_i(ethernet_rxbuf_data_r),
    .out3_sel_o(ethernet_rxbuf_sel),
    .out3_we_o(ethernet_rxbuf_we),
    .out3_stb_o(ethernet_rxbuf_stb),
    .out3_cyc_o(ethernet_rxbuf_cyc),
    .out3_cti_o(/*open*/),
    .out3_ack_i(ethernet_rxbuf_ack),
    .out3_stall_i(ethernet_rxbuf_stall),

    // Input 0x11000000 - 0x11FFFFFF
    .mem_addr_i(adr_i),
    .mem_data_i(dat_i),
    .mem_data_o(dat_o),
    .mem_sel_i(sel_i),
    .mem_we_i(we_i),
    .mem_stb_i(stb_i),
    .mem_cyc_i(cyc_i),
    .mem_cti_i(cti_i),
    .mem_ack_o(ack_o),
    .mem_stall_o(stall_o)
);

// Freq meter
freqmeters3
#(
    .INPUTS_COUNT(INPUTS_COUNT),
    .MASER_FREQ_COUNTER_LEN(MASER_FREQ_COUNTER_LEN),
    .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
) fm (
    .clk_i(clk_i),
    .rst_i(rst_i),
    .cyc_i(freqmeter_cyc),
    .stb_i(freqmeter_stb),
    .adr_i(freqmeter_addr[9:0]),
    .we_i(freqmeter_we),
    .dat_i(freqmeter_data_w),
    .dat_o(freqmeter_data_r),
    .ack_o(freqmeter_ack),
    .inta_o(freqmeter_inta),

    .F_master(F_master),
    .F_in(F_in),

    .devided_clocks(devided_clocks)
);

`ifdef ETHERNET_ENABLED
// ethernet
myminimac
#(
    .RX_MEMORY_BASE(32'h11300000),
    .TX_MEMORY_BASE(32'h11200000)
) ethernet (
    .sys_clk(clk_i),
    .sys_rst(rst_i),

    .irq_rx(ethernat_rx_int),
    .irq_tx(ethernat_tx_int),

    .csr_adr_i(ethernet_ctl_addr),
    .csr_we_i(ethernet_ctl_we),
    .csr_dat_i(ethernet_ctl_data_w),
    .csr_dat_o(ethernet_ctl_data_r),
    .csr_ack_o(ethernet_ctl_ack),
    .csr_stb_i(ethernet_ctl_stb),
    .csr_cyc_i(ethernet_ctl_cyc),

    .rx_mem_adr_i(ethernet_rxbuf_addr),
    .rx_mem_dat_i(ethernet_rxbuf_data_w),
    .rx_mem_dat_o(ethernet_rxbuf_data_r),
    .rx_mem_we_i(ethernet_rxbuf_we),
    .rx_mem_sel_i(ethernet_rxbuf_sel),
    .rx_mem_stb_i(ethernet_rxbuf_stb),
    .rx_mem_ack_o(ethernet_rxbuf_ack),
    .rx_mem_cyc_i(ethernet_rxbuf_cyc),
    .rx_mem_stall_o(ethernet_rxbuf_stall),

    .tx_mem_adr_i(ethernet_txbuf_addr),
    .tx_mem_dat_i(ethernet_txbuf_data_w),
    .tx_mem_dat_o(ethernet_txbuf_data_r),
    .tx_mem_we_i(ethernet_txbuf_we),
    .tx_mem_sel_i(ethernet_txbuf_sel),
    .tx_mem_stb_i(ethernet_txbuf_stb),
    .tx_mem_ack_o(ethernet_txbuf_ack),
    .tx_mem_cyc_i(ethernet_txbuf_cyc),
    .tx_mem_stall_o(ethernet_txbuf_stall),

    .phy_rmii_clk(phy_rmii_clk),
    .phy_rmii_crs(phy_rmii_crs),
    .phy_rmii_tx_data(phy_rmii_tx_data),
    .phy_rmii_rx_data(phy_rmii_rx_data),
    .phy_tx_en(phy_tx_en)
);
`else
assign ethernat_rx_int = 1'b0;

assign ethernat_tx_int = 1'b0;
assign ethernet_ctl_data_r = 32'b0;

assign ethernet_rxbuf_data_r = 32'b0;
assign ethernet_rxbuf_ack = 1'b0;
assign ethernet_rxbuf_stall = 1'b1;

assign ethernet_txbuf_data_r = 32'b0;
assign ethernet_txbuf_ack = 1'b0;
assign ethernet_txbuf_stall = 1'b1;
`endif

endmodule
