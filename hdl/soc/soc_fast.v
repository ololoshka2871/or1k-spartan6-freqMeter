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
    input  wire [31:0]                  adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [31:0]                  dat_i,         // data input
    output wire [31:0]                  dat_o,         // data output
    output wire				ack_o,         // normal bus termination
    output wire                         stall_o,       // stall
    input  wire                         sel_i,         //
    input  wire [2:0]                   cti_i,

    // Freqmeters
    input  wire                         F_master,      // частота образцовая
    input  wire [INPUTS_COUNT - 1:0]    F_in,          // входы для частоты
    output wire [MASER_FREQ_COUNTER_LEN-1:0] devided_clocks, // clk_i деленная на 2, 4, ... 2^MASER_FREQ_COUNTER_LEN

    // MII interface
    input  wire                         phy_tx_clk_i,
    output wire [3:0]                   phy_tx_data_o,
    output wire                         phy_tx_en_o,
    output wire                         phy_tx_er_o,
    input  wire                         phy_rx_clk_i,
    input  wire [3:0]                   phy_rx_data_i,
    input  wire                         phy_dv_i,
    input  wire                         phy_rx_er_i,
    input  wire                         phy_col_i,
    input  wire                         phy_crs_i,
    output wire                         phy_mii_clk_o,
    inout  wire                         phy_mii_data_io,

    output wire [2:0]                   interrupts_o
);

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
wire [3:0]          ethernet_ctl_sel;
wire                ethernet_ctl_we;
wire                ethernet_ctl_stb;
wire                ethernet_ctl_cyc;
wire [2:0]          ethernet_ctl_cti;
wire                ethernet_ctl_ack;
wire                ethernet_ctl_stall;

// Data Memory 2 (0x11200000 - 0x112FFFFF)
wire [31:0]         ethernet_txbuf_addr;
wire [31:0]         ethernet_txbuf_data_r;
wire [31:0]         ethernet_txbuf_data_w;
wire [3:0]          ethernet_txbuf_sel;
wire                ethernet_txbuf_we;
wire                ethernet_txbuf_stb;
wire                ethernet_txbuf_cyc;
wire [2:0]          ethernet_txbuf_cti;
wire                ethernet_txbuf_ack;
wire                ethernet_txbuf_stall;

// Data Memory 2 (0x11300000 - 0x113FFFFF)
wire [31:0]         ethernet_rxbuf_addr_o;
wire [31:0]         ethernet_rxbuf_data_r;
wire [31:0]         ethernet_rxbuf_data_w;
wire [3:0]          ethernet_rxbuf_sel_o;
wire                ethernet_rxbuf_we_o;
wire                ethernet_rxbuf_stb_o;
wire                ethernet_rxbuf_cyc_o;
wire [2:0]          ethernet_rxbuf_cti_o;
wire                ethernet_rxbuf_ack_i;
wire                ethernet_rxbuf_stall_i;

//------------------------------------------------------------------------------

wire                freqmeter_inta;
wire                ethernat_rx_int;
wire                ethernat_tx_int;

//------------------------------------------------------------------------------

assign interrupts = {ethernat_rx_int, ethernat_tx_int, freqmeter_inta};

// muxer
dmem_mux3
#(
    .ADDR_MUX_START()
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
    .out1_addr_o(ethernet_ctl_addr_o),
    .out1_data_o(ethernet_ctl_data_w),
    .out1_data_i(ethernet_ctl_data_r),
    .out1_sel_o(ethernet_ctl_sel_o),
    .out1_we_o(ethernet_ctl_we_o),
    .out1_stb_o(ethernet_ctl_stb_o),
    .out1_cyc_o(ethernet_ctl_cyc_o),
    .out1_cti_o(ethernet_ctl_cti_o),
    .out1_ack_i(ethernet_ctl_ack_i),
    .out1_stall_i(ethernet_ctl_stall_i),

    // 0x11200000 - 0x112FFFFF
    .out2_addr_o(ethernet_txbuf_addr_o),
    .out2_data_o(ethernet_txbuf_data_w),
    .out2_data_i(ethernet_txbuf_data_r),
    .out2_sel_o(ethernet_txbuf_sel_o),
    .out2_we_o(ethernet_txbuf_we_o),
    .out2_stb_o(ethernet_txbuf_stb_o),
    .out2_cyc_o(ethernet_txbuf_cyc_o),
    .out2_cti_o(ethernet_txbuf_cti_o),
    .out2_ack_i(ethernet_txbuf_ack_i),
    .out2_stall_i(ethernet_txbuf_stall_i),

    // 0x11300000 - 0x113FFFFF
    .out3_addr_o(ethernet_rxbuf_addr_o),
    .out3_data_o(ethernet_rxbuf_data_w),
    .out3_data_i(ethernet_rxbuf_data_r),
    .out3_sel_o(ethernet_rxbuf_sel_o),
    .out3_we_o(ethernet_rxbuf_we_o),
    .out3_stb_o(ethernet_rxbuf_stb_o),
    .out3_cyc_o(ethernet_rxbuf_cyc_o),
    .out3_cti_o(ethernet_rxbuf_cti_o),
    .out3_ack_i(ethernet_rxbuf_ack_i),
    .out3_stall_i(ethernet_rxbuf_stall_i),

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
freqmeters
#(
    .INPUTS_COUNT(INPUTS_COUNT),
    .MASER_FREQ_COUNTER_LEN(MASER_FREQ_COUNTER_LEN),
    .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
) fm (
    .clk_i(clk),
    .rst_i(reset),
    .cyc_i(freqmeter_cyc),
    .stb_i(freqmeter_stb),
    .adr_i(freqmeter_addr),
    .we_i(freqmeter_we),
    .dat_i(freqmeter_data_w),
    .dat_o(freqmeter_data_r),
    .ack_o(freqmeter_ack),
    .inta_o(freqmeter_inta),

    .F_master(F_master),
    .F_in(F_in),

    .devided_clocks(devided_clocks)
);


// ethernet
minmac
#(
    .csr_addr(4'h0)
) ethernet (
    .sys_clk(clk),
    .sys_rst(reset),

    .csr_a(ethernet_ctl_addr_o),
    .csr_we(ethernet_ctl_we_o),
    .csr_di(ethernet_ctl_data_w),
    .csr_do(ethernet_ctl_data_r),

    .irq_rx(ethernat_rx_int),
    .irq_tx(ethernat_tx_int),

    // ressive memory port
    .wbrx_adr_o(),
    .wbrx_cti_o(),
    .wbrx_cyc_o(),
    .wbrx_stb_o(),
    .wbrx_ack_i(),
    .wbrx_dat_o(),

    // transmitt memory port
    .wbtx_adr_o(),
    .wbtx_cti_o(),
    .wbtx_cyc_o(),
    .wbtx_stb_o(),
    .wbtx_ack_i(),
    .wbtx_dat_i(),

    .phy_tx_clk(phy_tx_clk_i),
    .phy_tx_data(phy_tx_data_o),
    .phy_tx_en(phy_tx_en_o),
    .phy_tx_er(phy_tx_er_o),
    .phy_rx_clk(phy_rx_clk_i),
    .phy_rx_data(phy_rx_data_i),
    .phy_dv(phy_dv_i),
    .phy_rx_er(phy_rx_er_i),
    .phy_col(phy_col_i),
    .phy_crs(phy_crs_i),
    .phy_mii_clk(phy_mii_clk_o),
    .phy_mii_data(phy_mii_data_io)
);

endmodule
