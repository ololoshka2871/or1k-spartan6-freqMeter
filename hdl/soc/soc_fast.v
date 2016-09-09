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
    input  wire [3:0]                   sel_i,         // byte sellect
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

//------------------------------------------------------------------------------

// ethernet frame size <= 1530 bytes
// need place to 4 frames
parameter ETHERNET_FRAME_SIZE = 1530;
parameter MEMORY_BLOCK_SIZE = 18 * 1024 / 8;
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

// Data Memory 3 (0x11300000 - 0x113FFFFF)
wire [31:0]         ethernet_rxbuf_addr;
wire [31:0]         ethernet_rxbuf_data_r;
wire [31:0]         ethernet_rxbuf_data_w;
wire [3:0]          ethernet_rxbuf_sel;
wire                ethernet_rxbuf_we;
wire                ethernet_rxbuf_stb;
wire                ethernet_rxbuf_cyc;
wire [2:0]          ethernet_rxbuf_cti;
wire                ethernet_rxbuf_ack;
wire                ethernet_rxbuf_stall;

//------------------------------------------------------------------------------

wire [31:0]         wbrx_adr;
wire                wbrx_cyc;
wire                wbrx_stb;
wire                wbrx_ack;
wire [31:0]         wbrx_dat;

wire [31:0]         wbtx_adr;
wire                wbtx_cyc;
wire                wbtx_stb;
wire                wbtx_ack;
wire [31:0]         wbtx_dat;

//------------------------------------------------------------------------------

wire                freqmeter_inta;
wire                ethernat_rx_int;
wire                ethernat_tx_int;

//------------------------------------------------------------------------------

assign interrupts = {ethernat_rx_int, ethernat_tx_int, freqmeter_inta};

// muxer
dmem_mux4
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
    .out1_addr_o(ethernet_ctl_addr),
    .out1_data_o(ethernet_ctl_data_w),
    .out1_data_i(ethernet_ctl_data_r),
    .out1_sel_o(ethernet_ctl_sel),
    .out1_we_o(ethernet_ctl_we),
    .out1_stb_o(ethernet_ctl_stb),
    .out1_cyc_o(ethernet_ctl_cyc),
    .out1_cti_o(ethernet_ctl_cti),
    .out1_ack_i(ethernet_ctl_ack),
    .out1_stall_i(ethernet_ctl_stall),

    // 0x11200000 - 0x112FFFFF
    .out2_addr_o(ethernet_txbuf_addr),
    .out2_data_o(ethernet_txbuf_data_w),
    .out2_data_i(ethernet_txbuf_data_r),
    .out2_sel_o(ethernet_txbuf_sel),
    .out2_we_o(ethernet_txbuf_we),
    .out2_stb_o(ethernet_txbuf_stb),
    .out2_cyc_o(ethernet_txbuf_cyc),
    .out2_cti_o(ethernet_txbuf_cti),
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
    .out3_cti_o(ethernet_rxbuf_cti),
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
freqmeters
#(
    .INPUTS_COUNT(INPUTS_COUNT),
    .MASER_FREQ_COUNTER_LEN(MASER_FREQ_COUNTER_LEN),
    .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
) fm (
    .clk_i(clk_i),
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
minimac
#(
    .csr_addr(4'h0)
) ethernet (
    .sys_clk(clk_i),
    .sys_rst(reset),

    .csr_a(ethernet_ctl_addr),
    .csr_we(ethernet_ctl_we),
    .csr_di(ethernet_ctl_data_w),
    .csr_do(ethernet_ctl_data_r),

    .irq_rx(ethernat_rx_int),
    .irq_tx(ethernat_tx_int),

    // ressive memory port
    .wbrx_adr_o(wbrx_adr),
    .wbrx_cti_o(/*open*/),
    .wbrx_cyc_o(wbrx_cyc),
    .wbrx_stb_o(wbrx_stb),
    .wbrx_ack_i(wbrx_ack),
    .wbrx_dat_o(wbrx_dat),

    // transmitt memory port
    .wbtx_adr_o(wbtx_adr),
    .wbtx_cti_o(/*open*/),
    .wbtx_cyc_o(wbtx_cyc),
    .wbtx_stb_o(wbtx_stb),
    .wbtx_ack_i(wbtx_ack),
    .wbtx_dat_i(wbtx_dat),

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

wb_dp_ram
#(
    .NUM_OF_18Kb_TO_USE(MEMORY_SIZE_BLOCKS),
    .DATA_WIDTH(32),
    .ADDR_WIDTH(MEMORY_ADDR_WIDTH)
)
rx_buffer (
    // port A - minimac
    .a_clk(phy_rx_clk_i),
    .a_adr_i(wbrx_adr[MEMORY_ADDR_WIDTH-1:0]),
    .a_dat_i(wbrx_dat),
    .a_dat_o(/* open */),
    .a_we_i(1'b1),
    .a_sel_i(4'b1111),
    .a_stb_i(wbrx_stb),
    .a_ack_o(wbrx_ack),
    .a_cyc_i(wbrx_cyc),
    .a_stall_o(/* open */),

    // port B - system bus
    .b_clk(clk_i),
    .b_adr_i(ethernet_rxbuf_addr[MEMORY_ADDR_WIDTH-1:0]),
    .b_dat_i(ethernet_rxbuf_data_w),
    .b_dat_o(ethernet_rxbuf_data_r),
    .b_we_i(ethernet_rxbuf_we),
    .b_sel_i(ethernet_rxbuf_sel),
    .b_stb_i(ethernet_rxbuf_stb),
    .b_ack_o(ethernet_rxbuf_ack),
    .b_cyc_i(ethernet_rxbuf_cyc),
    .b_stall_o(ethernet_rxbuf_stall)
),
tx_buffer (
    // port A - minimac
    .a_clk(phy_tx_clk_i),
    .a_adr_i(wbtx_adr[MEMORY_ADDR_WIDTH-1:0]),
    .a_dat_i(32'h00000000),
    .a_dat_o(wbtx_dat),
    .a_we_i(1'b0),
    .a_sel_i(4'b1111),
    .a_stb_i(wbtx_stb),
    .a_ack_o(wbtx_ack),
    .a_cyc_i(wbtx_cyc),
    .a_stall_o(/* open */),

    // port B - system bus
    .b_clk(clk_i),
    .b_adr_i(ethernet_txbuf_addr[MEMORY_ADDR_WIDTH-1:0]),
    .b_dat_i(ethernet_txbuf_data_w),
    .b_dat_o(ethernet_txbuf_data_r),
    .b_we_i(ethernet_txbuf_we),
    .b_sel_i(ethernet_txbuf_sel),
    .b_stb_i(ethernet_txbuf_stb),
    .b_ack_o(ethernet_txbuf_ack),
    .b_cyc_i(ethernet_txbuf_cyc),
    .b_stall_o(ethernet_txbuf_stall)
);


endmodule
