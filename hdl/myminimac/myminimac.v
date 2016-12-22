//****************************************************************************
//*
//*   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
//*   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
//*   Based on: Milkymist VJ SoC 2007, 2008, 2009, 2010 Sebastien Bourdeauducq
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
//****************************************************************************/

`include "config.v"

module myminimac
#(
    parameter RX_MEMORY_BASE      = 32'h00000000,
    parameter TX_MEMORY_BASE      = 32'h10000000
) (
    input                           sys_clk,        // WISHBONE clock
    input                           sys_rst,        // GLOBAL RESET

    output                          irq_rx,         // RX interrupt
    output                          irq_tx,         // TX interrupt

    input  wire [31:0]              csr_adr_i,      // control logic addr
    input  wire                     csr_we_i,       // control logick write enable
    input  wire [31:0]              csr_dat_i,      // control logick data input
    output wire [31:0]              csr_dat_o,      // control logick data output
    output                          csr_ack_o,      // control logick ack
    input                           csr_stb_i,      // control logick strobe
    input                           csr_cyc_i,      // control logick select

    // system bus port A (rx memory)
    input  wire [31:0]              rx_mem_adr_i,    // ADR_I() address
    input  wire [31:0]              rx_mem_dat_i,    // DAT_I() data in
    output wire [31:0]              rx_mem_dat_o,    // DAT_O() data out
    input  wire                     rx_mem_we_i,     // WE_I write enable input
    input  wire [3:0]               rx_mem_sel_i,    // SEL_I() select input
    input  wire                     rx_mem_stb_i,    // STB_I strobe input
    output wire                     rx_mem_ack_o,    // ACK_O acknowledge output
    input  wire                     rx_mem_cyc_i,    // CYC_I cycle input
    output wire                     rx_mem_stall_o,  // incorrect address

    // system bus port B (tx memory)
    input  wire [31:0]              tx_mem_adr_i,   // ADR_I() address
    input  wire [31:0]              tx_mem_dat_i,   // DAT_I() data in
    output wire [31:0]              tx_mem_dat_o,   // DAT_O() data out
    input  wire                     tx_mem_we_i,    // WE_I write enable input
    input  wire [3:0]               tx_mem_sel_i,   // SEL_I() select input
    input  wire                     tx_mem_stb_i,   // STB_I strobe input
    output wire                     tx_mem_ack_o,   // ACK_O acknowledge output
    input  wire                     tx_mem_cyc_i,   // CYC_I cycle input
    output wire                     tx_mem_stall_o, // incorrect address

    // RMII
    input  wire                     phy_rmii_clk,   // 50 MHZ input
    input  wire                     phy_rmii_crs,   // Ressiver ressiving data
    output wire [1:0]               phy_rmii_tx_data,// transmit data bis
    input  wire [1:0]               phy_rmii_rx_data,// ressive data bus
    output wire                     phy_tx_en       // transmitter enable
);

parameter RX_SLOTS = 4;
parameter TX_SLOTS = 1;
parameter MTU = 1530;
parameter RX_ADDR_WIDTH = $clog2($rtoi($ceil(MTU * $itor(RX_SLOTS) /
    (`MEMORY_UNIT_SIZE / 8))) * `MEMORY_UNIT_SIZE / 8);
parameter TX_ADDR_WIDTH = $clog2($rtoi($ceil(MTU * $itor(TX_SLOTS) /
    (`MEMORY_UNIT_SIZE / 8))) * `MEMORY_UNIT_SIZE / 8);


parameter csr_do_len = $clog2(MTU);

wire rx_rst;
wire tx_rst;

wire rx_valid;
wire [RX_ADDR_WIDTH - 1:2] rx_adr;
wire rx_resetcount;
wire rx_incrcount;
wire rx_endframe;
wire rx_error;

wire tx_valid;
wire tx_last_byte;
wire [TX_ADDR_WIDTH-1:2] tx_adr;
wire tx_next;

myminimac_ctlif_cd2
#(
    .RX_MEMORY_BASE(RX_MEMORY_BASE),
    .TX_MEMORY_BASE(TX_MEMORY_BASE),
    .MTU(MTU),
    .RX_ADDR_WIDTH(RX_ADDR_WIDTH),
    .TX_ADDR_WIDTH(TX_ADDR_WIDTH)
) ctlif (
    .sys_clk(sys_clk),
    .sys_rst(sys_rst),

    .irq_rx(irq_rx),
    .irq_tx(irq_tx),

    .csr_a(csr_adr_i[5:0]),
    .csr_we(csr_we_i),
    .csr_di(csr_dat_i),
    .csr_do(csr_dat_o),
    .csr_ack(csr_ack_o),
    .csr_stb(csr_stb_i),
    .csr_cyc(csr_cyc_i),

    .rmii_clk_i(phy_rmii_clk),

    .rx_rst(rx_rst),
    .rx_valid(rx_valid),
    .rx_adr(rx_adr),
    .rx_resetcount(rx_resetcount),
    .rx_incrcount(rx_incrcount),
    .rx_endframe(rx_endframe),
    .rx_error(rx_error),

    .tx_rst(tx_rst),
    .tx_valid(tx_valid),
    .tx_last_byte(tx_last_byte),
    .tx_adr(tx_adr),
    .tx_next(tx_next)
);

myminimac_rx
#(
    .MTU(MTU),
    .SLOTS_COUNT(RX_SLOTS)
) rx(
    .sys_clk(sys_clk),
    .sys_rst(sys_rst),

    .rx_mem_adr_i(rx_mem_adr_i),
    .rx_mem_dat_i(rx_mem_dat_i),
    .rx_mem_dat_o(rx_mem_dat_o),
    .rx_mem_we_i(rx_mem_we_i),
    .rx_mem_sel_i(rx_mem_sel_i),
    .rx_mem_stb_i(rx_mem_stb_i),
    .rx_mem_ack_o(rx_mem_ack_o),
    .rx_mem_cyc_i(rx_mem_cyc_i),
    .rx_mem_stall_o(rx_mem_stall_o),

    .rx_rst(rx_rst),
    .rx_valid(rx_valid),
    .rx_adr(rx_adr),
    .rx_resetcount(rx_resetcount),
    .rx_incrcount(rx_incrcount),
    .rx_endframe(rx_endframe),
    .rx_error(rx_error),

    .phy_rmii_clk(phy_rmii_clk),
    .phy_rmii_rx_data(phy_rmii_rx_data),
    .phy_rmii_crs(phy_rmii_crs)
);

myminimac_tx
#(
    .MTU(MTU),
    .SLOTS_COUNT(TX_SLOTS)
) tx (
    .sys_clk(sys_clk),
    .sys_rst(sys_rst),

    .tx_mem_adr_i(tx_mem_adr_i[TX_ADDR_WIDTH-1:0]),
    .tx_mem_dat_i(tx_mem_dat_i),
    .tx_mem_dat_o(tx_mem_dat_o),
    .tx_mem_we_i(tx_mem_we_i),
    .tx_mem_sel_i(tx_mem_sel_i),
    .tx_mem_stb_i(tx_mem_stb_i),
    .tx_mem_ack_o(tx_mem_ack_o),
    .tx_mem_cyc_i(tx_mem_cyc_i),
    .tx_mem_stall_o(tx_mem_stall_o),

    .tx_rst(tx_rst),
    .tx_valid(tx_valid),
    .tx_last_byte_i(tx_last_byte),
    .tx_adr(tx_adr),
    .tx_next(tx_next),

    .phy_rmii_clk(phy_rmii_clk),
    .phy_tx_en(phy_tx_en),
    .phy_rmii_tx_data(phy_rmii_tx_data)
);

endmodule
