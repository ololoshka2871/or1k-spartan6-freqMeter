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

module myminimac_tx
#(
    parameter MTU                   = 1530,
    parameter SLOTS_COUNT           = 1,                // memory to alocate
    parameter MEM_UNITS_TO_ALLOC    = $rtoi($ceil(MTU * $itor(SLOTS_COUNT) / (`MEMORY_UNIT_SIZE / 8))),
    parameter ADDR_LEN              = $clog2(MEM_UNITS_TO_ALLOC * `MEMORY_UNIT_SIZE / 8)
) (
    input                           sys_clk,            // System clock
    input                           sys_rst,            // System reset

    input  wire [ADDR_LEN-1:0]      tx_mem_adr_i,       // ADR_I() address
    input  wire [31:0]              tx_mem_dat_i,       // DAT_I() data in
    output wire [31:0]              tx_mem_dat_o,       // DAT_O() data out
    input  wire                     tx_mem_we_i,        // WE_I write enable input
    input  wire [3:0]               tx_mem_sel_i,       // SEL_I() select input
    input  wire                     tx_mem_stb_i,       // STB_I strobe input
    output wire                     tx_mem_ack_o,       // ACK_O acknowledge output
    input  wire                     tx_mem_cyc_i,       // CYC_I cycle input
    output wire                     tx_mem_stall_o,     // incorrect address

    input                           tx_rst,             // reset Tx request
    input                           tx_valid,           // address is valid
    input                           tx_last_byte_i,     // last byte remaining
    input       [ADDR_LEN-1:2]      tx_adr,             // addres to read from
    output                          tx_next,            // request to increment address

    input                           phy_rmii_clk,       // 50 MHz
    output                          phy_tx_en,          // transmit enable
    output      [1:0]               phy_rmii_tx_data    // data to transmit
);

parameter MEMORY_DATA_WIDTH = 32;
parameter RMII_BUS_WIDTH = 2;

parameter PREAMBLE_END = 31;

parameter NORMAL_COUNTER_WIDTH = $clog2(MEMORY_DATA_WIDTH / RMII_BUS_WIDTH);
parameter COUNTER_WIDTH = $clog2(PREAMBLE_END);

reg tx_en;
reg sending_preamble;
reg byte_count_stop;
reg [ADDR_LEN - 1:2] mem_tx_addr;
reg [MEMORY_DATA_WIDTH - 1:0] transmit_data;  // shift register to transmitt
reg [COUNTER_WIDTH-1:0] transmit_counter;

wire [MEMORY_DATA_WIDTH - 1:0] data_from_memory;

wire read_from_memory = (transmit_counter[NORMAL_COUNTER_WIDTH - 1:0] == 0);
wire transmitted28bits = (transmit_counter == (MEMORY_DATA_WIDTH / 2) - 2);
wire byte_transferted = ~|transmit_counter[1:0];

wire transmission_started = ~tx_en & tx_valid;

`include "convert.v"

wire [MEMORY_DATA_WIDTH - 1:0] data_from_memory_norm =
    ether_bitorder_convert32(data_from_memory);

wb_dma_ram
#(
    .NUM_OF_MEM_UNITS_TO_USE(MEM_UNITS_TO_ALLOC),
    .INIT_FILE_NAME(`COUNT_TEST_MEMORY_IMAGE)
) tx_ram (
    .wb_clk(sys_clk),
    .wb_adr_i(tx_mem_adr_i),
    .wb_dat_i(tx_mem_dat_i),
    .wb_dat_o(tx_mem_dat_o),
    .wb_we_i(tx_mem_we_i),
    .wb_sel_i(tx_mem_sel_i),
    .wb_stb_i(tx_mem_stb_i),
    .wb_ack_o(tx_mem_ack_o),
    .wb_cyc_i(tx_mem_cyc_i),
    .wb_stall_o(tx_mem_stall_o),

    .rawp_clk(phy_rmii_clk),
    .rawp_adr_i(mem_tx_addr),
    .rawp_dat_i(32'd0),
    .rawp_dat_o(data_from_memory),
    .rawp_we_i(1'b0),
    .rawp_stall_o(/* open */)
);

always @(posedge phy_rmii_clk) begin
    if (sys_rst | tx_rst) begin
        transmit_counter <= 0;
        tx_en <= 0;
        mem_tx_addr <= 0;
        transmit_data <= 0;
        byte_count_stop <= 1'b0;
        sending_preamble <= 1'b0;
    end else begin
        tx_en <= tx_valid;

        if (tx_valid) begin
            if (~tx_en | sending_preamble) begin
                sending_preamble <= 1'b1;
                transmit_counter <= transmission_started ? 0 :
                    transmit_counter + 1;
                transmit_data[MEMORY_DATA_WIDTH - 1 -: 2] <= {
                    transmit_counter == PREAMBLE_END - 1, 1'b1};
                if (transmit_counter == PREAMBLE_END) begin
                    sending_preamble <= 1'b0;
                    transmit_data <= data_from_memory_norm;
                    transmit_counter <= 1;
                end
            end else begin
                if (read_from_memory) begin
                    transmit_data <= data_from_memory_norm;
                    transmit_counter <= 1;
                    if (tx_last_byte_i) begin
                        byte_count_stop <= 1'b1;
                        transmit_data[MEMORY_DATA_WIDTH - 1 -: 2] <= 2'b00;
                    end
                end else begin
                    if (tx_next & tx_last_byte_i) begin
                        byte_count_stop <= 1'b1;
                        transmit_data[MEMORY_DATA_WIDTH - 1 -: 2] <= 2'b00;
                    end else begin
                        transmit_data <= {transmit_data[MEMORY_DATA_WIDTH-1-2 : 0], 2'd0};
                    end
                    transmit_counter <= transmit_counter + 1;
                    if (transmitted28bits) begin
                        mem_tx_addr <= mem_tx_addr + 1;
                    end
                end
            end
        end else begin
            byte_count_stop <= 1'b0;
            mem_tx_addr <= tx_adr;
            transmit_counter <= 0;
            transmit_data[MEMORY_DATA_WIDTH - 1 -: 2] <= 2'b00;
        end
    end
end

assign phy_tx_en = tx_en & ~byte_count_stop;
assign tx_next = tx_en & byte_transferted & ~sending_preamble;
assign phy_rmii_tx_data = transmit_data[MEMORY_DATA_WIDTH - 1 -: 2];

endmodule

