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

module myminimac_rx
#(
    parameter MTU                   = 1530,
    parameter SLOTS_COUNT           = 4,                // memory to alocate
    parameter MEM_UNITS_TO_ALLOC    = 1,//$rtoi($ceil(MTU * $itor(SLOTS_COUNT) / (`MEMORY_UNIT_SIZE / 8))),
    parameter ADDR_LEN              = $clog2(MEM_UNITS_TO_ALLOC * `MEMORY_UNIT_SIZE)
) (
    input                           sys_clk,            // System clock
    input                           sys_rst,            // System reset

    input  wire [31:0]              rx_mem_adr_i,       // ADR_I() address
    input  wire [31:0]              rx_mem_dat_i,       // DAT_I() data in
    output wire [31:0]              rx_mem_dat_o,       // DAT_O() data out
    input  wire                     rx_mem_we_i,        // WE_I write enable input
    input  wire [3:0]               rx_mem_sel_i,       // SEL_I() select input
    input  wire                     rx_mem_stb_i,       // STB_I strobe input
    output wire                     rx_mem_ack_o,       // ACK_O acknowledge output
    input  wire                     rx_mem_cyc_i,       // CYC_I cycle input
    output wire                     rx_mem_stall_o,     // incorrect address

    input                           rx_rst,             // reset rx request
    input                           rx_valid,           // rx memory ready to write
    input       [ADDR_LEN-1:0]      rx_adr,             // address to write
    output reg                      rx_resetcount,      // address reset request
    output                          rx_incrcount,       // address increment request
    output reg                      rx_endframe,        // ressive end request
    output                          rx_error,           // rx error occured

    input                           phy_rmii_clk,       // 50 MHz
    input       [1:0]               phy_rmii_rx_data,   // RMII data
    input                           phy_rmii_crs        // RMII ressiving data
);

parameter RMII_BUS_WIDTH = 2;
parameter COUNTER_WIDTH = $clog2(8 / RMII_BUS_WIDTH);

reg [5:0] input_data;  // shift register to ressive
reg [COUNTER_WIDTH-1:0] ressive_counter;
reg ressiving_frame;
reg rx_byte_error;
reg wr_request;

reg [7:0] data_to_write_memory;

wire shifting_in_progress = (ressive_counter != (8 / RMII_BUS_WIDTH));
wire memory_error;

wb_dma_ram
#(
    .NUM_OF_MEM_UNITS_TO_USE(MEM_UNITS_TO_ALLOC),

    .WB_DATA_WIDTH(32)
) rx_ram (
    .wb_clk(sys_clk),
    .wb_adr_i(rx_mem_adr_i),
    .wb_dat_i(rx_mem_dat_i),
    .wb_dat_o(rx_mem_dat_o),
    .wb_we_i(rx_mem_we_i),
    .wb_sel_i(rx_mem_sel_i),
    .wb_stb_i(rx_mem_stb_i),
    .wb_ack_o(rx_mem_ack_o),
    .wb_cyc_i(rx_mem_cyc_i),
    .wb_stall_o(rx_mem_stall_o),

    .rawp_clk(phy_rmii_clk),
    .rawp_adr_i(rx_adr),
    .rawp_dat_i(data_to_write_memory),
    .rawp_dat_o(/* open */),
    .rawp_we_i(wr_request),
    .rawp_stall_o(memory_error)
);

always @(posedge phy_rmii_clk) begin
    if (sys_rst | rx_rst) begin
        // reset
        input_data <= 7'b0;
        ressive_counter <= 0;
        ressiving_frame <= 1'b0;
        rx_resetcount <= 1'b0;
        wr_request <= 1'b0;
    end else begin
        rx_endframe <= 1'b0;
        rx_byte_error <= 1'b0;
        wr_request <= 1'b0;
        data_to_write_memory <= {input_data, phy_rmii_rx_data};
        input_data <= {input_data[5:2], phy_rmii_rx_data};
        rx_resetcount <= 1'b0;

        if (phy_rmii_crs) begin
            // start or continue ressiving
            if (ressiving_frame) begin
                ressive_counter <= ressive_counter + 1;
                if (~shifting_in_progress) begin
                    wr_request <= rx_valid; // write request if control says "ok"
                    ressive_counter <= 1;
                end
            end else
                // wait preamble 01 to start
                rx_resetcount <= 1'b1;
                if (phy_rmii_rx_data != 2'b00) begin
                    // start ressiving
                    ressiving_frame <= 1'b1;
                    ressive_counter <= 0;
                end
        end else
            if (ressiving_frame) begin
                // make stop
                ressiving_frame <= 1'b0;
                rx_endframe   <= ~shifting_in_progress;
                rx_byte_error <=  shifting_in_progress;
            end
    end
end

assign rx_error = rx_byte_error | memory_error; // drop pocket
assign rx_incrcount = wr_request;

endmodule
