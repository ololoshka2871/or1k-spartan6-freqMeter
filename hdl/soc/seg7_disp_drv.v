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
//* this module provides access to FPGA boot SPI flash
//*
//****************************************************************************/

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "config.v"

module seg7_disp_drv
#(
    parameter DIGITS_COUNT              = 4,
    parameter IS_COM_CATODE             = 1,
    parameter WB_DATA_WIDTH             = 32
) (
    // WISHBONE bus slave interface
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)
    input  wire				cyc_i,         // cycle
    input  wire				stb_i,         // strobe
    input  wire [7:0]                   adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [WB_DATA_WIDTH - 1:0]	dat_i,         // data input
    output wire [WB_DATA_WIDTH - 1:0]	dat_o,         // data output
    output wire				ack_o,         // normal bus termination

    // digits interface
    input  wire                         update_clock,  // digit switch clock
    output reg  [7:0]                   segments,      // segments drivers
    output reg  [DIGITS_COUNT - 1:0]    selectors      // digit selector
);

parameter  DIGIT_SELECTOR_SIZE          = $clog2(DIGITS_COUNT);

reg [7:0] digits_storage[DIGITS_COUNT - 1:0]; // memory

reg [7:0] dat_valid_o = {7{1'b0}};
reg ack_o_reg = 1'b0;

reg [DIGIT_SELECTOR_SIZE - 1:0] digit_selector = {DIGIT_SELECTOR_SIZE{1'b0}};

wire [DIGIT_SELECTOR_SIZE - 1:0] wb_addr_valid = adr_i[DIGIT_SELECTOR_SIZE - 1 + 2:2];
wire [7:0] dat_valid_i = dat_i[7:0];
wire a_incorrect_addr = (wb_addr_valid > (DIGITS_COUNT - 1));

assign dat_o = dat_valid_o;
assign ack_o = ack_o_reg;

integer i;

initial begin
                         //pGFEDCBA
    digits_storage[0] = 8'b01111100; // b
    digits_storage[1] = 8'b01011100; // o
    digits_storage[2] = 8'b01011100; // o
    digits_storage[3] = 8'b01111000; // t
    //for (i = 0; i < DIGITS_COUNT; i = i + 1)
    //begin
    //    digits_storage[i] = 8'b00000000;
    //end
end

// wishbone read/write
always @(posedge clk_i) begin
    ack_o_reg <= 1'b0;

    if (cyc_i & stb_i & ~ack_o) begin
        if (~a_incorrect_addr) begin
            if (we_i) begin
                digits_storage[wb_addr_valid] <= dat_valid_i;
            end
            dat_valid_o <= digits_storage[wb_addr_valid];
            ack_o_reg <= 1'b1;
        end
        else
        begin
            dat_valid_o <= 8'b0;
        end
    end
end

always @(posedge update_clock) begin
    if (digit_selector == DIGITS_COUNT - 1) begin
        digit_selector <= {DIGIT_SELECTOR_SIZE{1'b0}};
    end else begin
        digit_selector <= digit_selector + 1'b1;
    end
    segments <= IS_COM_CATODE ? digits_storage[digit_selector] :
        ~digits_storage[digit_selector];
    selectors <= IS_COM_CATODE ? ~(1'b1 << digit_selector) :
        1'b1 << digit_selector;
end

endmodule
