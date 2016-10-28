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

module multi_src_writer
#(
    parameter   INPUTS_COUNT            = 24, // количество входов (1 - 32)
    parameter   VALUE_WIDTH             = 32
) (
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)

    input wire [INPUTS_COUNT-1:0]       requests_i,    // write requests
    input wire [VALUE_WIDTH-1:0]        value_i,       // walue to write

    // memory bus
    input  wire                         access_clk,
    input  wire [8:0]                   access_adr_i,  // address
    output wire [VALUE_WIDTH-1:0]       access_dat_o   // data out
);

parameter ADDR_COUNTER_LEN = $clog2(INPUTS_COUNT);

//------------------------------------------------------------------------------

reg [ADDR_COUNTER_LEN-1:0]  addr_counter;
reg [INPUTS_COUNT-1:0]      request_checker;

//------------------------------------------------------------------------------

reg [INPUTS_COUNT-1:0]      request_holder;

//------------------------------------------------------------------------------

wire [INPUTS_COUNT-1:0]     request_repeat = requests_i | (request_holder & ~request_checker);
wire                        request_accept = |(request_holder & request_checker);

//------------------------------------------------------------------------------

dp_ram_1_block
#(
    .DATA_WIDTH_A(VALUE_WIDTH),
    .DATA_WIDTH_B(VALUE_WIDTH)
) mem (
    .a_clk(clk_i),
    .a_adr_i({{(9-ADDR_COUNTER_LEN){1'b0}}, addr_counter}),
    .a_dat_i(value_i),
    .a_dat_o(/*open*/),
    .a_we_i(request_accept),

    .b_clk(access_clk),
    .b_adr_i(access_adr_i),
    .b_dat_i({VALUE_WIDTH{1'b0}}),
    .b_dat_o(access_dat_o),
    .b_we_i(1'b0)
);

always @(posedge clk_i) begin
    if (rst_i) begin
        addr_counter <= 0;
        request_holder <= 0;
        request_checker <= {{INPUTS_COUNT{1'b0}}, 1'b1};
    end else begin
        if (addr_counter == INPUTS_COUNT - 1) begin
            addr_counter <= 0;
            request_checker <= {{INPUTS_COUNT{1'b0}}, 1'b1};
        end else begin
            addr_counter <= addr_counter + 1;
            request_checker <= {request_checker[INPUTS_COUNT-1:0], 1'b0};
        end

        request_holder <= request_repeat;
    end
end

endmodule
