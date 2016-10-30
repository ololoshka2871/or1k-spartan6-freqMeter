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


module catcher
#(
    parameter INPUT_COUNT                           = 24,
    parameter VALUE_WIDTH                           = 32,
    parameter INPUT_NUMBER_WIDTH                    = $clog2(INPUT_COUNT)
) (
    input  wire                                     clk_i,
    input  wire                                     rst_i,

    input  wire [VALUE_WIDTH-1:0]                   value2catch_i,
    input  wire [INPUT_COUNT-1:0]                   catch_requests_i,

    input  wire                                     read_clk_i,
    input  wire [INPUT_NUMBER_WIDTH-1:0]            read_addr_i,
    output reg  [VALUE_WIDTH-1:0]                   read_do_o,
    input  wire                                     cyc_i,
    output reg                                      ack_o
);

//------------------------------------------------------------------------------

reg [INPUT_COUNT-1:0] request_holder;

reg [INPUT_NUMBER_WIDTH-1:0] input_number;
reg code_ok;

//------------------------------------------------------------------------------

// (* RAM_STYLE="BLOCK" *)
reg [VALUE_WIDTH-1:0]   memory  [INPUT_COUNT-1:0];

//------------------------------------------------------------------------------

integer i;

always @(posedge clk_i) begin
    if (rst_i) begin
        request_holder <= 0;
        input_number <= 0;
        code_ok <= 1'b0;
    end else begin
        code_ok <= 1'b0;
        request_holder <= request_holder | catch_requests_i;
        for (i = 0; i < INPUT_COUNT; i = i + 1) begin : coder_loop
            if (request_holder[i]) begin
                input_number <= i[INPUT_NUMBER_WIDTH-1:0];
                request_holder[i] <= catch_requests_i[i];
                code_ok <= 1'b1;
                disable coder_loop;
            end
        end
    end
end

always @(posedge clk_i) begin
    if (code_ok) begin
        memory[input_number] <= value2catch_i;
    end
end

always @(posedge read_clk_i) begin
    if (cyc_i) begin
        if (read_addr_i < INPUT_COUNT)
            read_do_o <= memory[read_addr_i];
        else
            read_do_o <= 32'hDEADBEAF;
        ack_o <= 1'b1;
    end else begin
        ack_o <= 1'b0;
    end
end

endmodule
