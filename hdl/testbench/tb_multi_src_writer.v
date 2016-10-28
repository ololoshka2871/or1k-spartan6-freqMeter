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

module tb_multi_src_writer;

reg t_clk;
initial t_clk = 1'b0;
always #23 t_clk = ~t_clk;

/* 100 MHz master clock */
reg master_clk;
initial master_clk = 1'b0;
always #10 master_clk = ~master_clk;

reg w;
initial w = 1'b0;
always #117 w = ~w;

reg rst;
reg [9:0] val;
reg [9:0] requests;

multi_src_writer
#(
    .INPUTS_COUNT(10),
    .VALUE_WIDTH(10)
) tm (
    .clk_i(master_clk),
    .rst_i(rst),

    .value_i(val),
    .requests_i(requests),

    .access_clk(1'b0),
    .access_adr_i(0),
    .access_dat_o(/*open*/)
);

initial begin
    rst = 1'b1;
    val = 0;
    requests = 0;
    #100
    rst = 1'b0;

    #10000
    $finish();
end

always @(posedge w) begin
    requests <= val & (t_clk ? {5{2'b10}} : {5{2'b01}});
end

always @(posedge master_clk) begin
    requests <= 0;
    val <= val + 1;
end

endmodule
