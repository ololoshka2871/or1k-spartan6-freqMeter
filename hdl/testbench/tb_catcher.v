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


module tb_catcher;

reg clk = 1'b0;
always #5 clk <= ~clk;

reg clk_sys = 1'b0;
always #2 clk_sys <= ~clk_sys;


reg [9:0] v;
reg rst;

reg [9:0] requests;

catcher
#(
    .INPUT_COUNT(10),
    .VALUE_WIDTH(10)
) catcher_inst (
    .clk_i(clk),
    .rst_i(rst),

    .value2catch_i(v),
    .catch_requests_i(requests),

    .read_clk_i(clk_sys),
    .read_addr_i(0),
    .read_do_o()
);


initial begin
    rst = 1'b1;
    v = 0;
    requests = 0;
    #20;
    rst = 1'b0;

    #30;
    requests = 1;
    #20;

    requests = 3;
    #10;
    requests = 10'b1000000100;
    #60;

    $finish();
end

always @(posedge clk) begin
    v <= v + 1;
    requests <= 0;
end

endmodule
