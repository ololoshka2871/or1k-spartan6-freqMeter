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

module tb_data_synchronizier_1bit
(
    // data output
    output wire          Q,

    // transaction control
    output wire         data_changed_o
);

reg          clk_i;
reg          rst_i;

reg          D_hip_i;
reg          D_lop_i;
reg          WR_hip_i;
reg          WR_lop_i;
reg          change_accepted_i;

data_synchronizier_1bit
#(
    .INITIAL_VALUE(1'b0)
) ds (
    .clk_i(clk_i),
    .rst_i(rst_i),
    .Q(Q),
    .D_hip_i(D_hip_i),
    .D_lop_i(D_lop_i),
    .WR_hip_i(WR_hip_i),
    .WR_lop_i(WR_lop_i),
    .data_changed_o(data_changed_o),
    .change_accepted_i(change_accepted_i)
);

task waitclock;
begin
    @(posedge clk_i);
    #1;
end
endtask

task accept_transaction;
begin
    waitclock;
    change_accepted_i <= 1'b1;
    waitclock;
    change_accepted_i <= 1'b0;
    waitclock;
end
endtask

initial begin
    change_accepted_i = 0;
    clk_i = 0;
    rst_i = 1;

    D_hip_i = 0;
    D_lop_i = 0;
    WR_hip_i = 0;
    WR_lop_i = 0;

    waitclock;
    rst_i = 0;

    #20;
    D_hip_i = 1;
    D_lop_i = 0;

    #5;
    WR_hip_i = 1;
    accept_transaction;
    WR_hip_i = 0;

    #5;
    WR_lop_i = 1;
    accept_transaction;
    WR_lop_i = 0;

    #2;
    D_lop_i = 0;

    #5;
    WR_hip_i = 1;
    WR_lop_i = 0;
    accept_transaction;
    WR_hip_i = 0;
    WR_lop_i = 0;

    #20;
    $finish;
end

always #2 clk_i = ~clk_i;

endmodule
