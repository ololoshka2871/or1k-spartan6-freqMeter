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

module tb_clk_domain_cros_register;

parameter DATA_WIDTH = 2;

reg                        reset_i;
reg                        clk_sys_i;
reg                        clk_ctl_i;

reg    [DATA_WIDTH-1:0]    sys_data_i;
reg                        sys_write_act_i;
wire   [DATA_WIDTH-1:0]    sys_data_o;

reg    [DATA_WIDTH-1:0]    D_i;
wire   [DATA_WIDTH-1:0]    D_o;
reg                        we_i;

clk_domain_cros_register
#(
    .DATA_WIDTH(DATA_WIDTH)
) tb (
    .reset_i(reset_i),
    .clk_sys_i(clk_sys_i),
    .clk_ctl_i(clk_ctl_i),

    .sys_data_i(sys_data_i),
    .sys_write_act_i(sys_write_act_i),
    .sys_data_o(sys_data_o),

    .D_i(D_i),
    .D_o(D_o),
    .we_i(we_i)
);

task waitsysclock;
begin
    #1;
    @(posedge clk_sys_i);
end
endtask

task waitctlclock;
begin
    #1;
    @(posedge clk_ctl_i);
end
endtask

initial begin
    reset_i = 1'b1;
    clk_sys_i = 1'b0;
    clk_ctl_i = 1'b0;

    sys_data_i = 2'b00;
    sys_write_act_i = 1'b0;

    D_i = 1'b0;
    we_i = 1'b0;

    waitsysclock;
    reset_i = 1'b0;

    waitsysclock;
    waitsysclock;

    // test sys -> ctl
    sys_data_i = 2'b10;
    sys_write_act_i = 1'b1;
    waitsysclock;
    sys_write_act_i = 1'b0;
    waitsysclock;

    // -- это не сработает, слишком быстро
    sys_data_i = 2'b01;
    sys_write_act_i = 1'b1;
    waitsysclock;
    // -- а вот это запишется
    sys_data_i = 2'b11;
    waitsysclock;
    sys_write_act_i = 1'b0;
    waitsysclock;

    // test ctl -> sys
    waitctlclock;
    D_i = 2'b00;
    we_i = 1'b1;
    waitctlclock;
    we_i = 1'b0;

    #100;
    $finish;
end

always #20 clk_sys_i = ~clk_sys_i;
always #16 clk_ctl_i = ~clk_ctl_i;

endmodule
