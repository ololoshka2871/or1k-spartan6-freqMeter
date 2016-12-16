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


module tb_TaskAck_CrossDomain;

/* system clock */
reg sys_clk;
initial sys_clk = 1'b0;
always #7 sys_clk = ~sys_clk;

/* 50 MHz rmii clock */
reg rmii_clk;
initial rmii_clk = 1'b0;
always #10 rmii_clk = ~rmii_clk;

reg  wr;

wire wr_sys_busy;
wire wr_sys_done;

wire wr_ctl;
wire wr_ctl_busy;
reg  wr_ctl_done;

TaskAck_CrossDomain ti (
    .clkA(sys_clk),
    .TaskStart_clkA(wr),
    .TaskBusy_clkA(wr_sys_busy),
    .TaskDone_clkA(wr_sys_done),

    .clkB(rmii_clk),
    .TaskStart_clkB(wr_ctl),
    .TaskBusy_clkB(wr_ctl_busy),
    .TaskDone_clkB(wr_ctl_done)
);

initial begin
    wr = 0;
    wr_ctl_done = 0;

    #58;

    wr = 1;
end

always @(negedge sys_clk) begin
    if (wr)
        wr <= 1'b0;
end

always @(posedge rmii_clk) begin
    wr_ctl_done <= (wr_ctl & ~wr_ctl_done);
end

endmodule
