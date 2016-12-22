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

module clk_domain_cros_register
#(
    parameter DATA_WIDTH = 8,
    parameter INITIAL_VALUE = 0
) (
    input   wire                        reset_i,
    input   wire                        clk_sys_i,
    input   wire                        clk_ctl_i,

    // system bus side
    input   wire    [DATA_WIDTH-1:0]    sys_data_i,
    input   wire                        sys_write_act_i,
    output  reg     [DATA_WIDTH-1:0]    sys_data_o,

    // controlable side
    input   wire    [DATA_WIDTH-1:0]    D_i,
    output  wire    [DATA_WIDTH-1:0]    D_o,
    input   wire                        we_i
);

reg [DATA_WIDTH-1:0] sys_data_r = {DATA_WIDTH{1'b0}};

// read loop (control -> system)
always @(posedge clk_sys_i) begin
    sys_data_r <= D_o;
    sys_data_o <= sys_data_r;
end

// write
wire accept_write_act;
wire write_done;

srff res_sync_ff
(
    .clk(clk_sys_i),
    .s(sys_write_act_i),
    .r(write_done),
    .q(accept_write_act)
);

data_synchronizier
#(
    .DATA_WIDTH(DATA_WIDTH),
    .INITIAL_VALUE(INITIAL_VALUE)
) W_sync (
    .clk_i(clk_ctl_i),
    .rst_i(reset_i),

    .Q(D_o),

    .D_hip_i(D_i),
    .D_lop_i(sys_data_i),
    .WR_hip_i(we_i),
    .WR_lop_i(accept_write_act),

    .data_changed_o(write_done),
    .change_accepted_i(~accept_write_act)
);

endmodule
