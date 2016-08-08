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
//* 3. Neither the name NuttX nor the names of its contributors may be
//*    used to endorse or promote products derived from this software
//*    without specific prior written permission.
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

module tb_freq_meter_1
#(
    parameter MASER_FREQ_COUNTER_LEN = 30,
    parameter INPUT_FREQ_COUNTER_LEN = 24
) (
    output reg F_in,
    output reg clk_i,

    output wire write_start_req,
    output wire write_stop_req
);

reg [MASER_FREQ_COUNTER_LEN - 1:0]      master_counter;
reg restart_req;
reg [INPUT_FREQ_COUNTER_LEN - 1:0]      reload_val;

reg enable_start;
reg enable_stop;

reg rst_i;

freq_meter_1
#(
    .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
) fm (
    .rst_i(rst_i),
    .clk_i(clk_i),

    .reload_val(reload_val),

    .restart(restart_req),

    .write_start_req(write_start_req),
    .write_start_enable_i(enable_start),


    .write_stop_req(write_stop_req),
    .write_stop_enable_i(enable_stop),

    .Fin_unsync(F_in)
);


initial begin
        // Initialize Inputs
        clk_i = 0;

        rst_i = 1;

        reload_val = 1;
        F_in = 0;
        clk_i = 0;
        restart_req = 0;
        master_counter = 0;

        enable_start = 0;
        enable_stop = 0;

        #10;
        rst_i = 0;

        // Wait 101 ns for global reset to finish
        #101;

        restart_req = 1;

        #583
        enable_start = 1;

        #585
        enable_stop = 1;

end

    always #5 begin
        clk_i <= !clk_i;
        if (clk_i) begin
            master_counter <= master_counter + 1;
            if (restart_req)
                restart_req <= 0;
        end
    end

    always #253 begin
        F_in <= !F_in;
    end

endmodule
