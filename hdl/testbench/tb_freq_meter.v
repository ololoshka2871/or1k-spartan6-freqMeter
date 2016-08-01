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
//* this module provides access to FPGA boot SPI flash
//*
//****************************************************************************/

module tb_freq_meter
#(
    parameter MASER_FREQ_COUNTER_LEN = 30,
    parameter INPUT_FREQ_COUNTER_LEN = 24
) (
    output  wire  [MASER_FREQ_COUNTER_LEN - 1:0]      result_master_val_o,
    output  wire  [INPUT_FREQ_COUNTER_LEN - 1:0]      result_input_val_o,

    output  wire  [MASER_FREQ_COUNTER_LEN - 1:0]      result_timestamp_o,

    // events
    output  wire                                      ready_o, // сигнал того, что цыкл завершился и данные готовы
    output  wire                                      no_input_signal_o // флаг отсутсвия входной частоты
);

reg [MASER_FREQ_COUNTER_LEN - 1:0]      master_counter;
reg [INPUT_FREQ_COUNTER_LEN - 1:0]      input_counter_reload;

reg F_in;
reg clk_i;
reg rst_i;
reg restart_cycle;

reg ready_accept;
reg test_no_input_signal_i;

freq_meter
#(
    .MASER_FREQ_COUNTER_LEN(MASER_FREQ_COUNTER_LEN),
    .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
) fm (
    .master_counter_i(master_counter),
    .result_master_val_o(result_master_val_o),
    .result_input_val_o(result_input_val_o),
    .result_timestamp_o(result_timestamp_o),
    .input_counter_reload_i(input_counter_reload),
    .F_in(F_in),
    .clk_i(clk_i),
    .rst_i(rst_i),
    .restart_cycle_i(restart_cycle),

    .ready_o(ready_o),
    .ready_accept_i(ready_accept),

    .no_input_signal_o(no_input_signal_o),
    .test_no_input_signal_i(test_no_input_signal_i)
);


initial begin
        // Initialize Inputs
        clk_i = 0;

        rst_i = 1;

        master_counter = 0;
        input_counter_reload = 3;

        F_in = 0;
        restart_cycle = 0;

        ready_accept = 0;
        test_no_input_signal_i = 0;

        #10;
        rst_i = 0;

        // Wait 10 ns for global reset to finish
        #10;
end

    always #10 begin
        clk_i <= !clk_i;
        if (!clk_i) begin
            master_counter = master_counter + 1;
            ready_accept <= ready_o; // accept ready
            if (restart_cycle)
                restart_cycle <= 1'b0;
        end
    end

    always #253 begin
        F_in <= !F_in;
    end

    always #3000 begin
        restart_cycle <= 1'b1;
    end

endmodule
