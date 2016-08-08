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

module freq_meter_1
#(
    parameter   INPUT_FREQ_COUNTER_LEN  = 24        // длина региста, считающего входную частоту
) (
    input   wire                                    rst_i,      // reset
    input   wire                                    clk_i,

    input   wire    [INPUT_FREQ_COUNTER_LEN-1:0]    reload_val, // Значение для перезапуска


    input   wire                                    restart,    // запустить измерения

    // запрос на сохранение метки начала
    output  wire                                    write_start_req,
    input   wire                                    write_start_enable_i,

    // запрос на сохранение метки конца
    output  wire                                    write_stop_req,
    input   wire                                    write_stop_enable_i,

    input   wire                                    Fin_unsync
);

//------------------------------------------------------------------------------

reg [INPUT_FREQ_COUNTER_LEN-1:0]    input_counter;

//------------------------------------------------------------------------------

wire Fin;
wire input_enable;
wire Fin_count = Fin & input_enable;

wire overflow_detector = input_counter == {INPUT_FREQ_COUNTER_LEN{1'b1}};

wire w_await_start;

assign write_stop_req = write_stop_enable_i & write_stop_val_req;

//------------------------------------------------------------------------------

synchronizer sync(
    .clk(clk_i),
    .reset(rst_i),
    .din(Fin_unsync),
    .dout(Fin)
);

dff_async_rst input_enable_r(
    .data(1'b1),
    .clk(write_start_req),
    .reset( restart | overflow_detector | rst_i),
    .q(input_enable)
);

srff r_await_start(
    .q(w_await_start),
    .r(input_enable | rst_i),
    .s(restart),
    .clk(clk_i)
);

dff_async_rst write_start_detector(
    .data(w_await_start & write_start_enable_i),
    .clk(Fin),
    .reset((~clk_i & write_start_req) | rst_i),
    .q(write_start_req)
);

dff_async_rst write_stop_detector(
    .data(1'b1),
    .clk(~input_enable),
    .reset((~clk_i & write_stop_req) | rst_i),
    .q(write_stop_val_req)
);


//------------------------------------------------------------------------------

always @(posedge Fin_count or posedge restart) begin
    if (restart) begin
        input_counter <= reload_val;
    end else
        if (~overflow_detector)
            input_counter <= input_counter - 1;
end

endmodule
