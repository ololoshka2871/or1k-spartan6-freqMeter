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

reg input_enable;
reg p_input_enable;

reg r_write_start_req;
reg r_write_stop_req;

//------------------------------------------------------------------------------

wire Fin;
wire pFin;

wire write_stop_val_req;

wire overflow_detector = input_counter == {INPUT_FREQ_COUNTER_LEN{1'b0}};

wire w_await_start;

wire w_input_front_detector = ~Fin & pFin;

wire input_enable_rst_detector = (~input_enable & p_input_enable);

//------------------------------------------------------------------------------

assign write_start_req  = r_write_start_req & write_start_enable_i;
assign write_stop_req   = r_write_stop_req  & write_stop_enable_i;

//------------------------------------------------------------------------------

input_synchronizer sync(
    .clk(clk_i),
    .reset(rst_i),
    .din(Fin_unsync),
    .dout(Fin),
    .pdout(pFin)
);

srff r_await_start(
    .q(w_await_start),
    .r(w_input_front_detector | rst_i),
    .s(restart),
    .clk(clk_i)
);

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        input_enable <= 1'b0;
        p_input_enable <= 1'b0;
        r_write_start_req <= 1'b0;
        r_write_stop_req <= 1'b0;
    end else begin
        p_input_enable <= input_enable;

        if (restart) begin
            input_counter <= reload_val;
        end else begin
            input_counter <= input_counter - (input_enable & w_input_front_detector);
        end

        input_enable <= ~input_enable ?
            w_await_start & w_input_front_detector :
            ~overflow_detector;

        r_write_start_req <= r_write_start_req ?
            ~write_start_req:
            w_await_start & w_input_front_detector;
        r_write_stop_req  <= r_write_stop_req ?
            ~write_stop_req:
            input_enable & w_input_front_detector;
    end
end

endmodule
