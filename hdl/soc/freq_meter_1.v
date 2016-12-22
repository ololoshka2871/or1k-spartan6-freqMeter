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
    // запрос на сохранение метки конца
    output  wire                                    write_stop_req,

    input   wire                                    Fin_unsync,

    output  wire                                    ready_o,

    input   wire                                    signal_detect_reset, // сброс детектора входного сигнала
    output  reg                                     signal_present // 1, если на входе есть фронты
);

//------------------------------------------------------------------------------

reg [INPUT_FREQ_COUNTER_LEN-1:0]    input_counter;

reg input_enable;

reg r_write_start_req;
reg r_write_stop_req;

//------------------------------------------------------------------------------

wire Fin;
wire pFin;

wire write_stop_val_req;

wire _zero_detector = input_counter[INPUT_FREQ_COUNTER_LEN-1:1]
    == {(INPUT_FREQ_COUNTER_LEN-1){1'b0}};
wire zero_detector = _zero_detector & ~input_counter[0];
wire one_detector = _zero_detector & input_counter[0];

wire w_await_start;

wire w_input_front_detector = ~Fin & pFin;

//------------------------------------------------------------------------------

assign write_start_req  = r_write_start_req;
assign write_stop_req   = r_write_stop_req;

assign ready_o = zero_detector;

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
    .q1(/*open*/),
    .r(w_input_front_detector | rst_i),
    .s(restart),
    .clk(clk_i)
);

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

always @(posedge clk_i) begin
    if (rst_i) begin
        input_enable <= 1'b0;
        r_write_start_req <= 1'b0;
        r_write_stop_req <= 1'b0;
        input_counter <= 0;
        signal_present <= 1'b0;
    end else begin
        if (restart) begin
            input_counter <= reload_val;
            signal_present <= 1'b0;
        end else begin
            if (input_enable & w_input_front_detector) begin
                input_counter <= input_counter -
                    {{(INPUT_FREQ_COUNTER_LEN-1){1'b0}}, 1'b1};
                signal_present <= ~signal_detect_reset;
            end
        end

        input_enable <= input_enable ?
            ~zero_detector :
            w_await_start & w_input_front_detector;

        r_write_start_req <= w_await_start & w_input_front_detector;
        r_write_stop_req  <= input_enable & w_input_front_detector & one_detector;
    end
end

endmodule
