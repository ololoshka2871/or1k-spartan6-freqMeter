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

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "config.v"

module freq_meter
#(
    parameter MASER_FREQ_COUNTER_LEN = 30,
    parameter INPUT_FREQ_COUNTER_LEN = 24
) (
    // сюда подаётся значение счетчика образцовой частоты
    input   wire [MASER_FREQ_COUNTER_LEN - 1:0]       master_counter_i,

    // выход модуля - количество импульсов образцовой частоты пришедшее за
    // период пока было отсчитано input_counter_reload_i импульсов входной частоты
    output  wire [MASER_FREQ_COUNTER_LEN - 1:0]       result_master_val_o,

    // количество импульсов входной частоты для за время которых было насчитано
    // result_master_val_o импульсов образцовой частоты
    output  reg  [INPUT_FREQ_COUNTER_LEN - 1:0]       result_input_val_o,

    // значение счетчика образцовой частоты когда был получен результат
    output  wire [MASER_FREQ_COUNTER_LEN - 1:0]       result_timestamp_o,

    // количество импульсов входной частоты которое будет отсчитано на следующей
    // итерации
    input   wire [INPUT_FREQ_COUNTER_LEN - 1:0]       input_counter_reload_i,


    input   wire                                      F_in, // измеряемая частота
    input   wire                                      clk_i,
    input   wire                                      rst_i, // global reset
    input   wire                                      restart_cycle_i, // reset cycle manualy

    // events
    output  wire                                      ready_o, // сигнал того, что цыкл завершился и данные готовы
    input   wire                                      ready_accept_i, // сброс флага готовности

    output  reg                                       no_input_signal_o, // флаг отсутсвия входной частоты
    input   wire                                      test_no_input_signal_i // проверить входной синал
);

// временное хранилище для
// input_counter_reload_i -> result_input_val_tmp -> result_input_val_o
reg [INPUT_FREQ_COUNTER_LEN - 1:0] result_input_val_tmp;
// предидущее значение счетчика образцовой частоты
reg [MASER_FREQ_COUNTER_LEN:0] prev_master_freq_val;
// Счетчик измеряемой частоты
reg [INPUT_FREQ_COUNTER_LEN - 1:0] Fin_counter;
// output value
reg [MASER_FREQ_COUNTER_LEN:0] result_master_val;

reg input_detector;
reg [1:0] ready_protector;
reg ready;

// триггер конца цыкла
wire cycle_end = restart_cycle_i |
    (Fin_counter == {INPUT_FREQ_COUNTER_LEN{1'b0}});

// входная частота синхронизированная с образцовой частотой
wire Fin_sync;

assign result_master_val_o = result_master_val[MASER_FREQ_COUNTER_LEN] ?
    (~(result_master_val[MASER_FREQ_COUNTER_LEN - 1:0]) + 1) :
    result_master_val[MASER_FREQ_COUNTER_LEN - 1:0];

assign result_timestamp_o = prev_master_freq_val;

assign ready_o = ready & ready_protector[1];

// синхронизатор по входу
synchronizer input_synchronizer (
    .clk(clk_i),
    .reset(rst_i),
    .din(F_in),
    .dout(Fin_sync)
);

always @(posedge Fin_sync or posedge rst_i) begin
    if (rst_i == 1'b1)
    begin
        Fin_counter <= {{(INPUT_FREQ_COUNTER_LEN-1){1'b0}}, 1'b1};
    end
    else
    begin
        Fin_counter <= Fin_counter - 1; // считаем импульс если не конец цыкла
        input_detector <= 1'b1;
    end
end

always @(posedge restart_cycle_i) begin
    ready_protector <= 2'b00;
end

always @(posedge cycle_end) begin
    if (ready_protector[1] == 1'b0) begin
        ready_protector = ready_protector + 1;
    end
end

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i == 1'b1)
    begin
        result_master_val <= {MASER_FREQ_COUNTER_LEN{1'b0}};
        result_input_val_o <= {INPUT_FREQ_COUNTER_LEN{1'b0}};
        result_input_val_tmp <= {INPUT_FREQ_COUNTER_LEN{1'b0}};
        prev_master_freq_val <= {1'b0, master_counter_i};
        ready <= 1'b0;
        ready_protector <= 2'b00;
    end
    else
    begin
        if (ready_accept_i == 1'b1) begin
            ready <= 1'b0;
        end

        if (cycle_end == 1'b1 && input_detector == 1'b1) begin // reload
            result_input_val_o <= result_input_val_tmp; // result_input
            result_input_val_tmp <= input_counter_reload_i;

            result_master_val <= {1'b0, master_counter_i} -
                prev_master_freq_val; // result

            prev_master_freq_val <= {1'b0, master_counter_i}; // prev

            Fin_counter <= input_counter_reload_i; // reload working counter

            if (ready_accept_i == 1'b0)
                ready <= 1'b1;
        end
    end
end

always @(posedge test_no_input_signal_i or posedge rst_i) begin
    if (rst_i == 1'b1)
    begin
        no_input_signal_o <= 1'b0;
        input_detector <= 1'b0;
    end
    else
    begin
        if (input_detector == 1'b0)
        begin
            result_input_val_tmp <= input_counter_reload_i;
            prev_master_freq_val <= {1'b0, master_counter_i}; // prev
            Fin_counter <= input_counter_reload_i; // reload working counter
        end
        no_input_signal_o <= input_detector;
        input_detector <= 1'b0;
    end
end

endmodule
