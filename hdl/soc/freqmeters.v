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

`define WB_DATA_WIDTH 32 // только для 32 битной шмны WishBone

module freqmeters
#(
    parameter   INPUTS_COUNT            = 24, // количество входов (1 - 32)
    parameter   MASER_FREQ_COUNTER_LEN  = 30, // длина регистра, считающего опорную частоту
    parameter   INPUT_FREQ_COUNTER_LEN  = 24, // длина региста, считающего входную частоту
    parameter   TEST_INPUT_AT_2_POW_CLK = 20
) (
    // WISHBONE bus slave interface
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)
    input  wire				cyc_i,         // cycle
    input  wire				stb_i,         // strobe
    input  wire [10:0]                  adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [`WB_DATA_WIDTH - 1:0]	dat_i,         // data input
    output wire [`WB_DATA_WIDTH - 1:0]	dat_o,         // data output
    output reg 				ack_o,         // normal bus termination
    output wire				inta_o,        // interrupt output

    input  wire [INPUTS_COUNT - 1:0]    F_in,          // входы для частоты

    output wire                         irq,           // выход прерывания

    output wire [MASER_FREQ_COUNTER_LEN-1:0] devided_clocks // clk_i деленная на 2, 4, ... 2^MASER_FREQ_COUNTER_LEN
);

// Счетчик опорной частоты
// FM_MASTER_CNT (RO)
reg [MASER_FREQ_COUNTER_LEN-1:0] mester_freq_counter;

//-----------------------------32-bit regs--------------------------------------

// FM_FM_CNT_x (RO)
wire [MASER_FREQ_COUNTER_LEN-1:0] master_counter_results [INPUTS_COUNT-1:0]; // результаты

// FM_RDY_TS_x (RO)
wire [MASER_FREQ_COUNTER_LEN-1:0] timestamps_results [INPUTS_COUNT-1:0]; // таймштампы готовности

// FM_FI_CNT_x (RO)
wire [INPUT_FREQ_COUNTER_LEN-1:0] input_vals_results [INPUTS_COUNT-1:0]; // количества посчитаных импульсов

// FM_RLD_x (RW)
reg  [INPUT_FREQ_COUNTER_LEN-1:0] reload_vals        [INPUTS_COUNT-1:0]; // значения для перезагрузки счетчиков

// FM_RDY (RW)
wire [INPUTS_COUNT-1:0]           readys; // флаги готовности               (R)
reg  [INPUTS_COUNT-1:0]           accepts; // сбрасывалка флагов готовности (W)

// FM_IE (RW)
reg  [INPUTS_COUNT-1:0]           irq_enable; // разрешить прерывания на этих каналах (RW)

// FM_SC (RW)
wire [INPUTS_COUNT-1:0]           no_signals; // нет сигнала (R)
reg  [INPUTS_COUNT-1:0]           manual_restart_cycle; // принудительный перезапуск (W)

//------------------------------------------------------------------------------

wire test_no_input_signal_i = mester_freq_counter[TEST_INPUT_AT_2_POW_CLK];

//------------------------------------------------------------------------------

wire [8:0] reg_addr_valid = adr_i[10:2];

// номер регистра
// Для каждого частотомера
// 000 - FM_FM_CNT_x
// 001 - FM_RDY_TS_x
// 010 - FM_FI_CNT_x
// 011 - FM_RLD_x
// Для сегмента общих регистров
// 000 - FM_RDY
// 001 - FM_IE
// 010 - FM_SC
// 111 - FM_MASTER_CNT
wire [2:0] reg_number = reg_addr_valid[2:0];

// Номер частотомера
wire [5:0] freqmeter_number = reg_addr_valid[7:3];

// Выбран сегмент общих регистров
wire       comon_reg_selector = reg_addr_valid[8];

//------------------------------------------------------------------------------

// endian control

reg [`WB_DATA_WIDTH - 1:0] _dat_o;

assign dat_o = _dat_o;

wire [`WB_DATA_WIDTH - 1:0] _dat_i;

assign _dat_i = dat_i;

//------------------------------------------------------------------------------

function [`WB_DATA_WIDTH-1:0] read_reg_master;
input [MASER_FREQ_COUNTER_LEN-1:0] v;
read_reg_master = {{(`WB_DATA_WIDTH-MASER_FREQ_COUNTER_LEN){1'b0}}, v};
endfunction

function [`WB_DATA_WIDTH-1:0] read_reg_work;
input [INPUT_FREQ_COUNTER_LEN-1:0] v;
read_reg_work = {{(`WB_DATA_WIDTH-INPUT_FREQ_COUNTER_LEN){1'b0}}, v};
endfunction

function [`WB_DATA_WIDTH-1:0] read_reg_comon;
input [INPUTS_COUNT-1:0] v;
read_reg_comon = {{(`WB_DATA_WIDTH-INPUTS_COUNT){1'b0}}, v};
endfunction

//------------------------------------------------------------------------------

genvar i;
integer j;

generate
    for (i = 0; i < MASER_FREQ_COUNTER_LEN; i = i + 1) begin
        assign devided_clocks[i] = mester_freq_counter[i];
    end
endgenerate

generate
    for (i = 0; i < INPUTS_COUNT; i = i + 1) begin
        freq_meter
        #(
            .MASER_FREQ_COUNTER_LEN(MASER_FREQ_COUNTER_LEN),
            .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
        ) freq_meter_inst (
            .master_counter_i(mester_freq_counter),
            .result_master_val_o(master_counter_results[i]),
            .result_input_val_o(input_vals_results[i]),
            .result_timestamp_o(timestamps_results[i]),
            .input_counter_reload_i(reload_vals[i]),

            .F_in(F_in[i]),
            .clk_i(clk_i),
            .rst_i(rst_i),
            .restart_cycle_i(manual_restart_cycle[i]),

            .ready_o(readys[i]),
            .ready_accept_i(accepts[i]),

            .no_input_signal_o(no_signals[i]),
            .test_no_input_signal_i(test_no_input_signal_i)
        );
    end
endgenerate

// reset
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        mester_freq_counter <= {MASER_FREQ_COUNTER_LEN{1'b0}};
        accepts <= {INPUTS_COUNT{1'b0}};

        for (j = 0; j < INPUTS_COUNT; j = j + 1) begin
            reload_vals[j] <= {{(INPUT_FREQ_COUNTER_LEN-1){1'b0}}, 1'b1};
        end
    end
end

always @(posedge clk_i) begin
    ack_o <= 1'b0;
    if (cyc_i & stb_i & ~ack_o) begin
        if (we_i) begin
            // write
            if (comon_reg_selector)
                case (reg_number)
                    3'b000:
                        accepts <= _dat_i[INPUTS_COUNT-1:0];
                    3'b001:
                        irq_enable <= _dat_i[INPUTS_COUNT-1:0];
                    3'b010:
                        manual_restart_cycle <= _dat_i[INPUTS_COUNT-1:0];
                endcase
            else
                case (reg_number)
                    3'b011:
                        reload_vals[freqmeter_number] <=
                            _dat_i[INPUT_FREQ_COUNTER_LEN-1:0];
                endcase
        end
        else
        begin
            // reset clear flags registers
            accepts <= {INPUTS_COUNT{1'b0}};
            manual_restart_cycle <= {INPUTS_COUNT{1'b0}};
        end

        // read
        if (comon_reg_selector)
            case (reg_number)
                3'b000:
                    _dat_o <= read_reg_comon(readys);
                3'b001:
                    _dat_o <= read_reg_comon(irq_enable);
                3'b010:
                    _dat_o <= read_reg_comon(no_signals);
                3'b111:
                    _dat_o <= read_reg_master(mester_freq_counter);
                default:
                    _dat_o <= 4'h00000000;
            endcase
        else
            case (reg_number)
                3'b000:
                    _dat_o <= read_reg_master(master_counter_results[freqmeter_number]);
                3'b001:
                    _dat_o <= read_reg_master(timestamps_results[freqmeter_number]);
                3'b010:
                    _dat_o <= read_reg_work(input_vals_results[freqmeter_number]);
                3'b011:
                    _dat_o <= read_reg_work(reload_vals[freqmeter_number]);
                default:
                    _dat_o <= 4'h00000000;
            endcase
        ack_o <= 1'b1;
    end
end

endmodule
