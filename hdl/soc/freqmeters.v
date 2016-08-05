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

`define WB_DATA_WIDTH 32 // только для 32 битной шмны WishBone
`define MAX_FREQMETERS 32 // Максимальный размер модуля

module freqmeters
#(
    parameter   INPUTS_COUNT            = 24, // количество входов (1 - 32)
    parameter   MASER_FREQ_COUNTER_LEN  = 30, // длина регистра, считающего опорную частоту
    parameter   INPUT_FREQ_COUNTER_LEN  = 24, // длина региста, считающего входную частоту
    parameter   TEST_INPUT_AT_2_POW_CLK = 20  // сгенерировать сигнал проверки вхоов каждые 2^TEST_INPUT_AT_2_POW_CLK тактов опорной частоты
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

    input  wire                         F_master,      // частота образцовая
    input  wire [INPUTS_COUNT - 1:0]    F_in,          // входы для частоты

    output wire [MASER_FREQ_COUNTER_LEN-1:0] devided_clocks // clk_i деленная на 2, 4, ... 2^MASER_FREQ_COUNTER_LEN
);

// Счетчик опорной частоты
// FM_MASTER_CNT (RO)
reg  [MASER_FREQ_COUNTER_LEN-1:0] mester_freq_counter;

//--------------------------------registers-------------------------------------

// FM_IE (RW)
reg  [INPUTS_COUNT-1:0]           irq_enable; // разрешить прерывания на этих каналах (RW)

//--------------------------------memory----------------------------------------

// PORTA - freqmeters
// PORTB - wishbone

// 0x0 - 0x3F 
// [0x00 - 0x1f] - START_vals
// (* RAM_STYLE="BLOCK" *)
reg  [MASER_FREQ_COUNTER_LEN-1:0] START_vals [INPUTS_COUNT - 1:0];

// [0x20 - 0x3f] - STOP_vals
// (* RAM_STYLE="BLOCK" *)
reg  [MASER_FREQ_COUNTER_LEN-1:0] STOP_vals  [INPUTS_COUNT - 1:0];

wire memory_addr     =  adr_i[4:0];
wire START_selector  = ~adr_i[5];
wire memory_selector = ~adr_i[6];

//------------------------------------------------------------------------------

wire test_no_input_signal_i = mester_freq_counter[TEST_INPUT_AT_2_POW_CLK];

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

// endian control

reg [`WB_DATA_WIDTH - 1:0] _dat_o;

assign dat_o = _dat_o;

wire [`WB_DATA_WIDTH - 1:0] _dat_i;

assign _dat_i = dat_i;

//------------------------------------------------------------------------------
// interrupt
//------------------------------------------------------------------------------

integer i;

initial begin
    for(i = 0; i < INPUTS_COUNT; i = i + 1) begin
        START_vals[i] = 32'h00000000;
        STOP_vals[i]  = 32'h00000000;
    end
end

//------------------------------------------------------------------------------

always @(posedge clk_i) begin
    ack_o <= 1'b0;
    if (cyc_i & stb_i & ~ack_o) begin
        if (memory_selector) begin
            _dat_o <= START_selector ? START_vals[memory_addr] : STOP_vals[memory_addr];
        end else begin
            if (we_i) begin
            case (memory_addr)
                6'b111111:
                    irq_enable <= _dat_i[INPUTS_COUNT-1:0];
                default:
                    ;
                endcase
            end
            case (memory_addr)
                6'b111111:
                    _dat_o <= {{(`WB_DATA_WIDTH - INPUTS_COUNT){1'b0}}, irq_enable};
                default:
                    ;
            endcase
        end
        ack_o <= 1'b1;
    end
end

always @(posedge F_master or posedge rst_i) begin
    if (rst_i)
        mester_freq_counter <= {MASER_FREQ_COUNTER_LEN{1'b0}};
    else
        mester_freq_counter <= mester_freq_counter + 1;
end

//------------------------------------------------------------------------------

endmodule