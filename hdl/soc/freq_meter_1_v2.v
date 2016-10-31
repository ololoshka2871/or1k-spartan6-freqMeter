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

module freq_meter_1_v2
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

    output  reg                                     ready_o
);

//------------------------------------------------------------------------------

localparam STATE_READY          = 3'b000;
localparam STATE_AWAIT_START    = 3'b001;
localparam STATE_START          = 3'b010;
localparam STATE_COUNTING       = 3'b011;
localparam STATE_STOP           = 3'b100;

//------------------------------------------------------------------------------

reg [2:0] state;

reg [INPUT_FREQ_COUNTER_LEN-1:0]    input_counter;

reg r_write_start_req;
reg r_write_stop_req;

//------------------------------------------------------------------------------

wire Fin;
wire pFin;

wire front_detector = ~Fin & pFin;

//------------------------------------------------------------------------------

assign write_start_req  = r_write_start_req;
assign write_stop_req   = r_write_stop_req;

//------------------------------------------------------------------------------

input_synchronizer sync(
    .clk(clk_i),
    .reset(rst_i),
    .din(Fin_unsync),
    .dout(Fin),
    .pdout(pFin)
);

//------------------------------------------------------------------------------

always @(posedge clk_i) begin
    if (rst_i) begin
        state <= STATE_READY;
    end else begin
        case (state)
        STATE_READY: begin
            ready_o <= 1'b1;
            r_write_start_req <= 1'b0;
            r_write_stop_req <= 1'b0;
            if (restart) begin
                input_counter <= reload_val;
                state <= STATE_AWAIT_START;
            end
        end
        STATE_AWAIT_START: begin
            ready_o <= 1'b0;
            r_write_stop_req <= 1'b0;
            if (front_detector) begin
                state <= STATE_START;
                r_write_start_req <= 1'b1;
            end
        end
        STATE_START: begin
            ready_o <= 1'b0;
            r_write_start_req <= 1'b0;
            r_write_stop_req <= 1'b0;
            state <= STATE_COUNTING;
            input_counter <= input_counter - 1;
        end
        STATE_COUNTING: begin
            ready_o <= 1'b0;
            r_write_start_req <= 1'b0;
            r_write_stop_req <= 1'b0;
            if (front_detector) begin
                if (input_counter == 0)
                    state <= STATE_STOP;
                else
                    input_counter <= input_counter - 1;
            end
        end
        STATE_STOP: begin
            ready_o <= 1'b0;
            r_write_start_req <= 1'b0;
            r_write_stop_req <= 1'b1;
            state <= STATE_READY;
        end
        default:
            state <= STATE_READY;
        endcase
    end
end

endmodule
