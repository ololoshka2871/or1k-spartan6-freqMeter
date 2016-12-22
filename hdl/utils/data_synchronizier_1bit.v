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

module data_synchronizier_1bit
#(
    parameter INITIAL_VALUE = 0
) (
    input wire          clk_i,
    input wire          rst_i,

    // data output
    output reg          Q,

    // data inputs
    input wire          D_hip_i,
    input wire          D_lop_i,
    input wire          WR_hip_i,
    input wire          WR_lop_i,

    // transaction control
    output reg          data_changed_o,
    input wire          change_accepted_i
);

reg Qs;

always @(posedge clk_i) begin
    if (rst_i) begin
        Qs <= INITIAL_VALUE;
        Q <= INITIAL_VALUE;
        data_changed_o <= 1'b0;
    end else begin
        Q <= Qs;
        if (WR_hip_i) begin
            Qs <= D_hip_i;
            data_changed_o <= 1'b1;
        end else if (WR_lop_i) begin
                Qs <= D_lop_i;
                data_changed_o <= 1'b1;
            end
        if (change_accepted_i) begin
            data_changed_o <= 1'b0;
        end
    end
end

endmodule
