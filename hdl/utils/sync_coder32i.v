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

module sync_coder32i (
    input wire clk_i,
    input wire [31:0]   inputs,
    output reg [4:0]    outputs,
    output reg  error
);

always @(clk_i) begin
    case(inputs)
        1 << 0: begin
            outputs <= 0;
            error <= 1'b0;
            end
        1 << 1: begin
            outputs <= 1;
            error <= 1'b0;
            end
        1 << 2: begin
            outputs <= 2;
            error <= 1'b0;
            end
        1 << 3: begin
            outputs <= 3;
            error <= 1'b0;
            end
        1 << 4: begin
            outputs <= 4;
            error <= 1'b0;
            end
        1 << 5: begin
            outputs <= 5;
            error <= 1'b0;
            end
        1 << 6: begin
            outputs <= 6;
            error <= 1'b0;
            end
        1 << 7: begin
            outputs <= 7;
            error <= 1'b0;
            end
        1 << 8: begin
            outputs <= 8;
            error <= 1'b0;
            end
        1 << 9: begin
            outputs <= 9;
            error <= 1'b0;
            end
        1 << 10: begin
            outputs <= 10;
            error <= 1'b0;
            end
        1 << 11: begin
            outputs <= 11;
            error <= 1'b0;
            end
        1 << 12: begin
            outputs <= 12;
            error <= 1'b0;
            end
        1 << 13: begin
            outputs <= 13;
            error <= 1'b0;
            end
        1 << 14: begin
            outputs <= 14;
            error <= 1'b0;
            end
        1 << 15: begin
            outputs <= 15;
            error <= 1'b0;
            end
        1 << 16: begin
            outputs <= 16;
            error <= 1'b0;
            end
        1 << 17: begin
            outputs <= 17;
            error <= 1'b0;
            end
        1 << 18: begin
            outputs <= 18;
            error <= 1'b0;
            end
        1 << 19: begin
            outputs <= 19;
            error <= 1'b0;
            end
        1 << 20: begin
            outputs <= 20;
            error <= 1'b0;
            end
        1 << 21: begin
            outputs <= 21;
            error <= 1'b0;
            end
        1 << 22: begin
            outputs <= 22;
            error <= 1'b0;
            end
        1 << 23: begin
            outputs <= 23;
            error <= 1'b0;
            end
        1 << 24: begin
            outputs <= 24;
            error <= 1'b0;
            end
        1 << 25: begin
            outputs <= 25;
            error <= 1'b0;
            end
        1 << 26: begin
            outputs <= 26;
            error <= 1'b0;
            end
        1 << 27: begin
            outputs <= 27;
            error <= 1'b0;
            end
        1 << 28: begin
            outputs <= 28;
            error <= 1'b0;
            end
        1 << 29: begin
            outputs <= 29;
            error <= 1'b0;
            end
        1 << 30: begin
            outputs <= 30;
            error <= 1'b0;
            end
        1 << 31: begin
            outputs <= 31;
            error <= 1'b0;
            end
        default: begin
            outputs <= 0;
            error <= 1'b1;
            end
    endcase
end

endmodule
