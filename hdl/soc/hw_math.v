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

`include "config.v"

module hw_math
(
    // WISHBONE bus slave interface
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)
    input  wire [3:0]                   addr_i,        // address
    input  wire [31:0]                  dat_i,         // data input
    output wire [31:0]                  dat_o,         // data output
    input  wire				stb_i,         // strobe
    input  wire				we_i           // write enable
);

localparam REG_CRC32_IO = 2'b00;
localparam REG_CRC32_RST = 2'b01;
localparam REG_MULT_IO = 2'b10;
localparam REG_MULT_CTL = 2'b11;

///////////////////////////////////////////////////////////////////////////////

`ifdef CRC32_ENABLED

// addr 000 - add
wire crc32_add_value = stb_i & we_i & (addr_i[3:2] == REG_CRC32_IO);

// addr 100 - reset
wire reset_crc = (stb_i & we_i & (addr_i[3:2] == REG_CRC32_RST)) | rst_i;

// polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1
// data width: 8
// convention: the first serial bit is D[7]
function [31:0] nextCRC32_D8;

input [7:0] Data;
input [31:0] crc;
reg [7:0] d;
reg [31:0] c;
reg [31:0] newcrc;
begin
    d = Data;
    c = crc;

    newcrc[0] = d[6] ^ d[0] ^ c[24] ^ c[30];
    newcrc[1] = d[7] ^ d[6] ^ d[1] ^ d[0] ^ c[24] ^ c[25] ^ c[30] ^ c[31];
    newcrc[2] = d[7] ^ d[6] ^ d[2] ^ d[1] ^ d[0] ^ c[24] ^ c[25] ^ c[26] ^ c[30] ^ c[31];
    newcrc[3] = d[7] ^ d[3] ^ d[2] ^ d[1] ^ c[25] ^ c[26] ^ c[27] ^ c[31];
    newcrc[4] = d[6] ^ d[4] ^ d[3] ^ d[2] ^ d[0] ^ c[24] ^ c[26] ^ c[27] ^ c[28] ^ c[30];
    newcrc[5] = d[7] ^ d[6] ^ d[5] ^ d[4] ^ d[3] ^ d[1] ^ d[0] ^ c[24] ^ c[25] ^ c[27] ^ c[28] ^ c[29] ^ c[30] ^ c[31];
    newcrc[6] = d[7] ^ d[6] ^ d[5] ^ d[4] ^ d[2] ^ d[1] ^ c[25] ^ c[26] ^ c[28] ^ c[29] ^ c[30] ^ c[31];
    newcrc[7] = d[7] ^ d[5] ^ d[3] ^ d[2] ^ d[0] ^ c[24] ^ c[26] ^ c[27] ^ c[29] ^ c[31];
    newcrc[8] = d[4] ^ d[3] ^ d[1] ^ d[0] ^ c[0] ^ c[24] ^ c[25] ^ c[27] ^ c[28];
    newcrc[9] = d[5] ^ d[4] ^ d[2] ^ d[1] ^ c[1] ^ c[25] ^ c[26] ^ c[28] ^ c[29];
    newcrc[10] = d[5] ^ d[3] ^ d[2] ^ d[0] ^ c[2] ^ c[24] ^ c[26] ^ c[27] ^ c[29];
    newcrc[11] = d[4] ^ d[3] ^ d[1] ^ d[0] ^ c[3] ^ c[24] ^ c[25] ^ c[27] ^ c[28];
    newcrc[12] = d[6] ^ d[5] ^ d[4] ^ d[2] ^ d[1] ^ d[0] ^ c[4] ^ c[24] ^ c[25] ^ c[26] ^ c[28] ^ c[29] ^ c[30];
    newcrc[13] = d[7] ^ d[6] ^ d[5] ^ d[3] ^ d[2] ^ d[1] ^ c[5] ^ c[25] ^ c[26] ^ c[27] ^ c[29] ^ c[30] ^ c[31];
    newcrc[14] = d[7] ^ d[6] ^ d[4] ^ d[3] ^ d[2] ^ c[6] ^ c[26] ^ c[27] ^ c[28] ^ c[30] ^ c[31];
    newcrc[15] = d[7] ^ d[5] ^ d[4] ^ d[3] ^ c[7] ^ c[27] ^ c[28] ^ c[29] ^ c[31];
    newcrc[16] = d[5] ^ d[4] ^ d[0] ^ c[8] ^ c[24] ^ c[28] ^ c[29];
    newcrc[17] = d[6] ^ d[5] ^ d[1] ^ c[9] ^ c[25] ^ c[29] ^ c[30];
    newcrc[18] = d[7] ^ d[6] ^ d[2] ^ c[10] ^ c[26] ^ c[30] ^ c[31];
    newcrc[19] = d[7] ^ d[3] ^ c[11] ^ c[27] ^ c[31];
    newcrc[20] = d[4] ^ c[12] ^ c[28];
    newcrc[21] = d[5] ^ c[13] ^ c[29];
    newcrc[22] = d[0] ^ c[14] ^ c[24];
    newcrc[23] = d[6] ^ d[1] ^ d[0] ^ c[15] ^ c[24] ^ c[25] ^ c[30];
    newcrc[24] = d[7] ^ d[2] ^ d[1] ^ c[16] ^ c[25] ^ c[26] ^ c[31];
    newcrc[25] = d[3] ^ d[2] ^ c[17] ^ c[26] ^ c[27];
    newcrc[26] = d[6] ^ d[4] ^ d[3] ^ d[0] ^ c[18] ^ c[24] ^ c[27] ^ c[28] ^ c[30];
    newcrc[27] = d[7] ^ d[5] ^ d[4] ^ d[1] ^ c[19] ^ c[25] ^ c[28] ^ c[29] ^ c[31];
    newcrc[28] = d[6] ^ d[5] ^ d[2] ^ c[20] ^ c[26] ^ c[29] ^ c[30];
    newcrc[29] = d[7] ^ d[6] ^ d[3] ^ c[21] ^ c[27] ^ c[30] ^ c[31];
    newcrc[30] = d[7] ^ d[4] ^ c[22] ^ c[28] ^ c[31];
    newcrc[31] = d[5] ^ c[23] ^ c[29];
    nextCRC32_D8 = newcrc;
end
endfunction

///////////////////////////////////////////////////////////////////////////////

wire [7:0] next_byte = {dat_i[0], dat_i[1], dat_i[2], dat_i[3],
                        dat_i[4], dat_i[5], dat_i[6], dat_i[7]};

reg [31:0] crc32_;
always @(posedge clk_i) begin
    if (reset_crc)
        crc32_ <= 32'hFFFFFFFF;
    if(crc32_add_value)
        crc32_ <= nextCRC32_D8( next_byte , crc32_ );
end

wire [31:0] crc32_res = { ~crc32_[0], ~crc32_[1], ~crc32_[2], ~crc32_[3],
                 ~crc32_[4], ~crc32_[5], ~crc32_[6], ~crc32_[7],
                 ~crc32_[8], ~crc32_[9], ~crc32_[10], ~crc32_[11],
                 ~crc32_[12], ~crc32_[13], ~crc32_[14], ~crc32_[15],
                 ~crc32_[16], ~crc32_[17], ~crc32_[18], ~crc32_[19],
                 ~crc32_[20], ~crc32_[21], ~crc32_[22], ~crc32_[23],
                 ~crc32_[24], ~crc32_[25], ~crc32_[26], ~crc32_[27],
                 ~crc32_[28], ~crc32_[29], ~crc32_[30], ~crc32_[31]};

`else
wire [31:0] crc32_res = 32'd0;
`endif // CRC32_ENABLED

///////////////////////////////////////////////////////////////////////////////

`ifdef HW_MUL_ENABLED

// functionality of gcc math function: int __mulsi3 (int a, int b)

// W -> set arguments | R -> read result
wire multiplyer_io  = stb_i & (addr_i[3:2] == REG_MULT_IO);

// control
wire multiplyer_ctl = stb_i & (addr_i[3:2] == REG_MULT_CTL);


reg signed [31:0] product;

reg signed [31:0] a;
reg signed [31:0] b;

always @(posedge clk_i) begin
    if (rst_i) begin
        a <= 32'd0;
        b <= 32'd0;
    end else begin
        product <= a * b;

        if(multiplyer_io & we_i) begin
            b <= a;
            a <= dat_i;
        end
    end
end

`else
wire [31:0] product = 32'd0;
`endif // HW_MUL_ENABLED

////////////////////////////////////////////////////////////////////////////////

assign dat_o = addr_i[3] ? product : crc32_res;

endmodule
