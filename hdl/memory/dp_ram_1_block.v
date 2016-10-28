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

module dp_ram_1_block
#(
    parameter DATA_WIDTH_A = 32,
    parameter DATA_WIDTH_B = 32,
    parameter ADDL_LEN = 9
)
(
    // port A (WB)
    // port B (RAW)
    input  wire                         a_clk,
    input  wire [ADDL_LEN-1:0]          a_adr_i,  // address
    input  wire [DATA_WIDTH_A-1:0]      a_dat_i,  // data in
    output wire [DATA_WIDTH_A-1:0]      a_dat_o,  // data out
    input  wire                         a_we_i,   // write enable input

    // port B (RAW)
    input  wire                         b_clk,
    input  wire [ADDL_LEN-1:0]          b_adr_i,  // address
    input  wire [DATA_WIDTH_B-1:0]      b_dat_i,  // data in
    output wire [DATA_WIDTH_B-1:0]      b_dat_o,  // data out
    input  wire                         b_we_i    // write enable input
);

wire [31:0] a_Di = {{(32-DATA_WIDTH_A){1'b0}}, a_dat_i};
wire [31:0] b_Di = {{(32-DATA_WIDTH_B){1'b0}}, b_dat_i};

wire [31:0] a_Do;
wire [31:0] b_Do;

assign a_dat_o = a_Do[DATA_WIDTH_A-1:0];
assign b_dat_o = b_Do[DATA_WIDTH_B-1:0];

RAMB16BWER #(
    .DATA_WIDTH_A ( 36 ),
    .DATA_WIDTH_B ( 36 ),
    .DOA_REG ( 0 ),
    .DOB_REG ( 0 ),
    .EN_RSTRAM_A ( "TRUE" ),
    .EN_RSTRAM_B ( "TRUE" ),
    .RST_PRIORITY_A ( "CE" ),
    .RST_PRIORITY_B ( "CE" ),
    .RSTTYPE ( "SYNC" ),
    .WRITE_MODE_A ( "WRITE_FIRST" ),
    .WRITE_MODE_B ( "WRITE_FIRST" ),
    .INIT_A ( 36'h000000000 ),
    .INIT_B ( 36'h000000000 ),
    .SRVAL_A ( 36'h000000000 ),
    .SRVAL_B ( 36'h000000000 ),
    .SIM_COLLISION_CHECK ( "ALL" ),
    .SIM_DEVICE ( "SPARTAN6" )
) mem_inst  (
    .CLKA(a_clk),
    .CLKB(b_clk),
    .ENA(1'b1),
    .ENB(1'b1),
    .REGCEA(1'b0),
    .REGCEB(1'b0),
    .RSTA(1'b0),
    .RSTB(1'b0),
    // http://www.xilinx.com/support/documentation/user_guides/ug383.pdf
    // for this mode (512 x 32) used only [13:5]
    .ADDRA({a_adr_i, 5'b0}),
    .ADDRB({b_adr_i, 5'b0}),
    .DIA(a_Di),
    .DIB(b_Di),
    .DIPA(4'b0), // parity inputs
    .DIPB(4'b0),
    .DOA(a_Do),
    .DOB(b_Do),
    .DOPA( /* open */ ), // parity outputs
    .DOPB( /* open */ ),
    .WEA({4{a_we_i}}),
    .WEB({4{b_we_i}})
);

endmodule
