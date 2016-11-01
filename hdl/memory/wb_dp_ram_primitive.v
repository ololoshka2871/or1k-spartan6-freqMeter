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
//****************************************************************************/

`include "config.v"

module wb_dp_ram_primitive
#(
    parameter NUM_OF_MEM_UNITS_TO_USE  = 31,
    parameter DATA_WIDTH = 32,                                                              // width of data bus in bits (8, 16, 32, or 64)
    parameter ADDR_WIDTH = $clog2(NUM_OF_MEM_UNITS_TO_USE * `MEMORY_UNIT_SIZE / 8),         // width of address bus in bits
    parameter SELECT_WIDTH = (DATA_WIDTH/8)                                                 // width of word select bus (1, 2, 4, or 8)
)
(
    // port A
    input  wire                    a_clk,
    input  wire [ADDR_WIDTH-1:0]   a_adr_i,   // ADR_I() address
    input  wire [DATA_WIDTH-1:0]   a_dat_i,   // DAT_I() data in
    output wire [DATA_WIDTH-1:0]   a_dat_o,   // DAT_O() data out
    input  wire                    a_we_i,    // WE_I write enable input
    input  wire [SELECT_WIDTH-1:0] a_sel_i,   // SEL_I() select input
    input  wire                    a_stb_i,   // STB_I strobe input
    output wire                    a_ack_o,   // ACK_O acknowledge output
    input  wire                    a_cyc_i,   // CYC_I cycle input
    output reg 			   a_stall_o, // incorrect address

    // port B
    input  wire                    b_clk,
    input  wire [ADDR_WIDTH-1:0]   b_adr_i,   // ADR_I() address
    input  wire [DATA_WIDTH-1:0]   b_dat_i,   // DAT_I() data in
    output wire [DATA_WIDTH-1:0]   b_dat_o,   // DAT_O() data out
    input  wire                    b_we_i,    // WE_I write enable input
    input  wire [SELECT_WIDTH-1:0] b_sel_i,   // SEL_I() select input
    input  wire                    b_stb_i,   // STB_I strobe input
    output wire                    b_ack_o,   // ACK_O acknowledge output
    input  wire                    b_cyc_i,   // CYC_I cycle input
    output reg 			   b_stall_o  // incorrect address
);

//------------------------------------------------------------------------------

reg a_ack_o_reg = 1'b0;
reg b_ack_o_reg = 1'b0;

//------------------------------------------------------------------------------

wire [ADDR_WIDTH-3:0] a_adr_i_valid = a_adr_i[ADDR_WIDTH-1:2];
wire [ADDR_WIDTH-3:0] b_adr_i_valid = b_adr_i[ADDR_WIDTH-1:2];

//------------------------------------------------------------------------------

wire [8:0]  a_block_addr_each = a_adr_i_valid[8:0];
wire [ADDR_WIDTH-3:9] a_block_addr_sel = a_adr_i_valid[ADDR_WIDTH-3:9];
wire [8:0]  b_block_addr_each = b_adr_i_valid[8:0];
wire [ADDR_WIDTH-3:9] b_block_addr_sel = b_adr_i_valid[ADDR_WIDTH-3:9];

//------------------------------------------------------------------------------

// out data
wire [DATA_WIDTH-1:0] a_dat_o_block     [NUM_OF_MEM_UNITS_TO_USE-1:0];
wire [DATA_WIDTH-1:0] b_dat_o_block     [NUM_OF_MEM_UNITS_TO_USE-1:0];

// we
wire [NUM_OF_MEM_UNITS_TO_USE-1:0] _a_block_sel;
wire [NUM_OF_MEM_UNITS_TO_USE-1:0] _b_block_sel;

wire [3:0] a_block_we [NUM_OF_MEM_UNITS_TO_USE-1:0];
wire [3:0] b_block_we [NUM_OF_MEM_UNITS_TO_USE-1:0];

wire a_access = a_cyc_i & a_stb_i & ~a_ack_o;
wire b_access = b_cyc_i & b_stb_i & ~b_ack_o;

//------------------------------------------------------------------------------

assign wb_stall_o = 1'b0;
assign rawp_stall_o = 1'b0;

assign a_ack_o = a_ack_o_reg;
assign b_ack_o = b_ack_o_reg;

//------------------------------------------------------------------------------

// address to mem_inst decoders
decoder
#(
    .OUTPUTS_COUNT(NUM_OF_MEM_UNITS_TO_USE)
)  a_addr_decoder (
    .inputs(a_block_addr_sel),
    .outputs(_a_block_sel)
), b_addr_decoder (
    .inputs(b_block_addr_sel),
    .outputs(_b_block_sel)
);
//------------------------------------------------------------------------------

parameter IMAGE_FILE_NAME_PATERN_start        = "@IMAGE@-partA";
parameter IMAGE_FILE_NAME_PATERN_end          = ".hex";

genvar i;
generate

for(i = 0; i < NUM_OF_MEM_UNITS_TO_USE; i = i + 1) begin
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
        .SIM_DEVICE ( "SPARTAN6" ),
        .INIT_FILE({IMAGE_FILE_NAME_PATERN_start+i, IMAGE_FILE_NAME_PATERN_end})
    ) mem_inst (
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
        .ADDRA({a_block_addr_each, 5'b0}),
        .ADDRB({b_block_addr_each, 5'b0}),
        .DIA(a_dat_i),
        .DIB(b_dat_i),
        .DIPA(4'b0), // parity inputs
        .DIPB(4'b0),
        .DOA(a_dat_o_block[i]),
        .DOB(b_dat_o_block[i]),
        .DOPA( /* open */ ), // parity outputs
        .DOPB( /* open */ ),
        .WEA(a_block_we[i]),
        .WEB(b_block_we[i])
    );

    // sel + addr -> WE
    assign a_block_we[i] = (_a_block_sel[i] & a_access & a_we_i ) ?
        a_sel_i : 4'b0;
    assign b_block_we[i] = (_b_block_sel[i] & b_access & b_we_i ) ?
        b_sel_i : 4'b0;
end

for (i = 0; i < 4; i = i + 1) begin
    assign a_dat_o[((i + 1) * 8) - 1 -: 8] = a_sel_i[i] ?
        a_dat_o_block[a_block_addr_sel][((i + 1) * 8) - 1 -: 8] : 8'b0;
    assign b_dat_o[((i + 1) * 8) - 1 -: 8] = b_sel_i[i] ?
        b_dat_o_block[b_block_addr_sel][((i + 1) * 8) - 1 -: 8] : 8'b0;
end

endgenerate

always @(posedge a_clk) begin
    a_ack_o_reg <= a_access;
end

always @(posedge b_clk) begin
    b_ack_o_reg <= b_access;
end

endmodule
