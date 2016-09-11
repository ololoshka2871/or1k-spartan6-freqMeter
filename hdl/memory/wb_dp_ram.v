/*

Copyright (c) 2015-2016 Alex Forencich
    Modified by: Shilo_XyZ_

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`include "config.v"

module wb_dp_ram
#(
    parameter LOAD_IMAGE = 0,
    parameter NUM_OF_SYS_MEM_UNITS  = 31,
    parameter DATA_WIDTH = 32,                                                              // width of data bus in bits (8, 16, 32, or 64)
    parameter ADDR_WIDTH = $clog2(NUM_OF_SYS_MEM_UNITS * `MEMORY_UNIT_SIZE),                // width of address bus in bits
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

// for interfaces that are more than one word wide, disable address lines
parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(SELECT_WIDTH);
// width of data port in words (1, 2, 4, or 8)
parameter WORD_WIDTH = SELECT_WIDTH;
// size of words (8, 16, 32, or 64 bits)
parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH;

parameter MEMORY_SIZE_bits = NUM_OF_SYS_MEM_UNITS * `MEMORY_UNIT_SIZE;
parameter MEMORY_CELLS_NUMBER = MEMORY_SIZE_bits / DATA_WIDTH;

reg [DATA_WIDTH-1:0] a_dat_o_reg = {DATA_WIDTH{1'b0}};
reg a_ack_o_reg = 1'b0;

reg [DATA_WIDTH-1:0] b_dat_o_reg = {DATA_WIDTH{1'b0}};
reg b_ack_o_reg = 1'b0;

// (* RAM_STYLE="BLOCK" *)
reg [DATA_WIDTH-1:0] mem[MEMORY_CELLS_NUMBER - 1:0];

wire [VALID_ADDR_WIDTH-1:0] a_adr_i_valid = a_adr_i >> (ADDR_WIDTH - VALID_ADDR_WIDTH);
wire [VALID_ADDR_WIDTH-1:0] b_adr_i_valid = b_adr_i >> (ADDR_WIDTH - VALID_ADDR_WIDTH);

wire a_incorrect_addr = a_adr_i > MEMORY_CELLS_NUMBER;
wire b_incorrect_addr = b_adr_i > MEMORY_CELLS_NUMBER;

assign a_dat_o = a_dat_o_reg;
assign a_ack_o = a_ack_o_reg;

assign b_dat_o = b_dat_o_reg;
assign b_ack_o = b_ack_o_reg;

initial begin
    if (LOAD_IMAGE != 0) begin
        $readmemh("@IMAGE@", mem);
    end
end

integer i, j;

// port A
always @(posedge a_clk) begin
    a_ack_o_reg <= 1'b0;
    a_stall_o <= a_incorrect_addr;
    for (i = 0; i < WORD_WIDTH; i = i + 1) begin
	if (a_cyc_i & a_stb_i & ~a_ack_o & ~a_incorrect_addr) begin
            if (a_we_i & a_sel_i[i]) begin
                mem[a_adr_i_valid][WORD_SIZE*i +: WORD_SIZE] <= a_dat_i[WORD_SIZE*i +: WORD_SIZE];
            end
            a_dat_o_reg[WORD_SIZE*i +: WORD_SIZE] <= mem[a_adr_i_valid][WORD_SIZE*i +: WORD_SIZE];
            a_ack_o_reg <= 1'b1;
        end
    end
end

// port B
always @(posedge b_clk) begin
    b_ack_o_reg <= 1'b0;
    b_stall_o <= b_incorrect_addr;
    for (j = 0; j < WORD_WIDTH; j = j + 1) begin
	if (b_cyc_i & b_stb_i & ~b_ack_o & ~b_incorrect_addr) begin
	    if (b_we_i & b_sel_i[j]) begin
		mem[b_adr_i_valid][WORD_SIZE*j +: WORD_SIZE] <= b_dat_i[WORD_SIZE*j +: WORD_SIZE];
            end
	    b_dat_o_reg[WORD_SIZE*j +: WORD_SIZE] <= mem[b_adr_i_valid][WORD_SIZE*j +: WORD_SIZE];
            b_ack_o_reg <= 1'b1;
        end
    end
end

endmodule
