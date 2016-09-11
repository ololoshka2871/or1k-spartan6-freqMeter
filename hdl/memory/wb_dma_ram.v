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

module wb_dma_ram
#(
    parameter NUM_OF_MEM_UNITS_TO_USE = 1,

    parameter WB_DATA_WIDTH = 32,                                                   // width of data bus in bits (8, 16, 32, or 64)
    parameter WB_ADDR_WIDTH = $clog2(NUM_OF_MEM_UNITS_TO_USE * `MEMORY_UNIT_SIZE),  // width of address bus in bits
    parameter WB_SELECT_WIDTH = (WB_DATA_WIDTH/8)                                   // width of word select bus (1, 2, 4, or 8)
)
(
    // port A (WB)
    input  wire                         wb_clk,
    input  wire [WB_ADDR_WIDTH-1:0]     wb_adr_i,   // ADR_I() address
    input  wire [WB_DATA_WIDTH-1:0]     wb_dat_i,   // DAT_I() data in
    output wire [WB_DATA_WIDTH-1:0]     wb_dat_o,   // DAT_O() data out
    input  wire                         wb_we_i,    // WE_I write enable input
    input  wire [WB_SELECT_WIDTH-1:0]   wb_sel_i,   // SEL_I() select input
    input  wire                         wb_stb_i,   // STB_I strobe input
    output wire                         wb_ack_o,   // ACK_O acknowledge output
    input  wire                         wb_cyc_i,   // CYC_I cycle input
    output reg                          wb_stall_o, // incorrect address

    // port B (RAW) 8 bit
    input  wire                         rawp_clk,
    input  wire [WB_ADDR_WIDTH-1:0]     rawp_adr_i,  // address
    input  wire [7:0]                   rawp_dat_i,  // data in
    output wire [7:0]                   rawp_dat_o,  // data out
    input  wire                         rawp_we_i,   // write enable input
    output reg                          rawp_stall_o
);

// for interfaces that are more than one word wide, disable address lines
parameter WB_VALID_ADDR_WIDTH = WB_ADDR_WIDTH - $clog2(WB_SELECT_WIDTH);
// width of data port in words (1, 2, 4, or 8)
parameter WORD_WIDTH = WB_SELECT_WIDTH;
// size of words (8, 16, 32, or 64 bits)
parameter WORD_SIZE = WB_DATA_WIDTH/WORD_WIDTH;

parameter MEMORY_SIZE_bits = NUM_OF_MEM_UNITS_TO_USE * `MEMORY_UNIT_SIZE;
parameter MEMORY_CELLS_NUMBER = MEMORY_SIZE_bits / 8;

reg [WB_DATA_WIDTH-1:0] wb_dat_o_reg = {WB_DATA_WIDTH{1'b0}};
reg wb_ack_o_reg = 1'b0;

reg [WB_DATA_WIDTH-1:0] rawp_dat_o_reg;

// (* RAM_STYLE="BLOCK" *)
reg [WB_DATA_WIDTH-1:0] mem[MEMORY_CELLS_NUMBER - 1:0];

wire [WB_VALID_ADDR_WIDTH-1:0] wb_adr_i_valid = wb_adr_i >> (WB_ADDR_WIDTH - WB_VALID_ADDR_WIDTH);
wire [WB_VALID_ADDR_WIDTH-1:0] rawp_adr_i_valid = rawp_adr_i >> (WB_ADDR_WIDTH - WB_VALID_ADDR_WIDTH);
wire [(WB_ADDR_WIDTH - WB_VALID_ADDR_WIDTH)-1:0] raw_select_bits =
    rawp_adr_i[(WB_ADDR_WIDTH - WB_VALID_ADDR_WIDTH)-1:0];
wire [WB_SELECT_WIDTH-1:0] raw_select;

wire wb_incorrect_addr = wb_adr_i > MEMORY_CELLS_NUMBER;
wire rawp_incorrect_addr = wb_adr_i > MEMORY_CELLS_NUMBER;

//------------------------------------------------------------------------------

assign wb_dat_o = wb_dat_o_reg;
assign wb_ack_o = wb_ack_o_reg;

assign rawp_dat_o = rawp_dat_o_reg;

//------------------------------------------------------------------------------

decoder
#(
    .OUTPUTS_COUNT(WB_SELECT_WIDTH)
) selector (
    .inputs(raw_select_bits),
    .outputs(raw_select)
);

//------------------------------------------------------------------------------

integer i, j;

// port WB
always @(posedge wb_clk) begin
    wb_ack_o_reg <= 1'b0;
    wb_stall_o <= wb_incorrect_addr;
    for (i = 0; i < WORD_WIDTH; i = i + 1) begin
        if (wb_cyc_i & wb_stb_i & ~wb_ack_o & ~wb_incorrect_addr) begin
            if (wb_we_i & wb_sel_i[i]) begin
                mem[wb_adr_i_valid][WORD_SIZE*i +: WORD_SIZE] <= wb_dat_i[WORD_SIZE*i +: WORD_SIZE];
            end
            wb_dat_o_reg[WORD_SIZE*i +: WORD_SIZE] <= mem[wb_adr_i_valid][WORD_SIZE*i +: WORD_SIZE];
            wb_ack_o_reg <= 1'b1;
        end
    end
end

// port RAW
always @(posedge rawp_clk) begin
    rawp_stall_o <= rawp_incorrect_addr;
    for (j = 0; j < WORD_WIDTH; j = j + 1) begin
        if (~rawp_incorrect_addr) begin
            if (rawp_we_i & raw_select[i]) begin
                mem[rawp_adr_i_valid][WORD_SIZE*j +: WORD_SIZE] <= rawp_dat_i;
            end
            rawp_dat_o_reg[WORD_SIZE*j +: WORD_SIZE] <= mem[rawp_adr_i_valid][WORD_SIZE*j +: WORD_SIZE];
        end
    end
end

endmodule
