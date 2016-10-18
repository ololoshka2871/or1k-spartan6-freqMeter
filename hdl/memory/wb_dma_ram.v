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
    parameter WB_ADDR_WIDTH = $clog2(NUM_OF_MEM_UNITS_TO_USE * `MEMORY_UNIT_SIZE / 8),  // width of address bus in bits
    parameter INIT_FILE_NAME = "NONE"
)
(
    // port A (WB)
    input  wire                         wb_clk,
    input  wire [WB_ADDR_WIDTH-1:0]     wb_adr_i,   // ADR_I() address
    input  wire [31:0]                  wb_dat_i,   // DAT_I() data in
    output wire [31:0]                  wb_dat_o,   // DAT_O() data out
    input  wire                         wb_we_i,    // WE_I write enable input
    input  wire [3:0]                   wb_sel_i,   // SEL_I() select input
    input  wire                         wb_stb_i,   // STB_I strobe input
    output wire                         wb_ack_o,   // ACK_O acknowledge output
    input  wire                         wb_cyc_i,   // CYC_I cycle input
    output reg                          wb_stall_o, // incorrect address

    // port B (RAW)
    input  wire                         rawp_clk,
    input  wire [WB_ADDR_WIDTH-1:2]     rawp_adr_i,  // address
    input  wire [31:0]                  rawp_dat_i,  // data in
    output reg  [31:0]                  rawp_dat_o,  // data out
    input  wire                         rawp_we_i,   // write enable input
    output reg                          rawp_stall_o
);

parameter MEMORY_SIZE_bits = NUM_OF_MEM_UNITS_TO_USE * `MEMORY_UNIT_SIZE;
parameter MEMORY_CELLS_NUMBER = MEMORY_SIZE_bits / 32;
parameter WORD_SIZE = 8;
parameter WORD_WIDTH = 32 / WORD_SIZE;

reg [31:0] wb_dat_o_reg = 32'b0;
reg wb_ack_o_reg = 1'b0;

reg [31:0] rawp_dat_o_reg;

// (* RAM_STYLE="BLOCK" *)
reg [31:0] mem[MEMORY_CELLS_NUMBER - 1:0];

wire [WB_ADDR_WIDTH-3:0] wb_adr_i_valid = wb_adr_i[WB_ADDR_WIDTH-1:2];
wire [WB_ADDR_WIDTH-3:0] rawp_adr_i_valid = rawp_adr_i;

wire wb_incorrect_addr = wb_adr_i_valid >= MEMORY_CELLS_NUMBER;
wire rawp_incorrect_addr = rawp_adr_i_valid >= MEMORY_CELLS_NUMBER;

//------------------------------------------------------------------------------

assign wb_dat_o = wb_dat_o_reg;
assign wb_ack_o = wb_ack_o_reg;

//------------------------------------------------------------------------------

initial begin
    if (INIT_FILE_NAME != "NONE") begin
        $readmemh(INIT_FILE_NAME, mem);
    end
end

//------------------------------------------------------------------------------

integer i;

// port WB
always @(posedge wb_clk) begin
    wb_ack_o_reg <= 1'b0;
    wb_stall_o <= wb_incorrect_addr;
    for (i = 0; i < WORD_WIDTH; i = i + 1) begin
        if (wb_cyc_i & wb_stb_i & ~wb_ack_o & ~wb_incorrect_addr) begin
            if (wb_we_i & wb_sel_i[i]) begin
                mem[wb_adr_i_valid][WORD_SIZE*i +: WORD_SIZE] <=
                    wb_dat_i[WORD_SIZE*i +: WORD_SIZE];
                wb_dat_o_reg[WORD_SIZE*i +: WORD_SIZE] <=
                    wb_dat_i[WORD_SIZE*i +: WORD_SIZE];
            end else begin
                wb_dat_o_reg[WORD_SIZE*i +: WORD_SIZE] <=
                    mem[wb_adr_i_valid][WORD_SIZE*i +: WORD_SIZE];
            end
            wb_ack_o_reg <= 1'b1;
        end
    end
end

// port RAW
always @(posedge rawp_clk) begin
    rawp_stall_o <= rawp_incorrect_addr;
    if (~rawp_incorrect_addr) begin
        if (rawp_we_i) begin
            mem[rawp_adr_i_valid] <= rawp_dat_i;
            rawp_dat_o <= rawp_dat_i;
        end else begin
            rawp_dat_o <= mem[rawp_adr_i_valid];
        end
    end
end

endmodule
