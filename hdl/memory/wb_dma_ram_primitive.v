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

module wb_dma_ram_primitive
#(
    parameter NUM_OF_MEM_UNITS_TO_USE = 1,
    parameter WB_ADDR_WIDTH = $clog2(NUM_OF_MEM_UNITS_TO_USE * `MEMORY_UNIT_SIZE / 8),  // width of address bus in bits
    parameter INIT_FILE_NAME = "NONE" // unused
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
    output wire                         wb_stall_o, // incorrect address

    // port B (RAW)
    input  wire                         rawp_clk,
    input  wire [WB_ADDR_WIDTH-1:2]     rawp_adr_i,  // address
    input  wire [31:0]                  rawp_dat_i,  // data in
    output wire [31:0]                  rawp_dat_o,  // data out
    input  wire                         rawp_we_i,   // write enable input
    output wire                         rawp_stall_o
);

reg wb_ack_o_reg = 1'b0;

wire [WB_ADDR_WIDTH-3:0] wb_adr_i_valid = wb_adr_i[WB_ADDR_WIDTH-1:2];
wire [WB_ADDR_WIDTH-3:0] rawp_adr_i_valid = rawp_adr_i;

wire [8:0]  wb_block_addr_each = wb_adr_i_valid[8:0];
wire [WB_ADDR_WIDTH-3:9] wb_block_addr_sel = wb_adr_i_valid[WB_ADDR_WIDTH-3:9];
wire [8:0]  rawp_block_addr_each = rawp_adr_i_valid[8:0];
wire [WB_ADDR_WIDTH-3:9] rawp_block_addr_sel = rawp_adr_i_valid[WB_ADDR_WIDTH-3:9];

// out data
wire [31:0] wb_dat_o_block   [NUM_OF_MEM_UNITS_TO_USE-1:0];
wire [31:0] rawp_dat_o_block [NUM_OF_MEM_UNITS_TO_USE-1:0];

// we
wire [NUM_OF_MEM_UNITS_TO_USE-1:0] raw_block_sel;
wire [NUM_OF_MEM_UNITS_TO_USE-1:0] _wb_block_sel;

wire [3:0] wb_block_we [NUM_OF_MEM_UNITS_TO_USE-1:0];
wire [NUM_OF_MEM_UNITS_TO_USE-1:0] raw_block_we;

wire wb_access = wb_cyc_i & wb_stb_i & ~wb_ack_o;

//------------------------------------------------------------------------------

// address to mem_inst decoders
decoder
#(
    .OUTPUTS_COUNT(NUM_OF_MEM_UNITS_TO_USE)
)  wb_addr_decoder (
    .inputs(wb_block_addr_sel),
    .outputs(_wb_block_sel)
), rawp_addr_decoder (
    .inputs(rawp_block_addr_sel),
    .outputs(raw_block_sel)
);

//------------------------------------------------------------------------------

assign wb_stall_o = 1'b0;
assign rawp_stall_o = 1'b0;

assign rawp_dat_o = rawp_dat_o_block[rawp_block_addr_sel];

assign wb_ack_o = wb_ack_o_reg;

//------------------------------------------------------------------------------


genvar i;
generate

// A - raw
// B - WB

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

        .INITP_00 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INITP_01 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INITP_02 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INITP_03 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INITP_04 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INITP_05 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INITP_06 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INITP_07 ( 256'h0000000000000000000000000000000000000000000000000000000000000000 ),
        .INIT_00 ( 256'h0000000700000006000000050000000400000003000000020000000100000000 ),
        .INIT_01 ( 256'h0000000F0000000E0000000D0000000C0000000B0000000A0000000900000008 ),
        .INIT_02 ( 256'h0000001700000016000000150000001400000013000000120000001100000010 ),
        .INIT_03 ( 256'h0000001F0000001E0000001D0000001C0000001B0000001A0000001900000018 ),
        .INIT_04 ( 256'h0000002700000026000000250000002400000023000000220000002100000020 ),
        .INIT_05 ( 256'h0000002F0000002E0000002D0000002C0000002B0000002A0000002900000028 ),
        .INIT_06 ( 256'h0000003700000036000000350000003400000033000000320000003100000030 ),
        .INIT_07 ( 256'h0000003F0000003E0000003D0000003C0000003B0000003A0000003900000038 ),
        .INIT_08 ( 256'h0000004700000046000000450000004400000043000000420000004100000040 ),
        .INIT_09 ( 256'h0000004F0000004E0000004D0000004C0000004B0000004A0000004900000048 ),
        .INIT_0A ( 256'h0000005700000056000000550000005400000053000000520000005100000050 ),
        .INIT_0B ( 256'h0000005F0000005E0000005D0000005C0000005B0000005A0000005900000058 ),
        .INIT_0C ( 256'h0000006700000066000000650000006400000063000000620000006100000060 ),
        .INIT_0D ( 256'h0000006F0000006E0000006D0000006C0000006B0000006A0000006900000068 ),
        .INIT_0E ( 256'h0000007700000076000000750000007400000073000000720000007100000070 ),
        .INIT_0F ( 256'h0000007F0000007E0000007D0000007C0000007B0000007A0000007900000078 ),
        .INIT_10 ( 256'h0000008700000086000000850000008400000083000000820000008100000080 ),
        .INIT_11 ( 256'h0000008F0000008E0000008D0000008C0000008B0000008A0000008900000088 ),
        .INIT_12 ( 256'h0000009700000096000000950000009400000093000000920000009100000090 ),
        .INIT_13 ( 256'h0000009F0000009E0000009D0000009C0000009B0000009A0000009900000098 ),
        .INIT_14 ( 256'h000000A7000000A6000000A5000000A4000000A3000000A2000000A1000000A0 ),
        .INIT_15 ( 256'h000000AF000000AE000000AD000000AC000000AB000000AA000000A9000000A8 ),
        .INIT_16 ( 256'h000000B7000000B6000000B5000000B4000000B3000000B2000000B1000000B0 ),
        .INIT_17 ( 256'h000000BF000000BE000000BD000000BC000000BB000000BA000000B9000000B8 ),
        .INIT_18 ( 256'h000000C7000000C6000000C5000000C4000000C3000000C2000000C1000000C0 ),
        .INIT_19 ( 256'h000000CF000000CE000000CD000000CC000000CB000000CA000000C9000000C8 ),
        .INIT_1A ( 256'h000000D7000000D6000000D5000000D4000000D3000000D2000000D1000000D0 ),
        .INIT_1B ( 256'h000000DF000000DE000000DD000000DC000000DB000000DA000000D9000000D8 ),
        .INIT_1C ( 256'h000000E7000000E6000000E5000000E4000000E3000000E2000000E1000000E0 ),
        .INIT_1D ( 256'h000000EF000000EE000000ED000000EC000000EB000000EA000000E9000000E8 ),
        .INIT_1E ( 256'h000000F7000000F6000000F5000000F4000000F3000000F2000000F1000000F0 ),
        .INIT_1F ( 256'h000000FF000000FE000000FD000000FC000000FB000000FA000000F9000000F8 ),
        .INIT_20 ( 256'h0000010700000106000001050000010400000103000001020000010100000100 ),
        .INIT_21 ( 256'h0000010F0000010E0000010D0000010C0000010B0000010A0000010900000108 ),
        .INIT_22 ( 256'h0000011700000116000001150000011400000113000001120000011100000110 ),
        .INIT_23 ( 256'h0000011F0000011E0000011D0000011C0000011B0000011A0000011900000118 ),
        .INIT_24 ( 256'h0000012700000126000001250000012400000123000001220000012100000120 ),
        .INIT_25 ( 256'h0000012F0000012E0000012D0000012C0000012B0000012A0000012900000128 ),
        .INIT_26 ( 256'h0000013700000136000001350000013400000133000001320000013100000130 ),
        .INIT_27 ( 256'h0000013F0000013E0000013D0000013C0000013B0000013A0000013900000138 ),
        .INIT_28 ( 256'h0000014700000146000001450000014400000143000001420000014100000140 ),
        .INIT_29 ( 256'h0000014F0000014E0000014D0000014C0000014B0000014A0000014900000148 ),
        .INIT_2A ( 256'h0000015700000156000001550000015400000153000001520000015100000150 ),
        .INIT_2B ( 256'h0000015F0000015E0000015D0000015C0000015B0000015A0000015900000158 ),
        .INIT_2C ( 256'h0000016700000166000001650000016400000163000001620000016100000160 ),
        .INIT_2D ( 256'h0000016F0000016E0000016D0000016C0000016B0000016A0000016900000168 ),
        .INIT_2E ( 256'h0000017700000176000001750000017400000173000001720000017100000170 ),
        .INIT_2F ( 256'h0000017F0000017E0000017D0000017C0000017B0000017A0000017900000178 ),
        .INIT_30 ( 256'h0000018700000186000001850000018400000183000001820000018100000180 ),
        .INIT_31 ( 256'h0000018F0000018E0000018D0000018C0000018B0000018A0000018900000188 ),
        .INIT_32 ( 256'h0000019700000196000001950000019400000193000001920000019100000190 ),
        .INIT_33 ( 256'h0000019F0000019E0000019D0000019C0000019B0000019A0000019900000198 ),
        .INIT_34 ( 256'h000001A7000001A6000001A5000001A4000001A3000001A2000001A1000001A0 ),
        .INIT_35 ( 256'h000001AF000001AE000001AD000001AC000001AB000001AA000001A9000001A8 ),
        .INIT_36 ( 256'h000001B7000001B6000001B5000001B4000001B3000001B2000001B1000001B0 ),
        .INIT_37 ( 256'h000001BF000001BE000001BD000001BC000001BB000001BA000001B9000001B8 ),
        .INIT_38 ( 256'h000001C7000001C6000001C5000001C4000001C3000001C2000001C1000001C0 ),
        .INIT_39 ( 256'h000001CF000001CE000001CD000001CC000001CB000001CA000001C9000001C8 ),
        .INIT_3A ( 256'h000001D7000001D6000001D5000001D4000001D3000001D2000001D1000001D0 ),
        .INIT_3B ( 256'h000001DF000001DE000001DD000001DC000001DB000001DA000001D9000001D8 ),
        .INIT_3C ( 256'h000001E7000001E6000001E5000001E4000001E3000001E2000001E1000001E0 ),
        .INIT_3D ( 256'h000001EF000001EE000001ED000001EC000001EB000001EA000001E9000001E8 ),
        .INIT_3E ( 256'h000001F7000001F6000001F5000001F4000001F3000001F2000001F1000001F0 ),
        .INIT_3F ( 256'h000001FF000001FE000001FD000001FC000001FB000001FA000001F9000001F8 )
    ) mem_inst  (
        .CLKA(rawp_clk),
        .CLKB(wb_clk),
        .ENA(1'b1),
        .ENB(1'b1),
        .REGCEA(1'b0),
        .REGCEB(1'b0),
        .RSTA(1'b0),
        .RSTB(1'b0),
        // http://www.xilinx.com/support/documentation/user_guides/ug383.pdf
        // for this mode (512 x 32) used only [13:5]
        .ADDRA({rawp_block_addr_each, 5'b0}),
        .ADDRB({wb_block_addr_each, 5'b0}),
        .DIA(rawp_dat_i),
        .DIB(wb_dat_i),
        .DIPA(4'b0), // parity inputs
        .DIPB(4'b0),
        .DOA(rawp_dat_o_block[i]),
        .DOB(wb_dat_o_block[i]),
        .DOPA( /* open */ ), // parity outputs
        .DOPB( /* open */ ),
        .WEA({4{raw_block_we[i]}}),
        .WEB(wb_block_we[i])
    );

    assign wb_block_we[i] = (_wb_block_sel[i] & wb_access & wb_we_i ) ?
        wb_sel_i : 4'b0; // sel + addr -> WEB

    assign raw_block_we[i] = raw_block_sel[i] & rawp_we_i; // addr -> WEA
end

for (i = 0; i < 4; i = i + 1) begin
    assign wb_dat_o[((i + 1) * 8) - 1 -: 8] = wb_sel_i[i] ?
        wb_dat_o_block[wb_block_addr_sel][((i + 1) * 8) - 1 -: 8] : 8'b0;
end

endgenerate

integer j;

always @(posedge wb_clk) begin
    wb_ack_o_reg <= wb_access;
end

endmodule
