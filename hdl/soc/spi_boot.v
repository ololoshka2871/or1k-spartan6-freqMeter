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
//* this module provides access to FPGA boot SPI flash
//*
//****************************************************************************/

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "config.v"

`define     CS_REGISTER_ADDR        3'b101

module spi_boot
#(
    parameter WB_DATA_WIDTH     =	32,
    parameter SPI_CLK_DEVIDER   =        4             // actualy 2^SPI_CLK_DEVIDER
) (
    // WISHBONE bus slave interface
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)
    input  wire				cyc_i,         // cycle
    input  wire				stb_i,         // strobe
    input  wire [7:0]                   adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [WB_DATA_WIDTH - 1:0]	dat_i,         // data input
    output wire [WB_DATA_WIDTH - 1:0]	dat_o,         // data output
    output wire				ack_o,         // normal bus termination
    output wire				inta_o,        // interrupt output

    // SPI wires
    output wire				sck_o,         // serial clock output
    output wire				mosi_o,        // MasterOut SlaveIN
    input  wire				miso_i,        // MasterIn SlaveOut

    output reg   [6:0]                  cs_o           // Crystall select control
);

wire[2:0]   addr3 = adr_i[4:2];
wire[7:0]   data8_i;
reg [3:0]   cs_reg = 4'b0000;
reg         ack_cs;
wire        ack_spi;

wire cs_reg_sel = (addr3 == `CS_REGISTER_ADDR);
wire cyc_spi = cyc_i & ~cs_reg_sel;

assign ack_o = cs_reg_sel ? ack_cs : ack_spi;
assign data8_i = dat_i[7:0];

/*
wire[7:0]   data8_o;

simple_spi_top spi (
    .clk_i(clk_i),
    .rst_i(~rst_i),
    .cyc_i(cyc_spi),
    .stb_i(stb_i),
    .adr_i(addr3[1:0]),
    .we_i(we_i),
    .dat_i(data8_i),
    .dat_o(data8_o),
    .ack_o(ack_spi),
    .inta_o(inta_o),

    .sck_o(sck_o),
    .mosi_o(mosi_o),
    .miso_i(miso_i)
);*/

wire[WB_DATA_WIDTH-1:0]   data_spi_o;

tiny_spi #
(
    .SPI_MODE(-1),
    .BAUD_WIDTH()
) spi (
    .rst_i(rst_i),
    .clk_i(clk_i),

    .stb_i(stb_i),
    .we_i(we_i),
    .dat_o(data_spi_o),
    .dat_i(dat_i),
    .int_o(inta_o),
    .adr_i(addr3),
    .cyc_i(cyc_spi),
    .ack_o(ack_spi),

    .MOSI(mosi_o),
    .SCLK(sck_o),
    .MISO(miso_i)
);

//-----------------------------------------------------------------
// Peripheral Register Write
//-----------------------------------------------------------------
always @ (posedge rst_i or posedge clk_i )
begin
   if (rst_i == 1'b1)
   begin // reset
        cs_reg <= 4'b0;
   end
   else
   begin
        if (cs_reg_sel & stb_i)
        begin
            if (we_i) // write
            begin
                cs_reg <= data8_i[3:0];
            end
        end
   end
end

//-----------------------------------------------------------------
// Ack reset
//-----------------------------------------------------------------
always @ (posedge clk_i )
begin
    if (~stb_i)
        ack_cs <= 1'b0;
    else
        if (cs_reg_sel & stb_i)
            ack_cs <= 1'b1; // cs_ack if read/write
end

//-----------------------------------------------------------------
// Peripheral Register Read
//-----------------------------------------------------------------
assign dat_o = cs_reg_sel ?
    {{(WB_DATA_WIDTH - 4){1'b0}}, cs_reg} :
//    {{(WB_DATA_WIDTH - 8){1'b0}}, data8_o};
    data_spi_o;

//------------------------------------------------------------------
// cs selector
//-----------------------------------------------------------------

always @ (posedge rst_i or posedge clk_i )
begin
    if (rst_i == 1'b1)
    begin // reset
         cs_o <= ~7'b0000000;
    end
    else
    begin
        case(cs_reg)
        1'h0: cs_o <= ~7'b000000;
        1'h1: cs_o <= ~7'b000001;
        1'h2: cs_o <= ~7'b000010;
        1'h3: cs_o <= ~7'b000100;
        1'h4: cs_o <= ~7'b001000;
        1'h5: cs_o <= ~7'b010000;
        1'h6: cs_o <= ~7'b100000;
        endcase
    end
end

endmodule
