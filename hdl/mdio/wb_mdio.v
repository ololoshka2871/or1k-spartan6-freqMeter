//****************************************************************************
//*
//*   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
//*   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
//*   Based on: Milkymist VJ SoC 2007, 2008, 2009, 2010 Sebastien Bourdeauducq
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

module wb_mdio
#(
    parameter MASTER_CLK_FREQ_HZ = 50000000, /* Hz    */
    parameter MDIO_BAUDRATE      = 2500000   /* bit/s */
)(
    // WISHBONE bus slave interface
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)
    input  wire				cyc_i,         // cycle
    input  wire				stb_i,         // strobe
    input  wire [2:0]                   adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [31:0]                  dat_i,         // data input
    output reg  [31:0]                  dat_o,         // data output

    output wire				inta_o,        // interrupt output

    inout  wire                         mdio,          // MDIO to pin
    output wire                         mdclk_o        // MDCLK to pin
);

parameter MDIO_PHY_ADDR_LEN = 5;
parameter MDIO_PHY_REG_ADDR_LEN = 5;
parameter MDIO_DATA_LEN = 16;

parameter MDIO_REG_CTRL = 1'b0;
parameter MDIO_REG_DATA = 1'b1;

parameter MDIO_CTRL_START_BIT = 31;
parameter MDIO_CTRL_RW_BIT = 30;
parameter MDIO_CTRL_IE_BIT = 29;
parameter MDIO_CTRL_IF_BIT = 28;

reg [MDIO_PHY_ADDR_LEN + MDIO_PHY_REG_ADDR_LEN - 1 : 0] phy_addr;
reg [MDIO_DATA_LEN - 1 : 0] mdio_wr_data;

reg start_pulse;
reg rw;
reg mdio_ie;

//------------------------------------------------------------------------------

wire wb_addr_valid = adr_i[2];
wire [MDIO_DATA_LEN - 1 : 0] mdio_rd_data;
wire ready;

wire [MDIO_PHY_ADDR_LEN - 1 : 0] phy_addr_selector =
    phy_addr[MDIO_PHY_ADDR_LEN + MDIO_PHY_REG_ADDR_LEN - 1 : MDIO_PHY_REG_ADDR_LEN];
wire [MDIO_PHY_REG_ADDR_LEN - 1 : 0] phy_reg_selector =
    phy_addr[MDIO_PHY_REG_ADDR_LEN - 1 : 0];

//------------------------------------------------------------------------------

mdio_read_write
#(
    .MASTER_CLK_FREQ_HZ(MASTER_CLK_FREQ_HZ),
    .MDIO_BAUDRATE(MDIO_BAUDRATE)
) mdio_ip (
    .ip_sync_reset(rst_i),
    .ip_master_clk(clk_i),

    .op_mdio_clk(mdclk_o),
    .io_mdio_data(mdio),

    .ip_rd_wr(rw),                  // 0 - read, 1 - write
    .ip_start(start_pulse),         // begin reansaction pulse
    .ip_phy_addr(phy_addr_selector),// dest PHY
    .ip_reg_addr(phy_reg_selector), // dest PHY register addr
    .ip_data(mdio_wr_data),         // data to send
    .op_data(mdio_rd_data),         // result
    .op_data_ready(ready)           // to interrupt, make pulse from it
);

//------------------------------------------------------------------------------


always @(posedge clk_i) begin
    if (rst_i) begin
        phy_addr <= 0;
        mdio_wr_data <= 0;
        start_pulse <= 1'b0;
        mdio_ie <= 1'b0;
        dat_o <= 32'b0;
    end else begin
        start_pulse <= 1'b0;

        if (cyc_i & stb_i) begin
            if (we_i) begin
                // write
                if (wb_addr_valid == MDIO_REG_DATA) begin
                    mdio_wr_data <= dat_i[MDIO_DATA_LEN - 1 : 0];
                end else begin
                    phy_addr <= dat_i[MDIO_PHY_ADDR_LEN + MDIO_PHY_REG_ADDR_LEN - 1 : 0];
                    rw <= dat_i[MDIO_CTRL_RW_BIT];
                    mdio_ie <= dat_i[MDIO_CTRL_IE_BIT];
                    start_pulse <= dat_i[MDIO_CTRL_START_BIT];
                end
            end else begin
                // read
                if (wb_addr_valid == MDIO_REG_DATA) begin
                    dat_o <= {{(32-MDIO_DATA_LEN){1'b0}}, mdio_rd_data};
                end else begin
                    dat_o[MDIO_CTRL_START_BIT] <= 1'b0;
                    dat_o[MDIO_CTRL_IE_BIT] <= mdio_ie;
                    dat_o[MDIO_CTRL_RW_BIT] <= rw;
                    dat_o[MDIO_CTRL_IF_BIT] <= ready;
                    dat_o[MDIO_PHY_ADDR_LEN + MDIO_PHY_REG_ADDR_LEN - 1 : 0] <= phy_addr;
                end
            end
        end
    end
end

assign inta_o = mdio_ie & ready;

endmodule
