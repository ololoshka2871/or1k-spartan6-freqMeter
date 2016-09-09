/*
 * Milkymist VJ SoC
 * Copyright (C) 2007, 2008, 2009, 2010 Sebastien Bourdeauducq
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

module myminimac #(
	parameter csr_addr = 4'h0
) (
        input sys_clk,                                  // WISHBONE clock
        input sys_rst,                                  // GLOBAL RESET

        output irq_rx,                                  // RX interrupt
        output irq_tx,                                  // TX interrupt

        input  wire [31:0]  /*13*/      csr_adr_i,      // control logic addr
        input  wire                     csr_we_i,       // control logick write enable
        input  wire [31:0]              csr_dat_i,      // control logick data input
        output wire [31:0]              csr_dat_o,      // control logick data output

        // system bus port A (rx memory)
        input  wire [31:0]              rx_mem_adr_i,    // ADR_I() address
        input  wire [31:0]              rx_mem_dat_i,    // DAT_I() data in
        output wire [31:0]              rx_mem_dat_o,    // DAT_O() data out
        input  wire                     rx_mem_we_i,     // WE_I write enable input
        input  wire [3:0]               rx_mem_sel_i,    // SEL_I() select input
        input  wire                     rx_mem_stb_i,    // STB_I strobe input
        output wire                     rx_mem_ack_o,    // ACK_O acknowledge output
        input  wire                     rx_mem_cyc_i,    // CYC_I cycle input
        output wire                     rx_mem_stall_o,  // incorrect address

        // system bus port B (tx memory)
        input  wire [31:0]              tx_mem_adr_i,   // ADR_I() address
        input  wire [31:0]              tx_mem_dat_i,   // DAT_I() data in
        output wire [31:0]              tx_mem_dat_o,   // DAT_O() data out
        input  wire                     tx_mem_we_i,    // WE_I write enable input
        input  wire [3:0]               tx_mem_sel_i,   // SEL_I() select input
        input  wire                     tx_mem_stb_i,   // STB_I strobe input
        output wire                     tx_mem_ack_o,   // ACK_O acknowledge output
        input  wire                     tx_mem_cyc_i,   // CYC_I cycle input
        output wire			tx_mem_stall_o, // incorrect address

        // RMII
        output wire                     phy_mdclk,      // MDCLK
        inout  wire                     phy_mdio,       // MDIO
        input  wire                     phy_rmii_clk,   // 50 MHZ input
        input  wire                     phy_rmii_crs,   // Ressiver ressiving data
        output wire [2:0]               phy_tx_data,    // transmit data bis
        input  wire [2:0]               phy_rx_data,    // ressive data bus
        output wire                     phy_tx_en       // transmitter enable
);

assign wbrx_cti_o = 3'd0;
assign wbtx_cti_o = 3'd0;

wire rx_rst;
wire tx_rst;

wire rx_valid;
wire [29:0] rx_adr;
wire rx_resetcount;
wire rx_incrcount;
wire rx_endframe;

wire fifo_full;

wire tx_valid;
wire [29:0] tx_adr;
wire [1:0] tx_bytecount;
wire tx_next;

minimac_ctlif #(
	.csr_addr(csr_addr)
) ctlif (
        .sys_clk(sys_clk),                              // ok
        .sys_rst(sys_rst),                              // ok

        .irq_rx(irq_rx),                                // ok
        .irq_tx(irq_tx),                                // ok

        .csr_a(csr_adr_i),                              // ok
        .csr_we(csr_we_i),                              // ok
        .csr_di(csr_dat_i),                             // ok
        .csr_do(csr_dat_o),                             // ok

        .phy_mii_clk(phy_mii_clk),                      // ok
        .phy_mii_data(phy_mii_data),                    // ok

        //
	.rx_rst(rx_rst),
	.tx_rst(tx_rst),

	.rx_valid(rx_valid),
	.rx_adr(rx_adr),
	.rx_resetcount(rx_resetcount),
	.rx_incrcount(rx_incrcount),
	.rx_endframe(rx_endframe),

	.fifo_full(fifo_full),

	.tx_valid(tx_valid),
	.tx_adr(tx_adr),
	.tx_bytecount(tx_bytecount),
        .tx_next(tx_next)
);

minimac_rx rx(
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),
	.rx_rst(rx_rst),

	.wbm_adr_o(wbrx_adr_o),
	.wbm_cyc_o(wbrx_cyc_o),
	.wbm_stb_o(wbrx_stb_o),
	.wbm_ack_i(wbrx_ack_i),
	.wbm_dat_o(wbrx_dat_o),

	.rx_valid(rx_valid),
	.rx_adr(rx_adr),
	.rx_resetcount(rx_resetcount),
	.rx_incrcount(rx_incrcount),
	.rx_endframe(rx_endframe),

	.fifo_full(fifo_full),

	.phy_rx_clk(phy_rx_clk),
	.phy_rx_data(phy_rx_data),
	.phy_dv(phy_dv),
	.phy_rx_er(phy_rx_er)
);

minimac_tx tx(
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),
	.tx_rst(tx_rst),

	.tx_valid(tx_valid),
	.tx_adr(tx_adr),
	.tx_bytecount(tx_bytecount),
	.tx_next(tx_next),

	.wbtx_adr_o(wbtx_adr_o),
	.wbtx_cyc_o(wbtx_cyc_o),
	.wbtx_stb_o(wbtx_stb_o),
	.wbtx_ack_i(wbtx_ack_i),
	.wbtx_dat_i(wbtx_dat_i),

	.phy_tx_clk(phy_tx_clk),
	.phy_tx_en(phy_tx_en),
	.phy_tx_data(phy_tx_data)
);

endmodule
