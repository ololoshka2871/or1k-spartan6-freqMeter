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

module tb_myminimac();

/* 25MHz system clock */
reg sys_clk;
initial sys_clk = 1'b0;
always #20 sys_clk = ~sys_clk;

/* 50 MHz rmii clock */
reg rmii_clk;
initial rmii_clk = 1'b0;
always #10 rmii_clk = ~rmii_clk;

reg sys_rst;

reg [31:0] csr_a;
reg csr_we;
reg [31:0] csr_di;
wire [31:0] csr_do;


reg phy_rmii_crs;
wire [1:0] phy_rmii_tx_data;
reg [1:0] phy_rmii_rx_data;
wire phy_tx_en;

wire irq_rx;
wire irq_tx;

myminimac
#(
    .RX_MEMORY_BASE(32'h11300000),
    .TX_MEMORY_BASE(32'h11200000)
) ethernet (
    .sys_clk(sys_clk),
    .sys_rst(sys_rst),

    .irq_rx(irq_rx),
    .irq_tx(irq_tx),

    .csr_adr_i(csr_a),
    .csr_we_i(csr_we),
    .csr_dat_i(csr_di),
    .csr_dat_o(csr_do),

    .rx_mem_adr_i(32'b0),
    .rx_mem_dat_i(32'b0),
    .rx_mem_dat_o(),
    .rx_mem_we_i(1'b0),
    .rx_mem_sel_i(4'b0),
    .rx_mem_stb_i(1'b0),
    .rx_mem_ack_o(),
    .rx_mem_cyc_i(1'b0),
    .rx_mem_stall_o(),

    .tx_mem_adr_i(32'b0),
    .tx_mem_dat_i(32'b0),
    .tx_mem_dat_o(),
    .tx_mem_we_i(1'b0),
    .tx_mem_sel_i(4'b0),
    .tx_mem_stb_i(1'b0),
    .tx_mem_ack_o(),
    .tx_mem_cyc_i(1'b0),
    .tx_mem_stall_o(),

    .phy_mdclk(),
    .phy_mdio(),
    .phy_rmii_clk(rmii_clk),
    .phy_rmii_crs(phy_rmii_crs),
    .phy_rmii_tx_data(phy_rmii_tx_data),
    .phy_rmii_rx_data(phy_rmii_rx_data),
    .phy_tx_en(phy_tx_en)
);

task waitclock;
begin
    @(posedge sys_clk);
    #1;
end
endtask

task csrwrite;
input [29:0] address;
input [31:0] data;
begin
    csr_a = {address, 2'b00};
    csr_di = data;
    csr_we = 1'b1;
    waitclock;
    $display("Configuration Write: %x=%x", address, data);
    csr_we = 1'b0;
end
endtask

task csrread;
input [29:0] address;
begin
    csr_a = {address, 2'b00};
    waitclock;
    $display("Configuration Read : %x=%x", address, csr_do);
end
endtask

/*
always @(posedge sys_clk) begin
    if(wbrx_cyc_o & wbrx_stb_o & ~wbrx_ack_i & (($random % 5) == 0)) begin
        $display("Write: %x <- %x", wbrx_adr_o, wbrx_dat_o);
        wbrx_ack_i = 1'b1;
    end else
        wbrx_ack_i = 1'b0;
end

always @(posedge sys_clk) begin
    if(wbtx_cyc_o & wbtx_stb_o & ~wbtx_ack_i & (($random % 5) == 0)) begin
        wbtx_dat_i = $random;
        $display("Read : %x -> %x", wbtx_adr_o, wbtx_dat_i);
        wbtx_ack_i = 1'b1;
    end else
        wbtx_ack_i = 1'b0;
end
*/

// rx random pocket gen
always @(negedge rmii_clk) begin
    phy_rmii_rx_data <= $random;
    if(phy_rmii_crs) begin
        $display("rx: %x", phy_rmii_rx_data);
        if(($random % 125) == 0) begin
            phy_rmii_crs <= 1'b0;
            $display("** stopping transmission");
        end
    end else begin
        if(($random % 12) == 0) begin
            phy_rmii_crs <= 1'b1;
            $display("** starting transmission");
        end
    end
end

// tx info
always @(posedge rmii_clk) begin
    if(phy_tx_en)
        $display("tx: %x", phy_rmii_tx_data);
end

initial begin
    /* Reset / Initialize our logic */
    sys_rst = 1'b1;

    csr_a = 14'd0;
    csr_di = 32'd0;
    csr_we = 1'b0;
    phy_rmii_crs = 1'b0;

    waitclock;

    sys_rst = 1'b0;

    waitclock;

    csrwrite(32'h00, 2'b10); // enable rx
    csrwrite(32'h0C, 32'h10000014); // set address for rx slot 3
    csrwrite(32'h09, 32'h00000008); // set address for rx slot 2
    csrwrite(32'h08, 1); // set slot 2 state = ready

    // wait rx irq
    @(posedge irq_rx);
    waitclock;
    waitclock;
    waitclock;
    waitclock;

    csrwrite(32'h00, 2'b10); // enable rx

    // wait rx irq
    @(posedge irq_rx);
    csrwrite(32'h00, 2'b10); // enable rx

    // wait rx irq
    @(posedge irq_rx);

    csrread(32'h00);

    csrread(32'h14);
    csrread(32'h20);
    csrread(32'h2C);
    csrread(32'h38);

    waitclock;
    waitclock;
    waitclock;
    waitclock;

    // test tx
    // disable rx enable tx
    csrwrite(32'h00, 2'b01);
    waitclock;
    waitclock;
    csrwrite(32'd14, 14);
    waitclock;
    waitclock;
    csrwrite(32'd15, 10); // transmit 10 bytes

    @(posedge irq_tx);

    #30;

    $finish;
end

endmodule
