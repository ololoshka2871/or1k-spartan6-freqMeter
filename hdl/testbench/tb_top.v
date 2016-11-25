`include "../timescale.v"

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   11:16:19 04/10/2016
// Design Name:   top
// Module Name:   /home/shiloxyz/src/Xilinx/or32_boot/tb/tb_top.v
// Project Name:  or32_boot
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: top
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

`include "config.v"

module tb_top;

	// Inputs
	reg clk;
        reg rx;
	
	// Outputs
        wire tx;
	
	reg rst = 1'b1;

        wire     flash_CS;
        wire     sck_o;
        wire     mosi_o;

        wire     phy_tx_en;
        reg      rmii_clk;
        wire     mii_mdclk;
        wire     mii_mdio;
        wire  [1:0]   rmii_tx_data;

        wire [23:0] Fin;

        reg [32:0] devided_clocks;

        wire [11:0] test_sig;

        wire [`GPIO_COUNT-1:0]     gpio;

        wire sda;
        wire scl;

        assign test_sig[0] = devided_clocks[3] & devided_clocks[8];
        assign test_sig[1] = devided_clocks[2] & devided_clocks[9];
        assign test_sig[2] = devided_clocks[1] & devided_clocks[0];
        assign test_sig[3] = devided_clocks[8] & devided_clocks[1];
        assign test_sig[4] = devided_clocks[7] & devided_clocks[2];
        assign test_sig[5] = devided_clocks[6] & devided_clocks[3];
        assign test_sig[6] = devided_clocks[5] & devided_clocks[4];
        assign test_sig[7] = devided_clocks[4] & devided_clocks[5];
        assign test_sig[8] = devided_clocks[3] & devided_clocks[6];
        assign test_sig[9] = devided_clocks[2] & devided_clocks[7];
        assign test_sig[10] = devided_clocks[1] & devided_clocks[8];
        assign test_sig[11] = devided_clocks[0] & devided_clocks[9];

        assign Fin[11:0] = test_sig;
        assign Fin[23:12] = ~test_sig;

	// Bidirs

	// Instantiate the Unit Under Test (UUT)
	top uut (
            .clk_i(clk),
            .rx0(rx),
            .tx0(tx),
            .tx1(),
            .rst_i(rst),
            .flash_CS(flash_CS),
            .sck_o(sck_o),
            .mosi_o(mosi_o),
            .miso_i(mosi_o),

            .phy_rmii_rx_data(rmii_tx_data),
            .phy_rmii_crs(phy_tx_en),
            .phy_rmii_tx_data(rmii_tx_data),
            .phy_tx_en(phy_tx_en),
            .phy_rmii_clk(rmii_clk),
            .phy_mdclk(mii_mdclk),
            .phy_mdio(mii_mdio),

            .Fin(Fin[`F_INPUTS_COUNT-1:0]),

            .i2c_sda(sda),
            .i2c_scl(scl),

            .gpio(gpio)
	);

        PULLUP PULLUP_sda (
            .O(sda)
        ), PULLUP_scl (
            .O(scl)
        );

	initial begin
            // Initialize Inputs
            clk = 0;
            rx = 0;
            devided_clocks = 0;
            rmii_clk = 1;
            // Wait 100 ns for global reset to finish
            #100;

            // Add stimulus here

	end
	
	always #10 clk <= !clk;

        always #11 rmii_clk <= !rmii_clk;

        always @(posedge clk) begin
            devided_clocks = devided_clocks + 1;
        end
      
endmodule

