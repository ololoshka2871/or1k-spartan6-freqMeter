`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   17:45:35 08/05/2016
// Design Name:   freqmeters
// Module Name:   /home/shiloxyz/src/Xilinx/tests/test_freq_meter/tb_freqmeters.v
// Project Name:  test_freq_meter
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: freqmeters
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_freqmeters;

	// Inputs
	reg clk_i;
	reg rst_i;
	reg cyc_i;
	reg stb_i;
	reg [10:0] adr_i;
	reg we_i;
	reg [31:0] dat_i;
	reg F_master;
	reg [23:0] F_in;

	// Outputs
	wire [31:0] dat_o;
	wire ack_o;
	wire inta_o;
	wire [29:0] devided_clocks;

	// Instantiate the Unit Under Test (UUT)
	freqmeters uut (
		.clk_i(clk_i), 
		.rst_i(rst_i), 
		.cyc_i(cyc_i), 
		.stb_i(stb_i), 
		.adr_i(adr_i), 
		.we_i(we_i), 
		.dat_i(dat_i), 
		.dat_o(dat_o), 
		.ack_o(ack_o), 
		.inta_o(inta_o), 
		.F_master(F_master), 
		.F_in(F_in), 
		.devided_clocks(devided_clocks)
	);

	initial begin
		// Initialize Inputs
		clk_i = 0;
		rst_i = 1;
		cyc_i = 0;
		stb_i = 0;
		adr_i = 0;
		we_i = 0;
		dat_i = 0;
		F_master = 0;
		F_in = 0;

		// Wait 10 ns for global reset to finish
		#10;
		
		rst_i = 0;
	end
	
	always #10 begin
		clk_i <= ~clk_i;
	end
	
	always #1 begin
		F_master <= ~F_master;
	end
      
endmodule

