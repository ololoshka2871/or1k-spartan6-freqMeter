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
        reg [8:0] adr_i;
	reg we_i;
	reg [31:0] dat_i;
	reg F_master;

	// Outputs
	wire [31:0] dat_o;
	wire ack_o;
	wire inta_o;
	wire [29:0] devided_clocks;
        wire [23:0] F_in;

        reg [31:0] c;

	// Instantiate the Unit Under Test (UUT)
        freqmeters #(
            .INPUTS_COUNT(24)
        ) uut (
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

        assign F_in[11:0] = c[14:3];
        assign F_in[23:12] = ~c[14:3];

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
            c = 0;

            // Wait 10 ns for global reset to finish
            #10;

            rst_i = 0;

            cyc_i = 1;

            #55 // enable interrapts from all chanels
            adr_i = 4'h11000000;
            dat_i = 24'b111111111111111111111111;
            stb_i = 1;
            we_i = 1;

            #40 // start chanel 0 by writing 2 to counter
            adr_i = 4'h11000000 | (1 << 7);
            dat_i = 4'h00000002;
            stb_i = 1;
            we_i = 1;

            #40 // start chanel 1 by writing 1 to counter
            adr_i = 4'h11000004 | (1 << 7);
            dat_i = 4'h00000001;
            stb_i = 1;
            we_i = 1;

            #200 // start chanel 12 by writing 2 to counter
            adr_i = 4'h11000030 | (1 << 7);
            dat_i = 4'h00000002;
            stb_i = 1;
            we_i = 1;

            #1000 // start chanel 0 by writing 3 to counter
            adr_i = 4'h11000000 | (1 << 7);
            dat_i = 4'h00000003;
            stb_i = 1;
            we_i = 1;

            #40  // start chanel 1 by writing 1 to counter
            adr_i = 4'h11000004 | (1 << 7);
            dat_i = 4'h00000000;
            stb_i = 1;
            we_i = 1;

            #340  // start chanel 1 by writing 1 to counter
            adr_i = 4'h11000004 | (1 << 7);
            dat_i = 4'h00000001;
            stb_i = 1;
            we_i = 1;

            #1500 // read test
            adr_i = 4'h11000000 | (1 << 8);
            stb_i = 1;

            #40
            adr_i = 4'h11000000 | (1 << 8) | (1 << 7);
            stb_i = 1;
	end
	
	always #10 begin
            clk_i <= ~clk_i;
            if (clk_i) begin
                c <= c + 1;
                if (we_i)
                    we_i <= 0;

                if (stb_i)
                    stb_i <= 0;

            end
	end
	
        always #5 begin
            F_master <= ~F_master;
	end
      
endmodule

