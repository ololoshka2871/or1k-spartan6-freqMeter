/**
 * Company............: JSC Continuum (2015)
 * File Name..........: tb_mdio_read.v
 * Description........: Testbench for MDIO read register service
 * Creation Date......: 07.04.2015
 * Target Devices.....: N/A
 * Tool Versions......: ISE 14.3 (NT)
 * Dependencies.......: N/A
 *
 * Author(s):
 * Alexander Chizhov (interminatus.utopia@gmail.com)
 *
 * Additional Comments:
 * None for now
 */


`include "../timescale.v"


module tb_mdio_read;

    // Inputs
    reg  ip_sync_reset;
    reg  ip_master_clk;
    reg  ip_rd_wr;
    reg  ip_start;
    reg  [4:0] ip_phy_addr;
    reg  [4:0] ip_reg_addr;
    reg  [15:0] ip_data;

    // Outputs
    wire op_mdio_clk;
    wire [15:0] op_data;
    wire op_data_ready;

    // Bidirs
    wire io_mdio_data;
    reg  ip_mdio_data;


    // Parameters
    localparam MASTER_CLK_FREQ_HZ = 25_000_000;
    localparam MDIO_BAUDRATE      = 2_500_000;


    integer flg_transmit = 0;


    // Task for simulate ip_start signal (strobe)
    task start_task (
        );
        integer flg_done;

        begin
        flg_done = 0;

        while (flg_done == 0)
            begin
            @(posedge ip_master_clk)
                if (op_data_ready)
                   begin
                   ip_start <= 1'b1;
                   flg_done = 1;
                   end
            end
        @(posedge ip_master_clk)
            ip_start <= 1'b0;
        end
    endtask


    // Task for simulate read data
    task mdio_read_task (
        input [15:0] data
        );
        integer i;

        begin
        // Wait for transmit
        ip_mdio_data <= 0;
        for (i = 0; i < 47; i = i + 1)
            @(posedge op_mdio_clk);

        flg_transmit = 1;

        // Transmit data
        for (i = 0; i < 16; i = i + 1)
            @(posedge op_mdio_clk)
                ip_mdio_data <= data[15 - i];
        @(posedge op_mdio_clk);

        flg_transmit = 0;
        end
    endtask


    // Bidirectional assign for io_mdio_data line
    assign io_mdio_data = (flg_transmit) ? ip_mdio_data : 1'bz;


	// Instantiate the Unit Under Test (UUT)
	mdio_read_write #(
        .MASTER_CLK_FREQ_HZ (MASTER_CLK_FREQ_HZ),
        .MDIO_BAUDRATE      (MDIO_BAUDRATE)
    ) uut (
		.ip_sync_reset      (ip_sync_reset),
		.ip_master_clk      (ip_master_clk),

		.op_mdio_clk        (op_mdio_clk),
		.io_mdio_data       (io_mdio_data),

		.ip_start           (ip_start),

        .ip_rd_wr           (ip_rd_wr),

		.ip_phy_addr        (ip_phy_addr),
		.ip_reg_addr        (ip_reg_addr),
        .ip_data            (ip_data),
		.op_data            (op_data),
		.op_data_ready      (op_data_ready)
	);


	initial begin
        // Initialize Inputs
        ip_sync_reset = 0;
        ip_master_clk = 0;
        ip_mdio_data = 0;
        ip_rd_wr = 0; // Read operation
        ip_start = 0;
        ip_phy_addr = 5'h11;
        ip_reg_addr = 5'h0F;
        ip_data = 16'h00_00;

        // Wait 100 ns for global reset to finish
        #100;

        start_task();
        mdio_read_task(16'hAA_BB);
		  
		  #20
		  $finish;
	end

    always #20 ip_master_clk <= !ip_master_clk;

endmodule
