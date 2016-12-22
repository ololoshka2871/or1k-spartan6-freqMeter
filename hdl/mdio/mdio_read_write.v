/**
 * Company............: JSC Continuum (2015)
 * File Name..........: mdio_read_write.v
 * Description........: MDIO read/write register service
 * Creation Date......: 06.04.2015
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


`include "timescale.v"


/** MDIO read/write state machine states definitions */
`define MDIO_READ_WRITE_STATE_READY           0
`define MDIO_READ_WRITE_STATE_TX              1
`define MDIO_READ_WRITE_STATE_RX_TURN_AROUND  2
`define MDIO_READ_WRITE_STATE_RX              3
`define MDIO_READ_WRITE_STATE_RX_END          4


module mdio_read_write #(

    parameter MASTER_CLK_FREQ_HZ = 10_000_000, /* Hz    */
    parameter MDIO_BAUDRATE      = 2_500_000   /* bit/s */

    )(

    // Common interface
    ip_sync_reset,
    ip_master_clk,

    // External MDIO interface
    op_mdio_clk,
    io_mdio_data,

    // Internal interface
    ip_rd_wr,  //< 0 - read operation; 1 - write operation
    ip_start,
    ip_phy_addr,
    ip_reg_addr,
    ip_data,
    op_data,
    op_data_ready

    );

    localparam NBIT_SHIFT        = 64;
    localparam MDIO_PREAMBLE     = 32'hFF_FF_FF_FF;
    localparam MDIO_SFD          = 2'b01;
    localparam MDIO_OPCODE_READ  = 2'b10;
    localparam MDIO_OPCODE_WRITE = 2'b01;
    localparam MDIO_TURN_AROUND  = 2'b10;


    // Inputs / Outputs definitions
    input  wire ip_master_clk;
    input  wire ip_sync_reset;

    output wire op_mdio_clk;
    inout  wire io_mdio_data;

    input  wire        ip_rd_wr;
    input  wire        ip_start;
    input  wire [4:0]  ip_phy_addr;
    input  wire [4:0]  ip_reg_addr;
    input  wire [15:0] ip_data;
    output wire [15:0] op_data;
    output wire        op_data_ready;


    // Wires definitions
    wire wMDIODataIn;
    wire wBitSamplingCompleteStrobe;
    wire wBitSamplingCounterStrobe;


    // Registers definitions
    reg  [2:0]            rState          = `MDIO_READ_WRITE_STATE_READY;
    reg  [NBIT_SHIFT-1:0] rMDIOTxShiftReg = 0;
    reg  [15:0]           rMDIORxShiftReg = 0;
    reg                   rMDIODataDir    = 1; // Input
    reg                   rDataReady      = 1'b1;
    reg  [31:0]           rCounter        = 0;

    IOBUF #(
       .DRIVE(12), // Specify the output drive strength
       .IOSTANDARD("DEFAULT"), // Specify the I/O standard
       .SLEW("SLOW") // Specify the output slew rate
    ) IOBUF_inst (
       .O(wMDIODataIn),  // Buffer output
       .IO(io_mdio_data),   // Buffer inout port (connect directly to top-level port)
       .I(rMDIOTxShiftReg[NBIT_SHIFT-1]),     // Buffer input
       .T(rMDIODataDir)     // 3-state enable input, high=input, low=output
    );

    // Assign definitions
    assign op_data       = rMDIORxShiftReg;
    assign op_data_ready = (rDataReady ^ ip_start) & rDataReady;


    // MDIO baudrate generator logic
    mdio_baud_gen #(
        .MASTER_CLK_FREQ_HZ (MASTER_CLK_FREQ_HZ),
        .MDIO_BAUDRATE      (MDIO_BAUDRATE)
    ) mdio_baud_gen_read (
        .ip_sync_reset      (ip_sync_reset),
        .ip_master_clk      (ip_master_clk),

        .op_mdio_clk        (op_mdio_clk),

        .ip_enable          (rState != `MDIO_READ_WRITE_STATE_READY),
        .op_baud_strobe_neg (wBitSamplingCompleteStrobe),
        .op_baud_strobe_pos ()
    );


    // Bit counter logic
    always @(posedge ip_master_clk)
        if (wBitSamplingCompleteStrobe)
            if (rCounter == NBIT_SHIFT - 1)
                rCounter <= 0;
            else
                rCounter <= rCounter + 1;


    // MDIO read logic
    always @(posedge ip_master_clk)
        case (rState)

        `MDIO_READ_WRITE_STATE_READY:
            begin
            if (ip_start)
                begin
                // Prepare shift register
                rMDIOTxShiftReg <= {MDIO_PREAMBLE, MDIO_SFD, ip_rd_wr ? MDIO_OPCODE_WRITE : MDIO_OPCODE_READ, ip_phy_addr, ip_reg_addr, MDIO_TURN_AROUND, ip_data};
                rMDIODataDir <= 1'b0; // Output
                rDataReady <= 1'b0;
                rState <= `MDIO_READ_WRITE_STATE_TX;
                end
            else
                rDataReady <= 1'b1;
            end

        `MDIO_READ_WRITE_STATE_TX:
            begin
            if (wBitSamplingCompleteStrobe)
                begin
                rMDIOTxShiftReg <= {rMDIOTxShiftReg[NBIT_SHIFT-1:0], 1'b0};
                // Read stage
                if ((rCounter == 45) && (!ip_rd_wr))
                    begin
                    rMDIODataDir <= 1'b1; // Input
                    rState <= `MDIO_READ_WRITE_STATE_RX_TURN_AROUND;
                    end
                // Write stage
                else if ((rCounter == (NBIT_SHIFT - 1)) && (ip_rd_wr))
                    begin
                    rMDIODataDir <= 1'b1; // Input
                    rState <= `MDIO_READ_WRITE_STATE_READY;
                    end
                end
             end

         `MDIO_READ_WRITE_STATE_RX_TURN_AROUND:
            begin
            if (wBitSamplingCompleteStrobe)
                begin
                if (rCounter == 46)
                    rState <= `MDIO_READ_WRITE_STATE_RX;
                end
            end

         `MDIO_READ_WRITE_STATE_RX:
            begin
            if (wBitSamplingCompleteStrobe)
                begin
                rMDIORxShiftReg <= {rMDIORxShiftReg[14:0], wMDIODataIn};
                if (rCounter == (NBIT_SHIFT - 2))
                    rState <= `MDIO_READ_WRITE_STATE_RX_END;
                end
            end

        `MDIO_READ_WRITE_STATE_RX_END:
            begin
            if (wBitSamplingCompleteStrobe)
                begin
                if (rCounter == (NBIT_SHIFT - 1))
                    rState <= `MDIO_READ_WRITE_STATE_READY;
                end
            end

        endcase 

endmodule
