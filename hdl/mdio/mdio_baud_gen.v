/**
 * Company............: JSC Continuum (2015)
 * File Name..........: mdio_baud_gen.v
 * Description........: MDIO baudrate generator logic
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


`include "timescale.v"


module mdio_baud_gen #(

    parameter MASTER_CLK_FREQ_HZ = 10_000_000, /* Hz    */
    parameter MDIO_BAUDRATE      = 2_500_000   /* bit/s */

    )(

    // Common interface
    ip_sync_reset,
    ip_master_clk,

    // External MDIO (MDC) interface
    op_mdio_clk,

    // Internal interface
    ip_enable,
    op_baud_strobe_neg,
    op_baud_strobe_pos

    );

    `include "cf.v" // Include common functions file

    localparam _T_BAUD = (DIVCEIL(MASTER_CLK_FREQ_HZ, MDIO_BAUDRATE) + 1) / 2 * 2; // Rounding to the nearest even-more
    localparam T_BAUD  = (_T_BAUD < 4) ? 4 : _T_BAUD;


    // Inputs / Outputs definitions
    input  wire ip_sync_reset;
    input  wire ip_master_clk;

    output wire op_mdio_clk;

    input  wire ip_enable;
    output wire op_baud_strobe_neg;
    output wire op_baud_strobe_pos;


    // Registers definitions
    reg  [31:0] rTimer = 0;
    reg  [31:0] rhTimer = 0;


    // Assign definitions
    assign op_mdio_clk        = (ip_enable) && ((rTimer  < (T_BAUD / 2)) && (rhTimer == T_BAUD / 2));
    assign op_baud_strobe_neg = (ip_enable) && (rTimer == (T_BAUD / 2) - 1);
    assign op_baud_strobe_pos = (ip_enable) && (rTimer == (T_BAUD - 1));

    always @(posedge ip_master_clk)
        if (!ip_enable)
            rhTimer <= 0;
        else if (rhTimer < (T_BAUD / 2))
            rhTimer <= rhTimer + 1;
            

    // MDIO baudrate generator logic
    always @(posedge ip_master_clk)
        if ((rTimer < (T_BAUD - 1)) && ip_enable && (rhTimer == T_BAUD / 2))
            rTimer <= rTimer + 1;
        else
            rTimer <= 0;

endmodule
