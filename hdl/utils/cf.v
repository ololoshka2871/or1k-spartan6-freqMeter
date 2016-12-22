/**
 * Company............: JSC Continuum (2015)
 * File Name..........: cf.v
 * Description........: Common functions for all modules
 * Creation Date......: 22.01.2015
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


/**
 * Division with ceil rounding
 * @param dividend Dividend number
 * @param divisor Divisor number
 * @return ceil(dividend/divisor)
 */
function integer DIVCEIL;
    input [31:0] dividend;
    input [31:0] divisor;
    begin
        DIVCEIL = dividend / divisor;
        if (dividend % divisor > 0)
            DIVCEIL = DIVCEIL + 1;
    end
endfunction


/**
 * Calculate logarithm to the base 2
 * @param arg Number
 * @return log2(arg)
 */
function [31:0] LOG2;
    input [31:0] arg;
    integer i;
    begin
        LOG2 = 1;
        for (i = 0; 2**i <= arg; i = i + 1)
            begin
            LOG2 = i + 1;
            end
    end
endfunction


/**
 * Host to network byte order for short int
 * @param arg Number
 * @return htons(arg) (C/C++)
 */
function [15:0] HTONS;
    input [15:0] arg;
    begin
    HTONS = {arg[7:0], arg[15:8]};
    end
endfunction


/**
 * Host to network byte order for long int
 * @param arg Number
 * @return htonl(arg) (C/C++)
 */
function [31:0] HTONL;
    input [31:0] arg;
    begin
    HTONL = {arg[7:0], arg[15:8], arg[23:16], arg[31:24]};
    end
endfunction


/**
 * Host to network byte order for MAC address
 * @param arg MAC address
 * @return MAC address with network byte order
 */
function [47:0] HTONMAC;
    input [47:0] arg;
    begin
    HTONMAC = {arg[7:0], arg[15:8], arg[23:16], arg[31:24], arg[39:32], arg[47:40]};
    end
endfunction
