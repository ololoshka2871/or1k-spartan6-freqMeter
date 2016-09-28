//****************************************************************************
//*
//*   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
//*   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
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
//* this module provides access to FPGA boot SPI flash
//*
//****************************************************************************/

module tb_seg7_disp_drv (

    output wire [7:0]                   segments,      // segments drivers
    output wire [DIGITS_COUNT - 1:0]    selectors      // digit selector
);

parameter WB_DATA_WIDTH = 32;
parameter DIGITS_COUNT = 4;

// WISHBONE bus slave interface
reg				clk;
reg                             rst;
reg				cyc;
reg 				stb;
reg  [2:0]                      adr;
reg				we;
reg  [WB_DATA_WIDTH - 1:0]	dat_i;
wire [WB_DATA_WIDTH - 1:0]	dat_o;
wire				ack;

reg                             update_clock;

reg  [2:0]                      counter;

reg                             trigger;

wire [7:0] addr8 = {4'b0, adr, 2'b0};

seg7_disp_drv
#(
    .DIGITS_COUNT(DIGITS_COUNT),
    .IS_COM_CATODE(1),
    .WB_DATA_WIDTH(WB_DATA_WIDTH)
) disp_drv (
    .clk_i(clk),
    .rst_i(rst),
    .cyc_i(cyc),
    .stb_i(stb),
    .adr_i(addr8),
    .we_i(we),
    .dat_i(dat_i),
    .dat_o(dat_o),
    .ack_o(ack),

    .update_clock(update_clock),
    .segments(segments),
    .selectors(selectors)
);

initial begin
        // Initialize Inputs
        clk = 0;

        rst = 1;
        #10;
        rst = 0;

        adr = 0;
        dat_i = 0;
        we = 1;
        stb = 0;
        cyc = 0;
        update_clock = 0;
        counter = 0;
        trigger = 0;

        // Wait 10 ns for global reset to finish
        #10;
end

    always #10 begin
        clk <= !clk;
        counter <= counter + 1;
        if (trigger & clk) begin
            cyc <= 1'b1;
            stb <= 1'b1;
            trigger <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (stb)
            stb <= 1'b0;
    end

    always @(negedge clk) begin
        if (cyc & ack)
            cyc <= 1'b0;
    end

    always #63 begin
        update_clock <= !update_clock;
        dat_i <= counter;
        if (!clk) begin
            adr <= adr + 1;
            trigger <= 1'b1;
        end
    end



endmodule
