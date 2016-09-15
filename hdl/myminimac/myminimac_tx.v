//****************************************************************************
//*
//*   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
//*   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
//*   Based on: Milkymist VJ SoC 2007, 2008, 2009, 2010 Sebastien Bourdeauducq
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
//****************************************************************************/

`include "config.v"

module minimac_tx
#(
    parameter MTU                   = 1530,
    parameter SLOTS_COUNT           = 1,                // memory to alocate
    parameter MEM_UNITS_TO_ALLOC    = $rtoi($ceil(MTU * $itor(SLOTS_COUNT) / (`MEMORY_UNIT_SIZE / 8))),
    parameter ADDR_LEN              = $clog2(MEM_UNITS_TO_ALLOC * `MEMORY_UNIT_SIZE)
) (
    input                           sys_clk,            // System clock
    input                           sys_rst,            // System reset

    input  wire [31:0]              tx_mem_adr_i,       // ADR_I() address
    input  wire [31:0]              tx_mem_dat_i,       // DAT_I() data in
    output wire [31:0]              tx_mem_dat_o,       // DAT_O() data out
    input  wire                     tx_mem_we_i,        // WE_I write enable input
    input  wire [3:0]               tx_mem_sel_i,       // SEL_I() select input
    input  wire                     tx_mem_stb_i,       // STB_I strobe input
    output wire                     tx_mem_ack_o,       // ACK_O acknowledge output
    input  wire                     tx_mem_cyc_i,       // CYC_I cycle input
    output wire                     tx_mem_stall_o,     // incorrect address

    input                           tx_rst,             // reset Tx request
    input                           tx_valid,           // address is valid
    input       [ADDR_LEN-1:0]      tx_adr,             // addres to read from
    output reg                      tx_next,            // request to increment address

    input                           phy_rmii_clk,       // 50 MHz
    output reg                      phy_tx_en,          // transmit enable
    output reg  [1:0]               phy_rmii_tx_data    // data to transmit
);

parameter MEMORY_DATA_WIDTH = 32;
parameter RMII_BUS_WIDTH = 2;
parameter COUNTER_WIDTH = $clog2(MEMORY_DATA_WIDTH / RMII_BUS_WIDTH);

reg [MEMORY_DATA_WIDTH - RMII_BUS_WIDTH - 1:0] transmitt_data;  // shift register to transmitt
reg [COUNTER_WIDTH-1:0] transmitt_counter;

wire [31:0] data_from_memory;
wire memory_error;

wb_dma_ram
#(
    .NUM_OF_MEM_UNITS_TO_USE(MEM_UNITS_TO_ALLOC)
) tx_ram (
    .wb_clk(sys_clk),
    .wb_adr_i(tx_mem_adr_i),
    .wb_dat_i(tx_mem_dat_i),
    .wb_dat_o(tx_mem_dat_o),
    .wb_we_i(tx_mem_we_i),
    .wb_sel_i(tx_mem_sel_i),
    .wb_stb_i(tx_mem_stb_i),
    .wb_ack_o(tx_mem_ack_o),
    .wb_cyc_i(tx_mem_cyc_i),
    .wb_stall_o(tx_mem_stall_o),

    .rawp_clk(phy_rmii_clk),
    .rawp_adr_i(tx_adr),
    .rawp_dat_i(32'd0),
    .rawp_dat_o(data_from_memory),
    .rawp_we_i(1'b0),
    .rawp_stall_o(memory_error)
);

always @(posedge phy_rmii_clk) begin
    if (sys_rst | tx_rst) begin
        transmitt_counter <= 0;
        phy_tx_en <= 0;
    end else begin
        //{phy_rmii_tx_data} <= data_from_memory;
    end
end

`ifdef __UNUSED

reg bus_stb;
assign wbtx_cyc_o = bus_stb;
assign wbtx_stb_o = bus_stb;

assign wbtx_adr_o = {tx_adr, 2'd0};

reg stb;
reg [7:0] data;
wire full;
reg can_tx;
wire empty;

minimac_txfifo txfifo(
	.sys_clk(sys_clk),
	.tx_rst(tx_rst),

	.stb(stb),
	.data(data),
	.full(full),
	.can_tx(can_tx),
	.empty(empty),

	.phy_tx_clk(phy_tx_clk),
	.phy_tx_en(phy_tx_en),
	.phy_tx_data(phy_tx_data)
);

reg load_input;
reg [31:0] input_reg;

always @(posedge sys_clk)
	if(load_input)
		input_reg <= wbtx_dat_i;

always @(*) begin
	case(tx_bytecount)
		2'd0: data = input_reg[31:24];
		2'd1: data = input_reg[23:16];
		2'd2: data = input_reg[16: 8];
		2'd3: data = input_reg[ 7: 0];
	endcase
end

wire firstbyte = tx_bytecount == 2'd0;

reg purge;

/* fetch FSM */

reg [1:0] state;
reg [1:0] next_state;

parameter IDLE  = 2'd0;
parameter FETCH = 2'd1;
parameter WRITE1 = 2'd2;

always @(posedge sys_clk) begin
	if(sys_rst)
		state <= IDLE;
	else
		state <= next_state;
end

always @(*) begin
	next_state = state;
	
	load_input = 1'b0;
	tx_next = 1'b0;

	stb = 1'b0;

	bus_stb = 1'b0;

	case(state)
		IDLE: begin
			if(tx_valid & ~full & ~purge) begin
				if(firstbyte)
					next_state = FETCH;
				else begin
					stb = 1'b1;
					tx_next = 1'b1;
				end
			end
		end
		FETCH: begin
			bus_stb = 1'b1;
			load_input = 1'b1;
			if(wbtx_ack_i)
				next_state = WRITE1;
		end
		WRITE1: begin
			stb = 1'b1;
			tx_next = 1'b1;
			next_state = IDLE;
		end
	endcase
end

/* Byte counter */
reg reset_byte_counter;
reg [6:0] byte_counter;

always @(posedge sys_clk) begin
	if(sys_rst)
		byte_counter <= 7'd0;
	else begin
		if(reset_byte_counter)
			byte_counter <= 7'd0;
		else if(stb)
			byte_counter <= byte_counter + 7'd1;
	end
end

wire tx_level_reached = byte_counter[6];

/* FIFO control FSM */

reg [1:0] fstate;
reg [1:0] next_fstate;

parameter FIDLE		= 2'd0;
parameter FWAITFULL	= 2'd1;
parameter FTX		= 2'd2;
parameter FPURGE	= 2'd3;

always @(posedge sys_clk) begin
	if(sys_rst)
		fstate <= FIDLE;
	else
		fstate <= next_fstate;
end

always @(*) begin
	next_fstate = fstate;

	can_tx = 1'b0;
	purge = 1'b0;
	reset_byte_counter = 1'b1;
	
	case(fstate)
		FIDLE: begin
			if(tx_valid)
				next_fstate = FWAITFULL;
		end
		/* Wait for the FIFO to fill to 64 bytes (< ethernet minimum of 72)
		 * before starting transmission. */
		FWAITFULL: begin
			reset_byte_counter = 1'b0;
			if(tx_level_reached)
				next_fstate = FTX;
		end
		FTX: begin
			can_tx = 1'b1;
			if(~tx_valid) begin
				purge = 1'b1;
				next_fstate = FPURGE;
			end
		end
		FPURGE: begin
			can_tx = 1'b1;
			purge = 1'b1;
			if(empty)
				next_fstate = FIDLE;
			/* NB! there is a potential bug because of the latency
			 * introducted by the synchronizer on can_tx in txfifo.
			 * However, the interframe gap prevents it to happen
			 * unless f(sys_clk) >> f(phy_tx_clk).
			 */
		end
	endcase
end
`endif

endmodule

