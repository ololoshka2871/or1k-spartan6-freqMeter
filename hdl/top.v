//-----------------------------------------------------------------
//                           AltOR32 
//                Alternative Lightweight OpenRisc 
//                            V2.0
//                     Ultra-Embedded.com
//                   Copyright 2011 - 2013
//
//               Email: admin@ultra-embedded.com
//
//                       License: LGPL
//-----------------------------------------------------------------
//
// Copyright (C) 2011 - 2013 Ultra-Embedded.com
//
// This source file may be used and distributed without         
// restriction provided that this copyright statement is not    
// removed from the file and that any derivative work contains  
// the original copyright notice and the associated disclaimer. 
//
// This source file is free software; you can redistribute it   
// and/or modify it under the terms of the GNU Lesser General   
// Public License as published by the Free Software Foundation; 
// either version 2.1 of the License, or (at your option) any   
// later version.
//
// This source is distributed in the hope that it will be       
// useful, but WITHOUT ANY WARRANTY; without even the implied   
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      
// PURPOSE.  See the GNU Lesser General Public License for more 
// details.
//
// You should have received a copy of the GNU Lesser General    
// Public License along with this source; if not, write to the 
// Free Software Foundation, Inc., 59 Temple Place, Suite 330, 
// Boston, MA  02111-1307  USA
// 
// Modified by Shilo_XyZ_ or32-boot for spartan-6 lx9
//-----------------------------------------------------------------

`include "timescale.v"
`include "config.v"

//-----------------------------------------------------------------
// TOP
//-----------------------------------------------------------------
module top
(
    // 48MHz clock
    input           clk,

    // UART
    input           rx,
    output          tx,
	
    // leds
    inout wire[3:0] leds_io,

    // reset CPU key
    input wire	    rst_i,

    inout  wire     flash_CS,      // spi flash CS wire
    output wire     sck_o,         // serial clock output
    output wire     mosi_o,        // MasterOut SlaveIN
    input  wire     miso_i,        // MasterIn SlaveOut

    output wire[7:0] segments,
    output wire[3:0] seg_selectors
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter       OSC_KHZ             = `INPUT_CLOCK_MHZ * 1000;
parameter       CLK_KHZ             = OSC_KHZ; // for timer
parameter       UART_BAUD           = 115200;

//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
// Reset
reg                 reset           = 1'b1;

wire [31:0]         soc_addr;
wire [31:0]         soc_data_w;
wire [31:0]         soc_data_r;
wire                soc_we;
wire                soc_stb;
wire                soc_ack;
wire		    soc_cyc;

wire[31:0]          dmem_addr;
wire[31:0]          dmem_data_w;
wire[31:0]          dmem_data_r;
wire[3:0]           dmem_sel;
wire                dmem_we;
wire                dmem_stb;
wire                dmem_cyc;
wire                dmem_ack;
wire                dmem_stall;

wire[31:0]          imem_addr;
wire[31:0]          imem_data;
wire[3:0]           imem_sel;
wire                imem_stb;
wire                imem_cyc;
wire                imem_ack;
wire                imem_stall;

wire		    clk_io;
wire[6:0]           spi_cs_o;

wire[3:0]           GPIO_oe;
wire[3:0]           GPIO_o;
wire[3:0]           GPIO_i;

//-----------------------------------------------------------------
// Instantiation
//-----------------------------------------------------------------
parameter FPGA_RAM_SIZE		= (`NUM_OF_16k_MEM * 16 * 1024) / 8;
parameter RAM_ADDRESS_LEN	= $clog2(FPGA_RAM_SIZE);

//RAM
wb_dp_ram
#(
    .NUM_OF_16k_TO_USE(`NUM_OF_16k_MEM),
    .DATA_WIDTH(32),
    .ADDR_WIDTH(RAM_ADDRESS_LEN)
)
ram
(
    .a_clk(clk),
    .a_adr_i(imem_addr),
    .a_dat_i(32'b0),
    .a_dat_o(imem_data),
    .a_we_i(1'b0),
    .a_sel_i(imem_sel),
    .a_stb_i(imem_stb),
    .a_ack_o(imem_ack),
    .a_cyc_i(imem_cyc),
    .a_stall_o(imem_stall),
    
    .b_clk(clk),
    .b_adr_i(dmem_addr),
    .b_dat_i(dmem_data_w),
    .b_dat_o(dmem_data_r),
    .b_we_i(dmem_we),
    .b_sel_i(dmem_sel),
    .b_stb_i(dmem_stb),
    .b_ack_o(dmem_ack),
    .b_cyc_i(dmem_cyc),
    .b_stall_o(dmem_stall)
);

// CPU
cpu_if
#(
    .CLK_KHZ(CLK_KHZ),
    .BOOT_VECTOR(32'h10000000),
    .ISR_VECTOR(32'h10000000),
    .ENABLE_ICACHE("DISABLED"),
    .ENABLE_DCACHE("DISABLED"),
    .REGISTER_FILE_TYPE("XILINX")
)
u_cpu
(
    // General - clocking & reset
    .clk_i(clk),
    .rst_i(reset),
    .fault_o(),
    .break_o(),
    .nmi_i(1'b0),
    .intr_i(soc_irq),

    // Instruction Memory 0 (0x10000000 - 0x10FFFFFF)
    .imem0_addr_o(imem_addr),
    .imem0_data_i(imem_data),
    .imem0_sel_o(imem_sel),
    .imem0_cti_o(/* open */),
    .imem0_cyc_o(imem_cyc),
    .imem0_stb_o(imem_stb),
    .imem0_stall_i(imem_stall),
    .imem0_ack_i(imem_ack),
    
    // Data Memory 0 (0x10000000 - 0x10FFFFFF)
    .dmem0_addr_o(dmem_addr),
    .dmem0_data_o(dmem_data_w),
    .dmem0_data_i(dmem_data_r),
    .dmem0_sel_o(dmem_sel),
    .dmem0_cti_o(/* open */),
    .dmem0_cyc_o(dmem_cyc),
    .dmem0_we_o(dmem_we),
    .dmem0_stb_o(dmem_stb),
    .dmem0_stall_i(dmem_stall),
    .dmem0_ack_i(dmem_ack),

    // Data Memory 1 (0x11000000 - 0x11FFFFFF)
    .dmem1_addr_o(),
    .dmem1_data_o(),
    .dmem1_data_i(),
    .dmem1_sel_o(),
    .dmem1_we_o(),
    .dmem1_stb_o(),
    .dmem1_cyc_o(),
    .dmem1_cti_o(),
    .dmem1_stall_i(),
    .dmem1_ack_i(),
	  
    // Data Memory 2 (0x12000000 - 0x12FFFFFF)
    .dmem2_addr_o(soc_addr),
    .dmem2_data_o(soc_data_w),
    .dmem2_data_i(soc_data_r),
    .dmem2_sel_o(/*open*/),
    .dmem2_we_o(soc_we),
    .dmem2_stb_o(soc_stb),
    .dmem2_cyc_o(soc_cyc),
    .dmem2_cti_o(/*open*/),
    .dmem2_stall_i(1'b0),
    .dmem2_ack_i(soc_ack)
);

// CPU SOC
soc
#(
    .CLK_KHZ(CLK_KHZ),
    .ENABLE_SYSTICK_TIMER("ENABLED"),
    .ENABLE_HIGHRES_TIMER("ENABLED"),
    .UART_BAUD(UART_BAUD),
    .EXTERNAL_INTERRUPTS(1)
)
u_soc
(
    // General - clocking & reset
    .clk_i(clk),
    .rst_i(reset),
    .ext_intr_i(1'b0),
    .intr_o(soc_irq),

    .uart_tx_o(tx),
    .uart_rx_i(rx),

    // Memory Port
    .io_addr_i(soc_addr),    
    .io_data_i(soc_data_w),
    .io_data_o(soc_data_r),    
    .io_we_i(soc_we),
    .io_stb_i(soc_stb),
    .io_ack_o(soc_ack),
    .io_cyc_i(soc_cyc),

    .sck_o(sck_o),
    .mosi_o(mosi_o),
    .miso_i(miso_i),
    .spi_cs_o(spi_cs_o),

    .segments(segments),
    .seg_selectors(seg_selectors),

    .GPIO_oe(GPIO_oe),
    .GPIO_o(GPIO_o),
    .GPIO_i(GPIO_i)
);

//-----------------------------------------------------------------
// Implementation
//-----------------------------------------------------------------

// Reset Generator
always @(posedge clk) 
if (rst_i == 1'b0)
    reset       <= 1'b1;
else
    reset       <= 1'b0;
//else 
//    rst_next    <= 1'b0;

// bidirectional GPIO
genvar i;
generate
for (i = 0; i < 4; i = i + 1)
begin : iobuf_gen
    IOBUF
    #(
	.DRIVE(12), // Specify the output drive strength
	.IOSTANDARD("DEFAULT"), // Specify the I/O standard
	.SLEW("SLOW") // Specify the output slew rate
    )
    IOBUF_inst
    (
	.O(GPIO_i[i]),     // Buffer output
	.IO(leds_io[i]),   // Buffer inout port (connect directly to top-level port)
	.I(GPIO_o[i]),     // Buffer input
	.T(~GPIO_oe[i])    // 3-state enable input, high=input, low=output
    );
end
endgenerate

// flash_CS
assign flash_CS = spi_cs_o[0];

//-----------------------------------------------------------------
// Unused pins
//-----------------------------------------------------------------

endmodule
