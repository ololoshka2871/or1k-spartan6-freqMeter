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
    // input clock
    input               clk_i,

    // UART
    input               rx0,
    output              tx0,

`ifdef UART1_ENABLED_STR
    input               rx1,
    output              rx1,
`endif

    // reset CPU key
    input wire          rst_i,

    inout  wire         flash_CS,      // spi flash CS wire
    output wire         sck_o,         // serial clock output
    output wire         mosi_o,        // MasterOut SlaveIN
    input  wire         miso_i         // MasterIn SlaveOut

    // Ethernet RMII interface
`ifdef ETHERNET_ENABLED
    ,
    input  wire         phy_rmii_clk,   // 50 MHZ input
    output wire         phy_mdclk,      // MDCLK
    inout  wire         phy_mdio,       // MDIO
    input  wire         phy_rmii_crs,   // Ressiver ressiving data
    output wire [1:0]   phy_rmii_tx_data,// transmit data bis
    input  wire [1:0]   phy_rmii_rx_data,// ressive data bus
    output wire         phy_tx_en       // transmitter enable
`endif

`ifdef I2C_PRESENT
    ,
    inout  wire         i2c_sda,        // I2C SDA
    inout  wire         i2c_scl         // I2C SCL
`endif

`ifdef GPIO_PRESENT
    ,
    inout wire [`GPIO_COUNT-1:0]     gpio
`endif

`ifdef USE_PHISICAL_INPUTS
    ,
    input wire [`F_INPUTS_COUNT-1:0] Fin
`endif
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------

parameter CLK_KHZ = `DEVICE_REF_CLOCK_HZ * `CLOCK_CPU_PLL_MULTIPLYER / `CLOCK_CPU_CLOCK_DEVIDER / 1000;

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

wire[31:0]          sf_addr;
wire[31:0]          sf_data_r;
wire[31:0]          sf_data_w;
wire[3:0]           sf_sel;
wire[2:0]           sf_cti;
wire                sf_we;
wire                sf_stb;
wire                sf_cyc;
wire                sf_ack;
wire                sf_stall;

wire                clk;
wire                clk_ref;

wire		    clk_io;
wire[6:0]           spi_cs_o;

wire[2:0]           ext_intr;

wire[`SYSTEM_FREF_COUNTER_LEN-1:0] devided_clocks;
wire[15:0]          clock_devider16 = devided_clocks[15:0];

`ifdef USE_PHISICAL_INPUTS
`else
wire[`F_INPUTS_COUNT-1:0] Fin;
`endif

wire[`F_INPUTS_COUNT-1:0] Fin_inv_pars;

wire                rmii_clk;

//-----------------------------------------------------------------
// Instantiation
//-----------------------------------------------------------------
parameter FPGA_RAM_SIZE_BYTES   = `NUM_OF_SYS_MEM_UNITS * `MEMORY_UNIT_SIZE / 8;
parameter RAM_ADDRESS_LEN	= $clog2(FPGA_RAM_SIZE_BYTES);

//RAM
wb_dp_ram_primitive
#(
    //.LOAD_IMAGE(1),
    .NUM_OF_MEM_UNITS_TO_USE(`NUM_OF_SYS_MEM_UNITS),
    .DATA_WIDTH(32)
)
sys_ram
(
    .rst_i(reset),

    .a_clk(clk),
    .a_adr_i(imem_addr[RAM_ADDRESS_LEN-1:0]),
    .a_dat_i(32'b0),
    .a_dat_o(imem_data),
    .a_we_i(1'b0),
    .a_sel_i(imem_sel),
    .a_stb_i(imem_stb),
    .a_ack_o(imem_ack),
    .a_cyc_i(imem_cyc),
    .a_stall_o(imem_stall),
    
    .b_clk(clk),
    .b_adr_i(dmem_addr[RAM_ADDRESS_LEN-1:0]),
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
    .dmem1_addr_o(sf_addr),
    .dmem1_data_o(sf_data_w),
    .dmem1_data_i(sf_data_r),
    .dmem1_sel_o(sf_sel),
    .dmem1_we_o(sf_we),
    .dmem1_stb_o(sf_stb),
    .dmem1_cyc_o(sf_cyc),
    .dmem1_cti_o(sf_cti),
    .dmem1_stall_i(sf_stall),
    .dmem1_ack_i(sf_ack),
	  
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

// clocking provider
clock_provider clk_prov
(
    .clk_i(clk_i),
    .rmii_clk_to_PHY_i(phy_rmii_clk),

    .sys_clk_o(clk),
    .rmii_logick_clk_o(rmii_clk),
    .clk_ref_o(clk_ref)
);

// FREQMETER and ETHERNET
soc_fast
#(
    .INPUTS_COUNT(`F_INPUTS_COUNT),
    .MASER_FREQ_COUNTER_LEN(`SYSTEM_FREF_COUNTER_LEN),
    .INPUT_FREQ_COUNTER_LEN(`SYSTEM_INPUTS_COUNTER_LEN)
) sf (   
    .clk_i(clk),
    .rst_i(reset),

    .cyc_i(sf_cyc),
    .stb_i(sf_stb),
    .adr_i(sf_addr),
    .we_i(sf_we),
    .dat_i(sf_data_w),
    .dat_o(sf_data_r),
    .ack_o(sf_ack),
    .stall_o(sf_stall),
    .sel_i(sf_sel),
    .cti_i(sf_cti),

    .F_master(clk_ref),
    .F_in(Fin_inv_pars),
    .devided_clocks(devided_clocks),

`ifdef ETHERNET_ENABLED
    .phy_rmii_clk(rmii_clk),
    .phy_rmii_crs(phy_rmii_crs),
    .phy_rmii_tx_data(phy_rmii_tx_data),
    .phy_rmii_rx_data(phy_rmii_rx_data),
    .phy_tx_en(phy_tx_en),
`endif

    .interrupts_o(ext_intr)
);

// CPU SOC
soc
#(
    .CLK_KHZ(CLK_KHZ),
    .ENABLE_SYSTICK_TIMER("ENABLED"),
    .ENABLE_HIGHRES_TIMER("ENABLED"),
    .BAUD_UART0(`BAUD_UART0),
    .BAUD_UART0(`BAUD_UART1),
    .BAUD_MDIO(`BAUD_MDIO),
    .BAUD_I2C(`BAUD_I2C),
    .EXTERNAL_INTERRUPTS(3)
)
u_soc
(
    // General - clocking & reset
    .clk_i(clk),
    .rst_i(reset),
    .ext_intr_i(ext_intr),
    .intr_o(soc_irq),

    .uart0_tx_o(tx0),
    .uart0_rx_i(rx0),

`ifdef UART1_ENABLED_STR
    .uart0_tx_o(tx1),
    .uart0_rx_i(rx1),
`endif

    // Memory Port
    .io_addr_i(soc_addr),    
    .io_data_i(soc_data_w),
    .io_data_o(soc_data_r),    
    .io_we_i(soc_we),
    .io_stb_i(soc_stb),
    .io_ack_o(soc_ack),
    .io_cyc_i(soc_cyc),

    .devided_clocks(clock_devider16),

    .sck_o(sck_o),
    .mosi_o(mosi_o),
    .miso_i(miso_i),
    .spi_cs_o(spi_cs_o)
`ifdef ETHERNET_ENABLED
    ,
    .mdclk_o(phy_mdclk),
    .mdio(phy_mdio)
`endif

`ifdef I2C_PRESENT
    ,
    .i2c_sda(i2c_sda),
    .i2c_scl(i2c_scl)
`endif

`ifdef GPIO_PRESENT
    ,
    .gpio(gpio)
`endif
);

//-----------------------------------------------------------------
// Implementation
//-----------------------------------------------------------------

`ifdef USE_PHISICAL_INPUTS
assign Fin_inv_pars = Fin ^ {16{2'b10}};
`else
// test freqs (24)
wire [11:0] test_sig;

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

assign Fin[11:0] = /*devided_clocks[28:17] &*/ test_sig;
assign Fin[23:12] = /*devided_clocks[28:17] &*/ ~test_sig;

assign Fin_inv_pars = Fin;
`endif

// Reset Generator
always @(posedge clk) 
if (rst_i == 1'b0)
    reset       <= 1'b1;
else
    reset       <= 1'b0;
//else 
//    rst_next    <= 1'b0;

// flash_CS
assign flash_CS = spi_cs_o[0];

//-----------------------------------------------------------------
// Unused pins
//-----------------------------------------------------------------

endmodule
