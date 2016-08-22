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
    input           clk_i,

    // UART
    input           rx0,
    output          tx0,

    output          tx1,

    // leds
    inout wire[`LEDS_COUNT - 1:0] leds_io,

    // reset CPU key
    input wire	    rst_i,

    inout  wire     flash_CS,      // spi flash CS wire
    output wire     sck_o,         // serial clock output
    output wire     mosi_o,        // MasterOut SlaveIN
    input  wire     miso_i,        // MasterIn SlaveOut

    output wire[7:0] segments,
    output wire[3:0] seg_selectors

`ifdef USE_PHISICAL_INPUTS
    ,
    input wire [`F_INPUTS_COUNT-1:0] Fin
`endif
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter OSC_KHZ = `INPUT_CLOCK_HZ;
parameter CLK_KHZ = OSC_KHZ * `PLL_MULTIPLYER / `CPU_CLOCK_DEVIDER / 1000;

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

wire[31:0]          freqmeter_addr;
wire[31:0]          freqmeter_data_r;
wire[31:0]          freqmeter_data_w;
wire                freqmeter_we;
wire                freqmeter_stb;
wire                freqmeter_cyc;
wire                freqmeter_ack;
wire                freqmeter_inta;

wire                pllFB;
wire                clk;
wire                clk_ref;

wire		    clk_io;
wire[6:0]           spi_cs_o;

wire[3:0]           GPIO_oe;
wire[3:0]           GPIO_o;
wire[3:0]           GPIO_i;

wire[`MASER_FREQ_COUNTER_LEN-1:0] devided_clocks;
wire[15:0]          clock_devider16 = devided_clocks[15:0];

`ifdef USE_PHISICAL_INPUTS
`else
wire[`F_INPUTS_COUNT-1:0] Fin;
`endif

wire[`F_INPUTS_COUNT-1:0] Fin_inv_pars;


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
    .dmem1_addr_o(freqmeter_addr),
    .dmem1_data_o(freqmeter_data_w),
    .dmem1_data_i(freqmeter_data_r),
    .dmem1_sel_o(/* open */),
    .dmem1_we_o(freqmeter_we),
    .dmem1_stb_o(freqmeter_stb),
    .dmem1_cyc_o(freqmeter_cyc),
    .dmem1_cti_o(/* open */),
    .dmem1_stall_i(1'b0),
    .dmem1_ack_i(freqmeter_ack),
	  
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

// Freq meter
freqmeters
#(
    .INPUTS_COUNT(`F_INPUTS_COUNT),
    .MASER_FREQ_COUNTER_LEN(`MASER_FREQ_COUNTER_LEN),
    .INPUT_FREQ_COUNTER_LEN(`INPUT_FREQ_COUNTER_LEN)
) fm (
    .clk_i(clk),
    .rst_i(reset),
    .cyc_i(freqmeter_cyc),
    .stb_i(freqmeter_stb),
    .adr_i(freqmeter_addr[8:0]),
    .we_i(freqmeter_we),
    .dat_i(freqmeter_data_w),
    .dat_o(freqmeter_data_r),
    .ack_o(freqmeter_ack),
    .inta_o(freqmeter_inta),

    .F_master(clk_ref),
    .F_in(Fin_inv_pars),

    .devided_clocks(devided_clocks)
);

// CPU SOC
soc
#(
    .CLK_KHZ(CLK_KHZ),
    .ENABLE_SYSTICK_TIMER("ENABLED"),
    .ENABLE_HIGHRES_TIMER("ENABLED"),
    .UART0_BAUD(`UART0_BAUD),
    .UART1_BAUD(`UART1_BAUD),
    .EXTERNAL_INTERRUPTS(1)
)
u_soc
(
    // General - clocking & reset
    .clk_i(clk),
    .rst_i(reset),
    .ext_intr_i(freqmeter_inta),
    .intr_o(soc_irq),

    .uart0_tx_o(tx0),
    .uart0_rx_i(rx0),

    .uart1_tx_o(tx1),
    .uart1_rx_i(1'b1), // no input

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
    .spi_cs_o(spi_cs_o),

    .segments(segments),
    .seg_selectors(seg_selectors),

    .GPIO_oe(GPIO_oe),
    .GPIO_o(GPIO_o),
    .GPIO_i(GPIO_i)
);

// reference clock gen clk_ref = clk_i * `PLL_MULTIPLYER / `REFERENCE_CLOCK_DEVIDER
DCM_CLKGEN #(
   .CLKFXDV_DIVIDE(2),       // CLKFXDV divide value (2, 4, 8, 16, 32)
   .CLKFX_DIVIDE(`REFERENCE_CLOCK_DEVIDER),         // Divide value - D - (1-256)
   .CLKFX_MD_MAX(`PLL_MULTIPLYER * 2 / `REFERENCE_CLOCK_DEVIDER),       // Specify maximum M/D ratio for timing anlysis
   .CLKFX_MULTIPLY(`PLL_MULTIPLYER * 2),       // Multiply value - M - (2-256)
   .CLKIN_PERIOD(`INPUT_CLOCK_PERIOD_NS_F),       // Input clock period specified in nS
   .SPREAD_SPECTRUM("NONE"), // Spread Spectrum mode "NONE", "CENTER_LOW_SPREAD", "CENTER_HIGH_SPREAD",
                             // "VIDEO_LINK_M0", "VIDEO_LINK_M1" or "VIDEO_LINK_M2"
   .STARTUP_WAIT("TRUE")    // Delay config DONE until DCM_CLKGEN LOCKED (TRUE/FALSE)
)
DCM_CLKGEN_f_ref (
   .CLKFX(/* open */),         // 1-bit output: Generated clock output
   .CLKFX180(/* open */),   // 1-bit output: Generated clock output 180 degree out of phase from CLKFX.
   .CLKFXDV(clk_ref),     // 1-bit output: Divided clock output
   .LOCKED(/* open */),       // 1-bit output: Locked output
   .PROGDONE(/* open */),   // 1-bit output: Active high output to indicate the successful re-programming
   .STATUS(/* open */),       // 2-bit output: DCM_CLKGEN status
   .CLKIN(clk_i),         // 1-bit input: Input clock
   .FREEZEDCM(1'b0), // 1-bit input: Prevents frequency adjustments to input clock
   .PROGCLK(1'b0),     // 1-bit input: Clock input for M/D reconfiguration
   .PROGDATA(1'b0),   // 1-bit input: Serial data input for M/D reconfiguration
   .PROGEN(1'b0),       // 1-bit input: Active high program enable
   .RST(1'b0)              // 1-bit input: Reset input pin
);

// CPU clock gen clk = clk_i * `PLL_MULTIPLYER / `CPU_CLOCK_DEVIDER
DCM_CLKGEN #(
   .CLKFXDV_DIVIDE(2),       // CLKFXDV divide value (2, 4, 8, 16, 32)
   .CLKFX_DIVIDE(`CPU_CLOCK_DEVIDER),         // Divide value - D - (1-256)
   .CLKFX_MD_MAX(`PLL_MULTIPLYER * 2 / `CPU_CLOCK_DEVIDER),       // Specify maximum M/D ratio for timing anlysis
   .CLKFX_MULTIPLY(`PLL_MULTIPLYER * 2),       // Multiply value - M - (2-256)
   .CLKIN_PERIOD(`INPUT_CLOCK_PERIOD_NS_F),       // Input clock period specified in nS
   .SPREAD_SPECTRUM("NONE"), // Spread Spectrum mode "NONE", "CENTER_LOW_SPREAD", "CENTER_HIGH_SPREAD",
                             // "VIDEO_LINK_M0", "VIDEO_LINK_M1" or "VIDEO_LINK_M2"
   .STARTUP_WAIT("TRUE")    // Delay config DONE until DCM_CLKGEN LOCKED (TRUE/FALSE)
)
DCM_CLKGEN_f_cpu (
   .CLKFX(/* open */),         // 1-bit output: Generated clock output
   .CLKFX180(/* open */),   // 1-bit output: Generated clock output 180 degree out of phase from CLKFX.
   .CLKFXDV(clk),     // 1-bit output: Divided clock output
   .LOCKED(/* open */),       // 1-bit output: Locked output
   .PROGDONE(/* open */),   // 1-bit output: Active high output to indicate the successful re-programming
   .STATUS(/* open */),       // 2-bit output: DCM_CLKGEN status
   .CLKIN(clk_i),         // 1-bit input: Input clock
   .FREEZEDCM(1'b0), // 1-bit input: Prevents frequency adjustments to input clock
   .PROGCLK(1'b0),     // 1-bit input: Clock input for M/D reconfiguration
   .PROGDATA(1'b0),   // 1-bit input: Serial data input for M/D reconfiguration
   .PROGEN(1'b0),       // 1-bit input: Active high program enable
   .RST(1'b0)              // 1-bit input: Reset input pin
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

// bidirectional GPIO
genvar i;
generate
for (i = 0; i < `LEDS_COUNT; i = i + 1)
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
