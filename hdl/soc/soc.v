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
//-----------------------------------------------------------------

`include "config.v"

//-----------------------------------------------------------------
// Module:
//-----------------------------------------------------------------
module soc
(
    // General - Clocking & Reset
    clk_i,
    rst_i,

    ext_intr_i,
    intr_o,

    // UART0
    uart0_tx_o,
    uart0_rx_i,

    // Memory interface
    io_addr_i,
    io_data_i,
    io_data_o,
    io_we_i,
    io_stb_i,    
    io_ack_o,
    io_cyc_i,

    devided_clocks,

    // SPI
    sck_o,
    mosi_o,
    miso_i,
    spi_cs_o

    // MDIO
`ifdef ETHERNET_ENABLED
    ,
    mdio,
    mdclk_o
`endif

`ifdef I2C_PRESENT
    ,
    i2c_sda,
    i2c_scl
`endif

`ifdef GPIO_PRESENT
    ,
    gpio
`endif

);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter  [31:0]   CLK_KHZ              = 12288;
parameter  [31:0]   EXTERNAL_INTERRUPTS  = 1;
parameter           BAUD_UART0           = 115200;
parameter           BAUD_MDIO            = 2500000;
parameter           BAUD_I2C             = 100000;
parameter           SYSTICK_INTR_MS      = 1;
parameter           ENABLE_SYSTICK_TIMER = "ENABLED";
parameter           ENABLE_HIGHRES_TIMER = "ENABLED";

//-----------------------------------------------------------------
// I/O
//-----------------------------------------------------------------
input                   clk_i /*verilator public*/;
input                   rst_i /*verilator public*/;
input [(EXTERNAL_INTERRUPTS - 1):0]  ext_intr_i /*verilator public*/;
output                  intr_o /*verilator public*/;
output                  uart0_tx_o /*verilator public*/;
input                   uart0_rx_i /*verilator public*/;
// Memory Port
input [31:0]            io_addr_i /*verilator public*/;
input [31:0]            io_data_i /*verilator public*/;
output [31:0]           io_data_o /*verilator public*/;
input                   io_we_i /*verilator public*/;
input                   io_stb_i /*verilator public*/;
output                  io_ack_o /*verilator public*/;
input                   io_cyc_i /*verilator public*/;
// devided_clocks
input  [15:0]           devided_clocks /*verilator public*/;
// SPI
output                  sck_o /*verilator public*/;
output                  mosi_o /*verilator public*/;
input                   miso_i /*verilator public*/;
output [6:0]            spi_cs_o /*verilator public*/;
//MDIO
`ifdef ETHERNET_ENABLED
inout                   mdio /*verilator public*/;
output                  mdclk_o /*verilator public*/;
`endif

`ifdef I2C_PRESENT
inout                   i2c_sda /*verilator public*/;
inout                   i2c_scl /*verilator public*/;
`endif

`ifdef GPIO_PRESENT
inout [`GPIO_COUNT-1:0] gpio /*verilator public*/;
`endif

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
wire [7:0]         uart0_addr;
wire [31:0]        uart0_data_w;
wire [31:0]        uart0_data_r;
wire               uart0_we;
wire               uart0_stb;
wire               uart0_intr;

wire [31:0]        hw_math_data_w;
wire [31:0]        hw_math_data_r;
wire               hw_math_we;
wire               hw_math_stb;
wire [7:0]         hw_math_addr;

wire [7:0]         timer_addr;
wire [31:0]        timer_data_o;
wire [31:0]        timer_data_i;
wire               timer_we;
wire               timer_stb;
wire               timer_intr_systick;
wire               timer_intr_hires;

wire [7:0]         intr_addr;
wire [31:0]        intr_data_o;
wire [31:0]        intr_data_i;
wire               intr_we;
wire               intr_stb;

wire [7:0]         spi_addr;
wire [31:0]        spi_data_o;
wire [31:0]        spi_data_i;
wire               spi_we;
wire               spi_stb;
wire               spi_intr;

wire [7:0]         mdio_addr;
wire               mdio_stb;
wire               mdio_we;
wire [31:0]        mdio_data_w;
wire [31:0]        mdio_data_r;
wire               mdio_intr;

wire [7:0]         i2c_addr;
wire               i2c_stb;
wire               i2c_we;
wire [31:0]        i2c_data_w;
wire [31:0]        i2c_data_r;
wire               i2c_intr;

wire [7:0]         gpio_addr;
wire               gpio_stb;
wire               gpio_we;
wire [31:0]        gpio_dat_w;
wire [31:0]        gpio_dat_r;
wire               gpio_intr;

//-----------------------------------------------------------------
// Peripheral Interconnect
//-----------------------------------------------------------------
soc_pif8
u2_soc
(
    // General - Clocking & Reset
    .clk_i(clk_i),
    .rst_i(rst_i),

    // I/O bus (from mem_mux)
    // 0x12000000 - 0x12FFFFFF
    .io_addr_i(io_addr_i),
    .io_data_i(io_data_i),
    .io_data_o(io_data_o),
    .io_we_i(io_we_i),
    .io_stb_i(io_stb_i),
    .io_ack_o(io_ack_o),

    // Peripherals
    // UART0 = 0x12000000 - 0x120000FF
    .periph0_addr_o(uart0_addr),
    .periph0_data_o(uart0_data_w),
    .periph0_data_i(uart0_data_r),
    .periph0_we_o(uart0_we),
    .periph0_stb_o(uart0_stb),

    // Timer = 0x12000100 - 0x120001FF
    .periph1_addr_o(timer_addr),
    .periph1_data_o(timer_data_o),
    .periph1_data_i(timer_data_i),
    .periph1_we_o(timer_we),
    .periph1_stb_o(timer_stb),

    // Interrupt Controller = 0x12000200 - 0x120002FF
    .periph2_addr_o(intr_addr),
    .periph2_data_o(intr_data_o),
    .periph2_data_i(intr_data_i),
    .periph2_we_o(intr_we),
    .periph2_stb_o(intr_stb),

    // SPI = 0x12000300 - 0x120003FF
    .periph3_addr_o(spi_addr),
    .periph3_data_o(spi_data_o),
    .periph3_data_i(spi_data_i),
    .periph3_we_o(spi_we),
    .periph3_stb_o(spi_stb),

    // GPIO = 0x12000400 - 0x120004FF
    .periph4_addr_o(gpio_addr),
    .periph4_data_o(gpio_dat_w),
    .periph4_data_i(gpio_dat_r),
    .periph4_we_o(gpio_we),
    .periph4_stb_o(gpio_stb),

    // MDIO = 0x12000500 - 0x120005FF
    .periph5_addr_o(mdio_addr),
    .periph5_data_o(mdio_data_w),
    .periph5_data_i(mdio_data_r),
    .periph5_we_o(mdio_we),
    .periph5_stb_o(mdio_stb),

    // i2c = 0x12000600 - 0x120006FF
    .periph6_addr_o(i2c_addr),
    .periph6_data_o(i2c_data_w),
    .periph6_data_i(i2c_data_r),
    .periph6_we_o(i2c_we),
    .periph6_stb_o(i2c_stb),

    // CRC32 = 0x12000700 - 0x120007FF
    .periph7_addr_o(hw_math_addr),
    .periph7_data_o(hw_math_data_w),
    .periph7_data_i(hw_math_data_r),
    .periph7_we_o(hw_math_we),
    .periph7_stb_o(hw_math_stb)
);

//-----------------------------------------------------------------
// UART0
//-----------------------------------------------------------------
`ifdef UART0_ENABLED
uart_periph
#(
    .UART_DIVISOR(((CLK_KHZ * 1000) / BAUD_UART0))
)
u_uart0
(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .intr_o(uart0_intr),
    .addr_i(uart0_addr),
    .data_o(uart0_data_r),
    .data_i(uart0_data_w),
    .we_i(uart0_we),
    .stb_i(uart0_stb),
    .rx_i(uart0_rx_i),
    .tx_o(uart0_tx_o)
);
`else
assign uart0_intr = 1'b0;
assign uart0_data_r = 4'h000000;
assign uart0_tx_o = 1'b1;
`endif

//-----------------------------------------------------------------
// HW MATH
//-----------------------------------------------------------------
hw_math hw_math_periph
(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .addr_i(hw_math_addr),
    .dat_o(hw_math_data_r),
    .dat_i(hw_math_data_w),
    .we_i(hw_math_we),
    .stb_i(hw_math_stb)
);

//-----------------------------------------------------------------
// Timer
//-----------------------------------------------------------------
`ifdef TIMER_ENABLED
timer_periph
#(
    .CLK_KHZ(CLK_KHZ),
    .SYSTICK_INTR_MS(SYSTICK_INTR_MS),
    .ENABLE_SYSTICK_TIMER(ENABLE_SYSTICK_TIMER),
    .ENABLE_HIGHRES_TIMER(ENABLE_HIGHRES_TIMER)
)
u_timer
(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .intr_systick_o(timer_intr_systick),
    .intr_hires_o(timer_intr_hires),
    .addr_i(timer_addr),
    .data_o(timer_data_i),
    .data_i(timer_data_o),
    .we_i(timer_we),
    .stb_i(timer_stb)
);
`else
assign timer_intr_systick = 1'b0;
assign timer_intr_hires = 1'b0;
assign timer_data_i = 4'h000000;
`endif

//-----------------------------------------------------------------
// Interrupt Controller
//-----------------------------------------------------------------
intr_periph
#(
    .EXTERNAL_INTERRUPTS(EXTERNAL_INTERRUPTS)
)
u_intr
(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .intr_o(intr_o),

    .intr0_i(uart0_intr),
    .intr1_i(timer_intr_systick),
    .intr2_i(timer_intr_hires),
    .intr3_i(spi_intr),
    .intr4_i(1'b0),
    .intr5_i(mdio_intr),
    .intr6_i(i2c_intr),
    .intr7_i(1'b0),

    .intr_ext_i(ext_intr_i),

    .addr_i(intr_addr),
    .data_o(intr_data_i),
    .data_i(intr_data_o),
    .we_i(intr_we),
    .stb_i(intr_stb)
);

//-----------------------------------------------------------------
// SPI Controller
//-----------------------------------------------------------------
spi_boot
#(
    .WB_DATA_WIDTH(32),
    .SPI_CLK_DEVIDER(`BAUD_SPI_CLK_DEVIDER_LEN)
) spi (
    .clk_i(clk_i),
    .rst_i(rst_i),
    .cyc_i(io_cyc_i),
    .stb_i(spi_stb),
    .adr_i(spi_addr),
    .we_i(spi_we),
    .dat_i(spi_data_o),
    .dat_o(spi_data_i),
    .ack_o(/*open*/),
    .inta_o(spi_intr),

    .sck_o(sck_o),
    .mosi_o(mosi_o),
    .miso_i(miso_i),

    .cs_o(spi_cs_o)
);

//-----------------------------------------------------------------
// MDIO Controller
//-----------------------------------------------------------------
`ifdef ETHERNET_ENABLED
wb_mdio
#(
    .MASTER_CLK_FREQ_HZ(CLK_KHZ * 1000),
    .MDIO_BAUDRATE(BAUD_MDIO)
) mdio_ip (
    .clk_i(clk_i),
    .rst_i(rst_i),
    .cyc_i(io_cyc_i),
    .stb_i(mdio_stb),
    .adr_i(mdio_addr[2:0]),
    .we_i(mdio_we),
    .dat_i(mdio_data_w),
    .dat_o(mdio_data_r),

    .inta_o(mdio_intr),

    .mdio(mdio),
    .mdclk_o(mdclk_o)
);
`else
assign mdio_data_r = 32'b0;
assign mdio_intr = 1'b0;
`endif

//-----------------------------------------------------------------
// I2C Controller
//-----------------------------------------------------------------
`ifdef I2C_ENABLED

wire i2c_sda_ctl;
wire i2c_scl_ctl;

iicmb_m_wb
#(
    .g_bus_num(1),
    .g_f_clk($itor(CLK_KHZ)),
    .g_f_scl_0($itor(BAUD_I2C/1000))
) i2c_ip (
    .clk_i(clk_i),
    .rst_i(rst_i),

    .cyc_i(io_cyc_i),
    .stb_i(i2c_stb),
    .ack_o(/* open */),
    .adr_i(i2c_addr[3:2]),
    .we_i(i2c_we),
    .dat_i(i2c_data_w[7:0]),
    .dat_o(i2c_data_r[7:0]),

    .irq(i2c_intr),

    .scl_i(i2c_scl),
    .sda_i(i2c_sda),
    .scl_o(i2c_scl_ctl),
    .sda_o(i2c_sda_ctl)
);

assign i2c_sda = !i2c_sda_ctl ? 1'b0 : 1'bz;
assign i2c_scl = !i2c_scl_ctl ? 1'b0 : 1'bz;

assign i2c_data_r[31:8] = 24'h0;

`else
assign i2c_data_r = 32'b0;
assign i2c_intr = 1'b0;
`endif

//-----------------------------------------------------------------
// GPIO Controller
//-----------------------------------------------------------------
`ifdef GPIO_ENABLED

wire [`GPIO_COUNT-1:0] gpio_oe;
wire [`GPIO_COUNT-1:0] gpio_o;
wire [`GPIO_COUNT-1:0] gpio_i;

gpio_top
#(
    .dw(32),
    .gw(`GPIO_COUNT)
) gpio_ip (
    .wb_clk_i(clk_i),
    .wb_rst_i(rst_i),

    .wb_cyc_i(io_cyc_i),
    .wb_adr_i(gpio_addr),
    .wb_dat_i(gpio_dat_w),
    .wb_sel_i(4'b1111),
    .wb_we_i(gpio_we),
    .wb_stb_i(gpio_stb),
    .wb_dat_o(gpio_dat_r),
    .wb_ack_o( /* open */ ),
    .wb_err_o( /* open */ ),
    .wb_inta_o(gpio_intr),

    .ext_pad_i(gpio_i),
    .ext_pad_o(gpio_o),
    .ext_padoe_o(gpio_oe)
);

genvar gpio_index;

generate
    for(gpio_index = 0; gpio_index < `GPIO_COUNT; gpio_index = gpio_index + 1) begin : generate_GPIOS
`ifdef I2C_ENABLED
    // i2c independent module, so if gpio exists create it
    `ifdef GPIO_PRESENT
        IOBUF
        #(
            .DRIVE(12), // Specify the output drive strength
            .IOSTANDARD("DEFAULT"), // Specify the I/O standard
            .SLEW("SLOW") // Specify the output slew rate
        ) IOBUF_inst (
            .O(gpio_i[gpio_index]),   // Buffer output
            .IO(gpio[gpio_index]),    // Buffer inout port (connect directly to top-level port)
            .I(gpio_o[gpio_index]),   // Buffer input
            .T(~gpio_oe[gpio_index])  // 3-state enable input, high=input, low=output
        );
    `endif
`else
    `ifdef I2C_PRESENT // use gpio pins as i2c
        if ((gpio_index == `I2C_OVER_GPIO_SDA_PIN) || (gpio_index == `I2C_OVER_GPIO_SCL_PIN)) begin
            if (gpio_index == `I2C_OVER_GPIO_SDA_PIN) begin
                assign gpio_i[gpio_index] = i2c_sda;
                assign i2c_sda = gpio_oe[gpio_index] ? 1'b0 : 1'bz;
            end else begin
                assign gpio_i[gpio_index] = i2c_scl;
                assign i2c_scl = gpio_oe[gpio_index] ? 1'b0 : 1'bz;
            end
        end else
    `endif
        begin
            IOBUF
            #(
                .DRIVE(12), // Specify the output drive strength
                .IOSTANDARD("DEFAULT"), // Specify the I/O standard
                .SLEW("SLOW") // Specify the output slew rate
            ) IOBUF_inst (
                .O(gpio_i[gpio_index]),   // Buffer output
                .IO(gpio[gpio_index]),   // Buffer inout port (connect directly to top-level port)
                .I(gpio_o[gpio_index]),   // Buffer input
                .T(~gpio_oe[gpio_index])  // 3-state enable input, high=input, low=output
            );
        end
`endif
    end
endgenerate

`else
assign gpio_data_r = 32'b0;
assign gpio_intr = 1'b0;
`endif

//-------------------------------------------------------------------
// Hooks for debug
//-------------------------------------------------------------------
`ifdef verilator
   function [0:0] get_uart_wr;
      // verilator public
      get_uart_wr = uart0_stb & uart0_we;
   endfunction
   
   function [7:0] get_uart_data;
      // verilator public
      get_uart_data = uart0_data_w[7:0];
   endfunction
`endif

endmodule
