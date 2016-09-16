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

module myminimac_ctlif
#(
    parameter RX_MEMORY_BASE      = 32'h00000000,
    parameter TX_MEMORY_BASE      = 32'h10000000,
    parameter MTU                 = 1530,
    parameter RX_ADDR_WIDTH       = $clog2(4 * MTU),
    parameter TX_ADDR_WIDTH       = $clog2(1 * MTU)
) (
    input                           sys_clk,            // SYS clock
    input                           sys_rst,            // SYS reset

    output reg                      irq_rx,             // RX interrupt
    output reg                      irq_tx,             // TX interrupt

    input       [5:0]               csr_a,              // control logic addr
    input                           csr_we,             // control logick write enable
    input       [31:0]              csr_di,             // control logick data input
    output reg  [31:0]              csr_do,             // control logick data output

    // connected to reg
    output reg                      phy_mii_clk,        // MDCLK
    inout                           phy_mii_data,       // MDIO

    input                           rmii_clk_i,         // 50 MHz

    output reg                      rx_rst,             // reset rx request
    output                          rx_valid,           // rx memory ready to write
    output      [RX_ADDR_WIDTH-1:0] rx_adr,             // base address to write ressived bytes
    input                           rx_resetcount,      // reset rx address
    input                           rx_incrcount,       // if 1 we are increment curent rx slot ressived counter
    input                           rx_endframe,        // if 1 we are set state "10 -> slot has received a packet" for current slot
    input                           rx_error,           // error ocures during reset

    output reg                      tx_rst,             // reset Tx request
    output                          tx_valid,           // 1 - enable transmission
    output reg  [TX_ADDR_WIDTH-1:0] tx_adr,             // address of next byte to send
    input                           tx_next             // request to update tx_adr
);

parameter TRANSFER_COUNTER_LEN = $clog2(MTU);

reg mii_data_oe;
reg mii_data_do;

wire phy_mii_data_i;

IOBUF #(
   .DRIVE(12), // Specify the output drive strength
   .IOSTANDARD("DEFAULT"), // Specify the I/O standard
   .SLEW("SLOW") // Specify the output slew rate
) IOBUF_inst (
   .O(phy_mii_data_i),  // Buffer output
   .IO(phy_mii_data),   // Buffer inout port (connect directly to top-level port)
   .I(mii_data_do),     // Buffer input
   .T(~mii_data_oe)     // 3-state enable input, high=input, low=output
);

/* Be paranoid about metastability */
reg mii_data_di1;
reg mii_data_di;
always @(posedge sys_clk) begin
    mii_data_di1 <= phy_mii_data_i;
    mii_data_di <= mii_data_di1;
end

/*
 * RX Slots
 *
 * State:
 * 00 -> slot is not in use
 * 01 -> slot has been loaded with a buffer
 * 10 -> slot has received a packet
 * 11 -> invalid
 */
reg [1:0]                           slot0_state;
reg [RX_ADDR_WIDTH - 1:0]           slot0_adr;
reg [TRANSFER_COUNTER_LEN - 1:0]    slot0_count;
reg [1:0]                           slot1_state;
reg [RX_ADDR_WIDTH - 1:0]           slot1_adr;
reg [TRANSFER_COUNTER_LEN - 1:0]    slot1_count;
reg [1:0]                           slot2_state;
reg [RX_ADDR_WIDTH - 1:0]           slot2_adr;
reg [TRANSFER_COUNTER_LEN - 1:0]    slot2_count;
reg [1:0]                           slot3_state;
reg [RX_ADDR_WIDTH - 1:0]           slot3_adr;
reg [TRANSFER_COUNTER_LEN - 1:0]    slot3_count;

// detect ready-to-RX slot
// selectX = 1 if addr is set and prevs slots not ready
wire select0 = slot0_state[0];
wire select1 = slot1_state[0] & ~slot0_state[0];
wire select2 = slot2_state[0] & ~slot1_state[0] & ~slot0_state[0];
wire select3 = slot3_state[0] & ~slot2_state[0] & ~slot1_state[0] & ~slot0_state[0];

// rx_valid == 1 if any of rx slots are ready
assign rx_valid = slot0_state[0] | slot1_state[0] | slot2_state[0] | slot3_state[0];

// address of ready slot      
assign rx_adr =  select0 ? slot0_adr :
                 select1 ? slot1_adr :
                 select2 ? slot2_adr :
                 slot3_adr;

// TX
reg [TX_ADDR_WIDTH - 1:0] tx_remaining;
assign tx_valid = |tx_remaining;

// INCORRECT!!!
always @(posedge rmii_clk_i) begin
    if (rx_error) begin
        rx_rst <= 1'b1;
    end

    if(rx_resetcount) begin
        case(1'b1)
            select0: slot0_count <= 0;
            select1: slot1_count <= 0;
            select2: slot2_count <= 0;
            select3: slot3_count <= 0;
        endcase
    end

    if(rx_incrcount) begin
        case(1'b1)
            select0: slot0_count <= slot0_count + 1;
            select1: slot1_count <= slot1_count + 1;
            select2: slot2_count <= slot2_count + 1;
            select3: slot3_count <= slot3_count + 1;
        endcase
    end
/* 2 source of data for slotx_state and tx_adr and tx_remaining
    if(rx_endframe) begin
        case(1'b1)
            select0: slot0_state <= 2'b10;
            select1: slot1_state <= 2'b10;
            select2: slot2_state <= 2'b10;
            select3: slot3_state <= 2'b10;
        endcase
    end

    if(tx_next) begin
        tx_remaining <= tx_remaining - 1;
        tx_adr <= tx_adr + 1;
    end
    */
end

always @(posedge sys_clk) begin
    if(sys_rst) begin
        csr_do <= 32'd0;

        mii_data_oe <= 1'b0;
        mii_data_do <= 1'b0;
        phy_mii_clk <= 1'b0;

        slot0_state <= 2'b00;
        slot0_adr <= 0;
        slot0_count <= 0;
        slot1_state <= 2'b00;
        slot1_adr <= 0;
        slot1_count <= 0;
        slot2_state <= 2'b00;
        slot2_adr <= 0;
        slot2_count <= 0;
        slot3_state <= 2'b00;
        slot3_adr <= 0;
        slot3_count <= 0;

        tx_remaining <= 0;
        tx_adr <= 0;

        // star in reset until enable by software
        tx_rst <= 1'b1;
        rx_rst <= 1'b1;
    end else begin
        csr_do <= 32'd0;
        if(csr_we) begin
            case(csr_a[5:2])
                4'd0 : begin
                    tx_rst <= csr_di[1];
                    rx_rst <= csr_di[0];
                end

                4'd1 : begin // bitbang MDIO (set)
                    phy_mii_clk <= csr_di[3];
                    mii_data_oe <= csr_di[2];
                    mii_data_do <= csr_di[0];
                end

                // RX slots
                4'd2 : begin
                    slot0_state <= csr_di[1:0];
                    slot0_count <= 11'd0;
                end
                4'd3 : slot0_adr <= csr_di[RX_ADDR_WIDTH-1:0];
                // slot0_count is read-only
                4'd5 : begin
                    slot1_state <= csr_di[1:0];
                    slot1_count <= 11'd0;
                end
                4'd6 : slot1_adr <= csr_di[RX_ADDR_WIDTH-1:0];
                // slot1_count is read-only
                4'd8 : begin
                    slot2_state <= csr_di[1:0];
                    slot2_count <= 11'd0;
                end
                4'd9 : slot2_adr <= csr_di[RX_ADDR_WIDTH-1:0];
                // slot2_count is read-only
                4'd11: begin
                    slot3_state <= csr_di[1:0];
                    slot3_count <= 11'd0;
                end
                4'd12: slot3_adr <= csr_di[RX_ADDR_WIDTH-1:0];
                // slot3_count is read-only

                // TX slot
                4'd14: tx_adr <= csr_di[TX_ADDR_WIDTH-1:0];
                4'd15: begin
                    tx_remaining <= csr_di[10:0];
                end
            endcase
        end
        case(csr_a[3:0])
            4'd0 : csr_do <= {tx_rst, rx_rst};

            // bitbang MDIO read
            4'd1 : csr_do <= {phy_mii_clk, mii_data_oe, mii_data_di, mii_data_do};

            // RX slots
            4'd2 : csr_do <= slot0_state;
            4'd3 : csr_do <= {RX_MEMORY_BASE[31:RX_ADDR_WIDTH], slot0_adr};
            4'd4 : csr_do <= slot0_count;
            4'd5 : csr_do <= slot1_state;
            4'd6 : csr_do <= {RX_MEMORY_BASE[31:RX_ADDR_WIDTH], slot1_adr};
            4'd7 : csr_do <= slot1_count;
            4'd8 : csr_do <= slot2_state;
            4'd9 : csr_do <= {RX_MEMORY_BASE[31:RX_ADDR_WIDTH], slot1_adr};
            4'd10: csr_do <= slot2_count;
            4'd11: csr_do <= slot3_state;
            4'd12: csr_do <= {RX_MEMORY_BASE[31:RX_ADDR_WIDTH], slot1_adr};
            4'd13: csr_do <= slot3_count;

            // TX slots
            4'd14: csr_do <= {RX_MEMORY_BASE[31:TX_ADDR_WIDTH], tx_adr};
            4'd15: csr_do <= tx_remaining;

            default:
                   csr_do <= 32'd0;
        endcase
    end
end

/* Interrupt logic */

reg tx_valid_r; // prev of tx_valid

always @(posedge sys_clk) begin
    if(sys_rst) begin
        irq_rx <= 1'b0;
        tx_valid_r <= 1'b0;
        irq_tx <= 1'b0;
    end else begin
        // rx interrupt if any slot are ressived data
        irq_rx <= slot0_state[1] | slot1_state[1] | slot2_state[1] | slot3_state[1] | rx_rst;
        tx_valid_r <= tx_valid;
        irq_tx <= tx_valid_r & ~tx_valid; // tx interrupt if tx_valid 1 -> 0
    end
end

endmodule
