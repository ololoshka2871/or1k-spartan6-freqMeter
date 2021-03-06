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

module myminimac_ctlif_cd
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
    output                          csr_ack,            // control logick acknolage
    input                           csr_stb,            // control logick strobe

    input                           rmii_clk_i,         // 50 MHz

    output                          rx_rst,             // reset rx request
    output                          rx_valid,           // rx memory ready to write
    output      [RX_ADDR_WIDTH-1:2] rx_adr,             // base address to write ressived bytes
    input                           rx_resetcount,      // reset rx address
    input                           rx_incrcount,       // if 1 we are increment curent rx slot ressived counter
    input                           rx_endframe,        // if 1 we are set state "10 -> slot has received a packet" for current slot
    input                           rx_error,           // error ocures during reset

    output                          tx_rst,             // reset Tx request
    output                          tx_valid,           // 1 - enable transmission
    output                          tx_last_byte,       // last bate remaining
    output      [TX_ADDR_WIDTH-1:2] tx_adr,             // address of next byte to send
    input                           tx_next             // request to update tx_adr
);

assign csr_ack = 1'b1;

parameter TRANSFER_COUNTER_LEN = $clog2(MTU);

localparam MTU_ALIGNED = (MTU & 2'b11) ? ((MTU & ~32'b11) + 3'b100) : MTU;

localparam RX_SLOT0_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 0;
localparam RX_SLOT1_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 1;
localparam RX_SLOT2_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 2;
localparam RX_SLOT3_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 3;

localparam TX_SLOT_ADDR  = TX_MEMORY_BASE;

reg p_sys_we;
reg [5:2] p_sys_addr;
wire sys_wr = ((~p_sys_we & csr_we) | (p_sys_addr != csr_a[5:2])) & csr_we;
always @(posedge sys_clk) begin
    if (sys_rst) begin
        p_sys_we <= 1'b0;
        p_sys_addr <= 4'd0;
    end else begin
        p_sys_we <= csr_we;
        p_sys_addr <= csr_a[5:2];
    end
end

wire [5:2] reg_selector = csr_a[5:2];

/********************************* RX *****************************************/

/* RX state registers: Wishbone RW, ctl RW */
reg  [1:0] slot_state_write [3:0];

wire [1:0] slot_state_sys_o [3:0];
reg  [3:0] slot_state_sys_act;

wire [1:0] slot_state_ctl_o [3:0];
reg  [3:0] slot_write_ctl_act;
reg  [1:0] slot_state_ctl_i [3:0];

genvar i;

generate
    for (i = 0; i < 4; i = i + 1) begin
        clk_domain_cros_register
        #(
            .DATA_WIDTH(2)
        )  slot_state (
            .reset_i(sys_rst),
            .clk_sys_i(sys_clk),
            .clk_ctl_i(rmii_clk_i),

            .sys_data_i(slot_state_write[i]),
            .sys_write_act_i(slot_state_sys_act[i]),
            .sys_data_o(slot_state_sys_o[i]),

            .D_i(slot_state_ctl_i[i]),
            .D_o(slot_state_ctl_o[i]),
            .we_i(slot_write_ctl_act[i])
        );
    end
endgenerate

/* RX Count registers: Wishbone RW, ctl RW */
wire [TRANSFER_COUNTER_LEN - 1:0] slot_count_sys_o [3:0];
reg  [3:0] slot_count_sys_act;

wire [TRANSFER_COUNTER_LEN - 1:0] slot_count_ctl_o [3:0];
reg  [3:0] slot_count_ctl_act;
reg  [TRANSFER_COUNTER_LEN - 1:0] slot_count_ctl_i [3:0];

generate
    for (i = 0; i < 4; i = i + 1) begin
        clk_domain_cros_register
        #(
            .DATA_WIDTH(TRANSFER_COUNTER_LEN)
        )  slot_counter (
            .reset_i(sys_rst),
            .clk_sys_i(sys_clk),
            .clk_ctl_i(rmii_clk_i),

            .sys_data_i({TRANSFER_COUNTER_LEN{1'b0}}),
            .sys_write_act_i(slot_count_sys_act[i]),
            .sys_data_o(slot_count_sys_o[i]),

            .D_i(slot_count_ctl_i[i]),
            .D_o(slot_count_ctl_o[i]),
            .we_i(slot_count_ctl_act[i])
        );
    end
endgenerate

reg [3:0] active_slot_state;

// detect ready-to-RX slot
// selectX = 1 if addr is set and prevs slots not ready
wire select0 = slot_state_ctl_o[0][0];
wire select1 = slot_state_ctl_o[1][0] & ~slot_state_ctl_o[0][0];
wire select2 = slot_state_ctl_o[2][0] & ~slot_state_ctl_o[1][0] & ~slot_state_ctl_o[0][0];
wire select3 = slot_state_ctl_o[3][0] & ~slot_state_ctl_o[2][0] & ~slot_state_ctl_o[1][0]
    & ~slot_state_ctl_o[0][0];

// rx_valid == 1 if any of rx slots are ready
assign rx_valid = slot_state_ctl_o[0][0] | slot_state_ctl_o[1][0] |
    slot_state_ctl_o[2][0] | slot_state_ctl_o[3][0];

// address of ready slot
assign rx_adr =
        select0 ? RX_SLOT0_ADDR[RX_ADDR_WIDTH-1:2] :
        select1 ? RX_SLOT1_ADDR[RX_ADDR_WIDTH-1:2] :
        select2 ? RX_SLOT2_ADDR[RX_ADDR_WIDTH-1:2] :
        RX_SLOT3_ADDR[RX_ADDR_WIDTH-1:2];


/******************************** TX ******************************************/

/* tx addr register */
assign tx_adr = TX_SLOT_ADDR;

/* tx temaning register; Wishbone RW, ctl: RW */
reg [TRANSFER_COUNTER_LEN-1:0] tx_remaining_sys_i;
reg tx_remaining_sys_wr;
wire [TRANSFER_COUNTER_LEN-1:0] tx_remaining_sys_o;

reg [TRANSFER_COUNTER_LEN-1:0] tx_remaining_ctl_i;
wire [TRANSFER_COUNTER_LEN-1:0] tx_remaining_ctl_o;
reg tx_remaining_ctl_wr;
clk_domain_cros_register
#(
    .DATA_WIDTH(TRANSFER_COUNTER_LEN)
)  tx_remaining (
    .reset_i(sys_rst),
    .clk_sys_i(sys_clk),
    .clk_ctl_i(rmii_clk_i),

    .sys_data_i(tx_remaining_sys_i),
    .sys_write_act_i(tx_remaining_sys_wr),
    .sys_data_o(tx_remaining_sys_o),

    .D_i(tx_remaining_ctl_i),
    .D_o(tx_remaining_ctl_o),
    .we_i(tx_remaining_ctl_wr)
);

assign tx_last_byte = (tx_remaining_ctl_i == 1);
assign tx_valid = |tx_remaining_ctl_o;

/* RX/TX Reset: Wishbone RW, ctl RW */
reg [1:0] rst_ctl_sys_i;
reg rst_ctl_sys_wr;
wire [1:0] rst_ctl_sys_o;

reg [1:0] rst_ctl_ctl_i;
wire [1:0] rst_ctl_ctl_o;
reg rst_ctl_ctl_wr;
clk_domain_cros_register
#(
    .DATA_WIDTH(2),
    .INITIAL_VALUE(2'b11)
)  reset_ctrl (
    .reset_i(sys_rst),
    .clk_sys_i(sys_clk),
    .clk_ctl_i(rmii_clk_i),

    .sys_data_i(rst_ctl_sys_i),
    .sys_write_act_i(rst_ctl_sys_wr),
    .sys_data_o(rst_ctl_sys_o),

    .D_i(rst_ctl_ctl_i),
    .D_o(rst_ctl_ctl_o),
    .we_i(rst_ctl_ctl_wr)
);

assign rx_rst = rst_ctl_ctl_o[0];
assign tx_rst = rst_ctl_ctl_o[1];

//------------------------------------------------------------------------------

integer j;

always @(posedge rmii_clk_i) begin
    if (sys_rst) begin
        slot_write_ctl_act <= 4'd0;
        for (j = 0; j < 4; j = j + 1) begin
            slot_state_ctl_i[j] <= 2'b00;
            slot_count_ctl_i[j] <= 0;
        end
        slot_count_ctl_act <= 0;
        tx_remaining_ctl_i <= 0;
        tx_remaining_ctl_wr <= 0;
        rst_ctl_ctl_i <= 2'b0;
        rst_ctl_ctl_wr <= 1'b0;
        active_slot_state <= 4'b0;
    end else begin
        for (j = 0; j < 4; j = j + 1) begin
            slot_state_ctl_i[j] <= 2'b00;
            slot_count_ctl_i[j] <= 0;
        end
        slot_write_ctl_act <= 4'd0;
        slot_count_ctl_act <= 0;
        tx_remaining_ctl_wr <= 1'b0;
        rst_ctl_ctl_wr <= 1'b0;

        if (rx_error) begin
            rst_ctl_ctl_i <= rst_ctl_ctl_o | 2'b01;
            rst_ctl_ctl_wr <= 1'b1;
        end


        if(rx_resetcount) begin
            active_slot_state <= {select3, select2, select1, select0};
            case(1'b1)
                select0: begin
                    slot_count_ctl_i[0] <= 0;
                    slot_count_ctl_act[0] <= 1'b1;
                    end
                select1: begin
                    slot_count_ctl_i[1] <= 0;
                    slot_count_ctl_act[1] <= 1'b1;
                    end
                select2: begin
                    slot_count_ctl_i[2] <= 0;
                    slot_count_ctl_act[2] <= 1'b1;
                    end
                select3: begin
                    slot_count_ctl_i[3] <= 0;
                    slot_count_ctl_act[3] <= 1'b1;
                    end
            endcase
        end

        if(rx_incrcount) begin
            case(1'b1)
                active_slot_state[0]: begin
                    slot_count_ctl_i[0] <= slot_count_ctl_o[0] + 1;
                    slot_count_ctl_act[0] <= 1'b1;
                    end
                active_slot_state[1]: begin
                    slot_count_ctl_i[1] <= slot_count_ctl_o[1] + 1;
                    slot_count_ctl_act[1] <= 1'b1;
                    end
                active_slot_state[2]: begin
                    slot_count_ctl_i[2] <= slot_count_ctl_o[2] + 1;
                    slot_count_ctl_act[2] <= 1'b1;
                    end
                active_slot_state[3]: begin
                    slot_count_ctl_i[3] <= slot_count_ctl_o[3] + 1;
                    slot_count_ctl_act[3] <= 1'b1;
                    end
            endcase
        end

        if(rx_endframe) begin
            case(1'b1)
                active_slot_state[0]: begin
                    slot_state_ctl_i[0] <= 2'b10;
                    slot_write_ctl_act[0] <= 1'b1;
                    end
                active_slot_state[1]: begin
                    slot_state_ctl_i[1] <= 2'b10;
                    slot_write_ctl_act[1] <= 1'b1;
                    end
                active_slot_state[2]: begin
                    slot_state_ctl_i[2] <= 2'b10;
                    slot_write_ctl_act[2] <= 1'b1;
                    end
                active_slot_state[3]: begin
                    slot_state_ctl_i[3] <= 2'b10;
                    slot_write_ctl_act[3] <= 1'b1;
                    end
            endcase
        end

        if(tx_next) begin
            tx_remaining_ctl_i <= tx_remaining_ctl_o - 1;
            tx_remaining_ctl_wr <= 1'b1;
        end
    end
end

integer k;

always @(posedge sys_clk) begin
    if(sys_rst) begin
        csr_do <= 32'd0;

        for (k = 0; k < 4; k = k + 1) begin
            slot_state_write[k] <= 2'b00;
        end

        slot_state_sys_act <= 4'b0;
        slot_count_sys_act <= 4'b0;

        tx_remaining_sys_wr <= 1'b0;

        rst_ctl_sys_wr <= 1'b0;
    end else begin
        csr_do <= 32'd0;

        rst_ctl_sys_wr <= 1'b0;
        slot_state_sys_act <= 4'b0;
        slot_count_sys_act <= 4'b0;
        tx_remaining_sys_wr <= 1'b0;

        if(csr_we) begin
            // Write
            case(reg_selector)
                4'd0 : begin // reset
                    rst_ctl_sys_i <= csr_di[1:0];
                    rst_ctl_sys_wr <= sys_wr;
                end

                // RX slots
                4'd2 : begin
                    slot_state_write[0] <= csr_di[1:0];
                    slot_state_sys_act[0] <= sys_wr;
                end
                // slot0_count is read-only
                4'd5 : begin
                    slot_state_write[1] <= csr_di[1:0];
                    slot_state_sys_act[1] <= sys_wr;
                end
                // slot1_count is read-only
                4'd8 : begin
                    slot_state_write[2] <= csr_di[1:0];
                    slot_state_sys_act[2] <= sys_wr;
                end
                // slot2_count is read-only
                4'd11: begin
                    slot_state_write[3] <= csr_di[1:0];
                    slot_state_sys_act[3] <= sys_wr;
                end
                // slot3_count is read-only

                // TX slot
                4'd15: begin
                    tx_remaining_sys_i <= csr_di[TRANSFER_COUNTER_LEN-1:0];
                    tx_remaining_sys_wr <= sys_wr;
                end
            endcase
        end

        // Read
        case(reg_selector)
            4'd0 : csr_do <= rst_ctl_sys_o;

            4'd1 : csr_do <= 0;

            // RX slots
            4'd2 : csr_do <= slot_state_sys_o[0];
            4'd4 : csr_do <= slot_count_sys_o[0];
            4'd5 : csr_do <= slot_state_sys_o[1];
            4'd7 : csr_do <= slot_count_sys_o[1];
            4'd8 : csr_do <= slot_state_sys_o[2];
            4'd10: csr_do <= slot_count_sys_o[2];
            4'd11: csr_do <= slot_state_sys_o[3];
            4'd13: csr_do <= slot_count_sys_o[3];

            // TX slot
            4'd15: csr_do <= tx_remaining_sys_o;

            default:
                   csr_do <= 32'd0;
        endcase
    end
end

////////////////////////////////////////////////////////////////////////////////

wire [4:0]  irq_rx_status = {slot_state_ctl_o[0][1], slot_state_ctl_o[1][1],
                    slot_state_ctl_o[2][1], slot_state_ctl_o[3][1], rx_rst};
wire        irq_tx_status = ~tx_valid;

reg [4:0]   irq_rx_status_p;
reg         irq_tx_status_p;

always @(posedge sys_clk) begin
    if(sys_rst) begin
        irq_tx_status_p <= 1'b0;
        irq_tx <= 1'b0;

        irq_rx_status_p <= 1'b0;
        irq_rx <= 1'b0;
    end else begin
        irq_tx <= ~irq_tx_status_p & irq_tx_status;
        irq_tx_status_p <= irq_tx_status;

        irq_rx <= |((irq_rx_status_p ^ irq_rx_status) & irq_rx_status);
        irq_rx_status_p <= irq_rx_status;
    end
end

endmodule
