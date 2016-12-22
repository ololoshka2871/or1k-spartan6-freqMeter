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

module myminimac_ctlif_cd2
#(
    parameter RX_MEMORY_BASE        = 32'h00000000,
    parameter TX_MEMORY_BASE        = 32'h10000000,
    parameter MTU                   = 1530,
    parameter RX_ADDR_WIDTH         = $clog2(4 * MTU),
    parameter TX_ADDR_WIDTH         = $clog2(1 * MTU),
    parameter TRANSFER_COUNTER_LEN  = $clog2(MTU)
) (
    input                               sys_clk,            // SYS clock
    input                               sys_rst,            // SYS reset

    output reg                          irq_rx,             // RX interrupt
    output reg                          irq_tx,             // TX interrupt

    input       [5:0]                   csr_a,              // control logic addr
    input                               csr_we,             // control logick write enable
    input       [31:0]                  csr_di,             // control logick data input
    output wire [31:0]                  csr_do,             // control logick data output
    output reg                          csr_ack,            // control logick acknolage
    input                               csr_stb,            // control logick strobe
    input                               csr_cyc,            // control logick select

    input                               rmii_clk_i,         // 50 MHz

    output                              rx_rst,             // reset rx request
    output                              rx_valid,           // rx memory ready to write
    output      [RX_ADDR_WIDTH-1:2]     rx_adr,             // base address to write ressived bytes
    input                               rx_resetcount,      // reset rx address
    input                               rx_incrcount,       // if 1 we are increment curent rx slot ressived counter
    input                               rx_endframe,        // if 1 we are set state "10 -> slot has received a packet" for current slot
    input                               rx_error,           // error ocures during reset

    output                              tx_rst,             // reset Tx request
    output                              tx_valid,           // 1 - enable transmission
    output                              tx_last_byte,       // last bate remaining
    output      [TX_ADDR_WIDTH-1:2]     tx_adr,             // address of next byte to send
    input                               tx_next             // request to update tx_adr
);

localparam MTU_ALIGNED = (MTU & 2'b11) ? ((MTU & ~32'b11) + 3'b100) : MTU;

localparam RX_SLOT0_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 0;
localparam RX_SLOT1_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 1;
localparam RX_SLOT2_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 2;
localparam RX_SLOT3_ADDR = RX_MEMORY_BASE + MTU_ALIGNED * 3;

localparam TX_SLOT_ADDR  = TX_MEMORY_BASE;

//////////////////////////////// REGS /////////////////////////////////////////

localparam REG_SLOT0_STATE  = 4'd0;
localparam REG_SLOT1_STATE  = 4'd1;
localparam REG_SLOT2_STATE  = 4'd2;
localparam REG_SLOT3_STATE  = 4'd3;

localparam REG_SLOT0_COUNT  = 4'd4;
localparam REG_SLOT1_COUNT  = 4'd5;
localparam REG_SLOT2_COUNT  = 4'd6;
localparam REG_SLOT3_COUNT  = 4'd7;

localparam REG_TX_REMANING  = 4'd14;

localparam REG_RESET_CTL    = 4'd15;

//////////////////////////////// COMMON ///////////////////////////////////////

wire [3:0] reg_selector = csr_a[5:2];

wire       wr_transaction = csr_we & csr_stb;

reg  [TRANSFER_COUNTER_LEN-1:0] _csr_do;
assign csr_do = {{(32-TRANSFER_COUNTER_LEN){1'b0}}, _csr_do};

//////////////////////////////// RESET ///////////////////////////////////////

reg  [1:0] rst_ctl;
wire [1:0] rst_ctl_sys;
wire tx_enabled = !rst_ctl[1];
wire rst_ctl_selected = (reg_selector == REG_RESET_CTL) & wr_transaction;
wire rst_ctl_wr_done_sys;

wire rst_ctl_wr_req;

TaskAck_CrossDomain rst_ctl_wr_syncronyser (
    .clkA(sys_clk),
    .TaskStart_clkA(rst_ctl_selected),
    .TaskBusy_clkA(),
    .TaskDone_clkA(rst_ctl_wr_done_sys),

    .clkB(rmii_clk_i),
    .TaskStart_clkB(rst_ctl_wr_req),
    .TaskBusy_clkB(),
    .TaskDone_clkB(rst_ctl_wr_req)
);

Signal_n_CrossDomain
#(
    .CHANELS(2)
) rst_ctl_read_syncronyser (
    .clkA(rmii_clk_i),
    .SignalIn_clkA(rst_ctl),
    .clkB(sys_clk),
    .SignalOut_clkB(rst_ctl_sys)
);

always @(posedge rmii_clk_i) begin
    if (sys_rst) begin
        rst_ctl <= 2'b11;
    end else begin
        if (rx_error)
            rst_ctl[0] <= 1'b1;
        if (rst_ctl_wr_req) begin
            rst_ctl <= csr_di[1:0];
        end
    end
end

assign rx_rst = rst_ctl[0];
assign tx_rst = rst_ctl[1];

////////////////////////////////// RX /////////////////////////////////////////

localparam RX_SLOT_STATE_DISABLED       = 2'b00;
localparam RX_SLOT_STATE_READY          = 2'b01;
localparam RX_SLOT_STATE_DATA_RESSIVED  = 2'b10;
localparam RX_SLOT_STATE_INVALID        = 2'b11;

reg  [TRANSFER_COUNTER_LEN - 1:0]   slot_count_ctl [3:0];
reg  [1:0]                          slot_state_ctl [3:0];

reg  [3:0]                          active_slot;

wire [1:0]                          slot_state_sys [3:0];
wire [TRANSFER_COUNTER_LEN - 1:0]   slot_count_sys [3:0];

// rx_valid == 1 if any of rx slots are ready
assign rx_valid = slot_state_ctl[0][0] | slot_state_ctl[1][0] | slot_state_ctl[2][0] | slot_state_ctl[3][0];

// detect ready-to-RX slot
// selectX = 1 if addr is set and prevs slots not ready
wire select0 = slot_state_ctl[0][0];
wire select1 = slot_state_ctl[1][0] & ~slot_state_ctl[0][0];
wire select2 = slot_state_ctl[2][0] & ~slot_state_ctl[1][0] & ~slot_state_ctl[0][0];
wire select3 = slot_state_ctl[3][0] & ~slot_state_ctl[2][0] & ~slot_state_ctl[1][0] & ~slot_state_ctl[0][0];

wire [3:0] select = {select3, select2, select1, select0};

// address of ready slot
assign rx_adr =
        select0 ? RX_SLOT0_ADDR[RX_ADDR_WIDTH-1:2] :
        select1 ? RX_SLOT1_ADDR[RX_ADDR_WIDTH-1:2] :
        select2 ? RX_SLOT2_ADDR[RX_ADDR_WIDTH-1:2] :
                  RX_SLOT3_ADDR[RX_ADDR_WIDTH-1:2];


wire [3:0] rx_slot_state_selected = {
    (reg_selector == REG_SLOT3_STATE) & wr_transaction,
    (reg_selector == REG_SLOT2_STATE) & wr_transaction,
    (reg_selector == REG_SLOT1_STATE) & wr_transaction,
    (reg_selector == REG_SLOT0_STATE) & wr_transaction};

wire [3:0] rx_slot_state_wr_req;
wire [3:0] rx_slot_state_wr_done_sys;

genvar i;

generate
    for(i = 0; i < 4; i = i + 1) begin
        TaskAck_CrossDomain rx_slot_state_wr_syncronyser(
            .clkA(sys_clk),
            .TaskStart_clkA(rx_slot_state_selected[i]),
            .TaskBusy_clkA(),
            .TaskDone_clkA(rx_slot_state_wr_done_sys[i]),

            .clkB(rmii_clk_i),
            .TaskStart_clkB(rx_slot_state_wr_req[i]),
            .TaskBusy_clkB(),
            .TaskDone_clkB(rx_slot_state_wr_req[i])
        );

        Signal_n_CrossDomain
        #(
            .CHANELS(2)
        ) rx_slot_state_read_syncronyser (
            .clkA(rmii_clk_i),
            .SignalIn_clkA(slot_state_ctl[i]),
            .clkB(sys_clk),
            .SignalOut_clkB(slot_state_sys[i])
        );

        Signal_n_CrossDomain
        #(
            .CHANELS(TRANSFER_COUNTER_LEN)
        ) rx_slot_count_read_syncronyser (
            .clkA(rmii_clk_i),
            .SignalIn_clkA(slot_count_ctl[i]),
            .clkB(sys_clk),
            .SignalOut_clkB(slot_count_sys[i])
        );

        always @(posedge rmii_clk_i) begin
            if (sys_rst) begin
                slot_state_ctl[i] <= 2'b00;
                slot_count_ctl[i] <= 0;
                active_slot[i] <= 1'b0;
            end else begin
                if (rx_slot_state_wr_req[i])  begin
                    slot_state_ctl[i] <= csr_di[1:0];
                end else begin
                    if (rx_resetcount) begin
                        active_slot[i] <= select[i];
                        if (select[i]) begin
                            slot_count_ctl[i] <= 0;
                        end
                    end

                    if (active_slot[i]) begin
                        if (rx_incrcount) begin
                            slot_count_ctl[i] <= slot_count_ctl[i] + 1;
                        end

                        if (rx_endframe) begin
                            slot_state_ctl[i] <= RX_SLOT_STATE_DATA_RESSIVED;
                        end
                    end
                end
            end
        end
    end
endgenerate

////////////////////////////////// TX /////////////////////////////////////////

/* tx addr register */
assign tx_adr = TX_SLOT_ADDR[TX_ADDR_WIDTH-1:2];

reg  [TRANSFER_COUNTER_LEN-1:0]              tx_counter;

wire tx_counter_wr_req;
wire tx_counter_selected = (reg_selector == REG_TX_REMANING) & wr_transaction;
wire tx_counter_wr_done_sys;

wire tx_busy;

assign tx_last_byte = (tx_counter == 1);
assign tx_valid = (|tx_counter) & tx_enabled;

TaskAck_CrossDomain tx_counter_wr_syncronyser (
    .clkA(sys_clk),
    .TaskStart_clkA(tx_counter_selected),
    .TaskBusy_clkA(),
    .TaskDone_clkA(tx_counter_wr_done_sys),

    .clkB(rmii_clk_i),
    .TaskStart_clkB(tx_counter_wr_req),
    .TaskBusy_clkB(),
    .TaskDone_clkB(tx_counter_wr_req)
);

Signal_CrossDomain tx_busy_flag_syncronyser (
    .clkA(rmii_clk_i),
    .SignalIn_clkA(tx_valid),
    .clkB(sys_clk),
    .SignalOut_clkB(tx_busy)
);

always @(posedge rmii_clk_i) begin
    if (sys_rst) begin
        tx_counter <= 0;
    end else begin
        if (tx_next) begin
            tx_counter <= tx_counter - 1;
        end else if (tx_counter_wr_req) begin
            tx_counter <= csr_di[TRANSFER_COUNTER_LEN-1:0];
        end
    end
end

///////////////////////////////////////////////////////////////////////////////

// wishbone interfase
always  @(posedge sys_clk) begin
    if(sys_rst) begin
        csr_ack <= 1'b0;
        _csr_do <= 0;
    end else begin
        if (csr_cyc) begin
            if(csr_we) begin
                // write
                csr_ack <=  rst_ctl_wr_done_sys |
                            tx_counter_wr_done_sys |
                            (|rx_slot_state_wr_done_sys);
                _csr_do <= 0;
            end else begin
                //read
                case (reg_selector)
                    REG_RESET_CTL:
                        _csr_do <= rst_ctl_sys;

                    REG_SLOT0_STATE:
                        _csr_do <= slot_state_sys[0];
                    REG_SLOT1_STATE:
                        _csr_do <= slot_state_sys[1];
                    REG_SLOT2_STATE:
                        _csr_do <= slot_state_sys[2];
                    REG_SLOT3_STATE:
                        _csr_do <= slot_state_sys[3];

                    REG_SLOT0_COUNT:
                        _csr_do <= slot_count_sys[0];
                    REG_SLOT1_COUNT:
                        _csr_do <= slot_count_sys[1];
                    REG_SLOT2_COUNT:
                        _csr_do <= slot_count_sys[2];
                    REG_SLOT3_COUNT:
                        _csr_do <= slot_count_sys[3];

                    REG_TX_REMANING:
                        _csr_do <= {31'd0, tx_busy};
                    default:
                        _csr_do <= 0;
                endcase
                csr_ack <= !csr_ack;
            end
        end else begin
            csr_ack <= 1'b0;
        end
    end
end

///////////////////////////////////////////////////////////////////////////////

wire [4:0]  irq_rx_status = {slot_state_ctl[0][1], slot_state_ctl[1][1],
                    slot_state_ctl[2][1], slot_state_ctl[3][1], rx_rst};
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
