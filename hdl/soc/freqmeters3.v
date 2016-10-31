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
//*
//****************************************************************************/

`define WB_DATA_WIDTH 32 // только для 32 битной шмны WishBone
`define MAX_FREQMETERS 32 // Максимальный размер модуля

module freqmeters3
#(
    parameter   INPUTS_COUNT            = 24, // количество входов (1 - 32)
    parameter   MASER_FREQ_COUNTER_LEN  = 30, // длина регистра, считающего опорную частоту
    parameter   INPUT_FREQ_COUNTER_LEN  = 24  // длина региста, считающего входную частоту
) (
    // WISHBONE bus slave interface
    input  wire				clk_i,         // clock
    input  wire				rst_i,         // reset (asynchronous active low)
    input  wire				cyc_i,         // cycle
    input  wire				stb_i,         // strobe
    input  wire [9:0]                   adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [`WB_DATA_WIDTH - 1:0]	dat_i,         // data input
    output wire [`WB_DATA_WIDTH - 1:0]	dat_o,         // data output
    output wire				ack_o,         // normal bus termination
    output wire				inta_o,        // interrupt output

    input  wire                         F_master,      // частота образцовая
    input  wire [INPUTS_COUNT - 1:0]    F_in,          // входы для частоты

    output wire [MASER_FREQ_COUNTER_LEN-1:0] devided_clocks // clk_i деленная на 2, 4, ... 2^MASER_FREQ_COUNTER_LEN
);

//------------------------------------------------------------------------------

parameter REGFILE_ADDR_WIDTH = $clog2(INPUTS_COUNT);

// Счетчик опорной частоты
wire  [MASER_FREQ_COUNTER_LEN-1:0] mester_freq_counter;

hybrid_counter
#(
    .WIDTH(MASER_FREQ_COUNTER_LEN)
) mfc (
    .clk(F_master),
    .rst(rst_i),
    .result_o(mester_freq_counter)
);

//--------------------------------registers-------------------------------------

reg  [INPUTS_COUNT-1:0]           irq_enable; // разрешить прерывания на этих каналах (RW)
wire [INPUTS_COUNT-1:0]           irq_flags; // Флаги готовности каналов

//------------------------------------------------------------------------------

reg  [INPUT_FREQ_COUNTER_LEN - 1:0] reload_value;
reg  [INPUTS_COUNT - 1:0]         restarts;

wire [INPUTS_COUNT - 1:0]         freqmeter_selector;

//------------------------------------------------------------------------------

reg  [INPUT_FREQ_COUNTER_LEN - 1:0] reload_value_sync;
reg  [INPUTS_COUNT - 1:0]         restarts_sync;

//------------------------------------------------------------------------------

wire [INPUTS_COUNT - 1:0]         start_requests;
wire [INPUTS_COUNT - 1:0]         stop_requests;

//------------------------------------------------------------------------------

wire [7:0]                        freqmeter_ctl_addr;
reg  [`WB_DATA_WIDTH - 1:0]       freqmeter_ctl_do;
wire [`WB_DATA_WIDTH - 1:0]       freqmeter_ctl_di;
wire                              freqmeter_ctl_we;


// RO
wire [REGFILE_ADDR_WIDTH-1+2:0]   Start_vals_addr;
wire [MASER_FREQ_COUNTER_LEN-1:0] Start_vals_do;
wire                              Start_vals_cyc;
wire                              Start_vals_ack;

// RO
wire [REGFILE_ADDR_WIDTH-1+2:0]   Stop_vals_addr;
wire [MASER_FREQ_COUNTER_LEN-1:0] Stop_vals_do;
wire                              Stop_vals_cyc;
wire                              Stop_vals_ack;

//------------------------------------------------------------------------------

wire [5:0]                        addr_valid      = freqmeter_ctl_addr[7:2];
wire [4:0]                        freqmeter_addr  = addr_valid[4:0];

//------------------------------------------------------------------------------

assign inta_o = (irq_flags & irq_enable) != {INPUTS_COUNT{1'b0}};
assign devided_clocks = mester_freq_counter;

genvar i;

generate
    for (i = 0; i < INPUTS_COUNT; i = i + 1) begin : gen_freqmeters
        freq_meter_1//_v2
        #(
            .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
        ) fm_inst (
            .rst_i(rst_i),
            .clk_i(F_master),

            .reload_val(reload_value_sync),

            .restart(restarts_sync[i]),

            .write_start_req(start_requests[i]),
            .write_stop_req(stop_requests[i]),

            .Fin_unsync(F_in[i]),

            .ready_o(irq_flags[i])
        );
    end

    if (INPUTS_COUNT == 1) begin
        assign freqmeter_selector = 1'b1;
    end else begin
        decoder
        #(
            .OUTPUTS_COUNT(INPUTS_COUNT)
        ) freq_meter_n_decoder (
            .inputs(freqmeter_addr),
            .outputs(freqmeter_selector)
        );
    end

endgenerate

//------------------------------------------------------------------------------

catcher
#(
    .INPUT_COUNT(INPUTS_COUNT),
    .VALUE_WIDTH(MASER_FREQ_COUNTER_LEN)
)  start_catcher (
    .clk_i(F_master),
    .rst_i(rst_i),

    .value2catch_i(mester_freq_counter),
    .catch_requests_i(start_requests),

    .read_clk_i(clk_i),
    .read_addr_i(Start_vals_addr >> 2),
    .read_do_o(Start_vals_do),

    .cyc_i(Start_vals_cyc),
    .ack_o(Start_vals_ack)
) ,stop_catcher  (
    .clk_i(F_master),
    .rst_i(rst_i),

    .value2catch_i(mester_freq_counter),
    .catch_requests_i(stop_requests),

    .read_clk_i(clk_i),
    .read_addr_i(Stop_vals_addr >> 2),
    .read_do_o(Stop_vals_do),

    .cyc_i(Stop_vals_cyc),
    .ack_o(Stop_vals_ack)
);

//------------------------------------------------------------------------------

dmem_mux4
#(
    .ADDR_MUX_START(8)
) addr_mux (
    // Outputs
    // 0x11000000 - 0x110000FF (ctl)
    .out0_addr_o(freqmeter_ctl_addr),
    .out0_data_o(freqmeter_ctl_di),
    .out0_data_i(freqmeter_ctl_do),
    .out0_sel_o(/* open */),
    .out0_we_o(freqmeter_ctl_we),
    .out0_stb_o(/* open */),
    .out0_cyc_o(/* open */),
    .out0_cti_o(/* open */),
    .out0_ack_i(1'b1),
    .out0_stall_i(1'b0),

    // 0x11000100 - 0x110001FF (start vals)
    .out1_addr_o(Start_vals_addr),
    .out1_data_o(/* open */),
    .out1_data_i({{(32-MASER_FREQ_COUNTER_LEN){1'b0}}, Start_vals_do}),
    .out1_sel_o(/*open*/),
    .out1_we_o( /*open*/ ),
    .out1_stb_o(/* open */),
    .out1_cyc_o(Start_vals_cyc),
    .out1_cti_o(/*open*/),
    .out1_ack_i(Start_vals_ack),
    .out1_stall_i(1'b0),

    // 0x11000200 - 0x110002FF (stop vals)
    .out2_addr_o(Stop_vals_addr),
    .out2_data_o(/* open */),
    .out2_data_i({{(32-MASER_FREQ_COUNTER_LEN){1'b0}}, Stop_vals_do}),
    .out2_sel_o(/* open */),
    .out2_we_o(/* open */),
    .out2_stb_o(/* open */),
    .out2_cyc_o(Stop_vals_cyc),
    .out2_cti_o(/* open */),
    .out2_ack_i(Stop_vals_ack),
    .out2_stall_i(1'b0),

    // 0x11000300 - 0x110003ff
    .out3_addr_o(/* open */),
    .out3_data_o(/* open */),
    .out3_data_i(32'hDEADBEAF),
    .out3_sel_o(/* open */),
    .out3_we_o(/* open */),
    .out3_stb_o(/* open */),
    .out3_cyc_o(/* open */),
    .out3_cti_o(/* open */),
    .out3_ack_i(1'b1),
    .out3_stall_i(1'b0),

    // Input 0x11000000 - 0x11000FFF
    .mem_addr_i({23'h0, adr_i}),
    .mem_data_i(dat_i),
    .mem_data_o(dat_o),
    .mem_sel_i(4'b1111),
    .mem_we_i(we_i),
    .mem_stb_i(stb_i),
    .mem_cyc_i(cyc_i),
    .mem_cti_i(3'b0),
    .mem_ack_o(ack_o),
    .mem_stall_o(/*open*/)
);

//------------------------------------------------------------------------------

always @(posedge F_master) begin
    reload_value_sync <= reload_value;
    restarts_sync <= restarts;
end

//------------------------------------------------------------------------------

always @(posedge clk_i) begin
    if (rst_i) begin
        restarts <= 0;
        irq_enable <= 0;
    end else begin
        restarts <= 0;
        // write
        if (freqmeter_ctl_we) begin
            case (addr_valid)
                6'b000000:
                    irq_enable <= freqmeter_ctl_di[INPUTS_COUNT-1:0];
                default:
                    if (addr_valid[5]) begin
                        reload_value <= freqmeter_ctl_di[INPUT_FREQ_COUNTER_LEN-1:0];
                        restarts <= freqmeter_selector;
                    end
            endcase
        end
        // read
        case (addr_valid)
            6'b000000:
                freqmeter_ctl_do <= irq_enable;
            6'b000001:
                freqmeter_ctl_do <= irq_flags;
            default:
                freqmeter_ctl_do <= 32'h0;
        endcase
    end
end

//------------------------------------------------------------------------------

endmodule
