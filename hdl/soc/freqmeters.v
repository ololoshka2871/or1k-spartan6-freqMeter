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
//* 3. Neither the name NuttX nor the names of its contributors may be
//*    used to endorse or promote products derived from this software
//*    without specific prior written permission.
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

module freqmeters
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
    input  wire [8:0]                   adr_i,         // address
    input  wire				we_i,          // write enable
    input  wire [`WB_DATA_WIDTH - 1:0]	dat_i,         // data input
    output wire [`WB_DATA_WIDTH - 1:0]	dat_o,         // data output
    output reg 				ack_o,         // normal bus termination
    output wire				inta_o,        // interrupt output

    input  wire                         F_master,      // частота образцовая
    input  wire [INPUTS_COUNT - 1:0]    F_in,          // входы для частоты

    output wire [MASER_FREQ_COUNTER_LEN-1:0] devided_clocks // clk_i деленная на 2, 4, ... 2^MASER_FREQ_COUNTER_LEN
);

parameter REGFILE_ADDR_WIDTH = $clog2(INPUTS_COUNT);

// Счетчик опорной частоты
reg  [MASER_FREQ_COUNTER_LEN-1:0] mester_freq_counter;

//--------------------------------registers-------------------------------------

// FM_IE (RW)
reg  [INPUTS_COUNT-1:0]           irq_enable; // разрешить прерывания на этих каналах (RW)

reg  [INPUTS_COUNT-1:0]           irq_flags; // Флаги готовности каналов

//--------------------------------memory----------------------------------------

// PORTA - freqmeters
// PORTB - wishbone

// 0x0 - 0x3F 
// [0x00 - 0x1f] - START_vals
// (* RAM_STYLE="BLOCK" *)
reg  [31:0] START_vals [31:0];

// [0x20 - 0x3f] - STOP_vals
// (* RAM_STYLE="BLOCK" *)
reg  [31:0] STOP_vals  [31:0];

wire [6:0] addr_valid = adr_i[8:2]; // 32 bit addr

wire [4:0] memory_addr     = addr_valid[4:0];
wire START_selector        = addr_valid[5];
wire memory_selector       = addr_valid[6];

//------------------------------------------------------------------------------

reg  [INPUT_FREQ_COUNTER_LEN - 1:0] reload_value;
reg  [INPUTS_COUNT - 1:0] restarts;

//------------------------------------------------------------------------------

wire not_clk_i = ~clk_i;

wire [INPUTS_COUNT - 1:0] start_requests;
wire [INPUTS_COUNT - 1:0] stop_requests;

wire [REGFILE_ADDR_WIDTH - 1:0] start_req_addr;
wire [REGFILE_ADDR_WIDTH - 1:0] stop_req_addr;

wire ready_start;
wire ready_stop;

wire [INPUTS_COUNT - 1:0] decoded_freqmeter_num;

//------------------------------------------------------------------------------

assign inta_o = (irq_flags & irq_enable) != {INPUTS_COUNT{1'b0}};
assign devided_clocks = mester_freq_counter;

//------------------------------------------------------------------------------
// endian control
//------------------------------------------------------------------------------

reg [`WB_DATA_WIDTH - 1:0] _dat_o;

assign dat_o = _dat_o;

wire [`WB_DATA_WIDTH - 1:0] _dat_i;

assign _dat_i = dat_i;

//------------------------------------------------------------------------------

genvar i;

generate
    for (i = 0; i < INPUTS_COUNT; i = i + 1) begin
        freq_meter_1
        #(
            .INPUT_FREQ_COUNTER_LEN(INPUT_FREQ_COUNTER_LEN)
        ) fm_inst (
            .rst_i(rst_i),
            .clk_i(F_master),

            .reload_val(reload_value),

            .restart(restarts[i]),

            .write_start_req(start_requests[i]),
            .write_start_enable_i(ready_start),

            .write_stop_req(stop_requests[i]),
            .write_stop_enable_i(ready_stop),

            .Fin_unsync(F_in[i])
        );
    end
endgenerate

coder
#(
    .INPUTS_COUNT(INPUTS_COUNT)
)  start_addr_coder (
    .clk_i(F_master),
    .inputs(start_requests),
    .outputs(start_req_addr),
    .error(ready_start)
), stop_addr_coder  (
    .clk_i(F_master),
    .inputs(stop_requests),
    .outputs(stop_req_addr),
    .error(ready_stop)
);

decoder
#(
    .OUTPUTS_COUNT(INPUTS_COUNT)
) freq_meter_n_decoder (
    .inputs(addr_valid),
    .outputs(decoded_freqmeter_num)
);

//------------------------------------------------------------------------------

integer j;

initial begin
    for(j = 0; j < 32; j = j + 1) begin
        if (j < INPUTS_COUNT) begin
            START_vals[j] = j+1;
            STOP_vals[j]  = ~j;
        end else begin
            START_vals[j] = 32'hDEADBEAF;
            STOP_vals[j]  = 32'hDEADBEAF;
        end
    end
end

//------------------------------------------------------------------------------

always @(posedge F_master) begin
    if (~ready_start)
        START_vals[start_req_addr] <= mester_freq_counter; // захват старта
    if (~ready_stop)
        STOP_vals[stop_req_addr] <= mester_freq_counter; // захват стопа
end


always @(posedge F_master or posedge rst_i) begin
    if (rst_i) begin
        irq_flags <= {INPUTS_COUNT{1'b0}};
    end else begin
        irq_flags <= (irq_flags | stop_requests) & ~restarts;
        if (we_i & cyc_i & stb_i & ~ack_o & ~memory_selector) begin
            // reset ready flag on write 1 in irq_flags[bit]
            if ({START_selector, memory_addr} == 6'b000001)
                irq_flags <= irq_flags & (~_dat_i[INPUTS_COUNT-1:0]);
        end
    end
end

//------------------------------------------------------------------------------

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        irq_enable <= {INPUTS_COUNT{1'b0}};
    end else begin
        ack_o <= 1'b0;
        restarts <= {INPUTS_COUNT{1'b0}};

        if (cyc_i & stb_i & ~ack_o) begin
            if (memory_selector) begin
                if (START_selector)
                    _dat_o <= {{(`WB_DATA_WIDTH-MASER_FREQ_COUNTER_LEN){1'b0}}, START_vals[memory_addr]};
                else
                    _dat_o <= {{(`WB_DATA_WIDTH-MASER_FREQ_COUNTER_LEN){1'b0}}, STOP_vals[memory_addr]};
            end else begin
                if (we_i) begin
                    if (START_selector) begin
                        reload_value <= _dat_i[INPUT_FREQ_COUNTER_LEN - 1:0];
                        restarts <= decoded_freqmeter_num;
                    end else begin
                        if (memory_addr == 5'b00000)
                            irq_enable <= _dat_i[INPUTS_COUNT-1:0];
                    end
                end
                case ({START_selector, memory_addr}) // read registers
                    6'b000000:
                        _dat_o <= {{(`WB_DATA_WIDTH - INPUTS_COUNT){1'b0}}, irq_enable};
                    6'b000001:
                        _dat_o <= {{(`WB_DATA_WIDTH - INPUTS_COUNT){1'b0}}, irq_flags};
                    default:
                        _dat_o <= {`WB_DATA_WIDTH{1'b0}};
                endcase
            end
            ack_o <= 1'b1;
        end
    end
end

always @(posedge F_master or posedge rst_i) begin
    if (rst_i)
        mester_freq_counter <= {MASER_FREQ_COUNTER_LEN{1'b0}};
    else
        mester_freq_counter <= mester_freq_counter + 1;
end

//------------------------------------------------------------------------------

endmodule
