// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2015-2019 Sorgelig (original 3-channel design)
// Adapted from pocket-gba/sdram_pocket.sv; Pocket integration by kroy, 2026.
`timescale 1ns/1ps
`default_nettype none
module gamecom_sdram #(
    parameter integer STARTUP_CYCLES = 16400,
    parameter integer REFRESH_CYCLES = 450
) (
    input wire clk, clk_sdram, reset,
    input wire req_valid,
    output wire req_ready,
    input wire req_read,
    input wire [20:0] req_addr,       // four-byte-aligned BYTE address
    input wire [31:0] req_wdata,      // little-endian, first byte [7:0]
    output reg [31:0] rsp_rdata,
    output reg rsp_done,
    output reg initialized,
    output reg [12:0] dram_a,
    output reg [1:0] dram_ba,
    inout wire [15:0] dram_dq,
    output wire [1:0] dram_dqm,
    output wire dram_clk, dram_cke, dram_ras_n, dram_cas_n, dram_we_n
);
    localparam [2:0] NOP=3'b111, ACTIVE=3'b011, READ=3'b101,
        WRITE=3'b100, PRECHARGE=3'b010, REFRESH=3'b001, MODE=3'b000;
    localparam [3:0] POWERUP=0, INIT_PRE=1, INIT_REF1=2, INIT_REF2=3,
        INIT_MODE=4, INIT_EXT=5, INIT_DONE=6, IDLE=7, RCD=8,
        ACCESS=9, DATA=10, REF_WAIT=11;
    reg [3:0] state;
    reg [2:0] command;
    reg [15:0] startup_count;
    reg [9:0] refresh_count;
    reg [3:0] delay_count;
    reg [20:0] addr_q;
    reg [31:0] data_q;
    reg read_q, dq_drive;
    reg [15:0] dq_out;
    reg [15:0] dq_capture;

    assign {dram_ras_n,dram_cas_n,dram_we_n}=command;
    assign dram_dq=dq_drive ? dq_out : 16'hzzzz;
    assign dram_dqm=2'b00;
    assign dram_cke=1'b1;
    assign req_ready=initialized && state==IDLE && refresh_count<10'(REFRESH_CYCLES);

    always @(negedge clk) dq_capture <= dram_dq;

    always @(posedge clk) begin
        command <= NOP;
        dq_drive <= 1'b0;
        rsp_done <= 1'b0;
        if (initialized && refresh_count != 10'h3ff)
            refresh_count <= refresh_count + 1'b1;
        case (state)
            POWERUP: begin
                if (startup_count == 16'(STARTUP_CYCLES-1)) begin
                    command <= PRECHARGE;
                    dram_a <= 13'h400; // all banks
                    delay_count <= 0;
                    state <= INIT_PRE;
                end else startup_count <= startup_count+1'b1;
            end
            INIT_PRE: if (delay_count==1) begin
                command<=REFRESH; delay_count<=0; state<=INIT_REF1;
            end else delay_count<=delay_count+1'b1;
            INIT_REF1: if (delay_count==6) begin
                command<=REFRESH; delay_count<=0; state<=INIT_REF2;
            end else delay_count<=delay_count+1'b1;
            INIT_REF2: if (delay_count==6) begin
                command<=MODE; dram_ba<=0; dram_a<=13'h021;
                delay_count<=0; state<=INIT_MODE; // BL2, sequential, CAS2, burst writes
            end else delay_count<=delay_count+1'b1;
            INIT_MODE: if (delay_count==1) begin
                command<=MODE; dram_ba<=2'b10; dram_a<=0;
                delay_count<=0; state<=INIT_EXT; // full array, full drive
            end else delay_count<=delay_count+1'b1;
            INIT_EXT: if (delay_count==1) begin
                initialized<=1; refresh_count<=0; state<=IDLE;
            end else delay_count<=delay_count+1'b1;
            IDLE: begin
                if (refresh_count>=10'(REFRESH_CYCLES)) begin
                    command<=REFRESH; refresh_count<=0;
                    delay_count<=0; state<=REF_WAIT;
                end else if (req_valid) begin
                    addr_q<=req_addr; data_q<=req_wdata; read_q<=req_read;
                    dram_a<={5'd0,req_addr[20:13]};
                    dram_ba<=req_addr[12:11];
                    command<=ACTIVE; delay_count<=0; state<=RCD;
                end
            end
            RCD: if (delay_count==0) begin
                delay_count<=1;
            end else begin
                command<=read_q ? READ : WRITE;
                dram_a<={2'b00,1'b1,addr_q[10:1]}; // auto precharge
                if (!read_q) begin dq_drive<=1; dq_out<=data_q[15:0]; end
                delay_count<=0; state<=DATA;
            end
            DATA: begin
                if (!read_q && delay_count==0) begin
                    dq_drive<=1; dq_out<=data_q[31:16];
                end
                if (read_q && delay_count==2) rsp_rdata[15:0]<=dq_capture;
                if (read_q && delay_count==3) rsp_rdata[31:16]<=dq_capture;
                if (delay_count==6) begin rsp_done<=1; state<=IDLE; end
                else delay_count<=delay_count+1'b1;
            end
            REF_WAIT: if (delay_count==6) state<=IDLE;
                else delay_count<=delay_count+1'b1;
            default: state<=POWERUP;
        endcase
        if (reset) begin
            state<=POWERUP; startup_count<=0; refresh_count<=0;
            delay_count<=0; command<=NOP; initialized<=0; rsp_done<=0;
            rsp_rdata<=0; dram_a<=0; dram_ba<=0; dq_drive<=0; dq_out<=0;
            addr_q<=0; data_q<=0; read_q<=0;
        end
    end

`ifdef ALTERA_RESERVED_QIS
    altddio_out #(
        .extend_oe_disable("OFF"), .intended_device_family("Cyclone V"),
        .invert_output("OFF"), .lpm_hint("UNUSED"), .lpm_type("altddio_out"),
        .oe_reg("UNREGISTERED"), .power_up_high("OFF"), .width(1)
    ) sdram_clk_fwd (
        .datain_h(1'b1), .datain_l(1'b0), .outclock(clk_sdram),
        .dataout(dram_clk), .aclr(1'b0), .aset(1'b0), .oe(1'b1),
        .outclocken(1'b1), .sclr(1'b0), .sset(1'b0)
    );
`else
    assign dram_clk=clk_sdram;
`endif
endmodule
`default_nettype wire
