// SPDX-License-Identifier: GPL-3.0-or-later
`timescale 1ns/1ps
`default_nettype none
module gamecom_memory (
    input wire clk_mem, clk_sdram, reset_mem,
    input wire clk_sys, reset_sys,
    input wire load_valid,
    output wire load_ready,
    input wire load_read, load_bios,
    input wire [20:0] load_addr,
    input wire [31:0] load_data,
    output reg load_done,
    output reg [31:0] load_rdata,
    output wire initialized,
    output reg fault,
    input wire [20:0] cart_addr,
    input wire cart_rd, slot1_sel, slot2_sel,
    input wire [21:0] rom_size,
    input wire cart_present,
    output wire [7:0] cart_data,
    output wire rom_read_ready,
    output wire [12:0] dram_a,
    output wire [1:0] dram_ba,
    inout wire [15:0] dram_dq,
    output wire [1:0] dram_dqm,
    output wire dram_clk, dram_cke, dram_ras_n, dram_cas_n, dram_we_n,
    output wire [16:0] sram_a,
    inout wire [15:0] sram_dq,
    output wire sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n
);
    function automatic [31:0] byte_reverse(input [31:0] value);
        byte_reverse={value[7:0],value[15:8],value[23:16],value[31:24]};
    endfunction

    wire bios_region=cart_addr<21'h40000;
    wire selected_present=cart_present && slot1_sel;
    wire [20:0] rom_mask=rom_size[20:0]-21'd1;
    wire [20:0] mapped_addr=(rom_size==22'h1c0000) ? cart_addr : (cart_addr & rom_mask);
    wire cpu_rom_read=cart_rd && !bios_region && selected_present;
    reg [17:0] bios_addr_q;
    wire [17:0] bios_read_addr=(cart_rd && bios_region) ? cart_addr[17:0] : bios_addr_q;

    reg cpu_req;
    reg [20:0] cpu_req_addr;
    reg cpu_ack;
    reg [31:0] cpu_response;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg cpu_ack_meta, cpu_ack_sync;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg cpu_req_meta, cpu_req_sync;
    reg [1:0] cpu_state;
    reg [2:0] reset_drain;
    reg cache_valid;
    reg [18:0] cache_tag;
    reg [31:0] cache_data;
    wire cache_hit=cache_valid && cache_tag==mapped_addr[20:2];
    wire [7:0] cached_byte=cache_data[{mapped_addr[1:0],3'b000} +: 8];
    assign rom_read_ready=!cpu_rom_read || cache_hit;
    assign cart_data=(cart_rd && !bios_region) ?
        (selected_present && cache_hit ? cached_byte : 8'hff) :
        (bios_read_addr[0] ? sram_dq[15:8] : sram_dq[7:0]);

    always @(posedge clk_sys) begin
        cpu_ack_meta<=cpu_ack;
        cpu_ack_sync<=cpu_ack_meta;
        if (cart_rd && bios_region) bios_addr_q<=cart_addr[17:0];
        if (!cart_present) cache_valid<=0;
        case (cpu_state)
            0: if (cpu_rom_read && !cache_hit) begin
                cpu_req_addr<={mapped_addr[20:2],2'b00};
                cpu_req<=1; cpu_state<=1;
            end
            1: if (cpu_ack_sync) begin
                cache_tag<=cpu_req_addr[20:2]; cache_data<=cpu_response;
                cache_valid<=1; cpu_req<=0; cpu_state<=2;
            end
            2: if (!cpu_ack_sync) begin
                if (reset_drain!=0) reset_drain<=reset_drain-1'b1;
                else cpu_state<=0;
            end
            default: begin cpu_req<=0; cpu_state<=2; end
        endcase
        if (reset_sys) begin
            cpu_req<=0; cpu_req_addr<=0; cpu_state<=2; reset_drain<=7;
            cache_valid<=0; cache_tag<=0; cache_data<=0; bios_addr_q<=0;
        end
        if (reset_mem) begin cpu_ack_meta<=0; cpu_ack_sync<=0; end
    end

    localparam [2:0] M_IDLE=0, M_SRAM=1, M_DRAM_REQ=2, M_DRAM_WAIT=3;
    reg [2:0] mem_state;
    reg txn_read, txn_bios, txn_cpu;
    reg [20:0] txn_addr;
    reg [31:0] txn_data;
    reg [31:0] sram_read_data;
    reg sram_half;
    reg [3:0] sram_count;
    reg sram_write_enable;
    reg sram_drive;
    reg dram_valid;
    wire dram_ready, dram_done;
    wire [31:0] dram_result;

    wire bios_owned=(mem_state==M_SRAM);
    assign sram_a=bios_owned ? {txn_addr[17:2],sram_half} : bios_read_addr[17:1];
    assign sram_we_n=!sram_write_enable;
    assign sram_oe_n=bios_owned ? !txn_read : reset_mem;
    assign sram_ub_n=1'b0;
    assign sram_lb_n=1'b0;
    assign sram_dq=sram_drive ?
        (sram_half ? txn_data[31:16] : txn_data[15:0]) : 16'hzzzz;
    assign load_ready=initialized && !fault && mem_state==M_IDLE && !cpu_req_sync;

    gamecom_sdram u_sdram (
        .clk(clk_mem), .clk_sdram(clk_sdram), .reset(reset_mem),
        .req_valid(dram_valid), .req_ready(dram_ready), .req_read(txn_read),
        .req_addr(txn_addr), .req_wdata(txn_data), .rsp_rdata(dram_result),
        .rsp_done(dram_done), .initialized(initialized),
        .dram_a(dram_a), .dram_ba(dram_ba), .dram_dq(dram_dq),
        .dram_dqm(dram_dqm), .dram_clk(dram_clk), .dram_cke(dram_cke),
        .dram_ras_n(dram_ras_n), .dram_cas_n(dram_cas_n), .dram_we_n(dram_we_n)
    );

    always @(posedge clk_mem) begin
        cpu_req_meta<=cpu_req;
        cpu_req_sync<=cpu_req_meta;
        load_done<=0;
        sram_write_enable<=bios_owned && !txn_read && sram_count>=1 && sram_count<=5;
        if (!cpu_req_sync) cpu_ack<=0;
        case (mem_state)
            M_IDLE: begin
                if (load_valid && load_ready) begin
                    if (load_addr[1:0]!=0 || (load_bios && load_addr[20:18]!=0)) begin
                        fault<=1; load_done<=1;
                    end else begin
                        txn_read<=load_read; txn_bios<=load_bios; txn_cpu<=0;
                        txn_addr<=load_addr; txn_data<=byte_reverse(load_data);
                        if (load_bios) begin
                            mem_state<=M_SRAM; sram_count<=0; sram_half<=0;
                        end else begin mem_state<=M_DRAM_REQ; dram_valid<=1; end
                    end
                end else if (initialized && cpu_req_sync && !cpu_ack) begin
                    txn_read<=1; txn_bios<=0; txn_cpu<=1;
                    txn_addr<=cpu_req_addr; txn_data<=0;
                    mem_state<=M_DRAM_REQ; dram_valid<=1;
                end
            end
            M_SRAM: begin
                if (!txn_read && !sram_half && sram_count==2) sram_drive<=1;
                if (txn_read && sram_count==6) begin
                    if (sram_half) sram_read_data[31:16]<=sram_dq;
                    else sram_read_data[15:0]<=sram_dq;
                end
                if (sram_count==8) begin
                    sram_count<=0;
                    if (!sram_half) sram_half<=1;
                    else if (txn_read) begin
                        load_rdata<=byte_reverse(sram_read_data);
                        load_done<=1; mem_state<=M_IDLE;
                    end else begin
                        sram_drive<=0; sram_count<=9;
                    end
                end else if (sram_count==10) begin
                    load_rdata<=byte_reverse(sram_read_data);
                    load_done<=1; mem_state<=M_IDLE;
                end else sram_count<=sram_count+1'b1;
            end
            M_DRAM_REQ: if (dram_ready) begin
                dram_valid<=0; mem_state<=M_DRAM_WAIT;
            end
            M_DRAM_WAIT: if (dram_done) begin
                if (txn_cpu) begin cpu_response<=dram_result; cpu_ack<=1; end
                else begin load_rdata<=byte_reverse(dram_result); load_done<=1; end
                mem_state<=M_IDLE;
            end
            default: begin fault<=1; mem_state<=M_IDLE; end
        endcase
        if (reset_mem) begin
            mem_state<=M_IDLE; load_done<=0; load_rdata<=0; fault<=0;
            cpu_req_meta<=0; cpu_req_sync<=0; cpu_ack<=0; cpu_response<=0;
            txn_read<=0; txn_bios<=0; txn_cpu<=0; txn_addr<=0; txn_data<=0;
            sram_count<=0; sram_half<=0; sram_read_data<=0; dram_valid<=0;
            sram_write_enable<=0; sram_drive<=0;
        end
    end
    wire unused_slot2=slot2_sel;
endmodule
`default_nettype wire
