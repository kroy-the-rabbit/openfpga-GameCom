// SPDX-License-Identifier: MIT
module gamecom_rom_loader (
    input wire clk_bridge, reset_bridge,
    input wire clk_mem, reset_mem,
    input wire bridge_wr,
    input wire [31:0] bridge_addr, bridge_wr_data,
    input wire dataslot_requestwrite,
    input wire [15:0] dataslot_requestwrite_id,
    input wire [31:0] dataslot_requestwrite_size,
    output reg dataslot_ack, dataslot_ok, dataslot_reject,
    input wire dataslot_allcomplete,
    input wire memory_initialized, memory_fault,
    output wire load_valid,
    input wire load_ready,
    output wire load_bios, load_read,
    output wire [20:0] load_addr,
    output wire [31:0] load_data,
    input wire load_done,
    input wire [31:0] load_rdata,
    output wire assets_ready,
    output reg bios_loaded, cart_loaded,
    output reg [21:0] rom_size,
    output wire busy,
    output reg [7:0] error,
    output reg [31:0] announced_bytes, received_bytes,
    output wire [31:0] committed_bytes,
    output reg [31:0] readback_crc32,
    output wire [31:0] input_crc32,
    output reg [6:0] fifo_high_water,
    output wire [3:0] phase
);
    reg request_seen, pending_request;
    reg [15:0] pending_id;
    reg [31:0] pending_size;
    reg active, active_bios, epoch;
    reg [20:0] backing_base;
    reg [31:0] input_crc;
    reg bridge_wr_previous, allcomplete_previous, host_complete;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg initialized_b1, initialized_b2, fault_b1, fault_b2;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg done_b1, done_b2, started_b1, started_b2;
    reg done_seen;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [19:0] committed_gray_b1, committed_gray_b2;

    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg epoch_m1, epoch_m2;
    reg epoch_seen;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg fault_m1, fault_m2;
    reg [31:0] expected_crc_m1, expected_crc_m2;
    reg mem_started, mem_done, mem_crc_bad;
    reg [31:0] mem_result_crc;
    reg [19:0] committed_words, committed_gray;
    reg [21:0] mem_size, read_count;
    reg [20:0] mem_base;
    reg mem_bios;
    reg [31:0] read_crc;
    localparam M_IDLE=0, M_POP=1, M_CAPTURE=2, M_WRITE=3,
               M_WRITE_WAIT=4, M_READ=5, M_READ_WAIT=6, M_FAULT=7;
    reg [3:0] mem_state;
    reg [53:0] write_payload;
    wire fifo_full, fifo_empty;
    wire [6:0] fifo_used;
    wire [53:0] fifo_data;
    wire asset_write = bridge_wr && !bridge_wr_previous &&
                       ((bridge_addr[31:28] == 4'h1) || (bridge_addr[31:28] == 4'h3));
    wire [31:0] expected_address = (active_bios ? 32'h30000000 : 32'h10000000) + received_bytes;
    wire valid_write = asset_write && active && (error == 0) &&
                       bridge_addr == expected_address && received_bytes < announced_bytes;
    wire fifo_push = valid_write && !fifo_full;
    wire fifo_pop = mem_state == M_POP && !fifo_empty && !fault_m2;
    wire [20:0] write_backing_address = backing_base + received_bytes[20:0];

    function [31:0] crc32_word(input [31:0] crc, input [31:0] word_data);
        reg [31:0] c;
        reg [7:0] b;
        integer n, k;
        begin
            c = crc;
            for (n=0; n<4; n=n+1) begin
                b = word_data[31 - n*8 -: 8];
                c = c ^ b;
                for (k=0; k<8; k=k+1)
                    c = (c >> 1) ^ (c[0] ? 32'hedb88320 : 32'b0);
            end
            crc32_word = c;
        end
    endfunction
    function supported_rom_size(input [31:0] size);
        begin
            case (size)
                32'h8000,32'h40000,32'h80000,32'h100000,32'h1c0000,32'h200000:
                    supported_rom_size = 1'b1;
                default: supported_rom_size = 1'b0;
            endcase
        end
    endfunction
    function [19:0] gray_to_bin(input [19:0] gray);
        integer n;
        begin
            gray_to_bin[19] = gray[19];
            for (n=18; n>=0; n=n-1) gray_to_bin[n] = gray_to_bin[n+1] ^ gray[n];
        end
    endfunction

    assign busy = active || pending_request;
    assign assets_ready = bios_loaded && cart_loaded && host_complete &&
                          !busy && !dataslot_requestwrite && (error == 0) && initialized_b2;
    assign committed_bytes = started_b2 == epoch ? {10'b0,gray_to_bin(committed_gray_b2),2'b0} : 32'b0;
    assign input_crc32 = ~input_crc;
    assign phase = error != 0 ? 4'hf : assets_ready ? 4'h4 :
                   active ? (received_bytes == announced_bytes ? 4'h3 : 4'h2) :
                   initialized_b2 ? 4'h1 : 4'h0;
    assign load_valid = (mem_state == M_WRITE || mem_state == M_READ) && !fault_m2;
    assign load_read = mem_state == M_READ;
    assign load_bios = mem_bios;
    assign load_addr = mem_state == M_READ ? mem_base + read_count[20:0] : write_payload[52:32];
    assign load_data = write_payload[31:0];

    gamecom_async_fifo #(.WIDTH(54),.ADDR_BITS(6)) transfer_fifo (
        .wr_clk(clk_bridge), .wr_reset(reset_bridge), .wr_en(fifo_push),
        .wr_data({active_bios,write_backing_address,bridge_wr_data}),
        .wr_full(fifo_full), .wr_used(fifo_used),
        .rd_clk(clk_mem), .rd_reset(reset_mem), .rd_en(fifo_pop),
        .rd_data(fifo_data), .rd_empty(fifo_empty)
    );

    always @(posedge clk_bridge) begin
        if (reset_bridge) begin
            request_seen<=0; pending_request<=0; pending_id<=0; pending_size<=0;
            active<=0; active_bios<=0; epoch<=0; backing_base<=0;
            input_crc<=32'hffffffff; bridge_wr_previous<=0;
            allcomplete_previous<=0; host_complete<=0;
            initialized_b1<=0; initialized_b2<=0; fault_b1<=0; fault_b2<=0;
            done_b1<=0; done_b2<=0; done_seen<=0; started_b1<=0; started_b2<=0;
            committed_gray_b1<=0; committed_gray_b2<=0;
            bios_loaded<=0; cart_loaded<=0; rom_size<=0; error<=0;
            announced_bytes<=0; received_bytes<=0; readback_crc32<=0; fifo_high_water<=0;
            dataslot_ack<=0; dataslot_ok<=0; dataslot_reject<=0;
        end else begin
            initialized_b1<=memory_initialized; initialized_b2<=initialized_b1;
            fault_b1<=memory_fault; fault_b2<=fault_b1;
            done_b1<=mem_done; done_b2<=done_b1;
            started_b1<=mem_started; started_b2<=started_b1;
            committed_gray_b1<=committed_gray; committed_gray_b2<=committed_gray_b1;
            bridge_wr_previous<=bridge_wr;
            allcomplete_previous<=dataslot_allcomplete;
            dataslot_ack<=0; dataslot_ok<=0; dataslot_reject<=0;
            if (dataslot_allcomplete && !allcomplete_previous) host_complete<=1;
            if (!dataslot_requestwrite) request_seen<=0;
            if (dataslot_requestwrite && !request_seen) begin
                pending_id<=dataslot_requestwrite_id;
                pending_size<=dataslot_requestwrite_size;
                pending_request<=1;
                request_seen<=1;
                host_complete<=0;
                if (active && received_bytes != announced_bytes && error == 0) begin
                    error<=8'd8;
                    active<=0;
                end
            end
            if (pending_request && ((!active && initialized_b2) || error != 0)) begin
                pending_request<=0;
                dataslot_ack<=1;
                if (error != 0 || (pending_id != 1 && pending_id != 4) ||
                    (pending_id == 4 ? pending_size != 32'h40000 : !supported_rom_size(pending_size))) begin
                    dataslot_reject<=1;
                    if (error == 0) error<=(pending_id != 1 && pending_id != 4) ? 8'd1 : 8'd2;
                end else begin
                    dataslot_ok<=1; active<=1; active_bios<=pending_id == 4;
                    announced_bytes<=pending_size;
                    received_bytes<=0; input_crc<=32'hffffffff;
                    readback_crc32<=0; fifo_high_water<=0;
                    backing_base<=(pending_id == 1 && pending_size == 32'h1c0000) ? 21'h40000 : 21'b0;
                    epoch<=!epoch;
                    if (pending_id == 4) bios_loaded<=0;
                    else begin cart_loaded<=0; rom_size<=pending_size[21:0]; end
                end
            end
            if (fifo_used > fifo_high_water) fifo_high_water<=fifo_used;
            if (asset_write && error == 0) begin
                if (!active) begin error<=8'd3; active<=0; end
                else if (!valid_write) begin error<=8'd4; active<=0; end
                else if (fifo_full) begin error<=8'd5; active<=0; end
                else begin
                    received_bytes<=received_bytes+4;
                    input_crc<=crc32_word(input_crc,bridge_wr_data);
                end
            end
            if (done_b2 != done_seen) begin
                done_seen<=done_b2;
                readback_crc32<=mem_result_crc;
                active<=0;
                if (mem_crc_bad) begin
                    if (error == 0) error<=8'd7;
                end else if (error == 0) begin
                    if (active_bios) bios_loaded<=1;
                    else cart_loaded<=1;
                end
            end
            if (fault_b2 && error == 0) begin error<=8'd6; active<=0; end
        end
    end

    always @(posedge clk_mem) begin
        if (reset_mem) begin
            epoch_m1<=0; epoch_m2<=0; epoch_seen<=0;
            fault_m1<=0; fault_m2<=0; expected_crc_m1<=0; expected_crc_m2<=0;
            mem_started<=0; mem_done<=0; mem_crc_bad<=0; mem_result_crc<=0;
            committed_words<=0; committed_gray<=0; mem_size<=0; read_count<=0;
            mem_base<=0; mem_bios<=0; read_crc<=32'hffffffff;
            mem_state<=M_IDLE; write_payload<=0;
        end else begin
            epoch_m1<=epoch; epoch_m2<=epoch_m1;
            fault_m1<=error != 0; fault_m2<=fault_m1;
            expected_crc_m1<=input_crc; expected_crc_m2<=expected_crc_m1;
            case (mem_state)
                M_IDLE: if (epoch_m2 != epoch_seen && memory_initialized && !fault_m2) begin
                    epoch_seen<=epoch_m2; mem_started<=epoch_m2;
                    mem_size<=announced_bytes[21:0]; mem_bios<=active_bios; mem_base<=backing_base;
                    committed_words<=0; committed_gray<=0; read_count<=0;
                    read_crc<=32'hffffffff; mem_crc_bad<=0;
                    mem_state<=M_POP;
                end
                M_POP: if (fifo_pop) mem_state<=M_CAPTURE;
                M_CAPTURE: begin write_payload<=fifo_data; mem_state<=M_WRITE; end
                M_WRITE: if (load_valid && load_ready) mem_state<=M_WRITE_WAIT;
                M_WRITE_WAIT: if (load_done) begin
                    committed_words<=committed_words+1'b1;
                    committed_gray<=((committed_words+20'd1)>>1) ^ (committed_words+20'd1);
                    if ({committed_words,2'b0}+22'd4 == mem_size) mem_state<=M_READ;
                    else mem_state<=M_POP;
                end
                M_READ: if (load_valid && load_ready) mem_state<=M_READ_WAIT;
                M_READ_WAIT: if (load_done) begin
                    read_crc<=crc32_word(read_crc,load_rdata);
                    if (read_count+22'd4 == mem_size) begin
                        mem_result_crc<=~crc32_word(read_crc,load_rdata);
                        mem_crc_bad<=crc32_word(read_crc,load_rdata) != expected_crc_m2;
                        mem_done<=!mem_done;
                        mem_state<=M_IDLE;
                    end else begin read_count<=read_count+22'd4; mem_state<=M_READ; end
                end
                default: mem_state<=M_FAULT;
            endcase
            if (fault_m2 || memory_fault) mem_state<=M_FAULT;
        end
    end
endmodule
