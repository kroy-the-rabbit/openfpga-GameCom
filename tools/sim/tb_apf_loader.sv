`timescale 1ns/1ps
module tb_apf_loader;
    reg clk_bridge=0,clk_mem=0;
    always #6.734 clk_bridge=~clk_bridge;
    always #8.333 clk_mem=~clk_mem; // nominal 60MHz, slower than the bridge
    reg reset_bridge=1,reset_mem=1;
    reg bridge_wr=0,bridge_rd=0;
    reg [31:0] bridge_addr=0,bridge_wr_data=0;
    wire [31:0] bridge_rd_data;
    wire dataslot_requestwrite;
    wire [15:0] dataslot_requestwrite_id;
    wire [31:0] dataslot_requestwrite_size;
    wire dataslot_ack,dataslot_ok,dataslot_reject,dataslot_allcomplete;
    wire assets_ready,bios_loaded,cart_loaded,busy,reset_n;
    wire load_valid,load_bios,load_read;
    wire [20:0] load_addr;
    wire [31:0] load_data;
    reg load_done=0;
    wire [7:0] error;
    wire load_ready=!load_done;
    reg [31:0] response;
    gamecom_rom_loader loader (
        .clk_bridge(clk_bridge),.reset_bridge(reset_bridge),.clk_mem(clk_mem),.reset_mem(reset_mem),
        .bridge_wr(bridge_wr),.bridge_addr(bridge_addr),.bridge_wr_data(bridge_wr_data),
        .dataslot_requestwrite(dataslot_requestwrite),.dataslot_requestwrite_id(dataslot_requestwrite_id),
        .dataslot_requestwrite_size(dataslot_requestwrite_size),.dataslot_ack(dataslot_ack),
        .dataslot_ok(dataslot_ok),.dataslot_reject(dataslot_reject),.dataslot_allcomplete(dataslot_allcomplete),
        .memory_initialized(1'b1),.memory_fault(1'b0),.load_valid(load_valid),.load_ready(load_ready),
        .load_bios(load_bios),.load_read(load_read),.load_addr(load_addr),.load_data(load_data),
        .load_done(load_done),.load_rdata(32'b0),.assets_ready(assets_ready),.bios_loaded(bios_loaded),
        .cart_loaded(cart_loaded),.busy(busy),.error(error)
    );
    always @(posedge clk_mem) load_done <= !reset_mem && load_valid && load_ready;
    core_bridge_cmd command_handler (
        .clk(clk_bridge),.reset_n(reset_n),.bridge_endian_little(1'b0),
        .bridge_addr(bridge_addr),.bridge_rd(bridge_rd),.bridge_rd_data(bridge_rd_data),
        .bridge_wr(bridge_wr),.bridge_wr_data(bridge_wr_data),
        .status_boot_done(1'b1),.status_setup_done(assets_ready),.status_running(reset_n&&assets_ready),
        .dataslot_requestread_ack(1'b1),.dataslot_requestread_ok(1'b0),
        .dataslot_requestwrite(dataslot_requestwrite),.dataslot_requestwrite_id(dataslot_requestwrite_id),
        .dataslot_requestwrite_size(dataslot_requestwrite_size),.dataslot_requestwrite_ack(dataslot_ack),
        .dataslot_requestwrite_ok(dataslot_ok),.dataslot_requestwrite_reject(dataslot_reject),
        .dataslot_allcomplete(dataslot_allcomplete),
        .savestate_supported(1'b0),.savestate_addr(32'b0),.savestate_size(32'b0),.savestate_maxloadsize(32'b0),
        .savestate_start_ack(1'b0),.savestate_start_busy(1'b0),.savestate_start_ok(1'b0),.savestate_start_err(1'b0),
        .savestate_load_ack(1'b0),.savestate_load_busy(1'b0),.savestate_load_ok(1'b0),.savestate_load_err(1'b0),
        .target_dataslot_read(1'b0),.target_dataslot_write(1'b0),.target_dataslot_getfile(1'b0),.target_dataslot_openfile(1'b0),
        .target_dataslot_id(16'b0),.target_dataslot_slotoffset(32'b0),.target_dataslot_bridgeaddr(32'b0),
        .target_dataslot_length(32'b0),.target_buffer_param_struct(32'b0),.target_buffer_resp_struct(32'b0),
        .datatable_addr(10'b0),.datatable_wren(1'b0),.datatable_data(32'b0)
    );
    task write_bus(input [31:0] addr,input [31:0] data);
        begin
            @(negedge clk_bridge); bridge_addr=addr; bridge_wr_data=data; bridge_wr=1;
            @(negedge clk_bridge); bridge_wr=0;
        end
    endtask
    task read_bus(input [31:0] addr);
        begin
            @(negedge clk_bridge); bridge_addr=addr; bridge_rd=1;
            @(negedge clk_bridge); response=bridge_rd_data; bridge_rd=0;
        end
    endtask
    task issue_command(input [15:0] command,input [15:0] expect_result);
        integer n;
        begin
            write_bus(32'hf8000000,{16'h434d,command});
            n=0; response=0;
            while(response[31:16] != 16'h4f4b && n<1000000) begin
                repeat(2) @(negedge clk_bridge);
                read_bus(32'hf8000000); n=n+1;
            end
            if(response !== {16'h4f4b,expect_result})
                $fatal(1,"APF command %h result %h expected %h",command,response,expect_result);
        end
    endtask
    task request_slot(input [15:0] id,input [31:0] size,input [15:0] expect_result);
        begin
            write_bus(32'hf8000020,{16'b0,id}); write_bus(32'hf8000024,size);
            issue_command(16'h0082,expect_result);
        end
    endtask
    task send_zeros(input [31:0] base,input integer size);
        integer offset;
        begin
            for(offset=0;offset<size;offset=offset+4) begin
                write_bus(base+offset,32'b0);
                repeat(7) @(negedge clk_bridge);
            end
        end
    endtask
    task reset_loader;
        begin
            @(negedge clk_bridge); reset_bridge=1;reset_mem=1;
            repeat(8) @(negedge clk_bridge);
            reset_bridge=0;reset_mem=0;
            repeat(8) @(negedge clk_bridge);
        end
    endtask
    initial begin #200000000; $fatal(1,"APF integration timeout"); end
    initial begin
        reset_loader();
        issue_command(16'h0000,16'd2); // Initial setup can accept assets.
        issue_command(16'h0010,0);
        request_slot(1,32'h8000,0);
        send_zeros(32'h10000000,32'h8000);
        request_slot(4,32'h40000,0);
        send_zeros(32'h30000000,32'h40000);
        issue_command(16'h008f,0);
        if(assets_ready) $fatal(1,"APF All Complete bypassed physical verification");
        issue_command(16'h0011,0);
        if(!reset_n || assets_ready) $fatal(1,"reset exit/asset gate");
        wait(assets_ready);
        if(error || !bios_loaded || !cart_loaded) $fatal(1,"APF valid load failed");
        issue_command(16'h0000,16'd4);
        read_bus(32'hf8001000);
        if(response !== 32'h636d0140) $fatal(1,"Ready To Run command not emitted: %h",response);
        write_bus(32'hf8001000,32'h6f6b0000);
        issue_command(16'h0010,0);
        if(!assets_ready || reset_n) $fatal(1,"warm reset discarded assets");
        issue_command(16'h0011,0);
        request_slot(1,32'h8004,1);
        if(error != 2 || assets_ready) $fatal(1,"permanent invalid-size rejection");
        reset_loader();
        request_slot(9,32'h8000,1);
        if(error != 1 || assets_ready) $fatal(1,"permanent unknown-slot rejection");
        $display("PASS APF command integration: setup, slots, deferred acknowledgment, allcomplete, Ready To Run, warm reset, terminal errors");
        $finish;
    end
endmodule

module mf_datatable (
    input wire [9:0] address_a,address_b,
    input wire clock_a,clock_b,
    input wire [31:0] data_a,data_b,
    input wire wren_a,wren_b,
    output reg [31:0] q_a,q_b
);
    reg [31:0] data[0:1023];
    always @(posedge clock_a) begin if(wren_a)data[address_a]<=data_a; q_a<=data[address_a]; end
    always @(posedge clock_b) begin if(wren_b)data[address_b]<=data_b; q_b<=data[address_b]; end
endmodule
