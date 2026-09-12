`timescale 1ns/1ps
module tb_rom_loader;
    reg clk_bridge=0, clk_mem=0;
    always #6.734 clk_bridge=~clk_bridge;
    always #8.333 clk_mem=~clk_mem; // nominal 60MHz, slower than the bridge
    reg reset_bridge=1, reset_mem=1, bridge_wr=0;
    reg [31:0] bridge_addr=0, bridge_wr_data=0;
    reg dataslot_requestwrite=0;
    reg [15:0] dataslot_requestwrite_id=0;
    reg [31:0] dataslot_requestwrite_size=0;
    wire dataslot_ack, dataslot_ok, dataslot_reject;
    reg dataslot_allcomplete=0, memory_initialized=0, memory_fault=0;
    wire load_valid, load_bios, load_read;
    wire [20:0] load_addr;
    wire [31:0] load_data;
    reg load_done=0;
    reg [31:0] load_rdata=0;
    wire assets_ready,bios_loaded,cart_loaded,busy;
    wire [21:0] rom_size;
    wire [7:0] error;
    wire [31:0] announced_bytes,received_bytes,committed_bytes,readback_crc32,input_crc32;
    wire [6:0] fifo_high_water;
    wire [3:0] phase;
    reg stall=0, corrupt_read=0, transaction_pending=0;
    wire load_ready=!transaction_pending && !stall;
    gamecom_rom_loader dut (.*);
    reg [31:0] cart_memory [0:524287];
    reg [31:0] bios_memory [0:65535];
    reg held_bios,held_read;
    reg [20:0] held_addr;
    reg [31:0] held_data;
    integer delay_count=0, transactions=0, ack_count=0;
    reg checking_held=0;
    reg [54:0] last_held;
    always @(posedge clk_bridge) if(dataslot_ack) ack_count=ack_count+1;
    always @(posedge clk_mem) begin
        load_done<=0;
        if(reset_mem) begin transaction_pending<=0; checking_held<=0; transactions<=0; end
        else begin
            if (checking_held && {load_bios,load_read,load_addr,load_data} !== last_held)
                $fatal(1,"memory request changed under backpressure");
            checking_held<=load_valid && !load_ready;
            last_held<={load_bios,load_read,load_addr,load_data};
            if(load_valid && load_ready) begin
                if(load_addr[1:0] != 0) $fatal(1,"unaligned backend request");
                if(load_bios && load_addr >= 'h40000) $fatal(1,"BIOS overrun");
                held_bios<=load_bios; held_read<=load_read; held_addr<=load_addr; held_data<=load_data;
                transaction_pending<=1;
                delay_count<=2+(transactions%5);
                transactions<=transactions+1;
            end
            if(transaction_pending) begin
                if(delay_count != 0) delay_count<=delay_count-1;
                else begin
                    if(held_read) begin
                        load_rdata<=(held_bios ? bios_memory[held_addr>>2] : cart_memory[held_addr>>2]) ^ (corrupt_read ? 32'h1 : 32'h0);
                    end else if(held_bios) bios_memory[held_addr>>2]<=held_data;
                    else cart_memory[held_addr>>2]<=held_data;
                    load_done<=1; transaction_pending<=0;
                end
            end
        end
    end
    function [31:0] pattern(input integer offset);
        pattern=32'h01234567 ^ (offset*32'h01010101);
    endfunction
    task reset_loader;
        begin
            @(negedge clk_bridge);
            reset_bridge=1; reset_mem=1; bridge_wr=0; dataslot_requestwrite=0;
            dataslot_allcomplete=0; memory_initialized=0; memory_fault=0;
            stall=0; corrupt_read=0;
            repeat(8) @(negedge clk_bridge);
            reset_bridge=0; reset_mem=0;
            repeat(8) @(negedge clk_bridge);
            memory_initialized=1;
            repeat(8) @(negedge clk_bridge);
        end
    endtask
    task request_slot(input integer id,input integer size,input integer expect_ok);
        integer n;
        begin
            @(negedge clk_bridge);
            dataslot_allcomplete=0; dataslot_requestwrite=1;
            dataslot_requestwrite_id=id; dataslot_requestwrite_size=size;
            #1;
            if(assets_ready) $fatal(1,"readiness not gated immediately on a new request");
            n=0;
            while(!dataslot_ack && n<10000000) begin @(negedge clk_bridge); n=n+1; end
            if(!dataslot_ack || dataslot_ok !== expect_ok[0] || dataslot_reject !== !expect_ok[0])
                $fatal(1,"slot request response id=%0d size=%h ok=%b reject=%b",id,size,dataslot_ok,dataslot_reject);
            dataslot_requestwrite_id=16'hf00d; dataslot_requestwrite_size=1;
            repeat(9) begin @(negedge clk_bridge); if(dataslot_ack) $fatal(1,"duplicate acknowledgment"); end
            dataslot_requestwrite=0;
            repeat(3) @(negedge clk_bridge);
        end
    endtask
    task push_word(input [31:0] address,input [31:0] data,input integer gap);
        begin
            @(negedge clk_bridge);
            bridge_addr=address; bridge_wr_data=data; bridge_wr=1;
            @(negedge clk_bridge); bridge_wr=0;
            repeat(gap) @(negedge clk_bridge);
        end
    endtask
    task send_slot(input integer id,input integer size,input integer complete_on_final);
        integer offset;
        reg [31:0] base;
        begin
            base=id==4 ? 32'h30000000 : 32'h10000000;
            for(offset=0;offset<size;offset=offset+4) begin
                if(complete_on_final && offset==size-4) dataslot_allcomplete=1;
                push_word(base+offset,pattern(offset),12);
                if(error) $fatal(1,"unexpected error %0d at offset %h",error,offset);
            end
        end
    endtask
    task await_slot(input integer id,input integer size);
        integer n;
        begin
            n=0;
            while(busy && n<10000000) begin @(negedge clk_bridge); n=n+1; end
            if(busy || error || (id==4 ? !bios_loaded : !cart_loaded))
                $fatal(1,"slot did not finish id=%0d busy=%b error=%0d",id,busy,error);
            if(committed_bytes !== size || received_bytes !== size || readback_crc32 !== input_crc32)
                $fatal(1,"counts/CRC mismatch size=%h received=%h committed=%h readback=%h input=%h",size,received_bytes,committed_bytes,readback_crc32,input_crc32);
            $display("VERIFIED %h %h",size,readback_crc32);
        end
    endtask
    integer i, size, old_ack_count;
    initial begin
        #2000000000; $fatal(1,"loader watchdog timeout");
    end
    initial begin
        reset_loader(); memory_initialized=0;
        repeat(5) @(negedge clk_bridge);
        dataslot_requestwrite=1; dataslot_requestwrite_id=1; dataslot_requestwrite_size='h8000;
        repeat(12) begin @(negedge clk_bridge); if(dataslot_ack) $fatal(1,"ack before memory init"); end
        memory_initialized=1;
        wait(dataslot_ack); @(negedge clk_bridge); dataslot_requestwrite=0;
        send_slot(1,'h8000,0);
        await_slot(1,'h8000);
        if(assets_ready) $fatal(1,"ready without BIOS");
        request_slot(4,'h40000,1); send_slot(4,'h40000,1);
        if(assets_ready) $fatal(1,"ready before readback verification");
        await_slot(4,'h40000);
        if(!assets_ready || bios_memory[0] !== 32'h01234567) $fatal(1,"ready/order/BIOS byte order");

        for(i=0;i<1;i=i+1) begin
            size='h8000;
            request_slot(1,size,1);
            if(cart_loaded || !bios_loaded || assets_ready) $fatal(1,"reload lifecycle");
            send_slot(1,size,1); await_slot(1,size);
            if(!assets_ready || rom_size!==size) $fatal(1,"reload readiness/size");
            if(cart_memory[(size=='h1c0000 ? 'h40000 : 0)>>2] !== pattern(0)) $fatal(1,"layout base");
            if(cart_memory[((size=='h1c0000 ? 'h40000 : 0)+size-4)>>2] !== pattern(size-4)) $fatal(1,"layout end");
        end
        reset_loader(); request_slot(4,'h40000,1); send_slot(4,'h40000,0); await_slot(4,'h40000);
        request_slot(1,'h8000,1); send_slot(1,'h8000,0);
        dataslot_allcomplete=1; @(negedge clk_bridge); dataslot_allcomplete=0;
        await_slot(1,'h8000); if(!assets_ready) $fatal(1,"allcomplete pulse lost");
        reset_loader(); request_slot(1,'h8000,1); push_word('h10000000,0,10);
        dataslot_allcomplete=1; repeat(100) @(negedge clk_bridge);
        if(assets_ready || cart_loaded || !busy || committed_bytes!=4) $fatal(1,"truncation released core");
        request_slot(4,'h40000,0);
        if(error!=8) $fatal(1,"abandoned transfer did not terminate");
        reset_loader(); request_slot(9,'h8000,0); if(error!=1) $fatal(1,"unknown slot error");
        reset_loader(); request_slot(1,'h8004,0); if(error!=2) $fatal(1,"ROM size error");
        reset_loader(); request_slot(4,'h1000,0); if(error!=2) $fatal(1,"BIOS size error");
        reset_loader(); push_word('h10000000,0,1); if(error!=3) $fatal(1,"unexpected data error");
        reset_loader(); request_slot(1,'h8000,1); push_word('h10000004,0,1);
        if(error!=4) $fatal(1,"nonsequential write error");
        reset_loader(); request_slot(1,'h8000,1); push_word('h30000000,0,1);
        if(error!=4) $fatal(1,"wrong slot write error");
        reset_loader(); request_slot(1,'h8000,1);
        push_word('h10000000,0,1); push_word('h10000000,0,1);
        if(error!=4) $fatal(1,"duplicate write error");
        reset_loader(); request_slot(1,'h8000,1); stall=1;
        for(i=0;i<80;i=i+1) push_word('h10000000+4*i,i,0);
        if(error!=5 || fifo_high_water!=64 || assets_ready) $fatal(1,"FIFO overflow not closed");
        reset_loader(); request_slot(1,'h8000,1); memory_fault=1;
        repeat(10) @(negedge clk_bridge); if(error!=6) $fatal(1,"memory fault lost");
        reset_loader(); request_slot(1,'h8000,1); corrupt_read=1; send_slot(1,'h8000,1);
        wait(!busy); repeat(4) @(negedge clk_bridge);
        if(error!=7 || cart_loaded || assets_ready) $fatal(1,"CRC mismatch not closed");
        $display("PASS loader: CDC backpressure, readback CRC, lifecycle, malformed streams");
        $finish;
    end
endmodule
