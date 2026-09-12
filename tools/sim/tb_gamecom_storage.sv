`timescale 1ns/1ps
module tb_gamecom_storage #(
    parameter real MEMORY_PERIOD_NS=16.650,
    parameter real SYSTEM_PERIOD_NS=49.950,
    parameter real SDRAM_PHASE_NS=8.325
);
    reg clk_bridge=0,clk_mem=0,clk_sys=0,clk_sdram=0;
    always #6.734 clk_bridge=~clk_bridge;
    always #(MEMORY_PERIOD_NS/2.0) clk_mem=~clk_mem;
    always #(SYSTEM_PERIOD_NS/2.0) clk_sys=~clk_sys;
    initial begin #(SDRAM_PHASE_NS); forever #(MEMORY_PERIOD_NS/2.0) clk_sdram=~clk_sdram; end
    reg reset_bridge=1,reset_mem=1;
    reg bridge_wr=0;
    reg [31:0] bridge_addr=0,bridge_wr_data=0;
    reg dataslot_requestwrite=0,dataslot_allcomplete=0;
    reg [15:0] dataslot_requestwrite_id=0;
    reg [31:0] dataslot_requestwrite_size=0;
    wire dataslot_ack,dataslot_ok,dataslot_reject;
    wire load_valid,load_ready,load_bios,load_read,load_done;
    wire [20:0] load_addr;
    wire [31:0] load_data,load_rdata;
    wire initialized,fault,assets_ready,bios_loaded,cart_loaded,busy;
    wire [21:0] rom_size;
    wire [7:0] error;
    wire [31:0] announced_bytes,received_bytes,committed_bytes,readback_crc32,input_crc32;
    wire [6:0] fifo_high_water;
    wire [3:0] phase;
    wire reset_sys=reset_mem || !assets_ready;
    reg [20:0] cart_addr=0;
    reg cart_rd=0,slot1_sel=1,slot2_sel=0;
    wire cart_present=cart_loaded;
    wire [7:0] cart_data;
    wire rom_read_ready;
    wire [12:0] dram_a;
    wire [1:0] dram_ba,dram_dqm;
    wire [15:0] dram_dq,sram_dq;
    wire dram_clk,dram_cke,dram_ras_n,dram_cas_n,dram_we_n;
    wire [16:0] sram_a;
    wire sram_oe_n,sram_we_n,sram_ub_n,sram_lb_n;
    gamecom_rom_loader loader(.memory_initialized(initialized),.memory_fault(fault),.*);
    gamecom_memory memory(.*);
    gamecom_sram_model sram(sram_a,sram_dq,sram_oe_n,sram_we_n,sram_ub_n,sram_lb_n);
    gamecom_sram_turnaround_check ownership(memory.sram_drive,sram_oe_n,sram_we_n);
    gamecom_sdram_model #(.CLOCK_PERIOD_NS(MEMORY_PERIOD_NS)) sdram(dram_clk,dram_cke,dram_ras_n,dram_cas_n,dram_we_n,
        dram_a,dram_ba,dram_dqm,dram_dq);
    reg [31:0] bios_words[0:65535];
    reg [31:0] rom_words[0:524287];
    reg [31:0] size,expected_rom_crc,expected_bios_crc;
    reg [1023:0] rom_path,bios_path;
    integer i,ticks;
    reg last_write_seen=0;

    always @(posedge clk_bridge) if(!reset_bridge) begin
        if(error!=0) $fatal(1,"Storage loader error=%0d received=%h committed=%h phase=%h",error,received_bytes,committed_bytes,phase);
        if(assets_ready && !last_write_seen) $fatal(1,"CPU released before final host word");
    end
    task announce(input [15:0] id,input [31:0] bytes);
        begin
            @(negedge clk_bridge);dataslot_allcomplete=0;
            dataslot_requestwrite_id=id;dataslot_requestwrite_size=bytes;dataslot_requestwrite=1;
            ticks=0;
            do begin @(negedge clk_bridge);ticks=ticks+1;
                if(ticks>20000) $fatal(1,"Slot announce timed out");
            end while(!dataslot_ack);
            if(!dataslot_ok || dataslot_reject) $fatal(1,"Valid slot rejected");
            dataslot_requestwrite=0;
            repeat(4) @(negedge clk_bridge);
        end
    endtask
    task stream(input bit bios,input integer bytes);
        begin
            for(i=0;i<bytes/4;i=i+1) begin
                @(negedge clk_bridge);
                bridge_addr=(bios ? 32'h30000000 : 32'h10000000)+(i*4);
                bridge_wr_data=bios ? bios_words[i] : rom_words[i];
                bridge_wr=1;
                if(!bios && i==bytes/4-1) begin
                    dataslot_allcomplete=1;last_write_seen=1;
                end
                @(negedge clk_bridge);bridge_wr=0;
                repeat(73) @(negedge clk_bridge);
            end
            $display("Host transferred %s %0d bytes; committed=%0d FIFO high-water=%0d",bios ? "BIOS" : "ROM",bytes,committed_bytes,fifo_high_water);
            if(!bios && assets_ready) $fatal(1,"Readback CRC completed impossibly early");
            ticks=0;
            while(busy) begin @(negedge clk_bridge);ticks=ticks+1;
                if(ticks>20000000) $fatal(1,"Physical CRC completion timed out");
            end
            if(received_bytes!==bytes || committed_bytes!==bytes) $fatal(1,"Byte counts differ at completion");
            if(readback_crc32!==(bios ? expected_bios_crc : expected_rom_crc))
                $fatal(1,"Physical CRC got=%h want=%h",readback_crc32,bios ? expected_bios_crc : expected_rom_crc);
            if(input_crc32!==readback_crc32) $fatal(1,"Input and physical CRC differ");
        end
    endtask
    task cpu_check(input [20:0] address);
        reg [20:0] offset;
        reg [31:0] expected_word;
        reg [7:0] expected_byte;
        begin
            if(address<21'h40000) begin offset=address; expected_word=bios_words[offset>>2]; end
            else begin
                offset=size==32'h1c0000 ? address-21'h40000 : (address & (size-1));
                expected_word=rom_words[offset>>2];
            end
            expected_byte=expected_word >> (24-8*offset[1:0]);
            @(negedge clk_sys);cart_addr=address;cart_rd=1;
            #1;
            if(address<21'h40000) #199;
            else begin
                ticks=0;
                while(!rom_read_ready) begin @(negedge clk_sys);ticks=ticks+1;
                    if(ticks>100) $fatal(1,"CPU read timeout addr=%h",address);
                end
                #1;
            end
            if(cart_data!==expected_byte) $fatal(1,"CPU read addr=%h got=%h want=%h",address,cart_data,expected_byte);
        end
    endtask
    initial begin
        if(!$value$plusargs("ROM=%s",rom_path) || !$value$plusargs("BIOS=%s",bios_path) ||
           !$value$plusargs("SIZE=%h",size) || !$value$plusargs("ROMCRC=%h",expected_rom_crc) ||
           !$value$plusargs("BIOSCRC=%h",expected_bios_crc)) $fatal(1,"Missing fixture arguments");
        $readmemh(bios_path,bios_words,0,65535);
        $readmemh(rom_path,rom_words,0,size/4-1);
        repeat(10) @(negedge clk_bridge);reset_bridge=0;reset_mem=0;
        wait(initialized);repeat(10) @(negedge clk_bridge);
        announce(4,32'h40000);stream(1,32'h40000);
        if(!bios_loaded || assets_ready) $fatal(1,"BIOS-only lifecycle incorrect");
        $display("PASS BIOS physical CRC=%h",readback_crc32);
        announce(1,size);stream(0,size);
        if(!assets_ready || !cart_loaded || rom_size!==size) $fatal(1,"Assets did not become ready");
        repeat(20) @(negedge clk_sys);
        cpu_check(0);cpu_check(1);cpu_check(21'h3ffff);
        cpu_check(21'h40000);cpu_check(21'h40001);cpu_check(21'h47fff);
        cpu_check(21'h7ffff);cpu_check(21'hfffff);cpu_check(21'h1ffffc);cpu_check(21'h1fffff);
        $display("PASS storage integration size=%h physical CRC=%h FIFO high-water=%0d phase_ps=%0d period_ps=%0d",size,readback_crc32,fifo_high_water,$rtoi(SDRAM_PHASE_NS*1000),$rtoi(MEMORY_PERIOD_NS*1000));
        $finish;
    end
    initial begin #2000000000; $fatal(1,"Global storage simulation timeout"); end
endmodule
