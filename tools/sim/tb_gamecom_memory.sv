`timescale 1ns/1ps
module tb_gamecom_memory #(
    parameter real SDRAM_ACCESS_NS=6.0,
    parameter real MEMORY_PERIOD_NS=16.650,
    parameter real SYSTEM_PERIOD_NS=49.950,
    parameter real SDRAM_PHASE_NS=8.325
);
    reg clk_mem=0,clk_sys=0,clk_sdram=0;
    always #(MEMORY_PERIOD_NS/2.0) clk_mem=~clk_mem;
    always #(SYSTEM_PERIOD_NS/2.0) clk_sys=~clk_sys;
    initial begin #(SDRAM_PHASE_NS); forever #(MEMORY_PERIOD_NS/2.0) clk_sdram=~clk_sdram; end
    reg reset_mem=1,reset_sys=1;
    reg load_valid=0,load_read=0,load_bios=0;
    reg [20:0] load_addr=0;
    reg [31:0] load_data=0;
    wire load_ready,load_done,initialized,fault;
    wire [31:0] load_rdata;
    reg [20:0] cart_addr=0;
    reg cart_rd=0,slot1_sel=0,slot2_sel=0,cart_present=0;
    reg [21:0] rom_size=22'h200000;
    wire [7:0] cart_data;
    wire rom_read_ready;
    wire [12:0] dram_a;
    wire [1:0] dram_ba,dram_dqm;
    wire [15:0] dram_dq,sram_dq;
    wire dram_clk,dram_cke,dram_ras_n,dram_cas_n,dram_we_n;
    wire [16:0] sram_a;
    wire sram_oe_n,sram_we_n,sram_ub_n,sram_lb_n;
    gamecom_memory dut(.*);
    gamecom_sram_model sram(sram_a,sram_dq,sram_oe_n,sram_we_n,sram_ub_n,sram_lb_n);
    gamecom_sram_turnaround_check ownership(dut.sram_drive,sram_oe_n,sram_we_n);
    gamecom_sdram_model #(.ACCESS_DELAY_NS(SDRAM_ACCESS_NS),.CLOCK_PERIOD_NS(MEMORY_PERIOD_NS)) sdram(dram_clk,dram_cke,dram_ras_n,dram_cas_n,dram_we_n,
        dram_a,dram_ba,dram_dqm,dram_dq);
    integer cycles,checks=0;
    reg [31:0] result;

    task transaction(input bit bios,input bit rd,input [20:0] addr,input [31:0] data);
        begin
            @(negedge clk_mem);
            load_bios=bios;load_read=rd;load_addr=addr;load_data=data;load_valid=1;
            cycles=0;
            do begin @(posedge clk_mem); cycles=cycles+1;
                if(cycles>1000) $fatal(1,"Memory acceptance timeout");
            end while(!load_ready);
            @(negedge clk_mem);load_valid=0; cycles=0;
            while(!load_done) begin @(negedge clk_mem);cycles=cycles+1;
                if(cycles>1000) $fatal(1,"Memory completion timeout");
            end
            result=load_rdata;
            if(fault) $fatal(1,"Unexpected memory fault");
        end
    endtask
    task check_word(input bit bios,input [20:0] addr,input [31:0] data);
        begin
            transaction(bios,0,addr,data);
            transaction(bios,1,addr,0);
            if(result!==data) $fatal(1,"Readback bios=%0d addr=%h got=%h want=%h",bios,addr,result,data);
            checks=checks+1;
        end
    endtask
    task cpu_byte(input [20:0] addr,input [7:0] data);
        begin
            @(negedge clk_sys); cart_addr=addr;cart_rd=1;
            #1;
            if(addr<21'h40000) begin
                if(!rom_read_ready) $fatal(1,"BIOS may not wait");
                #199;
            end else begin
                cycles=0;
                while(!rom_read_ready) begin @(negedge clk_sys);cycles=cycles+1;
                    if(cycles>100) $fatal(1,"CPU ROM response timeout");
                end
                #1;
            end
            if(cart_data!==data) $fatal(1,"CPU addr=%h size=%h got=%h want=%h",addr,rom_size,cart_data,data);
            checks=checks+1;
        end
    endtask
    task new_cart(input [21:0] size);
        begin
            @(negedge clk_sys);reset_sys=1;cart_rd=0;cart_present=0;rom_size=size;
            repeat(12) @(negedge clk_sys);
        end
    endtask
    task run_cart;
        begin
            @(negedge clk_sys);cart_present=1;slot1_sel=1;slot2_sel=0;reset_sys=0;
            repeat(12) @(negedge clk_sys);
        end
    endtask

    integer corpus,rc,bios_i;
    reg [20:0] fixture_addr,physical_addr;
    reg [21:0] fixture_size;
    reg [31:0] fixture_data;
    reg [1023:0] corpus_path;
    initial begin
        repeat(8) @(negedge clk_mem);reset_mem=0;
        wait(initialized);repeat(10) @(negedge clk_mem);
        check_word(1,0,32'h01234567);
        check_word(1,21'h3fffc,32'h89abcdef);
        run_cart;
        cpu_byte(0,8'h01);cpu_byte(1,8'h23);cpu_byte(2,8'h45);cpu_byte(3,8'h67);
        cpu_byte(21'h3ffff,8'hef);
        @(negedge clk_sys);cart_rd=0;cart_addr=0;
        #200;
        if(cart_data!==8'hef) $fatal(1,"BIOS address/data not held after RD release");
        checks=checks+1;

        new_cart(22'h8000);check_word(0,0,32'h10325476);run_cart;
        cpu_byte(21'h40000,8'h10);cpu_byte(21'h48001,8'h32);
        cpu_byte(21'h1f8002,8'h54);cpu_byte(0,8'h01); // BIOS wins before masking
        new_cart(22'h40000);check_word(0,21'h3fffc,32'h21436587);run_cart;
        cpu_byte(21'h7fffc,8'h21);cpu_byte(21'h1fffff,8'h87);
        new_cart(22'h80000);check_word(0,21'h40000,32'h32547698);run_cart;
        cpu_byte(21'h40000,8'h32);cpu_byte(21'hc0001,8'h54);
        new_cart(22'h100000);check_word(0,21'h40000,32'h436587a9);run_cart;
        cpu_byte(21'h40000,8'h43);cpu_byte(21'h140003,8'ha9);
        new_cart(22'h1c0000);
        check_word(0,21'h40000,32'h547698ba);check_word(0,21'h1ffffc,32'h6587a9cb);run_cart;
        cpu_byte(21'h40000,8'h54);cpu_byte(21'h1fffff,8'hcb);
        new_cart(22'h200000);
        check_word(0,0,32'hffffffff);check_word(0,21'h40000,32'h7698badc);
        check_word(0,21'h1ffffc,32'h87a9cbed);run_cart;
        cpu_byte(21'h40001,8'h98);cpu_byte(21'h1ffffe,8'hcb);cpu_byte(0,8'h01);
        slot1_sel=0;slot2_sel=1;cpu_byte(21'h40000,8'hff);
        slot1_sel=1;cpu_byte(21'h40000,8'h76);
        cart_present=0;cpu_byte(21'h40000,8'hff);

        if($value$plusargs("CORPUS=%s",corpus_path)) begin
            corpus=$fopen(corpus_path,"r");
            if(!corpus) $fatal(1,"Unable to open private corpus fixture");
            while(!$feof(corpus)) begin
                rc=$fscanf(corpus,"%d %h %h %h %h\n",bios_i,fixture_size,fixture_addr,physical_addr,fixture_data);
                if(rc==5) begin
                    new_cart(fixture_size);
                    check_word(bios_i!=0,fixture_addr,fixture_data);run_cart;
                    cpu_byte(physical_addr,fixture_data[31:24]);
                    cpu_byte(physical_addr+1,fixture_data[23:16]);
                    cpu_byte(physical_addr+2,fixture_data[15:8]);
                    cpu_byte(physical_addr+3,fixture_data[7:0]);
                end else if(rc!=-1) $fatal(1,"Malformed corpus fixture");
            end
            $fclose(corpus);
        end
        #100000;
        if(sdram.refreshes<10) $fatal(1,"Refresh not running while idle");
        new_cart(22'h200000);
        @(negedge clk_mem);load_valid=1;load_addr=21'h40001;load_read=0;load_bios=0;
        do @(posedge clk_mem); while(!load_ready);
        @(negedge clk_mem);load_valid=0;
        if(!fault || !load_done || load_ready) $fatal(1,"Misaligned backend transaction did not fail closed");
        repeat(20) @(negedge clk_mem);
        if(!fault || load_ready) $fatal(1,"Backend fault was not persistent");
        checks=checks+1;
        $display("PASS gamecom_memory tAC=%0.1f ns period=%0.3f ns phase=%0.3f ns: %0d checks; SDRAM reads=%0d writes=%0d refreshes=%0d",SDRAM_ACCESS_NS,MEMORY_PERIOD_NS,SDRAM_PHASE_NS,checks,sdram.reads,sdram.writes,sdram.refreshes);
        $finish;
    end
    initial begin #200000000; $fatal(1,"Global simulation timeout"); end
endmodule
