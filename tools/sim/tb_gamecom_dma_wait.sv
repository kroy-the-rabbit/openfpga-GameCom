`timescale 1ns/1ps
module tb_gamecom_dma_wait;
    reg clk=0, resetb=0;
    always #25 clk=~clk;
    reg [1:0] divider=0;
    always @(posedge clk) if(!resetb) divider<=0; else divider<=divider+1'b1;
    wire phi0=divider==0,phi1=divider==2;
    reg ready=0;
    reg [7:0] din=8'hff;
    wire [20:0] address;
    wire [12:0] vaddress;
    wire [7:0] vdout;
    wire mce0b,mce1b,rdb,vce0b,vce1b,vrdb,vwrb,voe;
    sm8521 #(.RESET_WARMUP_CYCLES(18'd0)) dut (
        .clk_sys_i(clk),.phi0_ce_i(phi0),.phi1_ce_i(phi1),.resetb_i(resetb),
        .stop_disable_i(1'b1),.warm_boot_i(1'b0),.rom_read_ready_i(ready),
        .rtc_i(65'd0),.nmib_i(1'b1),.intb_i(1'b1),.power_stop_wake_i(1'b0),
        .m_i(3'd0),.d_din_i(din),.a_o(address),.mce0b_o(mce0b),
        .mce1b_o(mce1b),.rdb_o(rdb),.vd_din_i(8'h5a),.va_o(vaddress),
        .vd_dout_o(vdout),.vd_oe_o(voe),.vce0b_o(vce0b),.vce1b_o(vce1b),
        .vrdb_o(vrdb),.vwrb_o(vwrb),
        .p0_din_i(8'hff),.p1_din_i(8'hff),.p2_din_i(8'hff),.p3_din_i(8'hff),
        .rxdb_i(1'b1),.vr_i(1'b0),.savestate_pause_req_i(1'b0),
        .savestate_active_i(1'b0),.savestate_addr_i(12'd0),.savestate_rd_i(1'b0),
        .savestate_wr_i(1'b0),.savestate_wdata_i(8'd0),
        .cheat_clear_i(1'b1),.cheat_code_i(129'd0)
    );
    localparam READ_PATH=0, WINDOW_PATH=1, PREFETCH_PATH=2;
    localparam CART=0, BIOS=1, SRAM=2, VRAM=3;
    localparam READ_SETUP=37,READ_SAMPLE=38,WRITE_SETUP=41,WRITE_SAMPLE=42,
               WINDOW_SETUP=50,WINDOW_SAMPLE=51;
    integer cases=0,hold_beats=0;
    wire [115:0] dma_progress={dut.dma_src_addr_q,dut.dma_dst_addr_q,
        dut.dma_line_count_q,dut.dma_row_count_q,dut.dma_src_byte_q,
        dut.dma_src_byte_valid_q,dut.dma_src_next_byte_q,dut.dma_src_next_valid_q,
        dut.mem_byte_q,dut.dma_packet_byte_q,dut.dma_packet_pixels_q,
        dut.dma_src_phase_q,dut.dma_src_x_q,dut.dma_dst_x_q,
        dut.dma_src_y_q,dut.dma_dst_y_q,dut.dma_active_q};

    task beat;
        begin
            do @(negedge clk); while(divider!=0);
            @(posedge clk);#1;
        end
    endtask
    task seed(input integer path,input integer source_kind);
        begin
            @(negedge clk);resetb=0;ready=0;din=8'hff;
            repeat(8) @(negedge clk);
            resetb=1;
            wait(dut.gp_store_init_active_q==0);
            @(negedge clk);
            dut.idle_bus();
            dut.warmup_q=0;
            dut.dma_active_q=1;
            dut.dma_mode_q=source_kind==SRAM ? 2'b10 : source_kind==VRAM ? 2'b00 : 2'b01;
            dut.dma_ctl_q=source_kind==SRAM ? 8'h05 : source_kind==VRAM ? 8'h01 : 8'h03;
            dut.dmc_q=8'h83;
            dut.dma_dmpl_q=8'he4;
            dut.dma_dmbr_q=source_kind==BIOS ? 8'h0f : 8'h10;
            dut.dma_dmvp_q=8'h02;
            dut.mmu0_q=8'h20;
            dut.lcc_q=0;
            dut.lcdc_dma_div_q=0;
            dut.dma_src_addr_q=14'd3;
            dut.dma_src_line_q=14'd3;
            dut.dma_dst_addr_q=14'h120;
            dut.dma_dst_line_q=14'h120;
            dut.dma_src_phase_q=path==WINDOW_PATH ? 2'd2 : 2'd0;
            dut.dma_src_x_q=8'd12;
            dut.dma_src_y_q=0;
            dut.dma_dst_x_q=0;
            dut.dma_dst_y_q=0;
            dut.dma_line_count_q=7;
            dut.dma_row_count_q=0;
            dut.dma_arm_line_count_q=7;
            dut.dma_arm_src_x_q=12;
            dut.dma_src_byte_q=8'h6c;
            dut.dma_src_byte_valid_q=path!=READ_PATH;
            dut.dma_src_next_byte_q=8'h33;
            dut.dma_src_next_valid_q=0;
            dut.dma_packet_pixels_q=4;
            dut.dma_packet_byte_q=8'h12;
            dut.mem_byte_q=8'h12;
            dut.state_q=path==READ_PATH ? READ_SETUP : path==WINDOW_PATH ? WINDOW_SETUP : WRITE_SETUP;
        end
    endtask
    task run_case(input integer path,input integer source_kind);
        reg [5:0] expected_state;
        reg [20:0] held_address;
        reg [12:0] held_vaddress;
        reg [7:0] held_vdata;
        reg [5:0] held_vcontrol;
        reg [115:0] held_dma_progress;
        reg [7:0] expected_source,expected_packet;
        integer n;
        begin
            seed(path,source_kind);
            if(source_kind!=CART) din=8'ha5;
            beat(); // DUT sets up the ROM/SRAM/VRAM transaction itself.
            expected_state=path==READ_PATH ? READ_SAMPLE : path==WINDOW_PATH ? WINDOW_SAMPLE : WRITE_SAMPLE;
            if(dut.state_q!==expected_state) $fatal(1,"DMA setup did not reach sample path=%0d kind=%0d",path,source_kind);
            held_address=address;held_vaddress=vaddress;held_vdata=vdout;
            held_vcontrol={voe,vce0b,vce1b,vrdb,vwrb,rdb};
            held_dma_progress=dma_progress;
            if(source_kind==CART && (mce0b || rdb || address<21'h40000))
                $fatal(1,"DUT did not create a physical cartridge read");
            if(source_kind==BIOS && (mce0b || rdb || address>=21'h40000))
                $fatal(1,"DUT did not create a physical BIOS read");
            if(source_kind==SRAM && (!mce0b || mce1b || rdb))
                $fatal(1,"DUT did not create an SRAM-class read");
            if(path==PREFETCH_PATH && (!voe || vwrb || vdout!==8'h12 || vaddress!==13'h120))
                $fatal(1,"Prefetch did not pair the source read with the expected VRAM write");
            if(source_kind==CART) begin
                for(n=0;n<3;n=n+1) begin
                    beat();
                    if(dut.state_q!==expected_state || address!==held_address || rdb || mce0b)
                        $fatal(1,"DMA advanced/released ROM while READY=0 path=%0d state=%0d",path,dut.state_q);
                    if(dma_progress!==held_dma_progress)
                        $fatal(1,"DMA consumed poisoned data or advanced counters while waiting path=%0d",path);
                    if(path==PREFETCH_PATH && (vaddress!==held_vaddress || vdout!==held_vdata ||
                        {voe,vce0b,vce1b,vrdb,vwrb,rdb}!==held_vcontrol))
                        $fatal(1,"Stalled prefetch changed its outstanding VRAM write");
                    hold_beats=hold_beats+1;
                end
                @(negedge clk);ready=1;din=8'ha5;
            end
            beat();
            expected_source=source_kind==VRAM ? 8'h5a : 8'ha5;
            if(dut.state_q!==WRITE_SETUP) $fatal(1,"DMA did not resume/complete sample path=%0d kind=%0d state=%0d",path,source_kind,dut.state_q);
            if(path==WINDOW_PATH) begin
                expected_packet={4'hc,expected_source[7:4]};
                if(dut.dma_src_next_byte_q!==expected_source || !dut.dma_src_next_valid_q || dut.mem_byte_q!==expected_packet)
                    $fatal(1,"DMA window did not merge the ready neighboring byte");
            end else begin
                if(dut.dma_src_byte_q!==expected_source || !dut.dma_src_byte_valid_q || dut.mem_byte_q!==expected_source)
                    $fatal(1,"DMA consumed wrong ready source data path=%0d kind=%0d got=%h",path,source_kind,dut.mem_byte_q);
            end
            if(path==PREFETCH_PATH && (dut.dma_src_addr_q!==14'd4 || dut.dma_dst_addr_q!==14'h121 || dut.dma_line_count_q!==8'd3))
                $fatal(1,"DMA prefetch did not advance exactly once after READY");
            cases=cases+1;
            $display("PASS DMA path=%0d source=%0d",path,source_kind);
        end
    endtask
    integer path;
    initial begin
        for(path=0;path<3;path=path+1) run_case(path,CART);
        for(path=0;path<3;path=path+1) run_case(path,BIOS);
        for(path=0;path<3;path=path+1) run_case(path,SRAM);
        run_case(READ_PATH,VRAM);run_case(WINDOW_PATH,VRAM);
        $display("PASS DMA READY regression: %0d scenarios, %0d poisoned wait beats",cases,hold_beats);
        $finish;
    end
    initial begin #1000000;$fatal(1,"DMA regression timeout");end
endmodule
