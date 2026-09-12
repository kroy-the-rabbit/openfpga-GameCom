`timescale 1ns/1ps
module gamecom_machine_tb;
    reg sys=0,vid=0,bridge=0;
    always #25 sys=~sys;
    always #16.666667 vid=~vid;
    always #7 bridge=~bridge;
    reg cold=1,run=1;
    wire initialized;
    wire signed [15:0] audio;
    gamecom_machine dut(.clk_sys(sys),.clk_vid(vid),.clk_bridge(bridge),.reset_cold(cold),.reset_run(run),
        .cont1_key(32'd0),.power_pulse(1'b0),.rtc_valid(1'b0),.rtc_date_bcd(32'd0),.rtc_time_bcd(32'd0),
        .cart_data(8'hFF),.rom_ready(1'b1),.ram_initialized(initialized),.audio_sample(audio));
    task tick(input integer n); repeat(n) begin @(posedge sys); #1; end endtask
    integer i;
    initial begin
        tick(4);cold=0;tick(8191);
        if(initialized) $fatal(1,"RAM released before final clear write");
        tick(1);if(!initialized) $fatal(1,"RAM initialization did not finish");
        for(i=0;i<8192;i=i+1) if(dut.console_ram.mem_q[i] !== 0) $fatal(1,"cold RAM at %d",i);
        if(audio !== 0) $fatal(1,"reset audio not silent");
        run=0;tick(4);
        force dut.audio_unsigned=16'h8000;#1;if(audio !== 0) $fatal(1,"audio centre");
        force dut.audio_unsigned=16'h0000;#1;if(audio !== -32768) $fatal(1,"audio minimum");
        force dut.audio_unsigned=16'hFFFF;#1;if(audio !== 32767) $fatal(1,"audio maximum");
        release dut.audio_unsigned;
        force dut.save_addr=13'd17;force dut.save_dout=8'hA5;force dut.save_wren=1'b1;
        tick(2);release dut.save_addr;release dut.save_dout;release dut.save_wren;
        run=1;tick(4);run=0;tick(16);
        if(!initialized || dut.console_ram.mem_q[17] !== 8'hA5) $fatal(1,"warm reset lost RAM");
        for(i=0;i<6;i=i+1)
            if(dut.console_ram.mem_q['h1EE2+i] !== dut.machine.warm_boot_tuple_data(i[2:0]))
                $fatal(1,"warm BIOS tuple at %d",i);
        cold=1;tick(3);cold=0;tick(8192);
        if(dut.console_ram.mem_q[17] !== 0) $fatal(1,"cold reset retained RAM");
        $display("PASS machine: full 8KiB cold clear, warm retention/BIOS tuple, signed audio endpoints");$finish;
    end
    initial begin #2000000;$fatal(1,"machine test timeout");end
endmodule
