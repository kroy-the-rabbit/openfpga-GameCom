`timescale 1ns/1ps
module gamecom_rtc_tb;
    reg bridge = 0, sys = 0;
    always #7 bridge = ~bridge;
    always #25 sys = ~sys;
    reg cold=1, run=1, valid=0;
    reg [31:0] date_bcd=0,time_bcd=0;
    wire [64:0] rtc;
    reg cpu_toggle=0;
    reg [47:0] cpu_stamp=0;
    gamecom_rtc #(.CLOCK_HZ(80)) dut(.clk_bridge(bridge),.clk_sys(sys),.reset_cold(cold),
        .reset_run(run),.rtc_valid(valid),.rtc_date_bcd(date_bcd),.rtc_time_bcd(time_bcd),.rtc_bus(rtc));
    always @(posedge sys) begin
        if (cold || run) begin cpu_toggle <= 0; cpu_stamp <= 0; end
        else if (rtc[64] != cpu_toggle) begin
            cpu_toggle <= rtc[64]; cpu_stamp <= rtc[47:0];
        end
    end
    task tick(input integer n); repeat(n) begin @(posedge sys); #1; end endtask
    task send(input [31:0] d,t);
        @(negedge bridge);date_bcd=d;time_bcd=t;valid=1;
        @(negedge bridge);valid=0;
    endtask
    initial begin
        tick(3); cold=0;tick(3);
        if(rtc[47:0] !== 48'h00_01_01_00_00_00 || rtc[64]) $fatal(1,"fallback/reset");
        send(32'h2024_0228,32'h0323_5959);tick(14);
        if(rtc[47:0] !== 48'h24_02_28_23_59_59) $fatal(1,"BCD crossing %h",rtc);
        run=0;tick(12);if(!rtc[64]) $fatal(1,"initial publish");
        if(cpu_stamp !== 48'h24_02_28_23_59_59) $fatal(1,"CPU initial seed");
        wait(rtc[47:0] == 48'h24_02_29_00_00_00);
        run=1;tick(3);if(rtc[64]) $fatal(1,"warm reset toggle");
        run=0;tick(12);if(!rtc[64] || rtc[47:0] != 48'h24_02_29_00_00_00) $fatal(1,"warm RTC reseed");
        send(32'h2023_0228,32'h0223_5959);tick(14);
        if(cpu_stamp !== 48'h23_02_28_23_59_59 || rtc[64]) $fatal(1,"late host update not published");
        wait(rtc[47:0] == 48'h23_03_01_00_00_00);
        tick(2);if(cpu_stamp !== 48'h23_02_28_23_59_59) $fatal(1,"calendar tick reseeded CPU");
        send(32'h2099_1231,32'h0423_5959);tick(14);
        if(cpu_stamp !== 48'h99_12_31_23_59_59 || !rtc[64]) $fatal(1,"second live host update not published");
        wait(rtc[47:0] == 48'h00_01_01_00_00_00);
        send(32'h2026_0911,32'h0511_2233);
        send(32'h2026_0912,32'h0612_3456);tick(25);
        if(rtc[47:0] != 48'h26_09_12_12_34_56) $fatal(1,"latest mailbox update lost %h",rtc);
        if(cpu_stamp !== 48'h26_09_12_12_34_56) $fatal(1,"latest host update not delivered to CPU");
        $display("PASS RTC: coherent mailbox, late host publication, warm reset, leap/nonleap dates, year rollover");$finish;
    end
    initial begin #1000000; $fatal(1,"RTC timeout"); end
endmodule
