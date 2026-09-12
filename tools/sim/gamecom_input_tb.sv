`timescale 1ns/1ps
module gamecom_input_tb;
    reg clk = 0;
    always #25 clk = ~clk;
    reg reset = 1, power = 0;
    reg [31:0] keys = 0;
    wire [11:0] buttons;
    wire touch, cursor;
    wire [3:0] x,y;
    gamecom_pocket_input #(.REPEAT_DELAY(5),.REPEAT_PERIOD(2),.POWER_HOLD(8)) dut(
        .clk_sys(clk),.reset(reset),.cont1_key(keys),.power_pulse(power),
        .buttons(buttons),.touch_active(touch),.cursor_enable(cursor),.touch_x(x),.touch_y(y));
    task tick(input integer n);
        repeat(n) begin @(posedge clk); #1; end
    endtask
    initial begin
        tick(3); reset=0; tick(3);
        keys=32'h0000_C1FF; tick(4);
        if(buttons !== 12'h7FF || touch || cursor) $fatal(1,"normal mapping %h",buttons);
        keys=32'h0000_0218; tick(50);
        if(x != 12 || buttons[6] || |buttons[3:0] || !touch || !cursor) $fatal(1,"touch/right bound");
        keys=32'h0000_0216; tick(50);
        if(x != 0 || y != 9) $fatal(1,"touch left/down bound %d %d",x,y);
        keys=32'h0000_0201; tick(50);
        if(y != 0) $fatal(1,"touch upper bound");
        keys=32'h0000_020F; tick(20);
        if(x != 0 || y != 0) $fatal(1,"opposing directions must cancel");
        keys=0; tick(4);
        power=1; tick(1); power=0; tick(5);
        if(!buttons[11]) $fatal(1,"power pulse was not stretched");
        tick(4); if(buttons[11]) $fatal(1,"power pulse stuck");
        $display("PASS input: button mapping, 13x10 bounds, modifier masking, repeat, power pulse");
        $finish;
    end
endmodule
