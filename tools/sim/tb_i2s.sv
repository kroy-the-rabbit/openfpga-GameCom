`timescale 1ns/1ps
module tb_i2s;
    reg clk_sys=0, clk_audio=0, reset_sys=1, reset_audio=1;
    always #25 clk_sys=~clk_sys;
    always #162.760 clk_audio=~clk_audio;
    reg signed [15:0] sample=0;
    wire lrck,dac;
    gamecom_i2s dut(.*,.audio_lrck(lrck),.audio_dac(dac));
    reg previous_lrck=0;
    reg [15:0] decoded=0, left=0;
    integer bit_number=0, frames=0, checks=0;
    always @(negedge clk_audio) if (!reset_audio) begin
        if (lrck != previous_lrck) begin
            previous_lrck=lrck;
            bit_number=0;
            decoded=0;
        end else begin
            bit_number=bit_number+1;
            if (bit_number<=16) decoded={decoded[14:0],dac};
            if (bit_number==16) begin
                if (!lrck) left=decoded;
                else begin
                    if (decoded !== left) $fatal(1,"Stereo frame tears: %h %h",left,decoded);
                    if (frames>3 && decoded !== sample) $fatal(1,"I2S word/encoding: got%h expected%h",decoded,sample);
                    checks=checks+1;
                    frames=frames+1;
                end
            end
            if (bit_number>16 && dac!==0) $fatal(1,"Nonzero I2S padding");
        end
    end
    task value(input [15:0] v);
        begin
            @(negedge clk_sys); sample=v; frames=0;
            wait(frames==8);
        end
    endtask
    initial begin
        #2000; reset_sys=0; reset_audio=0;
        value(16'h0000); value(16'h8000); value(16'h7fff); value(16'h1234); value(16'hffff);
        $display("PASS signed I2S: %0d coherent stereo frames, 48kHz framing and zero padding",checks);
        $finish;
    end
    initial begin #2000000; $fatal(1,"audio timeout"); end
endmodule
