`timescale 1ns/1ps
module gamecom_video_tb;
    reg sys=0,vid=0;
    always #25 sys=~sys;
    always #16.666667 vid=~vid;
    reg reset=1;
    reg [1:0] phi=0;
    always @(posedge sys) if(reset) phi<=0; else phi<=phi+1'b1;
    wire ce,hb,vb,hs,vs;
    wire [2:0] shade;
    wire [23:0] rgb;
    wire out_hs,out_vs,de,skip;
    gamecom_video #(.VIDEO_CLOCK_DIV(5),.STOP_LOGO_FILE("../gamecom/rtl/gamecom_stop_logo.hex")) dut(
        .clk_sys_i(sys),.ce_pix_i(phi==0),.clk_vid_i(vid),.reset_i(reset),
        .display_enable_i(1'b1),.display_page_req_i(1'b0),.display_palette_i(2'b00),
        .display_normal_black_i(1'b0),.palette_four_color_i(1'b0),.video_60hz_i(1'b1),
        .stop_mode_i(1'b0),.cursor_enable_i(1'b0),.cursor_x_i(4'd0),.cursor_y_i(4'd0),
        .vram0_din_i(8'h1B),.vram1_din_i(8'hE4),.vram_addr_o(),.ce_pix_o(ce),
        .hblank_o(hb),.hsync_o(hs),.vblank_o(vb),.native_vblank_o(),.vsync_o(vs),.shade_o(shade));
    gamecom_video_adapter adapter(.clk_vid(vid),.reset(reset),.ce_pix(ce),.hblank(hb),
        .vblank(vb),.hsync(hs),.vsync(vs),.shade(shade),.video_rgb(rgb),
        .video_hs(out_hs),.video_vs(out_vs),.video_de(de),.video_skip(skip));
    integer frames=0,pixels=0,lines=0,dots=0,cycles=0,prev_accept=0;
    integer native_captures=0,native_pixels=0;
    always @(posedge sys) if(!reset && phi==0) native_pixels=native_pixels+1;
    reg previous_hs=0,previous_vs=0;
    always @(posedge vid) begin
        #1;
        if(!reset) begin
            cycles=cycles+1;
            if((out_hs && previous_hs) || (out_vs && previous_vs)) $fatal(1,"sync pulse longer than one clock");
            if((out_hs || out_vs) && skip) $fatal(1,"sync on skipped clock");
            previous_hs=out_hs;previous_vs=out_vs;
            if(!skip) begin
                if(cycles>20 && prev_accept!=0 && cycles-prev_accept!=5) $fatal(1,"pixel spacing %d",cycles-prev_accept);
                prev_accept=cycles;
                if(out_vs) begin
                    if(frames>0 && (pixels!=32000 || lines!=262 || dots!=381*262))
                        $fatal(1,"raster p=%d l=%d d=%d",pixels,lines,dots);
                    if(frames==3) begin
                        if(!dut.fb_display_valid_q || native_pixels-native_captures>2 || native_captures>native_pixels)
                            $fatal(1,"native 20/30MHz capture lost pulses %d/%d",native_captures,native_pixels);
                        $display("PASS video: 200x160, 381x262 at6MHz, one-cycle sync, 30MHz capture/display");$finish;
                    end
                    frames=frames+1;pixels=0;lines=0;dots=0;
                end
                dots=dots+1;
                if(out_hs) lines=lines+1;
                if(de) pixels=pixels+1;
            end
            if(dut.ce_native_w) native_captures=native_captures+1;
        end
    end
    initial begin repeat(8) @(posedge sys);#1;reset=0; end
    initial begin #100000000; $fatal(1,"video timeout");end
endmodule
