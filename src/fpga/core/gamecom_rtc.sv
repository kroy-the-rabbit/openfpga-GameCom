module gamecom_rtc #(
    parameter integer CLOCK_HZ = 20000000
) (
    input wire clk_bridge,
    input wire clk_sys,
    input wire reset_cold,
    input wire reset_run,
    input wire rtc_valid,
    input wire [31:0] rtc_date_bcd,
    input wire [31:0] rtc_time_bcd,
    output wire [64:0] rtc_bus
);
    reg [47:0] pending_data, mailbox_data;
    reg pending_valid, request_toggle;
    reg acknowledge_toggle;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg ack_meta, ack_sync;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg request_meta, request_sync;
    reg [47:0] host_time;
    reg published;
    reg [3:0] release_count;
    reg [24:0] second_count;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [1:0] bridge_reset_pipe;
    wire reset_bridge = bridge_reset_pipe[1];
    always @(posedge clk_bridge or posedge reset_cold) begin
        if (reset_cold) bridge_reset_pipe <= 2'b11;
        else bridge_reset_pipe <= {bridge_reset_pipe[0], 1'b0};
    end

    function [7:0] bcd_next;
        input [7:0] value;
        begin
            bcd_next = value[3:0] == 9 ? {value[7:4] + 4'd1, 4'd0} : value + 1'b1;
        end
    endfunction
    function [7:0] month_days;
        input [7:0] year, month;
        reg leap;
        begin
            leap = ({year[4], 1'b0} + year[1:0]) == 2'd0;
            case (month)
                8'h02: month_days = leap ? 8'h29 : 8'h28;
                8'h04, 8'h06, 8'h09, 8'h11: month_days = 8'h30;
                default: month_days = 8'h31;
            endcase
        end
    endfunction

    function [47:0] next_second;
        input [47:0] stamp;
        reg [7:0] year, month, day, hour, minute, second;
        begin
            {year, month, day, hour, minute, second} = stamp;
            if (second != 8'h59) second = bcd_next(second);
            else begin
                second = 0;
                if (minute != 8'h59) minute = bcd_next(minute);
                else begin
                    minute = 0;
                    if (hour != 8'h23) hour = bcd_next(hour);
                    else begin
                        hour = 0;
                        if (day != month_days(year, month)) day = bcd_next(day);
                        else begin
                            day = 8'h01;
                            if (month != 8'h12) month = bcd_next(month);
                            else begin
                                month = 8'h01;
                                year = year == 8'h99 ? 8'h00 : bcd_next(year);
                            end
                        end
                    end
                end
            end
            next_second = {year, month, day, hour, minute, second};
        end
    endfunction

    always @(posedge clk_bridge or posedge reset_bridge) begin
        if (reset_bridge) begin
            pending_data <= 0;
            mailbox_data <= 0;
            pending_valid <= 0;
            request_toggle <= 0;
            ack_meta <= 0;
            ack_sync <= 0;
        end else begin
            ack_meta <= acknowledge_toggle;
            ack_sync <= ack_meta;
            if (pending_valid && (request_toggle == ack_sync)) begin
                mailbox_data <= pending_data;
                request_toggle <= ~request_toggle;
                pending_valid <= 0;
            end
            if (rtc_valid) begin
                pending_data <= {rtc_date_bcd[23:0], rtc_time_bcd[23:0]};
                pending_valid <= 1;
            end
        end
    end

    always @(posedge clk_sys or posedge reset_cold) begin
        if (reset_cold) begin
            request_meta <= 0;
            request_sync <= 0;
            acknowledge_toggle <= 0;
            host_time <= 48'h00_01_01_00_00_00; // deterministic 2000-01-01 fallback
            published <= 0;
            release_count <= 0;
            second_count <= 0;
        end else begin
            request_meta <= request_toggle;
            request_sync <= request_meta;
            if (request_sync != acknowledge_toggle) begin
                host_time <= mailbox_data;
                acknowledge_toggle <= request_sync;
                second_count <= 0;
            end else if (second_count == CLOCK_HZ[24:0] - 25'd1) begin
                host_time <= next_second(host_time);
                second_count <= 0;
            end else second_count <= second_count + 1'b1;
            if (reset_run) begin
                published <= 0;
                release_count <= 0;
            end else if (release_count != 9) begin
                if (release_count == 8) published <= 1;
                release_count <= release_count + 1'b1;
            end else if (request_sync != acknowledge_toggle) begin
                published <= ~published;
            end
        end
    end
    assign rtc_bus = {published, 16'd0, host_time};
endmodule
