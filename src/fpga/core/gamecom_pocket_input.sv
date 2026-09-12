module gamecom_pocket_input #(
    parameter integer REPEAT_DELAY = 6000000, // 300 ms at 20 MHz
    parameter integer REPEAT_PERIOD = 1600000, // 80 ms
    parameter integer POWER_HOLD = 400000 // 20 ms: survives matrix scan cadence
) (
    input wire clk_sys,
    input wire reset,
    input wire [31:0] cont1_key,
    input wire power_pulse,
    output wire [11:0] buttons,
    output wire touch_active,
    output wire cursor_enable,
    output reg [3:0] touch_x,
    output reg [3:0] touch_y
);
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [31:0] key_meta, key_sync;
    reg [3:0] direction_prev;
    reg modifier_prev;
    reg [23:0] repeat_count;
    reg [18:0] power_count;
    wire modifier = key_sync[9];
    wire [3:0] direction = modifier ? key_sync[3:0] : 4'd0;
    wire move = (direction != 0) &&
                ((direction != direction_prev) || !modifier_prev || repeat_count == 0);

    assign buttons = {(power_pulse || power_count != 0), key_sync[15], key_sync[7], key_sync[6],
                      key_sync[5], (key_sync[4] && !modifier), key_sync[8],
                      key_sync[14], (key_sync[3:0] & {4{!modifier}})};
    assign touch_active = modifier && key_sync[4];
    assign cursor_enable = modifier;

    always @(posedge clk_sys) begin
        if (reset) begin
            key_meta <= 0;
            key_sync <= 0;
            direction_prev <= 0;
            modifier_prev <= 0;
            repeat_count <= 0;
            power_count <= 0;
            touch_x <= 4'd6;
            touch_y <= 4'd4;
        end else begin
            key_meta <= cont1_key;
            key_sync <= key_meta;
            if (power_pulse) power_count <= POWER_HOLD[18:0] - 19'd1;
            else if (power_count != 0) power_count <= power_count - 1'b1;
            direction_prev <= direction;
            modifier_prev <= modifier;
            if (direction == 0) repeat_count <= 0;
            else if (move) begin
                repeat_count <= ((direction != direction_prev) || !modifier_prev) ?
                                REPEAT_DELAY[23:0] - 24'd1 : REPEAT_PERIOD[23:0] - 24'd1;
                if (direction[2] && !direction[3] && touch_x != 0) touch_x <= touch_x - 1'b1;
                if (direction[3] && !direction[2] && touch_x != 12) touch_x <= touch_x + 1'b1;
                if (direction[0] && !direction[1] && touch_y != 0) touch_y <= touch_y - 1'b1;
                if (direction[1] && !direction[0] && touch_y != 9) touch_y <= touch_y + 1'b1;
            end else repeat_count <= repeat_count - 1'b1;
        end
    end
endmodule
