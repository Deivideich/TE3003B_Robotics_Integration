`timescale 1ns / 1ps
//rotates servo to left or right according to the pressing of push buttons
module servo(
    input clk,
    input pb1,  
    input pb2,
    output servo);

//create a simple counter
reg [20:0] counter = 0;
reg next_pulse = 0;
reg [19:0] pulse_width = PULSE_NEUTRAL;
reg [1:0] option;

parameter PWM_PERIOD = 540_000;
parameter PULSE_LEFT = 27_000;
parameter PULSE_NEUTRAL = 40_500;
parameter PULSE_RIGHT = 54_000;
parameter S1 = 2'b10;
parameter S2 = 2'b01;


always @(posedge clk) begin
    if (counter < PWM_PERIOD) begin
        counter <= counter + 1;
    end else begin
        counter <= 0; //reset to 0
        next_pulse <= next_pulse + 1;
    end
    //27000(1ms)->left , 40500(1.5ms)->neutral , 54000(2ms)->right -- rotate servo by manually putting any of these values between the extremes

    
end

always @(posedge next_pulse) begin
    case({pb1, pb2})
        S1: pulse_width = PULSE_RIGHT;
        S2: pulse_width = PULSE_LEFT;
        default: pulse_width = PULSE_NEUTRAL;
    endcase
end


begin
    assign servo = (counter < pulse_width) ? 1 : 0;
end


endmodule