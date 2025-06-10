`timescale 1ns / 1ps

module motor_stepper(
    input clk,
    input rst,
    input en,
    input dir,
    output a, //15
    output b,
    output c,
    output d,
    input sensor_down,
    input sensor_up,
    output out_up,
    output  out_down
);
// 48, 49 en, dir
    assign out_down = sensor_down;
    assign out_up = sensor_up;

    wire new_clk_net;
    wire [3:0] signal_out;

    clk_div clock_diver(
        .clk(clk),
        .rst(rst),
        .new_clk(new_clk_net)
    );

    step_driver control(
        .rst(rst),
        .dir(dir),
        .clk(new_clk_net),
        .en(en),
        .signal(signal_out)
    );

    assign {a, b, c, d} = signal_out;

endmodule