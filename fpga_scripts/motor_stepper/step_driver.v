`timescale 1ns / 1ps

module step_driver(
        input rst,
        input dir,
        input clk,
        input en,
        output reg [3:0] signal
    );

        localparam sig1 = 4'b0001;
        localparam sig2 = 4'b0010;
        localparam sig3 = 4'b0100;
        localparam sig4 = 4'b1000;

        reg [3:0] state, next_state;


        always @ (*) begin
            case(state)
            sig4:
            begin
                if (dir == 1'b0 && en == 1'b1)
                    next_state = sig3;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig1;
                else
                    next_state = sig1;
            end
            sig3:
            begin
                if (dir == 1'b0 && en == 1'b1)
                    next_state = sig2;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig4;
                else
                    next_state = sig1;
            end
            sig2:
            begin
                if (dir == 1'b0 && en == 1'b1)
                    next_state = sig1;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig3;
                else
                    next_state = sig1;
            end
            sig1:
            begin
                if (dir == 1'b0 && en == 1'b1)
                    next_state = sig4;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig2;
                else
                    next_state = sig1;
            end
            default: next_state = sig1;
            endcase
        end




        always @ (posedge clk or posedge rst) //
        begin
            if (rst)
                state <= sig1;
            else
                state <= next_state;
        end

        always @ (posedge clk) //, posedge rst
        begin
            case(state)
                sig4: signal <= 4'b1001;
                sig3: signal = 4'b1100;
                sig2: signal = 4'b0110;
                sig1: signal = 4'b0011;
                default: signal = 4'b0000;
            endcase
        end
            
endmodule
            