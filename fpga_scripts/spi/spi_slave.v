module spi_slave(clk, rst_n, SCK, MOSI, MISO, CS, data_received);
    input clk;              // System clock
    input rst_n;            // Active low reset
    input SCK;              // Serial clock from master
    input MOSI;             // Master Out Slave In
    output MISO;            // Master In Slave Out
    input CS;               // Chip Select (active low)
    output data_received;   // Flag to indicate data received

    reg [2:0] spi_bit_cnt;  // register holds bit count
    reg [7:0] spi_buffer;   // Shift register to hold spi data
    reg MISOr = 1'bx;
    reg data_avail = 1'b0;

    //Domain synchronization
    //Use three-stage shift registers for SCK and CS
    //Use two-stage shift register for MOSI

    reg [1:0] SCK_r;  always @(posedge clk) begin SCK_r <= { SCK_r[0], SCK }; end
	reg [2:0] CS_r;    always @(posedge clk) begin CS_r   <= {   CS_r[1:0],   CS }; end
	reg [1:0] MOSI_r;  always @(posedge clk) begin MOSI_r <= {   MOSI_r[0], MOSI }; end
	wire SCK_rising  = ( SCK_r[1:0] == 2'b01 );
	wire SCK_falling = ( SCK_r[1:0] == 2'b10 );
	wire CS_falling   = ( CS_r[2:1] == 2'b10 );
	wire CS_active    = ~CS_r[1];   // synchronous version of ~CS input
	wire MOSI_sync    = MOSI_r[1];  // synchronous version of MOSI input

    // Next state logic
    wire [7:0] spi_buffer_next = {spi_buffer[6:0], MOSI_sync};

    // spi transaction
	always @(posedge clk or posedge rst_n) begin
		if( rst_n ) begin
			spi_bit_cnt <= 3'd0;
        end
		else if ( CS_active ) begin
			if ( CS_falling )  begin // begin of message
				spi_bit_cnt <= 3'd0;
            end
            if ( SCK_rising )  begin// bit available
				spi_bit_cnt <= spi_bit_cnt + 3'd1;
            end
		end
    end

	// output logic
	assign MISO = CS_active ? MISOr : 1'bz;  // send MSB first

	always @(posedge clk or posedge rst_n) begin
		if( rst_n ) begin
				data_avail <= 1'b0;
			end
		else begin
			data_avail <= 1'b0;
			if ( CS_active ) begin
				if ( SCK_rising )  begin// input on rising PCI clock edge
					spi_buffer <= spi_buffer_next;
					if ( spi_bit_cnt == 3'd7 ) begin
						spi_buffer <= spi_buffer_next;
						data_avail <= 1'b1;
					end
            	end
				if ( SCK_falling )  begin // output on falling PCI clock edge
					MISOr <= spi_buffer[7];
            	end
			end
        end
	end

	assign data_received = data_avail;
endmodule
