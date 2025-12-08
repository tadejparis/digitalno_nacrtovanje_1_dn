module baud_rate_generator #(
    parameter integer BAUD_DIV = 10416
)(
    input  logic clock,
    input  logic reset,
    output logic baud_tick
);

    localparam CNT_WIDTH = $clog2(BAUD_DIV);
    logic [CNT_WIDTH-1:0] cnt;

    always_ff @(posedge clock or posedge reset) begin
        if (reset)
            cnt <= 0;
        else if (cnt == BAUD_DIV-1)
            cnt <= 0;
        else
            cnt <= cnt + 1;
    end

    assign baud_tick = (cnt == BAUD_DIV-1);

endmodule


module uart_receiver #(
    parameter integer DBITS = 8,
    parameter integer SBITS = 1
)(
    input  logic clock,
    input  logic reset,
    input  logic rx,
    output logic [DBITS-1:0] data_out,
    output logic rx_done
);

    localparam integer BAUD_DIV  = 10416;
    localparam integer HALF_BAUD = BAUD_DIV / 2;
    localparam BAUD_CNT_WIDTH = $clog2(BAUD_DIV);

    typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;
    state_t state;

    logic [BAUD_CNT_WIDTH-1:0] baud_cnt;
    logic [$clog2(DBITS)-1:0] bit_cnt;
    logic [DBITS-1:0] shift_reg;

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            state     <= IDLE;
            baud_cnt  <= 0;
            bit_cnt   <= 0;
            shift_reg <= 0;
            data_out  <= 0;
            rx_done   <= 0;
        end else begin
            rx_done <= 1'b0;
            case (state)
                IDLE: begin
                    if (rx == 1'b0) begin
                        state    <= START;
                        baud_cnt <= 0;
                    end
                end
                START: begin
                    if (baud_cnt == HALF_BAUD-1) begin
                        if (rx == 1'b0) begin
                            state    <= DATA;
                            baud_cnt <= 0;
                            bit_cnt  <= 0;
                        end else begin
                            state <= IDLE;  // false start
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end
                DATA: begin
                    if (baud_cnt == BAUD_DIV-1) begin
                        baud_cnt <= 0;
                        shift_reg <= {rx, shift_reg[DBITS-1:1]}; // LSB-first
                        if (bit_cnt == DBITS-1)
                            state <= STOP;
                        else
                            bit_cnt <= bit_cnt + 1;
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end
                STOP: begin
                    if (baud_cnt == BAUD_DIV-1) begin
                        baud_cnt <= 0;
                        data_out <= shift_reg;
                        rx_done  <= 1'b1;
                        state    <= IDLE;
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule


module shift_reg_parallel_read #(
    parameter BITS = 8,
    parameter TIMES = 6
)(
    input  logic clock,
    input  logic reset,
    input  logic [BITS-1:0] data_in,
    input  logic rx_done,
    output logic [5:0] token
);

    logic [BITS*TIMES-1:0] shift_reg;
    logic [1:0] r_val, g_val, b_val;
    logic [BITS-1:0] bytes [0:TIMES-1];

    always_ff @(posedge clock or posedge reset) begin
        if (reset)
            shift_reg <= 0;
        else if (rx_done)
            shift_reg <= {shift_reg[BITS*(TIMES-1)-1:0], data_in};
    end

    always_comb begin
        for (int i = 0; i < TIMES; i++) begin
            bytes[i] = shift_reg[BITS*(TIMES-i)-1 -: BITS];
        end

        r_val = 2'b00;
        g_val = 2'b00;
        b_val = 2'b00;

        if (bytes[0] == 8'h52 && (bytes[1] >= 8'b00110000 && bytes[5] <= 8'b00110011)) r_val = (bytes[1] - 8'b00110000); // 'R'
        if (bytes[2] == 8'h47 && (bytes[3] >= 8'b00110000 && bytes[5] <= 8'b00110011)) g_val = (bytes[3] - 8'b00110000); // 'G'
        if (bytes[4] == 8'h42 && (bytes[5] >= 8'b00110000 && bytes[5] <= 8'b00110011)) b_val = (bytes[5] - 8'b00110000); // 'B'

        token = {r_val, g_val, b_val};
    end

endmodule

module pwm_controller 
# (
    parameter RESOLUTION = 4
)    
(
    input logic clock,
    input logic reset,
    input logic [1:0] SW,
    input logic clock_enable,
    output logic PWM
);

    // define the parameters
    localparam DT_75 = 1 << (RESOLUTION-1);
    localparam DT_50 = 1 << (RESOLUTION-2);
    localparam DT_25 = 1 << (RESOLUTION-3);

    // resolution of the PWM
    logic [RESOLUTION-1:0] count;
    logic [RESOLUTION:0] duty_cycle;
    
		always_comb begin : DutyCycleCalculate
        unique case (SW)
            2'b00: duty_cycle = 0;
            2'b01: duty_cycle = DT_25;
            2'b10: duty_cycle = DT_50;
            2'b11: duty_cycle = DT_75;
            default: duty_cycle = 0;
        endcase
    end

    always_ff @(posedge clock) begin 
        if (reset)
            count <= 0;
        else if (clock_enable)
            count <= count + 1;
    end
		assign PWM = (count < duty_cycle);   
endmodule

module rgb_controller (
    input logic clock,
    input logic reset_pwm,
    input logic [5:0] SW,
    input logic ctrl,
    output logic PWM_R,
    output logic PWM_G,
    output logic PWM_B
);
		// define parameters
		localparam PRESCALER_WIDTH =  12;
		localparam LIMIT = 3125;
		
		// define the limit_value
		logic [PRESCALER_WIDTH-1:0] limit_value;
		assign limit_value = LIMIT;
		
		// instantiate the prescaler 
		// produces a 32 kHz clock
		prescaler #(
		    .PRESCALER_WIDTH(PRESCALER_WIDTH)
		) prescaler_inst (
		    .clock(clock),
		    .reset(reset_pwm),
		    .limit(limit_value),
		    .tick(clock_enable)
		);
		
		// instantiate the PWM controller for the red LED
		pwm_controller 
		#(
		    .RESOLUTION(4)
		) pwm_controller_red (
		    .clock(clock),
		    .reset(reset_pwm),
		    .SW(SW[1:0]),
		    .clock_enable(clock_enable),
		    .PWM(PWM_R)
		);
		
		
		// instantiate the PWM controller for the red LED
		pwm_controller 
		#(
		    .RESOLUTION(4)
		) pwm_controller_blue (
		    .clock(clock),
		    .reset(reset_pwm),
		    .SW(SW[3:2]),
		    .clock_enable(clock_enable),
		    .PWM(PWM_G)
		);
		
		// instantiate the PWM controller for the red LED
		pwm_controller #(
		    .RESOLUTION(4)
		) pwm_controller_green
		(
		    .clock(clock),
		    .reset(reset_pwm),
		    .SW(SW[5:4]),
		    .clock_enable(clock_enable),
		    .PWM(PWM_B)
		);
endmodule;

module top(
    input  logic clock,
    input  logic reset,
    input  logic rx,
    output logic [5:0] token,
    output logic reset_pwm,
    output logic read_enable,
    output logic ctrl,
    output logic done,
    output logic [7:0]message
);

    logic uart_rx_done_meta, uart_rx_done_sync;

    // 2-FF synchronizer for uart_rx_done
    logic uart_rx_done;
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            uart_rx_done_meta <= 0;
            uart_rx_done_sync <= 0;
        end else begin
            uart_rx_done_meta <= uart_rx_done;
            uart_rx_done_sync <= uart_rx_done_meta;
        end
    end

    // UART receiver
    uart_receiver uart_inst (
        .clock(clock),
        .reset(reset),
        .rx(rx),
        .data_out(message),
        .rx_done(uart_rx_done)
    );

    // Shift register collects 6 bytes
    shift_reg_parallel_read #(
        .BITS(8),
        .TIMES(6)
    ) sr_inst (
        .clock(clock),
        .reset(reset),
        .data_in(message),
        .rx_done(uart_rx_done_sync),
        .token(token)
    );

    // Direct signals on rx_done pulse
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            reset_pwm   <= 0;
            read_enable <= 0;
            ctrl        <= 0;
            done        <= 0;
        end else if (uart_rx_done_sync) begin
            reset_pwm   <= 1;
            read_enable <= 1;
            ctrl        <= 1;
            done        <= 1;
        end else begin
            reset_pwm   <= 0;
            read_enable <= 0;
            ctrl        <= 0;
            done        <= 0;
        end
    end
endmodule
