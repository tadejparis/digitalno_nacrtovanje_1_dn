`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2025 03:53:49 PM
// Design Name: 
// Module Name: receiver
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module prescaler // uporabi to kot sample_tick za FSM in clock_enable za pwm
    #(parameter PRESCALER_WIDTH = 8)
    (
        input logic clock,
        input logic reset,
        input logic [PRESCALER_WIDTH-1:0] limit, // kaj tukaj nastavit
        output logic tick
    );

    logic [PRESCALER_WIDTH-1:0] count;

    always_ff @( posedge clock) begin 
        if (reset) begin
            count <= 0;
        end
        else begin
            if (count == limit-1) begin
                count <= 0;
            end
            else begin
                count <= count + 1'b1;
            end
        end
    end
    
    assign tick = (count == limit - 1);
endmodule 

module uart_receiver_fsm #(
    parameter DATA_BITS = 6, // dva za vsako barvo
    parameter STOP_BITS = 1
) (
    input logic clock,
    input logic reset,
    input logic sample_tick,
    input logic rx, 
    output logic [DATA_BITS-1:0] data_out,
    output logic rx_done,
    output logic chg_state
);

    // instantiate the sample tick generator
    prescaler #(
		.PRESCALER_WIDTH(8)
	) stg (
		.clock(clock),
		.reset(reset),
		.limit(limit_value), //TODO
		.tick(sample_tick)
	);

    // define the parameters
    localparam STOP_TICKS = STOP_BITS*16;
    
    // define the states
    typedef enum logic [1:0] { // binary encoding
        IDLE,
        START,
        DATA,
        STOP
    } state_uart_t;

    state_uart_t state, next_state;

    // signal declarations 
    // trenutni signal dobi naslednjo vrednost signal_next na naslednjem active edge.
    // logic [DATA_BITS-1:0] shift_reg, shift_reg_next; // shift register to store the received data bits - zakaj  je to tukaj? a ni to posebej modul
    logic dout;
    logic [3:0] s_counter, s_counter_next; // counter for sample_tick
    logic [3:0] n_counter, n_counter_next; // counter for number of symbols 
    logic rx_done_next, chg_state_next;
        
    // state register - DFF
    always_ff @(posedge clock) begin : state_reg
        if (reset) begin
            state <= IDLE;
            // shift_reg <= 0;
            s_counter <= 0;
            n_counter <= 0;
            rx_done <= 0;
            chg_state <= 0;
        end
        else begin
            state <= next_state;
            // shift_reg <= shift_reg_next;
            s_counter <= s_counter_next;
            n_counter <= n_counter_next;
            rx_done <= rx_done_next;
            chg_state <= chg_state_next;
        end
    end

    // next state logic
    always_comb begin : next_state_logic
        // default values, otherwise we will have a latch
        // need to cover all the cases 
        next_state = state;
        rx_done_next = 0;
        chg_state_next = 0;
        // shift_reg_next = shift_reg;
        s_counter_next = s_counter;
        n_counter_next = n_counter;
        
        case (state)
            IDLE : begin
                if (rx == 0) begin
                    next_state = START;
                    s_counter_next = 0;
                    rx_done_next = 0;
                    chg_state_next = 1;
                end
            end
            START : begin
                if(sample_tick) begin
                    if (s_counter == 7) begin
                        // cannot do n_counter = 0, two blocks power the same signal 
                        n_counter_next = 0;
                        s_counter_next = 0;
                        chg_state_next = 1;
                        // do not forget to update state 
                        next_state = DATA;
                    end else begin
                        s_counter_next = s_counter + 1;
                    end
                end
            end 
            DATA : begin
                if(sample_tick) begin
                   if (s_counter == 15) begin
                        s_counter_next = 0;
                        // shift_reg_next = {rx,shift_reg[DATA_BITS-1:1]};
                        dout = rx;
                        if (n_counter == DATA_BITS-1) begin
                            next_state = STOP;
                            chg_state_next = 1;
                        end else begin
                            n_counter_next = n_counter + 1;
                        end
                   end  else begin
                        s_counter_next = s_counter + 1;
                   end
                end
            end
            STOP : begin
                if (sample_tick) begin
                    if (s_counter == STOP_TICKS - 1) begin
                        rx_done_next = 1;
                        next_state = IDLE;
                        chg_state_next = 1;
                    end else begin
                        s_counter_next = s_counter + 1;
                    end
                end
            end
        endcase
    end

    // output 
    assign data_out = dout;

endmodule

// 6-bit PIPO shift register
module shift_reg_parallel_read
    #(parameter BITS = 6)
    (
        input logic clock,
        input logic reset,
        input logic [BITS-1:0] data_in,
        input logic rx_done,
        input logic read_enable,
        output logic token
    );
    
    logic [BITS-1:0] shift_reg;
    logic [BITS-1:0] dout;
    
    always_ff @(posedge clock) begin
        if (reset) begin
            shift_reg <= 0;
        end else if (rx_done && read_enable) begin
            dout <= shift_reg;
        end else begin
            shift_reg <= {data_in, shift_reg[BITS-1:1]};
        end
    end
    
    assign token = dout;

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
    localparam DT_50 = 1 << (RESOLUTION-1);
    localparam DT_25 = 1 << (RESOLUTION-2);
    localparam DT_12_5 = 1 << (RESOLUTION-3);

    // resolution of the PWM
    logic [RESOLUTION-1:0] count;
    logic [RESOLUTION:0] duty_cycle;
    
		always_comb begin : DutyCycleCalculate
        unique case (SW)
            2'b00: duty_cycle = 0;
            2'b01: duty_cycle = DT_12_5;
            2'b10: duty_cycle = DT_25;
            2'b11: duty_cycle = DT_50;
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
    output logic [1:0] PWM_R,
    output logic [1:0] PWM_G,
    output logic [1:0] PWM_B

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
endmodule

module receiving (
    input logic clock,
    input logic reset,
    input logic rx,
    
    input logic read_enable, // generated by control unit
    input logic reset_pwm, // generated by control unit
    input logic ctrl, // generated by control unit
    
    output logic data_out, // to control unit
    
    output logic [1:0] PWM_R,
    output logic [1:0] PWM_G,
    output logic [1:0] PWM_B

);

    localparam DATA_BITS = 6;
    logic [DATA_BITS-1:0] data_out;
    logic rx_done;
    logic chg_state;

    uart_receiver_fsm urf
    (     
        .clock(clock),
        .reset(reset),
        // .sample_tick(sample_tick),
        .rx(rx), 
        .data_out(data_out),
        .rx_done(rx_done),
        .chg_state(chg_state)
    );
    
    logic [DATA_BITS-1:0] token;
    
    shift_reg_parallel_read srpr
    (
        .clock(clock),
        .reset(reset),
        .data_in(data_out),
        .rx_done(rx_done),
        .read_enable(read_enable),
        .token(token)
    );
    
    rgb_controller rgb
    (
        .clock(clock),
        .reset_pwm,
        .SW(reset_pwm),
        .ctrl(ctrl),
        .PWM_R(PWM_R),
        .PWM_G(PWM_G),
        .PWM_B(PWM_B)
    );
 
endmodule 