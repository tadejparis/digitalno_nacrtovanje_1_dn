module control_unit (
    input  logic clk, //baud_rate
    input  logic rst, //reset
    input  logic stop_flag, //stop
    input  logic tx_done, //from uart_transmitter

    output logic reset_pwm, //reset the color -> RGB controller
    output logic read_enable, //to shift register
    output logic ctrl, //to rgb controller (waits one clock cycle)
    output logic [3:0] tx_index, //multiplexer index
    output logic tx_start, //to UART transmitter -> start
    output logic done //local done
);

    typedef enum logic [1:0] {
        PRINT1,
        AWAIT,
        CONTROL,
        PRINT2
    } state_t;

    //states (current, next)
    state_t state, next_state;

    //next reset_pwm, read_enable, ctrl, tx_index, tx_start and done
    logic reset_pwm_n, read_enable_n, ctrl_n;
    logic [3:0] tx_index_n;
    logic tx_start_n, done_n;

    // reset the states or prepare for next state
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= PRINT1;
            tx_index     <= 0;
            reset_pwm    <= 0;
            read_enable  <= 0;
            ctrl         <= 0;
            tx_start     <= 0;
            done         <= 0;
        end else begin
            state        <= next_state;
            tx_index     <= tx_index_n;
            reset_pwm    <= reset_pwm_n;
            read_enable  <= read_enable_n;
            ctrl         <= ctrl_n;
            tx_start     <= tx_start_n;
            done         <= done_n;
        end
    end

    //reset the defaults and do state machine step
    always_comb begin
        next_state     = state;
        tx_index_n     = tx_index;
        reset_pwm_n    = 0;
        read_enable_n  = 0;
        ctrl_n         = 0;
        tx_start_n     = 0;
        done_n         = 0;

        case (state)
            PRINT1: begin
                if (!tx_done) begin
                    next_state = PRINT1;
                end else if (tx_index < 6) begin
                    tx_index_n = tx_index + 1;
                    tx_start_n = 1;
                    next_state = PRINT1;
                end else begin
                    tx_index_n = tx_index + 1;
                    done_n     = 1;
                    next_state = AWAIT;
                end
            end
    
            AWAIT: begin
                if (!stop_flag) begin
                    next_state = AWAIT;
                end else begin
                    read_enable_n = 1;
                    reset_pwm_n   = 1;
                    next_state    = CONTROL;
                end
            end
    
            CONTROL: begin
                done_n     = 1;
                ctrl_n     = 1;
                tx_start_n = 1;
                next_state = PRINT2;
            end
    
            PRINT2: begin
                if (!tx_done) begin
                    next_state = PRINT2;
                end else if (tx_index < 15) begin
                    tx_index_n = tx_index + 1;
                    tx_start_n = 1;
                    next_state = PRINT2;
                end else begin
                    tx_index_n = 0;
                    done_n     = 1;
                    tx_start_n = 1;
                    next_state = PRINT1;
                end
            end
        endcase
    end
endmodule

module uart_transmitter_fsm #(
    parameter DATA_BITS = 8,
    parameter STOP_BITS = 1
) (
    input  logic clock,
    input  logic reset,
    input  logic sample_tick,
    input  logic [DATA_BITS-1:0] data_in,
    input  logic start,            // pulse to begin transmission

    output logic tx,
    output logic tx_done,
    output logic chg_state
);

    localparam STOP_TICKS = STOP_BITS * 16;

    typedef enum logic [1:0] {
        IDLE,
        START,
        DATA,
        STOP
    } state_uart_t;

    state_uart_t state, next_state;

    logic [DATA_BITS-1:0] shreg, shreg_next;
    logic [3:0] s_counter, s_counter_next;
    logic [3:0] n_counter, n_counter_next;
    logic tx_done_next, chg_state_next;
    logic tx_next;

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            shreg <= 0;
            s_counter <= 0;
            n_counter <= 0;
            tx <= 1;
            tx_done <= 0;
            chg_state <= 0;
        end else begin
            state <= next_state;
            shreg <= shreg_next;
            s_counter <= s_counter_next;
            n_counter <= n_counter_next;
            tx <= tx_next;
            tx_done <= tx_done_next;
            chg_state <= chg_state_next;
        end
    end

    always_comb begin
        next_state = state;
        shreg_next = shreg;
        s_counter_next = s_counter;
        n_counter_next = n_counter;
        tx_next = tx;
        tx_done_next = 0;
        chg_state_next = 0;

        case (state)

        IDLE: begin
            tx_next = 1;
            if (start) begin
                shreg_next = data_in;
                next_state = START;
                s_counter_next = 0;
                chg_state_next = 1;
            end
        end

        START: begin
            tx_next = 0;
            if (sample_tick) begin
                if (s_counter == 15) begin
                    next_state = DATA;
                    s_counter_next = 0;
                    n_counter_next = 0;
                    chg_state_next = 1;
                end else
                    s_counter_next = s_counter + 1;
            end
        end

        DATA: begin
            tx_next = shreg[0];
            if (sample_tick) begin
                if (s_counter == 15) begin
                    s_counter_next = 0;
                    shreg_next = {1'b0, shreg[DATA_BITS-1:1]};
                    if (n_counter == DATA_BITS-1) begin
                        next_state = STOP;
                        chg_state_next = 1;
                    end else
                        n_counter_next = n_counter + 1;
                end else
                    s_counter_next = s_counter + 1;
            end
        end

        STOP: begin
            tx_next = 1;
            if (sample_tick) begin
                if (s_counter == STOP_TICKS - 1) begin
                    tx_done_next = 1;
                    next_state = IDLE;
                    s_counter_next = 0;
                    chg_state_next = 1;
                end else
                    s_counter_next = s_counter + 1;
            end
        end

        endcase
    end
endmodule

