module baud_rate_generator #(
    parameter integer BAUD_DIV = 10416
)(
    input  logic clock,
    input  logic reset,
    output logic baud_tick
);

    integer cnt;

    always_ff @(posedge clock) begin
        if (reset) begin
            cnt <= 0;
        end else if (cnt == BAUD_DIV-1) begin
            cnt <= 0;
        end else begin
            cnt <= cnt + 1;
        end
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
    localparam integer HALF_BAUD = BAUD_DIV/2;

    // FSM states
    typedef enum logic [1:0] {
        IDLE,
        START,
        DATA,
        STOP
    } state_t;

    state_t state;

    integer baud_cnt;
    integer bit_cnt;
    logic [DBITS-1:0] shift_reg;

    always_ff @(posedge clock) begin
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

                        // LSB-first shift in (correct for UART)
                        shift_reg <= {rx, shift_reg[DBITS-1:1]};

                        if (bit_cnt == DBITS-1) begin
                            state <= STOP;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                        end
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

module top(
    input  logic       clock,
    input  logic       reset,
    input  logic       rx,
    output logic [7:0] message,
    output logic       rx_done,
    output logic       lucka
);

    always_ff @(posedge rx_done) begin
        lucka <= ~lucka;
    end
    

    uart_receiver uart_inst (
        .clock(clock),
        .reset(reset),
        .rx(rx),
        .data_out(message),
        .rx_done(rx_done)
    );

endmodule