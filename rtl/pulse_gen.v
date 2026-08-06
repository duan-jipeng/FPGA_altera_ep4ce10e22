// ============================================================================
// Module: pulse_gen
// Description:
//   Command-driven pulse generator for one motor channel (instantiated once
//   per motor). Idle (output HIGH) after reset. On cmd_start it emits pulses on pulse_out
//   at a frequency derived from cmd_freq_u (units of 100 Hz):
//       actual_hz   = cmd_freq_u * 100
//       half_period = 25_000_000 / actual_hz = 250000 / cmd_freq_u  (clocks)
//   A small restoring divider computes half_period once per command.
//     - Fixed mode  (cmd_continuous=0): emit exactly cmd_count pulses, stop.
//     - Cont. mode  (cmd_continuous=1): emit pulses until cmd_stop.
//   cmd_stop at any time halts output immediately (pulse_out HIGH).
//   Active-LOW pulses: line idles HIGH and dips LOW for each pulse's first
//   half period, then returns HIGH. 50% duty cycle. 50MHz system clock.
// ============================================================================

module pulse_gen (
    input  wire        clk,            // 50MHz system clock
    input  wire        rst_n,          // active-low reset

    input  wire        cmd_start,      // 1-cycle: begin a new pulse job
    input  wire        cmd_stop,       // 1-cycle: stop immediately
    input  wire        cmd_continuous, // 1=continuous (ignore count), 0=fixed count
    input  wire [31:0] cmd_count,      // pulse count (fixed mode)
    input  wire [15:0] cmd_freq_u,     // frequency in units of 100 Hz (1..2000)

    output reg         pulse_out       // pulse output (bind pin in QSF)
);

// ============================================================================
// Divider constant: half_period = DIVIDEND / freq_u
//   DIVIDEND = 25_000_000 / 100 = 250000  (fits in 18 bits: 2^18 = 262144)
// ============================================================================
localparam [17:0] DIVIDEND = 18'd250000;

// ---- FSM ----
localparam [1:0] S_IDLE   = 2'd0,
                 S_DIV     = 2'd1,
                 S_DIV_FIN = 2'd2,
                 S_RUN     = 2'd3;
reg [1:0] state;

// ---- Latched command ----
reg        continuous_l;
reg [31:0] count_l;
reg [15:0] dsor;        // divisor = freq_u (clamped >= 1)

// ---- Restoring-divider working registers ----
reg [17:0] div_rem;
reg [17:0] div_quo;
reg [4:0]  div_bit;
wire [17:0] div_tmp = {div_rem[16:0], DIVIDEND[div_bit]};
wire [17:0] dsor18  = {2'b00, dsor};

// ---- Pulse generation ----
reg [17:0] half_period; // clocks per half period
reg [17:0] div_cnt;     // half-period counter
reg        phase;       // 0=LOW half, 1=HIGH half (active-low pulse)
reg [31:0] pulse_done;  // completed pulses (fixed mode)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_IDLE;
        pulse_out    <= 1'b1;   // idle HIGH (active-low pulses)
        continuous_l <= 1'b0;
        count_l      <= 32'd0;
        dsor         <= 16'd1;
        div_rem      <= 18'd0;
        div_quo      <= 18'd0;
        div_bit      <= 5'd0;
        half_period  <= 18'd1;
        div_cnt      <= 18'd0;
        phase        <= 1'b0;
        pulse_done   <= 32'd0;
    end else begin
        // cmd_stop always wins: halt output, return to idle.
        if (cmd_stop) begin
            pulse_out <= 1'b1;   // halt -> idle HIGH
            state     <= S_IDLE;
        end else if (cmd_start) begin
            // (Re)start from ANY state. A new command (e.g. a different
            // frequency or count) is latched immediately and the divider
            // re-runs, so the user does NOT have to stop first to change
            // frequency mid-run.
            continuous_l <= cmd_continuous;
            count_l      <= cmd_count;
            dsor         <= (cmd_freq_u == 16'd0) ? 16'd1 : cmd_freq_u;
            // init restoring divider (MSB-first, 18 bits)
            div_rem <= 18'd0;
            div_quo <= 18'd0;
            div_bit <= 5'd17;
            state   <= S_DIV;
        end else begin
            case (state)
                // --------------------------------------------------------
                // IDLE: output HIGH, wait for a command.
                // --------------------------------------------------------
                S_IDLE: begin
                    pulse_out <= 1'b1;   // idle HIGH
                end

                // --------------------------------------------------------
                // DIV: restoring division half_period = DIVIDEND / dsor,
                //      one dividend bit per clock (MSB first).
                // --------------------------------------------------------
                S_DIV: begin
                    if (div_tmp >= dsor18) begin
                        div_rem          <= div_tmp - dsor18;
                        div_quo[div_bit] <= 1'b1;
                    end else begin
                        div_rem <= div_tmp;
                    end
                    if (div_bit == 5'd0)
                        state <= S_DIV_FIN;
                    else
                        div_bit <= div_bit - 1'b1;
                end

                // --------------------------------------------------------
                // DIV_FIN: quotient settled -> latch half_period, start run.
                // --------------------------------------------------------
                S_DIV_FIN: begin
                    half_period <= (div_quo == 18'd0) ? 18'd1 : div_quo;
                    div_cnt     <= 18'd0;
                    phase       <= 1'b0;
                    pulse_done  <= 32'd0;
                    pulse_out   <= 1'b0;   // begin first LOW half (active-low pulse)
                    state       <= S_RUN;
                end

                // --------------------------------------------------------
                // RUN: toggle every half_period; count full pulses.
                // --------------------------------------------------------
                S_RUN: begin
                    if (div_cnt >= half_period - 1'b1) begin
                        div_cnt <= 18'd0;
                        if (phase == 1'b0) begin
                            // finished LOW half -> HIGH half
                            phase     <= 1'b1;
                            pulse_out <= 1'b1;
                        end else begin
                            // finished HIGH half -> one full pulse complete
                            phase <= 1'b0;
                            if (!continuous_l && (pulse_done + 32'd1 >= count_l)) begin
                                pulse_out <= 1'b1;    // last pulse done -> idle HIGH
                                state     <= S_IDLE;
                            end else begin
                                pulse_done <= pulse_done + 32'd1;
                                pulse_out  <= 1'b0;   // next pulse LOW half
                            end
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
end

endmodule
