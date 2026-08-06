// ============================================================================
// Module: w5500_receive
// Target: EP4CE10E22I7 (Cyclone IV E)
// Description:
//   W5500 Socket0 RX (receive) controller + runtime reconnect. Waits for
//   w5500_init to finish (init_done), then owns the SPI bus for the poll/
//   receive loop. On each received packet it captures the data into buffer[]
//   and hands off to w5500_send (via the main.v arbiter) to echo it back.
//
//   Runtime error recovery (DISCON / TIMEOUT / LISTEN fail) is handled here:
//   CLOSE the socket then re-run the socket-config chain
//   (Sn_MR -> Sn_PORT -> OPEN -> LISTEN). A private copy of that chain lives
//   in this module (the power-up copy lives in w5500_init).
//
//   Mirrors STM32 Read_SOCK_Data_Buffer for the receive path.
// ============================================================================

module w5500_receive (
    input  wire        clk,            // 50MHz system clock
    input  wire        rst_n,          // active-low reset

    // ---- SPI Master command interface (muxed onto shared spi_master) ----
    output reg         spi_start,      // pulse to start SPI transaction
    output reg         spi_cmd_rw,     // 0=read, 1=write
    output reg  [1:0]  spi_cmd_om,     // 00=VDM, 01=FDM1, 10=FDM2
    output reg  [4:0]  spi_cmd_bsb,    // Block Select Bits (raw)
    output reg  [15:0] spi_cmd_addr,   // 16-bit register offset
    output reg  [15:0] spi_cmd_wdata,  // Write data (FDM modes)
    output reg  [10:0] spi_cmd_len,    // Data length (VDM mode)
    input  wire        spi_done,       // SPI transaction done pulse
    input  wire        spi_busy,       // SPI transaction in progress
    input  wire [7:0]  spi_rx_data,    // Last received byte (low byte for FDM2)
    input  wire [7:0]  spi_rx_data_hi, // FDM2 read only: high byte

    // ---- Buffer init addresses (set before VDM transaction) ----
    output reg  [10:0] buf_wr_init,    // Buffer write start address
    output reg  [10:0] buf_rd_init,    // Buffer read start address (unused here, kept 0)

    // ---- W5500 interrupt ----
    input  wire        w5500_int,      // W5500 INTn (active low, RX detection)

    // ---- Init handshake ----
    input  wire        init_done,      // high once w5500_init completed

    // ---- Echo handshake (to w5500_send via main arbiter) ----
    output reg         rx_ready,       // 1-cycle pulse: RX data ready in buffer[0..rx_len-1]
    output reg  [10:0] rx_len,         // received data length in bytes
    input  wire        send_done,      // 1-cycle pulse: send controller finished echo

    // ---- Status LED ----
    output wire        led_activity,   // high for 0.5s after each RX (activity)

    // ---- Physical link status (to w5500_init via main arbiter) ----
    output reg         link_lost,      // 1-cycle pulse: PHY cable unplugged

    // ---- External input levels (from main.v, debounced) ----
    input  wire [15:0] in_status       // bit0..11 = input1..12 level (active-high)
);

// ============================================================================
// Parameters: W5500 Register Addresses
// ============================================================================
localparam [15:0] MR       = 16'h0000;
localparam [15:0] SIR      = 16'h0017;
localparam [15:0] SIMR     = 16'h0018;
localparam [15:0] IR       = 16'h0015;
localparam [15:0] IMR      = 16'h0016;
localparam [15:0] PHYCFGR  = 16'h002E;

localparam [15:0] Sn_MR         = 16'h0000;
localparam [15:0] Sn_CR         = 16'h0001;
localparam [15:0] Sn_IR         = 16'h0002;
localparam [15:0] Sn_SR         = 16'h0003;
localparam [15:0] Sn_PORT       = 16'h0004;
localparam [15:0] Sn_RX_RSR     = 16'h0026;
localparam [15:0] Sn_RX_RD      = 16'h0028;
localparam [15:0] Sn_TX_WR      = 16'h0024;

// ============================================================================
// Parameters: Control Byte Components
// ============================================================================
localparam      RWB_READ  = 1'b0;
localparam      RWB_WRITE = 1'b1;
localparam [1:0] OM_VDM   = 2'b00;
localparam [1:0] OM_FDM1  = 2'b01;
localparam [1:0] OM_FDM2  = 2'b10;
localparam [1:0] OM_FDM4  = 2'b11;

localparam [4:0] BSB_COMMON   = 5'd0;
localparam [4:0] BSB_S0_REG   = 5'd1;
localparam [4:0] BSB_S0_TXBUF = 5'd2;
localparam [4:0] BSB_S0_RXBUF = 5'd3;

// ============================================================================
// Parameters: Register Bit Definitions
// ============================================================================
localparam [7:0] PHY_LINK    = 8'h01;
localparam [7:0] Sn_MR_TCP   = 8'h01;
localparam [7:0] Sn_CR_LISTEN= 8'h02;
localparam [7:0] Sn_CR_CLOSE = 8'h10;
localparam [7:0] Sn_CR_RECV  = 8'h40;
localparam [7:0] Sn_CR_SEND  = 8'h20;
localparam [7:0] SIR_S0_INT    = 8'h01;
localparam [7:0] Sn_IR_CON     = 8'h01;
localparam [7:0] Sn_IR_DISCON  = 8'h02;
localparam [7:0] Sn_IR_RECV    = 8'h04;
localparam [7:0] Sn_IR_TIMEOUT = 8'h08;
localparam [7:0] Sn_IR_SENDOK  = 8'h10;
localparam [7:0] SOCK_INIT   = 8'h13;
localparam [7:0] SOCK_LISTEN = 8'h14;
localparam [7:0] SOCK_ESTAB  = 8'h17;

// ============================================================================
// Parameters: Network Configuration
// ============================================================================
localparam [15:0] LOCAL_PORT  = 16'd5000;
localparam [10:0] BUF_SIZE    = 11'd2048;

// ============================================================================
// Parameters: Timing (50MHz clock)
// ============================================================================
localparam T_1MS       = 26'd50000;
localparam T_5MS       = 26'd250000;
localparam T_10MS      = 26'd500000;
localparam T_50MS      = 26'd2500000;
localparam T_200MS     = 26'd10000000;
localparam T_500MS     = 26'd25000000;  // activity LED one-shot on-time (~0.5s)

// ============================================================================
// State Machine Encoding
// ============================================================================
localparam [6:0]
    S_WAIT_INIT         = 7'd0,    // wait for w5500_init to finish
    S_SOCK_PORT         = 7'd21,   // socket reconfig chain (reconnect copy)
    S_SOCK_OPEN_MR      = 7'd22,
    S_SOCK_OPEN_CR      = 7'd23,
    S_SOCK_OPEN_WAIT    = 7'd24,
    S_SOCK_CHECK_INIT   = 7'd25,
    S_SOCK_LISTEN_CR    = 7'd26,
    S_SOCK_LISTEN_WAIT  = 7'd27,
    S_SOCK_CHECK_LISTEN = 7'd28,
    S_SOCK_VERIFY_LISTEN= 7'd29,
    S_POLL_SIR          = 7'd30,
    S_POLL_CHECK        = 7'd31,
    S_POLL_S0_IR        = 7'd32,
    S_POLL_CHECK_IR     = 7'd33,
    S_LINK_CHECK        = 7'd34,   // parse PHYCFGR (physical link poll)
    S_LINK_DOWN_WAIT    = 7'd36,   // hold until init drops init_done (re-init)
    S_RECV_RSR_H        = 7'd35,   // FDM2 read Sn_RX_RSR (16-bit)
    S_RECV_CHECK_SIZE   = 7'd37,   // check rx_data_len > 0
    S_RECV_CALC_OFFSET  = 7'd39,
    S_RECV_DATA         = 7'd40,
    S_RECV_DATA2        = 7'd41,
    S_RECV_UPDATE_RD    = 7'd42,
    S_RECV_CMD          = 7'd43,
    S_SOCK_CLOSE        = 7'd51,
    S_ERROR_WAIT        = 7'd52,
    S_RETRY_DELAY       = 7'd53,
    S_DISCON_CLOSE      = 7'd60,
    S_DISCON_WAIT       = 7'd61,
    S_SEND_CLEAR_IR     = 7'd65,   // read Sn_IR after echo
    S_SEND_CLEAR_IR_WR  = 7'd66,   // write back Sn_IR to clear flags
    S_ECHO_REQ          = 7'd67,   // pulse rx_ready -> hand SPI bus to w5500_send
    S_ECHO_WAIT         = 7'd68,   // wait send_done, then clear Sn_IR
    S_STAT_RD_TXWR      = 7'd70,   // input-status frame: FDM2 read Sn_TX_WR
    S_STAT_TXWR_WAIT    = 7'd71,   // capture ptr/offset, write byte0 (0xAA)
    S_STAT_WR_B1        = 7'd72,   // write byte1 (0x55)
    S_STAT_WR_B2        = 7'd73,   // write byte2 (0x10)
    S_STAT_WR_B3        = 7'd74,   // write byte3 (status low)
    S_STAT_WR_B4        = 7'd75,   // write byte4 (status high)
    S_STAT_WR_B5        = 7'd79,   // write byte5 (xor checksum)
    S_STAT_UPD_WR       = 7'd76,   // FDM2 write Sn_TX_WR = ptr + 6
    S_STAT_SEND_CMD     = 7'd77,   // FDM1 write Sn_CR = SEND
    S_STAT_DONE         = 7'd78;   // latch last, clear Sn_IR

// ============================================================================
// Internal Signals
// ============================================================================
reg  [6:0]  main_state;

// Timer
reg  [25:0] timer_cnt;
reg         timer_start;
reg  [25:0] timer_target;
wire        timer_done;

// Data tracking
reg  [10:0] rx_data_len;
reg  [15:0] rx_rd_ptr;
reg  [15:0] rx_offset;     // 16-bit to match spi_cmd_addr width exactly
reg         need_split;

// Connection tracking
reg         sock_established;   // set on CON interrupt, cleared on DISCON/TIMEOUT

// Saved IR value for clear
reg  [7:0]  saved_ir;
reg  [7:0]  rx_saved_ir;  // latched Sn_IR for RECV branch (matches STM32 j)

// ---- Input-status reporting ----
reg  [15:0] in_status_last;   // last reported input level (12-bit)
reg  [15:0] stat_snap;        // snapshot of in_status for the current TX frame
reg  [15:0] stat_ptr;         // raw Sn_TX_WR pointer read from W5500
reg  [15:0] stat_offset;      // stat_ptr & (TX_SIZE-1)
reg         stat_force;       // force one report (currently unused; level-change only)

// INTn synchronization (active-low asynchronous input, 2-FF sync)
reg         w5500_int_d1;
reg         w5500_int_d2;
wire        w5500_int_n;       // synchronized, active-low level

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        w5500_int_d1 <= 1'b1;   // assume idle high
        w5500_int_d2 <= 1'b1;
    end else begin
        w5500_int_d1 <= w5500_int;
        w5500_int_d2 <= w5500_int_d1;
    end
end
assign w5500_int_n = w5500_int_d2;

// INTn falling-edge detector (matches STM32's EXTI + software-flag behavior).
reg         w5500_int_d3;      // extra FF for edge detection
wire        intn_fall_pulse;   // 1-cycle pulse on INTn falling edge

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        w5500_int_d3 <= 1'b1;  // idle high, same as d1/d2
    else
        w5500_int_d3 <= w5500_int_d2;
end

assign intn_fall_pulse = ~w5500_int_d2 & w5500_int_d3;

// ---- LED activity one-shot ----
reg         led3_on;       // RX activity: high for 0.5s after each RECV
reg         rx_led_trig;   // 1-cycle pulse from main FSM when RECV detected
reg  [25:0] led3_cnt;      // one-shot counter for led3
assign led_activity = led3_on;

// ---- Physical link (PHYCFGR) poll ----
reg  [25:0] link_poll_cnt;   // free-running ~200ms poll interval
reg         link_poll_due;   // 1 = time to issue a PHYCFGR read
reg         link_poll_ack;   // 1-cycle: FSM issued the PHYCFGR read

// ============================================================================
// Timer
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        timer_cnt <= 26'd0;
    else if (timer_start)
        timer_cnt <= 26'd0;
    else if (timer_cnt < timer_target)
        timer_cnt <= timer_cnt + 1'b1;
end

assign timer_done = (timer_cnt == timer_target) && (timer_target > 0);

// ============================================================================
// Physical-link poll timer: flag a PHYCFGR read every ~200ms while we own the
// bus. Held reset while waiting for init so we never poll before ownership.
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        link_poll_cnt <= 26'd0;
        link_poll_due <= 1'b0;
    end else if (link_poll_ack || (main_state == S_WAIT_INIT)) begin
        link_poll_cnt <= 26'd0;
        link_poll_due <= 1'b0;
    end else if (link_poll_cnt < T_200MS) begin
        link_poll_cnt <= link_poll_cnt + 1'b1;
    end else begin
        link_poll_due <= 1'b1;
    end
end

// ============================================================================
// LED activity one-shot: light for 0.2s on each received packet, auto-off.
// Retriggerable — a new RECV while lit restarts the 0.2s window.
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        led3_on  <= 1'b0;
        led3_cnt <= 26'd0;
    end else if (rx_led_trig) begin
        led3_on  <= 1'b1;       // received data -> LED on, restart 0.2s window
        led3_cnt <= 26'd0;
    end else if (led3_on) begin
        if (led3_cnt < T_200MS)
            led3_cnt <= led3_cnt + 1'b1;
        else
            led3_on <= 1'b0;    // 0.2s elapsed -> LED off
    end
end

// ============================================================================
// Main Receive Controller State Machine
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        main_state    <= S_WAIT_INIT;

        spi_start     <= 1'b0;
        spi_cmd_rw    <= RWB_READ;
        spi_cmd_om    <= OM_VDM;
        spi_cmd_bsb   <= BSB_COMMON;
        spi_cmd_addr  <= 16'd0;
        spi_cmd_wdata <= 16'd0;
        spi_cmd_len   <= 11'd0;

        timer_start   <= 1'b0;
        timer_target  <= 26'd0;

        buf_wr_init   <= 11'd0;
        buf_rd_init   <= 11'd0;

        rx_data_len   <= 11'd0;
        rx_rd_ptr     <= 16'd0;
        rx_offset     <= 16'd0;
        need_split    <= 1'b0;
        saved_ir      <= 8'd0;
        rx_saved_ir   <= 8'd0;
        sock_established <= 1'b0;

        rx_led_trig   <= 1'b0;

        rx_ready      <= 1'b0;
        rx_len        <= 11'd0;

        link_lost     <= 1'b0;
        link_poll_ack <= 1'b0;

        in_status_last <= 16'd0;
        stat_snap      <= 16'd0;
        stat_ptr       <= 16'd0;
        stat_offset    <= 16'd0;
        stat_force     <= 1'b0;   // do NOT auto-report on reset / new connection
    end else begin
        // Defaults
        spi_start   <= 1'b0;
        timer_start <= 1'b0;
        rx_led_trig <= 1'b0;     // default low, pulsed on RECV detect
        rx_ready    <= 1'b0;     // default low, pulsed in S_ECHO_REQ
        link_lost   <= 1'b0;     // default low, pulsed on PHY link loss
        link_poll_ack <= 1'b0;   // default low, pulsed when PHYCFGR read issued

        // While NOT connected, keep the "last reported" level tracking the
        // live input level. This way, at the instant a connection is
        // established, in_status_last already equals in_status, so no spurious
        // status frame is pushed on the (re)connect. Only genuine level
        // changes that occur AFTER connection are reported.
        if (!sock_established)
            in_status_last <= in_status;

        case (main_state)
            // ================================================================
            // WAIT INIT: hold until w5500_init completes, then start polling.
            // The main.v arbiter grants the SPI bus to this module at the same
            // time init_done goes high, so we do not touch the bus until then.
            // ================================================================
            S_WAIT_INIT: begin
                if (init_done)
                    main_state <= S_POLL_SIR;
            end

            // ================================================================
            // POLL LOOP
            // ================================================================
            S_POLL_SIR: begin
                // INTn falling-edge driven polling: each INTn falling edge
                // triggers EXACTLY ONE read-SIR sequence, mirroring the STM32
                // side's EXTI + software-flag pattern.
                if (intn_fall_pulse && !spi_busy && !spi_done) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM1;
                    spi_cmd_bsb  <= BSB_COMMON;
                    spi_cmd_addr <= SIR;
                    spi_start    <= 1'b1;
                    main_state   <= S_POLL_CHECK;
                end else if (link_poll_due && !spi_busy && !spi_done) begin
                    // Periodic physical-link check (cable-unplug detection).
                    spi_cmd_rw    <= RWB_READ;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_COMMON;
                    spi_cmd_addr  <= PHYCFGR;
                    spi_start     <= 1'b1;
                    link_poll_ack <= 1'b1;
                    main_state    <= S_LINK_CHECK;
                end else if (sock_established && w5500_int_n &&
                             (stat_force || (in_status != in_status_last)) &&
                             !spi_busy && !spi_done) begin
                    // Input level changed (or forced on a new connection):
                    // push a status frame to the PC via the TX buffer.
                    stat_snap  <= in_status;
                    stat_force <= 1'b0;
                    main_state <= S_STAT_RD_TXWR;
                end
            end

            S_POLL_CHECK: begin
                if (spi_done && !spi_busy) begin
                    if (spi_rx_data & SIR_S0_INT) begin
                        main_state <= S_POLL_S0_IR;
                    end else begin
                        main_state <= S_POLL_SIR;
                    end
                end
            end

            S_POLL_S0_IR: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM1;
                    spi_cmd_bsb  <= BSB_S0_REG;
                    spi_cmd_addr <= Sn_IR;
                    spi_start    <= 1'b1;
                    main_state   <= S_POLL_CHECK_IR;
                end
            end

            // STRICT STM32 alignment: Read Sn_IR -> Write back the SAME value
            // (j) -> dispatch by flag. ONE SPI write clears all latched flags.
            S_POLL_CHECK_IR: begin
                if (spi_done && !spi_busy) begin
                    saved_ir <= spi_rx_data;
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_IR;
                    spi_cmd_wdata <= {8'd0, spi_rx_data};
                    spi_start     <= 1'b1;

                    // Dispatch (order matches STM32's sequential if's):
                    if (spi_rx_data & Sn_IR_CON) begin
                        sock_established <= 1'b1;
                        main_state <= S_POLL_SIR;
                    end else if (spi_rx_data & Sn_IR_DISCON) begin
                        sock_established <= 1'b0;
                        main_state <= S_DISCON_CLOSE;
                    end else if (spi_rx_data & Sn_IR_RECV) begin
                        rx_led_trig   <= 1'b1;
                        rx_saved_ir   <= spi_rx_data;
                        main_state    <= S_RECV_RSR_H;
                    end else if (spi_rx_data & Sn_IR_TIMEOUT) begin
                        sock_established <= 1'b0;
                        main_state <= S_DISCON_CLOSE;
                    end else begin
                        main_state <= S_POLL_SIR;
                    end
                end
            end

            // ================================================================
            // PHYSICAL LINK CHECK: parse PHYCFGR. LNK=0 -> cable unplugged.
            // Pulse link_lost so the main arbiter hands the bus back to
            // w5500_init for a full re-init (drops link LED; re-LISTENs so the
            // client can reconnect once the cable is plugged back in).
            // ================================================================
            S_LINK_CHECK: begin
                if (spi_done && !spi_busy) begin
                    if ((spi_rx_data & PHY_LINK) == 8'd0) begin
                        link_lost  <= 1'b1;
                        main_state <= S_LINK_DOWN_WAIT;
                    end else begin
                        main_state <= S_POLL_SIR;
                    end
                end
            end

            S_LINK_DOWN_WAIT: begin
                // Hold until w5500_init drops init_done (re-init started),
                // then re-sync via S_WAIT_INIT until init completes again.
                if (!init_done)
                    main_state <= S_WAIT_INIT;
            end

            // ================================================================
            // RECEIVE DATA (matches STM32 Read_SOCK_Data_Buffer exactly)
            // ================================================================
            S_RECV_RSR_H: begin
                // FDM2 read Sn_RX_RSR -> {spi_rx_data_hi, spi_rx_data} = rx_size
                // SPI frame: [00 26 0A xx xx]
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM2;
                    spi_cmd_bsb  <= BSB_S0_REG;
                    spi_cmd_addr <= Sn_RX_RSR;
                    spi_start    <= 1'b1;
                    main_state   <= S_RECV_CHECK_SIZE;
                end
            end

            S_RECV_CHECK_SIZE: begin
                if (spi_done && !spi_busy) begin
                    // FDM2 read of Sn_RX_RSR complete.
                    if ({spi_rx_data_hi, spi_rx_data} == 16'd0) begin
                        rx_data_len <= 11'd0;
                        main_state <= S_POLL_SIR;
                    end else begin
                        if ({spi_rx_data_hi, spi_rx_data} > 16'd1460)
                            rx_data_len <= 11'd1460;
                        else
                            rx_data_len <= {spi_rx_data_hi[2:0], spi_rx_data};

                        // Issue FDM2 read of Sn_RX_RD (16-bit rx_rd_ptr)
                        // SPI frame: [00 28 0A xx xx]
                        spi_cmd_rw   <= RWB_READ;
                        spi_cmd_om   <= OM_FDM2;
                        spi_cmd_bsb  <= BSB_S0_REG;
                        spi_cmd_addr <= Sn_RX_RD;
                        spi_start    <= 1'b1;
                        main_state   <= S_RECV_CALC_OFFSET;
                    end
                end
            end

            S_RECV_CALC_OFFSET: begin
                if (spi_done && !spi_busy) begin
                    // FDM2 read Sn_RX_RD complete: {hi, lo} = rd_ptr
                    rx_rd_ptr <= {spi_rx_data_hi, spi_rx_data};
                    rx_offset <= {spi_rx_data_hi, spi_rx_data} & 16'h07FF;
                    need_split <= (({spi_rx_data_hi, spi_rx_data} & 16'h07FF)
                                   + {5'd0, rx_data_len} > 16'd2048);
                    main_state <= S_RECV_DATA;
                end
            end

            S_RECV_DATA: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw    <= RWB_READ;
                    spi_cmd_om    <= OM_VDM;
                    spi_cmd_bsb   <= BSB_S0_RXBUF;
                    spi_cmd_addr  <= rx_offset;
                    spi_cmd_len   <= need_split ? (BUF_SIZE - rx_offset[10:0]) : rx_data_len;
                    spi_start     <= 1'b1;
                    buf_wr_init   <= 11'd0;
                    main_state    <= need_split ? S_RECV_DATA2 : S_RECV_UPDATE_RD;
                end
            end

            S_RECV_DATA2: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_READ;
                    spi_cmd_om    <= OM_VDM;
                    spi_cmd_bsb   <= BSB_S0_RXBUF;
                    spi_cmd_addr  <= 16'd0;
                    spi_cmd_len   <= rx_data_len - (BUF_SIZE - rx_offset[10:0]);
                    spi_start     <= 1'b1;
                    buf_wr_init   <= BUF_SIZE - rx_offset[10:0];
                    main_state    <= S_RECV_UPDATE_RD;
                end
            end

            S_RECV_UPDATE_RD: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM2;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_RX_RD;
                    spi_cmd_wdata <= rx_rd_ptr + {5'd0, rx_data_len};
                    spi_start     <= 1'b1;
                    main_state    <= S_RECV_CMD;
                end
            end

            S_RECV_CMD: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_CR;
                    spi_cmd_wdata <= {8'd0, Sn_CR_RECV};
                    spi_start     <= 1'b1;
                    // RECV command issued. Hand off to w5500_send to echo
                    // buffer[0..rx_data_len-1]. Sn_IR is cleared afterwards in
                    // S_SEND_CLEAR_IR so INTn releases for the next RECV.
                    main_state    <= S_ECHO_REQ;
                end
            end

            // ================================================================
            // ECHO HANDSHAKE: pass the SPI bus to w5500_send, wait for it to
            // finish, then fall through to the Sn_IR clear sequence.
            // ================================================================
            S_ECHO_REQ: begin
                if (spi_done && !spi_busy) begin
                    rx_ready   <= 1'b1;          // 1-cycle pulse (auto-cleared by default)
                    rx_len     <= rx_data_len;   // bytes to echo (buffer[0..])
                    main_state <= S_ECHO_WAIT;
                end
            end

            S_ECHO_WAIT: begin
                // Bus released to w5500_send. Resume once it pulses send_done.
                if (send_done)
                    main_state <= S_SEND_CLEAR_IR;
            end

            // ================================================================
            // INPUT-STATUS REPORT: push a 6-byte frame to the PC when an input
            // level changes (or on a new connection). Frame (matches the PC's
            // 0x10 parser): AA 55 10 <status_lo> <status_hi> <xor>,
            // xor = 0xEF ^ status_lo ^ status_hi.
            // We own the SPI bus here (BUS_RECV), so we write the TX buffer
            // directly (byte-wise FDM1) then issue SEND, mirroring w5500_send.
            // ================================================================
            S_STAT_RD_TXWR: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM2;
                    spi_cmd_bsb  <= BSB_S0_REG;
                    spi_cmd_addr <= Sn_TX_WR;
                    spi_start    <= 1'b1;
                    main_state   <= S_STAT_TXWR_WAIT;
                end
            end

            S_STAT_TXWR_WAIT: begin
                if (spi_done && !spi_busy) begin
                    stat_ptr    <= {spi_rx_data_hi, spi_rx_data};
                    stat_offset <= {spi_rx_data_hi, spi_rx_data} & 16'h07FF;
                    // Write byte0 = 0xAA to TXBUF at offset.
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_TXBUF;
                    spi_cmd_addr  <= {spi_rx_data_hi, spi_rx_data} & 16'h07FF;
                    spi_cmd_wdata <= {8'd0, 8'hAA};
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_WR_B1;
                end
            end

            S_STAT_WR_B1: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_TXBUF;
                    spi_cmd_addr  <= (stat_offset + 16'd1) & 16'h07FF;
                    spi_cmd_wdata <= {8'd0, 8'h55};
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_WR_B2;
                end
            end

            S_STAT_WR_B2: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_TXBUF;
                    spi_cmd_addr  <= (stat_offset + 16'd2) & 16'h07FF;
                    spi_cmd_wdata <= {8'd0, 8'h10};
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_WR_B3;
                end
            end

            S_STAT_WR_B3: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_TXBUF;
                    spi_cmd_addr  <= (stat_offset + 16'd3) & 16'h07FF;
                    spi_cmd_wdata <= {8'd0, stat_snap[7:0]};   // status low byte
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_WR_B4;
                end
            end

            S_STAT_WR_B4: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_TXBUF;
                    spi_cmd_addr  <= (stat_offset + 16'd4) & 16'h07FF;
                    spi_cmd_wdata <= {8'd0, stat_snap[15:8]};  // status high byte
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_WR_B5;
                end
            end

            S_STAT_WR_B5: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_TXBUF;
                    spi_cmd_addr  <= (stat_offset + 16'd5) & 16'h07FF;
                    spi_cmd_wdata <= {8'd0, (8'hEF ^ stat_snap[7:0] ^ stat_snap[15:8])};
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_UPD_WR;
                end
            end

            S_STAT_UPD_WR: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM2;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_TX_WR;
                    spi_cmd_wdata <= stat_ptr + 16'd6;
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_SEND_CMD;
                end
            end

            S_STAT_SEND_CMD: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_CR;
                    spi_cmd_wdata <= {8'd0, Sn_CR_SEND};
                    spi_start     <= 1'b1;
                    main_state    <= S_STAT_DONE;
                end
            end

            S_STAT_DONE: begin
                if (spi_done && !spi_busy) begin
                    in_status_last <= stat_snap;
                    // Clear Sn_IR (incl. SENDOK) so INTn releases, then poll.
                    main_state     <= S_SEND_CLEAR_IR;
                end
            end

            // ================================================================
            // Clear Sn_IR after the echo completes.
            // ================================================================
            S_SEND_CLEAR_IR: begin
                // Entered from S_ECHO_WAIT after w5500_send released the bus:
                // no SPI transaction in flight, so wait for bus IDLE (not
                // spi_done, which would never re-pulse) before the Sn_IR read.
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM1;
                    spi_cmd_bsb  <= BSB_S0_REG;
                    spi_cmd_addr <= Sn_IR;
                    spi_start    <= 1'b1;
                    main_state   <= S_SEND_CLEAR_IR_WR;
                end
            end

            S_SEND_CLEAR_IR_WR: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_IR;
                    spi_cmd_wdata <= {8'd0, spi_rx_data};
                    spi_start     <= 1'b1;
                    main_state    <= S_POLL_SIR;
                end
            end

            // ================================================================
            // DISCONNECT / TIMEOUT: close socket then re-configure (reconnect)
            // ================================================================
            S_DISCON_CLOSE: begin
                // Sn_IR was just cleared by the caller (DISCON/TIMEOUT branch).
                // Wait for that write to finish, then issue CLOSE.
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_CR;
                    spi_cmd_wdata <= {8'd0, Sn_CR_CLOSE};
                    spi_start     <= 1'b1;
                    timer_target  <= T_10MS;
                    timer_start   <= 1'b1;
                    main_state    <= S_DISCON_WAIT;
                end
            end

            S_DISCON_WAIT: begin
                if (timer_done && !spi_busy) begin
                    // Re-init: Sn_MR -> Sn_PORT -> Sn_CR (same as retry)
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_MR;
                    spi_cmd_wdata <= {8'd0, Sn_MR_TCP};
                    spi_start     <= 1'b1;
                    main_state    <= S_SOCK_PORT;
                end
            end

            // ================================================================
            // SOCKET RE-CONFIG CHAIN (reconnect copy)
            // Sn_PORT -> Sn_MR=TCP -> OPEN -> check INIT -> LISTEN -> check
            // ================================================================
            S_SOCK_PORT: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM2;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_PORT;
                    spi_cmd_wdata <= LOCAL_PORT;
                    spi_start     <= 1'b1;
                    main_state    <= S_SOCK_OPEN_MR;
                end
            end

            S_SOCK_OPEN_MR: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_MR;
                    spi_cmd_wdata <= {8'd0, Sn_MR_TCP};
                    spi_start     <= 1'b1;
                    main_state    <= S_SOCK_OPEN_CR;
                end
            end

            S_SOCK_OPEN_CR: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_CR;
                    spi_cmd_wdata <= {8'd0, 8'h01};   // OPEN
                    spi_start     <= 1'b1;
                    main_state    <= S_SOCK_OPEN_WAIT;
                end
            end

            S_SOCK_OPEN_WAIT: begin
                if (spi_done) begin
                    timer_target <= T_5MS;
                    timer_start  <= 1'b1;
                    main_state   <= S_SOCK_CHECK_INIT;
                end
            end

            S_SOCK_CHECK_INIT: begin
                if (timer_done && !spi_busy) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM1;
                    spi_cmd_bsb  <= BSB_S0_REG;
                    spi_cmd_addr <= Sn_SR;
                    spi_start    <= 1'b1;
                    main_state   <= S_SOCK_LISTEN_CR;
                end
            end

            S_SOCK_LISTEN_CR: begin
                if (spi_done && !spi_busy) begin
                    if (spi_rx_data == SOCK_INIT) begin
                        spi_cmd_rw    <= RWB_WRITE;
                        spi_cmd_om    <= OM_FDM1;
                        spi_cmd_bsb   <= BSB_S0_REG;
                        spi_cmd_addr  <= Sn_CR;
                        spi_cmd_wdata <= {8'd0, Sn_CR_LISTEN};
                        spi_start     <= 1'b1;
                        main_state    <= S_SOCK_LISTEN_WAIT;
                    end else begin
                        main_state <= S_SOCK_CLOSE;
                    end
                end
            end

            S_SOCK_LISTEN_WAIT: begin
                if (spi_done) begin
                    timer_target <= T_5MS;
                    timer_start  <= 1'b1;
                    main_state   <= S_SOCK_CHECK_LISTEN;
                end
            end

            S_SOCK_CHECK_LISTEN: begin
                if (timer_done && !spi_busy) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM1;
                    spi_cmd_bsb  <= BSB_S0_REG;
                    spi_cmd_addr <= Sn_SR;
                    spi_start    <= 1'b1;
                    main_state   <= S_SOCK_VERIFY_LISTEN;
                end
            end

            S_SOCK_VERIFY_LISTEN: begin
                if (spi_done && !spi_busy) begin
                    if (spi_rx_data == SOCK_LISTEN)
                        main_state <= S_POLL_SIR;   // LISTEN OK -> resume poll
                    else
                        main_state <= S_SOCK_CLOSE;
                end
            end

            // ================================================================
            // ERROR HANDLING (LISTEN fail): CLOSE + delay + retry
            // ================================================================
            S_SOCK_CLOSE: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_CR;
                    spi_cmd_wdata <= {8'd0, Sn_CR_CLOSE};
                    spi_start     <= 1'b1;
                    main_state    <= S_ERROR_WAIT;
                end
            end

            S_ERROR_WAIT: begin
                if (spi_done) begin
                    timer_target <= T_10MS;
                    timer_start  <= 1'b1;
                    main_state   <= S_RETRY_DELAY;
                end
            end

            S_RETRY_DELAY: begin
                if (timer_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_MR;
                    spi_cmd_wdata <= {8'd0, Sn_MR_TCP};
                    spi_start     <= 1'b1;
                    main_state    <= S_SOCK_PORT;
                end
            end

            default: main_state <= S_WAIT_INIT;
        endcase
    end
end

endmodule
