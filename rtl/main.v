// ============================================================================
// Module: main (Top-Level)
// Target: EP4CE10E22I7 (Cyclone IV E), 50MHz clock (PIN_23)
//
// Hierarchy:
//   main (top)
//   ├── w5500_init    — W5500 power-up init + socket LISTEN (owns bus first)
//   ├── w5500_receive — poll + receive + echo handshake + reconnect
//   ├── w5500_send    — Socket0 TX echo controller
//   ├── spi_master    — SPI engine (Mode 0, 12.5MHz), shared via arbiter
//   └── buffer[2048]  — 2KB Block RAM for echo data
//
// W5500 TCP Server on port 5000 (IP: 192.168.123.98). Echoes received data.
// LED[0]=heartbeat, LED[1]=link, LED[2]=activity.
//
// Pin Mapping (test_lqfp.qsf):
//   sys_clk=PIN_23, sys_rst_n=PIN_24, key[0]=PIN_25,
//   led[0]=PIN_3, led[1]=PIN_2, led[2]=PIN_1,
//   w5500_rst=PIN_28, w5500_int=PIN_30, w5500_mosi=PIN_31,
//   w5500_miso=PIN_32, w5500_sclk=PIN_33, w5500_scs=PIN_34,
//   pulse_out[0..7] = PIN 64/66/68/70/98/100/103/105 (chan 0x01..0x08),
//   dir_out[0..7]   = PIN 65/67/69/71/99/101/104/106 (per-channel direction),
//   sig_in[0..11]   = PIN 110/111/112/113/114/115/119/120/121/124/125/126
// ============================================================================

module main (
    input  wire        sys_clk,        // 50MHz (PIN_23)
    input  wire        sys_rst_n,      // active-low reset (PIN_33)
    input  wire [2:0]  key,            // keys (unused, kept for compatibility)
    input  wire [11:0] sig_in,         // 12 external inputs, active-high

    output wire        w5500_sclk,
    output wire        w5500_mosi,
    input  wire        w5500_miso,
    output wire        w5500_scs,
    output wire        w5500_rst,
    input  wire        w5500_int,      // interrupt from W5500 (active low)

    output wire [7:0]  pulse_out,      // 8 motor pulse outputs (chan 0x01..0x08)
    output wire [7:0]  dir_out,        // 8 motor direction outputs (per-channel level)

    output wire [2:0]  led
);

// ============================================================================
// Network Constants (pre-loaded into buffer before VDM writes)
// ============================================================================
localparam [31:0] GATEWAY_IP  = {8'd192, 8'd168, 8'd123, 8'd1};
localparam [31:0] SUBNET_MASK = {8'd255, 8'd255, 8'd255, 8'd0};
localparam [47:0] MAC_ADDR    = {8'h0C, 8'h29, 8'hAB, 8'h7C, 8'h00, 8'h01};
localparam [31:0] LOCAL_IP    = {8'd192, 8'd168, 8'd123, 8'd98};
localparam [31:0] DIPR_IP     = {8'd193, 8'd169, 8'd124, 8'd99};  // IP+1 for Detect_Gateway

// Preload type encoding (matches w5500_init.v preload_type port)
localparam [2:0] PRELOAD_GAR  = 3'd0;
localparam [2:0] PRELOAD_SUBR = 3'd1;
localparam [2:0] PRELOAD_SHAR = 3'd2;
localparam [2:0] PRELOAD_SIPR = 3'd3;
localparam [2:0] PRELOAD_DIPR = 3'd4;

// ============================================================================
// Heartbeat
// ============================================================================
localparam T_HEARTBEAT = 26'd25000000;  // ~500ms @ 50MHz
reg [25:0] hb_cnt;
reg        hb_toggle;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hb_cnt    <= 26'd0;
        hb_toggle <= 1'b0;
    end else begin
        if (hb_cnt < T_HEARTBEAT)
            hb_cnt <= hb_cnt + 1'b1;
        else begin
            hb_cnt    <= 26'd0;
            hb_toggle <= ~hb_toggle;
        end
    end
end

// ============================================================================
// External Input Level Detection (12 inputs = sig_in[0..11])
// ----------------------------------------------------------------------------
// Debounced, active-high level sense. 2-FF synchronizer + a stable-candidate
// timer (~20ms @50MHz) rejects contact bounce/noise. The debounced 12-bit
// level is packed into in_status[11:0] and handed to w5500_receive, which
// pushes a status frame (AA 55 10 <lo> <hi> <xor>) to the PC whenever it
// changes (or on a new TCP connection).
// ============================================================================
localparam [19:0] IN_DEBOUNCE = 20'd1000000;   // ~20ms @ 50MHz
reg [11:0] in_s1, in_s2;      // 2-FF sync of async sig_in
reg [11:0] in_cand;           // current candidate level under debounce
reg [11:0] in_deb;            // debounced (stable) level
reg [19:0] in_deb_cnt;        // stable-candidate timer
wire [15:0] in_status = {4'd0, in_deb};

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        in_s1      <= 12'd0;
        in_s2      <= 12'd0;
        in_cand    <= 12'd0;
        in_deb     <= 12'd0;
        in_deb_cnt <= 20'd0;
    end else begin
        in_s1 <= sig_in;
        in_s2 <= in_s1;
        if (in_s2 != in_cand) begin
            in_cand    <= in_s2;      // new level -> restart debounce timer
            in_deb_cnt <= 20'd0;
        end else if (in_deb_cnt < IN_DEBOUNCE) begin
            in_deb_cnt <= in_deb_cnt + 1'b1;
        end else begin
            in_deb <= in_cand;        // candidate stable long enough -> accept
        end
    end
end

// ============================================================================
// 2KB Buffer RAM (inferred M9K Block RAM) — Single writer, synchronous read
// ============================================================================
// CRITICAL: Cyclone IV requires synchronous (registered) read to infer M9K
// Block RAM. Asynchronous read (assign buf_rd_data = buffer[addr]) would map
// the 2048×8 array to distributed logic (~16,000 LEs), exceeding the
// EP4CE10's 10,320 LE limit. With registered read, uses only 2 M9K blocks.
reg  [7:0]  buffer [0:2047];
wire [10:0] buf_rd_addr;    // from spi_master
reg  [7:0]  buf_rd_data;

always @(posedge sys_clk) begin
    buf_rd_data <= buffer[buf_rd_addr];
end

// ============================================================================
// Inter-module Wires
// ============================================================================
// ---- Shared SPI master command interface (MUXed between RX and SEND) ----
wire        spi_start;
wire        spi_cmd_rw;
wire [1:0]  spi_cmd_om;
wire [4:0]  spi_cmd_bsb;
wire [15:0] spi_cmd_addr;
wire [15:0] spi_cmd_wdata;
wire [10:0] spi_cmd_len;
wire [10:0] buf_wr_init;
wire [10:0] buf_rd_init;

// ---- SPI master status/data (broadcast to both controllers) ----
wire        spi_done;
wire        spi_busy;
wire [7:0]  spi_rx_data;
wire [7:0]  spi_rx_data_hi;   // FDM2 read high byte
wire [7:0]  debug_rx_buf_hi;  // DEBUG: direct access to rx_buf_hi

// ---- INIT controller (w5500_init) command outputs ----
wire        init_spi_start;
wire        init_spi_cmd_rw;
wire [1:0]  init_spi_cmd_om;
wire [4:0]  init_spi_cmd_bsb;
wire [15:0] init_spi_cmd_addr;
wire [15:0] init_spi_cmd_wdata;
wire [10:0] init_spi_cmd_len;
wire [10:0] init_buf_rd_init;
wire        init_done;         // high once init completes (permanent bus handoff)
wire        init_led_link;     // link indicator from init

// ---- RECV controller (w5500_receive) command outputs ----
wire        recv_spi_start;
wire        recv_spi_cmd_rw;
wire [1:0]  recv_spi_cmd_om;
wire [4:0]  recv_spi_cmd_bsb;
wire [15:0] recv_spi_cmd_addr;
wire [15:0] recv_spi_cmd_wdata;
wire [10:0] recv_spi_cmd_len;
wire [10:0] recv_buf_wr_init;
wire [10:0] recv_buf_rd_init;
wire        recv_led_activity; // RX activity indicator from receive
wire        recv_link_lost;    // 1-cycle: physical link lost (cable unplug)

// ---- SEND controller (w5500_send) command outputs ----
wire        send_spi_start;
wire        send_spi_cmd_rw;
wire [1:0]  send_spi_cmd_om;
wire [4:0]  send_spi_cmd_bsb;
wire [15:0] send_spi_cmd_addr;
wire [15:0] send_spi_cmd_wdata;
wire [10:0] send_spi_cmd_len;
wire [10:0] send_buf_rd_init;

// ---- Echo handshake between RECV and SEND controllers ----
wire        recv_rx_ready;    // RECV: data-ready pulse
wire [10:0] recv_rx_len;      // RECV: received length
wire        send_done_w;      // SEND: echo-complete pulse

// Buffer control from SPI master
wire [10:0] spi_buf_wr_addr;
wire        spi_buf_wr_en;

// Preload handshake from w5500_init
wire        preload_req;
wire [2:0]  preload_type;
reg         preload_req_d1;   // for rising-edge detection of preload_req

// (LED status now sourced directly: init_led_link + recv_led_activity)

// ============================================================================
// SPI Bus Arbiter: time-multiplex the single spi_master across THREE
// controllers (init, receive, send).
//   reset               -> BUS_INIT
//   BUS_INIT + init_done -> BUS_RECV   (one-time; never returns to INIT)
//   BUS_RECV + rx_ready  -> BUS_SEND   (kick off echo with recv_rx_len)
//   BUS_SEND + send_done -> BUS_RECV
// Ownership only switches during bus-idle gaps, so the command MUX never
// glitches an in-flight transaction.
// ============================================================================
localparam [1:0] BUS_INIT = 2'd0;
localparam [1:0] BUS_RECV = 2'd1;
localparam [1:0] BUS_SEND = 2'd2;

reg  [1:0]  bus_owner;
reg         rx_ready_d1;      // edge-detect recv_rx_ready
reg         send_start_r;     // 1-cycle pulse to w5500_send
reg  [10:0] send_len_r;       // latched length for w5500_send

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        bus_owner    <= BUS_INIT;
        rx_ready_d1  <= 1'b0;
        send_start_r <= 1'b0;
        send_len_r   <= 11'd0;
    end else begin
        rx_ready_d1  <= recv_rx_ready;
        send_start_r <= 1'b0;   // default: 1-cycle pulse

        case (bus_owner)
            BUS_INIT: begin
                // Wait for init to finish, then permanently grant to RECV.
                if (init_done)
                    bus_owner <= BUS_RECV;
            end

            BUS_RECV: begin
                // Physical link lost -> hand the bus back to INIT for a full
                // re-init (HW reset + reconfigure + re-LISTEN).
                if (recv_link_lost) begin
                    bus_owner <= BUS_INIT;
                end
                // RECV finished a receive -> hand the bus to SEND, start echo.
                else if (recv_rx_ready && !rx_ready_d1) begin
                    bus_owner    <= BUS_SEND;
                    send_start_r <= 1'b1;
                    send_len_r   <= recv_rx_len;
                end
            end

            BUS_SEND: begin
                // SEND finished -> hand the bus back to RECV.
                if (send_done_w)
                    bus_owner <= BUS_RECV;
            end

            default: bus_owner <= BUS_RECV;
        endcase
    end
end

// ---- Command MUX: drive spi_master from the current bus owner ----
assign spi_start     = (bus_owner == BUS_INIT) ? init_spi_start     :
                       (bus_owner == BUS_RECV) ? recv_spi_start     : send_spi_start;
assign spi_cmd_rw    = (bus_owner == BUS_INIT) ? init_spi_cmd_rw    :
                       (bus_owner == BUS_RECV) ? recv_spi_cmd_rw    : send_spi_cmd_rw;
assign spi_cmd_om    = (bus_owner == BUS_INIT) ? init_spi_cmd_om    :
                       (bus_owner == BUS_RECV) ? recv_spi_cmd_om    : send_spi_cmd_om;
assign spi_cmd_bsb   = (bus_owner == BUS_INIT) ? init_spi_cmd_bsb   :
                       (bus_owner == BUS_RECV) ? recv_spi_cmd_bsb   : send_spi_cmd_bsb;
assign spi_cmd_addr  = (bus_owner == BUS_INIT) ? init_spi_cmd_addr  :
                       (bus_owner == BUS_RECV) ? recv_spi_cmd_addr  : send_spi_cmd_addr;
assign spi_cmd_wdata = (bus_owner == BUS_INIT) ? init_spi_cmd_wdata :
                       (bus_owner == BUS_RECV) ? recv_spi_cmd_wdata : send_spi_cmd_wdata;
assign spi_cmd_len   = (bus_owner == BUS_INIT) ? init_spi_cmd_len   :
                       (bus_owner == BUS_RECV) ? recv_spi_cmd_len   : send_spi_cmd_len;
// buf_wr_init only meaningful for RECV (VDM read capture); INIT/SEND give 0.
assign buf_wr_init   = (bus_owner == BUS_RECV) ? recv_buf_wr_init   : 11'd0;
assign buf_rd_init   = (bus_owner == BUS_INIT) ? init_buf_rd_init   :
                       (bus_owner == BUS_RECV) ? recv_buf_rd_init   : send_buf_rd_init;

// ============================================================================
// Buffer Write Logic (SINGLE always block for all writes to buffer[])
// ============================================================================
// Three sources of buffer writes:
//   1. Preload: network constants before VDM init writes
//   2. SPI capture: data received from W5500 during VDM read (highest priority)
//   3. Test data: hardcoded test bytes from w5500_ctrl (medium priority)
//
// Preload serialization: 4~6 cycles (80~120ns @50MHz), well within
// the 10ms timer gap before the corresponding VDM write starts.

reg  [2:0]  preload_cnt;
reg         preload_busy;
reg  [2:0]  preload_type_reg;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        preload_req_d1   <= 1'b0;
        preload_busy     <= 1'b0;
        preload_cnt      <= 3'd0;
        preload_type_reg <= 2'd0;
    end else begin
        preload_req_d1 <= preload_req;

        // ---- Source 2: SPI VDM read capture (highest priority) ----
        if (spi_buf_wr_en) begin
            buffer[spi_buf_wr_addr] <= spi_rx_data;
        end
        // ---- Source 1: Preload network constants (sequential, 1 byte/cycle) ----
        else if (preload_busy) begin
            case (preload_type_reg)
                PRELOAD_GAR: begin
                    case (preload_cnt)
                        3'd0: buffer[preload_cnt] <= GATEWAY_IP[31:24];
                        3'd1: buffer[preload_cnt] <= GATEWAY_IP[23:16];
                        3'd2: buffer[preload_cnt] <= GATEWAY_IP[15:8];
                        3'd3: buffer[preload_cnt] <= GATEWAY_IP[7:0];
                        default: ;
                    endcase
                end
                PRELOAD_SUBR: begin
                    case (preload_cnt)
                        3'd0: buffer[preload_cnt] <= SUBNET_MASK[31:24];
                        3'd1: buffer[preload_cnt] <= SUBNET_MASK[23:16];
                        3'd2: buffer[preload_cnt] <= SUBNET_MASK[15:8];
                        3'd3: buffer[preload_cnt] <= SUBNET_MASK[7:0];
                        default: ;
                    endcase
                end
                PRELOAD_SHAR: begin
                    case (preload_cnt)
                        3'd0: buffer[preload_cnt] <= MAC_ADDR[47:40];
                        3'd1: buffer[preload_cnt] <= MAC_ADDR[39:32];
                        3'd2: buffer[preload_cnt] <= MAC_ADDR[31:24];
                        3'd3: buffer[preload_cnt] <= MAC_ADDR[23:16];
                        3'd4: buffer[preload_cnt] <= MAC_ADDR[15:8];
                        3'd5: buffer[preload_cnt] <= MAC_ADDR[7:0];
                        default: ;
                    endcase
                end
                PRELOAD_SIPR: begin
                    case (preload_cnt)
                        3'd0: buffer[preload_cnt] <= LOCAL_IP[31:24];
                        3'd1: buffer[preload_cnt] <= LOCAL_IP[23:16];
                        3'd2: buffer[preload_cnt] <= LOCAL_IP[15:8];
                        3'd3: buffer[preload_cnt] <= LOCAL_IP[7:0];
                        default: ;
                    endcase
                end
                PRELOAD_DIPR: begin
                    case (preload_cnt)
                        3'd0: buffer[preload_cnt] <= DIPR_IP[31:24];
                        3'd1: buffer[preload_cnt] <= DIPR_IP[23:16];
                        3'd2: buffer[preload_cnt] <= DIPR_IP[15:8];
                        3'd3: buffer[preload_cnt] <= DIPR_IP[7:0];
                        default: ;
                    endcase
                end
                default: ;
            endcase

            preload_cnt <= preload_cnt + 1'b1;

            // Stop: SHAR = 6 bytes (cnt 0..5), others = 4 bytes (cnt 0..3)
            if ((preload_type_reg == PRELOAD_SHAR && preload_cnt == 3'd5) ||
                (preload_type_reg != PRELOAD_SHAR && preload_cnt == 3'd3))
                preload_busy <= 1'b0;
        end
        // ---- Rising edge: start new preload ----
        else if (preload_req && !preload_req_d1) begin
            preload_busy     <= 1'b1;
            preload_cnt      <= 3'd0;
            preload_type_reg <= preload_type;
        end
    end
end

// ============================================================================
// Motor Pulse Command Decode (motors 1..4)
// ----------------------------------------------------------------------------
// The Python client sends framed commands (one per motor channel) that the
// W5500 receive path stores into buffer[0..rx_len-1]. Rather than re-read the
// buffer (whose single read port is owned by spi_master), we SNOOP the RX
// capture stream: every byte the spi_master writes to buffer[] during a
// BUS_RECV VDM read is mirrored into cmd_frame[]. On rx_ready we decode the
// frame and drive the matching pulse_gen. Frame formats (big-endian, XOR chk):
//   Fixed  (0x01, 11B): AA 55 01 <chan> [count:4] [freq_u:2] [xor(0..9)]
//   Cont.  (0x02,  7B): AA 55 02 <chan> [freq_u:2] [xor(0..5)]
//   Stop   (0x03,  5B): AA 55 03 <chan> [xor(0..3)]
// Channel byte (cmd_frame[3]) selects the motor:
//   0x01 -> motor 1, 0x08 -> motor 2, 0x0F -> motor 3, 0x10 -> motor 4.
// freq_u = freq_hz / 100 (1..2000). Echo of the received bytes is unaffected.
// ============================================================================
reg  [7:0]  cmd_frame [0:10];   // snooped first 11 bytes of the current RX frame

always @(posedge sys_clk) begin
    // Mirror each RX-captured byte (buffer[0..10]) into cmd_frame[].
    if (spi_buf_wr_en && (bus_owner == BUS_RECV) && (spi_buf_wr_addr <= 11'd10))
        cmd_frame[spi_buf_wr_addr] <= spi_rx_data;
end

// Combinational checksums over the snooped frame.
wire [7:0] chk_fixed = cmd_frame[0] ^ cmd_frame[1] ^ cmd_frame[2] ^ cmd_frame[3] ^
                       cmd_frame[4] ^ cmd_frame[5] ^ cmd_frame[6] ^ cmd_frame[7] ^
                       cmd_frame[8] ^ cmd_frame[9];
wire [7:0] chk_cont  = cmd_frame[0] ^ cmd_frame[1] ^ cmd_frame[2] ^
                       cmd_frame[3] ^ cmd_frame[4] ^ cmd_frame[5];
wire [7:0] chk_stop  = cmd_frame[0] ^ cmd_frame[1] ^ cmd_frame[2] ^ cmd_frame[3];
wire [7:0] chk_dir   = cmd_frame[0] ^ cmd_frame[1] ^ cmd_frame[2] ^
                       cmd_frame[3] ^ cmd_frame[4];

// Map the channel byte (cmd_frame[3]) to a motor index 0..7 (chan 0x01..0x08).
reg  [2:0] mtr_idx;
reg        mtr_valid;
always @(*) begin
    if (cmd_frame[3] >= 8'h01 && cmd_frame[3] <= 8'h08) begin
        mtr_idx   = cmd_frame[3] - 8'd1;   // 0x01->0 .. 0x08->7
        mtr_valid = 1'b1;
    end else begin
        mtr_idx   = 3'd0;
        mtr_valid = 1'b0;
    end
end

// ---- Per-motor pulse generator control (index 0=motor1 .. 7=motor8) ----
reg  [7:0]  pg_start;            // 1-cycle: start pulse job
reg  [7:0]  pg_stop;             // 1-cycle: stop immediately
reg  [7:0]  pg_continuous;       // 1=continuous, 0=fixed count
reg  [31:0] pg_count  [0:7];     // pulse count (fixed mode)
reg  [15:0] pg_freq_u [0:7];     // frequency in units of 100 Hz
reg  [7:0]  dir_reg;             // per-channel direction level (default HIGH)

assign dir_out = dir_reg;

integer i;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pg_start      <= 8'd0;
        pg_stop       <= 8'd0;
        pg_continuous <= 8'd0;
        dir_reg       <= 8'hFF;   // default HIGH (matches PC default)
        for (i = 0; i < 8; i = i + 1) begin
            pg_count[i]  <= 32'd0;
            pg_freq_u[i] <= 16'd0;
        end
    end else begin
        pg_start <= 8'd0;   // 1-cycle pulses (all motors)
        pg_stop  <= 8'd0;

        // recv_rx_ready fires with recv_rx_len valid; cmd_frame is settled
        // (all payload bytes were snooped before rx_ready in the RX FSM).
        if (recv_rx_ready && mtr_valid &&
            cmd_frame[0] == 8'hAA && cmd_frame[1] == 8'h55) begin
            case (cmd_frame[2])
                8'h01: begin   // fixed-count pulse
                    if (recv_rx_len >= 11'd11 && chk_fixed == cmd_frame[10]) begin
                        pg_count[mtr_idx]      <= {cmd_frame[4], cmd_frame[5],
                                                   cmd_frame[6], cmd_frame[7]};
                        pg_freq_u[mtr_idx]     <= {cmd_frame[8], cmd_frame[9]};
                        pg_continuous[mtr_idx] <= 1'b0;
                        pg_start[mtr_idx]      <= 1'b1;
                    end
                end
                8'h02: begin   // continuous pulse
                    if (recv_rx_len >= 11'd7 && chk_cont == cmd_frame[6]) begin
                        pg_freq_u[mtr_idx]     <= {cmd_frame[4], cmd_frame[5]};
                        pg_continuous[mtr_idx] <= 1'b1;
                        pg_start[mtr_idx]      <= 1'b1;
                    end
                end
                8'h03: begin   // stop
                    if (recv_rx_len >= 11'd5 && chk_stop == cmd_frame[4])
                        pg_stop[mtr_idx] <= 1'b1;
                end
                8'h04: begin   // direction: AA 55 04 <chan> <dir> <xor>
                    if (recv_rx_len >= 11'd6 && chk_dir == cmd_frame[5])
                        dir_reg[mtr_idx] <= cmd_frame[4][0];
                end
                default: ;
            endcase
        end
    end
end

// ============================================================================
// Module Instantiations
// ============================================================================

spi_master u_spi (
    .clk          (sys_clk),
    .rst_n        (sys_rst_n),

    .start        (spi_start),
    .cmd_rw       (spi_cmd_rw),
    .cmd_om       (spi_cmd_om),
    .cmd_bsb      (spi_cmd_bsb),
    .cmd_addr     (spi_cmd_addr),
    .cmd_wdata    (spi_cmd_wdata),
    .cmd_len      (spi_cmd_len),

    .buf_rd_data  (buf_rd_data),
    .buf_wr_addr  (spi_buf_wr_addr),
    .buf_rd_addr  (buf_rd_addr),
    .buf_wr_en    (spi_buf_wr_en),
    .buf_wr_init  (buf_wr_init),
    .buf_rd_init  (buf_rd_init),

    .done         (spi_done),
    .busy         (spi_busy),
    .rx_data      (spi_rx_data),
    .rx_data_hi   (spi_rx_data_hi),

    .spi_sclk     (w5500_sclk),
    .spi_mosi     (w5500_mosi),
    .spi_miso     (w5500_miso),
    .spi_cs       (w5500_scs),

    .debug_rx_buf_hi (debug_rx_buf_hi)
);

// ---- W5500 init controller: owns the bus first (BUS_INIT) ----
w5500_init u_init (
    .clk          (sys_clk),
    .rst_n        (sys_rst_n),

    .spi_start    (init_spi_start),
    .spi_cmd_rw   (init_spi_cmd_rw),
    .spi_cmd_om   (init_spi_cmd_om),
    .spi_cmd_bsb  (init_spi_cmd_bsb),
    .spi_cmd_addr (init_spi_cmd_addr),
    .spi_cmd_wdata(init_spi_cmd_wdata),
    .spi_cmd_len  (init_spi_cmd_len),
    .buf_rd_init  (init_buf_rd_init),
    .spi_done     (spi_done),
    .spi_busy     (spi_busy),
    .spi_rx_data     (spi_rx_data),
    .spi_rx_data_hi  (spi_rx_data_hi),

    .w5500_rst    (w5500_rst),

    .preload_req  (preload_req),
    .preload_type (preload_type),
    .preload_busy (preload_busy),

    .init_done    (init_done),
    .led_link     (init_led_link),
    .restart      (recv_link_lost)
);

// ---- W5500 receive controller: owns the bus after init (BUS_RECV) ----
w5500_receive u_recv (
    .clk          (sys_clk),
    .rst_n        (sys_rst_n),

    .spi_start    (recv_spi_start),
    .spi_cmd_rw   (recv_spi_cmd_rw),
    .spi_cmd_om   (recv_spi_cmd_om),
    .spi_cmd_bsb  (recv_spi_cmd_bsb),
    .spi_cmd_addr (recv_spi_cmd_addr),
    .spi_cmd_wdata(recv_spi_cmd_wdata),
    .spi_cmd_len  (recv_spi_cmd_len),
    .spi_done     (spi_done),
    .spi_busy     (spi_busy),
    .spi_rx_data     (spi_rx_data),
    .spi_rx_data_hi  (spi_rx_data_hi),

    .buf_wr_init  (recv_buf_wr_init),
    .buf_rd_init  (recv_buf_rd_init),

    .w5500_int    (w5500_int),
    .init_done    (init_done),

    .rx_ready     (recv_rx_ready),
    .rx_len       (recv_rx_len),
    .send_done    (send_done_w),

    .led_activity (recv_led_activity),
    .link_lost    (recv_link_lost),
    .in_status    (in_status)
);

// ---- W5500 send (echo) controller: shares spi_master via the arbiter ----
w5500_send u_send (
    .clk          (sys_clk),
    .rst_n        (sys_rst_n),

    .spi_start    (send_spi_start),
    .spi_cmd_rw   (send_spi_cmd_rw),
    .spi_cmd_om   (send_spi_cmd_om),
    .spi_cmd_bsb  (send_spi_cmd_bsb),
    .spi_cmd_addr (send_spi_cmd_addr),
    .spi_cmd_wdata(send_spi_cmd_wdata),
    .spi_cmd_len  (send_spi_cmd_len),
    .buf_rd_init  (send_buf_rd_init),
    .spi_done     (spi_done),
    .spi_busy     (spi_busy),
    .spi_rx_data     (spi_rx_data),
    .spi_rx_data_hi  (spi_rx_data_hi),

    .send_start   (send_start_r),
    .send_len     (send_len_r),
    .send_done    (send_done_w)
);

// ============================================================================
// Pulse Generators: command-driven, one per motor (see decode block above)
// ============================================================================
genvar g;
generate
    for (g = 0; g < 8; g = g + 1) begin : gen_pulse
        pulse_gen u_pulse_gen (
            .clk            (sys_clk),
            .rst_n          (sys_rst_n),
            .cmd_start      (pg_start[g]),
            .cmd_stop       (pg_stop[g]),
            .cmd_continuous (pg_continuous[g]),
            .cmd_count      (pg_count[g]),
            .cmd_freq_u     (pg_freq_u[g]),
            .pulse_out      (pulse_out[g])
        );
    end
endgenerate

// ============================================================================
// LED Output
// ============================================================================
assign led[0] = hb_toggle;         // Heartbeat (~1Hz)
assign led[1] = init_led_link;     // Link status (from init)
assign led[2] = recv_led_activity; // Activity (from receive)

endmodule
