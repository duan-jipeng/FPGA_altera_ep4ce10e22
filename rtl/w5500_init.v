// ============================================================================
// Module: w5500_init
// Target: EP4CE10E22I7 (Cyclone IV E)
// Description:
//   W5500 one-time power-up initialization controller. Drives the SHARED
//   spi_master via its own copy of the command interface; the main.v arbiter
//   grants the SPI bus to this module first (BUS_INIT), and permanently hands
//   it off to the RX controller (w5500_receive) once init_done goes high.
//
//   Init phases (aligned with FPGA_W5500 reference):
//     1. HW reset -> PHY link check -> MR soft reset
//     2. Network params (GAR, SUBR, SHAR, SIPR via VDM + buffer preload)
//     3. Socket buffer size init -> RTR -> RCR -> SIMR/Sn_IMR
//     4. Socket config (Sn_MSSR -> Sn_PORT -> Sn_MR -> OPEN -> LISTEN)
//     5. S_INIT_DONE: assert init_done, release the SPI bus forever
//
//   Power-up retry (LISTEN fail) stays here (CLOSE + reconfigure socket).
//   Runtime reconnect (DISCON/TIMEOUT) is handled by w5500_receive.
// ============================================================================

module w5500_init (
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
    output reg  [10:0] buf_rd_init,    // Buffer read start address (VDM write)
    input  wire        spi_done,       // SPI transaction done pulse
    input  wire        spi_busy,       // SPI transaction in progress
    input  wire [7:0]  spi_rx_data,    // Last received byte (low byte for FDM2)
    input  wire [7:0]  spi_rx_data_hi, // FDM2 read only: high byte

    // ---- W5500 hardware control ----
    output reg         w5500_rst,      // W5500 hardware reset (active low)

    // ---- Preload handshake (buffer preload coordination) ----
    output reg         preload_req,    // pulse: request top-level to preload buffer
    output reg  [2:0]  preload_type,   // 000=GAR, 001=SUBR, 010=SHAR, 011=SIPR
    input  wire        preload_busy,   // 1=preload in progress, 0=done

    // ---- Init done + status ----
    output reg         init_done,      // held high once init completes (bus handoff)
    output reg         led_link,       // link indicator (high after PHY link up)

    // ---- Runtime restart (physical link lost, from w5500_receive) ----
    input  wire        restart          // 1-cycle pulse: drop init_done, full re-init
);

// ============================================================================
// Parameters: W5500 Register Addresses
// ============================================================================
localparam [15:0] MR       = 16'h0000;
localparam [15:0] GAR      = 16'h0001;
localparam [15:0] SUBR     = 16'h0005;
localparam [15:0] SHAR     = 16'h0009;
localparam [15:0] SIPR     = 16'h000F;
localparam [15:0] SIR      = 16'h0017;
localparam [15:0] SIMR     = 16'h0018;
localparam [15:0] IR       = 16'h0015;
localparam [15:0] IMR      = 16'h0016;
localparam [15:0] RTR      = 16'h0019;
localparam [15:0] RCR      = 16'h001B;
localparam [15:0] PHYCFGR  = 16'h002E;

localparam [15:0] Sn_MR         = 16'h0000;
localparam [15:0] Sn_CR         = 16'h0001;
localparam [15:0] Sn_IR         = 16'h0002;
localparam [15:0] Sn_SR         = 16'h0003;
localparam [15:0] Sn_PORT       = 16'h0004;
localparam [15:0] Sn_DIPR       = 16'h000C;
localparam [15:0] Sn_MSSR       = 16'h0012;
localparam [15:0] Sn_RXBUF_SIZE = 16'h001E;
localparam [15:0] Sn_TXBUF_SIZE = 16'h001F;
localparam [15:0] Sn_IMR        = 16'h002C;

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
localparam [7:0] MR_RST      = 8'h80;
localparam [7:0] PHY_LINK    = 8'h01;
localparam [7:0] Sn_MR_TCP   = 8'h01;
localparam [7:0] Sn_CR_OPEN  = 8'h01;
localparam [7:0] Sn_CR_LISTEN= 8'h02;
localparam [7:0] Sn_CR_CLOSE = 8'h10;
localparam [7:0] SOCK_INIT   = 8'h13;
localparam [7:0] SOCK_LISTEN = 8'h14;

// ============================================================================
// Parameters: Network Configuration
// ============================================================================
localparam [15:0] LOCAL_PORT  = 16'd5000;

// ============================================================================
// Parameters: Timing (50MHz clock)
// ============================================================================
localparam T_1MS       = 26'd50000;
localparam T_5MS       = 26'd250000;
localparam T_10MS      = 26'd500000;
localparam T_50MS      = 26'd2500000;
localparam T_200MS     = 26'd10000000;

// ============================================================================
// State Machine Encoding
// ============================================================================
localparam [6:0]
    S_IDLE              = 7'd0,
    S_HW_RST_LOW        = 7'd1,
    S_HW_RST_WAIT1      = 7'd2,
    S_HW_RST_HIGH       = 7'd3,
    S_HW_RST_WAIT2      = 7'd4,
    S_WAIT_LINK         = 7'd5,
    S_SW_RST1           = 7'd6,
    S_SW_RST2           = 7'd7,
    S_WR_GAR_0          = 7'd8,
    S_WR_SUBR_0         = 7'd9,
    S_WR_SHAR_0         = 7'd10,
    S_WR_SIPR_0         = 7'd11,
    S_BUF_INIT_START    = 7'd12,
    S_BUF_INIT_TX       = 7'd14,
    S_BUF_INIT_NEXT     = 7'd15,
    S_WR_RTR            = 7'd16,
    S_WR_RCR            = 7'd17,
    S_WR_SIMR           = 7'd18,
    S_SOCK_MSSR         = 7'd20,
    S_SOCK_PORT         = 7'd21,
    S_SOCK_OPEN_MR      = 7'd22,
    S_SOCK_OPEN_CR      = 7'd23,
    S_SOCK_OPEN_WAIT    = 7'd24,
    S_SOCK_CHECK_INIT   = 7'd25,
    S_SOCK_LISTEN_CR    = 7'd26,
    S_SOCK_LISTEN_WAIT  = 7'd27,
    S_SOCK_CHECK_LISTEN = 7'd28,
    S_SOCK_VERIFY_LISTEN= 7'd29,
    S_SOCK_CLOSE        = 7'd51,
    S_ERROR_WAIT        = 7'd52,
    S_RETRY_DELAY       = 7'd53,
    S_PRELOAD_VDM       = 7'd56,
    S_PRELOAD_WAIT      = 7'd59,
    S_FDM1_WAIT_BUSY    = 7'd63,
    S_BUF_INIT_DONE     = 7'd64,
    S_INIT_DONE         = 7'd70;   // init complete: assert init_done, release bus

// ============================================================================
// Internal Signals
// ============================================================================
reg  [6:0]  main_state;

// Timer
reg  [25:0] timer_cnt;
reg         timer_start;
reg  [25:0] timer_target;
wire        timer_done;

// SIMR back-to-back write flag
reg         simr_done;

// Link-up polled flag
reg         link_up;

// Socket buffer init loop counter (0-7)
reg  [2:0]  sock_cnt;

// VDM target: which network param to write (0=GAR,1=SUBR,2=SHAR,3=SIPR)
reg  [1:0]  vdm_target;

// FDM1 next state: where to go after SPI becomes busy
reg  [6:0]  fdm1_next;

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
// Main Init State Machine
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        main_state    <= S_IDLE;

        spi_start     <= 1'b0;
        spi_cmd_rw    <= RWB_READ;
        spi_cmd_om    <= OM_VDM;
        spi_cmd_bsb   <= BSB_COMMON;
        spi_cmd_addr  <= 16'd0;
        spi_cmd_wdata <= 16'd0;
        spi_cmd_len   <= 11'd0;
        buf_rd_init   <= 11'd0;

        timer_start   <= 1'b0;
        timer_target  <= 26'd0;

        simr_done     <= 1'b0;

        w5500_rst     <= 1'b1;
        link_up       <= 1'b0;

        preload_req   <= 1'b0;
        preload_type  <= 3'd0;

        init_done     <= 1'b0;
        led_link      <= 1'b0;

        sock_cnt      <= 3'd0;
        vdm_target    <= 2'd0;
        fdm1_next     <= S_IDLE;
    end else begin
        // Defaults
        spi_start   <= 1'b0;
        timer_start <= 1'b0;
        preload_req <= 1'b0;  // default low, asserted in preload states

        case (main_state)
            // ================================================================
            S_IDLE: begin
                led_link   <= 1'b0;
                main_state <= S_HW_RST_LOW;
            end

            // ================================================================
            // HARDWARE RESET
            // ================================================================
            S_HW_RST_LOW: begin
                w5500_rst    <= 1'b0;
                timer_target <= T_50MS;
                timer_start  <= 1'b1;
                main_state   <= S_HW_RST_WAIT1;
            end

            S_HW_RST_WAIT1: begin
                if (timer_done) main_state <= S_HW_RST_HIGH;
            end

            S_HW_RST_HIGH: begin
                w5500_rst    <= 1'b1;
                timer_target <= T_200MS;
                timer_start  <= 1'b1;
                main_state   <= S_HW_RST_WAIT2;
            end

            S_HW_RST_WAIT2: begin
                if (timer_done) main_state <= S_WAIT_LINK;
            end

            // ================================================================
            // WAIT LINK UP (blocking loop, matches STM32 while loop)
            // ================================================================
            S_WAIT_LINK: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM1;
                    spi_cmd_bsb  <= BSB_COMMON;
                    spi_cmd_addr <= PHYCFGR;
                    spi_start    <= 1'b1;
                    main_state   <= S_SW_RST1;
                end
            end

            // ================================================================
            // SOFTWARE RESET
            // ================================================================
            S_SW_RST1: begin
                if (spi_done) begin
                    if ((spi_rx_data & PHY_LINK) == 0) begin
                        main_state <= S_WAIT_LINK;
                    end else begin
                        link_up       <= 1'b1;
                        led_link      <= 1'b1;   // Link ON
                        spi_cmd_rw    <= RWB_WRITE;
                        spi_cmd_om    <= OM_FDM1;
                        spi_cmd_bsb   <= BSB_COMMON;
                        spi_cmd_addr  <= MR;
                        spi_cmd_wdata <= {8'd0, MR_RST};
                        spi_start     <= 1'b1;
                        main_state    <= S_SW_RST2;
                    end
                end
            end

            S_SW_RST2: begin
                if (spi_done) begin
                    timer_target <= T_10MS;
                    timer_start  <= 1'b1;
                    main_state   <= S_WR_GAR_0;
                end
            end

            // ================================================================
            // WRITE GAR (Gateway IP, 4 bytes via VDM)
            // ================================================================
            S_WR_GAR_0: begin
                preload_req  <= 1'b1;
                preload_type <= 3'd0;  // GAR

                if (timer_done) begin
                    preload_req  <= 1'b0;  // clear for clean rising edge in next state
                    spi_cmd_rw   <= RWB_WRITE;
                    spi_cmd_om   <= OM_VDM;
                    spi_cmd_bsb  <= BSB_COMMON;
                    spi_cmd_addr <= GAR;
                    spi_cmd_len  <= 11'd4;
                    spi_start    <= 1'b1;
                    buf_rd_init  <= 11'd0;
                    main_state   <= S_WR_SUBR_0;
                end
            end

            S_WR_SUBR_0: begin
                if (spi_done && !spi_busy) begin
                    preload_req  <= 1'b1;
                    preload_type <= 3'd1;  // SUBR
                    vdm_target   <= 2'd1;
                    main_state   <= S_PRELOAD_VDM;
                end
            end

            S_WR_SHAR_0: begin
                if (spi_done && !spi_busy) begin
                    preload_req  <= 1'b1;
                    preload_type <= 3'd2;  // SHAR
                    vdm_target   <= 2'd2;
                    main_state   <= S_PRELOAD_VDM;
                end
            end

            S_WR_SIPR_0: begin
                if (spi_done && !spi_busy) begin
                    preload_req  <= 1'b1;
                    preload_type <= 3'd3;  // SIPR
                    vdm_target   <= 2'd3;
                    sock_cnt     <= 3'd0;
                    main_state   <= S_PRELOAD_VDM;
                end
            end

            // Shared wait: after any FDM1 write, wait for SPI to actually begin.
            S_FDM1_WAIT_BUSY: begin
                if (spi_busy) begin
                    main_state <= fdm1_next;
                end
            end

            // ================================================================
            // PRELOAD VDM: wait for buffer preload to complete, then start SPI
            // ================================================================
            S_PRELOAD_VDM: begin
                preload_req <= 1'b0;      // clear after 1 cycle
                main_state  <= S_PRELOAD_WAIT;
            end

            S_PRELOAD_WAIT: begin
                if (!preload_busy) begin
                    spi_cmd_rw   <= RWB_WRITE;
                    spi_cmd_om   <= OM_VDM;
                    spi_cmd_bsb  <= BSB_COMMON;
                    spi_cmd_len  <= (vdm_target == 2'd2) ? 11'd6 : 11'd4;
                    spi_start    <= 1'b1;
                    buf_rd_init  <= 11'd0;

                    case (vdm_target)
                        2'd0:    spi_cmd_addr <= GAR;
                        2'd1:    spi_cmd_addr <= SUBR;
                        2'd2:    spi_cmd_addr <= SHAR;
                        2'd3:    spi_cmd_addr <= SIPR;
                        default: spi_cmd_addr <= GAR;
                    endcase

                    case (vdm_target)
                        2'd1:    main_state <= S_WR_SHAR_0;    // SUBR done -> SHAR
                        2'd2:    main_state <= S_WR_SIPR_0;    // SHAR done -> SIPR
                        2'd3: begin                            // SIPR done -> wait SPI busy
                            main_state <= S_FDM1_WAIT_BUSY;
                            fdm1_next  <= S_BUF_INIT_START;
                        end
                        default: main_state <= S_WR_SHAR_0;
                    endcase
                end
            end

            // ================================================================
            // SOCKET BUFFER SIZE INIT (i=0..7: Sn_RXBUF_SIZE=2, Sn_TXBUF_SIZE=2)
            // ================================================================
            S_BUF_INIT_START: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= {sock_cnt, 2'b01};  // Socket N BSB = (N<<2)+1
                    spi_cmd_addr  <= Sn_RXBUF_SIZE;
                    spi_cmd_wdata <= {8'd0, 8'h02};     // 2KB
                    spi_start     <= 1'b1;
                    main_state    <= S_BUF_INIT_TX;
                end
            end

            S_BUF_INIT_TX: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= {sock_cnt, 2'b01};
                    spi_cmd_addr  <= Sn_TXBUF_SIZE;
                    spi_cmd_wdata <= {8'd0, 8'h02};     // 2KB
                    spi_start     <= 1'b1;
                    main_state    <= S_BUF_INIT_NEXT;
                end
            end

            S_BUF_INIT_NEXT: begin
                if (spi_done && !spi_busy) begin
                    if (sock_cnt < 3'd7) begin
                        sock_cnt   <= sock_cnt + 1'b1;
                        main_state <= S_BUF_INIT_START;
                    end else begin
                        main_state <= S_BUF_INIT_DONE;
                    end
                end
            end

            S_BUF_INIT_DONE: begin
                if (!spi_busy && !spi_done) begin
                    main_state <= S_WR_RTR;
                end
            end

            // ================================================================
            // RTR -> RCR -> SIMR -> Sn_IMR
            // ================================================================
            S_WR_RTR: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM2;
                    spi_cmd_bsb   <= BSB_COMMON;
                    spi_cmd_addr  <= RTR;
                    spi_cmd_wdata <= 16'h07D0;  // 2000 = 200ms timeout
                    spi_start     <= 1'b1;
                    main_state    <= S_WR_RCR;
                end
            end

            S_WR_RCR: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_COMMON;
                    spi_cmd_addr  <= RCR;
                    spi_cmd_wdata <= {8'd0, 8'h08};
                    spi_start     <= 1'b1;
                    main_state    <= S_WR_SIMR;
                end
            end

            S_WR_SIMR: begin
                if (spi_done && !spi_busy) begin
                    if (!simr_done) begin
                        spi_cmd_rw    <= RWB_WRITE;
                        spi_cmd_om    <= OM_FDM1;
                        spi_cmd_bsb   <= BSB_COMMON;
                        spi_cmd_addr  <= SIMR;
                        spi_cmd_wdata <= {8'd0, 8'h01};
                        spi_start     <= 1'b1;
                        simr_done     <= 1'b1;
                    end else begin
                        spi_cmd_rw    <= RWB_WRITE;
                        spi_cmd_om    <= OM_FDM1;
                        spi_cmd_bsb   <= BSB_S0_REG;
                        spi_cmd_addr  <= Sn_IMR;
                        spi_cmd_wdata <= {8'd0, 8'h1F};
                        spi_start     <= 1'b1;
                        simr_done     <= 1'b0;
                        main_state    <= S_SOCK_MSSR;
                    end
                end
            end

            // ================================================================
            // Socket_Init: Sn_MSSR = 1460 (FDM2)
            // ================================================================
            S_SOCK_MSSR: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM2;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_MSSR;
                    spi_cmd_wdata <= 16'd1460;   // 0x05B4
                    spi_start     <= 1'b1;
                    main_state    <= S_SOCK_PORT;
                end
            end

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

            // ================================================================
            // Socket_Listen: Sn_MR = TCP (FDM1)
            // ================================================================
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
                    spi_cmd_wdata <= {8'd0, Sn_CR_OPEN};
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
                        main_state <= S_INIT_DONE;   // LISTEN OK -> hand off bus
                    else
                        main_state <= S_SOCK_CLOSE;
                end
            end

            // ================================================================
            // POWER-UP RETRY (LISTEN fail): CLOSE + reconfigure socket
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
                    // Re-enter socket config: Sn_MR -> Sn_PORT -> Sn_CR
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
            // INIT DONE: assert init_done and stay here. Drive NO spi_start so
            // the main.v arbiter can permanently grant the SPI bus to the RX
            // controller (w5500_receive).
            // ================================================================
            S_INIT_DONE: begin
                init_done <= 1'b1;
                // remain in S_INIT_DONE forever (bus released)
            end

            default: main_state <= S_IDLE;
        endcase

        // --------------------------------------------------------------------
        // TOP-PRIORITY OVERRIDE: runtime physical-link loss (from receive).
        // Overrides the case above (last non-blocking assignment wins): drop
        // the bus-handoff flag + link LED and restart the full power-up
        // sequence (HW reset -> wait PHY link -> reconfigure -> LISTEN).
        // --------------------------------------------------------------------
        if (restart) begin
            init_done  <= 1'b0;
            led_link   <= 1'b0;
            simr_done  <= 1'b0;
            sock_cnt   <= 3'd0;
            vdm_target <= 2'd0;
            fdm1_next  <= S_IDLE;
            spi_start  <= 1'b0;
            main_state <= S_IDLE;
        end
    end
end

endmodule
