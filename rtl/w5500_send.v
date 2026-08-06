// ============================================================================
// Module: w5500_send
// Target: EP4CE10E22I7 (Cyclone IV E)
// Description:
//   W5500 Socket0 TX (send) controller. Drives the SHARED spi_master via its
//   own copy of the command interface; the main.v arbiter time-multiplexes
//   the SPI bus between this module and w5500_ctrl (the RX controller).
//
//   On a send_start pulse (with send_len), echoes buffer[0..send_len-1] to the
//   W5500 Socket0 TX buffer, mirroring STM32 Write_SOCK_Data_Buffer (W5500.c):
//     1. offset = Read_W5500_SOCK_2Byte(s, Sn_TX_WR);   [FDM2 read]
//     2. offset &= (S_TX_SIZE - 1);
//     3. VDM write data to TXBUF at offset (split on wrap)
//     4. Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset + size); [FDM2 write]
//     5. Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);            [FDM1 write]
//   When done, pulses send_done for one cycle and returns to idle.
// ============================================================================

module w5500_send (
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

    // ---- Echo handshake (from w5500_ctrl via main arbiter) ----
    input  wire        send_start,     // 1-cycle pulse: begin echo of send_len bytes
    input  wire [10:0] send_len,       // number of bytes to send (from buffer[0..])
    output reg         send_done       // 1-cycle pulse: echo complete
);

// ============================================================================
// Parameters: W5500 Register Addresses / Control Byte Components
// ============================================================================
localparam [15:0] Sn_CR    = 16'h0001;
localparam [15:0] Sn_TX_WR = 16'h0024;

localparam      RWB_READ  = 1'b0;
localparam      RWB_WRITE = 1'b1;
localparam [1:0] OM_VDM   = 2'b00;
localparam [1:0] OM_FDM1  = 2'b01;
localparam [1:0] OM_FDM2  = 2'b10;

localparam [4:0] BSB_S0_REG   = 5'd1;   // Socket0 register block
localparam [4:0] BSB_S0_TXBUF = 5'd2;   // Socket0 TX buffer block

localparam [7:0] Sn_CR_SEND = 8'h20;
localparam [10:0] TX_SIZE   = 11'd2048; // Socket0 TX buffer size (default 2KB)

// ============================================================================
// State Machine Encoding
// ============================================================================
localparam [2:0]
    S_IDLE      = 3'd0,
    S_RD_TXWR   = 3'd1,   // issue FDM2 read Sn_TX_WR
    S_TXWR_WAIT = 3'd2,   // capture tx_wr_ptr, issue VDM write (chunk1)
    S_WR_DATA2  = 3'd3,   // (split only) issue VDM write chunk2
    S_UPDATE_WR = 3'd4,   // issue FDM2 write Sn_TX_WR = ptr + len
    S_SEND_CMD  = 3'd5,   // issue FDM1 write Sn_CR = SEND
    S_FINISH    = 3'd6;   // pulse send_done

// ============================================================================
// Internal Signals
// ============================================================================
reg  [2:0]  state;
reg  [10:0] send_len_reg;   // latched length to echo
reg  [15:0] tx_wr_ptr;      // raw Sn_TX_WR pointer read from W5500
reg  [10:0] tx_offset;      // tx_wr_ptr & (TX_SIZE-1)
reg         need_split;     // 1 if (tx_offset + len) wraps past TX_SIZE

// ============================================================================
// Main State Machine
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state         <= S_IDLE;
        spi_start     <= 1'b0;
        spi_cmd_rw    <= RWB_READ;
        spi_cmd_om    <= OM_FDM2;
        spi_cmd_bsb   <= BSB_S0_REG;
        spi_cmd_addr  <= 16'd0;
        spi_cmd_wdata <= 16'd0;
        spi_cmd_len   <= 11'd0;
        buf_rd_init   <= 11'd0;
        send_done     <= 1'b0;
        send_len_reg  <= 11'd0;
        tx_wr_ptr     <= 16'd0;
        tx_offset     <= 11'd0;
        need_split    <= 1'b0;
    end else begin
        // Defaults: 1-cycle pulses auto-clear
        spi_start <= 1'b0;
        send_done <= 1'b0;

        case (state)
            // ============================================================
            // IDLE: wait for send_start, latch length.
            // ============================================================
            S_IDLE: begin
                if (send_start) begin
                    send_len_reg <= send_len;
                    state        <= S_RD_TXWR;
                end
            end

            // ============================================================
            // Read Sn_TX_WR (FDM2): SPI frame [00 24 0A xx xx].
            // Guard on !busy && !done so we do not collide with the tail of
            // the RX controller's last transaction before the bus was granted.
            // ============================================================
            S_RD_TXWR: begin
                if (!spi_busy && !spi_done) begin
                    spi_cmd_rw   <= RWB_READ;
                    spi_cmd_om   <= OM_FDM2;
                    spi_cmd_bsb  <= BSB_S0_REG;
                    spi_cmd_addr <= Sn_TX_WR;
                    spi_start    <= 1'b1;
                    state        <= S_TXWR_WAIT;
                end
            end

            // ============================================================
            // Capture Sn_TX_WR, compute offset/split, issue VDM write chunk1.
            //   offset  = ptr & (TX_SIZE-1)
            //   split   = (offset + len) > TX_SIZE
            //   chunk1 len = split ? (TX_SIZE - offset) : len, from buffer[0]
            // tx_offset stored as 16-bit into spi_cmd_addr directly (big-endian
            // send order handled by spi_master: addr[15:8] then addr[7:0]).
            // ============================================================
            S_TXWR_WAIT: begin
                if (spi_done && !spi_busy) begin
                    tx_wr_ptr  <= {spi_rx_data_hi, spi_rx_data};
                    tx_offset  <= {spi_rx_data_hi, spi_rx_data} & 16'h07FF;
                    need_split <= (({spi_rx_data_hi, spi_rx_data} & 16'h07FF)
                                   + {5'd0, send_len_reg} > 16'd2048);

                    // Issue VDM write chunk1 to TXBUF at offset.
                    spi_cmd_rw   <= RWB_WRITE;
                    spi_cmd_om   <= OM_VDM;
                    spi_cmd_bsb  <= BSB_S0_TXBUF;
                    spi_cmd_addr <= {spi_rx_data_hi, spi_rx_data} & 16'h07FF;
                    spi_cmd_len  <= (({spi_rx_data_hi, spi_rx_data} & 16'h07FF)
                                     + {5'd0, send_len_reg} > 16'd2048)
                                    ? (TX_SIZE - {spi_rx_data_hi[2:0], spi_rx_data})
                                    : send_len_reg;
                    buf_rd_init  <= 11'd0;
                    spi_start    <= 1'b1;
                    state        <= (({spi_rx_data_hi, spi_rx_data} & 16'h07FF)
                                     + {5'd0, send_len_reg} > 16'd2048)
                                    ? S_WR_DATA2 : S_UPDATE_WR;
                end
            end

            // ============================================================
            // (split only) VDM write chunk2 to TXBUF at address 0.
            //   chunk2 len = len - (TX_SIZE - offset)
            //   buffer read continues at (TX_SIZE - offset)
            // ============================================================
            S_WR_DATA2: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw   <= RWB_WRITE;
                    spi_cmd_om   <= OM_VDM;
                    spi_cmd_bsb  <= BSB_S0_TXBUF;
                    spi_cmd_addr <= 16'd0;
                    spi_cmd_len  <= send_len_reg - (TX_SIZE - tx_offset);
                    buf_rd_init  <= TX_SIZE - tx_offset;
                    spi_start    <= 1'b1;
                    state        <= S_UPDATE_WR;
                end
            end

            // ============================================================
            // Update Sn_TX_WR = ptr + len (FDM2 write). W5500 advances the
            // TX write pointer; SEND then transmits [old_ptr .. new_ptr).
            // ============================================================
            S_UPDATE_WR: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM2;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_TX_WR;
                    spi_cmd_wdata <= tx_wr_ptr + {5'd0, send_len_reg};
                    spi_start     <= 1'b1;
                    state         <= S_SEND_CMD;
                end
            end

            // ============================================================
            // Issue SEND command (FDM1 write Sn_CR = 0x20).
            // ============================================================
            S_SEND_CMD: begin
                if (spi_done && !spi_busy) begin
                    spi_cmd_rw    <= RWB_WRITE;
                    spi_cmd_om    <= OM_FDM1;
                    spi_cmd_bsb   <= BSB_S0_REG;
                    spi_cmd_addr  <= Sn_CR;
                    spi_cmd_wdata <= {8'd0, Sn_CR_SEND};
                    spi_start     <= 1'b1;
                    state         <= S_FINISH;
                end
            end

            // ============================================================
            // SEND command write complete: pulse send_done, release bus.
            // ============================================================
            S_FINISH: begin
                if (spi_done && !spi_busy) begin
                    send_done <= 1'b1;
                    state     <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
