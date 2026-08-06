// ============================================================================
// Module: spi_master
// Target: EP4CE10E22I7 (Cyclone IV E)
// Description:
//   SPI Master for W5500 communication.
//   Mode 0 (CPOL=0, CPHA=0), MSB first.
//   Timing matches STM32 GPIO bit-bang SPI1_Send_Byte exactly:
//     1. Set MOSI while SCLK low
//     2. SCLK goes high (slave samples MOSI)
//     3. SCLK goes low
//   MOSI output is registered on negedge for analyzer setup-time margin.
// ============================================================================

module spi_master (
    input  wire        clk,            // 50MHz system clock
    input  wire        rst_n,          // active-low reset

    // ---- Command interface (set by controller before pulsing start) ----
    input  wire        start,          // pulse to start transaction
    input  wire        cmd_rw,         // 0=read, 1=write
    input  wire [1:0]  cmd_om,         // Operation mode: 00=VDM, 01=FDM1, 10=FDM2
    input  wire [4:0]  cmd_bsb,        // Block Select Bits (raw, pre-shift)
    input  wire [15:0] cmd_addr,       // 16-bit register offset address
    input  wire [15:0] cmd_wdata,      // Write data (for FDM1/FDM2 modes)
    input  wire [10:0] cmd_len,        // Data length in bytes (for VDM mode)

    // ---- Buffer interface (for VDM data transfers) ----
    input  wire [7:0]  buf_rd_data,    // Buffer read data (VDM write)
    output reg  [10:0] buf_wr_addr,    // Buffer write address (VDM read)
    output reg  [10:0] buf_rd_addr,    // Buffer read address  (VDM write)
    output reg         buf_wr_en,      // Buffer write enable pulse (VDM read)
    input  wire [10:0] buf_wr_init,    // Initial buffer write address (loaded on start)
    input  wire [10:0] buf_rd_init,    // Initial buffer read address  (loaded on start)

    // ---- Status ----
    output wire        done,           // 1-cycle pulse when transaction complete
    output wire        busy,           // High during transaction
    output wire [7:0]  rx_data,        // Last received data byte (low byte for FDM2)
    output wire [7:0]  rx_data_hi,     // FDM2 read only: high byte (others: stale)

    // ---- W5500 SPI pins ----
    output wire        spi_sclk,       // SPI clock
    output wire        spi_mosi,       // SPI MOSI (master out, slave in)
    input  wire        spi_miso,       // SPI MISO (master in, slave out)
    output wire        spi_cs,          // SPI chip select (active low)

    // DEBUG: expose rx_buf_hi for logic analyzer
    output wire [7:0]  debug_rx_buf_hi
);

// ============================================================================
// Parameters
// ============================================================================
localparam SPI_DIV        = 8'd1;      // SCLK = 50MHz / (SPI_DIV+1) = 25MHz (1 low + 1 high cycle, 50% duty)
localparam CS_IDLE_CYCLES = 6'd50;     // Min CS high cycles between frames
localparam OM_VDM         = 2'b00;
localparam OM_FDM1        = 2'b01;
localparam OM_FDM2        = 2'b10;
localparam OM_FDM4        = 2'b11;  // Fixed 4-byte data mode

localparam S_IDLE = 2'd0;
localparam S_LOW  = 2'd1;   // SCLK low: MOSI set, wait, then go high
localparam S_HIGH = 2'd2;   // SCLK high: sample MISO, then go low
localparam S_DONE = 2'd3;   // Transaction complete

// ============================================================================
// Internal signals
// ============================================================================
reg  [1:0]  state;
reg  [2:0]  bit_idx;        // Bit index within current byte (0-7)
reg  [10:0] byte_cnt;       // Byte counter within transaction
reg  [10:0] total_bytes;    // Total bytes for this transaction

reg         sclk_reg;       // SCLK output register
reg         busy_reg;       // Busy flag (spi_cs = ~busy_reg)
reg         done_reg;       // Done pulse

reg  [7:0]  clk_cnt;        // Clock divider counter (counts to SPI_DIV)
reg  [5:0]  cs_idle_cnt;    // CS high hold-off counter

reg         start_latch;    // Latches start pulse until SPI begins

reg  [7:0]  tx_byte;        // Current byte being shifted out
reg  [7:0]  rx_byte;        // Current byte being shifted in
reg  [7:0]  rx_buf;         // Captured last received byte (low byte for FDM2)
reg  [7:0]  rx_buf_hi;      // FDM2 read only: captured high byte (byte_cnt=4)

wire [7:0]  rx_byte_next;   // Combinational: rx_byte after shifting in current MISO
assign rx_byte_next = {rx_byte[6:0], spi_miso};

reg         mosi_reg;       // Registered MOSI output (negedge capture)

wire [7:0]  ctrl_byte;

// ============================================================================
// Control byte construction: {BSB[4:0], RWB, OM[1:0]}
// ============================================================================
assign ctrl_byte = {cmd_bsb, cmd_rw, cmd_om};

// ============================================================================
// MOSI registered on negedge: MOSI updates at system clock falling edge,
// SCLK transitions at system clock rising edge → half-cycle offset.
// This guarantees the logic analyzer sees stable MOSI before SCLK rising edge.
// ============================================================================
always @(negedge clk or negedge rst_n) begin
    if (!rst_n)
        mosi_reg <= 1'b0;
    else
        mosi_reg <= tx_byte[7];
end

// ============================================================================
// Output assignments
// ============================================================================
assign spi_sclk = sclk_reg;
assign spi_cs   = ~busy_reg;
assign spi_mosi = mosi_reg;

assign busy       = busy_reg;
assign done       = done_reg;
assign rx_data    = rx_buf;       // 8-bit: last byte received
assign rx_data_hi = rx_buf_hi;    // 8-bit: FDM2 high byte only
assign debug_rx_buf_hi = rx_buf_hi;  // DEBUG output

// ============================================================================
// Helper: Get the byte to transmit for a given byte_cnt
// ============================================================================
function [7:0] get_tx_byte;
    input [10:0] bc;
begin
    case (bc)
        11'd0:  get_tx_byte = cmd_addr[15:8];    // Address high
        11'd1:  get_tx_byte = cmd_addr[7:0];     // Address low
        11'd2:  get_tx_byte = ctrl_byte;          // Control byte
        default: begin
            if (cmd_om == OM_VDM || cmd_om == OM_FDM4) begin
                if (cmd_rw == 1'b1)
                    get_tx_byte = buf_rd_data;    // VDM/FDM4 write: from buffer
                else
                    get_tx_byte = 8'h00;          // VDM/FDM4 read: dummy = 0x00 (matches STM32)
            end else if (cmd_om == OM_FDM2) begin
                if (cmd_rw == 1'b0)
                    get_tx_byte = 8'h00;          // FDM2 read: dummy = 0x00 (matches STM32)
                else if (bc == 11'd3)
                    get_tx_byte = cmd_wdata[15:8]; // FDM2 write: MSB
                else if (bc == 11'd4)
                    get_tx_byte = cmd_wdata[7:0];  // FDM2 write: LSB
                else
                    get_tx_byte = 8'hFF;
            end else begin // FDM1
                if (cmd_rw == 1'b0)
                    get_tx_byte = 8'h00;           // FDM1 read: dummy = 0x00 (matches STM32)
                else if (bc == 11'd3)
                    get_tx_byte = cmd_wdata[7:0];  // FDM1 write: data
                else
                    get_tx_byte = 8'hFF;
            end
        end
    endcase
end
endfunction

// ============================================================================
// Main SPI State Machine
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        bit_idx     <= 3'd0;
        byte_cnt    <= 11'd0;
        total_bytes <= 11'd0;
        sclk_reg    <= 1'b0;
        busy_reg    <= 1'b0;
        done_reg    <= 1'b0;
        tx_byte     <= 8'd0;
        rx_byte     <= 8'd0;
        rx_buf      <= 8'd0;
        clk_cnt     <= 8'd0;
        cs_idle_cnt <= 6'd0;
        start_latch <= 1'b0;
        buf_wr_addr <= 11'd0;
        buf_rd_addr <= 11'd0;
        buf_wr_en   <= 1'b0;
    end else begin
        done_reg  <= 1'b0;
        buf_wr_en <= 1'b0;

        // CS hold-off: decrement in idle
        if (state == S_IDLE && cs_idle_cnt > 0)
            cs_idle_cnt <= cs_idle_cnt - 1'b1;

        // Start latch: remember start request even if cs_idle_cnt > 0
        if (start)
            start_latch <= 1'b1;
        if (busy_reg)
            start_latch <= 1'b0;

        case (state)
            // ============================================================
            // IDLE: Wait for start pulse
            // ============================================================
            S_IDLE: begin
                sclk_reg <= 1'b0;
                if ((start || start_latch) && cs_idle_cnt == 6'd0) begin
                    busy_reg    <= 1'b1;
                    total_bytes <= (cmd_om == OM_VDM)   ? (3 + cmd_len) :
                                   (cmd_om == OM_FDM4)  ? (3 + cmd_len) :
                                   (cmd_om == OM_FDM1)  ? 11'd4 :
                                   (cmd_om == OM_FDM2)  ? 11'd5 : 11'd3;
                    // Load first byte
                    tx_byte     <= cmd_addr[15:8];
                    bit_idx     <= 3'd0;
                    byte_cnt    <= 11'd1;     // next byte will be #1
                    buf_wr_addr <= buf_wr_init - 1'b1;
                    buf_rd_addr <= buf_rd_init;
                    clk_cnt     <= 8'd0;
                    state       <= S_LOW;
                end
            end

            // ============================================================
            // SCLK LOW: MOSI already set (via negedge register).
            //           Wait for SCLK low period, then go to SCLK HIGH.
            // ============================================================
            S_LOW: begin
                sclk_reg <= 1'b0;
                if (clk_cnt >= SPI_DIV - 1) begin
                    clk_cnt  <= 8'd0;
                    sclk_reg <= 1'b1;
                    state    <= S_HIGH;
                end else begin
                    clk_cnt <= clk_cnt + 1'b1;
                end
            end

            // ============================================================
            // SCLK HIGH: Sample MISO. Then SCLK goes low.
            //            Shift tx_byte, advance bit/byte.
            // ============================================================
            S_HIGH: begin
                sclk_reg <= 1'b1;
                // Sample MISO on this cycle (SCLK is high)
                // Use combinational rx_byte_next so rx_buf gets the complete byte
                rx_byte <= rx_byte_next;

                // SCLK falls
                sclk_reg <= 1'b0;
                clk_cnt  <= 8'd0;

                if (bit_idx < 3'd7) begin
                    // ---- More bits in this byte ----
                    bit_idx <= bit_idx + 1'b1;
                    tx_byte <= {tx_byte[6:0], 1'b0};   // shift left (MSB first)
                    state   <= S_LOW;
                end else begin
                    // ---- Byte complete (8 bits sent) ----
                    rx_buf <= rx_byte_next;   // save complete received byte

                    // FDM2 read: capture the HIGH byte.
                    //
                    // CRITICAL byte_cnt alignment: byte_cnt starts at 1 (see
                    // S_IDLE init), so when completing physical byte p, the
                    // counter equals p+1. The 5-byte FDM2 frame is:
                    //   p0=addr_hi p1=addr_lo p2=ctrl p3=data_hi p4=data_lo
                    // MISO carries valid data only on p3 (data_hi) and
                    // p4 (data_lo). Completing p3 happens when byte_cnt==4.
                    //
                    // FIX: was byte_cnt==3 (captured MISO during the CONTROL
                    // byte p2 = dummy garbage). Now byte_cnt==4 captures the
                    // real data high byte at p3. Low byte lands in rx_buf at
                    // p4 (byte_cnt==5) via the unconditional rx_buf assignment.
                    if (cmd_om == OM_FDM2 && byte_cnt == 11'd4)
                        rx_buf_hi <= rx_byte_next;

                    // ----------------------------------------------------
                    // VDM/FDM4 READ capture (SAME byte_cnt+1 alignment as
                    // the FDM2 high-byte fix above). Real data bytes are
                    // p3..p(total-1); completing byte p => byte_cnt==p+1,
                    // so data completions land at byte_cnt==4..total_bytes.
                    //
                    // This block is deliberately OUTSIDE the
                    // "byte_cnt < total_bytes" branch below. The old code
                    // asserted buf_wr_en at byte_cnt>=3 INSIDE that branch,
                    // which (a) captured the dummy MISO of the CONTROL byte
                    // (p2) into buffer[0] -> "first echo byte is 0x03" bug,
                    // and (b) dropped the final data byte (byte_cnt==
                    // total_bytes never entered the load branch). Capturing
                    // at byte_cnt>=4 including the last byte fixes both.
                    if ((cmd_om == OM_VDM || cmd_om == OM_FDM4) &&
                        cmd_rw == 1'b0 && byte_cnt >= 11'd4) begin
                        buf_wr_en   <= 1'b1;
                        buf_wr_addr <= buf_wr_addr + 1'b1;
                    end

                    if (byte_cnt < total_bytes) begin
                        // ---- Load next byte ----
                        tx_byte  <= get_tx_byte(byte_cnt);
                        bit_idx  <= 3'd0;
                        byte_cnt <= byte_cnt + 1'b1;

                        // VDM/FDM4 WRITE: advance buffer read address for
                        // the next data byte (unchanged from original).
                        if ((cmd_om == OM_VDM || cmd_om == OM_FDM4) &&
                            cmd_rw == 1'b1 && byte_cnt >= 11'd3) begin
                            buf_rd_addr <= buf_rd_addr + 1'b1;
                        end

                        state <= S_LOW;
                    end else begin
                        // ---- All bytes done ----
                        state <= S_DONE;
                    end
                end
            end

            // ============================================================
            // DONE: Deassert CS, pulse done signal
            // ============================================================
            S_DONE: begin
                done_reg    <= 1'b1;
                busy_reg    <= 1'b0;
                cs_idle_cnt <= CS_IDLE_CYCLES;
                state       <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
