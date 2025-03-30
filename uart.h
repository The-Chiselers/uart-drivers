#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>

// Base address and mapping size for your UART core
#define UART_BASE_ADDRESS 0x43C10000
#define UART_MAP_SIZE     0x1000    // 4KB page mapping

// Register offsets relative to the base address
#define TX_LOAD_OFFSET 0x0
#define TX_DATAIN_OFFSET 0x4
#define TX_BAUDRATE_OFFSET 0x8
#define TX_CLOCKFREQ_OFFSET 0xC
#define TX_UPDATEBAUD_OFFSET 0x10
#define TX_NUMOUTPUTBITSDB_OFFSET 0x14
#define TX_USEPARITYDB_OFFSET 0x18
#define TX_PARITYODDDB_OFFSET 0x1C
#define TX_ALMOSTEMPTYLEVEL_OFFSET 0x20
#define TX_ALMOSTFULLLEVEL_OFFSET 0x24
#define TX_FIFOFULL_OFFSET 0x28
#define TX_FIFOEMPTY_OFFSET 0x2C
#define TX_FIFOALMOSTEMPTY_OFFSET 0x30
#define TX_FIFOALMOSTFULL_OFFSET 0x34
#define RX_DATA_OFFSET 0x38
#define RX_DATAPEEK_OFFSET 0x3C
#define RX_DATAAVAILABLE_OFFSET 0x40
#define ERROR_OFFSET 0x44
#define CLEARERROR_OFFSET 0x48
#define RX_BAUDRATE_OFFSET 0x4C
#define RX_CLOCKFREQ_OFFSET 0x50
#define RX_UPDATEBAUD_OFFSET 0x54
#define RX_NUMOUTPUTBITSDB_OFFSET 0x58
#define RX_USEPARITYDB_OFFSET 0x5C
#define RX_PARITYODDDB_OFFSET 0x60
#define RX_ALMOSTEMPTYLEVEL_OFFSET 0x64
#define RX_ALMOSTFULLLEVEL_OFFSET 0x68
#define RX_FIFOFULL_OFFSET 0x6C
#define RX_FIFOEMPTY_OFFSET 0x70
#define RX_FIFOALMOSTEMPTY_OFFSET 0x74
#define RX_FIFOALMOSTFULL_OFFSET 0x78
#define RX_CLOCKSPERBIT_OFFSET 0x7C
#define TX_CLOCKSPERBIT_OFFSET 0x80
#define RX_LSBFIRST_OFFSET 0x84
#define TX_LSBFIRST_OFFSET 0x88
#define RX_FLUSH_OFFSET 0x8C
#define TX_FLUSH_OFFSET 0x90


// Macro to access a register: given a mapped base pointer and offset
#define REG(base, offset) (*(volatile uint8_t *)((char *)(base) + (offset)))

// Generic register set macro: read current value, clear bits (mask), then set new bits.
#define REGISTER_SET(base, offset, mask, shift, val) do { \
    REG(base, offset) = (REG(base, offset) & ~(mask)) | (((val) << (shift)) & (mask)); \
} while (0)

// Generic register set macro: read current value, clear bits (mask), then set new bits.
#define REGISTER_SET_VAL(base, offset, val) do { \
  REG(base, offset) =  (val); \
} while (0)



void uart_initialize(void);
void uart_cleanup(void);

/* --------------------- Transmitter Functions --------------------- */

static inline void uart_tx_set_baudrate(volatile void* base, uint32_t clock_freq, uint32_t baud_rate) {
    REGISTER_SET_VAL(base, TX_CLOCKFREQ_OFFSET, clock_freq);
    REGISTER_SET_VAL(base, TX_BAUDRATE_OFFSET, baud_rate);
    REGISTER_SET_VAL(base, TX_UPDATEBAUD_OFFSET, 1);
}

static inline void uart_tx_set_num_data_bits(volatile void* base, uint8_t data_bits) {
    REGISTER_SET_VAL(base, TX_NUMOUTPUTBITSDB_OFFSET, data_bits);
}

static inline void uart_tx_set_use_parity(volatile void* base,
                                          int use_parity) {
    REGISTER_SET_VAL(base, TX_USEPARITYDB_OFFSET, use_parity ? 1 : 0);
}

static inline void uart_tx_set_parity_odd(volatile void* base, int parity_odd) {
    REGISTER_SET_VAL(base, TX_PARITYODDDB_OFFSET, parity_odd ? 1 : 0);
}

static inline void uart_tx_set_lsb_first(volatile void* base, int lsb_first) {
    REGISTER_SET_VAL(base, TX_LSBFIRST_OFFSET, lsb_first ? 1 : 0);
}

static inline int uart_tx_add_byte_to_queue(volatile void* base, uint8_t data) {
    if (REG(base, TX_FIFOFULL_OFFSET)) return -1;
    REGISTER_SET_VAL(base, TX_DATAIN_OFFSET, data);
    return 0;
}

static inline void uart_tx_send_queue(volatile void* base, uint8_t data) {
    REGISTER_SET_VAL(base, TX_LOAD_OFFSET, 1);
}


static inline int uart_tx_send_byte(volatile void* base, uint8_t data) {
    if (REG(base, TX_FIFOFULL_OFFSET)) return -1;
    REGISTER_SET_VAL(base, TX_DATAIN_OFFSET, data);
    REGISTER_SET_VAL(base, TX_LOAD_OFFSET, 1);
    return 0;
}

static inline void uart_tx_set_almost_full_level(volatile void* base, uint32_t almost_full_level) {
    REGISTER_SET_VAL(base, TX_ALMOSTFULLLEVEL_OFFSET, almost_full_level);
}

static inline int uart_tx_fifo_full(volatile void* base) {
    return (REG(base, TX_FIFOFULL_OFFSET) != 0);
}

static inline int uart_tx_fifo_almost_full(volatile void* base) {
    return (REG(base, TX_FIFOALMOSTFULL_OFFSET) != 0);
}

static inline void uart_tx_set_almost_empty_level(volatile void* base, uint32_t almost_empty_level) {
    REGISTER_SET_VAL(base, TX_ALMOSTEMPTYLEVEL_OFFSET, almost_empty_level);
}

static inline int uart_tx_fifo_empty(volatile void* base) {
    return (REG(base, TX_FIFOEMPTY_OFFSET) != 0);
}

static inline int uart_tx_fifo_almost_empty(volatile void* base) {
    return (REG(base, TX_FIFOALMOSTEMPTY_OFFSET) != 0);
}

static inline void uart_tx_flush(volatile void* base) {
    REGISTER_SET_VAL(base, TX_FLUSH_OFFSET, 1);
}

/* --------------------- Receiver Functions --------------------- */

static inline void uart_rx_set_baudrate(volatile void* base, uint32_t clock_freq, uint32_t baud_rate) {
    REGISTER_SET_VAL(base, RX_CLOCKFREQ_OFFSET, clock_freq);
    REGISTER_SET_VAL(base, RX_BAUDRATE_OFFSET, baud_rate);
    REGISTER_SET_VAL(base, RX_UPDATEBAUD_OFFSET, 1);
}

static inline void uart_rx_set_num_data_bits(volatile void* base, uint8_t data_bits) {
    REGISTER_SET_VAL(base, RX_NUMOUTPUTBITSDB_OFFSET, data_bits);
}

static inline void uart_rx_set_use_parity(volatile void* base,
                                          int use_parity) {
    REGISTER_SET_VAL(base, RX_USEPARITYDB_OFFSET, use_parity ? 1 : 0);
}

static inline void uart_rx_set_parity_odd(volatile void* base, int parity_odd) {
    REGISTER_SET_VAL(base, RX_PARITYODDDB_OFFSET, parity_odd ? 1 : 0);
}

static inline void uart_rx_set_lsb_first(volatile void* base, int lsb_first) {
    REGISTER_SET_VAL(base, RX_LSBFIRST_OFFSET, lsb_first ? 1 : 0);
}

static inline uint8_t uart_rx_read_byte(volatile void* base) {
    return (uint8_t)REG(base, RX_DATA_OFFSET);
}

static inline uint8_t uart_rx_peek_byte(volatile void* base) {
    return (uint8_t)REG(base, RX_DATAPEEK_OFFSET);
}

static inline int uart_rx_data_available(volatile void* base) {
    return (REG(base, RX_DATAAVAILABLE_OFFSET) != 0);
}

static inline void uart_rx_set_almost_full_level(volatile void* base, uint32_t almost_full_level) {
    REGISTER_SET_VAL(base, RX_ALMOSTFULLLEVEL_OFFSET, almost_full_level);
}

static inline int uart_rx_fifo_full(volatile void* base) {
    return (REG(base, RX_FIFOFULL_OFFSET) != 0);
}

static inline int uart_rx_fifo_almost_full(volatile void* base) {
    return (REG(base, RX_FIFOALMOSTFULL_OFFSET) != 0);
}

static inline void uart_rx_set_almost_empty_level(volatile void* base, uint32_t almost_empty_level) {
    REGISTER_SET_VAL(base, RX_ALMOSTEMPTYLEVEL_OFFSET, almost_empty_level);
}

static inline int uart_rx_fifo_empty(volatile void* base) {
    return (REG(base, RX_FIFOEMPTY_OFFSET) != 0);
}

static inline int uart_rx_fifo_almost_empty(volatile void* base) {
    return (REG(base, RX_FIFOALMOSTEMPTY_OFFSET) != 0);
}

static inline void uart_rx_flush(volatile void* base) {
    REGISTER_SET_VAL(base, RX_FLUSH_OFFSET, 1);
}

/* --------------------- Common Functions --------------------- */

static inline uint8_t uart_get_errors(volatile void* base) {
    return (uint8_t)REG(base, ERROR_OFFSET);
}

static inline uint8_t uart_get_top_errors(volatile void* base) {
    return (((uint8_t)REG(base, ERROR_OFFSET)) >> 7) & 0x1;
}

static inline uint8_t uart_get_rx_errors(volatile void* base) {
    return (((uint8_t)REG(base, ERROR_OFFSET)) >> 4) & 0x3;
}

static inline uint8_t uart_get_tx_errors(volatile void* base) {
    return (((uint8_t)REG(base, ERROR_OFFSET)) >> 1) & 0x3;
}

static inline uint8_t uart_get_addr_decode_errors(volatile void* base) {
    return ((uint8_t)REG(base, ERROR_OFFSET)) & 0x1;
}



static inline void uart_clear_errors(volatile void* base) {
    REGISTER_SET_VAL(base, CLEARERROR_OFFSET, 1);
}

static inline uint32_t uart_get_rx_clocks_per_bit(volatile void* base) {
    return REG(base, RX_CLOCKSPERBIT_OFFSET);
}

static inline uint32_t uart_get_tx_clocks_per_bit(volatile void* base) {
    return REG(base, TX_CLOCKSPERBIT_OFFSET);
}

#endif /* UART_DRIVER_H */
