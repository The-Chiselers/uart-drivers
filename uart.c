#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include "uart.h"  // Use your header with the updated register definitions

// Global variables for mmap and UART register base pointer.
static volatile void *uart_base = NULL;
static int uart_fd = -1;

/*
 * uart_initialize()
 *
 * Opens /dev/mem, maps the UART core registers, and configures CTRLA and CTRLB.
 * This function is called by the graphics library during initialization.
 */
void uart_initialize(void)
{
  // printf("INIT UART\n");
    // Open /dev/mem to access physical memory.
    uart_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (uart_fd < 0) {
        perror("uart_initialize: Error opening /dev/mem");
        exit(EXIT_FAILURE);
    }

    // Map a 4KB region starting at UART_BASE_ADDRESS.
    uart_base = mmap(NULL, UART_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, uart_fd, UART_BASE_ADDRESS);
    if (uart_base == MAP_FAILED) {
        perror("uart_initialize: Error mapping UART registers");
        close(uart_fd);
        exit(EXIT_FAILURE);
    }


    printf("INIT DONE\n");
}

/*
 * uart_cleanup()
 *
 * Unmaps the UART registers and closes the /dev/mem file.
 * Call this function when the UART core is no longer needed.
 */
void uart_cleanup(void)
{
    if (uart_base && uart_base != MAP_FAILED) {
        munmap((void *)uart_base, UART_MAP_SIZE);
        uart_base = NULL;
    }
    if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
    }
}

/* --------------------- Transmit Functions --------------------- */

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