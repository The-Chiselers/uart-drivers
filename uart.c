#include "uart.h"  // Use your header with the updated register definitions

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>


/*
 * uart_initialize()
 *
 * Opens /dev/mem, maps the UART core registers, and configures CTRLA and CTRLB.
 * This function is called by the graphics library during initialization.
 */
void uart_initialize(void) {
    printf("INIT UART start\n");
    // Open /dev/mem to access physical memory.

    uart_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (uart_fd < 0) {
        perror("uart_initialize: Error opening /dev/mem");
        exit(EXIT_FAILURE);
    }

    // Map a 4KB region starting at UART_BASE_ADDRESS.
    uart_base = mmap(NULL, UART_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                     uart_fd, UART_BASE_ADDRESS);
    if (uart_base == MAP_FAILED) {
        perror("uart_initialize: Error mapping UART registers");
        close(uart_fd);
        exit(EXIT_FAILURE);
    }

    uint32_t clk = 100000000;
    uint32_t baud = 115200;

    uart_tx_set_baudrate(clk, baud);
    usleep(10);
    uart_rx_set_baudrate(clk, baud);
    usleep(10);

    printf("Uart INIT DONE\n");
}

/*
 * uart_cleanup()
 *
 * Unmaps the UART registers and closes the /dev/mem file.
 * Call this function when the UART core is no longer needed.
 */
void uart_cleanup(void) {
    if (uart_base && uart_base != MAP_FAILED) {
        munmap((void*)uart_base, UART_MAP_SIZE);
        uart_base = NULL;
    }
    if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
    }
}

/* --------------------- Transmit Functions --------------------- */

void uart_tx_set_baudrate(uint32_t clock_freq, uint32_t baud_rate) {
    REGISTER_SET_VAL(uart_base, TX_CLOCKFREQ_OFFSET, clock_freq);
    REGISTER_SET_VAL(uart_base, TX_BAUDRATE_OFFSET, baud_rate);
    REGISTER_SET_VAL(uart_base, TX_UPDATEBAUD_OFFSET, 1);
}

void uart_tx_set_num_data_bits(uint8_t data_bits) {
    REGISTER_SET_VAL(uart_base, TX_NUMOUTPUTBITSDB_OFFSET, data_bits);
}

void uart_tx_set_use_parity(int use_parity) {
    REGISTER_SET_VAL(uart_base, TX_USEPARITYDB_OFFSET, use_parity ? 1 : 0);
}

void uart_tx_set_parity_odd(int parity_odd) {
    REGISTER_SET_VAL(uart_base, TX_PARITYODDDB_OFFSET, parity_odd ? 1 : 0);
}

void uart_tx_set_lsb_first(int lsb_first) {
    REGISTER_SET_VAL(uart_base, TX_LSBFIRST_OFFSET, lsb_first ? 1 : 0);
}

int uart_tx_add_byte_to_queue(uint8_t data) {
    if (REG(uart_base, TX_FIFOFULL_OFFSET)) return -1;
    REGISTER_SET_VAL(uart_base, TX_DATAIN_OFFSET, data);
    return 0;
}

void uart_tx_send_queue(uint8_t data) {
    REGISTER_SET_VAL(uart_base, TX_LOAD_OFFSET, 1);
}

int uart_tx_send_byte(uint8_t data) {
    if (REG(uart_base, TX_FIFOFULL_OFFSET)) return -1;
    REGISTER_SET_VAL(uart_base, TX_DATAIN_OFFSET, data);
    REGISTER_SET_VAL(uart_base, TX_LOAD_OFFSET, 1);
    return 0;
}

void uart_tx_set_almost_full_level(uint32_t almost_full_level) {
    REGISTER_SET_VAL(uart_base, TX_ALMOSTFULLLEVEL_OFFSET, almost_full_level);
}

int uart_tx_fifo_full(void) {
    return (REG(uart_base, TX_FIFOFULL_OFFSET) != 0);
}

int uart_tx_fifo_almost_full(void) {
    return (REG(uart_base, TX_FIFOALMOSTFULL_OFFSET) != 0);
}

void uart_tx_set_almost_empty_level(uint32_t almost_empty_level) {
    REGISTER_SET_VAL(uart_base, TX_ALMOSTEMPTYLEVEL_OFFSET, almost_empty_level);
}

int uart_tx_fifo_empty(void) {
    return (REG(uart_base, TX_FIFOEMPTY_OFFSET) != 0);
}

int uart_tx_fifo_almost_empty(void) {
    return (REG(uart_base, TX_FIFOALMOSTEMPTY_OFFSET) != 0);
}

void uart_tx_flush(void) {
    REGISTER_SET_VAL(uart_base, TX_FLUSH_OFFSET, 1);
}

void uart_tx_array(uint8_t* data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        // Attempt to add each byte to the queue.
        // If uart_tx_add_byte_to_queue returns -1 (FIFO full),
        // wait a short time and retry the same byte.
        while (uart_tx_add_byte_to_queue( data[i]) == -1) {
            usleep(1);
        }
    }
    printf("Sent %d bytes\n", length);
    // Once all bytes are queued, trigger the transmission.
    uart_tx_send_queue(1);
}

/* --------------------- Receiver Functions --------------------- */

void uart_rx_set_baudrate(uint32_t clock_freq,
                                        uint32_t baud_rate) {
    REGISTER_SET_VAL(uart_base, RX_CLOCKFREQ_OFFSET, clock_freq);
    REGISTER_SET_VAL(uart_base, RX_BAUDRATE_OFFSET, baud_rate);
    REGISTER_SET_VAL(uart_base, RX_UPDATEBAUD_OFFSET, 1);
}

void uart_rx_set_num_data_bits(uint8_t data_bits) {
    REGISTER_SET_VAL(uart_base, RX_NUMOUTPUTBITSDB_OFFSET, data_bits);
}

void uart_rx_set_use_parity(int use_parity) {
    REGISTER_SET_VAL(uart_base, RX_USEPARITYDB_OFFSET, use_parity ? 1 : 0);
}

void uart_rx_set_parity_odd(int parity_odd) {
    REGISTER_SET_VAL(uart_base, RX_PARITYODDDB_OFFSET, parity_odd ? 1 : 0);
}

void uart_rx_set_lsb_first(int lsb_first) {
    REGISTER_SET_VAL(uart_base, RX_LSBFIRST_OFFSET, lsb_first ? 1 : 0);
}

uint8_t uart_rx_read_byte(void) {
    return (uint8_t)REG(uart_base, RX_DATA_OFFSET);
}

uint8_t uart_rx_peek_byte(void) {
    return (uint8_t)REG(uart_base, RX_DATAPEEK_OFFSET);
}

int uart_rx_data_available(void) {
    return (REG(uart_base, RX_DATAAVAILABLE_OFFSET) != 0);
}

void uart_rx_set_almost_full_level( uint32_t almost_full_level) {
    REGISTER_SET_VAL(uart_base, RX_ALMOSTFULLLEVEL_OFFSET, almost_full_level);
}

int uart_rx_fifo_full(void) {
    return (REG(uart_base, RX_FIFOFULL_OFFSET) != 0);
}

int uart_rx_fifo_almost_full(void) {
    return (REG(uart_base, RX_FIFOALMOSTFULL_OFFSET) != 0);
}

void uart_rx_set_almost_empty_level(uint32_t almost_empty_level) {
    REGISTER_SET_VAL(uart_base, RX_ALMOSTEMPTYLEVEL_OFFSET, almost_empty_level);
}

int uart_rx_fifo_empty(void) {
    return (REG(uart_base, RX_FIFOEMPTY_OFFSET) != 0);
}

int uart_rx_fifo_almost_empty( void ) {
    return (REG(uart_base, RX_FIFOALMOSTEMPTY_OFFSET) != 0);
}

void uart_rx_flush(void) {
    REGISTER_SET_VAL(uart_base, RX_FLUSH_OFFSET, 1);
}

void uart_rx_read_array( uint8_t* response, uint16_t length) {
    uint16_t count = 0;
    // Continue reading while there is data available.
    while (uart_rx_data_available()) {
        // Ensure we don't exceed the provided buffer length.
        if (count < length) {
            response[count++] = uart_rx_read_byte();
        } else {
            break;
        }
    }
    return count;
}

/* --------------------- Common Functions --------------------- */

uint8_t uart_get_errors(void) {
    return (uint8_t)REG(uart_base, ERROR_OFFSET);
}

uint8_t uart_get_top_errors(void) {
    return (((uint8_t)REG(uart_base, ERROR_OFFSET)) >> 7) & 0x1;
}

uint8_t uart_get_rx_errors(void) {
    return (((uint8_t)REG(uart_base, ERROR_OFFSET)) >> 4) & 0x3;
}

uint8_t uart_get_tx_errors(void) {
    return (((uint8_t)REG(uart_base, ERROR_OFFSET)) >> 1) & 0x3;
}

uint8_t uart_get_addr_decode_errors(void) {
    return ((uint8_t)REG(uart_base, ERROR_OFFSET)) & 0x1;
}

void uart_clear_errors(void) {
    REGISTER_SET_VAL(uart_base, CLEARERROR_OFFSET, 1);
}

uint32_t uart_get_rx_clocks_per_bit(void) {
    return REG(uart_base, RX_CLOCKSPERBIT_OFFSET);
}

uint32_t uart_get_tx_clocks_per_bit(void) {
    return REG(uart_base, TX_CLOCKSPERBIT_OFFSET);
}
