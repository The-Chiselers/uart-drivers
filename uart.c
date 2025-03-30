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
