#include "uart.h"  // Use your header with the updated register definitions
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MAX_INPUT_LENGTH 256

int main(void) {
    char buffer[MAX_INPUT_LENGTH];

    // Initialize UART
    uart_initialize();

    // Prompt the user for input
    printf("Enter text to send over UART (press Enter to finish):\n");
    if (fgets(buffer, sizeof(buffer), stdin) == NULL) {
        fprintf(stderr, "Error reading input.\n");
        uart_cleanup();
        return EXIT_FAILURE;
    }

    // Remove the trailing newline character if present
    size_t len = strlen(buffer);
    if (len > 0 && buffer[len - 1] == '\n') {
        buffer[len - 1] = '\0';
        len--;
    }

    // Send the entered text over UART
    uart_tx_array((uint8_t *)buffer, (uint16_t)len);
    printf("Sent %zu bytes over UART.\n", len);

    // Clean up UART resources before exiting
    uart_cleanup();
    return EXIT_SUCCESS;
}
