#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdint.h>

// Base address and mapping size for your SPI core
#define GPIO_BASE_ADDRESS 0x43C00000
#define GPIO_MAP_SIZE     0x1000    // 4KB page mapping

// Macro to access the GPIO register (assuming a single 32-bit register at offset 0)
#define GPIO_REG(base) (*(volatile uint8_t *)((char *)(base)))
#define GPIO_DIRECTION_OFFSET   0x00
#define GPIO_OUTPUT_OFFSET      0x04
// GPIO bit definitions:
// Bit 7: DATA/COMMAND line
#define GPIO_DATA_COMMAND_BIT   7
#define GPIO_DATA_COMMAND_MASK  (1 << GPIO_DATA_COMMAND_BIT)
// Bit 3: LED line
#define GPIO_CS_BIT            6
#define GPIO_CS_MASK           (1 << GPIO_CS_BIT)
// Bit 4: RESET line
#define GPIO_RESET_BIT          5
#define GPIO_RESET_MASK         (1 << GPIO_RESET_BIT)
// Bit 3: LED line
#define GPIO_LED_BIT            4
#define GPIO_LED_MASK           (1 << GPIO_LED_BIT)

#define GPIO_DIRECTION_MASK     (GPIO_LED_MASK | GPIO_RESET_MASK | GPIO_DATA_COMMAND_MASK | GPIO_CS_MASK)

// Macro to access a register: given a mapped base pointer and offset
#define REG(base, offset) (*(volatile uint16_t *)((char *)(base) + (offset)))

// Generic register set macro: read current value, clear bits (mask), then set new bits.
#define REGISTER_SET(base, offset, mask, shift, val) do { \
    REG(base, offset) = (REG(base, offset) & ~(mask)) | (((val) << (shift)) & (mask)); \
} while (0)

// Generic register set macro: read current value, clear bits (mask), then set new bits.
#define REGISTER_SET_VAL(base, offset, val) do { \
  REG(base, offset) =  (val); \
} while (0)

// Macros for GPIO operations using the mapped GPIO base pointer
#define SET_TFT_DATA_DIR(base, val)         REGISTER_SET_VAL(base, GPIO_DIRECTION_OFFSET, (val))
#define SET_TFT_DATA_COMMAND(base, val)     REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_DATA_COMMAND_MASK, GPIO_DATA_COMMAND_BIT, (val))
#define SET_TFT_RESET(base, val)            REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_RESET_MASK, GPIO_RESET_BIT, (val))
#define SET_TFT_LED(base, val)              REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_LED_MASK, GPIO_LED_BIT, (val))
#define SET_NFC_RESET(base, val)            REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_DATA_COMMAND_MASK, GPIO_DATA_COMMAND_BIT, (val))


#endif /* GPIO_DRIVER_H */
