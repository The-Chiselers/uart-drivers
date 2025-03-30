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

#define DATA_COMMAND_PIN     0x10  // Data Register


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

//------------------------------------------------------------------------------
// Field Definitions for CTRLA (offset 0x0)
//------------------------------------------------------------------------------
// Bit 6 - DORD (Data Order: 0 = MSB first, 1 = LSB first)
#define SPI_DORD_MASK      (0x40)
#define SPI_DORD_SHIFT     (6)

// Bit 5 - MASTER (Master mode: 0 = Slave, 1 = Master)
#define SPI_MASTER_MASK    (0x20)
#define SPI_MASTER_SHIFT   (5)

// Bit 4 - CLK2X (Clock Double: 0 = not doubled, 1 = doubled)
#define SPI_CLK2X_MASK     (0x10)
#define SPI_CLK2X_SHIFT    (4)

// Bits 2-1 - PRESC (Prescaler: 00: divide by 4, 01: divide by 16, 10: divide by 64, 11: divide by 128)
#define SPI_PRESC_MASK     (0x06)
#define SPI_PRESC_SHIFT    (1)

// Bit 0 - ENABLE (SPI Enable: 0 = disabled, 1 = enabled)
#define SPI_ENABLE_MASK    (0x01)
#define SPI_ENABLE_SHIFT   (0)

//------------------------------------------------------------------------------
// Field Definitions for CTRLB (offset 0x4)
//------------------------------------------------------------------------------
// Bit 7 - BUFEN (Buffer Enable: 0 = disabled, 1 = enabled)
#define SPI_BUFEN_MASK     (0x80)
#define SPI_BUFEN_SHIFT    (7)

// Bits 1-0 - MODE (SPI Mode: 00: Mode 0, 01: Mode 1, 10: Mode 2, 11: Mode 3)
#define SPI_MODE_MASK      (0x03)
#define SPI_MODE_SHIFT     (0)

//------------------------------------------------------------------------------
// Field Definitions for INTCTRL (offset 0x8)
//------------------------------------------------------------------------------
// Bit 6 - TXCIE (Transfer Complete Interrupt Enable: 0 = disabled, 1 = enabled)
#define SPI_TXCIE_MASK     (0x40)
#define SPI_TXCIE_SHIFT    (6)

// Bit 0 - IE (Interrupt Enable: 0 = disabled, 1 = enabled)
#define SPI_INTCTRL_IE_MASK  (0x01)
#define SPI_INTCTRL_IE_SHIFT (0)

//------------------------------------------------------------------------------
// Field Definitions for INTFLAGS (offset 0xC)
//------------------------------------------------------------------------------
// Normal mode:
// Bit 7 - IF (Interrupt Flag: 0 = no interrupt, 1 = transfer complete)
#define SPI_IF_NORM_MASK        (0x80)
#define SPI_IF_NORM_SHIFT       (7)

// Buffer mode:
// Bit 7 - IF (Interrupt Flag: 0 = no interrupt, 1 = transfer complete)
#define SPI_TXCIF_SHIFT       (6)
#define SPI_TXCIF_MASK        (1 << SPI_TXCIF_SHIFT)

#define SPI_DREIF_SHIFT       (5)
#define SPI_DREIF_MASK        (1 << SPI_DREIF_SHIFT)



// Bit 6 - WRCOL (Write Collision Flag: 0 = no collision, 1 = collision occurred)
#define SPI_WRCOL_MASK     (0x40)
#define SPI_WRCOL_SHIFT    (6)

//------------------------------------------------------------------------------
// Macro for accessing the SPI data register (offset 0x10)
#define SPI_DATA(base) (*(volatile uint8_t *)((char *)(base) + DATA_OFFSET))

//------------------------------------------------------------------------------
// Macros for register operations using the mapped base pointer
//------------------------------------------------------------------------------

// CTRLA operations
#define SET_SPI_DORD(base, val)      REGISTER_SET(base, CTRLA_OFFSET, SPI_DORD_MASK,   SPI_DORD_SHIFT, (val))
#define SET_SPI_MASTER(base, val)    REGISTER_SET(base, CTRLA_OFFSET, SPI_MASTER_MASK, SPI_MASTER_SHIFT, (val))
#define SET_SPI_CLK2X(base, val)     REGISTER_SET(base, CTRLA_OFFSET, SPI_CLK2X_MASK,  SPI_CLK2X_SHIFT, (val))
#define SET_SPI_PRESC(base, val)     REGISTER_SET(base, CTRLA_OFFSET, SPI_PRESC_MASK,  SPI_PRESC_SHIFT, (val))
#define SET_SPI_ENABLE(base, val)    REGISTER_SET(base, CTRLA_OFFSET, SPI_ENABLE_MASK, SPI_ENABLE_SHIFT, (val))

// CTRLB operations
#define SET_SPI_BUFEN(base, val)     REGISTER_SET(base, CTRLB_OFFSET, SPI_BUFEN_MASK,  SPI_BUFEN_SHIFT, (val))
#define SET_SPI_MODE(base, val)      REGISTER_SET(base, CTRLB_OFFSET, SPI_MODE_MASK,   SPI_MODE_SHIFT, (val))


// INTCTRL operations
#define SET_SPI_TXCIE(base, val)     REGISTER_SET(base, INTCTRL_OFFSET, SPI_TXCIE_MASK, SPI_TXCIE_SHIFT, (val))
#define SET_SPI_INTCTRL_IE(base, val) REGISTER_SET(base, INTCTRL_OFFSET, SPI_INTCTRL_IE_MASK, SPI_INTCTRL_IE_SHIFT, (val))

// Macro to read the IF flag from INTFLAGS (transfer complete indicator)
#define GET_SPI_IF(base)             ((REG(base, INTFLAGS_OFFSET) & SPI_IF_NORM_MASK) >> SPI_IF_NORM_SHIFT)
#define GET_SPI_DORD(base)           ((REG(base, CTRLA_OFFSET) & SPI_DORD_MASK) >> SPI_DORD_SHIFT)

#define GET_SPI_MODE(base)           ((REG(base, CTRLB_OFFSET) & SPI_MODE_MASK) >> SPI_MODE_SHIFT)
#define GET_SPI_TXCIF(base)             ((REG(base, INTFLAGS_OFFSET) & SPI_TXCIF_MASK) >> SPI_TXCIF_SHIFT)
#define GET_SPI_DREIF(base)             ((REG(base, INTFLAGS_OFFSET) & SPI_DREIF_MASK) >> SPI_DREIF_SHIFT)

#define CLEAR_TXCIF(base)           REGISTER_SET(base, INTFLAGS_OFFSET, SPI_TXCIF_MASK, SPI_TXCIF_SHIFT, (1))





//=========================
// GPIO CORE DEFINITIONS
//=========================


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


// Macros for GPIO operations using the mapped GPIO base pointer
#define SET_TFT_DATA_DIR(base, val)              REGISTER_SET_VAL(base, GPIO_DIRECTION_OFFSET, (val))

#define SET_TFT_DATA_COMMAND(base, val)     REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_DATA_COMMAND_MASK, GPIO_DATA_COMMAND_BIT, (val))

#define SET_TFT_RESET(base, val)            REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_RESET_MASK, GPIO_RESET_BIT, (val))

#define SET_TFT_LED(base, val)              REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_LED_MASK, GPIO_LED_BIT, (val))
#define SET_TFT_CS_FORCE(base, val)         REGISTER_SET(base, GPIO_OUTPUT_OFFSET, GPIO_DATA_COMMAND_MASK, GPIO_DATA_COMMAND_BIT, (val))


void tft_spi_initialize(void);

void tft_spi_baud_rate(uint8_t speed);

void tft_hardware_initialize(void);

void tft_spi_write_data(uint8_t data);

void tft_spi_write_data16(uint16_t word);

void tft_spi_write_command(uint8_t cmd);

void tft_spi_disable(void);


#endif /* UART_DRIVER_H */
