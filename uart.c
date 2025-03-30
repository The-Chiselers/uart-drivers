#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include "spi.h"  // Use your header with the updated register definitions

// Global variables for mmap and SPI register base pointer.
static volatile void *spi_base = NULL;
static volatile void *gpio_base = NULL;
static int spi_fd = -1;

/*
 * tft_spi_initialize()
 *
 * Opens /dev/mem, maps the SPI core registers, and configures CTRLA and CTRLB.
 * This function is called by the graphics library during initialization.
 */
void tft_spi_initialize(void)
{
  // printf("INIT SPI\n");
    // Open /dev/mem to access physical memory.
    spi_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (spi_fd < 0) {
        perror("tft_spi_initialize: Error opening /dev/mem");
        exit(EXIT_FAILURE);
    }

    // Map a 4KB region starting at SPI_BASE_ADDRESS.
    spi_base = mmap(NULL, SPI_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, spi_fd, SPI_BASE_ADDRESS);
    if (spi_base == MAP_FAILED) {
        perror("tft_spi_initialize: Error mapping SPI registers");
        close(spi_fd);
        exit(EXIT_FAILURE);
    }
    

    int gpio_fd = open("/dev/mem", O_RDWR | O_SYNC);
    gpio_base = mmap(NULL, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, gpio_fd, GPIO_BASE_ADDRESS);

    // Configure CTRLA:
    //   - MASTER = 1 (Master mode)
    //   - CLK2X = 1 (Double the SPI clock)
    //   - PRESC = 0x01 (Example: Divide by 16; adjust as needed)
    //   - ENABLE = 1 (Enable SPI core)
    //   - DORD = 0 (MSB first; note: this bit is reused here for Data/Command switching)
    SET_TFT_DATA_DIR(gpio_base, GPIO_DIRECTION_MASK);
    SET_SPI_INTCTRL_IE(spi_base, 1);
    SET_SPI_TXCIE(spi_base, 1);
    SET_TFT_LED(gpio_base, 1);
    SET_TFT_DATA_COMMAND(gpio_base, 1);
    SET_SPI_MASTER(spi_base, 1);
    // SET_SPI_CLK2X(spi_base, 1);
    SET_SPI_PRESC(spi_base, 1);
    SET_SPI_ENABLE(spi_base, 1);
    SET_SPI_DORD(spi_base, 0); // Default to MSB first / data mode

    // Configure CTRLB:
    //   - MODE = 0 (SPI Mode 0)
    //   - BUFEN = 0 (Disable buffering)
    SET_SPI_MODE(spi_base, 3);
    SET_SPI_BUFEN(spi_base, 1);

    // SPI_DATA(spi_base) = 0x0017;            // Dummy transmission
    // // while(!GET_SPI_IF(spi_base));         // Wait for the flag to go high
    // while(GET_SPI_DREIF(spi_base));

    SET_TFT_CS_FORCE(gpio_base, 0);
    printf("INIT DONE\n");

    // // Disable interrupts in INTCTRL.
    // SET_SPI_TXCIE(spi_base, 0);
    // SET_SPI_INTCTRL_IE(spi_base, 0);
}

void tft_spi_disable(void) {
  SET_TFT_LED(gpio_base, 0);
  SET_TFT_DATA_COMMAND(gpio_base, 0);
}

/*
 * tft_hardware_initialize()
 *
 * Typically used to toggle a hardware reset. In this example no dedicated reset register is available,
 * so we simply wait a short time to allow the SPI core to stabilize.
 */
void tft_hardware_initialize(void)
{
    SET_TFT_DATA_COMMAND(gpio_base, 1);
    SET_TFT_RESET(gpio_base, 0);
    usleep(100000); // 100 ms delay
    SET_TFT_RESET(gpio_base, 1);
    usleep(1000); // 1 ms delay
}

/*
 * tft_spi_baud_rate()
 *
 * Adjusts the SPI baud rate by changing the prescaler field.
 * The input baud_rate is mapped (inversely) to the 2-bit prescaler value.
 * (There are 4 possible prescaler settings: 0, 1, 2, or 3.)
 */
void tft_spi_baud_rate(uint8_t baud_rate)
{
    // In this example we map the input such that a higher baud_rate value results in a lower prescaler.
    // Only the lower 2 bits of the prescaler are used.
    if (baud_rate > 3) {
      baud_rate = baud_rate % 3;
    }
    SET_SPI_PRESC(spi_base, baud_rate);
    // No settle time field is provided in this register mapping.
}

/*
 * tft_spi_write_command()
 *
 * Writes an 8-bit command to the SPI device.
 * The function first sets the Data/Command bit to command mode (0), writes the command,
 * then restores the line to data mode (1).
 */
void tft_spi_write_command(uint8_t cmd)
{
  // printf("Write command\n");
    // Set Data/Command to command mode.
    SET_TFT_DATA_COMMAND(gpio_base, 0);
    // while(!GET_SPI_DREIF(spi_base) && !GET_SPI_TXCIF(spi_base));
    

    // Write the command to the DATA register.
    SPI_DATA(spi_base) = cmd;
    // A short delay to allow the command to be transmitted.
    tft_wait();
    // Restore Data/Command to data mode.
    SET_TFT_DATA_COMMAND(gpio_base, 1);
}

/*
 * tft_spi_write_data()
 *
 * Writes an 8-bit data value over SPI.
 */
void tft_spi_write_data(uint8_t data)
{
    // Optional: Wait for any previous transmission to complete.
    // while(GET_SPI_DREIF(spi_base));
    // Write the data to the DATA register.
    SPI_DATA(spi_base) = data;
    tft_wait();
}

/*
 * tft_spi_write_data16()
 *
 * Writes a 16-bit data word over SPI.
 */
void tft_spi_write_data16(uint16_t word)
{
  // while(!GET_SPI_IF(spi_base));
  // while(!GET_SPI_DREIF(spi_base) && !GET_SPI_TXCIF(spi_base));
  

    
    // Check the SPI_DORD bit (0x40). If set, we are in LSB mode.
    if (GET_SPI_DORD(spi_base) ) {
        // LSB first: send lower byte first
        tft_spi_write_data((uint8_t)(word & 0xFF));
        tft_spi_write_data((uint8_t)(word >> 8));
    } else {
        // MSB first: send higher byte first
        tft_spi_write_data((uint8_t)(word >> 8));
        tft_spi_write_data((uint8_t)(word & 0xFF));
    }
}

tft_wait(void) {
  while(GET_SPI_DREIF(spi_base));
  if (GET_SPI_TXCIF(spi_base)) {
    CLEAR_TXCIF(spi_base);
  }
}


/*
 * tft_spi_cleanup()
 *
 * Unmaps the SPI registers and closes the /dev/mem file.
 * Call this function when the SPI core is no longer needed.
 */
void tft_spi_cleanup(void)
{
    tft_spi_disable();
    if (spi_base && spi_base != MAP_FAILED) {
        munmap((void *)spi_base, SPI_MAP_SIZE);
        spi_base = NULL;
    }
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
}
