#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "pn532.h"
// #include "pn532_rpi.h"


int main(int argc, char** argv) {
    uint8_t buff[255];
    uint8_t uid[MIFARE_UID_MAX_LENGTH];
    int32_t uid_len = 0;
    printf("Hello!\r\n");
    PN532_UART_Init();
    printf("Init Done!\r\n");

    if (PN532_GetFirmwareVersion( buff) == PN532_STATUS_OK) {
        printf("Found PN532 with firmware version: %d.%d\r\n", buff[1], buff[2]);
    } else {
        return -1;
    }
    PN532_SamConfiguration();
    printf("Waiting for RFID/NFC card...\r\n");
    while (1)
    {
        // Check if a card is available to read
        uid_len = PN532_ReadPassiveTarget(uid, PN532_MIFARE_ISO14443A, 1000);
        if (uid_len == PN532_STATUS_ERROR) {
            printf(".");
            fflush(stdout);
        } else {
            printf("Found card with UID: ");
            for (uint8_t i = 0; i < uid_len; i++) {
                printf("%02x ", uid[i]);
            }
            printf("\r\n");
            break;
        }
    }
}
