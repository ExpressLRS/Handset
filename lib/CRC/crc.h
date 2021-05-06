#pragma once
#include <stdint.h>

#define CRC_LENGTH_BYTES 256

class GENERIC_CRC8
{
private:
    uint8_t crc8tab[CRC_LENGTH_BYTES];
    uint8_t crcpoly;

public:
    GENERIC_CRC8(uint8_t poly);
    uint8_t calc(uint8_t *data, uint8_t len);
    uint8_t calc(volatile uint8_t *data, uint8_t len);
};

class GENERIC_CRC14
{
private:
    uint16_t crc14tab[CRC_LENGTH_BYTES];
    uint16_t crcpoly;

public:
    GENERIC_CRC14(uint16_t poly);
    uint16_t calc(volatile uint8_t *data, uint8_t len);
};
