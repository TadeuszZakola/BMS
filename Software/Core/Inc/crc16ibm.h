#ifndef CRC16IBM_H
#define CRC16IBM_H

#include "main.h"

typedef struct {
    uint16_t crc16lut[256];
    uint8_t crc16ref[256];
    int is_init;
} CRC16;

void CRC16_Init(CRC16 *crc);
uint16_t CRC16_Fast(uint8_t *data, size_t size);

#endif
