#include "crc16ibm.h"
#include <string.h>

void CRC16_Init(CRC16 *crc) {
    uint16_t remainder;
    for (size_t dividend = 0; dividend < 256; ++dividend) {
        remainder = dividend << 8;
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (remainder & 0x8000) remainder = (remainder << 1) ^ 0x8005;
            else remainder = (remainder << 1);
        }
        crc->crc16lut[dividend] = remainder;
    }
    for (size_t i = 0; i < 256; i++) {
        uint8_t data = i;
        uint8_t reflection = 0x00;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (data & 0x01) reflection |= (1 << (7 - bit));
            data = (data >> 1);
        }
        crc->crc16ref[i] = reflection;
    }
    crc->is_init = 1;
}

uint16_t CRC16_Fast(uint8_t *data, size_t size) {
    static CRC16 crc = { .is_init = 0 };
    if (!crc.is_init) CRC16_Init(&crc);
    uint16_t crc_val = 0xFFFF;
    for (size_t i = 0; i < size; ++i) {
        uint8_t tableIndex = (crc_val >> 8) ^ crc.crc16ref[data[i]];
        crc_val = (crc_val << 8) ^ crc.crc16lut[tableIndex];
    }
    crc_val = crc.crc16ref[(crc_val >> 8)] << 8 | crc.crc16ref[(crc_val & 0xff)];
    return crc_val;
}
