#ifndef BIT_READER_H
#define BIT_READER_H

#include <Arduino.h>

typedef enum _cj_request_bytes_: byte {
    HEATER_STATUS = 1,
}CJ_REQUEST_BYTES;

uint8_t readByte(uint16_t data, byte byte) {
    return ((data >> byte - 1) & 0x1);
}





#endif