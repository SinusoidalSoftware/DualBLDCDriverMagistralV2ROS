#ifndef CRC32_H
#define CRC32_H
// C library headers
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

uint32_t Crc32(uint32_t Crc, uint32_t Data);
uint32_t Crc32Fast(uint32_t Crc, uint32_t Data);
uint32_t GenerateCrc(uint8_t *bb, uint32_t size);


#endif