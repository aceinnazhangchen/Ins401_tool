#pragma once
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t calc_crc(uint8_t* buff, uint32_t nbyte);
uint32_t calc_crc32(uint32_t crc32val, uint8_t *void_ptr, uint32_t len);
int get_file_size(FILE* file);

#ifdef __cplusplus
}
#endif
