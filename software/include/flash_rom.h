#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"

bool FLASH_erasePage(uint32_t address); // Erase Flash Page(256Byte). "address" is Page start address.
bool FLASH_write32Byte(uint32_t address,uint32_t *message); // Write Flash only 32Byte. "message" must be uint32_t[8]. "address" is Page start address.
void FLASH_relock();
uint32_t FLASH_read4Byte(uint32_t address); // read flash

#ifdef __cplusplus
}
#endif
