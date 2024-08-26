#ifndef _READ_WRITE_FLASH_H
#define	_READ_WRITE_FLASH_H

#include "stm32f10x.h"
#define START_ADDR 0x0807F800


void FLASH_WriteByte(u32 addr,uint16_t Data);
void ReadFlashData(uint8_t *dest_Data, uint32_t num);


#endif
