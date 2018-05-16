#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"


#define FLASH_PAGE_SIZE    ((uint16_t)0x800)

#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08070000)
#define BANK1_WRITE_END_ADDR    ((uint32_t)0x08070800)//choose one page


typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

e_state flashHandle(uint16_t * data,uint8_t flag);


#endif
