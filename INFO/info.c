#include "stm32f10x.h"
#include "ProductModel.h"
#include "define.h"

#define FW_VER_HIGH (SOFTREV>>8)&0xff
#define FW_VER_LOW (SOFTREV)&0xff

#ifdef T322AI
const u8 pro_info[20] __attribute__((at(0x08008200))) = {'T', 'e', 'm', 'c', 'o', 'T', '3', '2', '2', 'I', ' ', ' ', 0, 0, 0, FW_VER_LOW, FW_VER_HIGH,'5', '0',   0,};
#endif
	
#ifdef T38AI8AO6DO
const u8 pro_info[20] __attribute__((at(0x08008200))) = {'T', 'e', 'm', 'c', 'o', 'T', '3', '8', 'I', 'O', ' ', ' ', 0, 0, 0,  FW_VER_LOW, FW_VER_HIGH,'5', '0', 0,};
#endif
	
#ifdef T3PT12
const u8 pro_info[20] __attribute__((at(0x08008200))) = {'T', 'e', 'm', 'c', 'o', 'T', '3', 'P', 'T', '1', '2', ' ', 0, 0, 0,  FW_VER_LOW, FW_VER_HIGH,'5', '0', 0,};
#endif

	