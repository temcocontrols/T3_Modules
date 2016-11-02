// in order to write to the hex time you have to enter this line in 
//		-> BL51_Locate
//			-> Code:
//	?CO?FLASH_INFO(0100h)

#include <KEIL_VRS1000_regs.H>
#include "LibIO_T3IO.h"


//unsigned char code product_model 	= T3_PRODUCT_MODEL;
//unsigned int code firmware_version 	= T3_SOFTWARE_VERSION;
#ifdef T3_32IN
unsigned char const code infor[20] = {'T', 'e', 'm', 'c', 'o', 'T', '3', '-', '3', '2', 'A', 'I', 0, 0, 0, FW_VER_LOW, FW_VER_HIGH,  0, 0, 64};
#endif
#ifdef T3_8IO_A
unsigned char const code infor[20] = {'T', 'e', 'm', 'c', 'o', 'T', '3', '-', '8', 'I', 'O', 'A', 0, 0, 0, FW_VER_LOW, FW_VER_HIGH,  0, 0, 64};
#endif


