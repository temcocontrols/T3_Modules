#ifndef  _STORE_H
#define  _STORE_H 

#include "ud_str.h"
#include "define.h"

extern Str_variable_point var[MAX_AV] ;


void Flash_Write_Mass(void) ;

void mass_flash_init(void) ;


extern uint8_t write_page_en[3]  ;

extern const uint8_t Variable_name[][9] ;








#endif

