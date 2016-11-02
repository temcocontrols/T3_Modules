#ifndef _OUTPUT_H_
#define _OUTPUT_H_

#include "bitmap.h"
#include "stm32f10x_adc.h"
#ifdef	T38AI8AO6DO
void update_digit_output(void) ;
void output_init(void) ;
void output_refresh(void) ;
void set_output(TIM_TypeDef* TIMx, u8 channel, u16 duty) ;

#endif














#endif
