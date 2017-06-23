#ifndef __INPUTS_H
#define __INPUTS_H

#include "bitmap.h"
#include "stm32f10x_adc.h"
#include "define.h"
void inputs_init(void) ;
void inputs_scan(void) ;
extern vu16 AD_Value[MAX_AI_CHANNEL]; 

#if defined T36CTA
#define ADC2_CHANNEL			6
#define  VOL_BUF_NUM              3
extern int16_t vol_buf[ADC2_CHANNEL][VOL_BUF_NUM]; 
//extern __IO uint16_t DMA_Buffer[192];
extern uint16_t air_flow_ad;
extern uint16_t CT_Vaule[6];
//extern uint16_t CT_first_AD[6];
//extern uint16_t CT_multiple[6];
extern uint16_t CT_first_AD;
extern uint16_t CT_multiple;
#endif
#ifndef  T3PT12

u16 ADC_getChannal(ADC_TypeDef* ADCx, u8 channal) ;
#endif

#if (defined T322AI) || (T38AI8AO6DO)	|| (defined T36CTA)
extern uint16_t data_change[MAX_AI_CHANNEL] ;
// void pulse_set(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) ;
 void pulse_set(uint8_t channel) ;
#endif



#endif































