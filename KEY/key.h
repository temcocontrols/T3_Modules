#ifndef __KEY_H
#define __KEY_H
 
#include "bitmap.h"
#include "define.h"	
#define KEY_NON		0X00
#define	KEY_1		0X01	//PC0
#define	KEY_2		0X02	//PC1
#define	KEY_3		0X04	//PC2
#define	KEY_4		0X08	//PC3


 void beeper_gpio_init(void) ;

void beeper_on(void) ;
void beeper_off(void) ;





#ifdef T38AI8AO6DO
extern u16 hand_status  ;
extern u16 auto_status ;
void KEY_IO_Init(void);	//IO初始化
void KEY_Status_Scan(void);  	//按键扫描函数
extern u8 switch_state_buf[SWITCH_NUM] ;
//extern u8 switch_test[10] ;
#endif
					    
#endif
