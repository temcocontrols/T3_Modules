#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"

void SPI1_Init(void);							//初始化SPI1口
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);	//设置SPI1速度   
u8 SPI1_ReadWriteByte(u8 TxData);				//SPI1总线读写一个字节



void SPI2_Init(void);							//初始化SPI1口
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);	//设置SPI1速度   
u8 SPI2_ReadWriteByte(u8 TxData);				//SPI1总线读写一个字节
		 
		 
//extern void simulate_spi_write_byte(u8 data);
//extern u8 simulate_spi_read_byte(void);
//extern void simulate_spi_init(void);
#endif
