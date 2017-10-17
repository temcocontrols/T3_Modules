#ifndef ACCELERO_METER_H
#define ACCELERO_METER_H

#include "bitmap.h"

//#if defined T36CTA
#define	IIC_WRITE	0x00
#define	IIC_READ	0x01

#define	ACK		1
#define	NACK	0


#define ACCELERO_ADDR_READ   0x3B
#define ACCELERO_ADDR_WRITE  0x3A

//#define ACCELERO_ADDR_READ   0xd5
//#define ACCELERO_ADDR_WRITE  0xd4
//IO方向设置
//#define ACCELERO_SDA_IN()	 {GPIOB->CRH &= 0XFFFF0FFF; GPIOB->CRH |= ((u32)8 << 12);}
//#define ACCELERO_SDA_OUT()	 {GPIOB->CRH &= 0XFFFF0FFF; GPIOB->CRH |= ((u32)3 << 12);}

//IO操作函数	 
#define ACCELERO_SCL		PBout(10)	//SCL
#define ACCELERO_SDA		PBout(11)	//SDA	 
#define READ_ACCELERO_SDA		PBin(11)		//输入SDA 

#define BUILD_UINT10_AXIS(MSB, LSB)   \
          ( ((MSB & 0x00ff) << 2) + ((LSB & 0x00ff) >> 6) )


extern uint16_t axis_value[];
extern uint8_t asix_sequence;

void ACCELERO_IO_Init(void);


void ACCELERO_I2C_init(void);
void ACCELERO_I2C_start(void);
void ACCELERO_I2C_stop(void) ;
void ACCELERO_I2C_NAck(void) ;
void ACCELERO_I2C_Ack(void) ;
void ACCELERO_I2C_write_byte(u8 byte) ;
u8 ACCELERO_I2C_read_byte(u8 ack) ;
u8 ACCELERO_I2C_wait_for_ack(void) ;

extern void ACCELERO_Write_Data(u8 Addr,u8 Value);
extern u8 ACCELERO_Read_Data(u8 Addr);
#endif
//#endif