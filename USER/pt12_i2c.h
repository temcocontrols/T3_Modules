#ifndef _PT12_I2C_H
#define _PT12_I2C_H
#include <math.h>
#include "define.h"
#include "modbus.h"

#ifdef T3PT12

//IO方向设置
#define PT12_SDA_IN()	{GPIOA->CRL &= 0X0FFFFFFF; GPIOA->CRL |= ((u32)8 << 28);}
#define PT12_SDA_OUT()	{GPIOA->CRL &= 0X0FFFFFFF; GPIOA->CRL |= ((u32)3 << 28);}

//IO操作函数	 
#define PT12_SCL		PAout(6)	//SCL
#define PT12_SDA		PAout(7)	//SDA	 
#define READ_PT12_SDA		PAin(7)		//输入SDA 


void PT12_IO_Init(void);


void PT12_I2C_init(void);
void PT12_I2C_start(void);
void PT12_I2C_stop(void) ;
void PT12_I2C_NAck(void) ;
void PT12_I2C_Ack(void) ;
void PT12_I2C_write_byte(u8 byte) ;
u8 PT12_I2C_read_byte(u8 ack) ;
u8 PT12_I2C_wait_for_ack(void) ;
u8 wait_for_ack(void);
u8 CLK_ACK( void );
void iic_write_byte(u8 byte);
u8 iic_read_byte(u8 ack);

u8 CLK_read( void );
void CLK_write( unsigned char ch );
void CLK_stop(void);
void CLK_start(void);



#endif






#endif

