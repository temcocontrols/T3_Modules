#include "pt12_i2c.h"
#include "read_pt.h"
#define PT12_DELAY   5
#ifdef T3PT12

void PT12_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
	
}

void PT12_I2C_init(void)
{
	PT12_SDA_OUT();
	PT12_SDA = 1;
	PT12_SCL = 1;
}

void PT12_I2C_start(void)
{
	PT12_SDA_OUT();
	PT12_SDA = 1;
	delay_us(PT12_DELAY);
	PT12_SCL = 1;
	delay_us(PT12_DELAY);
	PT12_SDA = 0;
	delay_us(PT12_DELAY);
	PT12_SCL = 0;
}

void PT12_I2C_stop(void)
{
	PT12_SDA_OUT();
	PT12_SCL = 0;
	PT12_SDA = 0;
	delay_us(PT12_DELAY);
	PT12_SCL = 1;
	PT12_SDA = 1;
	delay_us(PT12_DELAY);;
}

u8 PT12_I2C_wait_for_ack(void)
{
	u16 ucErrTime = 0;
	PT12_SDA_IN();		//SDA设置为输入 	
	PT12_SCL = 0;
	PT12_SDA = 1;
	delay_us(20);
	for(ucErrTime = 0; ucErrTime < 250; ucErrTime++)
	{
		if(READ_PT12_SDA == 0)
		{
			PT12_SDA_OUT();
			PT12_SDA = 0 ;
			delay_us(PT12_DELAY);
			PT12_SCL = 1;
			return ACK;
		}
	}
	return NACK ;
}
//产生ACK应答
void PT12_I2C_Ack(void)
{	PT12_SDA_OUT();
	PT12_SCL = 0;	
	PT12_SDA = 0;
	delay_us(PT12_DELAY);
	PT12_SCL = 1;
	delay_us(PT12_DELAY);
	PT12_SCL = 0;
}

//不产生ACK应答		    
void PT12_I2C_NAck(void)
{
	PT12_SCL = 0;
	PT12_SDA_OUT();
	PT12_SDA = 1;
	delay_us(PT12_DELAY);
	PT12_SCL = 1;
	delay_us(PT12_DELAY);
	PT12_SCL = 0;
}


void PT12_I2C_write_byte(u8 txd)
{
    u8 t;   
	PT12_SDA_OUT(); 
	PT12_SCL = 0;
    for(t = 0; t < 8; t++)
    {  		 		
		PT12_SDA = (txd & 0x80)>>7;
		txd <<= 1;
		delay_us(1);
		PT12_SCL = 1;
		delay_us(5);
		PT12_SCL = 0;
		delay_us(5);
    }	
}


u8 PT12_I2C_read_byte(u8 ack)
{
	u8 i, ret = 0;	
	PT12_SDA_IN();			//SDA设置为输入
	PT12_SCL = 0;
	PT12_SDA = 1 ;
	delay_us(10) ;
	for(i = 0; i < 8; i++)
	{	
		PT12_SCL = 1;
		delay_us(5);
        if(READ_PT12_SDA)	ret |= (1 << (7 - i));
		delay_us(5);  
		PT12_SCL = 0;
		delay_us(5); 		
    }
	if(ack == ACK)
	{
		PT12_I2C_Ack();
	}
	else
	{
		PT12_I2C_NAck();
	}
//	PT12_SCL = 1;
		   
    return ret;
}


#endif
