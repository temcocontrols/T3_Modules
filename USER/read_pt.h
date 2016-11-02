
#ifndef _READ_PT_H
#define _READ_PT_H
#include <math.h>
#include "define.h"
#include "modbus.h"
#include "t3-pt12.h"





#ifdef T3PT12

#define	IIC_WRITE	0x00
#define	IIC_READ	0x01

#define	ACK		1
#define	NACK	0

#define	IIC_RTD_ADDR	0x02


#define	SYNC_HEAD				0x7f


#define CALI_AD_VALUE			0X10
#define TWEVLE_CHANNAL_AD_VALUE 	0X11
#define	CMD_RESTORE_LINEAR		0x01
#define	CMD_SET_LINEAR			0x02
#define	CMD_GET_LINEAR			0x03
#define	CMD_SET_RTD_PARA		0x04
#define	CMD_GET_RTD_PARA		0x05
#define	CMD_SET_CAL_POINTS		0x06
#define	CMD_GET_AD				0x07
#define	CMD_GET_R				0x08
#define	CMD_GET_TEMP			0x09
#define CMD_SET_RANGE			0X11 
u8 read_rtd_bytes(u8 addr) ;

u8 read_celpoint_bytes(unsigned char addr) ;

	

	
extern u32 rs_data[20] ; 	

u8 init_celpoint(void);	
void read_rtd_data (void) ;

#endif
	
	
	
	
	
	
	
#endif 
