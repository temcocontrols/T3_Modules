#include "store.h"
#include "modbus.h"
#include "inputs.h"
#include "controls.h"
#include "bacnet.h"
#include "stmflash.h"
#include "delay.h"
uint8_t write_page_en[3]  = {0} ;

static uint8_t  tempbuf[1024] = {0};

Str_variable_point  var[MAX_AV] ;


// page 125 0x0803 e800  - 0x0803 efff    2K  OUT
// page 126 0x0803 f000  - 0x0803 f7ff    2K  IN
// page 127 0x0803 f800  - 0x0803 ffff    2K  VAR


//#define OUT_PAGE_FLAG	0x803effe
#define OUT_PAGE_FLAG	0x803e800
#define IN_PAGE_FLAG 	0x803f000	
#define AV_PAGE_FLAG 	0x803f800	 

#define OUT_PAGE	(OUT_PAGE_FLAG+2)	
#define IN_PAGE		(IN_PAGE_FLAG+2)
#define AV_PAGE		(AV_PAGE_FLAG+2)
/* caclulate detailed position for every table */
//void Flash_Inital(void)
//{
//	uint8_t loop;
//	uint16_t baseAddr = 0;	 	
//	uint16_t  len = 0;
////	memset(tempbuf,0,8000);
//	for(loop = 0;loop < MAX_POINT_TYPE;loop++)
//	{  		
//		switch(loop)
//		{	
//	  	case OUT:	
//			baseAddr = 0;
//			len = sizeof(Str_out_point) * MAX_OUTS;
//			break;
//		case IN:
//			baseAddr += len;
//			len = sizeof(Str_in_point) * MAX_INS;
//			break;
//		case VAR:
//			baseAddr += len;
//			len = sizeof(Str_variable_point) * MAX_AV;
//			break;
//		}
//		
//	}
//		
//}

void Flash_Write_Mass(void)
{
//	STR_flag_flash ptr_flash;
//	uint base_addr;

	uint16_t	len = 0 ;
	uint16_t loop1;	
	
			#ifdef T38AI8AO6DO
			if(write_page_en[0] == 1)
			{
				STMFLASH_Unlock();
				write_page_en[0] = 0 ;	
				STMFLASH_ErasePage(OUT_PAGE);				
					for(loop1 = 0;loop1 < MAX_OUTS;loop1++)
					{
						memcpy(&tempbuf[sizeof(Str_out_point) * loop1],&outputs[loop1],sizeof(Str_out_point));					
					}
					len = sizeof(Str_out_point) *MAX_OUTS ;
					iap_write_appbin(OUT_PAGE,(uint8_t*)tempbuf, len);
					STMFLASH_WriteHalfWord(OUT_PAGE_FLAG, 1000) ;
					STMFLASH_Lock();
			}
				#endif
			if(write_page_en[1] == 1)
			{
				write_page_en[1] = 0 ;
				STMFLASH_Unlock();
				STMFLASH_ErasePage(IN_PAGE);
				for(loop1 = 0;loop1 < MAX_INS;loop1++)
				{
					memcpy(&tempbuf[sizeof(Str_in_point) * loop1],&inputs[loop1],sizeof(Str_in_point));					
				}
				len = sizeof(Str_in_point)*MAX_INS ;
				iap_write_appbin(IN_PAGE,(uint8_t*)tempbuf, len);
				STMFLASH_WriteHalfWord(IN_PAGE_FLAG, 1000) ;
				STMFLASH_Lock();
			}
			if(write_page_en[EN_VAR] == 1)
			{
				write_page_en[EN_VAR] = 0 ;
				STMFLASH_Unlock();
				STMFLASH_ErasePage(AV_PAGE);
				for(loop1 = 0;loop1 < AVS;loop1++)
				{
					memcpy(&tempbuf[sizeof(Str_variable_point) * loop1],&var[loop1],sizeof(Str_variable_point));					
				}
				len = sizeof(Str_variable_point)*AVS ;
				iap_write_appbin(AV_PAGE,(uint8_t*)tempbuf, len);
				STMFLASH_WriteHalfWord(AV_PAGE_FLAG, 1000) ;
				STMFLASH_Lock();
			}				
}

void mass_flash_init(void)
{
	u16 temp = 0 ;
	u16 loop ;
	u16 len = 0;
	u8 label_buf[21] ;
	#ifdef  T38AI8AO6DO
	temp = STMFLASH_ReadHalfWord(OUT_PAGE_FLAG);
	if(temp == 0xffff)
	{
		STMFLASH_Unlock();
		STMFLASH_ErasePage(OUT_PAGE_FLAG);
		for(loop=0; loop<MAX_OUTS; loop++ )
		{
			if(loop <6)
			{
				sprintf((char*)label_buf, "Digit output%u", loop);
				memcpy(outputs[loop].description, label_buf, 21);
				sprintf((char*)label_buf, "DO%u", loop);
				memcpy(outputs[loop].label, label_buf, 9);
			}
			else
			{
				sprintf((char*)label_buf, "Analog output%u", (loop-6));
				memcpy(outputs[loop].description, label_buf, 21);
				sprintf((char*)label_buf, "AO%u", (loop-6));
				memcpy(outputs[loop].label, label_buf, 9);	
			}		
			outputs[loop].value = 0; 
			outputs[loop].auto_manual = 0 ;
			outputs[loop].digital_analog = 0 ;
			outputs[loop].switch_status = 0 ;
			outputs[loop].control = 0 ;
			outputs[loop].read_remote = 0 ;
			outputs[loop].decom = 0 ;
			outputs[loop].range = 0 ;
			outputs[loop].sub_id = 0 ;
			outputs[loop].sub_product = 0 ;
			outputs[loop].pwm_period = 0 ;
		}
		len = MAX_OUTS * sizeof(Str_out_point) ;
		memcpy(tempbuf,(void *)&outputs[0].description[0],len);	
//		memcpy(tempbuf,"abcdefghijklmnopqrstuvwxyz",len);	
		iap_write_appbin(OUT_PAGE,(uint8_t*)tempbuf, len);	
		STMFLASH_WriteHalfWord(OUT_PAGE_FLAG, 1000) ;
		STMFLASH_Lock();
		
	}
	else
	{
		len = MAX_OUTS * sizeof(Str_out_point) ;
		STMFLASH_MUL_Read(OUT_PAGE,(void *)&outputs[0].description[0], len );	
	}
	#endif

	
	temp = STMFLASH_ReadHalfWord(IN_PAGE_FLAG);
	if(temp == 0xffff)
	{
		STMFLASH_Unlock();
		STMFLASH_ErasePage(IN_PAGE_FLAG);
		for(loop=0; loop<MAX_INS; loop++ )
		{
			sprintf((char*)label_buf, "AI%u", loop);
			memcpy(inputs[loop].description, label_buf, 21);
			memcpy(inputs[loop].label, label_buf, 9);		
			inputs[loop].value = 0; 
			inputs[loop].filter = 5 ;
			inputs[loop].decom = 0 ;
			inputs[loop].sub_id = 0 ;
			inputs[loop].sub_product = 0 ;
			inputs[loop].control = 0 ;
			inputs[loop].auto_manual = 0 ;
			inputs[loop].digital_analog = 0 ;
			inputs[loop].calibration_sign = 0 ;
			inputs[loop].sub_number = 0 ;
			inputs[loop].calibration_hi =0 ;
			inputs[loop].calibration_lo = 0 ;
			inputs[loop].range = 0 ; 
		}
		len = MAX_INS * sizeof(Str_in_point) ;
		memcpy(tempbuf,(void*)&inputs[0], len);		
		iap_write_appbin(IN_PAGE,(uint8_t*)tempbuf, len);	
		STMFLASH_WriteHalfWord(IN_PAGE_FLAG, 1000) ;
		STMFLASH_Lock();
	}
	else
	{
		
		len = MAX_INS * sizeof(Str_in_point) ;
		STMFLASH_MUL_Read(IN_PAGE,(void *)&inputs[0].description[0], len );
		#ifdef  T322AI
		for(loop=0; loop<11; loop++)
		{
			if((inputs[loop].range == N0_2_32counts)||(inputs[loop].range == HI_spd_count))
			pulse_set(loop); 
		}
		#endif
	
	}
	
	temp = STMFLASH_ReadHalfWord(AV_PAGE_FLAG);
	if(temp == 0xffff)
	{
		STMFLASH_Unlock();
		STMFLASH_ErasePage(AV_PAGE_FLAG);
		for(loop=0; loop<MAX_AV; loop++ )
		{
			memcpy(var[loop].description, (char*)&Variable_name[loop], 9);
			memcpy(var[loop].label, (char*)&Variable_name[loop], 9);		
			var[loop].value = 0; 
//			var[loop].control = 0 ;
//			var[loop].auto_manual = 0 ;
//			var[loop].digital_analog = 0 ;
//			var[loop].range = 0 ; 
		}
		len = MAX_AV * sizeof(Str_variable_point) ;
		memcpy(tempbuf,(void*)&var[0], len);		
		iap_write_appbin(AV_PAGE,(uint8_t*)tempbuf, len);	
		STMFLASH_WriteHalfWord(AV_PAGE_FLAG, 1000) ;
		STMFLASH_Lock();
	}
	else
	{
		len = MAX_AV * sizeof(Str_variable_point) ;
		STMFLASH_MUL_Read(AV_PAGE,(void *)&var[0].description[0], len );
	
	}
	
}

