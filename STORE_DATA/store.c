#include "store.h"
#include "modbus.h"
#include "inputs.h"
#include "controls.h"
#include "bacnet.h"
#include "stmflash.h"
#include "delay.h"
uint8_t write_page_en[4]  = {0} ;

static uint8_t  tempbuf[1024] = {0};

Str_variable_point  var[MAX_AV] ;


// page 125 0x0803 e800  - 0x0803 efff    2K  OUT
// page 126 0x0803 f000  - 0x0803 f7ff    2K  IN
// page 127 0x0803 f800  - 0x0803 ffff    2K  VAR


//#define OUT_PAGE_FLAG	0x803effe
#define CUSR_PAGE_FLAG 	0x803e000	 // CUSTOMER RANGE
#define OUT_PAGE_FLAG	0x803e800
#define IN_PAGE_FLAG 	0x803f000	
#define AV_PAGE_FLAG 	0x803f800	 

#define OUT_PAGE	(OUT_PAGE_FLAG+2)	
#define IN_PAGE		(IN_PAGE_FLAG+2)
#define AV_PAGE		(AV_PAGE_FLAG+2)
#define CUSR_PAGE	(CUSR_PAGE_FLAG+2)
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
	
			#if (defined T38AI8AO6DO) || (defined T36CTA)
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
			if(write_page_en[EN_CUSTOMER_RANGE] == 1)
			{
				write_page_en[EN_CUSTOMER_RANGE] = 0 ;
				STMFLASH_Unlock();
				STMFLASH_ErasePage(CUSR_PAGE);
				for(loop1 = 0;loop1 < MAX_TBLS;loop1++)
				{
					memcpy(&tempbuf[sizeof(Str_table_point) * loop1],&custom_tab[loop1],sizeof(Str_table_point));					
				}
				len = sizeof(Str_table_point)*MAX_TBLS ;
				iap_write_appbin(CUSR_PAGE,(uint8_t*)tempbuf, len);
				STMFLASH_WriteHalfWord(CUSR_PAGE_FLAG, 1000) ;
				STMFLASH_Lock();
			}				
}

void mass_flash_init(void)
{
	u16 temp = 0 ;
	u16 loop ;
	u16 len = 0;
	u8 label_buf[21] ;
	#if (defined T38AI8AO6DO) || (defined T36CTA)
	temp = STMFLASH_ReadHalfWord(OUT_PAGE_FLAG);
	if(temp == 0xffff)
	{
		STMFLASH_Unlock();
		STMFLASH_ErasePage(OUT_PAGE_FLAG);
		for(loop=0; loop<MAX_OUTS; loop++ )
		{
			if(loop <6)
			{
				sprintf((char*)label_buf, "Digit output%u", (loop+1));
				memcpy(outputs[loop].description, label_buf, 21);
				sprintf((char*)label_buf, "DO%u", (loop+1));
				memcpy(outputs[loop].label, label_buf, 9);
			}
			else
			{
				sprintf((char*)label_buf, "Analog output%u", (loop-5));
				memcpy(outputs[loop].description, label_buf, 21);
				sprintf((char*)label_buf, "AO%u", (loop-5));
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
			inputs[loop].range = 0 ; 
			inputs[loop].digital_analog = 0 ;
			#if (defined T36CTA)
				if(t36ct_ver == T36CTA_REV1 )//#if (defined T36CTA_REV1)#if (defined T36CTA_REV1)
				{
					if( loop < 13)
						sprintf((char*)label_buf, "AI%u", loop);
					else
					{
						inputs[loop].digital_analog = 1 ;
						inputs[loop].range = I0_100Amps;
						sprintf((char*)label_buf, "CT%u", loop-12);
					}
				}//#endif
				if(t36ct_ver == T36CTA_REV2 )//#if (defined T36CTA_REV2)
				{
					if( loop < 8)
						sprintf((char*)label_buf, "AI%u", loop);
					else if(loop == 8)
					{
						inputs[loop].digital_analog = 1 ;
						inputs[loop].range = pressureInWc ; 
						sprintf((char*)label_buf, "AIR FLOW");
					}
					else if(loop == 9)
					{
						inputs[loop].digital_analog = 1 ;
						inputs[loop].range = Reserved3;
						sprintf((char*)label_buf, "ACCELEROMETER");
					}
					else if( loop < 16)
					{
						inputs[loop].digital_analog = 1 ;
						inputs[loop].range = I0_100Amps;
						sprintf((char*)label_buf, "CT%u", loop-9);
					}
					else
						sprintf((char*)label_buf, " ");
				}//#endif
			#else
			sprintf((char*)label_buf, "AI%u", loop);
			#endif
			memcpy(inputs[loop].description, label_buf, 21);
			memcpy(inputs[loop].label, label_buf, 9);		
			inputs[loop].value = 0; 
			inputs[loop].filter =1 ;
			inputs[loop].decom = 0 ;
			inputs[loop].sub_id = 0 ;
			inputs[loop].sub_product = 0 ;
			inputs[loop].control = 0 ;
			inputs[loop].auto_manual = 0 ;
			
			inputs[loop].calibration_sign = 0 ;
			inputs[loop].sub_number = 0 ;
			inputs[loop].calibration_hi =0 ;
			inputs[loop].calibration_lo = 0 ;
			
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
			memcpy(var[loop+1].description, (char*)&Variable_name[loop], 9);
			memcpy(var[loop+1].label, (char*)&Variable_name[loop], 9);		
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
	
	temp = STMFLASH_ReadHalfWord(CUSR_PAGE_FLAG);
	if(temp == 0xffff)
	{
		STMFLASH_Unlock();
		STMFLASH_ErasePage(CUSR_PAGE_FLAG);
		memset(custom_tab,0,MAX_TBLS * sizeof(Str_table_point));
//		for(loop=0; loop < MAX_TBLS; loop++ )
//		{
//			memcpy(var[loop].description, (char*)&Variable_name[loop], 9);
//			memcpy(var[loop].label, (char*)&Variable_name[loop], 9);		
//			var[loop].value = 0; 
//			var[loop].control = 0 ;
//			var[loop].auto_manual = 0 ;
//			var[loop].digital_analog = 0 ;
//			var[loop].range = 0 ; 
//		}
		len = MAX_TBLS * sizeof(Str_table_point) ;
		memcpy(tempbuf,(void*)&custom_tab[0], len);		
		iap_write_appbin(CUSR_PAGE,(uint8_t*)tempbuf, len);	
		STMFLASH_WriteHalfWord(CUSR_PAGE_FLAG, 1000) ;
		STMFLASH_Lock();
	}
	else
	{
		len = MAX_TBLS * sizeof(Str_table_point) ;
		STMFLASH_MUL_Read(CUSR_PAGE,(void *)&custom_tab[0].table_name[0], len );
	
	}
	
}

