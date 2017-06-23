#include "stdint.h"
#include "types.h"
#include "define.h"
#include "usart.h"
#include "rs485.h"
#include "bacnet.h"
//#include "eepDefine.h"
#include "24cxx.h"
#include "modbus.h"
#include "inputs.h"
#include "define.h"
#include "filter.h"
#include "registerlist.h"
#include "store.h"
#include "tapdev.h"

uint8_t panelname[20] ;



const u8 Variable_name[][9] = {
					
					"MAC ADDR",
					"BAUD",
					"PROTOCOL",
					"INSTANCE",
					"IP_ADR_1",
					"IP_ADR_2",
					"IP_ADR_3",
					"IP_ADR_4",
					"SUBMSK_1",
					"SUBMSK_2",
					"SUBMSK_3",
					"SUBMSK_4",
					"GATEWAY_1" ,
					"GATEWAY_2" ,
					"GATEWAY_3" ,
					"GATEWAY_4" ,
					"IP_PORT"		,
	                #if 0//(defined T36CTA_REV1)
					"AI1_RANGE"	,
					"AI2_RANGE"	,
					"AI3_RANGE"	,
					"AI4_RANGE"	,
					"AI5_RANGE"	,
					"AI6_RANGE"	,
					"AI7_RANGE"	,
					"AI8_RANGE"	,
					"AI9_RANGE"	,
					"AI10RANGE",
					"AI11RANGE"	,
					"AI12RANGE"	,
					"AI13RANGE"	,
					"AI14RANGE"	,
					"AI15RANGE"	,
					"AI16RANGE"	,
					"AI17RANGE"	,
					"AI18RANGE"	,
					"AI19RANGE"	,
					"DO1_SW",
					"DO2_SW",
					#endif
					#if 0//(defined T36CTA_REV2)
					"AI1_RANGE"	,
					"AI2_RANGE"	,
					"AI3_RANGE"	,
					"AI4_RANGE"	,
					"AI5_RANGE"	,
					"AI6_RANGE"	,
					"AI7_RANGE"	,
					"AI8_RANGE"	,
					"AI9_RANGE"	,
					"AI10RANGE",
					"AI11RANGE"	,
					"AI12RANGE"	,
					"AI13RANGE"	,
					"CT1"	,
					"CT2"	,
					"CT3"	,
					"CT4"	,
					"CT5"	,
					"CT6"	,
					"DO1_SW",
					"DO2_SW",
					#endif
					#if (defined T38AI8AO6DO) || (defined T36CTA)
					"AI1_RANGE"	,
					"AI2_RANGE"	,
					"AI3_RANGE"	,
					"AI4_RANGE"	,
					"AI5_RANGE"	,
					"AI6_RANGE"	,
					"AI7_RANGE"	,
					"AI8_RANGE"	,
					"AO1_SW",
					"AO2_SW",
					"AO3_SW",
					"AO4_SW",
					"AO5_SW",
					"AO6_SW",
					"AO7_SW",
					"AO8_SW",
					"DO1_SW",
					"DO2_SW",
					"DO3_SW",
					"DO4_SW",
					"DO5_SW",
					"DO6_SW",
					#endif
					#ifdef T322AI
					"RANGE1"	,
					"RANGE2"	,
					"RANGE3"	,
					"RANGE4"	,
					"RANGE5"	,
					"RANGE6"	,
					"RANGE7"	,
					"RANGE8"	,
					"RANGE9"	,
					"RANGE10"	,
					"RANGE11"	,
					"RANGE12"	,
					"RANGE13"	,
					"RANGE14"	,
					"RANGE15"	,
					"RANGE16"	,
					"RANGE17"	,
					"RANGE18"	,
					"RANGE19"	,
					"RANGE20"	,
					"RANGE21"	,
					"RANGE22"	,
					#endif
					#ifdef T3_PT12
					"TYPE_1"	,
					"TYPE_2"	,
					"TYPE_3"	,
					"TYPE_4"	,
					"TYPE_5"	,
					"TYPE_6"	,
					"TYPE_7"	,
					"TYPE_8"	,
					"TYPE_9"	,
					"TYPE_10"	,
					"TYPE_11"	,
					"TYPE_12"	,
					#endif
					"BAC_PORT"
				};	
void switch_to_modbus(void)
{
//	printf()
//	modbus.protocal = MODBUS;
//	write_eeprom(EEP_MODBUS_BACNET_SWITCH, MODBUS);
	
	if(modbus.baudrate  == BAUDRATE_19200)
		uart1_init(19200);
	else if(modbus.baudrate  == BAUDRATE_9600)
		uart1_init(9600);
	else if(modbus.baudrate  == BAUDRATE_38400)
		uart1_init(38400);
	else if(modbus.baudrate  == BAUDRATE_57600)
		uart1_init(57600);
	else if(modbus.baudrate  == BAUDRATE_115200)
		uart1_init(115200);
	else if(modbus.baudrate  == BAUDRATE_76800)
		uart1_init(76800);
}

uint16_t send_count;
//u16 far Test[50];

uint8_t RS485_Get_Baudrate(void)
{
 if(modbus.baudrate == BAUDRATE_9600)
  return 5;
 else if(modbus.baudrate == BAUDRATE_19200)
  return 6;
 else if(modbus.baudrate == BAUDRATE_38400)
  return 7;
 else if(modbus.baudrate == BAUDRATE_57600)
  return 8;
 else if(modbus.baudrate == BAUDRATE_115200)
  return 9;
  else if(modbus.baudrate == BAUDRATE_76800)
  return 10;
 else 
  return 6;// default is 19200
}

//----------------------------------------------------------
void Get_AVS(void)
{
//	//bacnet_AV.reg.avs_num = 50;
//	bacnet_AV.reg.address = Modbus.address;
//	//	bacnet_AV.reg.product_model = Modbus.product_model;
//	bacnet_AV.reg.hardRev = Modbus.hardRev;
//	bacnet_AV.reg.firwareRev = SW_REV;
//	bacnet_AV.reg.tcp_type = Modbus.tcp_type;
//	memcpy(bacnet_AV.reg.ip_addr,Modbus.ip_addr,4);
//	memcpy(bacnet_AV.reg.mac_addr,Modbus.mac_addr,6);
//	memcpy(bacnet_AV.reg.subnet,Modbus.subnet,4);
//	memcpy(bacnet_AV.reg.getway,Modbus.getway,4);
//	bacnet_AV.reg.tcp_port = Modbus.tcp_port;
//	//	bacnet_AV.reg.mini_type = Modbus.mini_type;
//	memcpy(bacnet_AV.reg.com_config,Modbus.com_config,3);
//	bacnet_AV.reg.com_baudrate[0] = uart0_baudrate;
//	bacnet_AV.reg.com_baudrate[1] = uart1_baudrate;
//	bacnet_AV.reg.com_baudrate[2] = uart2_baudrate;
//	//	memcpy(bacnet_AV.reg.start_adc,Modbus.start_adc,11);
//	bacnet_AV.reg.network_number = Modbus.network_number;
//	bacnet_AV.reg.panel_number = Station_NUM;
//	
}
//modbus.input[0]
//----------------------------------------------
float Get_bacnet_value_from_buf(uint8_t type,uint8_t priority,uint8_t i)
{	
	switch(type)
	{
		case AV:
			if(i == 1) return Station_NUM ;
			else if(i==2) return modbus.baudrate ;
			else if(i==3) return modbus.protocal ;
			else if(i==4) return Instance ;
			else if((i>= 5)&&(i<= 8)) return modbus.ip_addr[i-5] ;
			else if((i>= 9)&&(i<= 12)) return modbus.mask_addr[i-9] ;
			else if((i>= 13)&&(i<= 16)) return modbus.gate_addr[i-13] ;
			else if(i== 17)	return modbus.listen_port;
			#if (defined T38AI8AO6DO) || (defined T36CTA)
		  else if((i>= 18)&&(i<= 25)) return inputs[i-18].range ;
			else if((i>= 26)&&(i<= 33)) return outputs[i+MAX_DO-26].switch_status ;
			else if((i>= 34)&&(i<= 39)) return outputs[i-34].switch_status ;
		  else if(i== 40) return modbus.bacnet_port ;
			#endif
			#ifdef T322AI
		  else if((i>= 18)&&(i<= 39)) return inputs[i-18].range ;
		  else if(i== 40) return modbus.bacnet_port ;
			#endif
			#ifdef T3PT12
			else if((i>= 17)&&(i<= 28)) return inputs[i-17].range ;
			else if(i== 29) return modbus.bacnet_port ;
			#endif
			break;
		case AI:										
			if(inputs[i].digital_analog == 0 )
			{
				if(inputs[i].range <= LOW_HIGH )
				 {
						if(inputs[i].value >= 1000 )
						{
								inputs[i].value  = 1 ;
						}
						else
						{
								inputs[i].value  = 0 ;
						}
				 }
				 else
				 {
						if(inputs[i].value >= 1000 )
						{
							inputs[i].value  = 0 ;
						}
						else
						{
							inputs[i].value  = 1 ;
						}
				 }
				return  inputs[i].value ;						
		}
		else
		{	
			return ((float)inputs[i].value/1000) ;
		}
//		break;
		case AO:
		#if (defined T38AI8AO6DO) || (defined T36CTA)
		return ((float)outputs[i+MAX_DO].value/1000) ;

		break;
			
		case BO:
			//			if(outputs[i].auto_manual == 0)
			//				priority = 15; // AUTO
			//			else
			//				priority = 7; // manual

//			if(outputs[i].range > 0)
//				return outputs[i].control;
		return (outputs[i].control)? 1:0 ;
		break;
	#endif	
		default:
			break;
				
	}	

}

//------------------------------------------------------------
void wirte_bacnet_value_to_buf(uint8_t type,uint8_t priority,uint8_t i,float value)
{


		switch(type)
		{
			case AV:
				if(i== 1)
				{
					//modbus.address = value ;
					Station_NUM = value ;
					AT24CXX_WriteOneByte((u16)EEP_STATION_NUM, value);
					Inital_Bacnet_Server();
					dlmstp_init(NULL);
					Recievebuf_Initialize(0);
				}
				else if(i== 2)
				{
					modbus.baudrate = value ;
					if(modbus.baudrate == BAUDRATE_9600)
					{
						modbus.baud = 0 ;
						AT24CXX_WriteOneByte(EEP_BAUDRATE, modbus.baud);
						uart1_init(BAUDRATE_9600);
						SERIAL_RECEIVE_TIMEOUT = 6;
					}
					else if(modbus.baudrate == BAUDRATE_19200)
					{
						modbus.baud = 1 ;
						AT24CXX_WriteOneByte(EEP_BAUDRATE, modbus.baud);
						uart1_init(BAUDRATE_19200);	
						SERIAL_RECEIVE_TIMEOUT = 3;
					}
					else if(modbus.baudrate == BAUDRATE_38400)
					{
						modbus.baud = 2 ;
						AT24CXX_WriteOneByte(EEP_BAUDRATE, modbus.baud);
						uart1_init(BAUDRATE_38400);
						SERIAL_RECEIVE_TIMEOUT = 2;
					}
					else if(modbus.baudrate == BAUDRATE_57600)
					{
						modbus.baud = 3 ;
						AT24CXX_WriteOneByte(EEP_BAUDRATE, modbus.baud);
						SERIAL_RECEIVE_TIMEOUT = 1;
						uart1_init(BAUDRATE_57600);
					}
					else if(modbus.baudrate == BAUDRATE_115200)
					{
						modbus.baud = 4 ;
						AT24CXX_WriteOneByte(EEP_BAUDRATE, modbus.baud);
						SERIAL_RECEIVE_TIMEOUT = 1;
						uart1_init(BAUDRATE_115200);
					}
					else if(modbus.baudrate == BAUDRATE_76800)
					{
						modbus.baud = 5 ;
						AT24CXX_WriteOneByte(EEP_BAUDRATE, modbus.baud);
						SERIAL_RECEIVE_TIMEOUT = 1;
						uart1_init(BAUDRATE_76800);
					}
	
				}
				else if(i== 3)
				{
					modbus.protocal = value ;
					if(modbus.protocal == MODBUS)
					AT24CXX_WriteOneByte((u16)EEP_MODBUS_COM_CONFIG, modbus.protocal);
				}
				else if(i== 4)
				{
					Instance = value ;
					AT24CXX_WriteOneByte((u16)EEP_INSTANCE_1, (Instance>>24)&0xff);
					AT24CXX_WriteOneByte((u16)EEP_INSTANCE_2, (Instance>>16)&0xff);
					AT24CXX_WriteOneByte((u16)EEP_INSTANCE_3, (Instance>>8)&0xff);
					AT24CXX_WriteOneByte((u16)EEP_INSTANCE_4, Instance&0xff);
					Inital_Bacnet_Server();
					
				}
				else if((i>= 5)&&(i<= 8))
				{
					modbus.ip_addr[i-5] = value ;
					AT24CXX_WriteOneByte((u16)(EEP_IP_ADDRESS_1+i-5), modbus.ip_addr[i-5]);					
				}
				else if((i>= 9)&&(i<= 12))
				{
					modbus.mask_addr[i-9] = value ;
					AT24CXX_WriteOneByte((u16)(EEP_SUB_MASK_ADDRESS_1+i-9), modbus.mask_addr[i-9]);					
				}
				else if((i>= 13)&&(i<= 16))
				{
					modbus.gate_addr[i-13] = value ;
					AT24CXX_WriteOneByte((u16)(EEP_GATEWAY_ADDRESS_1+i-13), modbus.gate_addr[i-13]);					
				}
				else if(i== 17)
				{
					modbus.listen_port = value ;
					AT24CXX_WriteOneByte((u16)EEP_LISTEN_PORT_HI, (modbus.listen_port>>8)&0xff);
					AT24CXX_WriteOneByte((u16)EEP_LISTEN_PORT_LO, (modbus.listen_port)&0xff);
				}
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				else if((i>= 18)&&(i<= 25))
				{
					if(value < 31)
					{
						inputs[i-18].range = value ;
						inputs[i-18].digital_analog = 0 ;
					}
					else
					{
						inputs[i-18].range = value-30 ;
						inputs[i-18].digital_analog = 1 ;
					}
					write_page_en[1] = 1 ;
				}
				else if(i == 40)
				{
					modbus.bacnet_port = value ;
					AT24CXX_WriteOneByte((u16)EEP_BACNET_PORT_HI, (modbus.bacnet_port>>8)&0xff);
					AT24CXX_WriteOneByte((u16)EEP_BACNET_PORT_LO, (modbus.bacnet_port)&0xff);
					tapdev_init() ;
				}
				#endif
				#ifdef T322AI
				else if((i>= 18)&&(i<= 39))
				{
					if(value < 31)
					{
						inputs[i-18].range = value ;
						inputs[i-18].digital_analog = 0 ;
					}
					else
					{
						inputs[i-18].range = value-30 ;
						inputs[i-18].digital_analog = 1 ;
					}
					write_page_en[1] = 1 ;
				}
				else if(i == 40)
				{
					modbus.bacnet_port = value ;
					AT24CXX_WriteOneByte((u16)EEP_BACNET_PORT_HI, (modbus.bacnet_port>>8)&0xff);
					AT24CXX_WriteOneByte((u16)EEP_BACNET_PORT_LO, (modbus.bacnet_port)&0xff);
					tapdev_init() ;
				}
				#endif
			break;
			case AI:

			break;
			case BO:
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				outputs[i].range = 1 ;
				outputs[i].digital_analog = 0 ;
				outputs[i].value= value*1000;
				if(value) outputs[i].control = 1 ;
				else 			outputs[i].control = 0 ;
				#endif
			break;
			case AO:
					#if (defined T38AI8AO6DO) || (defined T36CTA)
					 outputs[i + MAX_DO].digital_analog = 1;
					 outputs[i + MAX_DO].range = V0_10;	
					 outputs[i+MAX_DO].value= value*1000;
//					 outputs[i+MAX_DO].value= i+MAX_DO;
					#endif
			break;
	
			default:
			break;
		}			

}
//-------------------------------------------------
void write_bacnet_name_to_buf(uint8_t type,uint8_t priority,uint8_t i,char* str)
{

		switch(type)
		{
			case AI:
				memcpy(inputs[i].label,str,8);
				write_page_en[EN_IN] = 1 ;
				break;
			case BO:
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				memcpy(outputs[i].label,str,8);
			  write_page_en[EN_OUT] = 1 ;
				#endif
				break;
			case AO:
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				memcpy(outputs[i+MAX_DO].label,str,8);
				write_page_en[EN_OUT] = 1 ;
				#endif
				break;
			case AV:
				memcpy(var[i+1].label,str,8);
			  write_page_en[EN_VAR] = 1 ;
			default:
			break;
		}	
}
//---------------------------------------------------
void write_bacnet_unit_to_buf(uint8_t type,uint8_t priority,uint8_t i,uint8_t unit)
{
			U8_T temp;
      switch(type)
      {
         case AI:
            if(i >=  MAX_AIS) break;
         if(unit == UNITS_NO_UNITS)
         {
            inputs[i].range = not_used_input;         
         }
         if(unit == UNITS_DEGREES_CELSIUS)
         {
            inputs[i].range = R10K_40_120DegC;
         }
         if(unit == UNITS_DEGREES_FAHRENHEIT)
         {
            inputs[i].range = R10K_40_250DegF;   
         }            
         if(unit == UNITS_AMPERES)
         {
            inputs[i].range = I0_20ma;
            // software jumper 
            temp = inputs[i].decom;
            temp &= 0x0f;
            temp |= (INPUT_I0_20ma << 4);
            inputs[i].decom = temp;
         }
         if(unit == UNITS_VOLTS)
         {
               inputs[i].range = V0_10_IN;
            // software jumper 
            temp = inputs[i].decom;
            temp &= 0x0f;
            temp |= (INPUT_0_10V << 4);
            inputs[i].decom = temp;
         }
         
         if( (unit != UNITS_VOLTS) && (unit != UNITS_AMPERES) )
         {
               // software jumper 
            temp = inputs[i].decom;
            temp &= 0x0f;
            temp |= (INPUT_THERM << 4);
            inputs[i].decom = temp;
         }
            break;
				 #if (defined T38AI8AO6DO) || (defined T36CTA)
         case BO:
            if(i >= MAX_BOS) break;
            outputs[i].digital_analog = 0;
            if(unit == UNITS_NO_UNITS)
               outputs[i].range = 0;
            else
               outputs[i].range = OFF_ON;
            break;
         case AO:
//            if(i + max_dos >= MAX_AOS) break;
            outputs[i + MAX_DO].digital_analog = 1;
            if(unit == UNITS_NO_UNITS)
               outputs[i + MAX_DO].range = 0;
            else
               outputs[i + MAX_DO].range = V0_10;         
            break;
						
					#endif
         default:
         break;
      }
}
//------------------------------------------------------------
char get_AM_Status(uint8_t type,uint8_t num)
{	
   	switch(type)
      {
         case AI:
				 if(num < MAX_AIS)
					return inputs[num].auto_manual;
		 break;
         case AO:
				 #if (defined T38AI8AO6DO) || (defined T36CTA)
				 if(num < AOS)
					return outputs[num+MAX_DO].auto_manual;
				 #endif
		 break;
		 case BO:
			 #if (defined T38AI8AO6DO) || (defined T36CTA)
			 if(num < BOS)
				return outputs[num].auto_manual;
			 #endif
		 break;
         default:
         break;
      }
}
//------------------------------------------------------------
void write_bacent_AM_to_buf(uint8_t type,uint8_t i,uint8_t am)
{
		switch(type)
		{
			case AI:
				inputs[i].auto_manual = am ;
				write_page_en[EN_IN] = 1 ;
				break;
			case BO:
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				outputs[i].auto_manual = am ;
			  write_page_en[EN_OUT] = 1 ;
				#endif
				break;
			case AO:
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				outputs[i + MAX_DO].auto_manual = am ;
				write_page_en[EN_OUT] = 1 ;
				#endif
				break;
	
			default:
			break;
		}
	
}
//------------------------------------------------------------
void add_remote_panel_db(uint32_t device_id,uint8_t panel)
{				
}
//------------------------------------------------------------

char* get_label(uint8_t type,uint8_t num)
{
	switch(type)
      {
		case AV: 
		 
			 if((num < MAX_AVS)&&(num >=1))
				return (char *)var[num].label;
		break;
		case AI:
			 if(num < MAX_AIS)
				return (char *)inputs[num].label;
	    break;
		case AO:
			 #if (defined T38AI8AO6DO) || (defined T36CTA)
			 if(num < AOS)
				return (char *)outputs[num+MAX_DO].label;
			 #endif
			 break;
			 case BO:
			 #if (defined T38AI8AO6DO) || (defined T36CTA)
			 if(num < BOS)
				return (char *)outputs[num].label;
			 #endif
		break;
		default:
		break;
      }
	  return "null";
}
char* get_description(uint8_t type,uint8_t num)
{
	switch(type)
	{
		case AV: 
		  if((num < MAX_AVS)&&(num >=1))
				return (char *)var[num].description;
		break;
		case AI:
			 if(num < MAX_AIS)
				return (char *)inputs[num].description;
			 break;
		case AO:
			 #if (defined T38AI8AO6DO) || (defined T36CTA)
			 if(num < AOS)
				return (char *)outputs[num+MAX_DO].description;
			 #endif
			 break;
			 case BO:
			 #if (defined T38AI8AO6DO) || (defined T36CTA)
			 if(num < BOS)
				return (char *)outputs[num].description;
			 #endif
			 break;
		default:
		break;
	}
	return "null";
}

char get_range(uint8_t type,uint8_t num)
{
 
   if(type == AV)   
   {
         return UNITS_NO_UNITS;  
   }
   if(type == AI)   
   {
      if(inputs[num].digital_analog == 0)  // digital
      {

         return UNITS_NO_UNITS;   
      }
      else  
      {
         if(inputs[num].range == 0)
            return UNITS_NO_UNITS;   
         else if((inputs[num].range == R10K_40_120DegC) || (inputs[num].range == KM10K_40_120DegC)) 
            return UNITS_DEGREES_CELSIUS;
         else if((inputs[num].range == R10K_40_250DegF) || (inputs[num].range == KM10K_40_250DegF)) 
            return UNITS_DEGREES_FAHRENHEIT;
         else if(inputs[num].range == I0_20ma) 
            return UNITS_MILLIAMPERES;
         else if((inputs[num].range == V0_10_IN) || (inputs[num].range == V0_5)) 
            return UNITS_VOLTS;
      }
	  
   }
	 #if (defined T38AI8AO6DO) || (defined T36CTA)
   if(type == AO)
   {       
         return UNITS_VOLTS;
   }
   if(type == BO)   
   {
      return UNITS_NO_UNITS;   
   }
	 #endif
#ifdef T3PT12
return UNITS_DEGREES_CELSIUS ;
#else
return 		UNITS_NO_UNITS ; 
#endif
}

void Set_Object_Name(char * name)	
{
	memcpy(panelname,name,20);
}
void write_bacnet_description_to_buf(uint8_t type, uint8_t priority, uint8_t i, char* str)
{				
		switch(type)
		{
			case AI:
				memcpy(inputs[i].description,str,21);
				write_page_en[EN_IN] = 1 ;
				break;
			case BO:
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				memcpy(outputs[i].description,str,21);
				write_page_en[EN_OUT] = 1 ;
				#endif
			break;
			case AO:
				#if (defined T38AI8AO6DO) || (defined T36CTA)
				memcpy(outputs[i+MAX_DO].description,str,21);
				write_page_en[EN_OUT] = 1 ;
				#endif
				break;
			case AV:
				memcpy(var[i+1].description,str,sizeof(var[i+1].description) );
				write_page_en[EN_VAR] = 1 ;
			break;
			default:
			break;
		}
}	

char* Get_Object_Name(void)
{
	return (char*)panelname;
}
//void uart_send_string(U8_T *p, U16_T length,U8_T port) 
//{
//    memcpy(uart_send, p, length);
//   USART_SendDataString(length);
//}
