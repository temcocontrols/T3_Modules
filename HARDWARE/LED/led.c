#include "led.h"
#include "modbus.h"
#include "controls.h"
#include "inputs.h"

u16 led_bank1 = 0 ;
u16 led_bank2 = 0 ;
u8  heart_beat_led =  0; 
u8	tx_count = 0 ;
u8  rx_count = 0 ;
u8  net_rx_count = 0 ;
u8  net_tx_count = 0 ;
//u8 dim_timer_setting[28];
#if defined T36CTA
bool t36ct_net_led = LED_OFF;
//bool acc_led = LED_OFF;
u8 rfm69_rx_count = 0;
u8 rfm69_tx_count = 0;
uint8 acc_led_count;
#endif
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);
#if T36CTA	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3 |GPIO_Pin_4 |GPIO_Pin_5 |
									GPIO_Pin_6|GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9 |GPIO_Pin_10 |GPIO_Pin_11
									|GPIO_Pin_12 | GPIO_Pin_15;
#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE, GPIO_InitStructure.GPIO_Pin);	
#if T36CTA	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);	
	led_bank1 = 0xffff;
	led_bank2 = 0xffff;
#endif	
}
/*****************************LED TABLE------T3-22AI**************************************/
/*
PE14 = 0
PE15 = 1
PE0-->HEATBEAT 
PE1-->INPUT0 PE2-->INPUT1 PE3-->INPUT2 PE4-->INPUT3 PE5-->INPUT4 PE6-->INPUT5 
PE7-->INPUT6 PE8-->INPUT7 PE9-->INPUT8 PE10-->INPUT9 PE11-->INPUT10 PE12-->TX PE13-->RX  

PE14 = 1
PE15 = 0
PE0-->HEATBEAT 
PE1-->INPUT11 PE2-->INPUT12 PE3-->INPUT13 PE4-->INPUT14 PE5-->INPUT15 PE6-->INPUT16 
PE7-->INPUT17 PE8-->INPUT18 PE9-->INPUT19 PE10-->INPUT20 PE11-->INPUT21 PE12-->TX PE12-->RX  
*/
/****************************************************************************/

//void tabulate_LED_STATE(void)
//{
//  static u16 beat_count = 0;	
//  static u8 i = 0 ;
//  beat_count ++ ;
//  if(beat_count >=200 ) 
//  {
//	beat_count = 0 ;
//	  if(heart_beat_led == LED_ON)
//		heart_beat_led = LED_OFF ;
//	else
//		heart_beat_led = LED_ON ;
//	if(heart_beat_led == LED_ON)
//	{
//		 dim_timer_setting[0] = 8 ;
//		 dim_timer_setting[14] = 8 ;	
//	}
//	else
//	{
//		 dim_timer_setting[0] = 0 ;
//		 dim_timer_setting[14] = 0 ;
//		
//	}
//  }
//  for(i=0; i<11; i++)
//  {
//	 dim_timer_setting[i+1] = modbus.input[i]/500 ;
//	 dim_timer_setting[i+15] = modbus.input[i+11]/500 ;
//  }
//  if(tx_count>0) tx_count-- ;
//  if(rx_count>0) rx_count-- ;
//  
//  if(tx_count>0) 
//  {
////	  led_bank2 &= ~(1<<12) ;
//	  dim_timer_setting[12] = 8 ;
//  }
//  else  		 //led_bank2 |= (1<<12) ;
//	   dim_timer_setting[12] = 0 ;
//  if(rx_count>0) 
//	  //led_bank2 &= ~(1<<13) ;
//		dim_timer_setting[13] = 8 ;
//  else  		
//	  //led_bank2 |= (1<<13) ;
//		dim_timer_setting[13] = 0 ;
//}





void tabulate_LED_STATE(void)
{
  static u16 beat_count = 0;	
  static u8 i = 0 ;
  uint8_t oneswitch_buf ;
  beat_count ++ ;
  if(beat_count >=25 ) 
  {
		beat_count = 0 ;
	  if(heart_beat_led == LED_ON)
		heart_beat_led = LED_OFF ;
		else
		heart_beat_led = LED_ON ;
		if(heart_beat_led == LED_ON)
		{
	
#if T36CTA			
			//GPIO_ResetBits(GPIOA, GPIO_Pin_13);
			//led_bank1 &= ~(1<<5) ;
			//led_bank2 &= ~(1<<13) ;
#else
			
				led_bank2 &= ~(1<<13) ;
#endif
			
		}
		else
		{
				
#if T36CTA	
			//GPIO_SetBits(GPIOA, GPIO_Pin_13);
			//led_bank1 |= (1<<5) ;
		//led_bank2 |= (1<<13) ;	
#else
		led_bank2 |= (1<<13) ;	
#endif
		}

  }
  #if	T38AI8AO6DO 
  
  for(i=0; i<8; i++)
  {
 if((inputs[i].digital_analog == 0)&&((inputs[i].range == ON_OFF)||(inputs[i].range == OPEN_CLOSED)||(inputs[i].range == START_STOP)
		||(inputs[i].range == ENABLED_DISABLED)||(inputs[i].range == ALARM_NORMAL)||(inputs[i].range ==HIGH_NORMAL)
		||(inputs[i].range == LOW_NORMAL)||(inputs[i].range == YES_NO)))	
		{
				if(AD_Value[i]<2048)  
				{
					led_bank2 &= ~(1<<(i+3)) ;
				}
				else
				{
					led_bank2 |= (1<<(i+3)) ;
				}
		}
//		else if(inputs[i].range == UNUSED)
//		{
//			led_bank2 |= (1<<(i+3)) ;
//		}
		else
		{	
				if((AD_Value[i]>2048)&&(inputs[i].range != UNUSED))  
				{
					led_bank2 &= ~(1<<(i+3)) ;
				}
				else
				{
					led_bank2 |= (1<<(i+3)) ;
				}
		}
  }
  for(i=0; i<6; i++)
  {
	oneswitch_buf = modbus.switch_gourp[0]>>(i*2) & 0x03 ;
	 if(i<3)
	 {
		if(oneswitch_buf == SW_OFF) led_bank2 |= (1<<i) ;
		else if(oneswitch_buf == SW_HAND) led_bank2 &= ~(1<<i) ;  
		else if(oneswitch_buf == SW_AUTO) 
		{
			if(outputs[i].value > 0)
			led_bank2 &= ~(1<<i) ; 
			else if(outputs[i].value == 0)
			led_bank2 |= (1<<i) ;			
		}
	 }
	 else
	 {
		if(oneswitch_buf == SW_OFF) led_bank1 |= (1<<(i-3)) ;
		else if(oneswitch_buf == SW_HAND) led_bank1 &= ~(1<<(i-3)) ; 
		else if(oneswitch_buf == SW_AUTO) 
		{
			if(outputs[i].value > 1)
			led_bank1 &= ~(1<<(i-3)) ; 
			else 
			led_bank1 |= (1<<(i-3)) ;			
		}	
	 }
	  
  }
  
  	for(i = 0; i< MAX_AO; i++)
	{
		oneswitch_buf = modbus.switch_gourp[1]>>(i*2) & 0x03 ;

		if(oneswitch_buf == SW_OFF) led_bank1 |= (1<<(i+3)) ;
		else if(oneswitch_buf == SW_HAND) led_bank1 &= ~(1<<(i+3)) ;
		else if(oneswitch_buf == SW_AUTO) 
		{
			if(outputs[i+MAX_DO].value  >=512)
			{
				led_bank1 &= ~(1<<(i+3)) ;	
			}
			else
			{
				led_bank1 |= (1<<(i+3)) ;	
			}		
		}
	}
  
  
  #endif

#if T36CTA	
	#if T36CTA_REV1

	for(i=0; i<5; i++)
	{
		if((inputs[i].digital_analog == 0)&&((inputs[i].range == ON_OFF)||(inputs[i].range == OPEN_CLOSED)||(inputs[i].range == START_STOP)
			||(inputs[i].range == ENABLED_DISABLED)||(inputs[i].range == ALARM_NORMAL)||(inputs[i].range ==HIGH_NORMAL)
			||(inputs[i].range == LOW_NORMAL)||(inputs[i].range == YES_NO)))	
			{
					if(AD_Value[i]<2048)  
					{
						led_bank2 &= ~(1<<(i+6)) ;
					}
					else
					{
						led_bank2 |= (1<<(i+6)) ;
					}
			}
			else
			{	
					if((AD_Value[i]>2048)&&(inputs[i].range != UNUSED))  
					{
						led_bank2 &= ~(1<<(i+6)) ;
					}
					else
					{
						led_bank2 |= (1<<(i+6)) ;
					}
			}
		}
	for(i=5; i< 13;i++)
	{
			if((inputs[i].digital_analog == 0)&&((inputs[i].range == ON_OFF)||(inputs[i].range == OPEN_CLOSED)||(inputs[i].range == START_STOP)
			||(inputs[i].range == ENABLED_DISABLED)||(inputs[i].range == ALARM_NORMAL)||(inputs[i].range ==HIGH_NORMAL)
			||(inputs[i].range == LOW_NORMAL)||(inputs[i].range == YES_NO)))	
			{
				if(AD_Value[i]<2048)  
				{
							led_bank1 &= ~(1<<(i+MAX_DO-5)) ;
				}
				else
				{
							led_bank1 |= (1<<(i+MAX_DO-5)) ;
				}
			
			}
			else
			{
				if((AD_Value[i]>2048)&&(inputs[i].range != UNUSED))  
				{
							led_bank1 &= ~(1<<(i+MAX_DO-5)) ;
				}
				else
				{
							led_bank1 |= (1<<(i+MAX_DO-5)) ;
				}
			}
	}	
	for(i=13; i< 19;i++)
	{
		if((CT_Vaule[i-13]>50) && (CT_Vaule[i-13]<10000))
		{
					led_bank2 &= ~(1<<(i-13)) ;
		}
		else
		{
					led_bank2 |= (1<<(i-13)) ;
		}
			
	}	
	#endif
	#if T36CTA_REV2
	for(i=0; i<8; i++)
	{
		if((inputs[i].digital_analog == 0)&&((inputs[i].range == ON_OFF)||(inputs[i].range == OPEN_CLOSED)||(inputs[i].range == START_STOP)
			||(inputs[i].range == ENABLED_DISABLED)||(inputs[i].range == ALARM_NORMAL)||(inputs[i].range ==HIGH_NORMAL)
			||(inputs[i].range == LOW_NORMAL)||(inputs[i].range == YES_NO)))	
			{
					if(AD_Value[i]<2048)  
					{
						led_bank1 &= ~(1<<(i+MAX_DO)) ;
					}
					else
					{
						led_bank1 |= (1<<(i+MAX_DO)) ;
					}
			}
			else
			{	
					if((AD_Value[i]>2048)&&(inputs[i].range != UNUSED))  
					{
						led_bank1 &= ~(1<<(i+MAX_DO)) ;
					}
					else
					{
						led_bank1 |= (1<<(i+MAX_DO)) ;
					}
			}
	}
	for(i=0; i< 6;i++)
	{
		if((CT_Vaule[i]>50) && (CT_Vaule[i]<10000))
		{
					led_bank2 &= ~(1<<(i+5)) ;
		}
		else
		{
					led_bank2 |= (1<<(i+5)) ;
		}
			
	}	
<<<<<<< HEAD
	  if(acc_led_count>0)
	  {
		  led_bank2 &= ~(1<<3) ;
	  }
	  else
	  {
		  led_bank2 |= (1<<3) ;
	  }
=======
>>>>>>> refs/remotes/origin/T3_MODULE_ARM
	#endif
	
  for(i=0; i<MAX_DO; i++)
  {
	oneswitch_buf = modbus.switch_gourp[0]>>(i*2) & 0x03 ;
	 if(i<3)
	 {
		if(oneswitch_buf == SW_OFF) led_bank1 |= (1<<i) ;
		else if(oneswitch_buf == SW_HAND) led_bank1 &= ~(1<<i) ;  
		else if(oneswitch_buf == SW_AUTO) 
		{
			if(outputs[i].value > 0)
			led_bank1 &= ~(1<<i) ; 
			else if(outputs[i].value == 0)
			led_bank1 |= (1<<i) ;			
		}
	 }
	 else
	 {
		if(oneswitch_buf == SW_OFF) led_bank1 |= (1<<(i-3)) ;
		else if(oneswitch_buf == SW_HAND) led_bank1 &= ~(1<<(i-3)) ; 
		else if(oneswitch_buf == SW_AUTO) 
		{
			if(outputs[i].value > 1)
			led_bank1 &= ~(1<<(i-3)) ; 
			else 
			led_bank1 |= (1<<(i-3)) ;			
		}	
	 }
	  
  }
	
#endif	
  #ifdef	T322AI
  for(i=0; i<11; i++)
  {
	if((inputs[i].digital_analog == 0)&&((inputs[i].range == ON_OFF)||(inputs[i].range == OPEN_CLOSED)||(inputs[i].range == START_STOP)
	||(inputs[i].range == ENABLED_DISABLED)||(inputs[i].range == ALARM_NORMAL)||(inputs[i].range ==HIGH_NORMAL)
	||(inputs[i].range == LOW_NORMAL)||(inputs[i].range == YES_NO)))
	{
	   if(AD_Value[i]<2048)  
		{
					led_bank2 &= ~(1<<i) ;
		}
		else
		{
					led_bank2 |= (1<<i) ;
		}
	
	}
	else if(inputs[i].range == UNUSED)
	{
			led_bank2 |= (1<<i) ;
	}
	else
	{
	  if(AD_Value[i]>2048)  
		{
					led_bank2 &= ~(1<<i) ;
		}
		else
		{
					led_bank2 |= (1<<i) ;
		}
	}
	if((inputs[i+11].digital_analog == 0)&&((inputs[i+11].range == ON_OFF)||(inputs[i+11].range == OPEN_CLOSED)||(inputs[i+11].range == START_STOP)
	||(inputs[i+11].range == ENABLED_DISABLED)||(inputs[i+11].range == ALARM_NORMAL)||(inputs[i+11].range ==HIGH_NORMAL)
	||(inputs[i+11].range == LOW_NORMAL)||(inputs[i+11].range == YES_NO)))
	{
		if(AD_Value[i+11]<2048)  
		{
					led_bank1 &= ~(1<<i) ;
		}
		else
		{
					led_bank1 |= (1<<i) ;
		}
	
	}
	else if(inputs[i+11].range == UNUSED)
	{
			led_bank1 |= (1<<i) ;
	}
	else
	{
		if(AD_Value[i+11]>2048)  
		{
					led_bank1 &= ~(1<<i) ;
		}
		else
		{
					led_bank1 |= (1<<i) ;
		}
	}
  }
  #endif
  
#if defined T36CTA
	if(rfm69_rx_count>0)rfm69_rx_count--;
    if(rfm69_tx_count>0)rfm69_tx_count--;
	if(rfm69_rx_count>0)
		led_bank1 &= ~(1<<11) ;
	else
		led_bank1 |= (1<<11) ;
	if(rfm69_tx_count>0)
		led_bank1 &= ~(1<<10) ;
	else
		led_bank1 |= (1<<10) ;
	
#endif
  
  if(net_rx_count> 0) net_rx_count -- ;  
  if(net_tx_count> 0) net_tx_count -- ;
//  led_bank1 &= ~(1<<12) ;
//  led_bank1 &= ~(1<<11) ;
  if(net_rx_count>0) 
#if defined T36CTA
	//GPIO_ResetBits(GPIOA, GPIO_Pin_13);
    //led_bank1 &= ~(1<<13) ;
	t36ct_net_led = LED_ON;
#else  
	  led_bank1 &= ~(1<<12) ;
#endif
  else  
#if defined T36CTA
	//GPIO_SetBits(GPIOA, GPIO_Pin_13);
  //led_bank1 |= (1<<13) ;
	t36ct_net_led = LED_OFF;
#else  
	  led_bank1 |= (1<<12) ;
#endif  

   if(net_tx_count>0) 
#if defined T36CTA
	led_bank1 &= ~(1<<12) ;
#else   
	  led_bank1 &= ~(1<<11) ;
#endif
  else  
#if defined T36CTA	 
	led_bank1 |= (1<<12) ;
#else
	  led_bank1 |= (1<<11) ;
#endif
  
  if(tx_count>0) tx_count-- ;
  if(rx_count>0) rx_count-- ;

  if(tx_count>0) 
  {
	  led_bank2 &= ~(1<<11) ;
  }
  else  		 
		led_bank2 |= (1<<11) ;
  if(rx_count>0) 
	  led_bank2 &= ~(1<<12) ;
  else  		
	  led_bank2 |= (1<<12) ;

}

void refresh_led(void)
{
	static u8 led_switch = 0 ;
#if T36CTA
    //u16 port_temp;
	u8 i;
	//port_temp = GPIO_ReadOutputData(GPIOE);
	//port_temp = 0x6000&port_temp ;
#endif	
	led_switch = !led_switch ;
	if(led_switch)
	{
#if T36CTA
		GPIO_ResetBits(GPIOA, GPIO_Pin_12);
		GPIO_SetBits(GPIOE, GPIO_Pin_15);
		
		if(t36ct_net_led == LED_ON)
			GPIO_ResetBits(GPIOA, GPIO_Pin_13);
		else
			GPIO_SetBits(GPIOA, GPIO_Pin_13);
		
		for(i = 0; i< 13; i++)
		{
			if( led_bank1 & (1<<i))
			{
				PEout(i) = 1;
			}
			else
				PEout(i) = 0;
		}
#else		
		led_bank1 &= ~(1<<14);
		led_bank1 |= (1<<15);
		
		GPIO_Write(GPIOE, led_bank1) ;
#endif
	}
	else
	{
#if T36CTA
		GPIO_SetBits(GPIOA, GPIO_Pin_12);
		GPIO_ResetBits(GPIOE, GPIO_Pin_15);
		
		if(heart_beat_led == LED_ON)
			GPIO_ResetBits(GPIOA, GPIO_Pin_13);
		else
			GPIO_SetBits(GPIOA, GPIO_Pin_13);

		for(i = 0; i< 13; i++)
		{
			if( led_bank2 & (1<<i))
			{
				PEout(i) = 1;
			}
			else
				PEout(i) = 0;
		}
		
#else				
		led_bank2 |= (1<<14);
		led_bank2 &= ~(1<<15);
		
		GPIO_Write(GPIOE, led_bank2) ;
#endif
	}
}


