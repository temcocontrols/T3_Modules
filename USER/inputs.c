
#include "define.h"
#include "inputs.h"

#include "delay.h"
#include "modbus.h"
#include "../filter/filter.h"
#include "controls.h"
#if (defined T322AI) || (T38AI8AO6DO)	|| (defined T36CTA)
uint16_t data_change[MAX_AI_CHANNEL] = {0} ;
#endif

#if defined T36CTA
uint16_t air_flow_ad = 0;
#endif

#ifndef T3PT12
void range_set_func(u8 channel) ;
#define ADC_DR_ADDRESS  0x4001244C  
#endif
vu16 AD_Value[MAX_AI_CHANNEL] = {0};

void inputs_io_init(void)
{
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_0|ADC_Channel_1, 1, ADC_SampleTime_28Cycles5);
// IO Configure 
	
GPIO_InitTypeDef GPIO_InitStructure;
#ifdef T322AI	
/**************************PortA configure---ADC1*****************************************/
////RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA, ENABLE);
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;  
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
//GPIO_Init(GPIOA, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
GPIO_Init(GPIOA, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
GPIO_Init(GPIOC, &GPIO_InitStructure);	
///**************************PortB configure----ADC1*****************************************/
//RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOB, ENABLE);
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
//GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#if (defined T38AI8AO6DO) 
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
GPIO_Init(GPIOA, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#if (defined T36CTA)

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
GPIO_Init(GPIOD, &GPIO_InitStructure);

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;//| GPIO_Pin_8 | GPIO_Pin_9;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
GPIO_Init(GPIOC, &GPIO_InitStructure);

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
GPIO_Init(GPIOE, &GPIO_InitStructure);

RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC2|RCC_APB2Periph_GPIOA, ENABLE);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
GPIO_Init(GPIOA, &GPIO_InitStructure);

RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC2|RCC_APB2Periph_GPIOC, ENABLE);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif


/**************************PortC configure----ADC1*****************************************/
RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOC, ENABLE);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
GPIO_Init(GPIOC, &GPIO_InitStructure);




	
}
void inputs_adc_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/* configuration ------------------------------------------------------*/  
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//
//	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//DMA CONTINUS MODE
//	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//
//	#ifdef T322AI
	ADC_InitStructure.ADC_NbrOfChannel = 1;//
	ADC_Init(ADC1, &ADC_InitStructure);
//	ADC_Init(ADC2, &ADC_InitStructure);
	 /* ADC1 regular channels configuration */ 
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_4 , 1, ADC_SampleTime_239Cycles5);  
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_5 , 2, ADC_SampleTime_239Cycles5); 
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_6 , 3, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_7 , 4, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_8 , 5, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_9 , 6, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_10 , 7, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_11 , 8, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_12 , 9, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_13 , 10, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_14 , 11, ADC_SampleTime_239Cycles5);
//	ADC_DMACmd(ADC1, ENABLE); //enable dma
//	ADC_Cmd(ADC2, ENABLE); 
//	ADC_ResetCalibration(ADC2);
//	while(ADC_GetResetCalibrationStatus(ADC2) == SET);
//	ADC_StartCalibration(ADC2);
//	while(ADC_GetCalibrationStatus(ADC2) == SET);
//	#endif
	ADC_Cmd(ADC1, ENABLE); 
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1)== SET)
	{
		;
	}		
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
     ADC_StartCalibration(ADC1);
     while(ADC_GetCalibrationStatus(ADC1) == SET);
	
#if defined T36CTA
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	/* configuration ------------------------------------------------------*/  
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 7;//
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Cmd(ADC2, ENABLE); 
	/* Enable ADC2 reset calibaration register */   
	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2)== SET)
	{
		;
	}		
     ADC_StartCalibration(ADC2);
     while(ADC_GetCalibrationStatus(ADC2) == SET);
#endif	
}

//void dma_adc_init(void)
//{
///************DMA configure*******************************/
//DMA_InitTypeDef DMA_InitStructure;
//DMA_DeInit(DMA1_Channel1);
//DMA_InitStructure.DMA_PeripheralBaseAddr = ADC_DR_ADDRESS;
//DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value  ;
//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//DMA_InitStructure.DMA_BufferSize = 11; 
//DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
//DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
//DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
//DMA_Init(DMA1_Channel1, &DMA_InitStructure); 
//DMA_Cmd(DMA1_Channel1, ENABLE); 
//}
void inputs_init(void) 
{
	#ifndef T3PT12
	inputs_io_init();
	inputs_adc_init();
	//dma_adc_init();
	#endif
	
}

#if defined T36CTA
void inputs_scan(void)
{
	static u16 new_ad[22] ;
	static u16 old_ad[22]= {4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095};
	static u8 channel_count =0 ;
	u16 port_temp;
	u16	swap_adc ;
	u8 i;
	uint16_t ad_buf = 0 ;
#if T36CTA_REV1
	if( channel_count <13)
	{
		if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
		{
			AD_Value[channel_count]=ADC_getChannal(ADC1,ADC_Channel_14);
			new_ad[channel_count] = AD_Value[channel_count] ;
			if((old_ad[channel_count]> new_ad[channel_count])&&((old_ad[channel_count] - new_ad[channel_count])>1000))
			{
				modbus.pulse[channel_count].word++ ;
				data_change[channel_count] = 1 ;
			}
			old_ad[channel_count] = new_ad[channel_count];

		}
		else
		{
			AD_Value[channel_count]= ADC_getChannal(ADC1,ADC_Channel_14);
		}

		GPIO_ResetBits(GPIOD,  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_12 );
		if(channel_count>7)
			GPIO_SetBits(GPIOC, GPIO_Pin_6);
		else
			GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		port_temp = GPIO_ReadOutputData(GPIOD);
		port_temp = 0x8fff&port_temp ;
		
		GPIO_Write(GPIOD, (port_temp|(channel_count<<12)));
		CHA_SEL4 = 1  ;
		range_set_func(channel_count);
		
		channel_count++;
	}
	else
	{
//		swap_adc = AD_Value[0];
//		for(i=0; i< 12; i++)
//		{
//			AD_Value[i] = AD_Value[i+1];
//		}
//		AD_Value[12] = swap_adc;
//		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_12));
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_0)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_1)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_2)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_3)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_4)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_5)*10/75);//air_flow_ad;//(ADC_getChannal(ADC2,ADC_Channel_5)*10/75);
		
//		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_0));
//		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_1));
//		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_2));
//		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_3));
//		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_4));
//		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_5));
		
		channel_count %= MAX_AI_CHANNEL;
	}
#else 
	if( channel_count <8)
	{
		
		GPIO_ResetBits(GPIOD,  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_12 );

		//port_temp = GPIO_ReadOutputData(GPIOD);
		//port_temp = 0x8fff&port_temp ;
		
		//GPIO_Write(GPIOD, (port_temp|(channel_count<<12)));
		for( i=0; i<8; i++)
		{
			if(channel_count & (1<<0))
			{
				PDout(12) = 1;
			}
			if(channel_count & (1<<0))
			{
				PDout(13) = 1;
			}
			if(channel_count & (1<<0))
			{
				PDout(14) = 1;
			}
		}
		CHA_SEL4 = 1  ;
		range_set_func(channel_count);
		
		if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
		{
			AD_Value[channel_count]=ADC_getChannal(ADC1,ADC_Channel_14);
			new_ad[channel_count] = AD_Value[channel_count] ;
			if((old_ad[channel_count]> new_ad[channel_count])&&((old_ad[channel_count] - new_ad[channel_count])>1000))
			{
				modbus.pulse[channel_count].word++ ;
				data_change[channel_count] = 1 ;
			}
			old_ad[channel_count] = new_ad[channel_count];

		}
		else
		{
			AD_Value[channel_count]= ADC_getChannal(ADC1,ADC_Channel_14);
		}

		
		channel_count++;
	}
	else if( (channel_count>= 8)&&(channel_count<13))
	{
		AD_Value[channel_count++]= 0;
		AD_Value[channel_count++]= 0;
		AD_Value[channel_count++]= 0;
		AD_Value[channel_count++]= 0;
		AD_Value[channel_count++]= 0;
	}
	else
	{
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_0)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_1)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_2)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_3)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_4)*10/75);
		AD_Value[channel_count++]= (ADC_getChannal(ADC2,ADC_Channel_5)*10/75);//air_flow_ad;//(ADC_getChannal(ADC2,ADC_Channel_5)*10/75);

		
		channel_count %= MAX_AI_CHANNEL;
	}
#endif	
}
#endif

#if (defined T38AI8AO6DO)
void inputs_scan(void)
{
	static u16 new_ad[22] ;
	static u16 old_ad[22]= {4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095};
	static u8 channel_count =0 ;
	uint16_t ad_buf = 0 ;
//	static u16 sum = 0 ;
//	static u8 sum_times = 0 ;


	if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
	{
		AD_Value[channel_count]=ADC_getChannal(ADC1,ADC_Channel_14);
		new_ad[channel_count] = AD_Value[channel_count] ;
		if((old_ad[channel_count]> new_ad[channel_count])&&((old_ad[channel_count] - new_ad[channel_count])>1000))
		{
			modbus.pulse[channel_count].word++ ;
			data_change[channel_count] = 1 ;
		}
		old_ad[channel_count] = new_ad[channel_count];

	}
	else
	{
//		sum += ADC_getChannal(ADC1,ADC_Channel_14);
//		sum_times ++ ;
//		if(sum_times == 1)
//		{
//			sum_times = 0 ;
//			AD_Value[channel_count] = sum / 1 ;
//			sum = 0 ;

//			
//		}
		AD_Value[channel_count]= ADC_getChannal(ADC1,ADC_Channel_14);
	}
			channel_count++;
			channel_count %= MAX_AI_CHANNEL;
			GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 );
			GPIO_SetBits(GPIOC, channel_count);
			CHA_SEL4 = 1  ;
//			range_set_func((inputs[channel_count].decom>>4)&0x0f);
			range_set_func(channel_count);
}
#endif
#ifdef T322AI
void inputs_scan(void)
{
	static u16 new_ad[22] ;
	static u16 old_ad[22]= {4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095};
	static u8 channel_count =0 ;
	u16 port_temp ;
	AD_Value[channel_count]=ADC_getChannal(ADC1,ADC_Channel_14);
	if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
	{
		new_ad[channel_count] = AD_Value[channel_count] ;
		if(channel_count>=11)
		{
			if((old_ad[channel_count]> new_ad[channel_count])&&((old_ad[channel_count] - new_ad[channel_count])>2000))
			{
				modbus.pulse[channel_count].word++ ;
				data_change[channel_count] = 1 ;
			}
			old_ad[channel_count] = new_ad[channel_count];
		}
	}
	channel_count++ ;
	if(channel_count == MAX_AI_CHANNEL) channel_count =0 ;
	if(channel_count <11)
	{
		while(1)
		{
			if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
			{
				channel_count ++ ;
				if(channel_count == 11) break ;
			}
			else
			{
				break ;
			}
		}
	
	}
//	else if(channel_count ==MAX_AI_CHANNEL)
//	{
//		channel_count = 0 ;
//		while(1)
//		{
//			if(channel_count<11)
//			{
//				if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
//				{
//					channel_count ++ ;
//				}
//				else
//				{
//					break ;
//				}
//			}
//			else
//			{					
//				break ;
//			}
//		}
//	}
	
//	if(channel_count == MAX_AI_CHANNEL)
//	{
//		channel_count = 0 ;
//		while(1)
//		{
//			if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
//			{
//				channel_count ++ ;
//			}
//			else
//			{
//				break ;
//			}
//		}
//	}
	//channel_count = 12 ;
//	channel_count++;
//	channel_count %= MAX_AI_CHANNEL;
	   
//   if(channel_count % 8 == 0)
//   {
//      range_set_func(T3_I020ma);
//      delay_ms(5);
//   }
  
//	while(1)
//	{
//		if(channel_count >= 11 )  break ;
//		if((inputs[channel_count].range == N0_2_32counts)||(inputs[channel_count].range ==HI_spd_count))
//		{
//			channel_count++;
//		}
//		else
//		{
//			break; 
//		}
//		
//	}
	
//	if(!((modbus.range[channel_count] == PULSE)&&(channel_count<11)))
	{
	
//		GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
//		GPIO_SetBits(GPIOA, channel_count);
		port_temp = GPIO_ReadOutputData(GPIOA);
		port_temp = 0xffe0&port_temp ;

//		port_temp = port_temp|0x10 ;		// modset = 1 diasble the first mode set
//		GPIO_Write(GPIOA, port_temp|channel_count);
//	    GPIO_Write(GPIOA, (port_temp|channel_count)&0xffef);//enasble the first mode set
		
//		GPIO_Write(GPIOA, port_temp|0x0010);
		
//		if(((modbus.range[channel_count] == PULSE)&&(channel_count<11)))
//		GPIO_Write(GPIOA, port_temp|channel_count|0x0010);
//		else
//		GPIO_Write(GPIOA, (port_temp|channel_count)&0xffef);
		GPIO_Write(GPIOA, (port_temp|channel_count));
		
		
		
//		GPIO_Write(GPIOA, port_temp|channel_count|0x0008);
//		GPIO_Write(GPIOA, port_temp|channel_count);
		
	}
	
//	range_set_func((inputs[channel_count].decom>>4)&0x0f);
		range_set_func(channel_count);
}
#endif
u16 ADC_getChannal(ADC_TypeDef* ADCx, u8 channal)
{
         uint16_t tem = 0;
		 ADC_ClearFlag(ADCx, ADC_FLAG_EOC);
         ADC_RegularChannelConfig(ADCx, channal, 1, ADC_SampleTime_55Cycles5);
         ADC_SoftwareStartConvCmd(ADCx, ENABLE);         
         while(ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
         tem = ADC_GetConversionValue(ADCx);
         return tem;        
 }

 
 void range_set_func(u8 channel)
 {
	 
	 #ifndef T3PT12
	if ((inputs[channel].digital_analog == 1)&&(inputs[channel].range ==V0_5||inputs[channel].range ==P0_100_0_5V))
	 {
				RANGE_SET0 = 1 ;
				RANGE_SET1 = 0 ;
				inputs[channel].decom = inputs[channel].decom &0x0f ;
				inputs[channel].decom |= (T3_V05<<4) ;
	 }
	else if ((inputs[channel].digital_analog == 1)&&(inputs[channel].range == V0_10_IN ))
	 {
				RANGE_SET0 = 0 ;
				RANGE_SET1 = 1 ;
				inputs[channel].decom = inputs[channel].decom &0x0f ;
				inputs[channel].decom |= (T3_V010<<4) ;
	 }
	else if ((inputs[channel].digital_analog == 1)&&(inputs[channel].range == I0_100Amps ||inputs[channel].range == I0_20ma||inputs[channel].range ==P0_100_4_20ma))
	 {
				RANGE_SET0 = 0 ;
				RANGE_SET1 = 0 ;
				inputs[channel].decom = inputs[channel].decom &0x0f ;
				inputs[channel].decom |= (T3_I020ma<<4) ;
	 }
	 else /*if((inputs[channel].digital_analog == 0)||(inputs[channel].digital_analog == 1 &&((inputs[channel].range>=Y3K_40_150DegC)&&inputs[channel].range<= A10K_60_200DegF))
		 ||(inputs[channel].range == Frequence)||(inputs[channel].range == HI_spd_count)||(inputs[channel].range == HI_spd_count))*/
	 {
				RANGE_SET0 = 1 ;
				RANGE_SET1 = 1 ;
				inputs[channel].decom = inputs[channel].decom &0x0f ;
				inputs[channel].decom |= (T3_NO_USE<<4) ;
	 }
	 #endif
 }
 

 
 #ifdef T322AI
  const struct
 {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	uint8_t PortSource;
	uint8_t PinSource;
	uint8_t NVIC_IRQChannel;
	uint32_t Exit_line ; 
	 
 } _STR_PLUSE_SETTING_[11] = 
 {
	{GPIOA, GPIO_Pin_7, GPIO_PortSourceGPIOA, GPIO_PinSource7, EXTI9_5_IRQn, EXTI_Line7},
	{GPIOA, GPIO_Pin_6, GPIO_PortSourceGPIOA, GPIO_PinSource6, EXTI9_5_IRQn, EXTI_Line6},
	{GPIOA, GPIO_Pin_5, GPIO_PortSourceGPIOA, GPIO_PinSource5, EXTI9_5_IRQn, EXTI_Line5},
	{GPIOB, GPIO_Pin_11, GPIO_PortSourceGPIOB, GPIO_PinSource11, EXTI15_10_IRQn, EXTI_Line11},
	{GPIOB, GPIO_Pin_10, GPIO_PortSourceGPIOB, GPIO_PinSource10, EXTI15_10_IRQn, EXTI_Line10},
	{GPIOC, GPIO_Pin_3, GPIO_PortSourceGPIOC, GPIO_PinSource3, EXTI3_IRQn, EXTI_Line3},
	{GPIOC, GPIO_Pin_1, GPIO_PortSourceGPIOC, GPIO_PinSource1, EXTI1_IRQn, EXTI_Line1},
	{GPIOC, GPIO_Pin_2, GPIO_PortSourceGPIOC, GPIO_PinSource2, EXTI2_IRQn, EXTI_Line2},
	{GPIOD, GPIO_Pin_12, GPIO_PortSourceGPIOD, GPIO_PinSource12, EXTI15_10_IRQn, EXTI_Line12},
	{GPIOC, GPIO_Pin_0, GPIO_PortSourceGPIOC, GPIO_PinSource0, EXTI0_IRQn, EXTI_Line0},
	{GPIOD, GPIO_Pin_14, GPIO_PortSourceGPIOD, GPIO_PinSource14, EXTI15_10_IRQn, EXTI_Line14},
 };
  void pulse_set(uint8_t channel)
 {
	//u8 port_source ;
	 
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	GPIO_InitStructure.GPIO_Pin = _STR_PLUSE_SETTING_[channel].GPIO_Pin ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(_STR_PLUSE_SETTING_[channel].GPIOx, &GPIO_InitStructure);
	GPIO_SetBits(_STR_PLUSE_SETTING_[channel].GPIOx, _STR_PLUSE_SETTING_[channel].GPIO_Pin );
	
	 
	GPIO_EXTILineConfig(_STR_PLUSE_SETTING_[channel].PortSource, _STR_PLUSE_SETTING_[channel].PinSource ); 
	//EXTI_InitStructure.EXTI_Line = (uint16_t)1<<GPIO_Pin ;
	EXTI_InitStructure.EXTI_Line  = _STR_PLUSE_SETTING_[channel].Exit_line ; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = _STR_PLUSE_SETTING_[channel].NVIC_IRQChannel ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
 
 }

void EXTI0_IRQHandler(void)				//input10
{
	if(EXTI->PR & (1 << 0))	//是0线的中断
	{    
		EXTI->PR = (1 << 0);	//清除LINE0上的中断标志位
	}
	modbus.pulse[9].word++ ;
	data_change[9] = 1 ;
}
void EXTI1_IRQHandler(void)		//input6
{
	if(EXTI->PR & (1 << 1))	//是3线的中断
	{    
		EXTI->PR = (1 << 1);	//清除LINE3上的中断标志位
	}
	modbus.pulse[6].word++ ;
	data_change[6] = 1 ;
}
void EXTI2_IRQHandler(void)
{
	if(EXTI->PR & (1 << 2))	//是3线的中断						//input7
	{    
		EXTI->PR = (1 << 2);	//清除LINE3上的中断标志位
	}
	modbus.pulse[7].word++ ;
	data_change[7] = 1 ;
}
void EXTI3_IRQHandler(void)											//input5
{
	if(EXTI->PR & (1 << 3))	//是3线的中断
	{    
		EXTI->PR = (1 << 3);	//清除LINE3上的中断标志位
	}
	modbus.pulse[5].word++ ;
	data_change[5] = 1 ;

}
//void EXTI4_IRQHandler(void)										//
//{
//	if(EXTI->PR & (1 << 4))	//是3线的中断
//	{    
//		EXTI->PR = (1 << 4);	//清除LINE3上的中断标志位
//	}
//}


void EXTI9_5_IRQHandler(void)
{

	if(EXTI->PR & (1 << 7))	//是7线的中断
	{      
		EXTI->PR  = (1 << 7);	//清除LINE7上的中断标志位
		modbus.pulse[0].word++ ;
		data_change[0] = 1 ;

	}
	if(EXTI->PR & (1 << 6))	//是6线的中断
	{      
		EXTI->PR  = (1 << 6);	//清除LINE6上的中断标志位
		modbus.pulse[1].word++ ;
		data_change[1] = 1 ;

	}
	if(EXTI->PR & (1 << 5))	//是5线的中断
	{      
		EXTI->PR  = (1 << 5);	//清除LINE5上的中断标志位
		modbus.pulse[2].word++ ;
		data_change[2] = 1 ;
	}
} 

void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & (1 << 11))	//是11线的中断
	{      
		EXTI->PR  = (1 << 11);	//清除LINE11上的中断标志位
		modbus.pulse[3].word++ ;
		data_change[3] = 1 ;
	}
	if(EXTI->PR & (1 << 12))	//是12线的中断
	{      
		EXTI->PR  = (1 << 12);	//清除LINE12上的中断标志位
		modbus.pulse[8].word++ ;
		data_change[8] = 1 ;
	}
	if(EXTI->PR & (1 << 10))	//是13线的中断
	{      
		EXTI->PR  = (1 << 10);	//清除LINE13上的中断标志位
		modbus.pulse[4].word++ ;
		data_change[4] = 1 ;
	}
	if(EXTI->PR & (1 << 14))	//是14线的中断
	{      
		EXTI->PR  = (1 << 14);	//清除LINE14上的中断标志位
		modbus.pulse[10].word++ ;
		data_change[10] = 1 ;
	}
}
#endif


 