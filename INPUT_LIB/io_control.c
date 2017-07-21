#include "modbus.h"
#include "ud_str.h"
#include "controls.h"
#include "inputs.h"
// 返回input的原始ADC值，10bit， 如果不是10位，需要做相应转化
uint16_t get_input_raw(uint8_t point)
{
//	return (modbus.input[point]+1)/4;
	return AD_Value[point]/4 ;
}
#ifndef  T3PT12
static uint16_t analog_buffer[MAX_AI_CHANNEL] ;
#if (defined T38AI8AO6DO) || (defined T36CTA)
uint16_t output_raw[MAX_OUTS]={0} ;
#endif

// 根据input_type[point]设置相应的硬件
void Set_Input_Type(uint8_t point)
{
	
}




// 设置input的原值
// if digital output, 0 - off, 1000 - on
// if analog ouput, 0 - 10v 对应 0-1023
#if (defined T38AI8AO6DO) || (defined T36CTA)
void set_output_raw(uint8_t point,uint16_t value)
{
	output_raw[point] = value;
//	inputs[value].value = value ;
}
#endif



// 把adc值转化成5v的range显示
// 如果是input最新模块不需要修改
uint32_t conver_by_unit_5v (uint32_t sample)
{	
//	sample =  ( sample * 5000L ) >> 10;
	return ( sample * 5000L ) >> 10;		
}


// 把adc值转化成10v的range显示
// 如果是input最新模块不需要修改
unsigned int conver_by_unit_10v(uint32_t sample)
{
	return  ( sample * 10000L ) >> 10;

}

// 把adc值转化成customer table的range显示
// 如果是input最新模块不需要修改
uint32_t conver_by_unit_custable(uint8_t point,uint32_t sample)
{	
	if(input_type[point] == INPUT_V0_5)
	{
			return  ( 5000L * sample) >> 10;		
	}
	else if(input_type[point] == INPUT_I0_20ma)
	{
		return ( 20000L * sample ) >> 10; 
	}
	else if(input_type[point] == INPUT_0_10V)
	{
			return ( 10000l * sample) >> 10;		

	}
	else if(input_type[point] == INPUT_THERM)
	{
		return 0;//get_input_value_by_range( inputs[point].range, sample );
	}
}


// 返回最大input数目
uint8_t get_max_input(void)
{	
	return MAX_AIS;
}
// 返回最大output数目
#if (defined T38AI8AO6DO )|| (defined T36CTA)
uint8_t get_max_output(void)
{	
	return MAX_OUTS;
}
#endif
// if有high speed 功能，返回high_spd_counter
uint32_t get_high_spd_counter(uint8_t point)
{
	return modbus.pulse[point].word ;
}
 void set_high_spd_counter(uint8_t point)
{
		modbus.pulse[point].word  = 0 ;
}



 unsigned int Filter(uint8_t channel,uint16_t input)
{
	// -------------FILTERING------------------
	s16  siDelta = 0;
	u16  uiResult = 0;
	u8  i = 0;
  	u16  uiTemp = 0;
	i = channel;
	uiTemp = input;  
	siDelta = uiTemp - analog_buffer[i];    //compare new reading and old reading

	// If the difference in new reading and old reading is greater than 5 degrees, implement rough filtering.
  if (( siDelta >= 100 ) || ( siDelta <= -100)) // deg f
	  	analog_buffer[i] = analog_buffer[i] + (siDelta >> 1);//1 
 			
	// Otherwise, implement fine filtering.
	else
	{			
	  analog_buffer[i] = (u32)analog_buffer[i]*inputs[i].filter;
	  analog_buffer[i] += (u32)uiTemp;
	  analog_buffer[i] = (u16)(analog_buffer[i]/(inputs[i].filter));			 	 
	}
	uiResult = analog_buffer[i]; 
 

	return uiResult;	
}
#endif
