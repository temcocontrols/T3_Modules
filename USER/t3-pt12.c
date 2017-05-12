#include "stdio.h"
#include "t3-pt12.h" 
#include "read_pt.h"
#include "modbus.h"
#include "inputs.h"
#ifdef T3PT12
#define PT1000  0x11 
#define PT100		0x12
#define NTC     0x13

_TEMP_VALUE_  linear_K , linear_B;	
_TEMP_VALUE_  linear_K_1 , linear_B_1;	
_TEMP_VALUE_ _temp_value[12];
//_TEMP_VALUE_ _offset_rc[12] ;	  // _offset_rc for temperture adujust.
unsigned char const  def_tab_pic_Type2_10K[21] =
			{					 // 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F 
//			 25, 41, 61, 83, 102, 113, 112, 101, 85, 67, 51, 38, 28, 21, 65 //MHF 20010_07 REVISE TEMP LOOKUP TABLE PER NATHANEAL			
			 // 10k termistor GREYSTONE -50 to 150 Deg.C 
			15,25,41,61,83,102,113,112,101,85,67,51,38,28,21,15,11,8,6,5,19	   //changed by ye

 			};

const float Rs[RPS] = {RP1, RP2, RP3, RP4}; //4 calibration resistance value for calculate K and B
const float Rs1[RPS] = {RP5, RP6, RP7, RP8}; //4 calibration resistance value for calculate K and B
int16_t NTC_look_up_table1(uint16_t count) ;
int16_t NTC_TEMPERATURE(uint16_t adc) ;

void min2method(float *K, float *B, u8 PointNum, float *Xbuf, const float *Ybuf)
{
	double sumx, sumx2, sumy, sumxy;
	u8 i;
	
	sumx = 0;
	sumx2 = 0;
	for(i = 0; i < PointNum; i++)
	{
		sumx += Xbuf[i];
		sumx2 += Xbuf[i] * Xbuf[i];
//		printf("sumx= %lf,sumx2=%lf\n\r ",sumx,sumx2 );
		
	}
	
	sumy = 0;
	for(i = 0; i < PointNum; i++)
	{
		sumy += Ybuf[i];
	}

	sumxy = 0;
	for(i = 0; i < PointNum; i++)
	{
		sumxy += Xbuf[i] * Ybuf[i];
	}
//	printf("sumy=%lf,sumxy=%lf\n\r ",sumy,sumxy );
	*K = ((PointNum*sumxy - sumx*sumy) / (PointNum*sumx2 - sumx*sumx));
	*B = ((sumx2*sumy - sumx*sumxy) / (PointNum*sumx2 - sumx*sumx));
}


float get_rtd_temperature(long rtd_ad , unsigned char channel)
{
	float fT0,fTX;
	float rtd_res ,rtd_tc;
	u16   RTD_R0  = 0 ;
//	u8  buffer_range = 0 ;
	float rtd_A = -0.000000577521439;
	float rtd_B = 0.003908319257;
	float rtd_C = -0.00000000000418347612;
	if(inputs[channel].digital_analog == 1)
	{
		if((inputs[channel].range == 1)||(inputs[channel].range == 2)||(inputs[channel].range == 5)||(inputs[channel].range == 6))
		{
			if((inputs[channel].range == 1)||(inputs[channel].range == 2))
			{
				rtd_res = linear_K.temp_C * rtd_ad + linear_B.temp_C;	//pt100 get rtd resistor
				RTD_R0 = 100 ;
			}
			else if((inputs[channel].range == 5)||(inputs[channel].range == 6))
			{
				rtd_res = linear_K_1.temp_C * rtd_ad + linear_B_1.temp_C;	//pt1000 get rtd resistor	
				RTD_R0 = 1000 ;

			}
			fT0 = (-1.0*rtd_B + sqrt(rtd_B*rtd_B - 4*rtd_A*(1 - rtd_res/RTD_R0))) / (2*rtd_A); // caculate the temperature
			if(rtd_res < RTD_R0)
			{
					fTX = (rtd_C*fT0*fT0*fT0*(fT0 - 100)) / (rtd_B + 2*rtd_A*fT0 + 3*rtd_C*fT0*fT0*(fT0 - 100) + rtd_C*fT0*fT0*fT0);
					rtd_tc = fT0 + fTX;
			}
			else
			{
				rtd_tc = fT0;
			}	

		}
		else if((inputs[channel].range == 3)||(inputs[channel].range == 4))
		{ 
			rtd_tc = (float)NTC_TEMPERATURE((rtd_ad>>14)&0x3ff)/10  ; 
		}
		if((inputs[channel].range == 1)||(inputs[channel].range == 3)||(inputs[channel].range == 5))
		{
			return rtd_tc ;
		}
		else if((inputs[channel].range == 2)||(inputs[channel].range == 4)||(inputs[channel].range == 6))
		{
				rtd_tc = rtd_tc * 9	;
				rtd_tc = rtd_tc / 5 ;
				rtd_tc = rtd_tc + 32 ;
				return rtd_tc ;	
		}
	}
	else
	{
			AD_Value[channel] = (rtd_ad>>10) ;
	}
}
    
int16_t NTC_TEMPERATURE(uint16_t adc)
{
	int16_t  siAdcResult;
	int16_t   siInput ;
	siAdcResult = NTC_look_up_table1(adc);
	if(siAdcResult & 0x8000)
	siInput = -(signed int)(siAdcResult & 0x7fff);
	else
	siInput = siAdcResult;
	
	return siInput ;
}
int16_t NTC_look_up_table1(uint16_t count)
{
	int16_t   xdata val;
  uint8_t  index=20;			//chagned by  ye 
	int16_t   xdata work_var;
 
	
			work_var= def_tab_pic_Type2_10K[index];	  
	if (work_var > count )
		{
			val =  (index-5)  * 100 ;	 //the highest temperature is 150c, changed by ye
			return ( val );
		  
		}
	do 
		{
			index--;

				work_var += def_tab_pic_Type2_10K[index];
			if( work_var > count)
				{
				val = ( work_var - count )*100;
				val /= def_tab_pic_Type2_10K[index];
				if(index >= 5)				      //changed by ye ,the 0c is fifth
				{
					val +=  (index - 5) * 100;
					val = val & 0x7fff;
				}
				else
				{
					val += index*100;
					val = 500 - val;			  //the lowest temperature is -50c
					val = val | 0x8000;
				}			 
				return (val);
				}
		} while (index) ;

			val =  33768;

			return ( val );
}
void update_temperature	(void)
{
	 u8  channel_loop = 0;

	#if 1
	for(channel_loop=0; channel_loop<12; channel_loop++)
	{
//			rs_data[channel_loop+8]=PT12_FITER(channel_loop,rs_data[channel_loop+8],inputs[channel_loop].filter);
//			rs_data[channel_loop+8] = 1985615 ;	
//			rs_data[channel_loop+8] = 1822311;
			_temp_value[channel_loop].temp_C =get_rtd_temperature(rs_data[channel_loop+8] , channel_loop)  ;			
//			inputs[channel_loop].value =  (int32_t)(_temp_value[channel_loop].temp_C);
//		 printf("%f,%d\n\r", _temp_value[channel_loop].temp_C,inputs[channel_loop].value);
//		 delay_ms(5); 
		//		printf("C[%u]=%u\n\r ",channel_loop,inputs[channel_loop].value);
	}
	#endif
}

float data_convert(float input_data , u8 resol_bit )
{
	u32   convert_temp;
	float  convert_output;
	input_data = input_data *10000 ;
	if(resol_bit == 2)
	{
		convert_temp =(u32)input_data	/ 100 ;
		convert_output =   convert_temp / 100.0 ;
	}
	else if(resol_bit == 1)
	{
		convert_temp =(u32)input_data	/ 1000 ;
		convert_output =   convert_temp / 10.0 ;	
	}
	else if(resol_bit == 3)
	{
		convert_temp =(u32)input_data	/ 10 ;
		convert_output =   convert_temp / 1000.0 ;	
	}
	else if(resol_bit == 0)
	{
		convert_temp =(u32)input_data	/ 10000 ;
		convert_output =   convert_temp / 1.0 ;	
	}
	else
	{
		convert_output= input_data ;
	}



	return 	convert_output;		
}


s32 PT12_FITER(u8 channel,u32 input, u8 filter)
{
	// -------------FILTERING------------------
	s32  siDelta;
	s32  siResult;
	u8  I;
  s32  siTemp;
	s32  slTemp;  
	static s32  adam_analog_filter[10];
	I = channel;
	siTemp = input;
 
 
 
	siDelta = siTemp - (signed long)adam_analog_filter[I] ;    //compare new reading and old reading
    if (( siDelta >= 1000 ) || ( siDelta <= -1000 ) ) // deg f
	//	adam_analog_filter[I] = adam_analog_filter[I] + (siDelta >> 2);//1 
		adam_analog_filter[I] =  siTemp; 

	// If the difference in new reading and old reading is greater than 5 degrees, implement rough filtering.
    else if (( siDelta >= 100 ) || ( siDelta <= -100 ) ) // deg f
	//	adam_analog_filter[I] = adam_analog_filter[I] + (siDelta >> 2);//1 
		adam_analog_filter[I] =  adam_analog_filter[I]+ (siDelta >> 4); 
 			
	// Otherwise, implement fine filtering.
	else
	{		      
	 
		slTemp = (signed long)filter*adam_analog_filter[I];
		slTemp += (signed long)siTemp * (100 - filter);
	 	adam_analog_filter[I] = (signed long)(slTemp/100);			 
	 
	}

	siResult = adam_analog_filter[I];
 	
 
	return siResult;
	
}
#endif
