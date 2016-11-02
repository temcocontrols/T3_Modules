

#include <KEIL_VRS1000_regs.h>
#include "LibIO_T3IO.h"
#include "define.h"
#include "math.h"

#define TYPE2_10K_THERM	  0
#define TYPE3_10K_THERM	  1 
#define TYPE4_10K_THERM	  2 
/*range define*/
#define RAW_DATA	0
#define C_TYPE2		1
#define F_TYPE2		2
#define PERCENT	  	3
#define ON_OFF		4
#define OFF_ON		5
#define PULSE		6
#define LIGHTING	7
#define C_TYPE3		8
#define F_TYPE3		9
#define NO_USE		10
#define V0_5		11
#define V0_10		12
#define I0_20ma		13
#define	I0_100ma	14
#define LM235_C	 	15
#define LM235_F		16
#define C_TYPE4		17
#define F_TYPE4		18
	


bit pic_exists = 1;
bit thermistor_type = 0 ;

#if defined T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS TO ONE
void start_timer( et_event timer_no , unsigned int timeout ) ;
extern unsigned char xdata gucPreviousInput[8];
extern unsigned char xdata gucTimer[8];
extern unsigned char xdata gucZone[13];
extern unsigned char xdata gucTimerLeft[8];
extern unsigned char xdata  gucOverOutput[8];
extern unsigned int  data guiManual;

extern bit flash_read_int(unsigned char id, unsigned int *value, unsigned char block_select);
extern void stop_timer( et_event timer_no );
unsigned char xdata gucTimerFilter[8];
extern unsigned long xdata previous_pulse_number[8];

typedef union   pulse_number_link {
		unsigned long      number_long[8];
		unsigned int	 half_number[16];
		unsigned char    pulse_number[32];
	};
extern union   pulse_number_link xdata   pic,flash; 

#endif
signed int idata guiTempBuffer[8];
extern unsigned char xdata  reading_filter_bypass;
unsigned char const code def_tab2[11] =
			{					 // 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F 
			 192, 209, 206, 187, 161, 131, 103, 79, 61, 45, 155
			};
//MHF:12-30-05,Added 4 values to make the tstat can measure minus degree
unsigned char const code def_tab_pic_Type2_10K[21] =
			{					 // 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F 
//			 25, 41, 61, 83, 102, 113, 112, 101, 85, 67, 51, 38, 28, 21, 65 //MHF 20010_07 REVISE TEMP LOOKUP TABLE PER NATHANEAL
			
			 // 10k termistor GREYSTONE -50 to 150 Deg.C 
			15,25,41,61,83,102,113,112,101,85,67,51,38,28,21,15,11,8,6,5,19	   //changed by ye

 			};
unsigned char const code def_tab_pic_Type3_10K[21] =
			{					 // 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F 
			 18,29, 45, 63, 81, 96, 104, 103, 94, 80, 65, 51, 40, 30, 23, 17,13,10,8,6,24 //MHF 20010_07 REVISE TEMP LOOKUP TABLE PER NATHANEAL
 			};
unsigned char const code def_tab_pic_Type4_10K[15] =
			{					 // 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F 
			 32, 46, 63, 81, 92, 98, 95, 87, 76, 63, 52, 40, 32, 24, 93 
 			};
//combine two look_up_table function to one.  add the thermistor type flag to deicde which table to look up
signed int   look_up_table1(unsigned int count)
{
	int   xdata val;
    char  index=20;			//chagned by  ye 
	int   xdata work_var;
 
	if (pic_exists)
	{	
		if(thermistor_type == TYPE2_10K_THERM)
			work_var= def_tab_pic_Type2_10K[index];
		else if(thermistor_type == TYPE3_10K_THERM)
			work_var= def_tab_pic_Type3_10K[index];
		else if(thermistor_type == TYPE4_10K_THERM)
			work_var= def_tab_pic_Type4_10K[index];
	}
	else
		work_var= def_tab2[index];
		  
	if (work_var > count )
		{
			val =  (index-5)  * 100 ;	 //the highest temperature is 150c, changed by ye
			return ( val );
		  
		}

	do 
		{
			index--;

			if (pic_exists)
			{
				if(thermistor_type == TYPE2_10K_THERM)
				work_var += def_tab_pic_Type2_10K[index];
				else if(thermistor_type == TYPE3_10K_THERM)
				work_var += def_tab_pic_Type3_10K[index];
				else if(thermistor_type == TYPE4_10K_THERM)
				work_var += def_tab_pic_Type4_10K[index];
			}
			else
				work_var += def_tab2[index];

			if( work_var > count)
				{
				val = ( work_var - count )*100;

				if (pic_exists)
				{
					if(thermistor_type == TYPE2_10K_THERM)
					val /= def_tab_pic_Type2_10K[index];
					else if(thermistor_type == TYPE3_10K_THERM)
					val /= def_tab_pic_Type3_10K[index];
					else if(thermistor_type == TYPE4_10K_THERM)
					val /= def_tab_pic_Type4_10K[index];

				}
				else
					val /= def_tab2[index];
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

/******************************************RangeConverter******************************************/
/*
Description: Convert the  raw data from adc to correspond engineer units.
parameter:	finction,	The engineer units want to get,
			para,		Raw data from ADC
			i, 			Be used for function = 4,customer sensor,because there are only two 
						customer tables,so should check this parameter not bigger than 2 on fun4.
			cal,		calibration data for the correspond input channel
Return:		Changed input to the expected engineer units.	
			
*/
/*********************************RangeConverter funtion start**************************************/
signed int RangeConverter(unsigned char function, signed int para,signed int cal,unsigned char i)
{
	signed int xdata siAdcResult;
	unsigned char xdata ucFunction;
	signed   int  xdata siInput;
	signed int  xdata uiCal;
	signed   int  xdata siResult;
	unsigned int xdata uiResult;
	const float code  K_A = 4.8876 ;
	const signed int code K_B = 2732 ;
	bit bAnalogInputStatus;
	ucFunction = function;
	siInput = para;
	uiCal = cal;
	switch(ucFunction)
	{
	 case RAW_DATA :
		  siResult = siInput + (signed int)(uiCal - CALIBRATION_OFFSET);
		  break ;		 
	//-----------10K Thermistor---------------
	case  C_TYPE2 :
	case  F_TYPE2 :
	case  C_TYPE3 : 
	case  F_TYPE3 :
	case  C_TYPE4 : 
	case  F_TYPE4 :
		if((ucFunction == C_TYPE2)||(ucFunction == F_TYPE2))
		thermistor_type = TYPE2_10K_THERM ;
		else if((ucFunction == C_TYPE3)||(ucFunction == F_TYPE3))
		thermistor_type = TYPE3_10K_THERM ;
		//else
		//thermistor_type = TYPE4_10K_THERM ;

       	siAdcResult = look_up_table1(siInput);
		if(siAdcResult & 0x8000)
		siInput = -(signed int)(siAdcResult & 0x7fff);
		else
		siInput = siAdcResult;
		//analog_input[i] = adc_result;
		
		if((ucFunction == F_TYPE2)||(ucFunction == F_TYPE3)||(ucFunction == F_TYPE4))  //F
		{		 
			siInput = (siInput * 9)/5 +320; 		 
		}
		// Add the calibration term to the input.
		siResult = siInput + (signed int)(uiCal - CALIBRATION_OFFSET); 
	break;	 	
	//-----------0-100%---------------
	case PERCENT: 					//MHF: Feb 24th 2005 new range setting for analog inputs
		siResult = (float)(siInput)/1023*100;
		break;

	//-----------ON/OFF---------------
	case ON_OFF : 
	case OFF_ON	:
		siAdcResult = (float)(siInput)/1023*50;
		if(siAdcResult <= 24)
		{
			if(ucFunction == OFF_ON)
				bAnalogInputStatus = 1; 
			else if(ucFunction == ON_OFF)
				bAnalogInputStatus = 0; 
		}
		else if(siAdcResult >= 26)
		{
			if(ucFunction == OFF_ON)
			bAnalogInputStatus = 0; 
			else if(ucFunction == ON_OFF)
			bAnalogInputStatus = 1; 
		}
		siResult = (unsigned int)(bAnalogInputStatus);
	 	break ;
#ifdef T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS
 	case LIGHTING :
		switch(i)
		{
			case 0:		 
			if(previous_pulse_number[i] != pic.number_long[i])
			{
				gucTimerFilter[i]++;
				if(gucTimerFilter[i] > 1)
				{
					gucTimerFilter[i] = 0;	
					previous_pulse_number[i] = pic.number_long[i];

					if(!flash_read_int(FLASH_INPUT1_TIMER,&uiResult,FLASH_MEMORY))
						gucTimer[i] = 0;
					else
						gucTimer[i] = uiResult;				
					
		                                                                                                                                                                			
					if(gucTimer[i] > 0)
					{
				
						gucOverOutput[i] = 1;
						gucTimerLeft[i] = gucTimer[i];
					 
						start_timer(INPUT1_TIMER,DEFAULT_TIMEOUT);
					
					}	
				}
				
			}
		 	else 
				gucTimerFilter[i] = 0;
			break;

			case 1:
			if(previous_pulse_number[i] != pic.number_long[i])
			{
				gucTimerFilter[i]++;
				if(gucTimerFilter[i] > 2)
				{
						gucTimerFilter[i] = 0;	
					previous_pulse_number[i] = pic.number_long[i];

					if(!flash_read_int(FLASH_INPUT2_TIMER,&uiResult,FLASH_MEMORY))
						gucTimer[i] = 0;
					else
						gucTimer[i] = uiResult;				
					
		                                                                                                                                                                			
					if(gucTimer[i] > 0)
					{
				
						gucOverOutput[i] = 1;
						gucTimerLeft[i] = gucTimer[i];
					 
						start_timer(INPUT2_TIMER,DEFAULT_TIMEOUT);
					
					}
				}	
				
			}	
			else 
				gucTimerFilter[i] = 0;
			break;
			case 2:
				if(previous_pulse_number[i] != pic.number_long[i])
				{
					gucTimerFilter[i]++;
					if(gucTimerFilter[i] > 2)
					{
							gucTimerFilter[i] = 0;	
						previous_pulse_number[i] = pic.number_long[i];
	
						if(!flash_read_int(FLASH_INPUT3_TIMER,&uiResult,FLASH_MEMORY))
							gucTimer[i] = 0;
						else
							gucTimer[i] = uiResult;				
						
			                                                                                                                                                                			
						if(gucTimer[i] > 0)
						{
					
							gucOverOutput[i] = 1;
							gucTimerLeft[i] = gucTimer[i];
						 
							start_timer(INPUT3_TIMER,DEFAULT_TIMEOUT);
						
						}
					}	
				
				}
	 			else 
				gucTimerFilter[i] = 0;
			break;
			case 3:
				if(previous_pulse_number[i] != pic.number_long[i])
				{
					gucTimerFilter[i]++;
					if(gucTimerFilter[i] > 2)
					{
						gucTimerFilter[i] = 0;	
						previous_pulse_number[i] = pic.number_long[i];
	
						if(!flash_read_int(FLASH_INPUT4_TIMER,&uiResult,FLASH_MEMORY))
							gucTimer[i] = 0;
						else
							gucTimer[i] = uiResult;				
						
			                                                                                                                                                                			
						if(gucTimer[i] > 0)
						{
					
							gucOverOutput[i] = 1;
							gucTimerLeft[i] = gucTimer[i];
						 
							start_timer(INPUT4_TIMER,DEFAULT_TIMEOUT);
						
						}	
					}
				
			}
	 		else 
				gucTimerFilter[i] = 0;
			break;
			case 4:
			if(previous_pulse_number[i] != pic.number_long[i])
			{
				gucTimerFilter[i]++;
				if(gucTimerFilter[i] > 2)
				{
					gucTimerFilter[i] = 0;	
					previous_pulse_number[i] = pic.number_long[i];

					if(!flash_read_int(FLASH_INPUT5_TIMER,&uiResult,FLASH_MEMORY))
						gucTimer[i] = 0;
					else
						gucTimer[i] = uiResult;				
					
		                                                                                                                                                                			
					if(gucTimer[i] > 0)
					{
				
						gucOverOutput[i] = 1;
						gucTimerLeft[i] = gucTimer[i];
					 
						start_timer(INPUT5_TIMER,DEFAULT_TIMEOUT);
					
					}	
				}
			}
	 		else 
				gucTimerFilter[i] = 0;
			break;
			case 5:
				if(previous_pulse_number[i] != pic.number_long[i])
				{
				gucTimerFilter[i]++;
				if(gucTimerFilter[i] > 2)
				{
					gucTimerFilter[i] = 0;	
					previous_pulse_number[i] = pic.number_long[i];

					if(!flash_read_int(FLASH_INPUT6_TIMER,&uiResult,FLASH_MEMORY))
						gucTimer[i] = 0;
					else
						gucTimer[i] = uiResult;				
					
		                                                                                                                                                                			
					if(gucTimer[i] > 0)
					{
				
						gucOverOutput[i] = 1;
						gucTimerLeft[i] = gucTimer[i];
					 
						start_timer(INPUT6_TIMER,DEFAULT_TIMEOUT);
					
					}	
				}
			}
				 	else 
				gucTimerFilter[i] = 0;
			break;
			case 6:
				if(previous_pulse_number[i] != pic.number_long[i])
				{
						
					gucTimerFilter[i]++;
					if(gucTimerFilter[i] > 2)
					{
						gucTimerFilter[i] = 0;
					previous_pulse_number[i] = pic.number_long[i];

					if(!flash_read_int(FLASH_INPUT7_TIMER,&uiResult,FLASH_MEMORY))
						gucTimer[i] = 0;
					else
						gucTimer[i] = uiResult;				
					
		                                                                                                                                                                			
					if(gucTimer[i] > 0)
					{
				
						gucOverOutput[i] = 1;
						gucTimerLeft[i] = gucTimer[i];
				 
						start_timer(INPUT7_TIMER,DEFAULT_TIMEOUT);
					
					}	
				}
			}
				 	else 
				gucTimerFilter[i] = 0;
			break;
			case 7:
			if(previous_pulse_number[i] != pic.number_long[i])
			{
				gucTimerFilter[i]++;
				if(gucTimerFilter[i] > 2)
				{
					gucTimerFilter[i] = 0;	
	
					previous_pulse_number[i] = pic.number_long[i];

					if(!flash_read_int(FLASH_INPUT8_TIMER,&uiResult,FLASH_MEMORY))
						gucTimer[i] = 0;
					else
						gucTimer[i] = uiResult;				
					
		                                                                                                                                                                			
					if(gucTimer[i] > 0)
					{

						gucOverOutput[i] = 1;
						gucTimerLeft[i] = gucTimer[i];
					 
						start_timer(INPUT8_TIMER,DEFAULT_TIMEOUT);
					
					}	
				}
			}
	 		else 
				gucTimerFilter[i] = 0;
			break;

			default:
			break;
		}
		siResult = siInput;
		break ;
#endif
		case NO_USE: 
		siResult = 0;
		break;
		case V0_5:
		siResult =  (5000L * siInput ) >> 10;
		break;
		case V0_10:
		siResult = ( 10000L * siInput ) >> 10;
		break;
		case I0_20ma:
		siResult = ( 2000L * siInput ) >> 10;
		break;
		case I0_100ma:
		siResult = ( 100000L * siInput ) >> 10;
		break;
		case LM235_C:
		case LM235_F:
		siInput = (signed int )(K_A *siInput - K_B) ;
		if((ucFunction == LM235_F))
		siInput = (siInput * 9)/5 +320;
		siResult =  siInput ;
		break;
		}
 	return siResult;
}