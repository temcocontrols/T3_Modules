#include "controls.h"
#include "modbus.h"

#ifdef T3PT12
#include "t3-pt12.h"
#endif
#if (defined T36CTA)
#include "air_flow.h"
#include "math.h"
#include "accelero_meter.h"
extern uint16 CT_first_AD;
extern uint16 CT_multiple;
#define NOMINUS(n)     (((n) < 0) ? 0 : (n))
extern uint8_t t36ct_ver;
 extern u16 outdoorTempC;
 extern u16 outdoorTempH;
 extern u16 outdoorHum;
 extern u16 outdoorLux;
  extern u16 outdoorEnthalpy;
#endif

#ifdef INPUT_CONTROL

Str_table_point			 custom_tab[MAX_TBLS];
Str_in_point             inputs[MAX_INS];

uint8_t  input_type[MAX_INS];
//uint8_t  input_type1[MAX_INS];

#define SENSOR_DELAY 10
#define FILTER_ADJUST 4
#define NO_TABLE_RANGES 16
#define MIDDLE_RANGE     8

#if (defined T36CTA)
uint32_t calcRms(uint16_t* pData, int nNum)
{
	uint8_t i;
    uint32_t fSum = 0;
    for( i=0; i<nNum; ++i)
    {
        fSum += pData[i] * pData[i];
    }
 
    return sqrt(fSum/nNum);
}

#endif

#ifdef T3PT12
void control_input(void)
{
	Str_in_point *ins;
	uint8_t point = 0;
	int32_t sample;
//  uint8_t temp;
	ins = inputs;	
	while( point < MAX_INS )
	{	
		if( ins -> digital_analog == 1)  // analog
		{
			sample = (int32_t)(_temp_value[point].temp_C*1000);
			if( !ins->calibration_sign )
				sample += 100L * (ins->calibration_hi * 256 + ins->calibration_lo);
			else
				sample += -100L * (ins->calibration_hi * 256 + ins->calibration_lo);			
			ins->value = sample;
			
			if(ins->value< -40000) ins->decom = (ins->decom&0xf0)| 0x01 ;
			else if(ins->value>=150000) ins->decom = (ins->decom&0xf0)| 0x02 ;
			else ins->decom = ins->decom&0xf0 ;
		}
		else if( ins -> digital_analog == 0)  // digital
		{						
			sample = get_input_raw(point);	
			if( ins->range >= ON_OFF  && ins->range <= HIGH_LOW )  // control 0=OFF 1=ON
			{
				ins->control = (sample > 512 ) ? 1 : 0;
			}
			else
			{
				ins->control = (sample < 512 ) ? 0 : 1;					
			}
			if( ins->range >= custom_digital1 && ins->range <= custom_digital8 )
			{
				ins->control = (sample < 512 ) ? 0 : 1;	
			}
			//ins->value = ins->control ? 1000L : 0;
			sample = ins->control ? 1000L : 0;
		}			
		point++;
		ins++;
	}
}
#endif



#ifndef T3PT12

const long  tab_int[10] = { 11875, 21375, 10000, 18000, 10000, 18000,10000, 18000, 10000, 18000 };

const long  limit[10][2] = { { -40000L, 150000L }, { -40000L, 302000L },
							{ -40000L, 120000L }, { -40000L, 248000L },
							{ -40000L, 120000L }, { -40000L, 248000L },
							{ -40000L, 120000L }, { -40000L, 248000L },
							{ -50000L, 110000L }, { -58000L, 230000L }
						  };
//const U16_T code def_tab[5][17] = {
// /* 3k termistor YSI44005 -40 to 150 Deg.C or -40 to 302 Deg.F */
//	{ 233*4,  211*4, 179*4, 141*4, 103*4, 71*4, 48*4, 32*4,
//		21*4, 14*4, 10*4, 7*4, 5*4, 4*4, 3*4, 2*4, 1*4 },

// /* 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F */  // type2 
//	{ 976, 952, 916, 866, 812, 754, 700, 656,
//		620, 592, 572, 556, 546, 536, 530, 526, 522},

// /* 3k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F */
//	{ 233*4, 215*4, 190*4, 160*4, 127*4, 96*4, 70*4, 50*4,
//		35*4, 25*4, 18*4, 13*4, 9*4, 7*4, 5*4, 4*4, 3*4 },

// /* 10k termistor KM -40 to 120 Deg.C or -40 to 248 Deg.F */  // type3 
//	{ 985, 960, 932, 880, 818, 764, 704, 656, 
//	618, 592, 566, 552, 540, 532, 528, 524, 520},

// /* 3k termistor AK -40 to 150 Deg.C or -40 to 302 Deg.F */
//	{ 246*4, 238*4, 227*4, 211*4, 191*4, 167*4, 141*4, 115*4,
//		92*4, 72*4, 55*4, 42*4, 33*4, 25*4, 19*4, 15*4, 12*4 }
//};

const uint16_t  def_tab[5][17] = {
 /* 3k termistor YSI44005 -40 to 150 Deg.C or -40 to 302 Deg.F */
	{ 233*4,  211*4, 179*4, 141*4, 103*4, 71*4, 48*4, 32*4,
		21*4, 14*4, 10*4, 7*4, 5*4, 4*4, 3*4, 2*4, 1*4 },

 /* 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F */  // type2
	{ 988, 964, 924, 862, 778, 682, 572, 462,
	 364, 282, 214, 164, 128, 100, 76, 62, 48 },

 /* 3k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F */
	{ 233*4, 215*4, 190*4, 160*4, 127*4, 96*4, 70*4, 50*4,
		35*4, 25*4, 18*4, 13*4, 9*4, 7*4, 5*4, 4*4, 3*4 },

 /* 10k termistor KM -40 to 120 Deg.C or -40 to 248 Deg.F */ // type3
	{ 976, 948, 906, 842, 764, 670, 566, 466,
		376, 296, 234, 180, 144, 114, 90, 76, 60 },

 /* 3k termistor AK -40 to 150 Deg.C or -40 to 302 Deg.F */
	{ 246*4, 238*4, 227*4, 211*4, 191*4, 167*4, 141*4, 115*4,
		92*4, 72*4, 55*4, 42*4, 33*4, 25*4, 19*4, 15*4, 12*4 }
};


uint32_t swap_double( uint32_t dat ) 	 //swap_double
{ 
//	#ifdef ASIX_CON
//	U8_T temp1,temp2,temp3,temp4;
//	temp1 = (U8_T)dat;
//	temp2 = (U8_T)(dat >> 8);
//	temp3 = (U8_T)(dat >> 16);
//	temp4 = (U8_T)(dat >> 24);
//	return( temp4 | (U16_T)temp3 << 8 | (U32_T)temp2 << 16 |  (U32_T)temp1 << 24);
//	#endif
	

	
	return dat;

}


uint16_t swap_word( uint16_t dat ) 	//	  swap_word
{ 
	return dat;
}


uint32_t get_input_value_by_range( uint8_t range, uint16_t raw )
{
	int index;
	long val;
	int work_var;
	int ran_in;
	int delta = MIDDLE_RANGE;
	uint16_t *def_tbl;
	Byte end = 0;
	range--;
	ran_in = range;
	range >>= 1;
	def_tbl = ( uint16_t * )&def_tab[range];

	if( raw <= def_tbl[NO_TABLE_RANGES] )
		return limit[ran_in][1];
	if( raw >= def_tbl[0] )
		return limit[ran_in][0];
	index = MIDDLE_RANGE;


	while( !end )
	{
		if( ( raw >= def_tbl[index] ) && ( raw <= def_tbl[index-1] ) )
		{
			index--;
			delta = def_tbl[index] - def_tbl[index+1];
			if( delta )
			{
				work_var = (int)( ( def_tbl[index] - raw ) * 100 );
				work_var /= delta;
				work_var += ( index * 100 );
				val = tab_int[ran_in];
				val *= work_var;
				val /= 100;
				val += limit[ran_in][0];
			}
			return val;
		}
		else
		{
			if( !delta )
				end = 1;
			delta /= 2;
			if( raw < def_tbl[index] )
				index += delta;
			else
				index -= delta;
			if( index <= 0 )
				return limit[ran_in][0];
			if( index >= NO_TABLE_RANGES )
				return limit[ran_in][1];
		}
	}
	
	return 0;
}



S32_T test_match_custom( S16_T range, S16_T raw )
{   /* custom tables */

	Tbl_point *table_point;
	S16_T index = 1;
	S32_T val, diff;
	range -= table1;



	do
	{
		 table_point = &custom_tab[range].dat[index];
		 if( ( raw == swap_word(table_point->value) ) )
		 {
				return swap_double(table_point->unit);
		 }
		 if( ( raw < swap_word(table_point->value) ) &&
				( raw > swap_word((table_point-1)->value) ) )
			{ index--; break; }
		 else
			index++;
	}
	while( index <= 14 );

	table_point = &custom_tab[range].dat[index];
	index = swap_word((table_point+1)->value) - swap_word(table_point->value);
	if( index )
	{
/*		val = ( raw - table_point->value ) * 1000 /
			( (table_point+1)->value - table_point->value );*/
		val = ( raw - swap_word(table_point->value) );
		val *= 1000;
		val /= index;
	}
	diff = swap_double((table_point+1)->unit) - swap_double(table_point->unit);
	if( diff )
	{
/*		val = table_point->unit + val *
			( (table_point+1)->unit - table_point->unit );*/
		val *= diff;
		val /= 1000;
		val += swap_double(table_point->unit);
	}
	return val;
}


void control_input(void)
{
	Str_in_point *ins;
//	In_aux *inx;
	uint8_t point = 0;
	U32_T sample;
//	U8_T max_input;
  U8_T temp;	
	#if (defined T36CTA)
//	uint16 tempWord;
	uint8_t tempBuf[8];
	u8 i;
	static u8 lastTempRange= R10K_40_250DegF;
	static u16 lastTempCalibration = 0;
	static u16 lastHumCalibration= 0;
	static u8 lastTempCalibrationSign= 0;
	static u8 lastHumCalibrationSign= 0;
	
	tempBuf[0] = 0xff;
	tempBuf[1] = 0x06;
	tempBuf[2] = 0x00;
	tempBuf[3] = 0xff;
	tempBuf[4] = 0xff;
	tempBuf[5] = 0xff;
	tempBuf[6] = 0xff;
	tempBuf[7] = 0xff;
	#endif

	ins = inputs;
	while( point < MAX_INS )
	{		
		if(point < get_max_input())
		{
			input_type[point] = ins->decom >> 4;

#ifdef ASIX_CON			
			if(point < get_max_internal_input())
#endif
			{
//				ins->sub_id = 0;
//				ins->sub_product = 0;
//				ins->sub_number = 0;
//				if(input_type[point] != input_type1[point])
//				{	
//					input_type1[point] = input_type[point];

//					// set input type
//					
//					Set_Input_Type(point);					
//					
//				}
			}
			
			if(ins->auto_manual == 0)  // auto			 
			{ 				
				// raw value			
				if(ins->range != not_used_input)
				{				
					sample = get_input_raw(point);//input_raw[point];
					
					if( ins -> digital_analog == 0)  // digital
					{						
						temp = ins->decom;
						temp &= 0xf0;
						temp |= IN_NORMAL;
						ins->decom = temp;
						
						if( ins->range >= ON_OFF  && ins->range <= HIGH_LOW )  // control 0=OFF 1=ON
						{
							ins->control = (sample > 512 ) ? 1 : 0;
						}
						else
						{
							ins->control = (sample < 512 ) ? 0 : 1;					
						}
						if( ins->range >= custom_digital1 && ins->range <= custom_digital8 )
						{
							ins->control = (sample < 512 ) ? 0 : 1;	
						}
						//ins->value = ins->control ? 1000L : 0;
							
						sample = ins->control ? 1000L : 0;
					}
					else if(ins -> digital_analog == 1)	// analog
					{						
						
						temp = ins->decom;
						temp &= 0xf0;
						temp |= IN_NORMAL;
						ins->decom = temp;
						// add filter 
						//sample = Filter(point,sample);	
							
						switch(ins->range)
						{
						case Y3K_40_150DegC:
						case Y3K_40_300DegF:
						case R3K_40_150DegC:
						case R3K_40_300DegF:
						case R10K_40_120DegC:
						case R10K_40_250DegF:
						case KM10K_40_120DegC:
						case KM10K_40_250DegF:
						case A10K_50_110DegC:
						case A10K_60_200DegF:
							if(get_input_raw(point) > 1000)   
							{
								temp = ins->decom;
								temp &= 0xf0;
								temp |= IN_OPEN;
								ins->decom = temp;
							}
							else if(get_input_raw(point) < 20)  
							{ 
								temp = ins->decom;
								temp &= 0xf0;
								temp |= IN_SHORT;
								ins->decom = temp;
							}
							#if 0//(defined T36CTA)
//							if(t36ct_ver == T36CTA_REV1 )
//								{
//									if(point == 19)
//									{
//										if((ins->range%2)==1)
//										{
//											sample = outdoorTempC*100;
//										}
//										else if((ins->range%2)==0)
//										{
//											sample = outdoorTempH*100;
//										}
//									}
//									else
//										sample = get_input_value_by_range( ins->range, sample );
//									break;
//								}
//								else if(t36ct_ver == T36CTA_REV2)
//								{
//									if(point == 16)
//									{
//										if((ins->range%2)==1)
//										{
//											sample = outdoorTempC*100;
//										}
//										else if((ins->range%2)==0)
//										{
//											sample = outdoorTempH*100;
//										}
//									}
//									else
//										sample = get_input_value_by_range( ins->range, sample );
//									break;
//								}
							#else	
							sample = get_input_value_by_range( ins->range, sample );
							break;
							#endif
						case V0_5:
							sample = conver_by_unit_5v(sample);		
							break;
						case V0_10_IN:
						
							sample = conver_by_unit_10v(sample);

							break;
						case I0_100Amps:
							#if 0//(defined T36CTA)
						    if(sample<=(510/4))
							{
								if(sample>(CT_first_AD/4))
									tempWord =sample-(CT_first_AD/4);
								else
									tempWord = 0;
								CT_multiple = tempWord;
							//	printf("tempWord= %d \r\n\r\n", tempWord);
								//sample = ( 100000L * NOMINUS(sample-(CT_first_AD/4)) ) >> 10;
								sample = ( 100000L * tempWord ) >> 10;
							}
							else
								sample = ( 100000L * sample ) >> 10;
						    #else
							sample = ( 100000L * sample ) >> 10;
						    #endif
							break;
						case I0_20Amps:
							sample = (( 100000L * sample )/5) >> 10;
							break;
						case I0_50Amps:
							sample = ( (100000L * sample )/2) >> 10;
							break;
						case I0_75Amps:
							sample = (( 100000L * sample )*3/4) >> 10;
							break;
						case I0_20ma:
							#if 0//(defined T36CTA)
							sample = ((20000L * sample)*16/20+4000L)>>10;
							#else
							sample = ( 20000L * sample ) >> 10;
							#endif
							break;
						case I0_20psi:
							sample = ( 20000L * sample ) >> 10;
							break;
						case N0_3000FPM_0_10V:
							sample = ( 2700000L * sample ) >> 10;
							break;
						case P0_100_0_5V:
							sample = ( 100000L * sample ) >> 10;
							break;
						case P0_100_4_20ma:
//							sample = 100000L * ( sample - 255 ) / 768;
							if(sample < 204) 
									sample = 0;
								else
									sample = 100000L * ( sample - 204 ) / 816;
							break;
						case table1:
						case table2:
						case table3:
						case table4:
						case table5:
								sample = conver_by_unit_custable(point,sample) / 100;
								sample = test_match_custom((int)ins->range, (int)sample);	
														//Test[5] = sample;

								sample = 1000l * sample;
								break;
						case N0_2_32counts:
						case HI_spd_count:	
	//						Test[11] = high_spd_counter[point];
	//						Test[12] = high_spd_counter_tempbuf[point];
							sample =  get_high_spd_counter(point)*1000;
						
							break;
						#if (defined T36CTA)
						case Humidty:
							if(t36ct_ver == T36CTA_REV1 )
							{
								if(point == 20)
								{
									sample = outdoorHum * 100;
								}
							}
							else if(t36ct_ver == T36CTA_REV2)
							{
								if(point == 17)
								{
									sample = outdoorHum * 100;
								}
							}
							break;
						case pressureInWc:
							if(t36ct_ver == T36CTA_REV2)
								sample = Pressure.org_val*1000l;
							break;
						case Reserved1:
							if(t36ct_ver == T36CTA_REV1 )
							{
								if(point == 21)
								{
									sample = outdoorLux*1000;
								}
								if(point == 22)
								{
									sample = outdoorEnthalpy*100;
								}
								else
									sample = Pressure.air_speed*1000l;
							}
							else if(t36ct_ver == T36CTA_REV2)
							{
								if(point == 18)
								{
									sample = outdoorLux*1000;
								}
								else if(point == 19)
								{
									sample = outdoorEnthalpy*100;
								}
								else
									sample = Pressure.air_speed*1000l;
									
							}
							break;
						case Reserved2:
							if(t36ct_ver == T36CTA_REV2)
								sample = ((uint32_t)((Pressure.air_flow.C_Type[3]&0x00ff)<<24)+(uint32_t)((Pressure.air_flow.C_Type[2]&0x00ff)<<16)
										+(uint32_t)((Pressure.air_flow.C_Type[1]&0x00ff)<<8)+(uint32_t)(Pressure.air_flow.C_Type[0]&0x00ff))*1000l;
							break;
						case Reserved3:
							if(t36ct_ver == T36CTA_REV2)
								sample = calcRms(axis_value, 3)*1000l;
							break;
						#endif
						default:
							//	sample = sample * 1000;	
							break;
						}
				
					//	if( ins->calibration_increment ) 
						{
							if( !ins->calibration_sign )
								sample += 100L * (ins->calibration_hi * 256 + ins->calibration_lo);
							else
								sample += -100L * (ins->calibration_hi * 256 + ins->calibration_lo);
						}
						
						if(t36ct_ver == T36CTA_REV1 )
						{
							if((point == 19) && (ins->range <= A10K_60_200DegF) && (ins->range>not_used_input))
							{
								if((ins->range%2)==1)
								{
									tempBuf[2] = 0x00;
									tempBuf[3] = 0x79;
									tempBuf[4] = 0x00;
									tempBuf[5] = 0x00;
									sample = outdoorTempC*100;
								}
								else if((ins->range%2)==0)
								{
									tempBuf[2] = 0x00;
									tempBuf[3] = 0x79;
									tempBuf[4] = 0x00;
									tempBuf[5] = 0x01;
									sample = outdoorTempH*100;
								}
							}
						}
						else if(t36ct_ver == T36CTA_REV2)
						{
							if(point == 16)
							{
								if((ins->range%2)==1)
								{
									tempBuf[2] = 0x00;
									tempBuf[3] = 0x79;
									tempBuf[4] = 0x00;
									tempBuf[5] = 0x00;
									sample = outdoorTempC*100;
								}
								else if((ins->range%2)==0)
								{
									tempBuf[2] = 0x00;
									tempBuf[3] = 0x79;
									tempBuf[4] = 0x00;
									tempBuf[5] = 0x01;
									sample = outdoorTempH*100;
								}
							}
						}
						if(t36ct_ver == T36CTA_REV1 )
						{
							if(point == 19)
							{
								if((lastTempRange != ins->range)&& (ins->range <= A10K_60_200DegF) && (ins->range>not_used_input))
								{
									init_crc16();
									for( i=0; i< 6; i++)
									{
										crc16_byte(tempBuf[i]);
									}
									tempBuf[6] = CRChi;
									tempBuf[7] = CRClo;
									memcpy(uart_send, tempBuf, 8);
									TXEN = SEND;
									USART_SendDataString(8);
									lastTempRange = ins->range;
									delay_ms(5);
								}
								if((lastTempCalibration != (ins->calibration_hi * 256 + ins->calibration_lo))||( lastTempCalibrationSign!= ins->calibration_sign))
								{
									if( (ins->range ==Y3K_40_300DegF)||(ins->range ==R10K_40_250DegF) || (ins->range ==R3K_40_300DegF) 
										|| (ins->range ==KM10K_40_250DegF) || (ins->range ==A10K_60_200DegF))
									{
										tempBuf[2] = 0x00;
										tempBuf[3] = 0x64;
										tempBuf[4] = (u8)(((sample/100)>>8)&0xff);
										tempBuf[5] = (u8)((sample/100)&0xff);
										init_crc16();
										for( i=0; i< 6; i++)
										{
											crc16_byte(tempBuf[i]);
										}
										tempBuf[6] = CRChi;
										tempBuf[7] = CRClo;
										memcpy(uart_send, tempBuf, 8);
										TXEN = SEND;
										USART_SendDataString(8);
									}
									if( (ins->range ==Y3K_40_150DegC)||(ins->range ==R10K_40_120DegC) || (ins->range ==R3K_40_150DegC) 
										|| (ins->range ==KM10K_40_120DegC) || (ins->range ==A10K_50_110DegC))
									{
										tempBuf[2] = 0x00;
										tempBuf[3] = 0x65;
										tempBuf[4] = (u8)(((sample/100)>>8)&0xff);
										tempBuf[5] = (u8)((sample/100)&0xff);
										init_crc16();
										for( i=0; i< 6; i++)
										{
											crc16_byte(tempBuf[i]);
										}
										tempBuf[6] = CRChi;
										tempBuf[7] = CRClo;
										memcpy(uart_send, tempBuf, 8);
										TXEN = SEND;
										USART_SendDataString(8);
									}
									delay_ms(5);
								}
								lastTempCalibrationSign = ins->calibration_sign;
								lastTempCalibration = (ins->calibration_hi * 256 + ins->calibration_lo);
							}
							
						}
						else if(t36ct_ver == T36CTA_REV2)
						{
							if(point == 16)
							{
								if((lastTempRange != ins->range)&& (ins->range <= A10K_60_200DegF) && (ins->range>not_used_input))
								{
									init_crc16();
									for( i=0; i< 6; i++)
									{
										crc16_byte(tempBuf[i]);
									}
									tempBuf[6] = CRChi;
									tempBuf[7] = CRClo;
									memcpy(uart_send, tempBuf, 8);
									TXEN = SEND;
									USART_SendDataString(8);
									lastTempRange = ins->range;
									delay_ms(5);
								}
								if((lastTempCalibration != (ins->calibration_hi * 256 + ins->calibration_lo))||( lastTempCalibrationSign!= ins->calibration_sign))
								{
									if( (ins->range ==Y3K_40_300DegF)||(ins->range ==R10K_40_250DegF) || (ins->range ==R3K_40_300DegF) 
										|| (ins->range ==KM10K_40_250DegF) || (ins->range ==A10K_60_200DegF))
									{
										tempBuf[2] = 0x00;
										tempBuf[3] = 0x64;
										tempBuf[4] = (u8)(((sample/100)>>8)&0xff);
										tempBuf[5] = (u8)((sample/100)&0xff);
										init_crc16();
										for( i=0; i< 6; i++)
										{
											crc16_byte(tempBuf[i]);
										}
										tempBuf[6] = CRChi;
										tempBuf[7] = CRClo;
										memcpy(uart_send, tempBuf, 8);
										TXEN = SEND;
										USART_SendDataString(8);
									}
									if( (ins->range ==Y3K_40_150DegC)||(ins->range ==R10K_40_120DegC) || (ins->range ==R3K_40_150DegC) 
										|| (ins->range ==KM10K_40_120DegC) || (ins->range ==A10K_50_110DegC))
									{
										tempBuf[2] = 0x00;
										tempBuf[3] = 0x65;
										tempBuf[4] = (u8)(((sample/100)>>8)&0xff);
										tempBuf[5] = (u8)((sample/100)&0xff);
										init_crc16();
										for( i=0; i< 6; i++)
										{
											crc16_byte(tempBuf[i]);
										}
										tempBuf[6] = CRChi;
										tempBuf[7] = CRClo;
										memcpy(uart_send, tempBuf, 8);
										TXEN = SEND;
										USART_SendDataString(8);
									}
									
									lastTempCalibrationSign = ins->calibration_sign;
									lastTempCalibration = (ins->calibration_hi * 256 + ins->calibration_lo);
									delay_ms(5);
								}
								
							}
							if((point == 17)&& (ins->range == Humidty))
							{
								if((lastHumCalibration != (ins->calibration_hi * 256 + ins->calibration_lo))||(lastHumCalibrationSign != ins->calibration_sign))
								{
									//if( ins->range == Humidty)
									{
										tempBuf[2] = 0x01;
										tempBuf[3] = 0x30;
										tempBuf[4] = (u8)(((sample/100)>>8)&0xff);
										tempBuf[5] = (u8)((sample/100)&0xff);
										init_crc16();
										for( i=0; i< 6; i++)
										{
											crc16_byte(tempBuf[i]);
										}
										tempBuf[6] = CRChi;
										tempBuf[7] = CRClo;
										memcpy(uart_send, tempBuf, 8);
										TXEN = SEND;
										USART_SendDataString(8);
										delay_ms(5);
									}
									
									lastHumCalibration = (ins->calibration_hi * 256 + ins->calibration_lo);
									lastHumCalibrationSign = ins->calibration_sign;
								}
							}
						}
					}
					ins->value = swap_double(sample);
				
				}

				else  // not_used_input
				{
					// if range is 0, show raw value
						temp = ins->decom;
						temp &= 0xf0;
						temp |= IN_NORMAL;
						ins->decom = temp;
//						ins->value = swap_double((U32_T)get_input_raw(point)/*input_raw[point]*/ * 1000);
						ins->value = (U32_T)get_input_raw(point)*3000/1023;
				}	
				
			}
		
	  } 
		else
		{
//			ins->sub_id = 0;
//			ins->sub_product = 0;
//			ins->sub_number = 0;
			
			temp = ins->decom;
			temp &= 0xf0;
			temp |= IN_NORMAL;
			ins->decom = temp;
			
		}
		point++;
	  ins++;
		
	}
}
#endif
#endif




