#include "air_flow.h"
#include "registerlist.h"
#include "inputs.h" 
#include "24cxx.h"
#include "delay.h"
#include "math.h"
#ifdef AIR_FLOW_SENSOR
_STR_PRESSURE_  Pressure;
uint8   Run_Timer; 
uint8 const   Prs_Unit[][6] = 
{
	"inWC ",
	"KPa  ",
	"Psi  ",
	"mmHg ",
	"inHg ",
	"Kgcm ",
	"atmos",
	"bar  ",
};

// *************************************************************************************
// 	input: calibrate points
//	output: none 
//  read the calibrate point from eep,
// ************************************************************************************* 
static void test_point_read( ) 
{ 
	u8 i;
	for(i=0;i < 10;i++)
	{
		Pressure.cal_pr[i]=  AT24CXX_ReadOneByte(EEP_CAL_PR0+i*4) +  ((unsigned int)AT24CXX_ReadOneByte(EEP_CAL_PR0+i*4+1)<<8);   	
		delay_ms(10);
		Pressure.cal_ad[i] = AT24CXX_ReadOneByte(EEP_CAL_AD0+i*4 ) + ((unsigned int)AT24CXX_ReadOneByte(EEP_CAL_AD0+i*4+1)<<8);
		delay_ms(10);
		Pressure.user_cal_pr[i]=  AT24CXX_ReadOneByte(EEP_USER_CAL_PR0+i*4) +  ((unsigned int)AT24CXX_ReadOneByte(EEP_USER_CAL_PR0+i*4+1)<<8);   	
		delay_ms(10);
		Pressure.user_cal_ad[i] = AT24CXX_ReadOneByte(EEP_USER_CAL_AD0+i*4 ) + ((unsigned int)AT24CXX_ReadOneByte(EEP_USER_CAL_AD0+i*4+1)<<8);
		delay_ms(10);
	} 
}
// *************************************************************************************
// 	input:  
//	output: caliobrate points number 
//  get the calibrate point number
// ************************************************************************************* 
static u8 get_cal_point(u16 *p) 
{
	unsigned char  i;  
	
	for(i=0;i < 10;i++)
	{
		if(p[i] == 0xffff)
			break;	 
	}
	
	return i; 
}
// *************************************************************************************
// --- most ten point calibration ---
//least squares fit ---最小二乘法
// 	input: none
//	output: none 
// ************************************************************************************* 
//static void min2method(float *K, float *B, unsigned char counter,u16 *x_val,u16 *y_val)
//{
//	   float xdata sumx, sumx2, sumy, sumxy;
//	   unsigned char  i;
//	   
//	   sumx = 0;
//	   sumx2 = 0;
//	   for(i = 0; i < counter; i++)
//	   {
//		  sumx += x_val[i];
//		  sumx2 += (float)x_val[i] * x_val[i];
//	   }

//	   sumy = 0;
//	   for(i = 0; i < counter; i++)
//	   {
//		  sumy += y_val[i];
//	   }

//	   sumxy = 0;
//	   for(i = 0; i < counter; i++)
//	   {
//		  sumxy += (float)x_val[i] * y_val[i];
//	   } 
//	   *K = ((counter*sumxy - sumx*sumy) / (counter*sumx2 - sumx*sumx)); 
//	   *B = ((sumx2*sumy - sumx*sumxy) / (counter*sumx2 - sumx*sumx));
//} 

// *************************************************************************************
// 	input: ad value
//	output: pressure value 
//  get the pressure value from ad vaue,
// ************************************************************************************* 

int16 line_count(int16 para_input,int8 i,u16 *x_val,u16 *y_val)
{
   int16 para_temp;
   para_temp = y_val[i-1] + (int32)(para_input - x_val[i-1])* (y_val[i]  - y_val[i-1] ) / (x_val[i]- x_val[i-1] );        
      
   return para_temp;
}

static s16 get_pressure(u16 input_ad)
{
	s16 result_pre; 
	if((Pressure.table_sel == USER_TABLE)&&(Pressure.user_cal_point > 1))
	{
		if(input_ad > Pressure.user_cal_ad[Pressure.user_cal_point - 1])
		{
			result_pre = line_count(input_ad,Pressure.user_cal_point -1,Pressure.user_cal_ad,Pressure.user_cal_pr);
		}
		else
		{ 
			int8 i;
			for(i = 1; i < Pressure.user_cal_point; i++)
				if(input_ad < Pressure.user_cal_ad[i]) break;
			result_pre = line_count(input_ad,i,Pressure.user_cal_ad,Pressure.user_cal_pr);
		} 
		result_pre +=Pressure.user_val_offset;
	}
	else
	{
		if(Pressure.cal_point < 2) return 0;
		else if(input_ad < Pressure.cal_ad[0]) return 0;
		else if(input_ad > Pressure.cal_ad[Pressure.cal_point - 1])
		{
			result_pre = line_count(input_ad,Pressure.cal_point -1,Pressure.cal_ad,Pressure.cal_pr);
		}
		else
		{ 
			int8 i;
			for(i = 1; i < Pressure.cal_point; i++)
				if(input_ad < Pressure.cal_ad[i]) break;
			result_pre = line_count(input_ad,i,Pressure.cal_ad,Pressure.cal_pr);
		}
		result_pre +=Pressure.org_val_offset;
		if((Pressure.table_sel == USER_TABLE)&&(Pressure.user_cal_point == 1))
			result_pre +=Pressure.user_val_offset;
	} 
	if(result_pre < 0) result_pre = 0;
	return result_pre;
}

// *************************************************************************************
// 	input: input the unit type
//	output: get the  ratio
//  get the ratio on the mmhg and the ratio by 10^7.
// ************************************************************************************* 

//static float get_mmhg_ratio(unsigned char unit)  //rate = unit/mmhg*10^7
//{
// 
//	float Ftemp;  //10^7
//	
//	switch (unit)
//	{													   
//	   case inWC:         //W.C				0.01		 0	
//				Ftemp = 5353000;
//		 break;
//	   case KPa1:          //KPa 			0.0001		 0	
//				Ftemp = 1332800;
//		 break;  
//	   case Psi:          //Psi				0.00001		0.1
//				Ftemp = 193400;				
//		 break; 
//	   case mmHg:         //mmHg			0.01		0
//				Ftemp = 10000000;			
//		 break;
//	   case inHg:         //inches of Hg	0.00001		0.01
//				Ftemp = 393700;				
//		 break;
//	   case Kg_cm2:       //Kg/cm2			0.000001	0.001
//				Ftemp = 13595;
//		 break;  
//	   case atmosphere:   //atmosphere		0.000001	0.001
//				Ftemp = 13158;
//		 break;
//	   case bar:          //bar				0.000001	0.001
//				Ftemp = 13328;				
//	   break; 
// 
//	}
//    return Ftemp;
//}



// *************************************************************************************
// 	input: new unit   default unit
//	output: get the  ratio between the two unit 
// ************************************************************************************* 

//static float get_units_ratio(unsigned char new_unit,unsigned char default_unit)
//{  
//	  float Ftemp; 
//	  Ftemp = get_mmhg_ratio(new_unit)/get_mmhg_ratio(default_unit);
//	  return Ftemp;
//} 


 
// *************************************************************************************
// 	input: pressure value, data base and data index unit
//	output: none  
// ************************************************************************************* 
u8 const code  decimal_num[2][8] = {
	{2,4,5,2,5,6,6,6},		//default unit = inWC
	{0,0,1,0,2,3,3,3}		//default unit = Psi
};

float Sys_Filter(float new_value,float pre_value,unsigned char input_filter)
{         
	float temp; 
	temp = pre_value * input_filter;
	pre_value = (temp + new_value) / (input_filter + 1);            
	return pre_value;
} 
 
 
//static void datatype_convert(float input, s32 *data_base, u16 *data_index,u8 data_unit,u8 data_default_unit)
//{
//	u8 i; 
//	u8 DECIMAL_NUM = 0; 
//	if(data_default_unit == inWC)
//		DECIMAL_NUM = decimal_num[0][data_unit];
//	else if(data_default_unit == Psi) 
//		DECIMAL_NUM = decimal_num[1][data_unit];
//	else
//		DECIMAL_NUM = 0;
//	 
//	for(i = 0;i<DECIMAL_NUM;i++)   
//		input *= 10;
//	 
//	
//	if(data_default_unit == inWC)		//when the default unit is inwc, it has two decimals
//		*data_base =  input/100;
//	else if(data_default_unit == Psi)   //when the default unit is psi, it has one decimals
//		*data_base =  input/10;
//	
//	
//	
//	if(*data_base == 0) *data_index = 0;
//	else
//		*data_index = 0 - DECIMAL_NUM; 
//	
//}
#define BUFFER_MAX		10
static u16 Read_Buffer[BUFFER_MAX+1];

static u16 get_ad_val(u16 read_ad)
{
    u8 i;
	u16 min,max,read_temp;
    s32 datacore;
	static uint8 pos = 0;
	if(pos < BUFFER_MAX)
	{
		Read_Buffer[pos++] = read_ad;
		read_temp = read_ad;
		
	}
	else
	{
		Read_Buffer[BUFFER_MAX] = read_ad;
		datacore = 0;
		min = Read_Buffer[0];
		max = Read_Buffer[0];
		for(i = 0; i < BUFFER_MAX; i++)
		{ 
			datacore += Read_Buffer[i]; 
			min = min < Read_Buffer[i] ? min : Read_Buffer[i];
			max = max > Read_Buffer[i] ? max : Read_Buffer[i]; 
			Read_Buffer[i] = Read_Buffer[i+1];
		}
		read_temp = (datacore - min - max) / (BUFFER_MAX - 2);
	}
    return read_temp;
} 

// *************************************************************************************
// 	input: none
//	output: none 
//  a task for pressure sensor
// ************************************************************************************* 
void Pressure_Task(void)
{ 
//	static float pre_value = 0;
	s16 stemp;
	if(Run_Timer <= FIRST_TIME) Run_Timer ++;
		
	air_flow_ad = ADC_getChannal(ADC2,ADC_Channel_12);
	AD_Value[8] = air_flow_ad;
	
 	Pressure.ad = get_ad_val(air_flow_ad);
	
	if(Pressure.cal_table_enable == 1)
	{
		Pressure.cal_table_enable = 0;
		
		if(Pressure.table_sel == FACTORY_TABLE)
		{
			Pressure.cal_point = get_cal_point(Pressure.cal_pr);
			AT24CXX_WriteOneByte(EEP_PRESSURE_CAL_POINT,Pressure.cal_point);
//			min2method(&Pressure.k_line ,&Pressure.b_line,Pressure.cal_point,Pressure.cal_ad,Pressure.cal_pr);
		}
//		else if(Pressure.table_sel == USER_TABLE)
//		{
//			if(Pressure.user_cal_point > 1)// 2 ~ 10 points
//				min2method(&Pressure.k_line ,&Pressure.b_line,Pressure.user_cal_point,Pressure.user_cal_ad,Pressure.user_cal_pr);
//			else if(Pressure.user_cal_point == 1)
//			{
//				min2method(&Pressure.k_line ,&Pressure.b_line,Pressure.cal_point,Pressure.cal_ad,Pressure.cal_pr);
//				Pressure.org_val_offset += (Pressure.user_cal_pr[0] - Pressure.org_val ); 
//				write_eeprom(EEP_PRESSURE_VALUE_ORG_OFFSET,Pressure.org_val_offset);
//				write_eeprom(EEP_PRESSURE_VALUE_ORG_OFFSET + 1,Pressure.org_val_offset >> 8);
//			}
//			else 
//				min2method(&Pressure.k_line ,&Pressure.b_line,Pressure.cal_point,Pressure.cal_ad,Pressure.cal_pr);
//				
//		}
	}
//	if((output_auto_manual & 0x04)||(Pressure.auto_manu))	//manu mode
//	{
//		Pressure.org_val = output_manual_value_co2;
////		Pressure.org_val += Pressure.org_val_offset; 
////		if((Pressure.org_val > Pressure.pr_range[1])||(Pressure.org_val < Pressure.pr_range[0]))
////			Pressure.out_rng_flag = 1;
////		else 
////			Pressure.out_rng_flag = 0; 
////		Pressure.unit = Pressure.default_unit;
// 		Pressure.val_temp = (float)Pressure.org_val * get_units_ratio(Pressure.unit,Pressure.default_unit);			
//		datatype_convert(Pressure.val_temp,&(Pressure.base),&(Pressure.index),Pressure.unit,Pressure.default_unit);  
//	}
//	else	//auto mode
	{  
		 
		stemp = get_pressure(Pressure.ad);
 		
		if(Run_Timer > FIRST_TIME)
			Pressure.pre_val = Sys_Filter(stemp,Pressure.pre_val,Pressure.filter );
		else
			Pressure.pre_val = stemp;
		
		
 		Pressure.org_val = Pressure.pre_val;// + Pressure.org_val_offset; 
// 		Pressure.org_val = 70; //test ,air_speed = 5.9m/s
		
		if(!Pressure.air_flow_unit) //metric
		{
			Pressure.air_speed = sqrt(Pressure.org_val/2)*Pressure.K_Flow_Speed/100; //P = k * V^2  (0.1m/s)
			//the s = 0.20*0.30 = 0.06m2;
			//the air_flow = s*speed(m/s) 
			//Pressure.air_flow.F_Type = Pressure.air_speed/10*k/1000*60*Pressure.duct_area/1000;
			Pressure.air_flow.F_Type = 6.0*Pressure.air_speed*Pressure.K_factor*Pressure.duct_area/1000000;  //
		}
		else//Imperial units
		{
			Pressure.air_speed = sqrt(Pressure.org_val/2)*Pressure.K_Flow_Speed*0.0328084; //(0.1fps)  1m/s = 3.28084
			Pressure.air_flow.F_Type = 6.0*Pressure.air_speed*Pressure.K_factor*Pressure.duct_area/1000000; 
		}

//		output_manual_value_co2 = Pressure.org_val;
//		if((Pressure.org_val > Pressure.pr_range[1])||(Pressure.org_val < Pressure.pr_range[0]))
//			Pressure.out_rng_flag = 1;
//		else 
//			Pressure.out_rng_flag = 0; 
//		
// 		Pressure.val_temp = (float)Pressure.org_val * get_units_ratio(Pressure.unit,Pressure.default_unit);			
//		datatype_convert(Pressure.val_temp,&(Pressure.base),&(Pressure.index),Pressure.unit,Pressure.default_unit);   	 
	} 
//	var[CHANNEL_PRE]. value = Pressure.org_val;
}
// *************************************************************************************
// 	input: product model
//	output: the defalut unit of the product 
//   
// ************************************************************************************* 
//u8 get_default_unit(u8 pro_model)
//{
//	u8 temp;
//	if(pro_model == DLVR_L01D)
//	{
//		temp = inWC;
//		Pressure.pr_range[0] = -100;
//		Pressure.pr_range[1] = 100;
//	}
//	else if(pro_model ==MPXV7002DP)
//	{ 
//		temp = inWC;
//		Pressure.pr_range[0] = -800;
//		Pressure.pr_range[1] = 800;
//	}
//	else if(pro_model ==MPXV7007DP)
//	{
//		temp = inWC;
//		Pressure.pr_range[0] = -2700;
//		Pressure.pr_range[1] = 2700;
//	}
//	else if(pro_model ==MS4515)
//	{
//		temp = inWC;
//		Pressure.pr_range[0] = -1000;
//		Pressure.pr_range[1] = 1000;
//	}
//	else if(pro_model ==PRS_26PCDFA)
//	{
//		temp = Psi;
//		Pressure.pr_range[0] = -1000;
//		Pressure.pr_range[1] = 1000;
//	}
//	else if(pro_model ==PRS_26PCGFA)
//	{
//		temp = Psi;
//		Pressure.pr_range[0] = -2500;
//		Pressure.pr_range[1] = 2500;
//	}
//	return temp;
//}

static void Pressure_EEP_Initial()
{ 
	
	if(AT24CXX_ReadOneByte(EEP_PRESSURE_TEST) != 123)
	{
		AT24CXX_WriteOneByte(EEP_PRESSURE_TEST,123);
		AT24CXX_WriteOneByte(EEP_PRESSURE_VALUE_ORG_OFFSET,0);
		AT24CXX_WriteOneByte(EEP_PRESSURE_VALUE_ORG_OFFSET + 1,0);
		AT24CXX_WriteOneByte(EEP_USER_CAL_OFFSET,0);
		AT24CXX_WriteOneByte(EEP_USER_CAL_OFFSET + 1,0);
//		AT24CXX_WriteOneByte(EEP_PRESSURE_SENSOR_MODEL,PRS_26PCDFA);
//		AT24CXX_WriteOneByte(EEP_PRESSURE_UNIT,Psi); 
		AT24CXX_WriteOneByte(EEP_PRESSURE_FILTER,DEFAULT_FILTER);   
		AT24CXX_WriteOneByte(EEP_PRESSURE_CAL_POINT,0);
		AT24CXX_WriteOneByte(EEP_TABLE_SEL,FACTORY_TABLE);
	    AT24CXX_WriteOneByte(EEP_USER_CAL_POINT,0);
		AT24CXX_WriteOneByte(EEP_PRESSURE_K,(u8)1000);
		AT24CXX_WriteOneByte(EEP_PRESSURE_K+1,(u8)(1000>>8));
		AT24CXX_WriteOneByte(EEP_K_FLOW_SPEED,(u8)1000);
		AT24CXX_WriteOneByte(EEP_K_FLOW_SPEED+1,(u8)(1000>>8));
		AT24CXX_WriteOneByte(EEP_AIR_FLOW_UNIT,0);
		Pressure.duct_area  = 60; //0.060m2;
		AT24CXX_WriteOneByte(EEP_DUCT_AREA,Pressure.duct_area);
		AT24CXX_WriteOneByte(EEP_DUCT_AREA + 1,Pressure.duct_area>>8); 	
	}
}
void Pressure_initial(void)
{ 
	Pressure_EEP_Initial();
	Pressure.duct_area = ((u16)AT24CXX_ReadOneByte(EEP_DUCT_AREA+1)<<8) | AT24CXX_ReadOneByte(EEP_DUCT_AREA); 
	Pressure.air_flow_unit = AT24CXX_ReadOneByte(EEP_AIR_FLOW_UNIT);
	Pressure.K_Flow_Speed = ((u16)AT24CXX_ReadOneByte(EEP_K_FLOW_SPEED+1)<<8) | AT24CXX_ReadOneByte(EEP_K_FLOW_SPEED);
	Pressure.K_factor = ((u16)AT24CXX_ReadOneByte(EEP_PRESSURE_K+1)<<8) | AT24CXX_ReadOneByte(EEP_PRESSURE_K);
	Pressure.org_val_offset = ((u16)AT24CXX_ReadOneByte(EEP_PRESSURE_VALUE_ORG_OFFSET+1)<<8) | AT24CXX_ReadOneByte(EEP_PRESSURE_VALUE_ORG_OFFSET); 
	Pressure.user_val_offset = ((u16)AT24CXX_ReadOneByte(EEP_USER_CAL_OFFSET+1)<<8) | AT24CXX_ReadOneByte(EEP_USER_CAL_OFFSET);
//	Pressure.SNR_Model = read_eeprom(EEP_PRESSURE_SENSOR_MODEL);
//	Pressure.default_unit = get_default_unit(Pressure.SNR_Model);
//	Pressure.unit = read_eeprom(EEP_PRESSURE_UNIT); 
//	Pressure.min_pre = ((u16)read_eeprom(EEP_OUTPUT_RANGE_MIN_PRESSURE+1)<<8) | read_eeprom(EEP_OUTPUT_RANGE_MIN_PRESSURE);					
//	Pressure.max_pre = ((u16)read_eeprom(EEP_OUTPUT_RANGE_MAX_PRESSURE+1)<<8) | read_eeprom(EEP_OUTPUT_RANGE_MAX_PRESSURE);	
	Pressure.filter = AT24CXX_ReadOneByte(EEP_PRESSURE_FILTER);  
//	Pressure.auto_manu = read_eeprom(EEP_INPUT_AUTO_MANUAL_PRE);//0 = AUTO,1 = MANUAL 
	
	Pressure.table_sel = AT24CXX_ReadOneByte(EEP_TABLE_SEL);
	Pressure.user_cal_point = AT24CXX_ReadOneByte(EEP_USER_CAL_POINT);
	Pressure.cal_point = AT24CXX_ReadOneByte(EEP_PRESSURE_CAL_POINT);
	test_point_read();
//	if(Pressure.table_sel == FACTORY_TABLE)
//		min2method(&Pressure.k_line ,&Pressure.b_line,Pressure.cal_point,Pressure.cal_ad,Pressure.cal_pr);
//	else if(Pressure.table_sel == USER_TABLE)
//		min2method(&Pressure.k_line ,&Pressure.b_line,Pressure.user_cal_point,Pressure.user_cal_ad,Pressure.user_cal_pr);
	Pressure.cal_table_enable = 1; 
	Pressure.unit_change = 1;
	Pressure.out_rng_flag = 0;
	Pressure.ad = 0;  
}

void vUpdate_Pressure_Task( void *pvParameters )
{ 
	 Pressure_initial(); 
	 delay_ms(100); 
	 for(;;)
	 {  
		 
		Pressure_Task();

		delay_ms(1000);
		//vTaskDelay(1000 / portTICK_RATE_MS);
	 }
}	

#endif

