#ifndef __AIRFLOW_H__
#define __AIRFLOW_H__ 

#include "types.h"
#include "define.h"
#include "bitmap.h"  
 
#ifdef AIR_FLOW_SENSOR

#define FACTORY_TABLE	0
#define USER_TABLE		1

#define DEFAULT_FILTER	5
#define FIRST_TIME   30

typedef union
{
	float F_Type;
	u8    C_Type[4]; 
}u_type;

typedef struct
{  
	u8 SNR_Model;   
	u8 filter;
	u8 auto_manu;  
	u8 unit;
	u8 default_unit;
	u16 ad;
	s16 org_val;
	float pre_val;
	s16 org_val_offset; 
	s16 user_val_offset;
	float val_temp; 
	u8  cal_point;
	u8 user_cal_point; 
	u16 cal_pr[10];
	u16 cal_ad[10];
	u16 user_cal_pr[10];
	u16 user_cal_ad[10]; 
	s16 pr_range[2];
	float b_line;
	float k_line; 
	u8 cal_table_enable	;
	u8 table_sel;		  // 0 = factory table, 1 = user table 
	u8 unit_change;
	u8 out_rng_flag;
	u8 sensor_status;     //0= no error, 2 = the value no update, 3 = invalid
	u16 K_factor;		  //default 1000;
	u16 K_Flow_Speed;     //default 1000;
	u16 air_speed;		  //0.1m/s  , fps
	u_type air_flow;	  //m3/min , cfm
	u8 air_flow_unit;	  //0=metric,1= Imperial units. 
	u16 duct_area;
}_STR_PRESSURE_;

enum
{
	inWC = 0,
	KPa1 = 1,
	Psi = 2,
	mmHg = 3,
	inHg = 4,
	Kg_cm2 = 5,
	atmosphere = 6,
	bar = 7, 
	Unit_End,
};  

extern _STR_PRESSURE_  Pressure;
extern uint8   Run_Timer; 
void vUpdate_Pressure_Task( void *pvParameters );

#endif

#endif

