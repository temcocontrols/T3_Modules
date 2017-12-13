#ifndef _DEFINE_H
#define _DEFINE_H

#define STM32F10X



#ifdef STM32F10X
#define far 
#define xdata
#define idata
#endif

#define READ_WRITE_PROPERTY 1

//#define MODBUS 0 
//#define BAC_MSTP 1
//#define TCP_IP 2
//#define BAC_IP 3
//#define BAC_PTP 4
//#define BAC_GSM 5

#define DEFAULT_FILTER  5
//#define T322AI		43
//#define T38AI8AO6DO   44
//#define T3PT12			46
#define T36CTA		   95

//#define SW_VER 20150112


#define	BAUDRATE_9600				9600			//0
#define	BAUDRATE_19200			19200			//1
#define	BAUDRATE_38400			38400			//2
#define	BAUDRATE_57600			57600			//3
#define	BAUDRATE_115200			115200		//4
#define	BAUDRATE_76800			76800			//5
#define EN_OUT 0 
#define EN_IN  1
#define EN_VAR 2
#define EN_CUSTOMER_RANGE 3

#ifdef  T3PT12
#define HW_VER			1
#define MAX_AI_CHANNEL	 12
#define MAX_AIS         MAX_AI_CHANNEL
#define MAX_INS					MAX_AIS
#define PRODUCT_ID 		T3PT12
#define  MAX_AVS  		29
#define  MAX_AV			MAX_AVS
//#define MAX_OUTS				1
//#define MAX_AOS					1
#define INPUT_CONTROL
#endif

#ifdef T36CTA
#define T36CTA_REV1    0
#define T36CTA_REV2    1
#define INPUT_CONTROL
#define OUTPUT_CONTROL
#define  MAX_AVS  	10//41
#define MAX_AV					MAX_AVS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif


#ifdef T38AI8AO6DO
#define INPUT_CONTROL
#define OUTPUT_CONTROL
#define  MAX_AVS  	41
#define MAX_AV					MAX_AVS
#endif


#ifdef T322AI 
#define INPUT_CONTROL
#define PRODUCT_ID 		T322AI
#define HW_VER				8
#define MAX_AI_CHANNEL	 	22
#define MAX_AIS        		MAX_AI_CHANNEL
#define MAX_INS  			MAX_AIS 
#define MAX_AVS  			40
#define MAX_AV				MAX_AVS
//#define MAX_OUTS				1
//#define MAX_AOS					1
//IO操作函数	 
//#define CHA_SEL0				PAout(0)	//SCL
//#define CHA_SEL1				PAout(1)	//SDA	 
//#define CHA_SEL2				PAout(2)		//输入SDA
//#define CHA_SEL3				PAout(3)		//输入SDA
//#define CHA_SEL4				PAout(4)

#define RANGE_SET0			PCout(8)
#define RANGE_SET1			PCout(9)
#endif 

#ifdef T38AI8AO6DO 
#define SWITCH_NUM 14

#define PRODUCT_ID 		T38AI8AO6DO
#define HW_VER				8
#define MAX_AI_CHANNEL	 	8
#define MAX_AIS         MAX_AI_CHANNEL
#define MAX_INS  				MAX_AIS 

#define MAX_AO					8
#define MAX_AOS					8
#define MAX_DO					6
#define MAX_OUTS				(MAX_AO+MAX_DO)
//#define MAX_AV					20


#define  SW_OFF  0
#define  SW_HAND 2
#define  SW_AUTO 1

////IO操作函数	 
//#define CHA_SEL0				PAout(0)	//SCL
//#define CHA_SEL1				PAout(1)	//SDA	 
//#define CHA_SEL2				PAout(2)		//输入SDA
//#define CHA_SEL3				PAout(3)		//输入SDA
//#define CHA_SEL4				PAout(4)

//#define RANGE_SET0			PCout(8)
//#define RANGE_SET1			PCout(9)



//#define PRODUCT_ID 		T38AI8AO6DO
//#define HW_VER			8
//#define MAX_AI_CHANNEL	 8
//#define MAX_AIS         MAX_AI_CHANNEL
//#define MAX_INS  MAX_AIS

//#define MAX_AOS					8
//#define MAX_DOS					6 
//#define MAX_OUTS				(MAX_AOS+MAX_DOS)


//IO操作函数	 
#define CHA_SEL0				PCout(0)		//		PIN15
#define CHA_SEL1				PCout(1)	//	 		PIN16
#define CHA_SEL2				PCout(2)		//	PIN13



#define CHA_SEL4				PAout(7)
#define RANGE_SET0			PAout(5)
#define RANGE_SET1			PAout(4)
#endif 

#ifdef T36CTA 
#define SWITCH_NUM 			2

#define PRODUCT_ID 		T36CTA
	#if T36CTA_REV1
		#define HW_VER				1
		#define MAX_AI_CHANNEL	 	19
		#define MAX_AIS         MAX_AI_CHANNEL
		#define MAX_INS  				MAX_AIS 
	#elif T36CTA_REV2
		#define HW_VER				2
		#define MAX_AI_CHANNEL	 	23    // 8 input 6 ct 1 airflow   //16
		#define MAX_AIS         MAX_AI_CHANNEL
		#define MAX_INS  				MAX_AIS 
		#define   FAC_TABLE                    0
		#define   USER_TABLE                   1
		#define AIR_FLOW_SENSOR
	#endif

#define MAX_AO					0
#define MAX_AOS					0
#define MAX_DO					2
#define MAX_OUTS				2
//#define MAX_AV					20


#define  SW_OFF  0
#define  SW_HAND 2
#define  SW_AUTO 1

//IO操作函数	 


#define CHA_SEL4				PDout(15)
#define RANGE_SET0			PEout(13)
#define RANGE_SET1			PEout(14)
#endif 


#endif 

