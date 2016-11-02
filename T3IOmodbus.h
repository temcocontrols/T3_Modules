// *******************T3IOmodbus.h***********************************
// Header file containing all of the register information for modbus 
// serial communications.



// -------------- TEMCO flash registers --------------------------
//	the following are reserved TEMCO registers common to all products
//	range from 0 to 100
//		note for now Robert has added pulsing in this section
//		plan to be removed in the future
// ---------------------------------------------------------------
#if defined (T3_32IN)
	#define NUM_INPUTS				32
#else
	#define NUM_INPUTS				8
#endif

enum {

	FLASH_SERIALNUMBER_LOWORD,             
	FLASH_SERIALNUMBER_HIWORD		= 2,
	EEPROM_VERSION_NUMBER			= 4,
	FLASH_SOFTWARE_VERSION_NUMBER	= 5,
	FLASH_ADDRESS					= 6,
	FLASH_PRODUCT_MODEL,
	FLASH_HARDWARE_REV,
	FLASH_PIC_VERSION,
	FLASH_ADDRESS_PLUG_N_PLAY		= 10,	// 10
	LAST_DISPLAY_ADDRESS,

	FLASH_SEQUENCE,			// JULY 16th by Ron	
			// SOP = 1  -->	normal operation
			// SOP = 0  -->	test sequence mode
			// SOP = 2  -->	Timer SOP mode
			// SOP = 3  -->	Switch States set to automatic mode
			// SOP = 5  -->	Calibration, produce LUT

  	FLASH_CALIBRATION,
  	FLASH_PWM_TIME_CALIBRATION,
	FLASH_BAUDRATE		= 15,

	// registers needed for updating status
	FLASH_UPDATE_STATUS	= 16,		// reg 16 August Ron
	FLASH_DEAD_MASTER,				// 17  UNITS IS MINUTE, 0 ~ 255,WILL RESET TO AUTO IN THIS TIME NO COMMUNICATION
	FLASH_init,
	FLASH_RESPOND_DELAY,
	FLASH_BASE_ADDRESS = 20,


#ifdef T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS

	// special registers for high speed input
	//	TBD move this to lower section of the code
	FLASH_START_PULSE ,

	FLASH_PULSE1_HI_HI = FLASH_START_PULSE ,
//	FLASH_PULSE1_HI_LO,
//	FLASH_PULSE1_LO_HI,
	FLASH_PULSE1_LO_LO,

	FLASH_PULSE2_HI_HI,
//	FLASH_PULSE2_HI_LO,
//	FLASH_PULSE2_LO_HI,
	FLASH_PULSE2_LO_LO,

	FLASH_PULSE3_HI_HI,
//	FLASH_PULSE3_HI_LO,
//	FLASH_PULSE3_LO_HI,
	FLASH_PULSE3_LO_LO,

	FLASH_PULSE4_HI_HI,
//	FLASH_PULSE4_HI_LO,
//	FLASH_PULSE4_LO_HI,
	FLASH_PULSE4_LO_LO,

	FLASH_PULSE5_HI_HI,
//	FLASH_PULSE5_HI_LO,
//	FLASH_PULSE5_LO_HI,
	FLASH_PULSE5_LO_LO,

	FLASH_PULSE6_HI_HI,
//	FLASH_PULSE6_HI_LO,
//	FLASH_PULSE6_LO_HI,
	FLASH_PULSE6_LO_LO,

	FLASH_PULSE7_HI_HI,
//	FLASH_PULSE7_HI_LO,
//	FLASH_PULSE7_LO_HI,
	FLASH_PULSE7_LO_LO,

	FLASH_PULSE8_HI_HI,
//	FLASH_PULSE8_HI_LO,
//	FLASH_PULSE8_LO_HI,
	FLASH_PULSE8_LO_LO,

	FLASH_PULSE1_YEAR,
	FLASH_PULSE1_MONTH,
	FLASH_PULSE1_DATE,
	FLASH_PULSE1_HOUR,
	FLASH_PULSE1_MINUTE,

	FLASH_PULSE2_YEAR,
	FLASH_PULSE2_MONTH,
	FLASH_PULSE2_DATE,
	FLASH_PULSE2_HOUR,
	FLASH_PULSE2_MINUTE,

	FLASH_PULSE3_YEAR,
	FLASH_PULSE3_MONTH,
	FLASH_PULSE3_DATE,
	FLASH_PULSE3_HOUR,
	FLASH_PULSE3_MINUTE,
	
	FLASF_PULSE4_YEAR,
	FLASF_PULSE4_MONTH,
	FLASH_PULSE4_DATE,
	FLASH_PULSE4_HOUR,
	FLASH_PULSE4_MINUTE,

	FLASH_PULSE5_YEAR,
	FLASH_PULSE5_MONTH,
	FLASH_PULSE5_DATE,
	FLASH_PULSE5_HOUR,
	FLASH_PULSE5_MINUTE,

	FLASH_PULSE6_YEAR,
	FLASH_PULSE6_MONTH,
	FLASH_PULSE6_DATE,
	FLASH_PULSE6_HOUR,
	FLASH_PULSE6_MINUTE,
	
	FLASH_PULSE7_YEAR,
	FLASH_PULSE7_MONTH,
	FLASH_PULSE7_DATE,
	FLASH_PULSE7_HOUR,
	FLASH_PULSE7_MINUTE,
	
	FLASH_PULSE8_YEAR,
	FLASH_PULSE8_MONTH,
	FLASH_PULSE8_DATE,
	FLASH_PULSE8_HOUR,
	FLASH_PULSE8_MINUTE,
#endif

	FLASH_CHANNEL_TYPE, 

	FLASH_INPUT1_RANGE,
	FLASH_INPUT2_RANGE,
	FLASH_INPUT3_RANGE,
	FLASH_INPUT4_RANGE,
	FLASH_INPUT5_RANGE,
	FLASH_INPUT6_RANGE,
	FLASH_INPUT7_RANGE,
	FLASH_INPUT8_RANGE,
#ifdef T3_32IN
	FLASH_INPUT9_RANGE,
	FLASH_INPUT10_RANGE,
	FLASH_INPUT11_RANGE,
	FLASH_INPUT12_RANGE,
	FLASH_INPUT13_RANGE,
	FLASH_INPUT14_RANGE,
	FLASH_INPUT15_RANGE,
	FLASH_INPUT16_RANGE,

	FLASH_INPUT17_RANGE,
	FLASH_INPUT18_RANGE,
	FLASH_INPUT19_RANGE,
	FLASH_INPUT20_RANGE,
	FLASH_INPUT21_RANGE,
	FLASH_INPUT22_RANGE,
	FLASH_INPUT23_RANGE,
	FLASH_INPUT24_RANGE,

	FLASH_INPUT25_RANGE,
	FLASH_INPUT26_RANGE,
	FLASH_INPUT27_RANGE,
	FLASH_INPUT28_RANGE,
	FLASH_INPUT29_RANGE,
	FLASH_INPUT30_RANGE,
	FLASH_INPUT31_RANGE,
	FLASH_INPUT32_RANGE,
#endif

	FLASH_INPUT1_FILTER,
	FLASH_INPUT2_FILTER,
	FLASH_INPUT3_FILTER,
	FLASH_INPUT4_FILTER,
	FLASH_INPUT5_FILTER,
	FLASH_INPUT6_FILTER,
	FLASH_INPUT7_FILTER,
	FLASH_INPUT8_FILTER,
#ifdef T3_32IN
	FLASH_INPUT9_FILTER,
	FLASH_INPUT10_FILTER,
	FLASH_INPUT11_FILTER,
	FLASH_INPUT12_FILTER,
	FLASH_INPUT13_FILTER,
	FLASH_INPUT14_FILTER,
	FLASH_INPUT15_FILTER,
	FLASH_INPUT16_FILTER,
	FLASH_INPUT17_FILTER,
	FLASH_INPUT18_FILTER,
	FLASH_INPUT19_FILTER,
	FLASH_INPUT20_FILTER,
	FLASH_INPUT21_FILTER,
	FLASH_INPUT22_FILTER,
	FLASH_INPUT23_FILTER,
	FLASH_INPUT24_FILTER,
	FLASH_INPUT25_FILTER,
	FLASH_INPUT26_FILTER,
	FLASH_INPUT27_FILTER,
	FLASH_INPUT28_FILTER,
	FLASH_INPUT29_FILTER,
	FLASH_INPUT30_FILTER,
	FLASH_INPUT31_FILTER,
	FLASH_INPUT32_FILTER,	
	FLASH_INPUT0_CALIBRATION,  //POINTER TO FIRST CALIBRATION REGISTER
	FLASH_INPUT1_CALIBRATION = FLASH_INPUT0_CALIBRATION+ NUM_INPUTS -1,//POINTER TO LAST CALIBRATION REGISTER
	FLASH_RANGE_LO,
	FLASH_RANGE_HI,	
    #endif 

	FLASH_INPUT1_TIMER,
	FLASH_INPUT2_TIMER,
	FLASH_INPUT3_TIMER,
	FLASH_INPUT4_TIMER,
	FLASH_INPUT5_TIMER,
	FLASH_INPUT6_TIMER,
	FLASH_INPUT7_TIMER,
	FLASH_INPUT8_TIMER,
	FLASH_OUTPUT_MANUAL,
	FLASH_ZONE_OUTPUT1,
	FLASH_ZONE_OUTPUT2,
	FLASH_ZONE_OUTPUT3,
	FLASH_ZONE_OUTPUT4,
	FLASH_ZONE_OUTPUT5,
	FLASH_ZONE_OUTPUT6,
	FLASH_ZONE_OUTPUT7,
	FLASH_ZONE_OUTPUT8,
	FLASH_ZONE_OUTPUT9,
	FLASH_ZONE_OUTPUT10,
	FLASH_ZONE_OUTPUT11,
	FLASH_ZONE_OUTPUT12,
	FLASH_ZONE_OUTPUT13,
	FLASH_REVERSE_OUTPUT,
	#ifndef T3_32IN
	FLASH_INPUT0_CALIBRATION,  //POINTER TO FIRST CALIBRATION REGISTER
	FLASH_INPUT1_CALIBRATION = FLASH_INPUT0_CALIBRATION+ NUM_INPUTS -1,//POINTER TO LAST CALIBRATION REGISTER
	#endif
	#ifdef T3_8IN13OUT
	FLASH_FLASH_WRITE_TIME ,  //set the flash write  time  the unit is minutes.
	#endif
	MAX_FLASH_CONSTRANGE,
	
	#ifdef T3_8IO_A
	FLASH_DEADMASTER_AO0 ,
	FLASH_DEADMASTER_AO1 ,
	FLASH_DEADMASTER_AO2 ,
	FLASH_DEADMASTER_AO3 ,
	FLASH_DEADMASTER_AO4 ,
	FLASH_DEADMASTER_AO5 ,
	FLASH_DEADMASTER_AO6 ,
	FLASH_DEADMASTER_AO7 ,	
	#endif
	CALIBRATION_STORAGE_LOCATION = 350,	// flash data, in modbus will start at 300
	CALIBRATION_GRID = 500				// calibration LUT, in modbus will start at 500
};




// -------------- Registers above 100 are application register --------------------------
//	the following are custom to the code's application registers
//	starting at 100
//
//
// --------------------------------------------------------------------------------------
#define ORIGINALADDRESSVALUE	100 			// Offset for 2nd block of registers

// --------------- 32 inputs module -----------------------------------------------------
// --------------------------------------------------------------------------------------

#if defined (T3_32IN)
//		T3 32Input module
enum{
	// 100  			
	T32I_INPUT1  = ORIGINALADDRESSVALUE,		// Input 1, register 100
	T32I_INPUT2  ,                   			// Input 2 
	T32I_INPUT3  ,                   			// Input 3 
	T32I_INPUT4  ,                   			// Input 4 
	T32I_INPUT5  ,                   			// Input 5 
	T32I_INPUT6  ,                   			// Input 6 
	T32I_INPUT7  ,                   			// Input 7 
	T32I_INPUT8  ,                   			// Input 8

	T32I_INPUT9  ,                   			// Input 9, register 108
	T32I_INPUT10  ,                   			// Input 10
	T32I_INPUT11  ,                   			// Input 11
	T32I_INPUT12  ,                   			// Input 12
	T32I_INPUT13  ,                   			// Input 13
	T32I_INPUT14  ,                   			// Input 14
	T32I_INPUT15  ,                   			// Input 15
	T32I_INPUT16  ,                   			// Input 16

	T32I_INPUT17  ,                   			// Input 17, register 116
	T32I_INPUT18  ,                   			// Input 18
	T32I_INPUT19  ,                   			// Input 19
	T32I_INPUT20  ,                   			// Input 20
	T32I_INPUT21  ,                   			// Input 21
	T32I_INPUT22  ,                   			// Input 22
	T32I_INPUT23  ,                   			// Input 23
	T32I_INPUT24  ,                   			// Input 24

	T32I_INPUT25  ,                   			// Input 25, register 124
	T32I_INPUT26  ,                   			// Input 26
	T32I_INPUT27  ,                   			// Input 27
	T32I_INPUT28  ,                   			// Input 28
	T32I_INPUT29  ,                   			// Input 29
	T32I_INPUT30  ,                   			// Input 30
	T32I_INPUT31  ,                   			// Input 31
	T32I_INPUT32  ,                   			// Input 32


	FlexDriver_LED1,	// register 132
	FlexDriver_LED2,
	FlexDriver_LED3,
	FlexDriver_LED4,
	FlexDriver_LED5,
	FlexDriver_LED6,
	FlexDriver_LED7,
	FlexDriver_LED8,

	FlexDriver_Blink_Set1_LED1,		// register 140
	FlexDriver_Blink_Set1_LED2,
	FlexDriver_Blink_Set1_LED3,
	FlexDriver_Blink_Set1_LED4,
	FlexDriver_Blink_Set1_LED5,
	FlexDriver_Blink_Set1_LED6,
	FlexDriver_Blink_Set1_LED7,
	FlexDriver_Blink_Set1_LED8,

	FlexDriver_Blink_Set2_LED1,
	FlexDriver_Blink_Set2_LED2,
	FlexDriver_Blink_Set2_LED3,
	FlexDriver_Blink_Set2_LED4,
	FlexDriver_Blink_Set2_LED5,
	FlexDriver_Blink_Set2_LED6,
	FlexDriver_Blink_Set2_LED7,
	FlexDriver_Blink_Set2_LED8 , 

	
	
	// --- LED set A ----------
	
	FlexDriver_LEDA_PAIR1,	// 156
	FlexDriver_LEDA_PAIR2,
	FlexDriver_LEDA_PAIR3,
	FlexDriver_LEDA_PAIR4,
	FlexDriver_LEDA_PAIR5,
	FlexDriver_LEDA_PAIR6,
	FlexDriver_LEDA_PAIR7,
	FlexDriver_LEDA_PAIR8,
	
	FlexDriver_LEDA_PAIR9,	// 164
	FlexDriver_LEDA_PAIR10,
	FlexDriver_LEDA_PAIR11,
	FlexDriver_LEDA_PAIR12,
	FlexDriver_LEDA_PAIR13,
	FlexDriver_LEDA_PAIR14,
	FlexDriver_LEDA_PAIR15,
	FlexDriver_LEDA_PAIR16,
	
	FlexDriver_LEDA_PAIR17,	// 172
	FlexDriver_LEDA_PAIR18,
	FlexDriver_LEDA_PAIR19,
	FlexDriver_LEDA_PAIR20,
	FlexDriver_LEDA_PAIR21,
	FlexDriver_LEDA_PAIR22,
	FlexDriver_LEDA_PAIR23,
	FlexDriver_LEDA_PAIR24,
	
	FlexDriver_LEDA_PAIR25,	// 180
	FlexDriver_LEDA_PAIR26,
	FlexDriver_LEDA_PAIR27,
	FlexDriver_LEDA_PAIR28,
	FlexDriver_LEDA_PAIR29,
	FlexDriver_LEDA_PAIR30,
	FlexDriver_LEDA_PAIR31,
	FlexDriver_LEDA_PAIR32,
	
	// --- LED set B --------------
	
	FlexDriver_LEDB_PAIR1,	// 188
	FlexDriver_LEDB_PAIR2,
	FlexDriver_LEDB_PAIR3,
	FlexDriver_LEDB_PAIR4,
	FlexDriver_LEDB_PAIR5,
	FlexDriver_LEDB_PAIR6,
	FlexDriver_LEDB_PAIR7,
	FlexDriver_LEDB_PAIR8,
	
	FlexDriver_LEDB_PAIR9,	// 196
	FlexDriver_LEDB_PAIR10,
	FlexDriver_LEDB_PAIR11,
	FlexDriver_LEDB_PAIR12,
	FlexDriver_LEDB_PAIR13,
	FlexDriver_LEDB_PAIR14,
	FlexDriver_LEDB_PAIR15,
	FlexDriver_LEDB_PAIR16,
	
	FlexDriver_LEDB_PAIR17,	// 204
	FlexDriver_LEDB_PAIR18,
	FlexDriver_LEDB_PAIR19,
	FlexDriver_LEDB_PAIR20,
	FlexDriver_LEDB_PAIR21,
	FlexDriver_LEDB_PAIR22,
	FlexDriver_LEDB_PAIR23,
	FlexDriver_LEDB_PAIR24,
	
	FlexDriver_LEDB_PAIR25,	// 212
	FlexDriver_LEDB_PAIR26,
	FlexDriver_LEDB_PAIR27,
	FlexDriver_LEDB_PAIR28,
	FlexDriver_LEDB_PAIR29,
	FlexDriver_LEDB_PAIR30,
	FlexDriver_LEDB_PAIR31,
	FlexDriver_LEDB_PAIR32,
	
	// --- LED status for on/off/blinking ----------
	
	FlexDriver_LED_STATUS_PAIR1,	// 220
	FlexDriver_LED_STATUS_PAIR2,
	FlexDriver_LED_STATUS_PAIR3,
	FlexDriver_LED_STATUS_PAIR4,
	FlexDriver_LED_STATUS_PAIR5,
	FlexDriver_LED_STATUS_PAIR6,
	FlexDriver_LED_STATUS_PAIR7,
	FlexDriver_LED_STATUS_PAIR8,
	
	FlexDriver_LED_STATUS_PAIR9,	// 228
	FlexDriver_LED_STATUS_PAIR10,
	FlexDriver_LED_STATUS_PAIR11,
	FlexDriver_LED_STATUS_PAIR12,
	FlexDriver_LED_STATUS_PAIR13,
	FlexDriver_LED_STATUS_PAIR14,
	FlexDriver_LED_STATUS_PAIR15,
	FlexDriver_LED_STATUS_PAIR16,
	
	FlexDriver_LED_STATUS_PAIR17,	// 236
	FlexDriver_LED_STATUS_PAIR18,
	FlexDriver_LED_STATUS_PAIR19,
	FlexDriver_LED_STATUS_PAIR20,
	FlexDriver_LED_STATUS_PAIR21,
	FlexDriver_LED_STATUS_PAIR22,
	FlexDriver_LED_STATUS_PAIR23,
	FlexDriver_LED_STATUS_PAIR24,
	
	FlexDriver_LED_STATUS_PAIR25,	//  244
	FlexDriver_LED_STATUS_PAIR26,
	FlexDriver_LED_STATUS_PAIR27,
	FlexDriver_LED_STATUS_PAIR28,
	FlexDriver_LED_STATUS_PAIR29,
	FlexDriver_LED_STATUS_PAIR30,
	FlexDriver_LED_STATUS_PAIR31,
	FlexDriver_LED_STATUS_PAIR32,

	T38IO_INPUT1_RANGE,					//228	Set range for each input. 0 = raw data,1 = 10K Celsius,2 = 10K Fahrenheit			  
	T38IO_INPUT2_RANGE,					//		3 = 0 - 100%,4 = ON/OFF,5 = OFF/ON
	T38IO_INPUT3_RANGE,
	T38IO_INPUT4_RANGE,	
	T38IO_INPUT5_RANGE,
	T38IO_INPUT6_RANGE,
	T38IO_INPUT7_RANGE,
	T38IO_INPUT8_RANGE,

	T38IO_INPUT9_RANGE,	 	//236
	T38IO_INPUT10_RANGE,
	T38IO_INPUT11_RANGE,
	T38IO_INPUT12_RANGE,
	T38IO_INPUT13_RANGE,
	T38IO_INPUT14_RANGE,
	T38IO_INPUT15_RANGE,
	T38IO_INPUT16_RANGE,

	T38IO_INPUT17_RANGE,	//244
	T38IO_INPUT18_RANGE,
	T38IO_INPUT19_RANGE,
	T38IO_INPUT20_RANGE,
	T38IO_INPUT21_RANGE,
	T38IO_INPUT22_RANGE,
	T38IO_INPUT23_RANGE,
	T38IO_INPUT24_RANGE,

	T38IO_INPUT25_RANGE,	 //252
	T38IO_INPUT26_RANGE,
	T38IO_INPUT27_RANGE,
	T38IO_INPUT28_RANGE,
	T38IO_INPUT29_RANGE,
	T38IO_INPUT30_RANGE,
	T38IO_INPUT31_RANGE,
	T38IO_INPUT32_RANGE,
	
	T38IO_INPUT1_FILTER,	  //260
	T38IO_INPUT2_FILTER,
	T38IO_INPUT3_FILTER,
	T38IO_INPUT4_FILTER,
	T38IO_INPUT5_FILTER,
	T38IO_INPUT6_FILTER,
	T38IO_INPUT7_FILTER,
	T38IO_INPUT8_FILTER,
	T38IO_INPUT9_FILTER,
	T38IO_INPUT10_FILTER,
	T38IO_INPUT11_FILTER,
	T38IO_INPUT12_FILTER,
	T38IO_INPUT13_FILTER,
	T38IO_INPUT14_FILTER,
	T38IO_INPUT15_FILTER,
	T38IO_INPUT16_FILTER,

	T38IO_INPUT17_FILTER,
	T38IO_INPUT18_FILTER,
	T38IO_INPUT19_FILTER,
	T38IO_INPUT20_FILTER,
	T38IO_INPUT21_FILTER,
	T38IO_INPUT22_FILTER,
	T38IO_INPUT23_FILTER,
	T38IO_INPUT24_FILTER,
	T38IO_INPUT25_FILTER,
	T38IO_INPUT26_FILTER,
	T38IO_INPUT27_FILTER,
	T38IO_INPUT28_FILTER,
	T38IO_INPUT29_FILTER,
	T38IO_INPUT30_FILTER,
	T38IO_INPUT31_FILTER,
	T38IO_INPUT32_FILTER,
	T38IO_RESET	  = 300				,
	

	
	
};




// --------------- 8 inputs 16 outputs module -----------------------------------------------------
// ------------------------------------------------------------------------------------------------

#elif defined (T3_8IN16OUT)


//  REGISTER ADDRESSES TO BE USED IN MODBUS SERIAL COMMUNICATION
enum {
	T38I16O_OUTPUT1 = ORIGINALADDRESSVALUE,  		// Output 1, 	register 100
	T38I16O_OUTPUT2  ,                   			// Output 2 
	T38I16O_OUTPUT3  ,                   			// Output 3 
	T38I16O_OUTPUT4  ,                   			// Output 4 
	T38I16O_OUTPUT5  ,                   			// Output 5 
	T38I16O_OUTPUT6  ,                   			// Output 6 
	T38I16O_OUTPUT7  ,                   			// Output 7 
	T38I16O_OUTPUT8  ,                   			// Output 8
 
	T38I16O_INPUT1  ,                   			// Input 1, 	register 108 
	T38I16O_INPUT2  ,                   			// Input 2 
	T38I16O_INPUT3  ,                   			// Input 3 
	T38I16O_INPUT4  ,                   			// Input 4 
	T38I16O_INPUT5  ,                   			// Input 5 
	T38I16O_INPUT6  ,                   			// Input 6 
	T38I16O_INPUT7  ,                   			// Input 7 
	T38I16O_INPUT8  ,                   			// Input 8

	T38I16O_OUTPUT9 ,						  		// Output 9, 	register 116
	T38I16O_OUTPUT10 ,                   			// Output 10 
	T38I16O_OUTPUT11 ,                   			// Output 11 
	T38I16O_OUTPUT12 ,                   			// Output 12 
	T38I16O_OUTPUT13 ,                   			// Output 13 
	T38I16O_OUTPUT14 ,                   			// Output 14 
	T38I16O_OUTPUT15 ,                   			// Output 15 
	T38I16O_OUTPUT16 ,                   			// Output 16

  	T38I16O_SWITCH_STATE1,						// First bank of switches, register 124
  	T38I16O_SWITCH_STATE2,						// Second bank of switches

  	T38I16O_IN_COUNT_HI,
  	T38I16O_IN_COUNT_LO,

	T38IO_INPUT1_RANGE,					//128	Set range for each input. 0 = raw data,1 = 10K Celsius,2 = 10K Fahrenheit			  
	T38IO_INPUT2_RANGE,					//		3 = 0 - 100%,4 = ON/OFF,5 = OFF/ON
	T38IO_INPUT3_RANGE,
	T38IO_INPUT4_RANGE,	
	T38IO_INPUT5_RANGE,
	T38IO_INPUT6_RANGE,
	T38IO_INPUT7_RANGE,
	T38IO_INPUT8_RANGE,
	
	T38IO_INPUT1_FILTER,
	T38IO_INPUT2_FILTER,
	T38IO_INPUT3_FILTER,
	T38IO_INPUT4_FILTER,
	T38IO_INPUT5_FILTER,
	T38IO_INPUT6_FILTER,
	T38IO_INPUT7_FILTER,
	T38IO_INPUT8_FILTER,



	T38IO_CHANNEL_TYPE


};

// --------------- PWM transducer -------------------------------------------------------
// --------------------------------------------------------------------------------------

#elif defined (PWM_TRANSDUCER)

typedef /*idata*/ enum 
{

  EEP_OUTPUT1 = T38IO_OUTPUT1 - ORIGINALADDRESSVALUE,  				// 00 Output 0 
  EEP_OUTPUT2 = T38IO_OUTPUT2 - ORIGINALADDRESSVALUE,				// 01 Output 1 
  EEP_OUTPUT3 = T38IO_OUTPUT3 - ORIGINALADDRESSVALUE,  				// 02 Output 2 
  EEP_OUTPUT4 = T38IO_OUTPUT4 - ORIGINALADDRESSVALUE,				// 03 Output 3 
  EEP_OUTPUT5 = T38IO_OUTPUT5 - ORIGINALADDRESSVALUE,				// 04 Output 4 
  EEP_OUTPUT6 = T38IO_OUTPUT6 - ORIGINALADDRESSVALUE,      			// 05 Output 5 
  EEP_OUTPUT7 = T38IO_OUTPUT7 - ORIGINALADDRESSVALUE,  				// 06 Output 6 
  EEP_OUTPUT8 = T38IO_OUTPUT8 - ORIGINALADDRESSVALUE,      			// 07 Output 7 

  EEP_INPUT1 = T38IO_INPUT1 - ORIGINALADDRESSVALUE,  	   			// 08 Input 0 
  EEP_INPUT2 = T38IO_INPUT2 - ORIGINALADDRESSVALUE, 				// 09 Input 1 
  EEP_INPUT3 = T38IO_INPUT3 - ORIGINALADDRESSVALUE,  				// 10 Input 2 
  EEP_INPUT4 = T38IO_INPUT4 - ORIGINALADDRESSVALUE,					// 11 Input 3 
  EEP_INPUT5 = T38IO_INPUT5 - ORIGINALADDRESSVALUE,					// 12 Input 4 
  EEP_INPUT6 = T38IO_INPUT6 - ORIGINALADDRESSVALUE,      			// 13 Input 5 
  EEP_INPUT7 = T38IO_INPUT7 - ORIGINALADDRESSVALUE,  				// 14 Input 6 
  EEP_INPUT8 = T38IO_INPUT8 - ORIGINALADDRESSVALUE,      			// 15 Input 7 

  EEP_INPUT1_TIMER,	   			// 08 Input 0 
  EEP_INPUT2_TIMER,				// 09 Input 1 
  EEP_INPUT3_TIMER,				// 10 Input 2 
  EEP_INPUT4_TIMER,				// 11 Input 3 
  EEP_INPUT5_TIMER,				// 12 Input 4 
  EEP_INPUT6_TIMER,    			// 13 Input 5 
  EEP_INPUT7_TIMER,				// 14 Input 6 
  EEP_INPUT8_TIMER,    			// 15 Input 7 

  EEP_INPUT1_RANGE,					//124	Set range for each input. 0 = raw data,1 = 10K Celsius,2 = 10K Fahrenheit			  
  EEP_INPUT2_RANGE,					//		3 = 0 - 100%,4 = ON/OFF,5 = OFF/ON
  EEP_INPUT3_RANGE,
  EEP_INPUT4_RANGE,	
  EEP_INPUT5_RANGE,
  EEP_INPUT6_RANGE,
  EEP_INPUT7_RANGE,
  EEP_INPUT8_RANGE,




  TOTAL_EE_PARAMETERS         //  18

  } et_menu_parameter ;


// --------------- By default 8 inputs module -----------------------------------------------------
// ------------------------------------------------------------------------------------------------


#else
//  REGISTER ADDRESSES TO BE USED IN MODBUS SERIAL COMMUNICATION
enum {
	T38IO_OUTPUT1 = ORIGINALADDRESSVALUE,  		// Output 1, register 100
	T38IO_OUTPUT2  ,                   			// Output 2 
	T38IO_OUTPUT3  ,                   			// Output 3 
	T38IO_OUTPUT4  ,                   			// Output 4 
	T38IO_OUTPUT5  ,                   			// Output 5 
	T38IO_OUTPUT6  ,                   			// Output 6 
	T38IO_OUTPUT7  ,                   			// Output 7 
	T38IO_OUTPUT8  ,                   			// Output 8
 
	T38IO_INPUT1  ,                   			// Input 1, register 108
	T38IO_INPUT2  ,                   			// Input 2 
	T38IO_INPUT3  ,                   			// Input 3 
	T38IO_INPUT4  ,                   			// Input 4 
	T38IO_INPUT5  ,                   			// Input 5 
	T38IO_INPUT6  ,                   			// Input 6 
	T38IO_INPUT7  ,                   			// Input 7 
	T38IO_INPUT8  ,                   			// Input 8


  	T38IO_SWITCH_STATE1,						// First bank of switches, register 116
  	T38IO_SWITCH_STATE2,						// Second bank of switches

	// Use a long type data to store pulse number,and the number be divided two integer data 
	T38IO_PULSE1_HI_WORD,						// register 118
	T38IO_PULSE1_LO_WORD,						// register 119

	T38IO_PULSE2_HI_WORD, 
	T38IO_PULSE2_LO_WORD,

	T38IO_PULSE3_HI_WORD, 
	T38IO_PULSE3_LO_WORD,

	T38IO_PULSE4_HI_WORD, 
	T38IO_PULSE4_LO_WORD,

	T38IO_PULSE5_HI_WORD, 
	T38IO_PULSE5_LO_WORD,

	T38IO_PULSE6_HI_WORD, 
	T38IO_PULSE6_LO_WORD,

	T38IO_PULSE7_HI_WORD, 
	T38IO_PULSE7_LO_WORD,

	T38IO_PULSE8_HI_WORD, 
	T38IO_PULSE8_LO_WORD,				// register 133

	T38IO_PULSE1_YEAR,					//134	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE1_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE1_DATE,
	T38IO_PULSE1_HOUR,
	T38IO_PULSE1_MINUTE,
 
	T38IO_PULSE2_YEAR,					//139	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE2_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE2_DATE,
	T38IO_PULSE2_HOUR,
	T38IO_PULSE2_MINUTE,	
 
	T38IO_PULSE3_YEAR,					//144	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE3_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE3_DATE,
	T38IO_PULSE3_HOUR,
	T38IO_PULSE3_MINUTE,
 
	T38IO_PULSE4_YEAR,					//149	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE4_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE4_DATE,
	T38IO_PULSE4_HOUR,
	T38IO_PULSE4_MINUTE,
 
	T38IO_PULSE5_YEAR,					//154	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE5_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE5_DATE,
	T38IO_PULSE5_HOUR,
	T38IO_PULSE5_MINUTE,	
 
	T38IO_PULSE6_YEAR,					//159	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE6_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE6_DATE,
	T38IO_PULSE6_HOUR,
	T38IO_PULSE6_MINUTE,
 
	T38IO_PULSE7_YEAR,					//164	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE7_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE7_DATE,
	T38IO_PULSE7_HOUR,
	T38IO_PULSE7_MINUTE,
 
	T38IO_PULSE8_YEAR,					//169	//the pulse number is cleared as soon as this register be wrote
	T38IO_PULSE8_MONTH,							//not clear pulse number when write the following 4 registers
	T38IO_PULSE8_DATE,
	T38IO_PULSE8_HOUR,
	T38IO_PULSE8_MINUTE,

	T38IO_CHANNEL_TYPE,					//174   //USE 8 bits in one byte represent each channel type.LST is channel0 and 1 means digital,0 means digital 

				
	T38IO_INPUT1_READING,				//175	analog reading for channel 1 through 8,whatever the channel be set analog or high speeed pulse mode		
	T38IO_INPUT2_READING,
	T38IO_INPUT3_READING,
	T38IO_INPUT4_READING,
	
	T38IO_INPUT5_READING,
	T38IO_INPUT6_READING,
	T38IO_INPUT7_READING,
	T38IO_INPUT8_READING,

	T38IO_INPUT1_RANGE,					//183	Set range for each input. 0 = raw data,1 = 10K Celsius,2 = 10K Fahrenheit			  
	T38IO_INPUT2_RANGE,					//		3 = 0 - 100%,4 = ON/OFF,5 = OFF/ON
	T38IO_INPUT3_RANGE,
	T38IO_INPUT4_RANGE,	
	T38IO_INPUT5_RANGE,
	T38IO_INPUT6_RANGE,
	T38IO_INPUT7_RANGE,
	T38IO_INPUT8_RANGE,

	T38IO_INPUT1_FILTER,				//191  Filter for all inputs,0 through 100.
	T38IO_INPUT2_FILTER,
	T38IO_INPUT3_FILTER,
	T38IO_INPUT4_FILTER,
	T38IO_INPUT5_FILTER,
	T38IO_INPUT6_FILTER,
	T38IO_INPUT7_FILTER,
	T38IO_INPUT8_FILTER,
	T38IO_INPUT1_TIMER,					//199  Timer,how long time the lighting control take over the outputs
	T38IO_INPUT2_TIMER,
	T38IO_INPUT3_TIMER,
	T38IO_INPUT4_TIMER,
	T38IO_INPUT5_TIMER,
	T38IO_INPUT6_TIMER,
	T38IO_INPUT7_TIMER,
	T38IO_INPUT8_TIMER,
	T38IO_INPUT1_TIMER_LEFT,			//207  Timer Left,how much time left for the lighting control
	T38IO_INPUT2_TIMER_LEFT,
	T38IO_INPUT3_TIMER_LEFT,
	T38IO_INPUT4_TIMER_LEFT,
	T38IO_INPUT5_TIMER_LEFT,
	T38IO_INPUT6_TIMER_LEFT,
	T38IO_INPUT7_TIMER_LEFT,
	T38IO_INPUT8_TIMER_LEFT,
	T38IO_OUTPUT_MANUAL,				//215 light control disable/enable ,each bit correspond to one output
										//	  output1 correspond to least significant bit,0 = disable,1 = enable
	T38IO_ZONE_OUTPUT1,
	T38IO_ZONE_OUTPUT2,
	T38IO_ZONE_OUTPUT3,
	T38IO_ZONE_OUTPUT4,
	T38IO_ZONE_OUTPUT5,
	T38IO_ZONE_OUTPUT6,
	T38IO_ZONE_OUTPUT7,
	T38IO_ZONE_OUTPUT8,
	T38IO_ZONE_OUTPUT9,
	T38IO_ZONE_OUTPUT10,
	T38IO_ZONE_OUTPUT11,
	T38IO_ZONE_OUTPUT12,
	T38IO_ZONE_OUTPUT13,
	
	T38IO_REVERSE_OUTPUT,				// 229 in override mode, reverse relay output

	T38IO_STATUS_OUTPUT1,				// 230 status for each output to show if the relay output working on override mode
	T38IO_STATUS_OUTPUT2,
	T38IO_STATUS_OUTPUT3,
	T38IO_STATUS_OUTPUT4,
	T38IO_STATUS_OUTPUT5,
	T38IO_STATUS_OUTPUT6,
	T38IO_STATUS_OUTPUT7,
	T38IO_STATUS_OUTPUT8,
	T38IO_STATUS_OUTPUT9,
	T38IO_STATUS_OUTPUT10,
	T38IO_STATUS_OUTPUT11,
	T38IO_STATUS_OUTPUT12,
	T38IO_STATUS_OUTPUT13,
	T38IO_CALIBRATION_OFFSET0,
	T38IO_CALIBRATION_OFFSET1 = T38IO_CALIBRATION_OFFSET0 +	NUM_INPUTS,
	T38IO_FLASH_WRITE_TIME ,
	
	#ifdef T3_8IO_A
	T38IO_DeadMaster_Ao0 ,
	T38IO_DeadMaster_Ao1 ,
	T38IO_DeadMaster_Ao2 ,
	T38IO_DeadMaster_Ao3 ,
	T38IO_DeadMaster_Ao4 ,
	T38IO_DeadMaster_Ao5 ,
	T38IO_DeadMaster_Ao6 ,
	T38IO_DeadMaster_Ao7 ,
	#endif
/*	TT1,  // for test registers added by chelsea
	TT2,
	TT3,
	TT4,
	TT5,
	TT6,
	TT7,
	TT8*/

/*	T38IO_INPUT9_RANGE,
	T38IO_INPUT10_RANGE,
	T38IO_INPUT11_RANGE,
	T38IO_INPUT12_RANGE,
	T38IO_INPUT13_RANGE,
	T38IO_INPUT14_RANGE,
	T38IO_INPUT15_RANGE,
	T38IO_INPUT16_RANGE,

	T38IO_INPUT17_RANGE,
	T38IO_INPUT18_RANGE,
	T38IO_INPUT19_RANGE,
	T38IO_INPUT20_RANGE,
	T38IO_INPUT21_RANGE,
	T38IO_INPUT22_RANGE,
	T38IO_INPUT23_RANGE,
	T38IO_INPUT24_RANGE,

	T38IO_INPUT25_RANGE,
	T38IO_INPUT26_RANGE,
	T38IO_INPUT27_RANGE,
	T38IO_INPUT28_RANGE,
	T38IO_INPUT29_RANGE,
	T38IO_INPUT30_RANGE,
	T38IO_INPUT31_RANGE,
	T38IO_INPUT32_RANGE*/

};


#endif

