// to select proper module comment-in the hardware model and comment-out the rest
// by default it is set to T3_8IO_REV8 given it is the most recent basic board

//****************tstatio.h************************************

// --- SEQUENCE OF OPERATION ----------------------------------
//	write to FLASH_SEQUENCE register in modbus
//		0 = testing sequence for the 8IO module ONLY
//		1 = normal operation
//		2 = Timer_SOP feature added for input one of 8IO module
//				will control all TSTAT on network
//				*** thinking of removing this given not used anymore ***
//		3 = operation where the T3-modules do not have switches, thus always automatic mode
//			will respond according to output register values


// *************************************************************

// important notes... timers must be changed according to different configuration boards...
//	for instance, 32inputs must go through inputs much faster rate than for 8 inputs...

// ---Analog T3 8Input 8 analog Output board
#define T3_8IO_A			0x0808



// ---Digital T3 8In 13 relay Out Board
//#define T3_8IO			0x0808	// for hardware rev 5 and below
//#define T3_8IO_REV6		0x0808	// for hardware rev6
//#define T3_8IO_REV8		0x0808	// for hardware rev8 and above
//#define T3_8IN13OUT			0x0108 // HARDWARE MODEL 





// ---T3 8In 16Out board
//#define T3_8IN16OUT		0x0810
	// reminder for REX boards...
	//			- No Pull-ups on the board
	//			- No Switches required
	//			- Fast reactive code for calibration.  incraesed readings and reduce filtering



// ---T3 32Input board
//#define T3_32IN			0x2000  
	// for given 32IN requires a lot more ram, cannot use the calibration option simultaneously
	// must set REFRESH_INPUTS ticks to 1 for fast reaction

// ---PWM transducer board
//#define PWM_TRANSDUCER



// -----------------------------------------------------------------
// below are special features which can be applied to different boards

//#define BENNY_FEATURES
//#define REX_FEATURES
//#define FLEXDRIVER_FEATURE
//#define TIMER_SOP_FEATURE
//#define CALIBRATION_OPTION




// -----------------------------------------------------------------
// below are special defines for software characteristics
#define T3_SOFTWARE_VERSION		76

#define FW_VER_LOW  	T3_SOFTWARE_VERSION & 0xff
#define FW_VER_HIGH  	(T3_SOFTWARE_VERSION >> 8) & 0xff


////////////////////////////////////////////////////////////////////
////////////////// PWM TRANSDUCER //////////////////////////////////
////////////////////////////////////////////////////////////////////
#if defined (PWM_TRANSDUCER)

	sbit IN_1			= P0^0;
	sbit IN_2			= P0^1;
	sbit IN_3			= P0^2;
	sbit IN_4			= P0^3;
	sbit IN_5			= P0^4;
	sbit IN_6			= P0^5;

	sbit MUX_OFF1		= P1^0;
	sbit MUX_OFF2		= P1^1;
	sbit CARDA0			= P1^2;
	sbit CARDA1			= P1^3;
	sbit CARDA2			= P1^4;

	sbit PWM_OUTPUT		= P1^6;

	sbit AD0			= P2^0;
	sbit AD1			= P2^1;
	sbit AD2			= P2^2;
	sbit AD3			= P2^3;
	sbit AD4			= P2^4;
	sbit AD5			= P2^5;
	sbit AD6			= P2^6;
	sbit AD7			= P2^7;

	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
	sbit RS232STATE		= P3^2;

	sbit E2CLOCK		= P3^4;
	sbit E2DATA			= P3^5;
	sbit LED_DRIVE1		= P3^6;
	sbit LED_DRIVE2		= P3^7;	

//	sbit TESTBIT		= P4^2;	//	 P4.2 pin 1

	#define T3_PRODUCT_MODEL	31


////////////////////////////////////////////////////////////////////
/////////////////// T3 32INPUTS ////////////////////////////////////
////////////////////////////////////////////////////////////////////
#elif defined (T3_32IN)



	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;
	
	sbit CARDA0			= P1^0;
	sbit CARDA1			= P1^1;
	sbit CARDA2			= P1^3;
	sbit BANKA_ENABLE	= P1^4;
	sbit BANKB_ENABLE	= P1^5;
	
	sbit RS232STATE 	= P4^0;
	
	sbit LED_DRIVE2		= P2^0;
	sbit LED_DRIVE3		= P2^1;
	sbit LED_DRIVE5		= P2^2;

	sbit KEYPAD1_HAND	= P2^3;
	sbit KEYPAD1_AUTO	= P2^4;
	
	sbit LED_DRIVE4		= P2^5;
	sbit LED_DRIVE1		= P2^6;
	
	
	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
	
	
	
	sbit E2CLOCK		= P3^4;
	sbit E2DATA			= P3^5;
	sbit WP_MEMORY		= P3^6;
	sbit KEYPAD1_ENABLE = P3^7;

	sbit TESTBIT		= P4^3;	// ISP line

	#define T3_PRODUCT_MODEL	22

////////////////////////////////////////////////////////////////////
//////////////// T3 8INPUT 16 OUTPUTS //////////////////////////////
////////////////////////////////////////////////////////////////////
#elif defined (T3_8IN16OUT)



/*
	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;
	
	sbit CARDA2			= P1^0;
	sbit CARDA1			= P1^1;	
	sbit CARDA0			= P1^2;
	sbit MUX_OFF1		= P1^3;
	sbit KEYPAD1_HAND	= P1^4;
	sbit KEYPAD1_AUTO	= P1^5;
	sbit KEYPAD2_HAND	= P1^6;
	sbit KEYPAD2_AUTO	= P1^7;

	sbit WRITEPORT2		= P2^0;
	sbit WRITEPORT1		= P2^1;
	sbit ZERO_CROSSING	= P2^3;
	sbit LOW_VOLTAGE	= P2^4;
	sbit ENABLE_2003	= P2^5;
	sbit E2CLOCK		= P2^6;
	sbit E2DATA			= P2^7;


	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
	sbit LED_DRIVE4		= P3^2;
	sbit KEYPAD2_ENABLE = P3^3;
	sbit KEYPAD1_ENABLE = P3^4;
	sbit LED_DRIVE2		= P3^5;
	sbit LED_DRIVE3		= P3^6;
	sbit LED_DRIVE1		= P3^7;	

	sbit RS232STATE 	= P4^0;
	sbit ENABLE12V		= P4^1;
*/

// --- rev02 ---------------------

	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;
	
	sbit CARDA2			= P1^0;
	sbit CARDA1			= P1^1;	
	sbit CARDA0			= P1^2;
	sbit MUX_OFF1		= P1^3;
	sbit KEYPAD1_HAND	= P1^4;
	sbit KEYPAD1_AUTO	= P1^5;
	sbit KEYPAD2_HAND	= P1^6;
	sbit KEYPAD2_AUTO	= P1^7;

	sbit WRITEPORT2		= P2^0;
	sbit WRITEPORT1		= P2^1;
	sbit LED_DRIVE4		= P2^2;
	sbit ZERO_CROSSING	= P2^3;
	sbit LOW_VOLTAGE	= P2^4;
	sbit ENABLE_2003	= P2^5;
	sbit E2CLOCK		= P2^6;
	sbit E2DATA			= P2^7;

	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
	sbit HIGH_SPEED_IN	= P3^2;
	sbit KEYPAD2_ENABLE = P3^3;
	sbit KEYPAD1_ENABLE = P3^4;
	sbit LED_DRIVE2		= P3^5;
	sbit LED_DRIVE3		= P3^6;
	sbit LED_DRIVE1		= P3^7;	

	sbit RS232STATE 	= P4^0;
	sbit ENABLE12V		= P4^1;

// -------------------------------------------

/*	rev00

	
	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;
	
	
	sbit MUX_OFF1		= P1^0;
	sbit CARDA2			= P1^1;
	sbit RS232STATE 	= P1^2;
	sbit CARDA0			= P1^3;
	sbit WRITEPORT1		= P1^4;
	sbit WRITEPORT2		= P1^5;
	sbit spare1			= P1^6;
	sbit LED_DRIVE4		= P1^7;
	
	
	sbit LED_DRIVE2		= P2^0;
	sbit LED_DRIVE3		= P2^1;
	sbit LED_DRIVE1		= P2^2;
	sbit KEYPAD1_HAND	= P2^3;
	sbit KEYPAD1_AUTO	= P2^4;
	sbit KEYPAD2_HAND	= P2^5;
	sbit KEYPAD2_AUTO	= P2^6;
	sbit spare2			= P2^7;
	
	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
	sbit CARDA1			= P3^2;
	sbit KEYPAD2_ENABLE = P3^3;
	sbit E2CLOCK		= P3^4;
	sbit E2DATA			= P3^5;
	
	sbit KEYPAD1_ENABLE = P3^7;
	
	sbit ENABLE12V		= P4^1;

*/


	#define T3_PRODUCT_MODEL	23

	
////////////////////////////////////////////////////////////////////
////////////// T3 8IO ANALOG ///////////////////////////////////////
////////////////////////////////////////////////////////////////////
#elif defined (T3_8IO_A)


	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;

	sbit CARDA0			= P1^0;
	sbit CARDA1			= P1^1;
	sbit CARDA2			= P1^2;
	sbit MUX_OFF1		= P1^3;
	sbit KEYPAD1_HAND	= P1^4;
	sbit KEYPAD1_AUTO	= P1^5;
	
	sbit ENABLE_2003	= P1^7;

	sbit RELAY1			= P2^0;
	sbit RELAY2			= P2^1;
	sbit RELAY3			= P2^2;
	sbit RELAY4			= P2^3;
	sbit RELAY5			= P2^4;
	sbit RELAY6			= P2^5;
	sbit RELAY7			= P2^6;
	sbit RELAY8			= P2^7;

	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
	sbit MUX_OFF2		= P3^2;
	sbit ENABLE12V		= P3^3;
	sbit KEYPAD1_ENABLE = P3^4;
	sbit LED_DRIVE2		= P3^5;	// STATUS LED
	sbit LED_DRIVE3		= P3^6;
	sbit LED_DRIVE1		= P3^7;	

	sbit RS232STATE 	= P4^0;
	sbit E2CLOCK		= P4^1;
	sbit E2DATA			= P4^2;

	#define T3_PRODUCT_MODEL	21

////////////////////////////////////////////////////////////////////
////////////////// T3 8IO ///BEFORE REV6////////////////////////////
////////////////////////////////////////////////////////////////////
#elif defined (T3_8IO)
	
	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;
	
	sbit MUX_OFF1		= P1^0;
	sbit CARDA2			= P1^1;
	sbit RS232STATE 	= P1^2;
	sbit CARDA0			= P1^3;
	sbit MUX_OFF2		= P1^4;

	sbit ENABLE12V		= P1^7;	// here for compiling purposes... not actually used
	
	sbit LED_DRIVE2		= P2^0;
	sbit LED_DRIVE3		= P2^1;	// STATUS LED
	sbit LED_DRIVE1		= P2^2;	// BANK1
	sbit KEYPAD1_HAND	= P2^3;
	sbit KEYPAD1_AUTO	= P2^4;
//	sbit KEYPAD2_HAND	= P2^5;
//	sbit KEYPAD2_AUTO	= P2^6;
	
	sbit ENABLE_2003	= P2^7;
	
	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
	sbit CARDA1			= P3^2;
//	sbit KEYPAD2_ENABLE = P3^3;
	sbit E2CLOCK		= P3^4;
	sbit E2DATA			= P3^5;
	sbit WP_MEMORY		= P3^6;
	sbit KEYPAD1_ENABLE = P3^7;
	
	
	// previous relay setting, relay was switched so that when switch 1 it turns relay 8
	sbit RELAY1			= P1^4;
	sbit RELAY2			= P1^5;
	sbit RELAY3			= P1^6;
	sbit RELAY4			= P1^7;
	
	sbit RELAY5			= P4^0;
	sbit RELAY6			= P4^1;
	sbit RELAY7			= P4^2;
	sbit RELAY8			= P4^3;

	#define T3_PRODUCT_MODEL	20


////////////////////////////////////////////////////////////////////
////////////////// T3 8IO /// REV6 - REV7 //////////////////////////
////////////////////////////////////////////////////////////////////
#elif defined (T3_8IO_REV6)

	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;

	sbit CARDA0			= P1^0;
	sbit CARDA1			= P1^1;
	sbit CARDA2			= P1^2;
	sbit MUX_OFF1		= P1^3;
	sbit KEYPAD1_HAND	= P1^4;
	sbit KEYPAD1_AUTO	= P1^5;

	sbit RELAY1			= P2^0;
	sbit RELAY2			= P2^1;
	sbit RELAY3			= P2^2;
	sbit RELAY4			= P2^3;
	sbit RELAY5			= P2^4;
	sbit RELAY6			= P2^5;
	sbit RELAY7			= P2^6;
	sbit RELAY8			= P2^7;


	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
//	sbit TESTBIT		= P3^2;
	sbit MUX_OFF2		= P3^3;
	sbit KEYPAD2_ENABLE = P3^3;
	sbit ENABLE12V		= P3^3;
	sbit KEYPAD1_ENABLE = P3^4;
	sbit LED_DRIVE2		= P3^5;	// STATUS LED
	sbit LED_DRIVE3		= P3^6;
	sbit LED_DRIVE1		= P3^7;	

	sbit RS232STATE 	= P4^0;
	sbit ENABLE_2003	= P4^1;
	sbit E2CLOCK		= P4^2;
	sbit E2DATA			= P4^3;



	#define T3_PRODUCT_MODEL	20
#elif defined (T3_8IN13OUT) 

	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;

	sbit CARDA0			= P1^0;
	sbit CARDA1			= P1^1;
	sbit CARDA2			= P1^2;
	sbit MUX_OFF1		= P1^3;
	sbit KEYPAD1_HAND	= P1^4;
	sbit KEYPAD1_AUTO	= P1^5;

	sbit RELAY1			= P2^0;
	sbit RELAY2			= P2^1;
	sbit RELAY3			= P2^2;
	sbit RELAY4			= P2^3;
	sbit RELAY5			= P2^4;
	sbit RELAY6			= P2^5;
	sbit RELAY7			= P2^6;
	sbit RELAY8			= P2^7;
	sbit RELAY9			= P1^0;
	sbit RELAY10			= P1^1;
	sbit RELAY11			= P1^2;
	sbit RELAY12			= P1^3;
	sbit RELAY13			= P4^3;


	sbit RX2				 = P3^0;
	sbit TX2				 = P3^1;
 	sbit MUX_OFF2		= P3^3;
	sbit KEYPAD2_ENABLE = P3^2;	//CHANGED BY KANSON
	sbit ENABLE12V		= P3^3;
	sbit KEYPAD1_ENABLE = P3^4;
	sbit KEYPAD2_HAND    = P1^4;
	sbit KEYPAD2_AUTO	= P1^5;
	sbit LED_DRIVE2		= P3^5;	// STATUS LED
	sbit LED_DRIVE3		= P3^6;	
	sbit LED_DRIVE1		= P3^7;	
	sbit LED_DRIVE4 		= P4^2;	//ADD BY KANSON

	sbit RS232STATE 		 = P4^0;
	sbit ENABLE_2003	= P4^1;
	sbit E2CLOCK			= P1^6;
	sbit E2DATA			= P1^7;
	#define T3_PRODUCT_MODEL	20
////////////////////////////////////////////////////////////////////
////////////////// DEFAULT T3 8IO DIGITAL REV8 AND ABOVE ///////////
////////////////////////////////////////////////////////////////////
#else



	sbit AD0			= P0^0;
	sbit AD1			= P0^1;
	sbit AD2			= P0^2;
	sbit AD3			= P0^3;
	sbit AD4			= P0^4;
	sbit AD5			= P0^5;
	sbit AD6			= P0^6;
	sbit AD7			= P0^7;

	sbit CARDA0			= P1^0;
	sbit CARDA1			= P1^1;
	sbit CARDA2			= P1^2;
	sbit MUX_OFF1		= P1^3;
	sbit KEYPAD1_HAND	= P1^4;
	sbit KEYPAD1_AUTO	= P1^5;

	sbit RELAY1			= P2^0;
	sbit RELAY2			= P2^1;
	sbit RELAY3			= P2^2;
	sbit RELAY4			= P2^3;
	sbit RELAY5			= P2^4;
	sbit RELAY6			= P2^5;
	sbit RELAY7			= P2^6;
	sbit RELAY8			= P2^7;

	sbit RX2			= P3^0;
	sbit TX2			= P3^1;
//	sbit TESTBIT		= P3^2;
	sbit MUX_OFF2		= P3^3;
	sbit KEYPAD2_ENABLE = P3^3;
	sbit ENABLE12V		= P3^3;
	sbit KEYPAD1_ENABLE = P3^4;
	sbit LED_DRIVE2		= P3^5;	// STATUS LED
	sbit LED_DRIVE3		= P3^6;
	sbit LED_DRIVE1		= P3^7;	

	sbit RS232STATE 	= P4^0;
	sbit ENABLE_2003	= P4^1;

#if defined  T3_8IN13OUT  ////MHF 20010_07 COMBINE TWO IFDEFS TO ONE
	sbit E2CLOCK		= P1^6;
	sbit E2DATA			= P1^7;
#else

	sbit E2CLOCK		= P4^2;
	sbit E2DATA			= P4^3;

//	sbit E2CLOCK		= P1^6;
//	sbit E2DATA			= P1^7;

#endif

//sbit TESTBIT5 = P4^3;


	#define T3_PRODUCT_MODEL	20





#endif



















// -----------------------------------------------------------------
// -----------------------------------------------------------------

//----------Hardware definitions ---------------------
