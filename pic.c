#include <KEIL_VRS1000_regs.h>
#include "LibIO_T3IO.h"
#include "define.h"
#include "pic.h"
#include "math.h"		// to use the abs function

#if defined (T3_32IN) 
#define MAX_CHANNEL  32
#define CHANNEL_7	 7
#define CHANNEL_16	 16
#define CHANNEL_23	 23
#define CHANNEL_31	 31

#elif defined (PWM_TRANSDUCER)
#define MAX_CHANNEL  6
#else
#define MAX_CHANNEL  8
#endif

#define UNUSED 10

//#define 	CRC     1
 
//extern bit flag_comm;

unsigned int CRC16(unsigned char *puchMsg, unsigned char usDataLen);
unsigned char xdata PIC_CRChi;
unsigned char xdata PIC_CRClo;
extern unsigned char const code auchCRCHi[256];
extern unsigned char const code auchCRCLo[256];
extern signed int RangeConverter(unsigned char function, signed int para,signed int cal,unsigned char i);
extern int xdata calibration_offset[NUM_INPUTS];
signed int xdata guiAnalogInput[NUM_INPUTS],guiAnalogInput_temp[NUM_INPUTS];
#ifndef T3_32IN
unsigned int  xdata guiBuffer[8][10];
unsigned char xdata gucCounter[8];
unsigned char xdata gucStartFilter;
extern unsigned char xdata range[8];
extern unsigned char xdata gucPreviousInput[8];
unsigned int xdata FilterTemp[8];
#else
extern unsigned char xdata range[32];
#endif
bit gbAnotherProtocol = 0;
unsigned char xdata PICV;
//extern unsigned int xdata T,TT,TTT;
// *****************************************************************************************************
// pic detection routine
//	determine which pic is being used in the T3 modules
//		- old PIC
//		- PIC with interrupt
//		- PIC with 8 AD inputs
// 
// *****************************************************************************************************
void pic_detect(void)
{
	unsigned char pic_trial = 0;	
	unsigned char temp;
	// default mode is no pic.  If one of the responses below works, then we know there is pic present
	// possibility that pic had missed a ack.  
	// Thus if we have pic_type0 we would then call this multiple times until we would detect a pic...

	// first call original pic code
	// if obtain response then type is 1
	pic_type = 1;
	if( read_pic_original(EEP_INPUT1, HI) )
		pic_trial = 1;
	
	// second call interrupt pic code
	// as well changed start condition to comply with Tstat
	// if obtain response then type is 2
	pic_type = 2;
	if(read_pic())
		pic_trial = 2;
	
	// third call pic with 8AD
	// if obtain response then type is 3
 	pic_type = 3;
	temp = ReadPicVersion();
	if(temp > 0)
		pic_trial = temp;


	// store the result if a pic was found
	pic_type = pic_trial;
	flash_write_int(FLASH_PIC_VERSION,pic_type, FLASH_MEMORY);	// if there was nothign in the flash, then write the pic detected
#ifdef T3_32IN
if(pic_type != 2)
{
	gbAnotherProtocol = 1;	
	pic_type = 2;
}
#endif

}

bit read_pic(void)
{

	unsigned int xdata pic_reading, checksum, checksum_verify;
	static bit Read_Twice = 0 ;

	// start the i2c routine, pic has been interrupt
	i2c_pic_start();					//PIC will stop read ADC AT THIS POINT
	// Set the Mux banks before the pic Interrupts out
	#if defined (T3_32IN)
		if(gbAnotherProtocol)
			i2c_pic_write( 0x40 + input_channel_select );
		else
		{	
			// we are now selecting the upper or lower set
			if(input_channel_select < 8)
				i2c_pic_write( 0xb5 );
			else if(input_channel_select < 16)
				i2c_pic_write( 0xb1 );
			else if(input_channel_select < 24)
				i2c_pic_write( 0xb5 );
			else if(input_channel_select < 32)
				i2c_pic_write( 0xb1 );
		}
		#elif defined (PWM_TRANSDUCER)
		i2c_pic_write( 0xb1 );
		#else
		i2c_pic_write( 0xb1 );
		
	#endif
	// ----------------------------------------------------------------------
	if (GET_ACK()){
		i2c_stop();
		return FALSE;		 
	}

	delay_us(1);	// needed for timing purposes
	reading_counter = 0;
	// read first byte
	reading_counter = i2c_pic_read();
	if(!GIVE_PIC_ACK()){
		i2c_stop(); 	  	 
		return FALSE;
	}
	delay_us(1);
	
	// read second byte
	pic_reading = i2c_pic_read();			// note we are now collecting data of previous channel
	if(!GIVE_PIC_ACK()){
		i2c_stop();
		return FALSE;     
	}

	delay_us(1);

	// read checksum
	checksum = i2c_pic_read();

	// stop the i2c routine
	i2c_stop();
 
 	if((input_channel_select == 0) ||(input_channel_select == 8)||(input_channel_select == 16)||(input_channel_select == 24))
	{
		if(Read_Twice == 0)
		{
			Read_Twice = 1;
			return TRUE;
		}
		else
		{
			Read_Twice = 0;
		}
	}

	// ----------------------------------------------------------------------
	// sort the data received
	checksum_verify = (reading_counter + pic_reading) & 0x3FF;
 
	if(pic_reading > 1023)
	return FALSE;
	if(reading_counter != pic_reading) ;
//	if(input_channel_select == EEP_INPUT1)
//	{
//		#if defined (T3_32IN)
//			if((checksum == checksum_verify)&&Read_Twice == 0)
//				store_to_registers(pic_reading, EEP_INPUT32 );	
//		#elif defined (PWM_TRANSDUCER)
//			if(checksum == checksum_verify )
//				store_to_registers(pic_reading, EEP_INPUT6 );	
//		#else
//			if(checksum == checksum_verify )
//				store_to_registers(pic_reading, EEP_INPUT8 );	
//		#endif
//	}
//	else
//	{
		if(checksum == checksum_verify)
		{
//			if(Read_Twice == 0)
			store_to_registers(pic_reading,input_channel_select);
//			else
//			{
//				Read_Twice = 0;
//				return FALSE; 
//			}	
		}	
//	}	
	// ----------------------------------------------------------------------
	// Set the next channel for the input MUX before the pic gets out of Interrupt Service Routine
//	CARDA0 = (input_channel_select) & 0x01;
//	CARDA1 = (input_channel_select) & 0x02;
//	CARDA2 = (input_channel_select) & 0x04;

//	if( input_channel_select == 0)
//	{	
//		BANKA_ENABLE = 0;
//		BANKB_ENABLE = 1;
//	}
//	else if( input_channel_select == 16)
//	{	
//		BANKA_ENABLE = 1;
//		BANKB_ENABLE = 0;
//	}

	// ----------------------------------------------------------------------
	// increment input channel	
	input_channel_select++;
//	input_channel_select = 0; 
	#if defined (T3_32IN)
		if(input_channel_select > EEP_INPUT32)
			input_channel_select = EEP_INPUT1;
		if( input_channel_select < 16)
		{	
			BANKA_ENABLE = 0;
			BANKB_ENABLE = 1;
		}
		else 
		{	
			BANKA_ENABLE = 1;
			BANKB_ENABLE = 0;
		}
	#elif defined (PWM_TRANSDUCER)
		if(input_channel_select > EEP_INPUT6)
			input_channel_select = EEP_INPUT1;
	#else
		if(input_channel_select > EEP_INPUT8)
			input_channel_select = EEP_INPUT1;
	#endif

	CARDA0 = (input_channel_select) & 0x01;
	CARDA1 = (input_channel_select) & 0x02;
	CARDA2 = (input_channel_select) & 0x04;
 

 
	// ----------------------------------------------------------------------
	
	return TRUE;	// protocol was successful
}



//
//bit read_pic(void)
//{
//
//	unsigned int xdata pic_reading, checksum, checksum_verify;
//	//input_channel_select = 7 ;
//	// start the i2c routine, pic has been interrupt	
//	unsigned char xdata ctr = 0;
//
////	while(1)
////	{
////		if(range[input_channel_select] == UNUSED)
////		{
////			input_channel_select ++ ;
////			#if defined (T3_32IN)			
////			if(input_channel_select > EEP_INPUT32)
////			    input_channel_select = EEP_INPUT1 ;
////			#elif defined (PWM_TRANSDUCER)
////			if(input_channel_select > EEP_INPUT6)
////				input_channel_select = EEP_INPUT1;
////			#else
////			if(input_channel_select > EEP_INPUT8)
////				input_channel_select = EEP_INPUT1;
////			#endif
////			ctr++;
////			if(ctr > (MAX_CHANNEL - 1))
////				return FALSE;
////			else
////				continue;
////		}
////		break;
////	}
//	i2c_pic_start();					//PIC will stop read ADC AT THIS POINT
////	CARDA0 = (input_channel_select) & 0x01;
////	CARDA1 = (input_channel_select) & 0x02;
////	CARDA2 = (input_channel_select) & 0x04;
	// Set the Mux banks before the pic Interrupts out
//	#if defined (T3_32IN)
////		if( input_channel_select == 0)
////		{	BANKA_ENABLE = 0;
////			BANKB_ENABLE = 1;
////		}
////		else if( input_channel_select == 16)
////		{	BANKA_ENABLE = 1;
////			BANKB_ENABLE = 0;
////		}
//
//
//		if(gbAnotherProtocol)
//			i2c_pic_write( 0x40 + input_channel_select );
//		else
//		{	
//			// we are now selecting the upper or lower set
//			if(input_channel_select < 8)
//				i2c_pic_write( 0xb5 );
//			else if(input_channel_select < 16)
//				i2c_pic_write( 0xb1 );
//			else if(input_channel_select < 24)
//				i2c_pic_write( 0xb5 );
//			else if(input_channel_select < 32)
//				i2c_pic_write( 0xb1 );
//		}
//	
//			#elif defined (PWM_TRANSDUCER)
//			i2c_pic_write( 0xb1 );
//	
//			#else
//			i2c_pic_write( 0xb1 );
//		
//	#endif
//	// ----------------------------------------------------------------------
//
//	if (GET_ACK()){
//		i2c_stop();
//		return FALSE;		 
//	}
//
//	delay_us(1);	// needed for timing purposes
//	reading_counter = 0;
//	// read first byte
//	reading_counter = i2c_pic_read();
//	if(!GIVE_PIC_ACK()){
//		i2c_stop(); 	  	 
//		return FALSE;
//	}
//	delay_us(1);
//	
//	// read second byte
//	pic_reading = i2c_pic_read();			// note we are now collecting data of previous channel
//	if(!GIVE_PIC_ACK()){
//		i2c_stop();
//		return FALSE;     
//	}
//
//	delay_us(1);
//
//	// read checksum
//	checksum = i2c_pic_read();
//
//	// stop the i2c routine
//	i2c_stop();
// 
// 
//	// ----------------------------------------------------------------------
//	// sort the data received
//	checksum_verify = (reading_counter + pic_reading) & 0x3FF;
// 
//	if(pic_reading > 1023)
//	return FALSE;
//	if(reading_counter != pic_reading) ;//	return FALSE;
//	#if defined (T3_32IN)
//	if(checksum == checksum_verify )
//	{
//		if(input_channel_select >= (ctr+1))
//		store_to_registers(pic_reading, input_channel_select - ctr -1);
//		else 
//		store_to_registers(pic_reading, EEP_INPUT32 + input_channel_select - ctr);
//	}
//	#else
//	if(input_channel_select == EEP_INPUT1)
//	{	
//		#if defined (PWM_TRANSDUCER)
//			if(checksum == checksum_verify )
//				store_to_registers(pic_reading, EEP_INPUT6 );	
//		#else
//			if(checksum == checksum_verify )
//				store_to_registers(pic_reading, EEP_INPUT8 );	
//		#endif
//	}
//	else
//	{
//		if(checksum == checksum_verify )
//			store_to_registers(pic_reading,input_channel_select-1);	
//	}
//   #endif
//	// ----------------------------------------------------------------------		
//	CARDA0 = (input_channel_select) & 0x01;
//	CARDA1 = (input_channel_select) & 0x02;
//	CARDA2 = (input_channel_select) & 0x04;
//	if( input_channel_select == 0)
//	{	
//		BANKA_ENABLE = 0;
//		BANKB_ENABLE = 1;
//	}
//	else if( input_channel_select == 16)
//	{
//		BANKA_ENABLE = 1;
//		BANKB_ENABLE = 0;
//	}
//	// increment input channel	
//	input_channel_select++;
//
//	#if defined (T3_32IN)
//		if(input_channel_select > EEP_INPUT32)
//			input_channel_select = EEP_INPUT1;
//
//	#elif defined (PWM_TRANSDUCER)
//		if(input_channel_select > EEP_INPUT6)
//			input_channel_select = EEP_INPUT1;
//	#else
//		if(input_channel_select > EEP_INPUT8)
//			input_channel_select = EEP_INPUT1;
//	#endif
//	// ----------------------------------------------------------------------
//
//
//	return TRUE;	// protocol was successful
//}					//PIC will start read ADC AT THIS POINT

 
#if defined T3_8IN13OUT   //MHF 20010_07 SINGLE IFDEF ISTEAD OF TWO BEFORE
bit WritePicCommand(unsigned char channel)
{
	// start the i2c routine, pic has been interrupt
	i2c_pic_start();
	// 		 
	i2c_pic_write( PIC16_PULSE_CLEAR);	 
 
	// ----------------------------------------------------------------------

	if (GET_ACK()){
		i2c_stop();
		return 0;
	}

	delay_us(1);	// needed for timing purposes

	i2c_pic_write(channel);
 
	// ---------------------------------------------------------------------
	 
	i2c_stop();
return 1;
}
#endif

unsigned char ReadPicVersion(void)
{
	unsigned char pic_version,check;
	// start the i2c routine, pic has been interrupt
	i2c_pic_start();
	// 		 
			i2c_pic_write( PIC16_VERSION);	 
 
	// ----------------------------------------------------------------------

	if (GET_ACK()){
		i2c_stop();
		return 0 ;
	}

	delay_us(1);	// needed for timing purposes

	
	pic_version = i2c_pic_read();
 
	GIVE_PIC_ACK();		
	delay_us(1);

	// read checksum
	check = i2c_pic_read();
//	GIVE_PIC_ACK();
	check  ^= pic_version;
	// stop the i2c routine
	i2c_stop();
 
	// ---------------------------------------------------------------------- 
	if(check == 0)
		pic_version = pic_version & 0xff;
	else
		pic_version= 0;	
	PICV = pic_version;
	// ----------------------------------------------------------------------
	return pic_version;
}

#if defined T3_8IN13OUT  //MHF 20010_07 SINGLE IFDEF INSTEAD OF TWO BEFORE
bit WritePicType(unsigned char channelType)
{
	// start the i2c routine, pic has been interrupt
	i2c_pic_start();
	// 		 
			i2c_pic_write( PIC16_CHANNEL_TYPE );	 
 
	// ----------------------------------------------------------------------

	if (GET_ACK()){
		i2c_stop();
		return 0;
	}

	delay_us(1);	// needed for timing purposes

	i2c_pic_write(channelType);
 
	// ----------------------------------------------------------------------
	 
		i2c_stop();
		return 1;
}
#endif
#ifdef T3_8IN13OUT//MHF 20010_07 SINGLE IFDEF

//-------------------crc16_tstat ---------------------
// calculate crc with one byte
void CRC16_PIC(unsigned char ch)
{
	unsigned char uIndex ;
	uIndex = PIC_CRChi ^ ch ; // calculate the CRC 
	PIC_CRChi = PIC_CRClo ^ auchCRCHi[uIndex] ;
	PIC_CRClo = auchCRCLo[uIndex] ;
}



void read_pulse_pic(void)
{	 
	
 	unsigned char checksum;
	unsigned char check1,check2;
	PIC_CRChi = 0xff;
	PIC_CRClo = 0xff;
	// start the i2c routine, pic has been interrupt
//	if(flag_comm)	return;
	i2c_pic_start();
  
	// send command
	i2c_pic_write( PIC16_PULSE );
 
	// ----------------------------------------------------------------------
//	delay_us(5);
	if (GET_ACK()){
		i2c_stop();
		return;
	}
 
	delay_us(1);	// needed for timing purposes

	 
 	i2c_pic_write(input_channel_select - EEP_INPUT1);
	// ----------------------------------------------------------------------
//	delay_us(5);
	if (GET_ACK()){
		i2c_stop();
		return;
	}
 
	delay_us(1);	// needed for timing purposes
	// read first byte
	number.pic_buffer[0] = i2c_pic_read();	
//	delay_us(5);	 
	GIVE_PIC_ACK();
	delay_us(1);
#ifdef CRC
	CRC16_PIC( number.pic_buffer[0]);

#else
checksum = number.pic_buffer[0];
#endif
 
	// read second byte
	number.pic_buffer[1] = i2c_pic_read();	
//	delay_us(5);
	GIVE_PIC_ACK();	
	delay_us(1);
#ifdef CRC
	CRC16_PIC(number.pic_buffer[1]); 
#else
checksum ^= number.pic_buffer[1];
#endif
 	// read third byte
	number.pic_buffer[2] = i2c_pic_read();	
//	delay_us(5);
	GIVE_PIC_ACK();		
	delay_us(1);
#ifdef CRC
	CRC16_PIC( number.pic_buffer[2]); 
#else
checksum ^= number.pic_buffer[2];
#endif
	// read fourth byte
	number.pic_buffer[3] = i2c_pic_read();	
//	delay_us(5);

	GIVE_PIC_ACK();		
	delay_us(1);
#ifdef CRC
	CRC16_PIC(number.pic_buffer[3]); 
#else
checksum ^= number.pic_buffer[3];
#endif
	// read fiveth byte
	adam.dual_byte[(input_channel_select - EEP_INPUT1) << 1] = i2c_pic_read();	
//	delay_us(5);
	GIVE_PIC_ACK();		
	delay_us(1);
#ifdef CRC
	CRC16_PIC(adam.dual_byte[(input_channel_select - EEP_INPUT1) << 1]); 
#else
checksum ^= adam.dual_byte[(input_channel_select - EEP_INPUT1) << 1];
#endif
	// read sixth byte
	adam.dual_byte[((input_channel_select - EEP_INPUT1) << 1) + 1] = i2c_pic_read();
	//delay_us(5);
	GIVE_PIC_ACK();		
	delay_us(1);
#ifdef CRC
	CRC16_PIC(adam.dual_byte[((input_channel_select - EEP_INPUT1) << 1) + 1]); 

#else
checksum ^= adam.dual_byte[((input_channel_select - EEP_INPUT1) << 1) + 1];
#endif
	if(PICV > 11)
	{
	// read sixth byte
	check1 = i2c_pic_read();   //MHF 20010_07 CHANGED THE PROTOCOL , I DONT KNOW WHY
	//delay_us(5);
	GIVE_PIC_ACK();		
	delay_us(1); 
	checksum ^= check1;
	// read sixth byte
	check2 = i2c_pic_read();
	//delay_us(5);
	GIVE_PIC_ACK();		
	delay_us(1); 
	checksum ^= check2; 
	}
	checksum ^= i2c_pic_read();
	// stop the i2c routine
	i2c_stop(); 
	#ifdef CRC
	//	if(check1 == PIC_CRChi && check2 == PIC_CRClo)
			store_pulse_registers(number.pic_pulse, input_channel_select);	
	#else
		if(checksum == 0)
		{ 
			store_pulse_registers(number.pic_pulse, input_channel_select);	
		}
	#endif
 

	// ----------------------------------------------------------------------
	input_channel_select++;
	 
	if(input_channel_select > EEP_INPUT8)
		input_channel_select = EEP_INPUT1;
 
	// ----------------------------------------------------------------------
}

#endif

bit read_pic_original( unsigned char store_location, bit set )
{

	unsigned int data_buf[10]=0;
	unsigned int checksum;
	unsigned char i;


	// Set the first channel for the input MUX
	CARDA0 = 0;
	CARDA1 = 0;
	CARDA2 = 0;


	i2c_pic_start();
	
	// we are now selecting the upper or lower set
	if ( set == HI )
		i2c_pic_write( 0xb1 );
	else if ( set == LO )
		i2c_pic_write( 0xb2 );
	delay_us(1);

	if (GET_ACK()){
		i2c_stop();
		return FALSE;
	}
	delay_us(1);

	data_buf[0] = i2c_pic_read();


	for (i = 1; i <= 9; i++)
	{
		if(!GIVE_PIC_ACK()){
			i2c_stop();
			return FALSE;
		}
		delay_us(1);
		// Set the next channel for the input MUX
		CARDA0 = i & 0x01;
		CARDA1 = i & 0x02;
		CARDA2 = i & 0x04;
		data_buf[i] = i2c_pic_read();
	}

	i2c_stop();

	checksum = (data_buf[0]+data_buf[1]+data_buf[2]+data_buf[3]
			+data_buf[4]+data_buf[5]+data_buf[6]+data_buf[7]+data_buf[8]) & 0x3FF;


	if (data_buf[9] == checksum)
	;//	analog_input_buffer(data_buf, 8, store_location);	


	return TRUE;
}



/*   not used 
unsigned int read_feedback(void)
{
	int feedback_data;	

	i2c_pic_start();
	
	i2c_pic_write( 0xb1 );
	delay_us(1);

	if (GET_ACK()){
		i2c_stop();
		EA = 1;
		return 0;
	}
	delay_us(1);

	feedback_data = i2c_pic_read();
	i2c_stop();

	return feedback_data;
}
*/

/*
// --- write data to the pic chip --------------------------------------
// first send addr and then send value
void write_pic( unsigned char addr, unsigned char value )
{
	// disable the global interrupts
	EA = 0;
//[note] taking off the enable and disable interrupts lines will make the lights run normally

	// send starting conditions
	i2c_pic_start();

	// send control byte telling we want to write to pic chip
	i2c_pic_write( 0xb0 );
	delay_us(1);
	// get acknowledge that message was received
	if (GET_ACK()){
		i2c_stop();
		EA = 1;
		return;
	}
	delay_us(1);

	// acknowledge received, now want to sent data
	i2c_pic_write( addr );
	// get acknowledge that message was received
	if (GET_ACK()){
		i2c_stop();
		EA = 1;
		return;
	}
	delay_us(1);

	// acknowledge received, now want to sent data
	i2c_pic_write( value );
	// get acknowledge that message was received
	if (GET_ACK()){
		i2c_stop();
		EA = 1;
		return;
	}
	delay_us(1);

	i2c_stop();
	EA = 1;
	return;
}
*/
//;***********************************/
/*;i2c_startup sequence of 24Cxx*/

void i2c_pic_start()
{
	// PIC chip requires a special double start condition in order
	// to initiate communication.  This is to insure the PIC does not 
	// read any false starts.
	// done so to comply with new I2C pic protocol
	// 06/04/05	RL
	//if(flag_comm) return; 
	// clock pulse to trigger interrupt on PIC
	E2CLOCK = 0;
	delay_us(1);

	E2DATA = 1;
	E2CLOCK = 1;
	delay_us(2);

	// 1st start condition
	E2DATA = 0;
	delay_us(5);
	E2CLOCK = 0;
	delay_us(2);

	if(pic_type == 2)
	{
		// reset bus
		E2DATA = 1;
		E2CLOCK = 1;
		delay_us(2);
		
		// 2nd start condition
		E2DATA = 0;
		delay_us(5);
		E2CLOCK = 0;
		delay_us(2);
	}
	else if(pic_type >= 3)	// pic done by Robert
	{
		// reset bus
		E2DATA = 1;
		E2CLOCK = 1;
		delay_us(2);
		
		// 2nd start condition
		E2DATA = 0;
		delay_us(5);
		E2CLOCK = 0;
		delay_us(2);	
	
	} 
	

}



bit GIVE_PIC_ACK( void )
{
 	int j=0;
	// Wait until the data signal goes high
	while (!E2DATA){
		j++;
		// If no clock, exit i2c_read routine
		if (j == 100)
			return FALSE;
	}
 
   E2DATA=1;
   delay_us (1);
   E2CLOCK=1;
   E2DATA=0;
   delay_us (1);
   E2DATA=1;
   E2CLOCK=0;

   return TRUE;
}

/*;************************************/
/*;send a 8-bit data to 16F676 */
void i2c_pic_write( unsigned char ch )
{
	
	unsigned char i = 8;
//	EA = 0;
	//if(flag_comm) return;
	while( i!=0 )
	{    
		E2DATA = ( ch & 0x80 );
		E2CLOCK=1;
		delay_us(1);
		E2CLOCK=0;
		delay_us(1);
		ch<<=1;
		i--;
	}
//	EA = 1;
}


/*;**************************************/
/*;receive a 8-bit data from 16F676 */
unsigned int i2c_pic_read( void )
{
	unsigned char i;
	unsigned int data1 = 0;
	unsigned char count;
//	ET0 = 0;
	//if(flag_comm) return 0;
 	if(pic_type >= 3)
		count = 8;
	else 
		count = 10;
 
	for( i=0; i<count; i++ )
	{
 
		E2CLOCK = 1;
		delay_us(1);
		E2DATA = 1;	  
		data1 = ( data1 << 1 ) | E2DATA;
		E2CLOCK = 0;
		delay_us(1);
	}
//	ET0 = 0;
	return data1;
}


/*;************************************/
/*;i2c_stop sequence of 24Cxx*/
void i2c_stop(void)
{
	E2DATA = 0;
	E2CLOCK = 1;
	E2DATA = 1;
}

/*;**************************************/
/*;detect the ACK signal to 24Cxx*/			


unsigned int GET_ACK( void )
{
  // unsigned int c=0;
   unsigned int i;

	if(pic_type == 1)
	{
	   E2DATA=1;
	   E2CLOCK=1;
	   for (i=0; i<10; i++)
	   {
			if (E2DATA == 0){
			    E2CLOCK=0;
				return 0;
			}		
	   }
	   E2CLOCK=0;
	   return 1;
	}
	else if(pic_type == 2)
	{
	    // Wait for data line to be pulled low.
	    for (i=0; i<50; i++)
	    {
			if (E2DATA == 0)
			{
				// if data line is low, pulse the clock.
			    E2CLOCK=1;
				delay_us(1);
			    E2CLOCK=0;
				return 0;
			}		
	    }
	    E2CLOCK=0;
	    return 1;
	}
	else if(pic_type >= 3)	// PIc code done by Robert
	{
 
		    // Wait for data line to be pulled low.
		    for (i=0; i< 50; i++)
		    {
				E2DATA = 1;
				if (E2DATA == 0)
				{
					// if data line is low, pulse the clock.
				    E2CLOCK=1;
					delay_us(1);
				    E2CLOCK=0;
					return 0;
				}		
		    }
		    E2CLOCK=0;
		    return 1;
	}


}




 /**********************************Filter***********************************************************/
/*
Description: Filter the data sampled from ADC to get rid of illegal value caused by noise.
parameter:	channel ,there are eight channels for this product,they should be 0 to 7.if channel = 8,
			it means that the signal from internal thermistor.
			input, Reading from ADC ,need to be filtered
Return:		Filtered value for send to PC or use for other purpose	
			
*/
/*********************************Filter funtion start***********************************************/
unsigned int Filter(unsigned char channel,unsigned input)
{
	// -------------FILTERING------------------
 	// -------------FILTERING------------------
	signed int xdata siDelta;
	signed int xdata siResult;
    signed int xdata siTemp;
	signed long xdata slTemp;
    unsigned char xdata I;  
    I = channel;
	siTemp = input;
	/*if((I <=CHANNEL_7)||((I>=CHANNEL_16)&&(I<= CHANNEL_23)))
	{
		if(siTemp < old_reading[I+8])
		{
			siTemp -= (signed int)((1.0*(1023 - siTemp)/1023)* 30) ;
			if(siTemp < 0) siTemp = 0;
		}
		else
		{
			siTemp += (signed int)((2.6*(1023 - old_reading[I+8])/1023)* 30) ;
			if(siTemp > 1023) siTemp = 1023;	
		}				
	}
	else if(((I > CHANNEL_7)&&(I < CHANNEL_16))||((I>CHANNEL_23)&&(I<= CHANNEL_31)))
	{
		if(siTemp < old_reading[I -8])
		{
			siTemp -= (signed int)((1.0*(1023 - siTemp)/1023)* 30) ;
			if(siTemp < 0) siTemp = 0;
		}
		else
		{
			siTemp += (signed int)((2.6*(1023 - old_reading[I -8])/1023)* 30) ;
			if(siTemp > 1023) siTemp = 1023;	
		}				
	}*/
 	if((range[I] == 4)||(range[I] == 5))
	  siResult=	siTemp;
	else
	{
		if(filter[I] >= 0)
		{
			siDelta = siTemp - (signed int)old_reading[I] ;    //compare new reading and old reading
		
			// If the difference in new reading and old reading is greater than 5 degrees, implement rough filtering.
		    if (( siDelta >= 200 ) || ( siDelta <= -200 ) ) // deg f
			{
					old_reading[I] =  siTemp ;
			}			
			// Otherwise, implement fine filtering.
			else
			{		      			    
				slTemp = (signed long)filter[I]*old_reading[I];
				slTemp += (signed long)siTemp;
			 	old_reading[I] = (signed int)(slTemp/(filter[I] +1));			 
		    }
			siResult = old_reading[I];
	    }
	}
	
	//old_reading[I] = siResult ;
	return siResult;	
}



// *****************************************************************************************************
// data read from pic and now stored into registers
// analog_input_buffer()
// refresh_inputs()
// *****************************************************************************************************
void store_to_registers (unsigned int pic_data, unsigned char register_location)
{
	


#ifndef T3_32IN
	unsigned int xdata uiSum,uiMin,uiMax,uiTemp,temp,temp1;
	unsigned char xdata ucTemp;
#else
	unsigned int xdata uiTemp,temp,temp1;  //MHF 20010_07 TEMP1 HOLDS SOME FILTER INFO
	unsigned char xdata ucTemp;
#endif

	if(reading_filter_bypass > 0)
	reading_filter_bypass--;

#ifdef T3_32IN
		uiTemp = Filter(register_location, pic_data);
		ucTemp = register_location;
		if(uiTemp > 1023)
		uiTemp = 1023;
	/*	if((ucTemp <=CHANNEL_7)||((ucTemp>=CHANNEL_16)&&(ucTemp<= CHANNEL_23)))
		{
			if(uiTemp < old_reading[ucTemp+8])
			{
				uiTemp -= (signed int)((1.0*(1023 - uiTemp)/1023)* 29) ;
				if(uiTemp < 0) uiTemp = 0;
			}
			else
			{
				uiTemp += (signed int)((2.6*(1023 - old_reading[ucTemp+8])/1023)* 29) ;
				if(uiTemp > 1023) uiTemp = 1023;	
			}				
		}
		else if(((ucTemp > CHANNEL_7)&&(ucTemp < CHANNEL_16))||((ucTemp>CHANNEL_23)&&(ucTemp<= CHANNEL_31)))
		{
			if(uiTemp < old_reading[ucTemp -8])
			{
				uiTemp -= (signed int)((1.0*(1023 - uiTemp)/1023)* 29) ;
				if(uiTemp < 0) uiTemp = 0;
			}
			else
			{
				uiTemp += (signed int)((2.6*(1023 - old_reading[ucTemp -8])/1023)* 29) ;
				if(uiTemp > 1023) uiTemp = 1023;	
			}				
		}*/
		if(reading_filter_bypass) // || filter[ucTemp] == 0) Rev37 get rid of filter = 0
		{
	 
			old_reading[ucTemp] = uiTemp;	 
			temp = uiTemp;

		}
		else
		{			 
				//temp = Filter(ucTemp,uiTemp);
			temp1 = (unsigned int)(old_reading[ucTemp]); //MHF 20010_07 FILETR TO THROW OUT GARBAGE FROM PIC READINGS
			temp = temp1;
            
			if(abs(temp1 - uiTemp) > 300)
			{
				gucFilterCounter[ucTemp]++;
				if(gucFilterCounter[ucTemp] > 2)
				{
					temp  = uiTemp;
					gucFilterCounter[ucTemp] = 0;
					old_reading[ucTemp] = uiTemp;
				}	 
				
			}
			else
			{
				temp  = uiTemp;
				gucFilterCounter[ucTemp] = 0;
				old_reading[ucTemp] = uiTemp;
			}
			 
			if(temp > 1023)
				temp = 1023;				 
			 
		}
 
		modbus.registers[ucTemp] = temp; 
	
	//	if(ucTemp<16)  //MHF 20010_07  DEBUG
	//	if(abs(modbus.registers[ucTemp] - guiPreviousValue[ucTemp]) > 200);	
		//modbus.registers[ucTemp] = 555 ;
#else
		uiTemp = pic_data;
		if(uiTemp > 1023)
		uiTemp = 1023;
		if(reading_filter_bypass || filter[register_location - 8] == 0)
		{
	 
			old_reading[register_location - 8] = uiTemp;	 
			temp = uiTemp; 	
			for(ucTemp = 0;ucTemp < 10;ucTemp++)
				guiBuffer[register_location-8][ucTemp] = uiTemp;
			gucCounter[register_location-8] = 9;
		}
	else
	{

		gucCounter[register_location - 8] += 1;
		if ( gucCounter[register_location - 8]  > 9)
		 gucCounter[register_location - 8]  = 0;
		guiBuffer[register_location- 8][ gucCounter[register_location-8] ] = uiTemp;

		uiMin = guiBuffer[register_location-8][0];
		uiMax = guiBuffer[register_location-8][0];
		for(ucTemp=1;ucTemp<10;ucTemp++)
		{
			if(uiMin > guiBuffer[register_location-8][ucTemp])
			uiMin = guiBuffer[register_location-8][ucTemp];
			if(uiMax < guiBuffer[register_location-8][ucTemp])
			uiMax = guiBuffer[register_location-8][ucTemp];
		}
		uiSum = 0;
		for(ucTemp=0;ucTemp<10;ucTemp++)
		uiSum +=   guiBuffer[register_location-8][ucTemp];
		uiSum = uiSum - uiMin - uiMax;
		//modbus.registers[8] = register_location; // uiSum >> 3;4*/
		temp1 = uiSum >> 3;
		temp = Filter(register_location-8,temp1);
	}
	modbus.registers[register_location] = temp;



#endif



#ifndef T3_32IN
	//guiAnalogInput[register_location - 8] = RangeConverter(range[register_location - 8],modbus.registers[register_location],128,0);
	guiAnalogInput[register_location - 8] = RangeConverter(range[register_location - 8],modbus.registers[register_location],128,0);
#else
	guiAnalogInput[register_location] = RangeConverter(range[register_location],modbus.registers[register_location],128,0);
//	guiAnalogInput_temp[register_location] = guiAnalogInput[register_location]-DEFAULT_OFFSET +calibration_offset[register_location]; 
  
#endif
	


	  	/*
//		modbus.registers[register_location] = pic_data + (output_calibration-CALIBRATION_OFFSET);
		modbus.registers[register_location] = pic_data;
	

	
	/*
    else if ( pic_data > modbus.registers[register_location]  )
    	analog_input_filter[register_location]++ ; 					//counters, keep track of number of times with reading GREATER
    else if ( pic_data < modbus.registers[register_location] )
        analog_input_filter[register_location]-- ;					//counters, keep track of number of times with reading SMALLER


    if( (analog_input_filter[register_location] > INPUT_FILTER) || (analog_input_filter[register_location] < (0 - INPUT_FILTER )) )
	{
		if(buffer_difference > BUFFER_DIFFERENCE_THRESHOLD)
		{
			if(analog_input_filter[register_location] > INPUT_FILTER)
	    		modbus.registers[register_location] += (buffer_difference / BUFFER_DIVIDER) ; 
			else
				modbus.registers[register_location] -= (buffer_difference / BUFFER_DIVIDER) ; 
		}
		else if(analog_input_filter[register_location] > INPUT_FILTER)
	    	modbus.registers[register_location]++ ; 

	    else if(analog_input_filter[register_location] < (0 - INPUT_FILTER ))
		{
			if ( modbus.registers[register_location] != 0 )
	     	   modbus.registers[register_location]-- ;
		}
	
		// reset the counterS
		analog_input_filter[register_location] = 0;
	}
	*/
	//modbus.registers[register_location] = pic_data;

	//gr-/

	// rectification of register value (top)
	if( modbus.registers[register_location] > 1024)
		modbus.registers[register_location] = 1024;

	// rectification of register value (bottom)
	if( modbus.registers[register_location] > 60000)	// if the register rolls-over below zero, reset value to zero
		modbus.registers[register_location] = 0;






	
	#ifdef FLEXDRIVER_FEATURE
		// in this case state_buffer bit should be 2
		if( (modbus.registers[register_location] < STATE_CHANGE_LOWER) || (modbus.registers[register_location] > STATE_CHANGE_UPPER) )
//		if(modbus.registers[register_location] < 512)	// debug purposes
//		if(modbus.registers[register_location] < 200)	// debug purposes
		{	
			// there was a change in state, SET the flag and SET modbus.registers[EEP_LED_STATUS_PAIR1]
			if(modbus.registers[EEP_LED_STATUS_PAIR1 + register_location] != 2)
			{
				modbus.registers[EEP_LED_STATUS_PAIR1 + register_location] = 2;
				master_com_serial( 20, FlexDriver_LED_STATUS_PAIR1+register_location, 2 );
			}



		}
		// in this case state_buffer bit should be 1
		else
		{
			// there was a change in state, SET the flag and CLEAR modbus.registers[EEP_LED_STATUS_PAIR1]
			if(modbus.registers[EEP_LED_STATUS_PAIR1 + register_location] != 1)
			{
				modbus.registers[EEP_LED_STATUS_PAIR1 + register_location] = 1;
				master_com_serial( 20, FlexDriver_LED_STATUS_PAIR1+register_location, 1 );
			}
			
		}
	

	
	
	#endif

}



 
#if defined T3_8IN13OUT//MHF 20010_07  COMBINE TWO IFDEFS INTO ONE

signed int FilterAdam(unsigned char channel,signed input)
{
	// -------------FILTERING------------------
	signed int xdata siDelta;
	signed int xdata siResult;
	unsigned char xdata I;
    signed int xdata siTemp;
	signed long xdata slTemp;
	I = channel;
	siTemp = input;
 
 
 
	siDelta = siTemp - (signed int)old_reading[I] ;    //compare new reading and old reading

	// If the difference in new reading and old reading is greater than 5 degrees, implement rough filtering.
    if (( siDelta >= 100 ) || ( siDelta <= -100 ) ) // deg f
		adam_old_reading[I] = adam_old_reading[I] + (siDelta >> 1);//1 
 			
	// Otherwise, implement fine filtering.
	else
	{		      
	 
		slTemp = (signed long)filter[I]*adam_old_reading[I];
		slTemp += (signed long)siTemp * (256 - filter[I]);
	 	adam_old_reading[I] = (signed int)(slTemp/256);			 
	 
	}

	siResult = adam_old_reading[I];
 	
 
	return siResult;
	
}

// *****************************************************************************************************
// data read from pic and now stored into registers
// analog_input_buffer()
// refresh_inputs()
// *****************************************************************************************************

void store_pulse_registers (unsigned long data_buffer, unsigned char register_location)
{

	unsigned char i;
//	signed int Delta;
	signed long result;
	unsigned int temp,temp1,temp2;
	//signed int analog;
	i = register_location - EEP_INPUT1;
	if(reading_filter_bypass > 0)
	reading_filter_bypass--;

	if(GetBit(i,&channel_type))
	{
		if(reading_filter_bypass)
		{
			pic.number_long[i] = data_buffer + flash.number_long[register_location - EEP_INPUT1];
			old_reading[i] = data_buffer;	
				previous_pulse_number[i] = pic.number_long[i];	 
		}
		else
		{
			result = data_buffer - old_reading[i] ;
			if(result > -4096 && result < 4096)
	       	{
		  		pic.number_long[i] = data_buffer + flash.number_long[register_location - EEP_INPUT1];
				old_reading[i] = data_buffer;

				RangeConverter(range[i],512,128,i);
			}
		}

  	}
	else
	{
		if(data_buffer > 1023)
		data_buffer = 1023;
		if(reading_filter_bypass || filter[i] == 0)
		{
	 		FilterTemp[i] = data_buffer;
			old_reading[i] = data_buffer;	 
			pic.number_long[i] = RangeConverter(range[i],(signed int)data_buffer,128,i);
		}
		else
		{
			 
			temp1 = FilterTemp[i];  
			temp = temp1; 
			if(abs(temp1 - data_buffer) > 20)
			{
				gucFilterCounter[i]++;
				if(gucFilterCounter[i] > 2)
				{
					temp  = data_buffer;
					gucFilterCounter[i] = 0;
					FilterTemp[i] = data_buffer;
				}	 
				
			}
			else
			{
				 if(FilterTemp[i] > data_buffer ) 
				 { 
				 	fliter_slow_high[i]++;
					fliter_slow_low[i] = 0 ;
					if(fliter_slow_high[i] >5)
					{
				 		temp =FilterTemp[i] -1 ; 
						fliter_slow_high[i] = 0;
					}
				 }
				 else if(data_buffer > FilterTemp[i] )
				 { 
				 	fliter_slow_low[i]++;
					fliter_slow_high[i] = 0;
					if(fliter_slow_low[i] >5)
					{
				 		temp =FilterTemp[i] +1 ; 
						fliter_slow_low[i] = 0;
					}
				 }				  
				 else  temp =  data_buffer ;
				 //temp  = data_buffer;

				//temp  = FilterTemp[i]+(int)(FilterTemp[i]-data_buffer)>>2;
				gucFilterCounter[i] = 0;
				FilterTemp[i] = data_buffer;
			}
		 			
			 
				temp2 = Filter(i,temp);
				if(temp2 > 1023)
					temp2 = 1023;
		      pic.number_long[i] = RangeConverter(range[i],temp2,128,i);
			//pic.number_long[i] = RangeConverter(range[i],500,128,i);
			 
		}
		
	}
 

		if(reading_filter_bypass || filter[i] == 0)
		{
		 
			adam_old_reading[i] = (unsigned int)(adam.dual_byte[i << 1] << 8) + adam.dual_byte[(i << 1) + 1];
	 
			adam_buffer[i] = (unsigned int)(adam.dual_byte[i << 1] << 8) + adam.dual_byte[(i << 1) + 1];

		}
		else
		{	
			temp = (unsigned int)(adam.dual_byte[i << 1] << 8) + adam.dual_byte[(i << 1) + 1];	
		//	adam_buffer[i] = 500 ;   		 
			adam_buffer[i] = FilterAdam(i,temp); 
			if(adam_buffer[i] > 1023)
			adam_buffer[i] = 1023;
			 
		}
/*{
 
		// If the temperature filter bypass has been set, ignore all filtering and simply take the new reading.
		if (reading_filter_bypass)
		{
			reading_filter_bypass--;
			pic.number_long[i] = data_buffer;
			old_reading[i] = data_buffer*100;
		}
		else 
		{
			Delta = data_buffer - old_reading[i]/100 ;    //compare new reading and old reading

			// If the difference in new reading and old reading is greater than 100, implement rough filtering.
		    if (( Delta >= 10 ) || ( Delta <= -10 ) ) // 
    			old_reading[i] = old_reading[i] + 50*Delta; 
			else if(( Delta > 0 ) && ( Delta <= -1 ) )
			{
				old_reading[i]++;
			}
			else if(( Delta >= 1 ) && ( Delta < 0 ) )
			{
				old_reading[i]--;
			}
			else if( Delta == 0 )
				old_reading[i] = old_reading[i];
			// Otherwise, implement fine filtering.
			else
			{

			 	old_reading[i] = old_reading[i] * (5 - 1);	
		    	old_reading[i] =  old_reading[i] +  data_buffer*100;
				old_reading[i] /= 5; 
					   
			}
		 
			adc_result = (unsigned int)(old_reading[i] / 100);
			Delta = data_buffer - adc_result;
			if(Delta == -1)
			{
				if(old_reading[i] % 100 < 30)
				adc_result--;
			}
			 
			pic.number_long[i] = adc_result;
		}
	}*/

}
#endif

 

