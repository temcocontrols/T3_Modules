#include <KEIL_VRS1000_regs.H>
#include "LibIO_T3IO.h"
#include "define.h"
#include "read_outputs.h"

extern bit switch_to_auto;

extern unsigned int xdata threshold ;
extern unsigned long xdata  guiDeadmasterTime; 
#if defined T3_8IN13OUT  //MHF 20010_07 COMBINE TWO IFDEFS

extern unsigned char  xdata gucOverOutput[8];

extern unsigned int data guiManual;

extern unsigned char xdata gucTimer[8];
extern unsigned char xdata gucZone[13];
unsigned char xdata gucStatus[13];
extern unsigned char xdata gucTimerFilter[8];	
extern bit flash_read_int(unsigned char id, unsigned int *value, unsigned char block_select);
extern unsigned char xdata gucReverseOutput ;
void LightOutput(unsigned char overOut);
#endif
#ifdef  T3_8IO_A
extern unsigned int xdata deadmaster_output[8];
#endif

// ******************************************************************************************************
// for Benny feature
// ******************************************************************************************************

#if defined (BENNY_FEATURES)

void refresh_outputs (void)
{
	unsigned int output_sequence_buffer;
	unsigned char mask = 0;
	unsigned char i = 0;
	unsigned char mux0, mux1, mux2;


	signed int output_buffer;

	// if we are in digital mode, if statemet is false thus we skip the following part
	if(!digital_mode)
	{
		//disable the input MUX
		MUX_OFF1 = 1;
		// Save the current MUX channel.
		mux0 = CARDA0;
		mux1 = CARDA1;
		mux2 = CARDA2;
		
		// Set the channel for the output MUX
		CARDA0 = output_channel & 0x01;
		CARDA1 = output_channel & 0x02;
		CARDA2 = output_channel & 0x04;
		// Enable the Output MUX
		MUX_OFF2 = 0;
		delay_us(500);
		// Disable the Output MUX
		MUX_OFF2 = 1;

		//Restore the MUX channel and enable the input MUX
		CARDA0 = mux0;
		CARDA1 = mux1;
		CARDA2 = mux2;
		MUX_OFF1 = 0;

	}


	// Switch to the next channel
	output_channel = (output_channel+1)%MAX_OUTPUT_CHANNELS  ;    //increment the output_channel

	// Set the PWM for the testing sequence case
	if(output_sequence == 0)		// july 22 Ron
	{	
		if(digital_mode)
		{	// August Ron
			// testing sequence when in digital mode
			if(Enable_set_output)
			{	// set modbus.registers one channel at a time to test one output at a time
				if (output_sequence_count == 12)
				{	// set a different channel each time we get into this if statemen
					output_sequence_channel = (output_sequence_channel+1)%MAX_OUTPUT_CHANNELS;
		
					Enable_set_output = 0;	// stop setting outputs, wait till reading is done
					Enable_read_input = 1;	// given output is stopped, can now start reading inputs
	
					if(incrementing)
					{	// in this case set output to a high
						modbus.registers[output_sequence_channel] = 1000;
						// once we have reached a full cycle, start setting output to low
						if(output_sequence_channel==7)
							incrementing = 0;
						// reset counter
						output_sequence_count = 0;
						// turn LEDs off and now start testing SOP
						if ( startup_flag != 0)
							startup_flag--;

					}
					else
					{	// in this case set output to a low
						modbus.registers[output_sequence_channel] = 0;
						// once we have reached a ful cycle, start settign output to high
						if(output_sequence_channel==7)
							incrementing = 1;
						// reset counter
						output_sequence_count = 0;
						// turn LEDs off and now start testing SOP
						if ( startup_flag != 0)
							startup_flag--;
					}			
				}

				output_sequence_count++;
			}
		}
		else
			if (Enable_set_output)		// in SOP mode and do nothing if reading inputs
			{							// testing the analog mode
				// add a buffer value so that adjacent switches do not have same value
				// at every even switch add 500mV
				// august 9 Ron
				if ( output_channel%2 )
					output_sequence_buffer = 1000 - testing_increment_init;
				else
					output_sequence_buffer = testing_increment_init;
		
				// set PWM to be sent to channel
				modbus.registers[7-output_channel] = output_sequence_buffer;
	
				// once have set 8 channels for four full cycle, pause and wait for inputs
				// we do this multiple times in order to ensure that the signal is settled
				if (output_sequence_count == 32)
				{	
					Enable_set_output = 0;	// stop setting outputs, wait till reading is done
					Enable_read_input = 1;	// given output is stopped, can now start reading inputs
	
					// increment/decrement testing signal and make sure it is between the range
					if (incrementing)
					{	testing_increment_init = testing_increment_init + TESTING_INCREMENT_VALUE;
						if (testing_increment_init == 1000)
							incrementing = 0;
					}
					else
					{	testing_increment_init = testing_increment_init - TESTING_INCREMENT_VALUE;
						if (testing_increment_init == 0)
							incrementing = 1;
					}
	
					// reset counter
					output_sequence_count = 0;
					// turn LEDs off and now start testing SOP
					if ( startup_flag != 0)
						startup_flag--;

				}
				// each cycle only sets one channel at a time
				output_sequence_count++;
			}

	}

	if (digital_mode)
	{
		switch(output_channel)
			{
				case 0:

					if (switch_state[output_channel] == 0 && output_sequence )
					{	if (reverse_logic_output)
							RELAY1 = 0;
						else
							RELAY1 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence )
					{	if (reverse_logic_output)
							RELAY1 = 1;
						else
							RELAY1 = 0;
					}
					else
					{
						if (modbus.registers[output_channel+8] <= 512)
							RELAY1 = 1;
						else
							RELAY1 = 0;
						}

					break;


				case 1:
					if (switch_state[output_channel] == 0 && output_sequence )
					{	if (reverse_logic_output)
							RELAY2 = 0;
						else
							RELAY2 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence )
					{	if (reverse_logic_output)
							RELAY2 = 1;
						else
							RELAY2 = 0;
					}
					else
					{
						if (modbus.registers[output_channel+8] <= 512)
							RELAY2 = 1;
						else
							RELAY2 = 0;
					}

					break;


				case 2:
					if (switch_state[output_channel] == 0 && output_sequence )
					{	if (reverse_logic_output)
							RELAY3 = 0;
						else
							RELAY3 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence )
					{	if (reverse_logic_output)
							RELAY3 = 1;
						else
							RELAY3 = 0;
					}
					else
					{
						if (out3)
							RELAY3 = 1;
						else
							RELAY3 = 0;
					}

					break;
				case 3:
					if (switch_state[output_channel] == 0 && output_sequence )
					{	if (reverse_logic_output)
							RELAY4 = 0;
						else
							RELAY4 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence )
					{	if (reverse_logic_output)
							RELAY4 = 1;
						else
							RELAY4 = 0;
					}
					else
					{
						if (modbus.registers[output_channel+8] <= 512)
							RELAY4 = 1;
						else
							RELAY4 = 0;
					}

					break;

				case 4:
					if (switch_state[output_channel] == 0 && output_sequence )
					{	if (reverse_logic_output)
							RELAY5 = 0;
						else
							RELAY5 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence )
					{	if (reverse_logic_output)
							RELAY5 = 1;
						else
							RELAY5 = 0;
					}
					else
					{
						if (modbus.registers[output_channel+8] <= 512)
							RELAY5 = 1;
						else
							RELAY5 = 0;
					}

					break;


				case 5:
					if (switch_state[output_channel] == 0 && output_sequence)
					{	if (reverse_logic_output)
							RELAY6 = 0;
						else
							RELAY6 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence )
					{	if (reverse_logic_output)
							RELAY6 = 1;
						else
							RELAY6 = 0;
					}
					else
					{
						if (modbus.registers[output_channel+8] <= 512)
							RELAY6 = 1;
						else
							RELAY6 = 0;
					}

					break;


				case 6:
					if (switch_state[output_channel] == 0 && output_sequence )
					{	if (reverse_logic_output)
							RELAY7 = 0;
						else
							RELAY7 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence  )
					{	if (reverse_logic_output)
							RELAY7 = 1;
						else
							RELAY7 = 0;
					}
					else
					{
						if (modbus.registers[output_channel+8] <= 512)
							RELAY7 = 1;
						else
							RELAY7 = 0;
					}

					break;


				case 7:
					if (switch_state[output_channel] == 0 && output_sequence  )
					{	if (reverse_logic_output)
							RELAY8 = 0;
						else
							RELAY8 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence   )
					{	if (reverse_logic_output)
							RELAY8 = 1;
						else
							RELAY8 = 0;
					}
					else
					{
						if (modbus.registers[output_channel+8] <= 512)
							RELAY8 = 1;
						else
							RELAY8 = 0;
					}

					break;



			}


	}

}


// ******************************************************************************************************
// for PWM
// ******************************************************************************************************
#elif defined (PWM_TRANSDUCER)


void refresh_outputs (void)
{
	unsigned int output_sequence_buffer;
	unsigned char mask = 0;
	unsigned char i = 0;
	unsigned char mux0, mux1, mux2;
	signed int output_buffer;

	//disable the input MUX
	MUX_OFF1 = 1;
	// Save the current MUX channel.
	mux0 = CARDA0;
	mux1 = CARDA1;
	mux2 = CARDA2;
	
	// Set the channel for the output MUX
	CARDA0 = output_channel & 0x01;
	CARDA1 = output_channel & 0x02;
	CARDA2 = output_channel & 0x04;

	// Enable the Output MUX
	MUX_OFF2 = 0;
	delay_us(500);
	// Disable the Output MUX
	MUX_OFF2 = 1;


	//Restore the MUX channel and enable the input MUX
	CARDA0 = mux0;
	CARDA1 = mux1;
	CARDA2 = mux2;
	MUX_OFF1 = 0;


	// Switch to the next channel
	output_channel = (output_channel+1)%MAX_OUTPUT_CHANNELS  ;    //increment the output_channel

	// Set the PWM for the testing sequence case
	if(output_sequence == 0)		// july 22 Ron
	{	

		if (Enable_set_output)		// in SOP mode and do nothing if reading inputs
		{							// testing the analog mode
			// add a buffer value so that adjacent switches do not have same value
			// at every even switch add 500mV
			// august 9 Ron
			if ( output_channel%2 )
				output_sequence_buffer = 1000 - testing_increment_init;
			else
				output_sequence_buffer = testing_increment_init;
	
			// set PWM to be sent to channel
			modbus.registers[7-output_channel] = output_sequence_buffer;

			// once have set 8 channels for four full cycle, pause and wait for inputs
			// we do this multiple times in order to ensure that the signal is settled
			if (output_sequence_count == 32)
			{	
				Enable_set_output = 0;	// stop setting outputs, wait till reading is done
				Enable_read_input = 1;	// given output is stopped, can now start reading inputs

				// increment/decrement testing signal and make sure it is between the range
				if (incrementing)
				{	testing_increment_init = testing_increment_init + TESTING_INCREMENT_VALUE;
					if (testing_increment_init == 1000)
						incrementing = 0;
				}
				else
				{	testing_increment_init = testing_increment_init - TESTING_INCREMENT_VALUE;
					if (testing_increment_init == 0)
						incrementing = 1;
				}

				// reset counter
				output_sequence_count = 0;
				// turn LEDs off and now start testing SOP
				if ( startup_flag != 0)
					startup_flag--;

			}
			// each cycle only sets one channel at a time
			output_sequence_count++;
		}
	



	}
	
	// analog mode output
	output_buffer = modbus.registers[EEP_OUTPUT1 + output_channel]+output_calibration-CALIBRATION_OFFSET;
	// rectify reading through calibration
	if( LUT_present )
		output_buffer = rectify_reading( EEP_OUTPUT1 + output_channel, output_buffer);

	if (output_buffer < 0)
		output_buffer = 0;

	PWMD3 = look_up_table(output_buffer);

}





// ******************************************************************************************************
// for 32IN, do nothing
// ******************************************************************************************************
#elif defined (T3_32IN)



// ******************************************************************************************************
// for T3-8IO or T3-8i16o
// ******************************************************************************************************


#else

void refresh_outputs (void)
{
	unsigned int output_sequence_buffer;
	unsigned char mask = 0;
	unsigned char i = 0;
	
	#ifndef T3_8IN16OUT
		unsigned char mux0, mux1, mux2;
		//signed int output_buffer;

		// if we are in digital mode, statement is false thus we skip the following part
		if(!digital_mode)
		{
			//disable the input MUX
			MUX_OFF1 = 1;
			// Save the current MUX channel.
			mux0 = CARDA0;
			mux1 = CARDA1;
			mux2 = CARDA2;		
			// Set the channel for the output MUX
			CARDA0 = output_channel & 0x01;
			CARDA1 = output_channel & 0x02;
			CARDA2 = output_channel & 0x04;
			MUX_OFF2 = 0;
			delay_us(500);			
			MUX_OFF2 = 1;	// Disable the Output MUX
			
			//Restore the MUX channel and enable the input MUX
			CARDA0 = mux0;
			CARDA1 = mux1;
			CARDA2 = mux2;
			MUX_OFF1 = 0;
		}
	#else
		bit led_drive1_buf, led_drive2_buf, led_drive3_buf, led_drive4_buf;
		unsigned char output_state[2], P0_buffer;
	#endif
//	 output_channel ++ ;
//	 if(output_channel== MAX_OUTPUT_CHANNELS ) 	 output_channel = 0 ;

	 //Switch to the next channel
	output_channel = (output_channel+1)%MAX_OUTPUT_CHANNELS  ;    //increment the output_channel

	// Set the PWM for the testing sequence case
	if(output_sequence == 0)		// july 22 Ron
	{	
		if(digital_mode)
		{	// August Ron
			// testing sequence when in digital mode
			if(Enable_set_output)
			{	// set modbus.registers one channel at a time to test one output at a time
				if (output_sequence_count == 12)
				{	// set a different channel each time we get into this if statemen
					output_sequence_channel = (output_sequence_channel+1)%MAX_OUTPUT_CHANNELS;
		
					Enable_set_output = 0;	// stop setting outputs, wait till reading is done
					Enable_read_input = 1;	// given output is stopped, can now start reading inputs
	
					if(incrementing)
					{	// in this case set output to a high
						modbus.registers[output_sequence_channel] = 1000;
						// once we have reached a full cycle, start setting output to low
						if(output_sequence_channel==7)
							incrementing = 0;
						// reset counter
						output_sequence_count = 0;
						// turn LEDs off and now start testing SOP
						if ( startup_flag != 0)
							startup_flag--;

					}
					else
					{	// in this case set output to a low
						modbus.registers[output_sequence_channel] = 0;
						// once we have reached a ful cycle, start settign output to high
						if(output_sequence_channel==7)
							incrementing = 1;
						// reset counter
						output_sequence_count = 0;
						// turn LEDs off and now start testing SOP
						if ( startup_flag != 0)
							startup_flag--;
					}			
				}

				output_sequence_count++;
			}
		}
		else
			if (Enable_set_output)		// in SOP mode and do nothing if reading inputs
			{							// testing the analog mode
				// add a buffer value so that adjacent switches do not have same value
				// at every even switch add 500mV
				// august 9 Ron
				if ( output_channel%2 )
					output_sequence_buffer = 1000 - testing_increment_init;
				else
					output_sequence_buffer = testing_increment_init;
		
				// set PWM to be sent to channel
				modbus.registers[7-output_channel] = output_sequence_buffer;
	
				// once have set 8 channels for four full cycle, pause and wait for inputs
				// we do this multiple times in order to ensure that the signal is settled
				if (output_sequence_count == 32)
				{	
					Enable_set_output = 0;	// stop setting outputs, wait till reading is done
					Enable_read_input = 1;	// given output is stopped, can now start reading inputs
	
					// increment/decrement testing signal and make sure it is between the range
					if (incrementing)
					{	testing_increment_init = testing_increment_init + TESTING_INCREMENT_VALUE;
						if (testing_increment_init == 1000)
							incrementing = 0;
					}
					else
					{	testing_increment_init = testing_increment_init - TESTING_INCREMENT_VALUE;
						if (testing_increment_init == 0)
							incrementing = 1;
					}
	
					// reset counter
					output_sequence_count = 0;
					// turn LEDs off and now start testing SOP
					if ( startup_flag != 0)
						startup_flag--;

				}
				// each cycle only sets one channel at a time
				output_sequence_count++;
			}
		



	}
//07/01/KR
#if defined (T3_8IN13OUT)
	if (digital_mode)
	{
		switch(output_channel)
			{
				case 8:

					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto)
					{	//if (reverse_logic_output)
							RELAY9 = 0;
					//	else
					//		RELAY9 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto )
					{//	if (reverse_logic_output)
							RELAY9 = 1;
					//	else
						//	RELAY9 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	//if (reverse_logic_output)
								RELAY9 = 1;
						//else
						//		RELAY9 = 0;
						}
						else
						{//	if (reverse_logic_output)
								RELAY9 = 0;
							//else
							//	RELAY9 = 1;
						}
				#if defined T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS
						LightOutput(8);
					#endif
					}
	

					break;			
				case 9:

					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto )
					{	//if (reverse_logic_output)
							RELAY10 = 0;
						//else
						//	RELAY10 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto )
					{	//if (reverse_logic_output)
							RELAY10 = 1;
					//	else
						//	RELAY10 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	//if (reverse_logic_output)
								RELAY10 = 1;
						//	else
						//		RELAY10 = 0;
						}
						else
						{	//if (reverse_logic_output)
								RELAY10 = 0;
						//	else
							//	RELAY10 = 1;
						}
				#if defined T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS
						LightOutput(9);
					#endif
					}
	

					break;
				case 10:

					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto )
					{//	if (reverse_logic_output)
							RELAY11 = 0;
					//	else
						//	RELAY11 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto )
					{	//if (reverse_logic_output)
							RELAY11 = 1;
					//	else
						//	RELAY11 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	//if (reverse_logic_output)
								RELAY11 = 1;
							//else
							//	RELAY11 = 0;
						}
						else
						{//	if (reverse_logic_output)
								RELAY11 = 0;
							//else
							//	RELAY11 = 1;
						}
		#if defined T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS
						LightOutput(10); 
					#endif
					}
			
					break;
				case 11:

					if (switch_state[output_channel] == 0 && output_sequence  && !switch_to_auto)
					{	//if (reverse_logic_output)
							RELAY12 = 0;
						//else
						//	RELAY12 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	//if (reverse_logic_output)
							RELAY12 = 1;
					//	else
						//	RELAY12 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	//if (reverse_logic_output)
								RELAY12 = 1;
							//else
							//	RELAY12 = 0;
						}
						else
						{	//if (reverse_logic_output)
								RELAY12 = 0;
						//	else
							//	RELAY12 = 1;
						}
			#if defined T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS
						LightOutput(11);
					#endif
					}
		
					break;	
				case 12:

					if (switch_state[output_channel] == 0 && output_sequence  && !switch_to_auto)
					{//	if (reverse_logic_output)
							RELAY13 = 0;
						//else
						//	RELAY13 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence  && !switch_to_auto)
					{//	if (reverse_logic_output)
							RELAY13 = 1;
					//	else
						//	RELAY13 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	//if (reverse_logic_output)
								RELAY13 = 1;
							//else
							//	RELAY13 = 0;
						}
						else
						{	//if (reverse_logic_output)
								RELAY13 = 0;
						//	else
						//		RELAY13 = 1;
						}
			#if defined T3_8IN13OUT
						LightOutput(12);
					#endif
					}
		

					break;	//4051 4052 34063 813 232 485 			
			}
	}

#endif
#if defined (T3_8IN16OUT)


	// --- evaluate the first set of output -----------------------------------------
	for (i = 0; i < 8; i++)
	{
		if (switch_state[i] == 2 && modbus.registers[i+16] > threshold)	// auto mode
			output_state[1] = (output_state[1] >> 1) | 0x80;
		else if (switch_state[i] == 1)					// hand mode
			output_state[1] = (output_state[1] >> 1) | 0x80;
		else												// off mode
			output_state[1] = output_state[1] >> 1;
	}
	// -------------------------------------------------------------------------------------


	// --- evaluate the second set of output -----------------------------------------
	for (i = 8; i < 16; i++)
	{
		if (switch_state[i] == 2 && modbus.registers[i-8] > threshold)	// auto mode
			output_state[0] = (output_state[0] >> 1) | 0x80;
		else if (switch_state[i] == 1)					// hand mode
			output_state[0] = (output_state[0] >> 1) | 0x80;
		else												// off mode
			output_state[0] = output_state[0] >> 1;
	}
	// --------------------------------------------------------------------------------------



	// --- Save state of status LEDs ------------------------------------------------------
	P0_buffer = P0;
	led_drive1_buf = LED_DRIVE1;
	led_drive2_buf = LED_DRIVE2;
	led_drive3_buf = LED_DRIVE3;
	led_drive4_buf = LED_DRIVE4;


	// Turn off status LEDs
	LED_DRIVE1 = 0;
	LED_DRIVE2 = 0;
	LED_DRIVE3 = 0;
	LED_DRIVE4 = 0;
	// -----------------------------------------------------------------------------------

	P0 = 0xFF;

	if (digital_mode)
	{
		if(output_channel % 2)
		{
			P0 = output_state[0];

			// rising edge will activate first batch of switches
//			delay_us(1);
			WRITEPORT1 = 1;
			WRITEPORT1 = 0;

		}
		else
		{
			P0 = output_state[1];

			// rising edge will activate second batch of switches
//			delay_us(1);
			WRITEPORT2 = 1;
			WRITEPORT2 = 0;
		}


	}


	// --- Restore state of status LEDs ------------------------------------------------------
	P0 = P0_buffer;
	LED_DRIVE1 = led_drive1_buf;
	LED_DRIVE2 = led_drive2_buf;
	LED_DRIVE3 = led_drive3_buf;
	LED_DRIVE4 = led_drive4_buf;
	// ---------------------------------------------------------------------------------------

#else
// turn relay on or off for 8output mode

	if (digital_mode)
	{
		switch(output_channel)
			{
				case 0:

					if (switch_state[output_channel] == 0 && output_sequence  && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY1 = 0;
						else
							RELAY1 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY1 = 1;
						else
							RELAY1 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY1 = 1;
							else
								RELAY1 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY1 = 0;
							else
								RELAY1 = 1;
						}
					#if defined T3_8IN13OUT
				
					
					LightOutput(0);
				
					#endif
					}
					
			

					break;


				case 1:
					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY2 = 0;
						else
							RELAY2 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY2 = 1;
						else
							RELAY2 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY2 = 1;
							else
								RELAY2 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY2 = 0;
							else
								RELAY2 = 1;
						}
					
						#if defined T3_8IN13OUT
	
						LightOutput(1);
						#endif
					}
					break;


				case 2:
					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto) 
					{	if (reverse_logic_output)
							RELAY3 = 0;
						else
							RELAY3 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY3 = 1;
						else
							RELAY3 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY3 = 1;
							else
								RELAY3 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY3 = 0;
							else
								RELAY3 = 1;
						}
			#if defined T3_8IN13OUT

						LightOutput(2);
					#endif
					}
		

					break;
				case 3:
					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY4 = 0;
						else
							RELAY4 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY4 = 1;
						else
							RELAY4 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY4 = 1;
							else
								RELAY4 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY4 = 0;
							else
								RELAY4 = 1;
						}
			#if defined T3_8IN13OUT
		
						LightOutput(3);
					#endif
					}
		

					break;

				case 4:
					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY5 = 0;
						else
							RELAY5 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY5 = 1;
						else
							RELAY5 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY5 = 1;
							else
								RELAY5 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY5 = 0;
							else
								RELAY5 = 1;
						}
					#if defined T3_8IN13OUT
		
						LightOutput(4);
					#endif
					}
		

					break;


				case 5:
					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY6 = 0;
						else
							RELAY6 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY6 = 1;
						else
							RELAY6 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY6 = 1;
							else
								RELAY6 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY6 = 0;
							else
								RELAY6 = 1;
						}
				#if defined T3_8IN13OUT			

						LightOutput(5);
					#endif
					}
	

					break;


				case 6:
					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY7 = 0;
						else
							RELAY7 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY7 = 1;
						else
							RELAY7 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY7 = 1;
							else
								RELAY7 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY7 = 0;
							else
								RELAY7 = 1;
						}
			#if defined T3_8IN13OUT
				
						LightOutput(6);
					#endif
					}
		

					break;


				case 7:
					if (switch_state[output_channel] == 0 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY8 = 0;
						else
							RELAY8 = 1;
					}
					else if (switch_state[output_channel] == 1 && output_sequence && !switch_to_auto)
					{	if (reverse_logic_output)
							RELAY8 = 1;
						else
							RELAY8 = 0;
					}
					else
					{
						if (modbus.registers[output_channel] > threshold)
						{	if (reverse_logic_output)
								RELAY8 = 1;
							else
								RELAY8 = 0;
						}
						else
						{	if (reverse_logic_output)
								RELAY8 = 0;
							else
								RELAY8 = 1;
						}
					#if defined T3_8IN13OUT
						LightOutput(7);
					#endif
					}
			

					break;
							
					
			}


	}
	else
	{	// analog mode
			int analog_temp;
			if (switch_state[7-output_channel] == 0 && output_sequence)
				PWMD3 = look_up_table(0);
			else if (switch_state[7-output_channel] == 1 && output_sequence)
	//			PWMD3 = look_up_table(1000+output_calibration-CALIBRATION_OFFSET);
				PWMD3 = look_up_table(1000);
			else
			{			
				if(guiDeadmasterTime != 0)
				{
					analog_temp = modbus.registers[7-output_channel]/*+output_calibration-CALIBRATION_OFFSET*/;
					//output_buffer = output_calibration;
					if (analog_temp < 0)
						analog_temp = 0;
					PWMD3 = look_up_table(analog_temp);
					//PWMD3 = analog_temp ;
				}			
				else
				{
				
					PWMD3 = look_up_table(deadmaster_output[output_channel]);
				}
			}
		}

		


#endif




}
#endif



#if defined T3_8IN13OUT

bit GetWBit(unsigned char bit_number,unsigned int *word)
{
	unsigned int mask; 
	mask = 0x01;	 
	mask = mask << bit_number;
	return (bit)(*word & mask);
}

void LightOutput(unsigned char overOut)
{
 
	if(overOut == 0 && gucZone[0])
	{
 //tbd: MAKE A LOOP FOR THIS CODE, NOT STRIAGHT LINE CODE
		if(GetWBit(0,&guiManual) == 1 && gucOverOutput[gucZone[0]-1] == 1)
		{
 
			if (gucReverseOutput)
				RELAY1 = 1;
			else
				RELAY1 = 0;
			gucStatus[0] = 1;
		}
		else 
			gucStatus[0] = 0;
	}
	else if(overOut == 1 && gucZone[1])
	{
		if(GetWBit(1,&guiManual) == 1 && gucOverOutput[gucZone[1]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY2 = 1;
			else
				RELAY2 = 0;
			gucStatus[1] = 1;
		}
		else 
			gucStatus[1] = 0;
	}
	else if(overOut == 2 && gucZone[2])
	{
		if(GetWBit(2,&guiManual) == 1 && gucOverOutput[gucZone[2]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY3 = 1;
			else
				RELAY3 = 0;
			gucStatus[2] = 1;
		}
		else 
			gucStatus[2] = 0;
	}
	else if(overOut == 3 && gucZone[3])
	{
		if(GetWBit(3,&guiManual) == 1 && gucOverOutput[gucZone[3]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY4 = 1;
			else
				RELAY4 = 0;
			gucStatus[3] = 1;
		}
		else 
			gucStatus[3] = 0;
	}
	else if(overOut == 4 && gucZone[4])
	{
		if(GetWBit(4,&guiManual) == 1 && gucOverOutput[gucZone[4]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY5 = 1;
			else
				RELAY5 = 0;
			gucStatus[4] = 1;
		}
		else 
			gucStatus[4] = 0;
	
	}
	else if(overOut == 5 && gucZone[5])
	{
		if(GetWBit(5,&guiManual) == 1 && gucOverOutput[gucZone[5]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY6 = 1;
			else
				RELAY6 = 0;
			gucStatus[5] = 1;
		}
		else 
			gucStatus[5] = 0;
	}
	else if(overOut == 6 && gucZone[6])
	{
		if(GetWBit(6,&guiManual) == 1 && gucOverOutput[gucZone[6]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY7 = 1;
			else
				RELAY7 = 0;
			gucStatus[6] = 1;
		}
		else 
			gucStatus[6] = 0;
	}
	else if(overOut == 7 && gucZone[7])
	{
		if(GetWBit(7,&guiManual) == 1 && gucOverOutput[gucZone[7]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY8 = 1;
			else
				RELAY8 = 0;
			gucStatus[7] = 1;
		}
		else 
			gucStatus[7] = 0;
	}
	else if(overOut == 8 && gucZone[8])
	{
		if(GetWBit(8,&guiManual) == 1 && gucOverOutput[gucZone[8]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY9 = 1;
			else
				RELAY9 = 0;
			gucStatus[8] = 1;
		}
		else 
			gucStatus[8] = 0;
	}
	else if(overOut == 9 && gucZone[9])
	{
		if(GetWBit(9,&guiManual) == 1 && gucOverOutput[gucZone[9]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY10 = 1;
			else
				RELAY10 = 0;
			gucStatus[9] = 1;
		}
		else 
			gucStatus[9] = 0;
	}
	else if(overOut == 10 && gucZone[10])
	{
		if(GetWBit(10,&guiManual) == 1 && gucOverOutput[gucZone[10]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY11 = 1;
			else
				RELAY11 = 0;
			gucStatus[10] = 1;
		}
		else 
			gucStatus[10] = 0;
	}
	else if(overOut == 11 && gucZone[11])
	{
		if(GetWBit(11,&guiManual) == 1 && gucOverOutput[gucZone[11]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY12 = 1;
			else
				RELAY12 = 0;
			gucStatus[11] = 1;
		}
		else 
			gucStatus[11] = 0;
	}
	else if(overOut == 12 && gucZone[12])
	{
		if(GetWBit(12,&guiManual) == 1 && gucOverOutput[gucZone[12]-1] == 1)
		{
			if (gucReverseOutput)
				RELAY13 = 1;
			else
				RELAY13 = 0;
			gucStatus[12] = 1;
		}
		else 
			gucStatus[12] = 0;
	}
}
/*void LightOutput(unsigned char overOut)
{
	if(overOut == 0)
	{
		if(GetWBit(0,&guiManual) == 1 && gucOverOutput[0] == 1)
		{
			if (reverse_logic_output)
				RELAY1 = 1;
			else
				RELAY1 = 0;
		}
	}
	else if(overOut == 1)
	{
		if(GetWBit(1,&guiManual) == 1 && gucOverOutput[1] == 1)
		{
			if (reverse_logic_output)
				RELAY2 = 1;
			else
				RELAY2 = 0;
		}
	}
	else if(overOut == 2)
	{
		if(GetWBit(2,&guiManual) == 1 && gucOverOutput[2] == 1)
		{
			if (reverse_logic_output)
				RELAY3 = 1;
			else
				RELAY3 = 0;
		}
	}
	else if(overOut == 3)
	{
		if(GetWBit(3,&guiManual) == 1 && gucOverOutput[3] == 1)
		{
			if (reverse_logic_output)
				RELAY4 = 1;
			else
				RELAY4 = 0;
		}
	}
	else if(overOut == 4)
	{
		if(GetWBit(4,&guiManual) == 1 && gucOverOutput[4] == 1)
		{
			if (reverse_logic_output)
				RELAY5 = 1;
			else
				RELAY5 = 0;
		}
	
	}
	else if(overOut == 5 )
	{
		if(GetWBit(5,&guiManual) == 1 && gucOverOutput[5] == 1)
		{
			if (reverse_logic_output)
				RELAY6 = 1;
			else
				RELAY6 = 0;
		}
	}
	else if(overOut == 6 )
	{
		if(GetWBit(6,&guiManual) == 1 && gucOverOutput[6] == 1)
		{
			if (reverse_logic_output)
				RELAY7 = 1;
			else
				RELAY7 = 0;
		}
	}
	else if(overOut == 7 )
	{
		if(GetWBit(7,&guiManual) == 1 && gucOverOutput[7] == 1)
		{
			if (reverse_logic_output)
				RELAY8 = 1;
			else
				RELAY8 = 0;
		}
	}
	else if(overOut == 8)
	{
		if(GetWBit(8,&guiManual) == 1 && gucOverOutput[8] == 1)
		{
			if (reverse_logic_output)
				RELAY9 = 1;
			else
				RELAY9 = 0;
		}
	}
	else if(overOut == 9)
	{
		if(GetWBit(9,&guiManual) == 1 && gucOverOutput[9] == 1)
		{
			if (reverse_logic_output)
				RELAY10 = 1;
			else
				RELAY10 = 0;
		}
	}
	else if(overOut == 10)
	{
		if(GetWBit(10,&guiManual) == 1 && gucOverOutput[10] == 1)
		{
			if (reverse_logic_output)
				RELAY11 = 1;
			else
				RELAY11 = 0;
		}
	}
	else if(overOut == 11)
	{
		if(GetWBit(11,&guiManual) == 1 && gucOverOutput[11] == 1)
		{
			if (reverse_logic_output)
				RELAY12 = 1;
			else
				RELAY12 = 0;
		}
	}
	else if(overOut == 12)
	{
		if(GetWBit(12,&guiManual) == 1 && gucOverOutput[12] == 1)
		{
			if (reverse_logic_output)
				RELAY13 = 1;
			else
				RELAY13 = 0;
		}
	}
}*/
#endif


// ******************************************************************************************************
// defines for PWM output
// ******************************************************************************************************

#ifndef T3_32IN

//unsigned char const code def_tab[51] =
//			{				
////			 1036, 1021, 1007, 993, 978, 961, 943, 926, 906, 885, 863, 837, 809, 778, 743, 703, 657, 605, 549, 494, 441, 394, 351, 315, 284, 258, 235, 215, 196, 180, 164, 150, 138, 126, 115, 105, 95, 86, 78, 70, 63, 56, 49, 43, 37, 31, 25, 20, 15, 10, 6
//		//		6, 10, 15, 20, 25, 31, 37 ,43 ,49 ,56 ,63 ,70 ,78 ,86 ,95 ,105 ,115 ,126, 138, 150 ,164 ,180 ,196 ,215 ,235 ,258 ,284 ,315 ,351 ,394 ,441 ,494 ,549 ,605 ,657 ,703 ,743 ,778 ,809 ,837 ,863 ,885 ,906 ,926 ,943 ,961 ,978 ,993 ,1007 ,1021 ,1036
// //				6, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 14, 16, 16, 19, 20, 23, 26, 31, 36, 43, 47, 53, 55, 56, 52, 46, 42, 37, 31, 28, 26, 22, 21, 20, 19, 18, 17, 15, 14, 14, 15
////Rev10				4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 10, 10, 11, 12, 13, 14, 15, 17, 18, 21, 23, 26, 30, 37, 41, 47, 55, 57, 59, 53, 45, 43, 37, 31, 30, 26, 25, 21, 23, 17, 20, 18, 17, 13, 15
//		
//				2,5,5,5,6,6,6,6,7,7,7,8,8,9,9,10,11,12,12,13,14,16,18,19,21,23,27,30,37,42,47,54,58,59,55,49,44,38,35,30,29,24,24,21,21,19,17,17,16,15,15

//			};

unsigned char const code def_tab[10] = {80,118,139,152,162,171,182,195,214,242} ;

unsigned int   look_up_table(unsigned int count)
{
	int   val;
	unsigned int i =1 ;
	if(count > 1000) count = 1000 ;
	if(count<100)
	{
				return val = (int)(0.8 *count) ;
	}
	else
	{
		while(1) 
		{
			i++ ;
			if(count<=100*i)
			{
					return val = (int)(def_tab[i-1]-def_tab[i-2])*count/100 + (int)def_tab[i-2]*(i)-(int)def_tab[i-1]*(i-1);
			}
		
		}	
	
	}
//	int   val;
//    char  index=0;
//	int   work_var= def_tab[0];

//	if (work_var > count )
//		{
//			val =  0 ;
//			return ( val );
//		}

//	do 
//		{
//			index++;
//			work_var += def_tab[index] ;

//			if( work_var > count )
//				{
//				val = def_tab[index] - (work_var - count);
//				val *= 10;
//				val =  val / def_tab[index];
//				val = (val * 5) / 10;
//				val += (index * 5);
//				return (val);
//				}
//		} while (index) ;

//			val =  255 ;
//			return ( val );
}


#endif

