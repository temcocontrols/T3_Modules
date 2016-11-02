#include <KEIL_VRS1000_regs.H>
#include "LibIO_T3IO.h"
#include "define.h"
#include "T3_IO.h"
//#include "revision.h"
#include "math.h"		// to use the abs function, August Ron

//#include "T3IOmodbus.h"
//extern bit flag_comm;
extern unsigned char xdata com_beat ;
extern unsigned char xdata SERIAL_RECEIVE_TIMEOUT;
extern unsigned char xdata reading_filter_bypass;
unsigned  char  xdata info[20]; 
#ifdef  T3_8IO_A
extern unsigned int xdata deadmaster_output[8];
#endif

#ifdef T3_32IN
extern unsigned int  xdata  old_reading[32];
extern unsigned char xdata filter[32];
extern unsigned char xdata gucFilterCounter[32];
unsigned int xdata range_lo = 0 ;
unsigned int xdata range_hi = 500 ;
#else
extern unsigned char xdata gucFilterCounter[8];
extern unsigned long xdata  old_reading[8];
extern unsigned char xdata filter[8];
unsigned char xdata gucPreviousInput[8] _at_ 0x2F0;
#endif
extern bit  WritePicCommand(unsigned char channel);
extern bit WritePicType(unsigned char channelType);

 
extern unsigned char xdata gucDeadmaster;
extern unsigned long xdata  guiDeadmasterTime;
extern bit serial_no_activity;
extern bit switch_to_auto;
unsigned int xdata threshold; 

extern void iap_erase_block(unsigned char block);
extern bit GetBit(unsigned char bit_number,unsigned char *byte);
#ifndef T3_32IN
extern unsigned int  xdata guiBuffer[8][10];
extern unsigned char xdata gucCounter[8];
extern unsigned char xdata gucStartFilter;


#endif
extern unsigned char xdata gucReverseOutput;
//#define LUIS 			//	this guy need write 0 close the relay and 1 open the relay,not 500
extern void iap_erase_block(unsigned char block);
extern signed int RangeConverter(unsigned char function, signed int para,signed int cal,unsigned char i);
//extern signed int   look_up_table1(unsigned int count);
unsigned char xdata com_dealy;
void CRC16_Tstat(unsigned char ch);
void InitCRC16(void);
int xdata calibration_offset[NUM_INPUTS];
//unsigned char xdata dec[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
extern bit transmit_finished;
extern signed int xdata guiAnalogInput[NUM_INPUTS];
/*void debug_print(unsigned int temp)
{
	unsigned char xdata p[7];
	unsigned char xdata i;
	
	p[0] = dec[temp / 10000];
	p[1] = dec[temp % 10000 / 1000];
	p[2] = dec[temp % 1000 / 100];
	p[3] = dec[temp % 100 / 10];
	p[4] = dec[temp % 10];
	p[5] = 32;
	p[6] = 32;
	
	for(i = 0; i < 7; i++)
	{
		RS232STATE = SEND;
		if(p[i]!= 0)
		{
			transmit_finished = 0;
			SBUF = p[i];
			while (!transmit_finished);
		}
	}	
}*/

main()
{

	unsigned char i = 0;
	unsigned char cnt = 0;
//	signed int aa;
 
 	P2 = 0;
	P1 = 0;
	P4 = 0;
 
 

	initial();
 		 	
 

	
	#ifdef  T3_8IO_A
		hardware_rev = 0;
	#else
		hardware_rev = 3;
	#endif


	// set the hardware according to the input from user
	set_hardware();
	pic_detect();

	#if defined T3_8IN13OUT
		gbSetChannel = 1;
		gucSetChannel = channel_type;
		WritePicType(channel_type);
	#endif
	#if defined LUIS
		threshold = 0;
	#elif defined T3_8IN13OUT
		threshold = 0;
	#else 
	    threshold = 500;
	#endif
	

	while(1)
	{
//		if(gucDeadmaster && (guiDeadmasterTime == 0))
//		{
//		//initSerial();		
//		}
		
		if(new_heartbeat)     // run the route every heart beat
		{
			new_heartbeat = FALSE;
		
			// serial message has been received, now want to deal with data or receive response
			if (DealwithTag)
			{
				DealwithTag--;
				if( (DealwithTag==1) )//&& !Serial_Master )
					dealwithData();
			}


			// serial timeout will stop the serial if there was ever a lock in there
			if (serial_receive_timeout_count > 0)
			{

				serial_receive_timeout_count--;

				if (serial_receive_timeout_count == 0)
				{
					serial_restart();
					delay_us(10);
				}

			}
			// --------------------------------------------------------------------------------
			// --------------------------------------------------------------------------------


			// --------------------------------------------------------------------------------
			// --- specifications according to devices ----------------------------------------
			#if defined (PWM_TRANSDUCER)
				check_digital_inputs();
			#endif
			// --------------------------------------------------------------------------------

			// --- special features -----------------------------------------------------------
			#ifdef CALIBRATION_OPTION

			// check if calibration has been used...
				if( calibration_address != 0xFFFF)
				{	store_data_to_grid(calibration_address);
					calibration_address = 0xFFFF;
				}

				if( output_sequence == 5 )
				{	// pr-oduce LUT for calibration
					produce_LUT();
					// set SOP back to normal operation
					flash_write_int(FLASH_SEQUENCE, 1, FLASH_MEMORY);
					output_sequence = 1;
				}

			#endif//


			// --------------------------------------------------------------------------------
		}
 
		// MDF changed this to 19 to make the primary pulses 50ms
		// Ron changed this to 3 to increase pic reading.  Needed for 32in module
		if ( heart_beat > 3 ) // The heartbeat flag is set to 1 by the timer0 interrupt function
		{
			heart_beat = 0 ;
			watchdog ( ) ;
 
			tabulate_LED_STATE();
 		 	 
 
			// For each event timer, reduce the remaining number of ticks by one.
			// If the number of ticks has reached 0, push the event into the event queue
			for( timer_no =	0 ; timer_no < TOTAL_TIMERS; timer_no++	)
			{
				//watchdog ( );
							
				temp_unsigned_char = big_ticks[timer_no] ;   //load the remaining ticks of the event timer	    
		
				// If the event timer is paused (ticks = 0) do not push the event or reduce the ticks
				if( temp_unsigned_char != 0 ) 
				{			  
					temp_unsigned_char--;		   
					big_ticks[timer_no] = temp_unsigned_char; // Reduce remaining ticks
	
					if( temp_unsigned_char == 0 ) 
						{	
							push_event( timer_no ); // Add event to event queue
						}
				}
			}
			// --------------------------------------------------------------------------------
			// --------------------------------------------------------------------------------



		}
	 

		// For each event in the queue, activate the corresponding sub-routine
		while(event_count )	  
			{
				watchdog ( ) ;
				event_count-- ;
				switch(event_queue[event_count])
					{
						case HEARTBEAT_PULSE:
							cnt++;
							
							if(cnt >= 6)
							{
								cnt = 0;
								pulse_flag = !pulse_flag;
							 
							}
				
							start_timer(HEARTBEAT_PULSE , DEFAULT_TIMEOUT );
							
							break;

						case REFRESH_OUTPUTS:
							//pulse_flag = !pulse_flag;
							//start_timer(REFRESH_INPUTS , DEFAULT_TIMEOUT );
							#ifndef T3_32IN
								refresh_outputs ();
							#endif
 
							//debug_print(modbus.registers[15]);
							start_timer(REFRESH_OUTPUTS , DEFAULT_TIMEOUT );
							break ;

						case REFRESH_INPUTS:
							// keep looking for pic until one has been found							
							if(pic_type==0)
								pic_detect();
 

							if(gbClearPulse == 1)
							{
								gbClearPulse = 0;
								for(i=0;i<5;i++)
								{
									if(WritePicCommand(gucClearPulse) == 1)
									break;
									delay_us(5);
							
								}
							}
							else if(gbSetChannel)
							{
								gbSetChannel = 0;
								for(i=0;i<5;i++)
								{
									if(WritePicType(gucSetChannel) == 1)
									break;
									delay_us(5);
							
								}
								
							} 
							else
							{
							  
						  	refresh_inputs ();
							  
 
							}
							start_timer(REFRESH_INPUTS , DEFAULT_TIMEOUT );	 													

							break ;	  

						/*case FLEXDRIVER :

							// want to send complete grid data to refresh every once in a while
							// thus perform a multiple write
							#ifdef FLEXDRIVER_FEATURE
								if(send_table_error_flag || send_table_counter == 0)
								{
									master_com_send_table(20, FlexDriver_LED_STATUS_PAIR1, 32);
									send_table_counter = TABLE_REFRESH_RATE;
								}
								else
									send_table_counter--;
							
							#endif

							start_timer(FLEXDRIVER , DEFAULT_TIMEOUT );
							break ;	  */

//						case TIMER_SOP:	// September RON

///							#ifdef TIMER_SOP_FEATURE
								//--------------------- Timer SOP feature ------------------------------------------------------
/*								if(digital_mode)
								{	
							
									// in digital mode, input is set to low
									if(modbus.registers[8]<512)
									{	
										// we have just switched to the UN-OCCUPIED state
										switching_flag_present = 0;
							
										if ( switching_flag_previous != switching_flag_present)
										{	//means just switched states, need to write to everyonez
											switching_flag_previous = 0;
											SNWriteflag &= 0x7F;
											flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, SNWriteflag, FLASH_MEMORY);
								
											// must SET the error_check[15] MSB given it is control bit, look at timer_SOP.h file
											error_check[15] = error_check[15] | 0x8000;
								
											// reset id to start from fresh
											timer_id = START_ID;
										}
							
										timer_serial_signal( START_ID , END_ID, MODBUS_FAN_SPEED, 0);
							
									}
									// in digital mode, if input 1 is set to a high, everything AUTO
									else if(modbus.registers[8]>512)
									{	
										// we have just switched to the OCCUPIED state
										switching_flag_present = 1;
							
										if ( switching_flag_previous != switching_flag_present)
										{	//means just switched states, need to write to everyone
											switching_flag_previous = 1;
											SNWriteflag |= 0x80;
											flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, SNWriteflag, FLASH_MEMORY);
								
											// must SET the error_check[15] MSB given it is control bit, look at timer_SOP.h file
											error_check[15] = error_check[15] | 0x8000;
								
											// reset id to start from fresh
											timer_id = START_ID;
							
										}
							
										timer_serial_signal( START_ID , END_ID, MODBUS_FAN_SPEED, 4);	
							
									}
							
							
								}
								// ---------------------------------------------------------------------------------------------------------
							
								start_timer(TIMER_SOP , DEFAULT_TIMEOUT );
*/
//							#endif

//							break ;

/*						case INPUT_PWM_TIMER :

							#ifdef PWM_TRANSDUCER
								PWM_timer_count();
								start_timer(INPUT_PWM_TIMER , DEFAULT_TIMEOUT );														
							#endif

							break ;*/
 
/*						case HIGH_SPEED :

							#ifdef REX_FEATURES
								// store data into array
								high_speed_buffer[high_speed_index] = high_speed_counter;
								// then reset counter
								high_speed_counter = 0;
								// then update index
								if( high_speed_index < HIGH_BUFFER )
									high_speed_index++;
								else
								{
									high_speed_index = 0;
								}
	
									start_timer(HIGH_SPEED , DEFAULT_TIMEOUT );
								
								temp_sum = 0;	// reset cummulated sum
								
								for(i=0; i<HIGH_BUFFER; i++)
									temp_sum += high_speed_buffer[i];	// tabulate total sum
								
								modbus.registers[EEP_IN_COUNT_LO] = temp_sum & 0x00FF;			// seperate into two char
								modbus.registers[EEP_IN_COUNT_HI] = (temp_sum >> 8) & 0x00FF;	// seperate into two char

							#endif

							break ;	  
*/

						case CHECK_SWITCHES :


							#if defined (T3_8IN16OUT)
								check_switches(select_switch);
								select_switch = !select_switch;

								// ensure driver chip remains ON
								ENABLE_2003 = 0;	//	 active high, thus 1 is ON
								start_timer(CHECK_SWITCHES , DEFAULT_TIMEOUT );
								
							#elif defined (T3_8IO_REV8) || (T3_8IO_A) || (T3_8IO) || (T3_8IO_REV6)

								check_switches(LO);
								start_timer(CHECK_SWITCHES , DEFAULT_TIMEOUT );
							#elif defined(T3_8IN13OUT)
								check_switches(select_switch);
								select_switch = !select_switch;
								start_timer(CHECK_SWITCHES , DEFAULT_TIMEOUT );
							#endif

							break ;
#ifdef  T3_8IN13OUT
			 			case INPUT1_TIMER:
							gucMinuteCounter[0]++;
 							
							if(gucMinuteCounter[0] > MINUTE_COUNTER)
							{
								gucMinuteCounter[0] = 0;
 								
								if(gucTimerLeft[0] > 0)
								{
									gucTimerLeft[0]--;
								 
							 
								}
								else
								{

								 	
										gucTimerLeft[0] = 0;
										gucOverOutput[0] = 0;
										stop_timer(INPUT1_TIMER);
										break;
								 
								}
								
								
							}
							start_timer(INPUT1_TIMER,DEFAULT_TIMEOUT);

						break;
			 			case INPUT2_TIMER:
							gucMinuteCounter[1]++;
							if(gucMinuteCounter[1] > MINUTE_COUNTER)
							{
								gucMinuteCounter[1] = 0;
								if(gucTimerLeft[1] > 0)
								{
									gucTimerLeft[1]--;
								 
							 
								}
								else
								{

							 
										gucTimerLeft[1] = 0;
										gucOverOutput[1] = 0;
										stop_timer(INPUT2_TIMER);
										break;
									 
								}
								
							}
							start_timer(INPUT2_TIMER,DEFAULT_TIMEOUT);

						break;
			 			case INPUT3_TIMER:
							gucMinuteCounter[2]++;
							if(gucMinuteCounter[2] > MINUTE_COUNTER)
							{
								gucMinuteCounter[2] = 0;
								if(gucTimerLeft[2] > 0)
								{
									gucTimerLeft[2]--;
								 
							 
								}
								else
								{

								 
										gucTimerLeft[2] = 0;
										gucOverOutput[2] = 0;
										stop_timer(INPUT3_TIMER);
										break;
									 
								}
								
							}
							start_timer(INPUT3_TIMER,DEFAULT_TIMEOUT);

						break;
			 			case INPUT4_TIMER:
							gucMinuteCounter[3]++;
							if(gucMinuteCounter[3] > MINUTE_COUNTER)
							{
								gucMinuteCounter[3] = 0;
								if(gucTimerLeft[3] > 0)
								{
									gucTimerLeft[3]--;
								 
							 
								}
								else
								{

								 
										gucTimerLeft[3] = 0;
										gucOverOutput[3] = 0;
										stop_timer(INPUT4_TIMER);
										break;
									 
								}
								
							}
							start_timer(INPUT4_TIMER,DEFAULT_TIMEOUT);

						break;
			 			case INPUT5_TIMER:
							gucMinuteCounter[4]++;
							if(gucMinuteCounter[4] > MINUTE_COUNTER)
							{
								gucMinuteCounter[4] = 0;
								if(gucTimerLeft[4] > 0)
								{
									gucTimerLeft[4]--;
								 
							 
								}
								else
								{

								 
										gucTimerLeft[4] = 0;
										gucOverOutput[4] = 0;
										stop_timer(INPUT5_TIMER);
										break;
									 
								}
								
							}
							start_timer(INPUT5_TIMER,DEFAULT_TIMEOUT);

						break;
			 			case INPUT6_TIMER:
							gucMinuteCounter[5]++;
							if(gucMinuteCounter[5] > MINUTE_COUNTER)
							{
								gucMinuteCounter[5] = 0;
								if(gucTimerLeft[5] > 0)
								{
									gucTimerLeft[5]--;
								 
							 
								}
								else
								{

								 
										gucTimerLeft[5] = 0;
										gucOverOutput[5] = 0;
										stop_timer(INPUT6_TIMER);
										break;
								 
								}
								
							}
							start_timer(INPUT6_TIMER,DEFAULT_TIMEOUT);

						break;
			 			case INPUT7_TIMER:
							gucMinuteCounter[6]++;
							if(gucMinuteCounter[6] > MINUTE_COUNTER)
							{
								gucMinuteCounter[6] = 0;
								if(gucTimerLeft[6] > 0)
								{
									gucTimerLeft[6]--;
								 
							 
								}
								else
								{

									 
										gucTimerLeft[6] = 0;
										gucOverOutput[6] = 0;
										stop_timer(INPUT7_TIMER);
										break;
								 
								}
								
							}
							start_timer(INPUT7_TIMER,DEFAULT_TIMEOUT);

						break;
			 			case INPUT8_TIMER:
							gucMinuteCounter[7]++;
 
							if(gucMinuteCounter[7] > MINUTE_COUNTER)
							{
								gucMinuteCounter[7] = 0;
 
								if(gucTimerLeft[7] > 0)
								{
									gucTimerLeft[7]--;
								 
							 
								}
								else
								{

								 
										gucTimerLeft[7] = 0;
										gucOverOutput[7] = 0;
										stop_timer(INPUT8_TIMER);
										break;
								 
								}
								
							}
							start_timer(INPUT8_TIMER,DEFAULT_TIMEOUT);

						break;
#endif
			 
			 
						case STORE_PULSE :						
 
						#if defined T3_8IN13OUT
							StorePulseToFlash( ) ;
							start_timer(STORE_PULSE , DEFAULT_TIMEOUT );
						#endif
							break ;


						default :
						break ;

					}
			}

	}
}
// *****************************************************************************************************
// INITIALISE functions
// *****************************************************************************************************


void initial()
{
	unsigned char xdata i, temp_read;
	unsigned int xdata pulse_buf;
	
	P0 = 0;
	// --- initialise flash memory ---
	flash_init();
	reading_filter_bypass = 30;

	//guiDeadmasterTime = 0;
	
	//gucDeadmaster = 0;



	for(i=0; i<NUM_INPUTS; i++)
	{
		if((!flash_read_int(FLASH_INPUT0_CALIBRATION + i,&pulse_buf,FLASH_MEMORY))|| (pulse_buf == 0))
		{
			pulse_buf = 500 ;
			flash_write_int(FLASH_INPUT0_CALIBRATION + i,pulse_buf, FLASH_MEMORY);		
		}
		calibration_offset[i] = pulse_buf ;		
	}
	if(!flash_read_int(FLASH_DEAD_MASTER,&pulse_buf,FLASH_MEMORY))
	{
		pulse_buf = 60;
		flash_write_int(FLASH_DEAD_MASTER, pulse_buf, FLASH_MEMORY);
	}
	if(pulse_buf == 0 )
	{
		pulse_buf = 60;
		flash_write_int(FLASH_DEAD_MASTER, pulse_buf, FLASH_MEMORY);
	}
	gucDeadmaster = pulse_buf ;
	guiDeadmasterTime = (long)gucDeadmaster*24000;
	#if defined T3_8IN13OUT

	for(i=0;i<8;i++)
	{
		old_reading[i] = 0;
		gucPreviousInput[i] = 1;
	
	}

	// initialise ram memory
	if(!flash_read_int(FLASH_START_PULSE,&pulse_buf,FLASH_MEMORY))
 
	{
		if(!flash_read_int(FLASH_START_PULSE + 1,&pulse_buf,FLASH_MEMORY))
 
		{
			pulse_buf = 0;
			for(i=0;i<16;i++)
			{
				flash_write_int(FLASH_START_PULSE + i,pulse_buf,FLASH_MEMORY);
				flash.pulse_number[i<<1] = 0;
				flash.pulse_number[(i<<1) + 1] = 0;
			}
		}
		else
		{				
			for(i=0;i<16;i++)
			{
				flash_read_int(FLASH_START_PULSE + i,&pulse_buf,FLASH_MEMORY);		 
				flash.pulse_number[i<<1] = (pulse_buf >> 8) & 0xff;
				flash.pulse_number[(i<<1) + 1] = pulse_buf & 0xff;
			}
		}
	}
	else
	{
		for(i=0;i<16;i++)
		{
			flash_read_int(FLASH_START_PULSE + i,&pulse_buf,FLASH_MEMORY);		 
			flash.pulse_number[i<<1] = (pulse_buf >> 8) & 0xff;
			flash.pulse_number[(i<<1) + 1] = pulse_buf & 0xff;
		}
	}
	if(!flash_read_int(FLASH_FLASH_WRITE_TIME,&pulse_buf,FLASH_MEMORY))
	{
		pulse_buf = 100 ;
		flash_write_int(FLASH_FLASH_WRITE_TIME,pulse_buf,FLASH_MEMORY);	
	}
	flash_write_count = pulse_buf ;
	for(i=0;i<32;i++)
	pic.pulse_number[i] = 0;
	watchdog();
	for(i=16;i<56;i++)
	{
		pulse_buf = 0;
		flash_read_int(FLASH_START_PULSE + i,&pulse_buf,FLASH_MEMORY);			 
		pulse_number[i-16] = pulse_buf & 0xff;
	}


	if(!flash_read_int(FLASH_CHANNEL_TYPE,&pulse_buf,FLASH_MEMORY))			 
	channel_type = 0xff;
	else
	channel_type = pulse_buf & 0xff;



	watchdog();
/*	for(i=0;i<8;i++)
	{
		if(GetBit(i,&channel_type))
		{		
			range[i] = 6;
			flash_write_int(FLASH_INPUT1_RANGE + i, range[i], FLASH_MEMORY);		
		}
	}*/

	for(i=0;i<8;i++)
	{
		
		if(!flash_read_int(FLASH_INPUT1_RANGE + i,&pulse_buf,FLASH_MEMORY))			 
			range[i] = 0;
	
		else
		{
			if(pulse_buf > 20)
			pulse_buf = 0;
	 		range[i] = pulse_buf;
		}	
	
	}
	watchdog();
	for(i=0;i<8;i++)
	{
		if(!flash_read_int(FLASH_INPUT1_FILTER + i,&pulse_buf,FLASH_MEMORY))
		{
		 	pulse_buf = 2;	
			flash_write_int(FLASH_INPUT1_FILTER + i, 2, FLASH_MEMORY);	
		}
		filter[i] = pulse_buf & 0xff;
	 
	}
	watchdog();

	for(i=0;i<8;i++)
	{
		if(!flash_read_int(FLASH_INPUT1_TIMER + i,&pulse_buf,FLASH_MEMORY))
		{
		 	pulse_buf = 60;	
			flash_write_int(FLASH_INPUT1_TIMER + i, 60, FLASH_MEMORY);	
		}
		gucTimer[i] = pulse_buf & 0xff;
		gucTimerLeft[i] = 0;
		gucMinuteCounter[i] = 0;
		gucTimerFilter[i] = 0;
	 	gucOverOutput[i] = 0;
		gucPreviousInput[i] = 1;
		gucFilterCounter[i] = 0;
		
	}

   	if(!flash_read_char(FLASH_RESPOND_DELAY , &pulse_buf,FLASH_MEMORY))
	{
		pulse_buf = 2;	
		flash_write_int(FLASH_RESPOND_DELAY, pulse_buf, FLASH_MEMORY);	
	}
	com_dealy = pulse_buf ;
	watchdog();
	if(!flash_read_int(FLASH_OUTPUT_MANUAL,&pulse_buf,FLASH_MEMORY))
	{
	 	pulse_buf = 0;	
		flash_write_int(FLASH_OUTPUT_MANUAL, 0, FLASH_MEMORY);	
	}
	guiManual= pulse_buf; 
	if(!flash_read_int(FLASH_REVERSE_OUTPUT,&pulse_buf,FLASH_MEMORY))
	{
	 	pulse_buf = 1;	
		flash_write_int(FLASH_REVERSE_OUTPUT, 1, FLASH_MEMORY);	
	}
	gucReverseOutput = pulse_buf; 





 	for(i=0;i<13;i++)
	{
		if(!flash_read_int(FLASH_ZONE_OUTPUT1 + i,&pulse_buf,FLASH_MEMORY))
		{
		 	pulse_buf = 0;	
			flash_write_int(FLASH_ZONE_OUTPUT1 + i, 0, FLASH_MEMORY);	
		}
		gucZone[i] = pulse_buf & 0xff;
		gucStatus[i] = 0;	 
	 
	}

	#endif
#ifdef T3_32IN
		reading_filter_bypass = 255;
 		// channels_init();
		flash_read_int(FLASH_init, &pulse_buf, FLASH_MEMORY);
		if(pulse_buf != 0x55)
		{
			flash_write_int(FLASH_init , 0x55, FLASH_MEMORY);	
			for(i=0;i<32;i++)
			{
				flash_write_int(FLASH_INPUT1_FILTER + i, 5, FLASH_MEMORY);
				filter[i] = 2;
			}
		}
		else
		{
			for(i=0;i<32;i++)
			{
				if(!flash_read_int(FLASH_INPUT1_FILTER + i,&pulse_buf,FLASH_MEMORY))
				{
				 	pulse_buf = 5;	
					flash_write_int(FLASH_INPUT1_FILTER + i, 5, FLASH_MEMORY);	
				}
				filter[i] = pulse_buf & 0xff;
			}
		}
	 	for(i=0;i<32;i++)
		{
			
			if(!flash_read_int(FLASH_INPUT1_RANGE + i,&pulse_buf,FLASH_MEMORY))			 
				range[i] = 0;
		
			else
			{
				if(pulse_buf > 20)
				pulse_buf = 0;
		 		range[i] = pulse_buf & 0xff;
			}
			gucFilterCounter[i] = 0;			
		
		}
	 	// add these two register for robert to set the range .
		if(!flash_read_int(FLASH_RANGE_LO ,&pulse_buf,FLASH_MEMORY))
		{
			pulse_buf = 0;
			flash_write_int(FLASH_RANGE_LO ,pulse_buf,FLASH_MEMORY);	
		}
		range_lo = pulse_buf; 
		if(!flash_read_int(FLASH_RANGE_HI ,&pulse_buf,FLASH_MEMORY))
		{
			pulse_buf = 500;
			flash_write_int(FLASH_RANGE_HI ,pulse_buf,FLASH_MEMORY);	
		}
		range_hi = pulse_buf; 

		#elif defined(T3_8IO_A)//16OUT)
			for(i=0;i<8;i++)
			{
				if(!flash_read_int(FLASH_INPUT1_FILTER + i,&pulse_buf,FLASH_MEMORY))
				{
				 	pulse_buf = 5;	
					flash_write_int(FLASH_INPUT1_FILTER + i, 5, FLASH_MEMORY);	
				}
				filter[i] = pulse_buf & 0xff;
			 
			}
						
			for(i=0;i<8;i++)
			{
				if(!flash_read_int(FLASH_DEADMASTER_AO0 + i,&pulse_buf,FLASH_MEMORY))
				{
				 	pulse_buf = 0;	
					flash_write_int(FLASH_DEADMASTER_AO0 + i, pulse_buf, FLASH_MEMORY);	
				}
				deadmaster_output[i] = pulse_buf ;			 
			}
			for(i=0;i<8;i++)
			{
				
				if(!flash_read_int(FLASH_INPUT1_RANGE + i,&pulse_buf,FLASH_MEMORY))			 
					range[i] = 0;
			
				else
				{
					if(pulse_buf >20)
					pulse_buf = 0;
			 		range[i] = pulse_buf & 0xff;
				}	
			
			}
		#endif



	#if defined (T3_8IO) || (T3_8IO_REV8) || (T3_8IO_A) || (T3_8IO_REV6)
	
		// make sure upon initialization that relays remain off
		if ( reverse_logic_output)
		{	RELAY1 = 1;
			RELAY2 = 1;
			RELAY3 = 1;
			RELAY4 = 1;
			
			RELAY5 = 1;
			RELAY6 = 1;
			RELAY7 = 1;
			RELAY8 = 1;

		}
		else
		{	RELAY1 = 0;
			RELAY2 = 0;
			RELAY3 = 0;
			RELAY4 = 0;
			
			RELAY5 = 0;
			RELAY6 = 0;
			RELAY7 = 0;
			RELAY8 = 0;
		}
		
		KEYPAD1_ENABLE = 1;
		KEYPAD1_HAND = 1;
		KEYPAD1_AUTO = 1;
		LED_DRIVE1 = 0;
		LED_DRIVE2 = 0;
		LED_DRIVE3 = 0;
		MUX_OFF1 = 0;	 

		ENABLE_2003 = 0;
		ENABLE12V = 0;


	//07/01/KR	
	#elif defined (T3_8IN13OUT)
		if(reverse_logic_output)
		{	RELAY1 = 1;
			RELAY2 = 1;
			RELAY3 = 1;
			RELAY4 = 1;
			
			RELAY5 = 1;
			RELAY6 = 1;
			RELAY7 = 1;
			RELAY8 = 1;
			RELAY9 = 1;
			RELAY10 = 1;
			RELAY11 = 1;
			RELAY12 = 1;
			RELAY13 = 1;

		}
		else
		{	RELAY1 = 0;
			RELAY2 = 0;
			RELAY3 = 0;
			RELAY4 = 0;
			
			RELAY5 = 0;
			RELAY6 = 0;
			RELAY7 = 0;
			RELAY8 = 0;
			RELAY9 = 0;
			RELAY10 = 0;
			RELAY11 = 0;
			RELAY12 = 0;
			RELAY13 = 0;
		}
		
		KEYPAD1_ENABLE = 1;
		KEYPAD2_ENABLE = 1;
		KEYPAD1_HAND = 1;
		KEYPAD1_AUTO = 1;
		LED_DRIVE1 = 0;
		LED_DRIVE2 = 0;
		LED_DRIVE3 = 0;
		LED_DRIVE4 = 0;
	//	MUX_OFF1 = 0;	 

		ENABLE_2003 = 0;
		ENABLE12V = 0;
		
	#elif defined (T3_8IN16OUT)
	
		// --- ensure that the latches are all set initially to avoid flickering upon start-up ---
		// turn-off the 12V supply Chips
		ENABLE12V = 1;   // active low

		ENABLE_2003 = 1;	//	 active high, thus 1 is ON

		// send off signal to driver chip buffers
		P0 = 0;
		// rising edge will activate first batch of switches
		delay_us(1);
		WRITEPORT1 = 1;
		WRITEPORT1 = 0;
		delay_us(1);
		WRITEPORT2 = 1;
		WRITEPORT2 = 0;

		// ----------------------------------------------------------------------------------------

		KEYPAD1_ENABLE = 1;
		KEYPAD2_ENABLE = 1;
		KEYPAD1_HAND = 1;
		KEYPAD1_AUTO = 1;
		KEYPAD2_HAND = 1;
		KEYPAD2_AUTO = 1;
		MUX_OFF1 = 0;

		LED_DRIVE1 = 1;
		LED_DRIVE2 = 1;
		LED_DRIVE3 = 1;
		LED_DRIVE4 = 1;

		// turn-on the 12V supply Chip
		ENABLE12V = 0;

		// turn-on the driver chip
		ENABLE_2003 = 0;	//	 active high, thus 1 is ON

		HIGH_SPEED_IN = 1;

		#ifdef REX_FEATURES
			for(i=0; i<HIGH_BUFFER; i++)
				high_speed_buffer[i] = 0;
		#endif

	#elif defined (PWM_TRANSDUCER)
		
		PWME = PWME | 0x40;
		PWMC = 0x03;

	#endif

	// set the baudrate
	if(!flash_read_char(FLASH_BAUDRATE,&temp_read, FLASH_MEMORY))
	{
		temp_read = 1;	// by default baudrate set to 19200
		flash_write_int(FLASH_BAUDRATE,temp_read, FLASH_MEMORY);
	}
 if(temp_read == 0)
	{
 		TH1	  = 0xfd;
		TL1	  = 0xfd;
		PCON  = 0x00 ;
		SERIAL_RECEIVE_TIMEOUT = 6;
	}
	else if(temp_read == 2)
	{
		TH1	  = 0xff;
		TL1	  = 0xff;
		PCON  = 0x80 ;
		SERIAL_RECEIVE_TIMEOUT = 3;
	}
	else 
	{
		TH1	  = 0xfd;
		TL1	  = 0xfd;
		PCON  = 0x80 ;
		SERIAL_RECEIVE_TIMEOUT = 3;
	}
	

	for(i=0; i<20; i++)
	{
		if((!flash_read_int(FLASH_SERIALNUMBER_LOWORD + i,&pulse_buf,FLASH_MEMORY)))
		{
			if(i == 6)	pulse_buf = 254;
			else		pulse_buf = 0 ;
			flash_write_int(FLASH_SERIALNUMBER_LOWORD + i,pulse_buf, FLASH_MEMORY);		
		}
		info[i] = pulse_buf ;		
	}

	P0=0xFF;
	
	SCON  = 0x50;
	TMOD  = 0x21;
//	TH1	  = 0xfd;
//	TL1	  = 0xfd;
	TL0   = 0x04;
	TH0   = 0xF7;
	TR1	  = 1;
	TR0   = 1;
	ET0   = 1;
	//PT0     = 1;
	PS = 1;

	#ifdef REX_FEATURES
		IT0   = 1;	// sets ext. int. 0 to be edge triggered
		EX0   =	1;	// enable ext. int. 0
//		PX0   = 1;	// give ext. int. 0 priority
	#endif

	 


	SYSCON |= 0X02;

	initSerial();

	// --- enable interrupts ---------------------------------------------
	EA    = 1;	 // global interrupt enable
	ES 	  = 1;	 // setup serial port interrupt
	// -------------------------------------------------------------------

//	LED_bank = 1;
	startup_flag = 1;  

	// delay to make sure all is stable
	watchdog();
	delay_us(50000);
	watchdog();
	delay_us(50000);
	watchdog();
	delay_us(50000);
	watchdog();
	delay_us(50000);
	watchdog();
	delay_us(50000);
	watchdog();

	startup_flag = 0;

	// --- initialize modbus.registers ---
	for (i = 0; i < TOTAL_EE_PARAMETERS; i++)
		modbus.registers[i] = 0;
	// --- initialize switch registers ---
	for (i = 0; i < MAX_OUTPUT_CHANNELS; i++)
		switch_state[i] = 0;

	// --- initialize event timers ---
	for(i=0;i<TOTAL_TIMERS;i++)
		big_ticks[i] = 0;

	#ifdef PWM_TRANSDUCER
		for(i=0; i<6; i++)
			digital_in_counter[i] = 0;

		if(!flash_read_char(FLASH_PWM_TIME_CALIBRATION,&time_calibration_offset, FLASH_MEMORY))
			time_calibration_offset = CALIBRATION_OFFSET;
	#endif

 	if(!flash_read_char(EEP_SERINALNUMBER_WRITE_FLAG,&SNWriteflag, FLASH_MEMORY))
	{
		SNWriteflag = 0;
		flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, 0, FLASH_MEMORY);	
	}

	// set output calibration
	if(!flash_read_char(FLASH_CALIBRATION,&output_calibration, FLASH_MEMORY))
		output_calibration = CALIBRATION_OFFSET;
 
	// write to FLASH_SEQUENCE 1 if nothing
	if(!flash_read_char(FLASH_SEQUENCE,&output_sequence, FLASH_MEMORY))
		flash_write_int(FLASH_SEQUENCE, 1, FLASH_MEMORY);

	if(output_sequence == 0)	// if was previously under test condition, want to set back to normal
	{
		flash_write_int(FLASH_SEQUENCE, 1, FLASH_MEMORY);
		output_sequence = 1;
	}
 

	flash_write_int(FLASH_SOFTWARE_VERSION_NUMBER, T3_SOFTWARE_VERSION, FLASH_MEMORY);
	flash_write_int(FLASH_PRODUCT_MODEL, T3_PRODUCT_MODEL, FLASH_MEMORY);
	flash_read_char(FLASH_HARDWARE_REV,&hardware_rev, FLASH_MEMORY);


	// --- need to start the timers to switche between cases in the main loop
	start_timer(HEARTBEAT_PULSE , DEFAULT_TIMEOUT );
	start_timer(REFRESH_INPUTS , DEFAULT_TIMEOUT );


	#ifndef T3_32IN
		start_timer(REFRESH_OUTPUTS , DEFAULT_TIMEOUT );
		start_timer(CHECK_SWITCHES , DEFAULT_TIMEOUT );
		gucStartFilter = 15;
	#endif


	#if defined (FLEXDRIVER_FEATURE)
		start_timer(FLEXDRIVER , DEFAULT_TIMEOUT );
	#elif defined (PWM_TRANSDUCER)
;//		start_timer(INPUT_PWM_TIMER , DEFAULT_TIMEOUT );
	#elif defined (T3_8IN16OUT)
;//		start_timer(HIGH_SPEED , DEFAULT_TIMEOUT );														
	#endif


	/*if ( output_sequence == 2 )
	{
		start_timer(TIMER_SOP , DEFAULT_TIMEOUT );

		// initialize error_check			//September 2004, Ron
	//	for(i=0; i<16; i++)
		//	error_check[i] = 0x0000;
	
		if ( (SNWriteflag & 0x80) == 0x80)
			switching_flag_previous = 1;
		else
			switching_flag_previous = 0;
	}
*/

	// produce LUT useful for calibration purposes
	#ifdef CALIBRATION_OPTION
		produce_LUT();	
	#endif

	#if defined T3_8IN13OUT
	start_timer(STORE_PULSE,DEFAULT_TIMEOUT);
	#endif
	//start_timer(HEARTBEAT_PULSE , DEFAULT_TIMEOUT );//test
}

// *****************************************************************************************************
// TIMER interrupts and event control subroutines
// *****************************************************************************************************

////////////////////////
void timer0(void) interrupt 1
{
    static unsigned int xdata com_count = 0 ;
	  TL0   = 0x00 ;			// T=15540 Formula: (65536-T)*12/11059200s = 2.5ms
    TH0   = 0xF7 ;
    heart_beat++;
	new_heartbeat = TRUE;
 
	if(guiDeadmasterTime)
		guiDeadmasterTime--;

	//GW
	P0 = 0xFF;	// turn off leds in advance to avoid ghosting 
	
	// increment the LEB_bank for Refresh routine
	LED_bank++;
	if(LED_bank > MAX_LED_BANK)LED_bank = 1;
			
	refresh_LEDs();  //changed by chelsea
	com_count++ ;
	if(com_count>= 12000)
	{
		com_count = 0 ;
		com_beat++ ;
	}
}

// This function adds an event to the event queue.
// An event must be pushed into the event queue in order for the main routine
// to activate the corresponding sub-routine
void push_event( et_event event_type )
{

	if( event_count	< EVENT_QUEUE_SIZE )  //Maximum	of three events	QUEUE'd
	{
		event_queue[ event_count++ ] = event_type;
	}
}


// This function initializes the various event timers.
void start_timer( et_event timer_no , unsigned int timeout )
{
	
	if( timeout == DEFAULT_TIMEOUT )	// DEFAULT_TIMEOUT = 0
	{
		big_ticks[ timer_no] = init_big_ticks[timer_no ];	//load the timer with the default number of ticks
	}
	else
	{
		big_ticks[ timer_no ]= timeout;    
	}

}


// This function pauses the various event timers by setting the number of ticks to 0.
void stop_timer( et_event timer_no )
{ 
  big_ticks[ timer_no ] = DEFAULT_TIMEOUT;
}


// This routine keeps track of the serial receive.  If no commands are being received, 
// everything is automatically shutoff.
/*
void serial_receive_autoshutoff (void)
{
	if (serial_receive_watchdog)
	{
		serial_receive_watchdog = 0;
		OnLine = 1 ;
	}
	else
		autoshutoff();


}


//This routine shuts everything off

void autoshutoff (void)
{
	P1=0xff;
	OnLine = 0 ;
}
*/


// *****************************************************************************************************
// HARDWARE rev setup only done once
// set_hardware()
// *****************************************************************************************************


void set_hardware(void)
{

// --- set the defines for the hardware revisions -------------------------------
	// November Ron

	// analog board
	if ( hardware_rev == 0 )
	{	// set pins
		PWME = PWME | 0x40;
		PWMC = 0x03;

// believe this is such for PWM board???		
//		MUX_OFF1 = 0;
	}

	// hardware rev2 boards with digital relay
	if ( hardware_rev >=1 )
		digital_mode = 1;

	// hardware rev3, boards with reversed logic transistors
	if ( hardware_rev >=3 )
		reverse_logic_output = 1;

// --------------------------------------------------------------------------------


}




// *****************************************************************************************************
// refresh_inputs
// 
// *****************************************************************************************************
void refresh_inputs (void)
{	
		unsigned char xdata i;

	#if defined (T3_8IO) || (T3_8IO_REV8) || (T3_8IO_A) || (T3_8IO_REV6)


		// bypass this step if in digital mode given the pin has been changed
		if(!digital_mode)
			MUX_OFF2 = 1;

	#elif defined (PWM_TRANSDUCER)
		//disable the Output MUX
		MUX_OFF2 = 1;
		
		// Enable the input MUX
		MUX_OFF1 = 0;
	#endif


	// then read the analog input
	if(pic_type==1)
		read_pic_original(EEP_INPUT1, HI);
	else if (pic_type==2)
		read_pic();
#ifdef  T3_8IN13OUT
	else if (pic_type>=3)	// Robert pic routine
	{
		for(i=0;i<2;i++)
		read_pulse_pic();
	}
#endif

	#if defined (T3_8IO) || (T3_8IO_REV8) || (T3_8IO_A) || (T3_8IO_REV6)
		// error check every input with every output by checking the difference
		// once error is found, increment counter and light on LED according to counter
		// August Ron
		if(Enable_read_input)
		{
			// Digital Mode
			// check that as long as one is high, the other is low.
			// error found if both input and output are the same
			if (digital_mode)
			{	


				for(i = 0; i < 8; i++)
					if( ( (modbus.registers[i]<=100) && (modbus.registers[i+8]<=100) ) || ( (modbus.registers[i]>=900) && (modbus.registers[i+8]>=900) ) )
					{	// if one is low and other is high, is not true, we have an error
						testing_error_flag++;
//						channel_error_flag = ( (channel_error_flag << 7-i) | 0x80 ) >> 7-i;
					}
//					else	// statement if want to reset the light back to off state
//						channel_error_flag = ( (channel_error_flag >> 7-i) & 0xFE ) << 7-i;
	
			}
			// Analog mode
			// check registers by pairs and variation should be between the acceptable margin
			// the if statement sets the bit to true and the else sets the bit to false
			// thus if we remove the else statement the bit will always stay on given will never
			// return to zero
			else
			{	for(i = 0; i < 8; i++)
					if( abs( modbus.registers[i]-modbus.registers[i+8] ) > TESTING_ERROR_MARGIN)
					{	// in this case if above absolute value is big we note that there is error
						testing_error_flag++;
						// use a type char and every bit represents a true or false for every input/output
						channel_error_flag = ( (channel_error_flag >> 7-i) | 0x01 ) << 7-i;
					}
		//			else	// statement if want to reset the light back to off state
		//				channel_error_flag = ( (channel_error_flag >> 7-i) & 0xFE ) << 7-i;
			}
	
		
			// now that reading is done, turn read off and turn output on
			// I have added the if statement so that we can control stopping the sequence
			// once a certain number of error is found
			// I placed value to be -1 if I do not want the testing to end
			if (testing_error_flag == -1)
			{	Enable_set_output = 0;
				Enable_read_input = 0;
			}
			else
			{	Enable_set_output = 1;
				Enable_read_input = 0;
			}
		}

	#endif



#ifdef BENNY_FEATURES
	// *** special added one time feature for Benny *************************************************************
	// look at input 3
	if ( modbus.registers[2+8] < 512 )
	{
		out3 = 1;
		input_holding_counter = HOLDING_LENGTH;

	}
	else
	{
		input_holding_counter--;

		if (input_holding_counter <= 0)
		{
			input_holding_counter = 0;
			out3 = 0;
		}
		
	}


/*	for( i = 0; i<4; i++ )
	{
		if ( modbus.registers[i+8] > 650 )
		{
			input_pulsing_counter[i]++;
		
			if (input_pulsing_counter[i] >= (PULSE_LENGTH+10) )
				input_pulsing_counter[i] = PULSE_LENGTH+10;
			else if( (input_pulsing_counter[i] > PULSE_LENGTH) && (input_holding_counter[i] == 0) )
			{
				switch(i)
				{
					case 0:
						out1 = 1;
						break;
					case 1:
						out2 = 1;
						break;
					case 2:
						out3 = 1;
						break;
					case 3:
						out4 = 1;
						break;

				}

				input_pulsing_counter[i] = PULSE_LENGTH + 10;
				input_holding_counter[i] = 5;
			}
		}
		else if ( modbus.registers[i+8] < 200 )
		{
			input_pulsing_counter[i] = 0;

			if (input_holding_counter[i] < HOLDING_LENGTH)
				input_holding_counter[i] = 0;
			else
				input_holding_counter[i]--;
			
		}

	}
*/

	// **********************************************************************************************************
#endif



}



// *****************************************************************************************************
// CHECK input switches and change output accordingly
// check_switches()
// refresh_outputs()
// *****************************************************************************************************


#ifndef T3_32IN
#ifndef PWM_TRANSDUCER

void check_switches(unsigned char switch_set)
{

	unsigned char hand_state_buffer, auto_state_buffer, i, j, P0_buffer;
	unsigned char off_state_buffer = 0;
	bit led_drive1_buf, led_drive2_buf, led_drive3_buf;
	bit switch_state_change = 1;
#if defined (T3_8IN16OUT)||(T3_8IN13OUT)
	bit led_drive4_buf;
#else
	bit KEYPAD2_ENABLE, KEYPAD2_AUTO, KEYPAD2_HAND;
#endif




	
	
	switch(switch_set)
	{
		case LO:
		
				//--------------Next check the state of the switches--------------------
				// Save state of status LEDs
				P0_buffer = P0;
				led_drive1_buf = LED_DRIVE1;
				led_drive2_buf = LED_DRIVE2;
				led_drive3_buf = LED_DRIVE3;
			#if defined (T3_8IN16OUT)||(T3_8IN13OUT)
				led_drive4_buf = LED_DRIVE4;
			#endif
			
			
				// Turn off status LEDs
				LED_DRIVE1 = 0;
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
			#if defined (T3_8IN16OUT)||(T3_8IN13OUT)
				LED_DRIVE4 = 0;
			#endif
			
				P0 = 0xFF;	
			
				// Now check the switches
				KEYPAD1_ENABLE = 0;
				delay_us(10);	// we have increased to filter out noise
			
				// --- cycle through hand state -------------------------
				KEYPAD1_HAND = 0;
				hand_state_buffer = P0;
				KEYPAD1_HAND = 1;
				// ------------------------------------------------------
			
				P0 = 0xFF;
			
				// --- cycle through auto state -------------------------
				KEYPAD1_AUTO = 0;
				auto_state_buffer = P0;
				KEYPAD1_AUTO = 1;
				// ------------------------------------------------------
			
				KEYPAD1_ENABLE = 1;

			
				// Restore state of status LEDs
				P0 = P0_buffer;
				LED_DRIVE1 = led_drive1_buf;
				LED_DRIVE2 = led_drive2_buf;
				LED_DRIVE3 = led_drive3_buf;
			#if defined (T3_8IN16OUT)||(T3_8IN13OUT)
				LED_DRIVE4 = led_drive4_buf;
			#endif

		break;

		case HI:

				P0_buffer = P0;
				led_drive1_buf = LED_DRIVE1;
				led_drive2_buf = LED_DRIVE2;
				led_drive3_buf = LED_DRIVE3;
			#if defined (T3_8IN16OUT)||(T3_8IN13OUT)
				led_drive4_buf = LED_DRIVE4;
			#endif
			
			
				// Turn off status LEDs
				LED_DRIVE1 = 0;
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
			#if defined (T3_8IN16OUT)||(T3_8IN13OUT)
				LED_DRIVE4 = 0;
			#endif
			
				P0 = 0xFF;	
			
				// Now check the switches
				KEYPAD2_ENABLE = 0;
				delay_us(10);	// we have increased to filter out noise
			
				// --- cycle through hand state -------------------------
				KEYPAD1_HAND = 0;
				hand_state_buffer = P0;
				KEYPAD1_HAND = 1;
				// ------------------------------------------------------
			
				P0 = 0xFF;
			
				// --- cycle through auto state -------------------------
				KEYPAD1_AUTO = 0;
				auto_state_buffer = P0;
				KEYPAD1_AUTO = 1;
				// ------------------------------------------------------
			
				KEYPAD2_ENABLE = 1;
			
				// Restore state of status LEDs
				P0 = P0_buffer;
				LED_DRIVE1 = led_drive1_buf;
				LED_DRIVE2 = led_drive2_buf;
				LED_DRIVE3 = led_drive3_buf;
			#if defined (T3_8IN16OUT)||(T3_8IN13OUT)
				LED_DRIVE4 = led_drive4_buf;
			#endif
		break;

	}


	// RL NOV 04
	// add a state for the OFF buffer	
	j = 1;
	for (i = 0; i < 8; i++)
	{
		if( !( (hand_state_buffer & j) == 0) || !( (auto_state_buffer & j) == 0) )
			off_state_buffer |= j;

		j = j << 1;
	}



	//filter the switch inputs so that noise is disregarded
	if (off_state_buffer != off_state[switch_set])
	{
		off_state_filter[switch_set]++;
		if (off_state_filter[switch_set] >= 10)
		{
			off_state[switch_set] = off_state_buffer;	
			off_state_filter[switch_set] = 0;
			switch_state_change = 1;
		}

	}
	else
		off_state_filter[switch_set] = 0;



	if (hand_state_buffer != hand_state[switch_set])
	{
		hand_state_filter[switch_set]++;
		if (hand_state_filter[switch_set] >= 10)
		{
			hand_state[switch_set] = hand_state_buffer;	
			hand_state_filter[switch_set] = 0;
			switch_state_change = 1;
		}
	}
	else
		hand_state_filter[switch_set] = 0;


	if (auto_state_buffer != auto_state[switch_set])
	{
		auto_state_filter[switch_set]++;
		if (auto_state_filter[switch_set] >= 10)
		{
			auto_state[switch_set] = auto_state_buffer;	
			auto_state_filter[switch_set] = 0;
			switch_state_change = 1;
		}
	}
	else
		auto_state_filter[switch_set] = 0;







	// Determine state of each individual switch	
	j = 1;
	if (switch_state_change)
	{
		switch_state_change = 0;

		if ( switch_set == LO)
		{

			for (i = 0; i < 8; i++)
			{	// zero is true, 1 is false
				if (!(off_state[switch_set] & j))
					switch_state[i] = 0;
				else if (!(hand_state[switch_set] & j))
					switch_state[i] = 1;
				else if (!(auto_state[switch_set] & j))
					switch_state[i] = 2;
				else
					switch_state[i] = 0;
				
				j *= 2;
			}

			// Combine state of switches into one 2-byte number
			modbus.registers[EEP_SWITCH_STATE1] = 0;
			for (i = 0; i < 8; i++)
			{
				modbus.registers[EEP_SWITCH_STATE1] = modbus.registers[EEP_SWITCH_STATE1] << 2;
				modbus.registers[EEP_SWITCH_STATE1] = modbus.registers[EEP_SWITCH_STATE1] | switch_state[7-i];
			}
						//debug 

		}
		else
		{
			
			for (i = 8; i < 16; i++)
			{	// zero is true, 1 is false
				if (!(off_state[switch_set] & j))
					switch_state[i] = 0;
				else if (!(hand_state[switch_set] & j))
					switch_state[i] = 1;
				else if (!(auto_state[switch_set] & j))
					switch_state[i] = 2;
				else
					switch_state[i] = 0;
				
				j *= 2;
			}

			// Combine state of switches into one 2-byte number
			modbus.registers[EEP_SWITCH_STATE2] = 0;
			for (i = 8; i < 16; i++)
			{
				modbus.registers[EEP_SWITCH_STATE2] = modbus.registers[EEP_SWITCH_STATE2] << 2;
				modbus.registers[EEP_SWITCH_STATE2] = modbus.registers[EEP_SWITCH_STATE2] | switch_state[i];
			}
			//DEBUG LINE
			//if(switch_state[8] == 1)
			//{
			//	ENABLE12V = 1;  	
			//}
		}


	}

	// run a test sequence to verify is switches are running correctly
	// basically just check the switch register and set a flag according to desired combination
	// august Ron

	if (output_sequence == 0)
	{
		if ( switch_set == LO)
		{
			if ( (modbus.registers[EEP_SWITCH_STATE1]==0) || (modbus.registers[EEP_SWITCH_STATE1]==0xAAAA) || (modbus.registers[EEP_SWITCH_STATE1]==0x5555) )
				switch_error_flag = 1;
			else
				switch_error_flag = 0;
		}
		else
		{
			if ( (modbus.registers[EEP_SWITCH_STATE2]==0) || (modbus.registers[EEP_SWITCH_STATE2]==0xAAAA) || (modbus.registers[EEP_SWITCH_STATE2]==0x5555) )
				switch_error_flag = 1;
			else
				switch_error_flag = 0;
		}
	}
	if (output_sequence == 3)
	{
		for(i=0; i<MAX_OUTPUT_CHANNELS; i++)
			switch_state[i] = 2;
	}

}
#endif // not PWM_TRANSDUCER

#endif // not T3_32IN

#if defined T3_8IN13OUT
void StorePulseToFlash( void )
{
	unsigned char i;
	static unsigned char xdata flash_write_flag = 0 ;
	if(flash_write_count == 0) return ;
	flash_write_flag ++ ;
	if(flash_write_flag == flash_write_count)
	{
		flash_write_flag = 0 ;
		for(i=0;i<8;i++)
		{
			if(GetBit(i,&channel_type))
			{
				if(GetBit(i,&clear_pulse_channel))
				{
					ClearBit(i,& clear_pulse_channel); 
					flash_write_int(FLASH_START_PULSE + (i << 1),(pic.pulse_number[i << 2] << 8) + pic.pulse_number[(i << 2) + 1],FLASH_MEMORY);
					flash_write_int(FLASH_START_PULSE + 1 +(i << 1),(pic.pulse_number[(i << 2) + 2] << 8) + pic.pulse_number[(i << 2) + 3],FLASH_MEMORY);	
					flash_write_int(i*5 + FLASH_PULSE1_YEAR,pulse_number[i*5],FLASH_MEMORY);
					flash_write_int(i*5 + 1 + FLASH_PULSE1_YEAR,pulse_number[1 + i*5],FLASH_MEMORY);
					flash_write_int(i*5 + 2 + FLASH_PULSE1_YEAR,pulse_number[2 + i*5],FLASH_MEMORY);
					flash_write_int(i*5 + 3 + FLASH_PULSE1_YEAR,pulse_number[3 + i*5],FLASH_MEMORY);
					flash_write_int(i*5 + 4 + FLASH_PULSE1_YEAR,pulse_number[4 + i*5],FLASH_MEMORY);
	 
	 
				}
				else
				{
	 
					flash_write_int(FLASH_START_PULSE + (i << 1),(unsigned int)(pic.pulse_number[i << 2] << 8) + pic.pulse_number[(i << 2) + 1],FLASH_MEMORY);
					flash_write_int(FLASH_START_PULSE + 1 +(i << 1),(unsigned int)(pic.pulse_number[(i << 2) + 2] << 8) + pic.pulse_number[(i << 2) + 3],FLASH_MEMORY);	
				
				}
			}
	 
		}
	}
}
#endif

// *************************************************************************
// tabulate_LED_STATE
//	perform logic for LED display
//
// *************************************************************************

void tabulate_LED_STATE(void)
{
	unsigned char i;

	ET0 =0;
	if( output_sequence == 0)
	{
		if (pulse_flag)
			LED_State[1] = LED_State[1] & 0xFE;
		else 
			LED_State[1] = LED_State[1] | 0x01;

		if (switch_error_flag)
			LED_State[1] = LED_State[1] & 0xFD;
		else 
			LED_State[1] = LED_State[1] | 0x02;

	}
	else
	{
		if (pulse_flag)
			LED_State[1] = LED_State[1] & 0xFE;
		else 
			LED_State[1] = LED_State[1] | 0x01;

		if (com_LED_count>0)
		{
			com_LED_count--;
			LED_State[1] = LED_State[1] & 0xFD;
		}
		else 
			LED_State[1] = LED_State[1] | 0x02;




// debug below for 32in module
/*		if (send_table_counter>0)
		{
			LED_State[1] = LED_State[1] & 0xFD;
		}
		else 
			LED_State[1] = LED_State[1] | 0x02;
*/
	}


	
/*
	// August Ron
	if (digital_mode)
		LED_State[1] = LED_State[1] & 0xBF;
	else 
		LED_State[1] = LED_State[1] | 0x40;

	// August 10 Ron
	if (Enable_read_input)
		LED_State[1] = LED_State[1] & 0xDF;
	else 
		LED_State[1] = LED_State[1] | 0x20;

	// August 10 Ron
	if (testing_error_flag > 0 || switch_error_flag)
		LED_State[1] = LED_State[1] & 0xEF;
	else 
		LED_State[1] = LED_State[1] | 0x10;

*/



	
	
	#if defined (T3_32IN)

		#ifdef FLEXDRIVER_FEATURE
			for (i = 0; i < 8; i++)
				if( (modbus.registers[i+EEP_INPUT25] > STATE_CHANGE_UPPER) || (modbus.registers[i+EEP_INPUT25] < STATE_CHANGE_LOWER) )
					LED_State[0] = LED_State[0] >> 1;			// right most bank of LEDs
				else
					LED_State[0] = (LED_State[0] >> 1) | 0x80;
	
	
			for (i = 0; i < 8; i++)
				if( (modbus.registers[i+EEP_INPUT1] > STATE_CHANGE_UPPER) || (modbus.registers[i+EEP_INPUT1] < STATE_CHANGE_LOWER) )
					LED_State[2] = LED_State[2] >> 1;			// left most bank of LEDs
				else
					LED_State[2] = (LED_State[2] >> 1) | 0x80;
	
			for (i = 0; i < 8; i++)
				if( (modbus.registers[i+EEP_INPUT9] > STATE_CHANGE_UPPER) || (modbus.registers[i+EEP_INPUT9] < STATE_CHANGE_LOWER) )
					LED_State[3] = LED_State[3] >> 1;			// second to the left bank of LEDs
				else
					LED_State[3] = (LED_State[3] >> 1) | 0x80;
	
	
			for (i = 0; i < 8; i++)
				if( (modbus.registers[i+EEP_INPUT17] > STATE_CHANGE_UPPER) || (modbus.registers[i+EEP_INPUT17] < STATE_CHANGE_LOWER) )
					LED_State[4] = LED_State[4] >> 1;			// second to the right bank of LEDs
				else
					LED_State[4] = (LED_State[4] >> 1) | 0x80;
	
	
		#else

			for (i = 0; i < 8; i++)
	
				if(modbus.registers[i+EEP_INPUT25] > 500)		
					LED_State[0] = LED_State[0] >> 1;			// right most bank of LEDs
				else
					LED_State[0] = (LED_State[0] >> 1) | 0x80;
	
	
			for (i = 0; i < 8; i++)
				if (modbus.registers[EEP_INPUT1+i] > 500)
					LED_State[2] = LED_State[2] >> 1;			// left most bank of LEDs
				else
					LED_State[2] = (LED_State[2] >> 1) | 0x80;
	
			for (i = 0; i < 8; i++)
				if (modbus.registers[EEP_INPUT9+i] > 500)
					LED_State[3] = LED_State[3] >> 1;			// second to the left bank of LEDs
				else
					LED_State[3] = (LED_State[3] >> 1) | 0x80;
	
	
			for (i = 0; i < 8; i++)
				if (modbus.registers[i+EEP_INPUT17] > 500)
					LED_State[4] = LED_State[4] >> 1;			// second to the right bank of LEDs
				else
					LED_State[4] = (LED_State[4] >> 1) | 0x80;
		#endif
	
	#elif defined (T3_8IN16OUT)
	
		for (i = 0; i < 8; i++)
		{
			if (switch_state[i] == 2 && modbus.registers[EEP_OUTPUT9+i] > threshold)	// auto mode
				LED_State[0] = LED_State[0] >> 1;
			else if (switch_state[i] == 1)					// hand mode
				LED_State[0] = LED_State[0] >> 1;
			else												// off mode
				LED_State[0] = (LED_State[0] >> 1) | 0x80;
		}
	
		for (i = 0; i < 8; i++)
			if (modbus.registers[EEP_INPUT1+i] > 512)
				LED_State[2] = LED_State[2] >> 1;
			else
				LED_State[2] = (LED_State[2] >> 1) | 0x80;
	
		for (i = 0; i < 8; i++)
		{
			if (switch_state[8+i] == 2 && modbus.registers[i] > threshold)	// auto mode
				LED_State[3] = LED_State[3] >> 1;
			else if (switch_state[8+i] == 1)					// hand mode
				LED_State[3] = LED_State[3] >> 1;
			else												// off mode
				LED_State[3] = (LED_State[3] >> 1) | 0x80;
		}
	
	#elif defined (T3_8IO_A)
//		if(guiDeadmasterTime != 0)
//		{
			for (i = 0; i < 8; i++)
			{
				if (switch_state[i] == 1)					// hand mode
				LED_State[0] = LED_State[0] >> 1;
				else if (switch_state[i] == 0)					// off mode
				LED_State[0] = (LED_State[0] >> 1) | 0x80;
				else
				{
					if(guiDeadmasterTime != 0)
					{
						if ( modbus.registers[i] > threshold)	// auto mode
						LED_State[0] = LED_State[0] >> 1;
						else												// off mode
						LED_State[0] = (LED_State[0] >> 1) | 0x80;
					}
					else
					{
						if(deadmaster_output[i]> 500)
							LED_State[0] = LED_State[0] >> 1;						
						else												// off mode
							LED_State[0] = (LED_State[0] >> 1) | 0x80;
					}
				
				}
			}
//		}
//		else
//		{	for (i = 0; i < 8; i++)
//			{
//			if(deadmaster_output[i]> 500)
//				LED_State[0] = LED_State[0] >> 1;
//			
//			else												// off mode
//				LED_State[0] = (LED_State[0] >> 1) | 0x80;
//			}
//		}
		for (i = 0; i < 8; i++)
			if (modbus.registers[EEP_INPUT1+i] > 512)
				LED_State[2] = LED_State[2] >> 1;
			else
				LED_State[2] = (LED_State[2] >> 1) | 0x80;
	
	#elif defined (PWM_TRANSDUCER)
	
		// input LEDs
	// given not using PIC chip, just read from P0
	//	for (i = 0; i < 8; i++)
	//	{
	//		if (modbus.registers[EEP_INPUT1 + i] > 500)	// ON mode
	//			LED_State[0] = LED_State[0] >> 1;
	//		else										// OFF mode
	//			LED_State[0] = (LED_State[0] >> 1) | 0x80;
	//	}
	
		LED_State[0] = P0;
	
	
		// output LEDs	 
		for (i = 0; i < 8; i++)
		{
			if (modbus.registers[EEP_OUTPUT1 + i] > 500)	// ON mode
				LED_State[1] = LED_State[1] >> 1;
			else										// OFF mode
				LED_State[1] = (LED_State[1] >> 1) | 0x80;
		}
	
		// status LEDs
		if (pulse_flag)
			LED_State[1] = LED_State[1] & 0xBF;
		else 
			LED_State[1] = LED_State[1] | 0x40;
		
		#else
	
	// code below for LED controlled by relays
	
		if ( RELAY1 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF

		if ( RELAY2 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF

		if ( RELAY3 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF

		if ( RELAY4 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF

	
		if ( RELAY5 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF

		if ( RELAY6 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF
		
		if ( RELAY7 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF
		
		if ( RELAY8 )
			LED_State[0] = LED_State[0] >> 1;	// relay is ON
		else
			LED_State[0] = (LED_State[0] >> 1) | 0x80;	// relay is OFF
	
		//07/01/KR
		#if defined (T3_8IN13OUT)
			
		if ( RELAY9 )
		{
			LED_State[3] &= 0xFE;
		}
		else
		{
			LED_State[3] |= 0xE1;
		}	
		if ( RELAY10 )
		{
			LED_State[3] &= 0xFD;
		}
		else
		{
			LED_State[3] |= 0xE2;
		}	
		if ( RELAY11 )
		{
			LED_State[3] &= 0xFB;
		}
		else
		{
			LED_State[3] |= 0xE4;
		}	
		
		if ( RELAY12 )
		{
			LED_State[3] &= 0xF7;
		}
		else
		{
			LED_State[3] |= 0xE8;
		}	
		if ( RELAY13 )
		{
			LED_State[3] &= 0xEF;
		}
		else
		{
			LED_State[3] |= 0xF0;
		}
		#endif
	// decided that it made more sense to make LED controlled by relay states	
	// code below for LED controlled by switches
	/*
		for (i = 0; i < 8; i++)
		{
			if (switch_state[i] == 2 && modbus.registers[i] > 500)	// auto mode
				LED_State[0] = LED_State[0] >> 1;
			else if (switch_state[i] == 1)					// hand mode
				LED_State[0] = LED_State[0] >> 1;
			else												// off mode
				LED_State[0] = (LED_State[0] >> 1) | 0x80;
		}
	*/
	
		#ifdef T3_8IN13OUT
		 
		for (i = 0; i < 8; i++)
			if (adam.dual_word[i] > 512)
				LED_State[2] = LED_State[2] >> 1;
			else
				LED_State[2] = (LED_State[2] >> 1) | 0x80;
		
		#else
		for (i = 0; i < 8; i++)
			if (modbus.registers[i+8] > 512)
				LED_State[2] = LED_State[2] >> 1;
			else
				LED_State[2] = (LED_State[2] >> 1) | 0x80;
		#endif
	
	#endif
	
	
	#ifdef BENNY_FEATURES
		for (i = 0; i < 8; i++)
			if (modbus.registers[i+8] < 512)
				LED_State[2] = LED_State[2] >> 1;
			else
				LED_State[2] = (LED_State[2] >> 1) | 0x80;
	
	#endif
	ET0 = 1;

}

// ***********************************************************************************
// 		refresh LEDs
//	perform operation to cycle through display
//
// ***********************************************************************************
void refresh_LEDs (void)
{
	//GW: move this sentenc ahead of the subroutine's calling to save time and donot need delay anymore
	//P0 = 0xFF;	// turn off to avoid ghosting 
	//delay_us(1);

	switch (LED_bank)
	{
		// LED state for outputs drive1
		case 1:

			#if defined (T3_32IN)
				// Turn off other LED Banks
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
				LED_DRIVE4 = 0;
				LED_DRIVE5 = 0;

				// Set the port and turn on LED Bank

				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[0];

				LED_DRIVE1 = 1;
	
			#elif defined (T3_8IN16OUT)
				// Turn off other LED Banks
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
				LED_DRIVE4 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[0];

				LED_DRIVE1 = 1;

			#elif defined (PWM_TRANSDUCER)
				// Turn off other LED Banks
				LED_DRIVE2 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P2 = 0;
				else
					P2 = LED_State[0];

				LED_DRIVE1 = 1;
			#elif defined(T3_8IN13OUT)
				// Turn off other LED Banks
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
				LED_DRIVE4 = 0;
				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[0];

				LED_DRIVE1 = 1;
				
			#else
				// Turn off other LED Banks
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
				
				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[0];

				LED_DRIVE1 = 1;
	
			#endif



			break;

	 	// LED state for status
		case 2:

			#if defined (T3_32IN)
				// Turn off other LED Banks
				LED_DRIVE1 = 0;	
				LED_DRIVE3 = 0;
				LED_DRIVE4 = 0;
				LED_DRIVE5 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[1];
	
				LED_DRIVE2 = 1;
			
			#elif defined (T3_8IN16OUT)
				// Turn off other LED Banks
				LED_DRIVE1 = 0;	
				LED_DRIVE3 = 0;
				LED_DRIVE4 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[1];
	
				LED_DRIVE2 = 1;

			#elif defined (PWM_TRANSDUCER)
				LED_DRIVE1 = 0;	

				// Set the port and turn on LED Bank
				if (startup_flag)
					P2 = 0;
				else
					P2 = LED_State[1];
	
				LED_DRIVE2 = 1;
			#elif defined(T3_8IN13OUT)
				LED_DRIVE1 = 0;
				LED_DRIVE3 = 0;
				LED_DRIVE4 = 0;
				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[1];	

				LED_DRIVE2 = 1;			
			#else
				// Turn off other LED Banks
				LED_DRIVE1 = 0;
				LED_DRIVE3 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[1];

				LED_DRIVE2 = 1;
	
			#endif
	

			break;

	  	// LED state for inputs drive3
		case 3:

			#if defined (T3_32IN)
				// Turn off other LED Banks
				LED_DRIVE1 = 0;	
				LED_DRIVE2 = 0;
				LED_DRIVE4 = 0;
				LED_DRIVE5 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[2];

				LED_DRIVE3 = 1;
	
			#elif defined (T3_8IN16OUT)

				// Turn off other LED Banks
				LED_DRIVE1 = 0;	
				LED_DRIVE2 = 0;
				LED_DRIVE4 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[2];

				LED_DRIVE3 = 1;
			#elif defined(T3_8IN13OUT)
				LED_DRIVE1 = 0;
				LED_DRIVE2 = 0;
				LED_DRIVE4 = 0;
				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[2];

				LED_DRIVE3 = 1;
			#elif defined (PWM_TRANSDUCER)
				// do nothing
			#else
				// Turn off other LED Banks
				LED_DRIVE1 = 0;
				LED_DRIVE2 = 0;

				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[2];

				LED_DRIVE3 = 1;
	
	
			#endif


			break;


		case 4:

			#if defined (T3_32IN)
				// Turn off other LED Banks
				LED_DRIVE1 = 0;	
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
				LED_DRIVE5 = 0;
	
				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[3];
	
				LED_DRIVE4 = 1;
	
			#elif defined (T3_8IN16OUT)||(T3_8IN13OUT)//add by kanson
				// Turn off other LED Banks
				LED_DRIVE1 = 0;
				LED_DRIVE2 = 0;	
				LED_DRIVE3 = 0;
				
				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[3];
	
	
				LED_DRIVE4 = 1;
	
			#endif

			break;


		case 5:

			#if defined (T3_32IN)
				// Turn off other LED Banks
				LED_DRIVE1 = 0;	
				LED_DRIVE2 = 0;
				LED_DRIVE3 = 0;
				LED_DRIVE4 = 0;
	
				// Set the port and turn on LED Bank
				if (startup_flag)
					P0 = 0;
				else
					P0 = LED_State[4];
	
	
				LED_DRIVE5 = 1;
			#endif

			break;


	}

}










// *****************************************************************************************************
// WDT, Delay and calibration subroutines
// *****************************************************************************************************



void watchdog ( void )
{
	WDTC = 0xA7; 	//set watchdog
}


void delay_us (unsigned	int time)
{
   while( (time--) !=0 ){ ; }
}



/*

#ifndef T3_32IN

unsigned char const code def_tab[51] =
			{				
//			 1036, 1021, 1007, 993, 978, 961, 943, 926, 906, 885, 863, 837, 809, 778, 743, 703, 657, 605, 549, 494, 441, 394, 351, 315, 284, 258, 235, 215, 196, 180, 164, 150, 138, 126, 115, 105, 95, 86, 78, 70, 63, 56, 49, 43, 37, 31, 25, 20, 15, 10, 6
		//		6, 10, 15, 20, 25, 31, 37 ,43 ,49 ,56 ,63 ,70 ,78 ,86 ,95 ,105 ,115 ,126, 138, 150 ,164 ,180 ,196 ,215 ,235 ,258 ,284 ,315 ,351 ,394 ,441 ,494 ,549 ,605 ,657 ,703 ,743 ,778 ,809 ,837 ,863 ,885 ,906 ,926 ,943 ,961 ,978 ,993 ,1007 ,1021 ,1036
 //				6, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 14, 16, 16, 19, 20, 23, 26, 31, 36, 43, 47, 53, 55, 56, 52, 46, 42, 37, 31, 28, 26, 22, 21, 20, 19, 18, 17, 15, 14, 14, 15
//Rev10				4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 10, 10, 11, 12, 13, 14, 15, 17, 18, 21, 23, 26, 30, 37, 41, 47, 55, 57, 59, 53, 45, 43, 37, 31, 30, 26, 25, 21, 23, 17, 20, 18, 17, 13, 15
//Rev11			
				2,5,5,5,6,6,6,6,7,7,7,8,8,9,9,10,11,12,12,13,14,16,18,19,21,23,27,30,37,42,47,54,58,59,55,49,44,38,35,30,29,24,24,21,21,19,17,17,16,15,15

			};

unsigned int   look_up_table(unsigned int count )
{
	int   val;
    char  index=0;
	int   work_var= def_tab[0];

//count = 490;

	if (work_var > count )
		{
			val =  0 ;
			return ( val );
		}

	do 
		{
			index++;
			work_var += def_tab[index] ;

			if( work_var > count )
				{
				val = def_tab[index] - (work_var - count);
				val *= 10;
				val =  val / def_tab[index];
				val = (val * 5) / 10;
				val += (index * 5);
				return (val);
				}
		} while (index) ;

			val =  255 ;
			return ( val );
}


#endif

*/








