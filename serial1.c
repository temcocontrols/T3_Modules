#include <KEIL_VRS1000_regs.H>
#include "LibIO_T3IO.h"
#include "define.h"
#include "serial.h"
#include "stdlib.h"
unsigned char xdata com_beat = 1 ;
unsigned char update_flash = 0;
#ifdef  T3_8IO_A
unsigned int xdata deadmaster_output[8];
#endif
#ifdef T3_32IN
extern unsigned int xdata range_lo  ;
extern unsigned int xdata range_hi  ;
#endif 
extern int xdata calibration_offset[NUM_INPUTS];
extern unsigned char xdata info[20];
unsigned char xdata gucDeadmaster;
unsigned long xdata  guiDeadmasterTime;
bit serial_no_activity = 1;
bit switch_to_auto  = 0;
extern unsigned int xdata threshold ;
unsigned char xdata gucReverseOutput;
void refresh_outputs(void);

#if defined T3_8IN13OUT
extern unsigned char xdata flash_write_count ;
void SetBit(unsigned char bit_number,unsigned char *byte)
{
	unsigned char mask; 
	mask = 0x01;
	mask = mask << bit_number;
	*byte = *byte | mask;
}
void ClearBit(unsigned char bit_number,unsigned char *byte)
{
	unsigned char mask; 
	mask = 0x01;
	mask = mask << bit_number;
 	mask = ~mask;
	*byte = *byte & mask;
}
bit GetBit(unsigned char bit_number,unsigned char *byte)
{
	unsigned char mask; 
	mask = 0x01;	 
	mask = mask << bit_number;
	return (bit)(*byte & mask);
}
#endif
//---------------------initserial ------------------------------------
//	init serial port .
void initSerial(void)
{
//	unsigned char address_retrieve_flag;

	RS232STATE = RECEIVE;
	rece_count = 0;
	DealwithTag = 0;

	// check if during ISP mode if the address has been changed
	FlashRead_Absolute(DATA_TO_FIRMWARE + FLASH_ADDRESS,&laddress);
  
	// if it has not been changed, check the flash memory
	if( (laddress == 255) || (laddress == 0) )
	{
		if(!flash_read_char(FLASH_ADDRESS, &laddress, FLASH_MEMORY) )
		{
			laddress = 254;
			flash_write_int(FLASH_ADDRESS, laddress, FLASH_MEMORY);
		}
 		 
	}
	else
    {
		flash_write_int(FLASH_ADDRESS, laddress, FLASH_MEMORY);
	 
	}

	// if data is blank, means first time programming, thus put as default
	// Added by RL 02/11/04
	if(laddress == 0 || laddress == 255 ) 
		laddress = 254;

}

//-------------serial_restart -------------------------------
// restart the serial communications . 
void serial_restart(void)
{
	RS232STATE = RECEIVE;
	rece_count = 0;
	DealwithTag = 0;
}

//-------------initsend_com ---------------------------------
// it is ready to send data by serial port . 
void initSend_COM(void)
{
	transmit_finished = 0;
	RS232STATE = SEND;
}

//------------------------serialport ----------------------------------
//	serial port interrupt , deal with all action about serial port. include receive data and 
//		send data and deal with interal.
void SerialPort() interrupt 4
{
	//flag_comm = 1;
	if(RI == 1  )
	{
		// Make sure that you are not in SEND mode and that you do not exceed
		// the length of the data bufer
		if(rece_count < DATABUFLEN)
			data_buffer[rece_count++] = SBUF;
		RI = 0;

		// timeout count adjusted to half a packet length
		serial_receive_timeout_count = SERIAL_RECEIVE_TIMEOUT;
		if(rece_count == 1)
		{
			// This starts a timer that will reset communication.  If you do not
			// receive the full packet, it insures that the next receive will be fresh.
			// The timeout is roughly 7.5ms.  (3 ticks of the hearbeat)
			packet_size = 8;
			serial_receive_timeout_count = SERIAL_RECEIVE_TIMEOUT;	
		}

		// need to evaluate the packet size
		// once the sixth byte is received we can then figure out what is packet size
		 if(rece_count == 4)
		{
				//check if it is a scan command
			if((unsigned int)(data_buffer[2]<<8) + data_buffer[3] == 0x0a && data_buffer[1] == WRITE_VARIABLES)
			{
					
					packet_size = 12;
					serial_receive_timeout_count = SERIAL_RECEIVE_TIMEOUT;	
			}
		}
		else if(rece_count == 7 )
		{
			/*if(data_buffer[1] == READ_VARIABLES || data_buffer[1] == WRITE_VARIABLES)
				packet_size = 8;
			else if(data_buffer[1] == MULTIPLE_WRITE && response_receive_finished == 0)
				packet_size = 8;
			else*/ if(data_buffer[1] == MULTIPLE_WRITE)
				// take the quantity amount, and add 9 more for other information needed
			{
				packet_size = data_buffer[6] + 9;
				serial_receive_timeout_count = packet_size;
			}
		}		
		else if(rece_count == 3 )
		{
			if(data_buffer[1] == CHECKONLINE)
				packet_size = 6;
		}
		// As soon as you receive the final byte, switch to SEND mode
		else if(rece_count == packet_size)		
		{
			
			// full packet received - turn off serial timeout
			serial_receive_timeout_count = 0;
			DealwithTag =  com_dealy ;
            if(com_dealy == 0)
			DealwithTag = 2;		// making this number big to increase delay
			else 
			DealwithTag = 4;
			// if was dealing with a response, must know it is done
			response_receive_finished = 1;
		}


	}
	else if(TI == 1)
	{
		TI=0;
		transmit_finished = 1;
	}
	return;
}



// ------------------------dealwithdata -------------------------
// the routine dealwith data ,it has three steps.
// the 3 step is : 1 prepare to send data and init crc for next time
//				   2 dealwith interal
//                 3 organize the data of sending, and send data.

void dealwithData(void)
{	unsigned int address;
	
	// given this is used in multiple places, decided to put it as an argument
	address = (unsigned int)(data_buffer[2]<<8) + data_buffer[3];

	if (checkData(address))
	{
		com_LED_count = 5;

		// Initialize tranmission
		initSend_COM();
	
		// Initialize CRC
		InitCRC16();
				// Respond with any data requested
		responseData(address);
		
		

// Store any data being written
		internalDeal(address);

		guiDeadmasterTime = (unsigned long)gucDeadmaster *24000;
		serial_no_activity = 1;
		switch_to_auto = 0;
		 

	}	

	// Restart the serial receive.
	serial_restart();

}

//---------------------checkdata ----------------------
//This function calculates and verifies the checksum
bit checkData(unsigned int address)
{
	static unsigned char xdata rand_read_ten_count = 0 ;
	unsigned int crc_val;
	unsigned char minaddr,maxaddr, variable_delay;
	unsigned char i;

	// check if packet completely received
	if(rece_count != packet_size)
		return FALSE;

	// check if talking to correct device ID
	if(data_buffer[0] != 255 && data_buffer[0] != laddress && data_buffer[0] != 0)
		return FALSE;	

	//  --- code to verify what is on the network ---------------------------------------------------
	if( data_buffer[1] == CHECKONLINE)
	{
		crc_val = CRC16(data_buffer,4) ;
		if(crc_val != (data_buffer[4]<<8) + data_buffer[5] )
		{
			return FALSE;
		}
		minaddr = (data_buffer[2] >= data_buffer[3] ) ? data_buffer[3] : data_buffer[2] ;	
		maxaddr = (data_buffer[2] >= data_buffer[3] ) ? data_buffer[2] : data_buffer[3] ;	
		if(laddress < minaddr || laddress > maxaddr)
			return FALSE;
		else
		{	// in the TRUE case, we add a random delay such that the Interface can pick up the packets
			srand(heart_beat);
			variable_delay = rand() % 20;
			for ( i=0; i<variable_delay; i++)
				delay_us(100);
	
			return TRUE;
		}

	}
	// ------------------------------------------------------------------------------------------------------



	// check that message is one of the following
	if( (data_buffer[1]!=READ_VARIABLES) && (data_buffer[1]!=WRITE_VARIABLES) && (data_buffer[1]!=MULTIPLE_WRITE) )
		return FALSE;
	// ------------------------------------------------------------------------------------------------------
		
	if(data_buffer[2]*256 + data_buffer[3] ==  FLASH_ADDRESS_PLUG_N_PLAY)
	{
		 com_LED_count = 5; 
		if(data_buffer[1] == WRITE_VARIABLES)
		{
			if(data_buffer[6] != info[0]) 
			return FALSE;
			if(data_buffer[7] != info[1]) 
			return FALSE;
			if(data_buffer[8] != info[2])  
			return FALSE;
			if(data_buffer[9] != info[3]) 
			return FALSE;
		}
		if (data_buffer[1] == READ_VARIABLES)
		{
			randval = rand() % 10 / 2 ;
		}
		if(randval != RESPONSERANDVALUE)
		{
//mhf:12-29-05 if more than 5 times does not response read register 10,reponse manuly.
			rand_read_ten_count++;
			if(rand_read_ten_count%5 == 0)
			{
				rand_read_ten_count = 0;
				randval = RESPONSERANDVALUE;
				variable_delay = rand() % 10;
				for ( i=0; i<variable_delay; i++)
					delay_us(75);
			}
			else
				return FALSE;
		}
		else
		{		
			// in the TRUE case, we add a random delay such that the Interface can pick up the packets
			rand_read_ten_count = 0;
			variable_delay = rand() % 10;
			for ( i=0; i<variable_delay; i++)
				delay_us(75);				
		}
		
	}	




	// if trying to write the Serial number, first check to see if it has been already written
	// note this does not take count of multiple-write, thus if try to write into those reg with multiple-write, command will accept
	if( (data_buffer[1]==WRITE_VARIABLES)  && (address<=FLASH_HARDWARE_REV) )
	{
//		// Return false if trying to write SN Low word that has already been written
//		if(data_buffer[3] < 2)
//		{
//			if(SNWriteflag & 0x01)                // low byte of SN writed
//				return FALSE;
//		}
//		// Return false if trying to write SN High word that has already been written
//		else if (data_buffer[3] < 4)
//		{
//			if(SNWriteflag & 0x02)                 // high byte of SN writed
//				return FALSE;
//		}
//		else if (data_buffer[3] ==  FLASH_HARDWARE_REV)
//		{
//			if(SNWriteflag & 0x04)                 // hardware byte writed
//				return FALSE;
//		}

	}





	// --- plug and play feature in case there are two devices in the network with the same ID --------
/*	if(address ==  FLASH_ADDRESS_PLUG_N_PLAY)
	{
		if (data_buffer[1] == READ_VARIABLES)
		{
			randval = rand() % 5 ;
		}

		if(randval != RESPONSERANDVALUE)
		{
			return FALSE;
		}
		else
		{	// in the TRUE case, we add a random delay such that the Interface can pick up the packets
			variable_delay = rand() % 20;
			for ( i=0; i<variable_delay; i++)
				delay_us(100);
		}
	}*/
	// ------------------------------------------------------------------------------------------------------


	crc_val = CRC16(data_buffer,packet_size-2);

	if(crc_val == (data_buffer[packet_size-2]<<8) + data_buffer[packet_size-1] )
		return TRUE;
	else
		return FALSE;
   
 	return TRUE;
 }



//-------------------responsedata ---------------------------
// the routine organize  data back and send the data to buffer.
void responseData(unsigned int address)

{
	unsigned int flash_data;
	unsigned char num, send_buffer;
	unsigned char i=0;
	unsigned char block_to_erase;
	bit read_time_pulse = 0;
	// --- variables for calibration ---
	#ifdef CALIBRATION_OPTION
		unsigned char input_select=0;
		unsigned int location;
	#endif


	if(data_buffer[1] == WRITE_VARIABLES)
	{
		for (i=0;i<8;i++)
		{
			SBUF = data_buffer[i];
			transmit_finished = 0;
			while (!transmit_finished){}
			watchdog();
		}

	}
	else if(data_buffer[1] == READ_VARIABLES)
	{
	 
		i = 0;

		num = data_buffer[5];					// number of registers to be read

		send_buffer = data_buffer[0];			// re-send first byte of information
		SBUF = send_buffer;
		transmit_finished = 0;
		CRC16_Tstat(send_buffer);

		send_buffer = data_buffer[1];			// re-send second byte of information
		while (!transmit_finished){}
		SBUF = send_buffer;
		transmit_finished = 0;
		CRC16_Tstat(send_buffer);

		send_buffer = num<<1;					// re-send third byte of information
		while (!transmit_finished){}
		SBUF = send_buffer;
		transmit_finished = 0;
		CRC16_Tstat(send_buffer);
 		watchdog();

		for (;i<num;i++)
		{
						// read registers below 100, flash memory
			if ( i+address < 21 )
			{	
			
				if( !flash_read_int(address+i, &flash_data, FLASH_MEMORY) )	 flash_data = 0 ;
			//	flash_data = info[address+i];
				// --- send first byte -------------
				if (i+address >= MAX_FLASH_CONSTRANGE)
					send_buffer = 0;
				else
					send_buffer = (flash_data >> 8) & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------
				if (i+address >= MAX_FLASH_CONSTRANGE)
					send_buffer = 1;
				else
					send_buffer = flash_data & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if( i+address == 22 )
			{
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = com_beat;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address < 96)// ORIGINALADDRESSVALUE-1 )
			{	
			//	if( !flash_read_int(address+i, &flash_data, FLASH_MEMORY) )
					//flash_data = 0;

				flash_data = 0;
				// --- send first byte -------------
				if (i+address >= MAX_FLASH_CONSTRANGE)
					send_buffer = 0;
				else
					send_buffer = (flash_data >> 8) & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------
				if (i+address >= MAX_FLASH_CONSTRANGE)
					send_buffer = 1;
				else
					send_buffer = flash_data & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
		/*else if( i+address == 96)
			{	
				// --- send first byte -------------			
				send_buffer= T >> 8;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------
				send_buffer = T & 0XFF;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
		else if( i+address == 97)
			{	
				// --- send first byte -------------			
				send_buffer= TT >> 8;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------
				send_buffer = TT & 0XFF;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if( i+address == 98)
			{	
				// --- send first byte -------------			
				send_buffer= TTT >> 8;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------
				send_buffer = TTT & 0XFF;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}*/
			else if( i+address == ORIGINALADDRESSVALUE-1)
			{	
				// --- send first byte -------------			
				send_buffer=0;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------
				send_buffer = SNWriteflag;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
		/*	else if( i+address == 186 )	// dummy variable
			{
				// --- send first byte -------------			
				send_buffer= TT >> 8;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------

//				send_buffer = reading_counter;
//				send_buffer = send_table_counter;
				send_buffer = TT & 0xff;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

			}
			else if( i+address == 187 )	// dummy variable
			{
				// --- send first byte -------------			
				send_buffer = T >> 8;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				// --- send second byte ------------

//				send_buffer = reading_counter;
//				send_buffer = send_table_counter;
				send_buffer = T & 0xff;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

			}*/
			// read registers between 100 and TOTAL_EE_PARAMETERS, RAM memory
#ifdef T3_32IN

			else if ( i+address - ORIGINALADDRESSVALUE <= EEP_INPUT32 )
			{
				guiAnalogInput_temp[i + address - ORIGINALADDRESSVALUE-EEP_INPUT1] = guiAnalogInput[i + address - ORIGINALADDRESSVALUE-EEP_INPUT1]-DEFAULT_OFFSET +
				calibration_offset[i + address - ORIGINALADDRESSVALUE-EEP_INPUT1]; 
				
				send_buffer = (guiAnalogInput_temp[i + address - ORIGINALADDRESSVALUE-EEP_INPUT1] >> 8) & 0xFF;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = guiAnalogInput_temp[i + address - ORIGINALADDRESSVALUE-EEP_INPUT1] & 0xFF;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address - ORIGINALADDRESSVALUE < EEP_INPUT1_RANGE )
			{				

				send_buffer = ((modbus.registers[i+address - ORIGINALADDRESSVALUE])>> 8) & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = (modbus.registers[i+address - ORIGINALADDRESSVALUE]) & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address - ORIGINALADDRESSVALUE < EEP_INPUT1_FILTER)
			{				
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = range[i+address - ORIGINALADDRESSVALUE - EEP_INPUT1_RANGE];

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address - ORIGINALADDRESSVALUE < EEP_RESET )
			{
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = filter[i+address - ORIGINALADDRESSVALUE - EEP_INPUT1_FILTER];

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address - ORIGINALADDRESSVALUE == EEP_RESET )
			{
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			
			else if ( i+address - ORIGINALADDRESSVALUE <= EEP_INPUT1_CALIBRATION )
			{
				send_buffer = (calibration_offset[i+address - ORIGINALADDRESSVALUE - EEP_INPUT0_CALIBRATION]>>8)& 0XFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = calibration_offset[i+address - ORIGINALADDRESSVALUE - EEP_INPUT0_CALIBRATION];
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}			
			else if ( i+address - ORIGINALADDRESSVALUE == EEP_RANGE_LO)
			{
				send_buffer = (range_lo>>8)& 0XFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = range_lo;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address - ORIGINALADDRESSVALUE == EEP_RANGE_HI)
			{
				send_buffer = (range_hi>>8)& 0XFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = range_hi;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}


#else

			else if ( i+address - ORIGINALADDRESSVALUE <= EEP_OUTPUT8 )
			{
				unsigned int xdata data_temp ;
				if(switch_state[i+address - ORIGINALADDRESSVALUE - EEP_OUTPUT1] == 0 )
					data_temp = 0 ;
				else if(switch_state[i+address - ORIGINALADDRESSVALUE - EEP_OUTPUT1] == 1 )
					data_temp = 1000 ;
				else if(switch_state[i+address - ORIGINALADDRESSVALUE - EEP_OUTPUT1] == 2 )
					data_temp = modbus.registers[i+address - ORIGINALADDRESSVALUE] ;
				send_buffer = ((data_temp)>> 8) & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = data_temp & 0xFF;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
		#ifndef T3_8IN13OUT
				else if ( i+address - ORIGINALADDRESSVALUE <= EEP_INPUT8 )
					{
						send_buffer = (guiAnalogInput[i + address - ORIGINALADDRESSVALUE - EEP_INPUT1]>> 8) & 0xFF;
		       
						while (!transmit_finished){}
						SBUF = send_buffer;
						transmit_finished = 0;
						CRC16_Tstat(send_buffer);
		
						send_buffer = guiAnalogInput[i + address - ORIGINALADDRESSVALUE - EEP_INPUT1] & 0xFF;
		         
						while (!transmit_finished){}
						SBUF = send_buffer;
						transmit_finished = 0;
						CRC16_Tstat(send_buffer);
					}
		#endif
			#ifdef T3_8IO_A
			else if (( i+address - ORIGINALADDRESSVALUE >= EEP_DeadMaster_Ao0 )&&( i+address - ORIGINALADDRESSVALUE <= EEP_DeadMaster_Ao7 ))
		{
				send_buffer = (deadmaster_output[i+address -ORIGINALADDRESSVALUE -  EEP_DeadMaster_Ao0] >>8)&0xff;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = deadmaster_output[i+address -ORIGINALADDRESSVALUE -  EEP_DeadMaster_Ao0] &0xff;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}

	#endif
			else if ( i+address - ORIGINALADDRESSVALUE <= EEP_SWITCH_STATE2 )
			{				

				send_buffer = ((modbus.registers[i+address - ORIGINALADDRESSVALUE])>> 8) & 0xFF;
         // send_buffer=0;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = (modbus.registers[i+address - ORIGINALADDRESSVALUE]) & 0xFF;
          //send_buffer=0;
				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}

    	
 #ifndef T3_8IN13OUT
		else if ( i+address - ORIGINALADDRESSVALUE < EEP_INPUT1_FILTER )
			{
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = range[i + address - ORIGINALADDRESSVALUE - EEP_INPUT1_RANGE];

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address - ORIGINALADDRESSVALUE < TOTAL_EE_PARAMETERS )
			{
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = filter[i + address - ORIGINALADDRESSVALUE - EEP_INPUT1_FILTER];

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
			else if ( i+address - ORIGINALADDRESSVALUE == EEP_RESET )
			{
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = 99;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
	#endif
#endif
			//read pulse number register
	#if defined T3_8IN13OUT
				else if(i + address < T38IO_PULSE1_YEAR)
				{
								if(((i + address - T38IO_PULSE1_HI_WORD)%2)==1)
								{
								guiAnalogInput_temp[(i + address - T38IO_PULSE1_HI_WORD)/2]= pic.half_number[(i + address - T38IO_PULSE1_HI_WORD)]-DEFAULT_OFFSET +	 calibration_offset[(i + address - T38IO_PULSE1_HI_WORD)/2]; 							 	
								send_buffer = (guiAnalogInput_temp[(i + address - T38IO_PULSE1_HI_WORD)/2]>>8)&0xff;
								}
								else 
								send_buffer = pic.pulse_number[(i + address - T38IO_PULSE1_HI_WORD) << 1]; 
							
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
								
								if(((i + address - T38IO_PULSE1_HI_WORD)%2)==1)	
								send_buffer = (guiAnalogInput_temp[(i + address - T38IO_PULSE1_HI_WORD)/2])&0xff;
								else						 
								send_buffer = pic.pulse_number[((i + address - T38IO_PULSE1_HI_WORD) << 1) + 1];
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				
				}
				else if(i + address <= T38IO_PULSE8_MINUTE)
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 		 
								send_buffer = pulse_number[i + address - T38IO_PULSE1_YEAR];
							 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address == T38IO_CHANNEL_TYPE)
				{
								send_buffer = 0;				 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 	if(!flash_read_int(FLASH_CHANNEL_TYPE, &flash_data, FLASH_MEMORY))
								flash_data = 0xff;	 
								send_buffer = flash_data & 0xff;
							 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address <= 	T38IO_INPUT8_READING)
				{
						 	 
								send_buffer = adam_buffer[i + address - T38IO_INPUT1_READING] >> 8;
								//send_buffer = (guiAnalogInput[i + address - ORIGINALADDRESSVALUE - EEP_INPUT1]>> 8) & 0xFF; 
               // send_buffer=RangeConverter(/*rang[register_location]*/range[i + address - T38IO_INPUT1_READING],adam_buffer[i + address - T38IO_INPUT1_READING],128,0) >> 8 ;
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 		 
								send_buffer = adam_buffer[i + address - T38IO_INPUT1_READING] & 0xff;
							 // send_buffer = (guiAnalogInput[i + address - ORIGINALADDRESSVALUE - EEP_INPUT1]) & 0xFF;
              // send_buffer=RangeConverter(/*rang[register_location]*/range[i + address - T38IO_INPUT1_READING],adam_buffer[i + address - T38IO_INPUT1_READING],128,0);
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address <= 	T38IO_INPUT8_RANGE)
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 		 
							if(!flash_read_int(i + address - T38IO_INPUT1_RANGE + FLASH_INPUT1_RANGE, &flash_data, FLASH_MEMORY))
									flash_data = 0;		 // change the default range is 0 ;
								if(flash_data > 20)
									flash_data = 0;	
								send_buffer =  flash_data; 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address <= 	T38IO_INPUT8_FILTER)
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 		 
								if(!flash_read_int(i + address - T38IO_INPUT1_FILTER + FLASH_INPUT1_FILTER, &flash_data, FLASH_MEMORY))
									flash_data = 0;
							
								send_buffer = flash_data & 0xff;

								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address <= 	T38IO_INPUT8_TIMER)
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 		 
							   if(!flash_read_int(i + address - T38IO_INPUT1_TIMER + FLASH_INPUT1_TIMER, &flash_data, FLASH_MEMORY))
									flash_data = 89;
							 
								send_buffer = flash_data & 0xff;
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address <= 	T38IO_INPUT8_TIMER_LEFT)
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 
							 
								send_buffer = gucTimerLeft[i + address - T38IO_INPUT1_TIMER_LEFT];
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address == 	T38IO_OUTPUT_MANUAL)
				{
						 	 
								send_buffer = guiManual >> 8;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 
							 
								send_buffer = guiManual & 0xff;
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address <= 	T38IO_ZONE_OUTPUT13) 
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 
							 
								send_buffer = gucZone[i + address - T38IO_ZONE_OUTPUT1];
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address == T38IO_REVERSE_OUTPUT)
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 
							 	if(!flash_read_int(FLASH_REVERSE_OUTPUT, &flash_data, FLASH_MEMORY))
									flash_data = 89;
								send_buffer = flash_data & 0Xff;
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address <= 	T38IO_STATUS_OUTPUT13) 
				{
						 	 
								send_buffer = 0;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 
							 
								send_buffer = gucStatus[i + address - T38IO_STATUS_OUTPUT1];
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address < 	T38IO_CALIBRATION_OFFSET1) 
				{
						 	 //	send_buffer = 0 ;
								send_buffer = (calibration_offset[i + address - T38IO_CALIBRATION_OFFSET0]>>8)& 0XFF;	 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 
							 //	send_buffer = 88 ;
								send_buffer = (calibration_offset[i + address - T38IO_CALIBRATION_OFFSET0])& 0XFF;
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}
				else if(i + address == T38IO_FLASH_WRITE_TIME)
				{
					send_buffer = 0;	 
					while (!transmit_finished){}
					SBUF = send_buffer;
					transmit_finished = 0;
					CRC16_Tstat(send_buffer);
					
					send_buffer = flash_write_count& 0XFF;
					while (!transmit_finished){}
					SBUF = send_buffer;
					transmit_finished = 0;
					CRC16_Tstat(send_buffer);	
				}

					/* add debug information by chelsea*/
			/*	else if (i + address <= 	TT8) 
				{
								send_buffer = temp[i + address - TT1] >> 8;
								 
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
							 
							 
								send_buffer = temp[i + address - TT1];
								while (!transmit_finished){}
								SBUF = send_buffer;
								transmit_finished = 0;
								CRC16_Tstat(send_buffer);
				}*/
			#endif

			#ifdef CALIBRATION_OPTION
				// read registers between 300 and 460, calibration grid table saved in flash
				else if ( i+address - CALIBRATION_STORAGE_LOCATION < (MAX_INPUT_CHANNELS*MAX_CALIBRATION_POINTS) )
				{
					location = i+address - CALIBRATION_STORAGE_LOCATION;
	
					if(!flash_read_int(location,&flash_data, CALIBRATION_MEMORY) )
						flash_data = 8888;
				
					send_buffer = (flash_data>>8) & 0xFF;
					while (!transmit_finished){}
					SBUF = send_buffer;
					transmit_finished = 0;
					CRC16_Tstat(send_buffer);
					
					send_buffer = flash_data & 0xFF;
					while (!transmit_finished){}
					SBUF = send_buffer;
					transmit_finished = 0;
					CRC16_Tstat(send_buffer);
				}
				// read registers between 500 and 588, calibration tabs stored in flash memory, or dummy variables
				else if ( i+address - CALIBRATION_GRID < DEFINE_TABS*MAX_INPUT_CHANNELS )
				{
					location = i+address - CALIBRATION_GRID;
					input_select = location/DEFINE_TABS;
					
					flash_data = VoltageReading_defines[input_select][location - input_select*DEFINE_TABS];
//					flash_data = serial_input_grid[location/2][location%2];
				
					send_buffer = (flash_data>>8) & 0xFF;
					while (!transmit_finished){}
					SBUF = send_buffer;
					transmit_finished = 0;
					CRC16_Tstat(send_buffer);
					
					send_buffer = flash_data & 0xFF;
					while (!transmit_finished){}
					SBUF = send_buffer;
					transmit_finished = 0;
					CRC16_Tstat(send_buffer);
				}
			
			#endif
			// send default value of 1
			else
			{
				send_buffer = 0;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);

				send_buffer = 1;

				while (!transmit_finished){}
				SBUF = send_buffer;
				transmit_finished = 0;
				CRC16_Tstat(send_buffer);
			}
		}
		read_time_pulse = 0;
		// --- send CRC bits ---
		while (!transmit_finished){}
		SBUF = CRChi;
		transmit_finished = 0;
		while (!transmit_finished){}
		SBUF = CRClo;
		transmit_finished = 0;
		while (!transmit_finished){}
		// ----------------------

	}
	else if(data_buffer[1] == MULTIPLE_WRITE)
	{

		// --- response to a multiple write function ---
		// the 6 first bits are the same and then send the crc bits
		for ( i=0; i<6; i++)
		{
			send_buffer = data_buffer[i];
			SBUF = send_buffer;
			transmit_finished = 0;
			while (!transmit_finished){}
			CRC16_Tstat(send_buffer);
			watchdog();
		
		}
	

		// send the two last CRC bits
		SBUF = CRChi;
		transmit_finished = 0;
		while (!transmit_finished){}
		SBUF = CRClo;
		transmit_finished = 0;
		while (!transmit_finished){}

	}
	else if (data_buffer[1] == CHECKONLINE)
	{
		// send first byte of information
		send_buffer = data_buffer[0];
		SBUF = send_buffer;
		transmit_finished = 0;
		while (!transmit_finished){}
		CRC16_Tstat(send_buffer);
		watchdog();


		// send second byte of information
		send_buffer = data_buffer[1];
		SBUF = send_buffer;
		transmit_finished = 0;
		while (!transmit_finished){}
		CRC16_Tstat(send_buffer);
		watchdog();

		// send address of device
		send_buffer = laddress;
		SBUF = send_buffer;
		transmit_finished = 0;
		while (!transmit_finished){}
		CRC16_Tstat(send_buffer);
		watchdog();

		
		send_buffer = info[0];
		SBUF = send_buffer;
		transmit_finished = 0;
		while (!transmit_finished){}
		CRC16_Tstat(send_buffer);
		watchdog();
		send_buffer = info[1];
		SBUF = send_buffer;
		transmit_finished = 0;
		while (!transmit_finished){}
		CRC16_Tstat(send_buffer);
		watchdog();
		send_buffer = info[2];
		SBUF = send_buffer;
		transmit_finished = 0;
		while (!transmit_finished){}
		CRC16_Tstat(send_buffer);
		watchdog();
		send_buffer = info[3];
		SBUF = send_buffer;
		transmit_finished = 0;
		while (!transmit_finished){}
		CRC16_Tstat(send_buffer);
		watchdog();

		// send the two last CRC bits
		SBUF = CRChi;
		transmit_finished = 0;
		while (!transmit_finished){}
		SBUF = CRClo;
		transmit_finished = 0;
		while (!transmit_finished){}


		watchdog();
	}
	

}


// ---------------------- internaldeal ----------------------------
// when had received data ,the routine begin to dealwith internal by command external.
// --- variables for ISP ---

void internalDeal(unsigned int start_address)
{
	int itemp;
	unsigned char i, ucTemp,ucTemp1,temp;
	unsigned char uc_filter_temp = 0;
	unsigned int address_increment;
	unsigned char packet_counter;

	
	unsigned char block_to_erase;

	// --- variables for calibration ---
	unsigned char input_select=0;
	unsigned int cal_temp ;


	if(data_buffer[1] == WRITE_VARIABLES)
	{
		if(start_address  < ORIGINALADDRESSVALUE  )
		{					
			// If writing to Serial number Low word, set the Serial number Low flag
			if(data_buffer[3] <= FLASH_SERIALNUMBER_LOWORD+1)
			{	
				flash_write_int(FLASH_SERIALNUMBER_LOWORD, data_buffer[5], FLASH_MEMORY);
				flash_write_int(FLASH_SERIALNUMBER_LOWORD+1, data_buffer[4], FLASH_MEMORY);
				info[0] = data_buffer[5] ;
				info[1] = data_buffer[4] ;
				SNWriteflag |= 0x01;
				flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, SNWriteflag, FLASH_MEMORY);
				flash_write_int(FLASH_UPDATE_STATUS, 0, FLASH_MEMORY);
			}
			// If writing to Serial number High word, set the Serial number High flag
			else if(data_buffer[3] <= FLASH_SERIALNUMBER_HIWORD+1)
			{	
				flash_write_int(FLASH_SERIALNUMBER_HIWORD, data_buffer[5], FLASH_MEMORY);
				flash_write_int(FLASH_SERIALNUMBER_HIWORD+1, data_buffer[4], FLASH_MEMORY);
				info[2] = data_buffer[5] ;
				info[3] = data_buffer[4] ;
				SNWriteflag |= 0x02;
				flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, SNWriteflag, FLASH_MEMORY);
				flash_write_int(FLASH_UPDATE_STATUS, 0, FLASH_MEMORY);
			}
			else if(data_buffer[3] == FLASH_HARDWARE_REV)
			{	
				flash_write_int(FLASH_HARDWARE_REV, data_buffer[5]+ (data_buffer[4]<<8), FLASH_MEMORY);
				SNWriteflag |= 0x04;
				flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, SNWriteflag, FLASH_MEMORY);
				hardware_rev = data_buffer[5];
				
			}
			else if(data_buffer[3] == FLASH_PIC_VERSION)
			{	
				flash_write_int(FLASH_PIC_VERSION, data_buffer[5]+ (data_buffer[4]<<8), FLASH_MEMORY );
				pic_type = data_buffer[5]; 
			}

//			else if(data_buffer[3] == FLASH_SOFTWARE_VERSION_NUMBER)
//			{}	// cannot write to software version
//			else if(data_buffer[3] == FLASH_PRODUCT_MODEL)
//			{}	// cannot write to product model, hardcoded
			else if(data_buffer[3] == FLASH_ADDRESS )
			{
				flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
				laddress = data_buffer[5] ;
				iap_program_data_byte(0, DATA_TO_FIRMWARE + FLASH_ADDRESS);
				info[6] = data_buffer[5];
			}
			else if(data_buffer[3] == FLASH_CALIBRATION )
			{
				flash_write_int(data_buffer[3], data_buffer[5]/*+ (data_buffer[4]<<8)*/, FLASH_MEMORY);
				output_calibration = data_buffer[5];
			}
			#ifdef PWM_TRANSDUCER
				else if(data_buffer[3] == FLASH_PWM_TIME_CALIBRATION )	// sept 2005, Ron
				{
					flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
					time_calibration_offset = data_buffer[5] ;
				}
			#endif
			else if(data_buffer[3] == FLASH_SEQUENCE )				// july 21 Ron
			{
				flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
				output_sequence = data_buffer[5] ;
				// turn all LEDs when under testing SOP
				if (output_sequence == 0)
					startup_flag = 1;

			}
			else if(data_buffer[3] == FLASH_UPDATE_STATUS )			// july 21 Ron
			{
				flash_write_int(FLASH_UPDATE_STATUS, data_buffer[5], FLASH_MEMORY);
				update_flash = data_buffer[5] ;
			}

			else if(data_buffer[3] == FLASH_DEAD_MASTER )			//  
			{
				flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
				gucDeadmaster = data_buffer[5] ;
			 
			}
			else if(data_buffer[3] == FLASH_ADDRESS_PLUG_N_PLAY )
			{
				if(randval == RESPONSERANDVALUE)
				{
					flash_write_int(FLASH_ADDRESS, data_buffer[5], FLASH_MEMORY);
					laddress = 	data_buffer[5];	
					iap_program_data_byte(0, DATA_TO_FIRMWARE + FLASH_ADDRESS);
				}
			}
			else if( start_address == FLASH_BAUDRATE )
			{
				
			
				if(data_buffer[5] == 1)
				{
					TH1	  = 0xfd;
					TL1	  = 0xfd;
					PCON  = 0X80 ;
					SERIAL_RECEIVE_TIMEOUT = 3;
					flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
				}
				else if(data_buffer[5] == 0)
				{
					TH1	  = 0xfd;
					TL1	  = 0xfd;
					PCON  = 0X00 ;
					SERIAL_RECEIVE_TIMEOUT = 6;
					flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
				}
				else if(data_buffer[5] == 2)
				{
				TH1	  = 0xff;
				TL1	  = 0xff;
				PCON  = 0x80 ;
				SERIAL_RECEIVE_TIMEOUT = 3;
				flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
				}
			}
			else if( start_address == FLASH_RESPOND_DELAY )
			{
				if((data_buffer[5] >= 1)&&(data_buffer[5] <= 30))
				{
					com_dealy =  data_buffer[5] ;
					flash_write_int(FLASH_RESPOND_DELAY, com_dealy, FLASH_MEMORY);
				} 
			}			
			else if( start_address == FLASH_BASE_ADDRESS )
			{
//				flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);
//			
//				info[5] = 	data_buffer[5] ;
//				if(data_buffer[5] == 1)
//				{
//					PCON  = 0X80 ;
//					SERIAL_RECEIVE_TIMEOUT = 3;
//				}
//				else
//				{
//			 		PCON  = 0X00 ;
//					SERIAL_RECEIVE_TIMEOUT = 6;
//				}
			}
			else
				flash_write_int(data_buffer[3], data_buffer[5], FLASH_MEMORY);

		}

		else if(start_address  < CALIBRATION_STORAGE_LOCATION  )	// thus between 100 and 300
		{
			#if defined (T3_32IN)


				// store register address in start_address
				start_address = (unsigned int)(data_buffer[2]<<8) + data_buffer[3] - ORIGINALADDRESSVALUE;
				if ( (start_address >= EEP_INPUT1) && (start_address <= EEP_INPUT32) )
				{
					// store register value in itemp
				cal_temp = (data_buffer[4]<<8) + data_buffer[5];
				calibration_offset[start_address-EEP_INPUT1] =cal_temp-guiAnalogInput[start_address-EEP_INPUT1]+DEFAULT_OFFSET  ; 
				flash_write_int(FLASH_INPUT0_CALIBRATION + start_address-EEP_INPUT1, calibration_offset[start_address-EEP_INPUT1], FLASH_MEMORY);
				}
	
				else if ( (start_address >= EEP_LEDA_PAIR1) && (start_address <= EEP_LED_STATUS_PAIR32) )
				{
					// store register value in itemp
					itemp = (data_buffer[4]<<8) + data_buffer[5];
					if(itemp <= 255)
						modbus.registers[start_address] = itemp;
	

//					master_com_serial( 20, FlexDriver_LED_STATUS_PAIR1, itemp );
				}
				else if((start_address >= EEP_INPUT1_RANGE) && (start_address <= EEP_INPUT32_RANGE))
				{

					temp =  start_address - EEP_INPUT1_RANGE;
					if(data_buffer[5] < 20)
					{
						range[temp] = data_buffer[5];
						if((data_buffer[5] == 4)||(data_buffer[5] == 5))
						{
							for(uc_filter_temp = 0; uc_filter_temp< 32 ; uc_filter_temp++)
							{
								//flash_write_int(FLASH_INPUT1_FILTER + uc_filter_temp, 0, FLASH_MEMORY);
								filter[uc_filter_temp] = 0;
							}	
						}
						calibration_offset[temp] = 500 ;					 
						flash_write_int(FLASH_INPUT1_RANGE + temp, range[temp], FLASH_MEMORY);
						flash_write_int(FLASH_INPUT0_CALIBRATION + temp, calibration_offset[temp], FLASH_MEMORY);
					}
					reading_filter_bypass = 255;
				}
				else if((start_address >= EEP_INPUT1_FILTER) && (start_address <= EEP_INPUT32_FILTER)) //do this for work testing easier
				{
					temp =  start_address - EEP_INPUT1_FILTER;
					filter[temp] = data_buffer[5];
					if(data_buffer[4] >0)   data_buffer[5] = 255;
					flash_write_int(FLASH_INPUT1_FILTER + temp, filter[temp], FLASH_MEMORY);
					//for(uc_filter_temp = 0; uc_filter_temp< 32 ; uc_filter_temp++)
					//{
					//	flash_write_int(FLASH_INPUT1_FILTER + uc_filter_temp, data_buffer[5], FLASH_MEMORY);
					//	filter[uc_filter_temp] = data_buffer[5];
				//	}					
				}
				else if((start_address >= EEP_INPUT0_CALIBRATION) && (start_address <= EEP_INPUT1_CALIBRATION))
				{
					temp =  start_address - EEP_INPUT0_CALIBRATION;
					calibration_offset[temp] = (data_buffer[4]<<8)+data_buffer[5];
					flash_write_int(FLASH_INPUT0_CALIBRATION + temp, calibration_offset[temp], FLASH_MEMORY);			
				}
				else if(start_address == EEP_RANGE_LO)
				{
					range_lo = (data_buffer[4]<<8)+data_buffer[5];
					flash_write_int(FLASH_RANGE_LO , range_lo, FLASH_MEMORY);			
				}
				else if(start_address == EEP_RANGE_HI)
				{
					range_hi = (data_buffer[4]<<8)+data_buffer[5];
					flash_write_int(FLASH_RANGE_HI , range_hi, FLASH_MEMORY);	
				}
				else if(start_address == EEP_RESET)
				{
					if(data_buffer[5] == 1)
					{
							for(i=0; i< 32; i++)
							{
								range[i] = 0 ;
								flash_write_int(FLASH_INPUT1_RANGE + i, range[i] , FLASH_MEMORY);
								filter[i] = 2;
								flash_write_int(FLASH_INPUT1_FILTER + i, filter[i] , FLASH_MEMORY);
							}
					}		
				}
			#else

	
				// store register address in itemp
				start_address = (unsigned int)(data_buffer[2]<<8) + data_buffer[3] - ORIGINALADDRESSVALUE;
	
				if (start_address <= EEP_SWITCH_STATE2) // if the address is within the list of parameters
				{
					// store register value in itemp
					itemp = (unsigned int)(data_buffer[4]<<8) + data_buffer[5];
					if(itemp>1000) 	itemp = 1000 ;
					if(itemp <= 1023)
					{
					   #ifdef  T3_8IN13OUT
					   if(itemp > 1) itemp = 1;
					   #endif
						modbus.registers[start_address] = itemp;

					}		
	
				}
				#ifndef  T3_8IN13OUT
				else if (start_address < EEP_INPUT1_FILTER) // if the address is within the list of parameters
				{
					temp =  start_address - EEP_INPUT1_RANGE;
					if(data_buffer[5] < 20)
					{
						range[temp] = data_buffer[5];	
						flash_write_int(FLASH_INPUT1_RANGE + temp, range[temp], FLASH_MEMORY);
					}
	
				}
				else if (start_address < TOTAL_EE_PARAMETERS) // if the address is within the list of parameters
				{
					temp =  start_address - EEP_INPUT1_FILTER;
					//filter[temp] = data_buffer[5];
					if(data_buffer[4] >0)   data_buffer[5] = 255;
					for(i = 0 ; i<8; i++)
					{
				    	filter[i]  =  data_buffer[5] ;
						flash_write_int(FLASH_INPUT1_FILTER + i, filter[i], FLASH_MEMORY);
					}	
				}
				else if (start_address == EEP_RESET) // if the address is within the list of parameters
				{

					if(data_buffer[5] == 1)
					{
							for(i = 0 ; i<8; i++)
							{
								filter[i]  =  2 ;
								flash_write_int(FLASH_INPUT1_FILTER + i, filter[i], FLASH_MEMORY);
								range[i]  =  0 ;
								flash_write_int(FLASH_INPUT1_RANGE + i, range[i], FLASH_MEMORY);
							}	
					}
				}
							else if ((start_address >= EEP_DeadMaster_Ao0 )&&( start_address <= EEP_DeadMaster_Ao7 ))
			{
				
				temp =  start_address - EEP_DeadMaster_Ao0;
				deadmaster_output[temp] = (data_buffer[4]<<8)|data_buffer[5] ;
				flash_write_int(FLASH_DEADMASTER_AO0 + temp, deadmaster_output[temp], FLASH_MEMORY);
			}
				
				#endif
				#if defined T3_8IN13OUT
				else if(start_address >= (T38IO_PULSE1_HI_WORD - ORIGINALADDRESSVALUE) && start_address <= T38IO_PULSE8_LO_WORD - ORIGINALADDRESSVALUE)
					{
											
						if(((start_address- T38IO_PULSE1_HI_WORD)%2)==1)
						{	
							cal_temp = (data_buffer[4]<<8) + data_buffer[5]; 
							calibration_offset[(start_address- T38IO_PULSE1_HI_WORD+ORIGINALADDRESSVALUE)/2] =cal_temp-pic.half_number[(start_address- T38IO_PULSE1_HI_WORD+ORIGINALADDRESSVALUE)]+DEFAULT_OFFSET  ;  
							flash_write_int(FLASH_INPUT0_CALIBRATION + (start_address- T38IO_PULSE1_HI_WORD+ORIGINALADDRESSVALUE)/2, calibration_offset[(start_address- T38IO_PULSE1_HI_WORD+ORIGINALADDRESSVALUE)/2], FLASH_MEMORY);	
						}	
					} 
					else if(start_address >= (T38IO_PULSE1_YEAR - ORIGINALADDRESSVALUE) && start_address <= (T38IO_PULSE8_MINUTE - ORIGINALADDRESSVALUE))
					{												   
						{	//flash_write_int(FLASH_INPUT1_RANGE, 1, FLASH_MEMORY);
							temp = (start_address - (T38IO_PULSE1_YEAR - ORIGINALADDRESSVALUE))/5;
							if(GetBit(temp,&channel_type))
							{
								if((start_address - (T38IO_PULSE1_YEAR - ORIGINALADDRESSVALUE)) % 5 == 0)
								{
 									
									SetBit(temp ,&clear_pulse_channel);
									flash.number_long[temp] = 0;
									old_reading[temp] = 0;
 									gbClearPulse = 1;
									gucClearPulse = temp; 
									
									start_timer(STORE_PULSE,1);

								}
								pulse_number[start_address - (T38IO_PULSE1_YEAR - ORIGINALADDRESSVALUE)] = data_buffer[5];
							}
						}
					}
				else if(start_address == T38IO_CHANNEL_TYPE - ORIGINALADDRESSVALUE)
				{
					channel_type = data_buffer[5];
					temp = data_buffer[5];
					gbSetChannel = 1;
					gucSetChannel = channel_type;
				 
					flash_write_int(FLASH_CHANNEL_TYPE, temp, FLASH_MEMORY);
					reading_filter_bypass = 30;

					for(i=0;i<8;i++)
					{
						if(GetBit(i,&channel_type))
						{		
							range[i] = 6;
							flash_write_int(FLASH_INPUT1_RANGE + i, range[i], FLASH_MEMORY);		
						}
						else
						{
							if(range[i] == 6)
							{
								range[i] = 0;
								flash_write_int(FLASH_INPUT1_RANGE + i, range[i], FLASH_MEMORY);
							}
						}
					}
					 
				}
				else if(start_address >= T38IO_INPUT1_RANGE - ORIGINALADDRESSVALUE && start_address <= T38IO_INPUT8_RANGE - ORIGINALADDRESSVALUE)
				{					
					temp =  start_address - (T38IO_INPUT1_RANGE - ORIGINALADDRESSVALUE);
					calibration_offset[temp] = 500 ;  
					flash_write_int(FLASH_INPUT0_CALIBRATION + temp, calibration_offset[temp], FLASH_MEMORY);
					if(data_buffer[5] < 20)
					{
						range[temp] = data_buffer[5];					 		
						flash_write_int(FLASH_INPUT1_RANGE + temp, range[temp], FLASH_MEMORY);						

						if(data_buffer[5] == 6)//pulse input
						{
							SetBit(temp,&channel_type);
						
						}
						else
							ClearBit(temp,&channel_type);
						if(data_buffer[5] == 7)//lighting control
						{
							filter[temp] = 0;	
					 		SetBit(temp,&channel_type);
							flash_write_int(FLASH_INPUT1_FILTER + temp, filter[temp], FLASH_MEMORY);
						}

						gbSetChannel = 1;
						gucSetChannel = channel_type;				 
						flash_write_int(FLASH_CHANNEL_TYPE, channel_type, FLASH_MEMORY);
						reading_filter_bypass = 30;
					}
				 		 
					 
				} 
				else if(start_address >= T38IO_INPUT1_FILTER - ORIGINALADDRESSVALUE && start_address <= T38IO_INPUT8_FILTER - ORIGINALADDRESSVALUE)
				{
					temp =  start_address - (T38IO_INPUT1_FILTER - ORIGINALADDRESSVALUE);
				    filter[temp] =  data_buffer[5];
					if(data_buffer[4] >0) filter[temp] = 255;
					for(uc_filter_temp = 0; uc_filter_temp < 8 ; uc_filter_temp++)	//do this for work testing easier
					{
						filter[uc_filter_temp] =  data_buffer[5];
						flash_write_int(FLASH_INPUT1_FILTER + uc_filter_temp, filter[uc_filter_temp], FLASH_MEMORY);	
					}
                    					 
				}
				else if(start_address >= T38IO_INPUT1_TIMER - ORIGINALADDRESSVALUE && start_address <= T38IO_INPUT8_TIMER - ORIGINALADDRESSVALUE)
				{

					temp =  start_address - (T38IO_INPUT1_TIMER - ORIGINALADDRESSVALUE);				  
					gucTimer[temp] = data_buffer[5];				 
					flash_write_int(FLASH_INPUT1_TIMER + temp, gucTimer[temp], FLASH_MEMORY); 	
				 			 
					 
				} 
				else if(start_address == T38IO_OUTPUT_MANUAL - ORIGINALADDRESSVALUE) 
				{
 				 
					 
					guiManual= (unsigned int)(data_buffer[4] << 8) + data_buffer[5];				 
					flash_write_int(FLASH_OUTPUT_MANUAL, guiManual, FLASH_MEMORY); 	
				  		 
					 
				} 
				else if(start_address >= T38IO_ZONE_OUTPUT1 - ORIGINALADDRESSVALUE && start_address <= T38IO_ZONE_OUTPUT13 - ORIGINALADDRESSVALUE)
				{

					temp =  start_address - (T38IO_ZONE_OUTPUT1 - ORIGINALADDRESSVALUE);
					if(data_buffer[5] < 9)
					{			 
					 
						gucZone[temp] = data_buffer[5];				 
						flash_write_int(FLASH_ZONE_OUTPUT1 + temp, gucZone[temp], FLASH_MEMORY); 
					}	
				 			 
					 
				}  
				else if(start_address == T38IO_REVERSE_OUTPUT - ORIGINALADDRESSVALUE)
				{	 
					gucReverseOutput = data_buffer[5];				 
					flash_write_int(FLASH_REVERSE_OUTPUT, gucReverseOutput, FLASH_MEMORY); 		 
				}
				else if(start_address == T38IO_FLASH_WRITE_TIME - ORIGINALADDRESSVALUE )
				{
					flash_write_count = data_buffer[5] ;
					flash_write_int(FLASH_FLASH_WRITE_TIME, (unsigned int)flash_write_count, FLASH_MEMORY); 
				} 
			 	#endif


				else if( start_address == EEP_SERINALNUMBER_WRITE_FLAG )
				{	itemp = data_buffer[5];
					flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, itemp, FLASH_MEMORY);
					SNWriteflag = data_buffer[5];
				}

			#endif
		}

		#ifdef CALIBRATION_OPTION
			else if(start_address  < (CALIBRATION_STORAGE_LOCATION+(MAX_INPUT_CHANNELS<<1) )  )	// thus between 300 and 316
			{
				start_address = (data_buffer[2]<<8) + data_buffer[3] - CALIBRATION_STORAGE_LOCATION;
				itemp = (data_bu ffer[4]<<8) + data_buffer[5];

				serial_input_grid[start_address>>1][start_address%2] = itemp;
				calibration_address = start_address;	// writing this variable will activate the storing function (store_data_to_grid)
														// when all point have been entered, user must activate SOP to tabulate LUT
			}
		#endif

		 
	}
	else if(data_buffer[1] == MULTIPLE_WRITE)
	{
		#if defined (T3_32IN)
			packet_counter = data_buffer[5];			// for now buffer[4] will remain zero...
			start_address = (data_buffer[2]<<8) + data_buffer[3] - ORIGINALADDRESSVALUE;	// start address of where writing begins
	
	
			// --- program stuff subroutine ---
			// use Flash Write subroutine multiple times in order to write to the flash.
			// I have allowed to code to be able to handle a message size of 10 value, thus packet size 29=(10*2 + 9)
			for(address_increment=0; address_increment<packet_counter; address_increment++)
			{	
				watchdog();
				// only allow multiple-write to b.register for the LED pair table
				if ( ( (start_address+address_increment) >= EEP_LEDA_PAIR1) || ( (start_address+address_increment) <= EEP_LED_STATUS_PAIR32) )
				{	itemp = (data_buffer[7 + (address_increment<<1)]<<8) + data_buffer[8 + (address_increment<<1)];
					if(itemp <= 255)
						modbus.registers[ start_address ] = itemp;

					// given just received data, now want to translate to table and then send data out
				}
				ucTemp = start_address + address_increment;
				if(ucTemp >= 128 && ucTemp <= 159)
				{
					ucTemp1 = ucTemp - 128;
					range[ucTemp1] = itemp;
					flash_write_int(FLASH_INPUT1_RANGE + ucTemp1, range[ucTemp1], FLASH_MEMORY);
				} 
				
			}
		#else
			packet_counter = data_buffer[5];			// for now buffer[4] will remain zero...
			start_address = (data_buffer[2]<<8) + data_buffer[3];	// start address of where writing begins

			start_address = start_address - 100;
	
	
			// --- program stuff subroutine ---
			// use Flash Write subroutine multiple times in order to write to the flash.
			// I have allowed to code to be able to handle a message size of 10 value, thus packet size 29=(10*2 + 9)
			for(address_increment=0; address_increment<packet_counter; address_increment++)
			{	
				watchdog();
				itemp = (data_buffer[7 + (address_increment<<1)]<<8) + data_buffer[8 + (address_increment<<1)];
				ucTemp = start_address + address_increment;
				modbus.registers[ucTemp] = itemp;
				#ifdef T3_8IN16OUT
				if(ucTemp >= 28 && ucTemp <= 35)
				{
					ucTemp1 = ucTemp - 28;
					range[ucTemp1] = itemp;
					flash_write_int(FLASH_INPUT1_RANGE + ucTemp1, range[ucTemp1], FLASH_MEMORY);
				} 
								
				#elif defined T3_8IN13OUT
				if(ucTemp >= 83 && ucTemp <= 90)
				{
					ucTemp1 = ucTemp - 83;
					range[ucTemp1] = itemp;
					flash_write_int(FLASH_INPUT1_RANGE + ucTemp1, range[ucTemp1], FLASH_MEMORY);
				} 
				#elif defined  T3_8IO_A
				if(ucTemp >= 18 && ucTemp <= 25)
				{
					ucTemp1 = ucTemp - 18;
					range[ucTemp1] = itemp;
					flash_write_int(FLASH_INPUT1_RANGE + ucTemp1, range[ucTemp1], FLASH_MEMORY);
				} 
				

				#endif
			}
					
		#endif


	}

	// --------------- update initialize --------------------

	if (update_flash == 0x7F)
	{
		unsigned char temp;	
		// erase stuff on FLASH memory and write to it such that upon reset can know to jump to ISP
		block_to_erase = DATA_TO_ISP>>9;
		iap_erase_block(block_to_erase);


		// read usefull info stored on flash and store it in a specific memory location
		for( i=0; i<=16; i++)
		{
			if( flash_read_char(i,&temp,FLASH_MEMORY ) )
				iap_program_data_byte(temp, DATA_TO_ISP + i);
		}
		iap_program_data_byte(0, DATA_TO_ISP + 19);
		// disable the global interrupts
		EA = 0;

		WDTC = 0x80; 	//reset the CPU
		while(1){};

	}
	else if(update_flash == 0x8e)
	{
		SNWriteflag = 0x00;
		flash_write_int(EEP_SERINALNUMBER_WRITE_FLAG, SNWriteflag, FLASH_MEMORY);
//		EA = 0;
//		WDTC = 0x80; 	//reset the CPU
//		while(1){};	
	}
	// --------------- reset board -------------------------------------------
	else if (update_flash == 0xFF)
	{	
		// disable the global interrupts
		EA = 0;
		WDTC = 0x80; 	//reset the CPU
		while(1){};

	}

}

/////------------------------------- initcrc16 ---------------------
///  init crc 
void InitCRC16(void)
{
	CRClo = 0xFF;
	CRChi = 0xFF;
}
//-------------------crc16_tstat ---------------------
// calculate crc with one byte
void CRC16_Tstat(unsigned char ch)
{
	unsigned char uIndex ;
	uIndex = CRChi ^ ch ; // calculate the CRC 
	CRChi = CRClo ^ auchCRCHi[uIndex] ;
	CRClo = auchCRCLo[uIndex] ;
}


/// ----------------------crc16 --------------------------
//calculate crc 
unsigned int CRC16(unsigned char *puchMsg, unsigned char usDataLen)
{
	unsigned int uchCRCHi = 0xFF ; // high byte of CRC initialized 
	unsigned char uchCRCLo = 0xFF ; // low byte of CRC initialized 
	unsigned char uIndex ; // will index into CRC lookup table 
	while (usDataLen--) // pass through message buffer 
		{
			uIndex = uchCRCHi ^ *puchMsg++ ; // calculate the CRC 
			uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
			uchCRCLo = auchCRCLo[uIndex] ;
		}
	return (uchCRCHi << 8 | uchCRCLo) ;
}




