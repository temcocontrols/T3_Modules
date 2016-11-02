//;**************************************************
/*	LCD header functions
	
	USER must define sbit externally for clock, RS and RW lines

*/
//;**************************************************

//;**************************************************
//	command line necessary for LCD controls
// as well need to wire serial ports to P0
/*
sbit	LCD_ECLK     = P3^4 ;	// clock line
sbit	LCD_RS       = P3^5 ;	// data line
sbit	LCD_RW     	 = P2^1 ;	// uually grounded given always write
*/
//;**************************************************

#include "LCD_display.h"

extern void watchdog(void);
extern void delay_us(unsigned int time);


//;**************************************************
void write_command(unsigned char value )
	{
		LCD_RS=0 ;
		LCD_RW=0 ;
		LCD_ECLK=0 ;
		P0=value ;   
		delay_us(10);
		LCD_ECLK=1 ;
		delay_us(10);
		LCD_ECLK=0 ;
	}

//;**************************************************
void write_ddram(unsigned char value ,unsigned char address)
	{

		LCD_RW=0 ;
		LCD_RS=0 ;
		LCD_ECLK=0 ;
		P0=address|0x80 ;   
		LCD_ECLK=1 ;
		LCD_ECLK=0 ;

		delay_us(30);
		LCD_RS=1 ;
		P0=value ;   
		LCD_ECLK=1 ;
		LCD_ECLK=0 ;
	}

//;**************************************************
void set_cgram_address(unsigned char address)
	{
		LCD_RW=0 ;
		LCD_RS=0 ;
		LCD_ECLK=0 ;
		P0=address|0x40 ;   
		LCD_ECLK=1 ;
		LCD_ECLK=0 ;
		delay_us(5);
	}


//;**************************************************
void write_cgram(unsigned char value)
	{
		LCD_RS=1 ;
		P0=value ;   
		LCD_ECLK=1 ;
		LCD_ECLK=0 ;
		delay_us(50);
	}

//;**************************************************
void init_lcd(void)
	{
		LCD_RS=0 ;
		LCD_RW=0 ;
		LCD_ECLK=0 ;
		write_command(0x38); 
		delay_us(1000);
		write_command(0x0c);
		delay_us(1000);
		write_command(0x01);
		delay_us(1000);
		write_command(0x06);  //Set address to automatically increment when read or write RAM
		delay_us(1000);


		// Write custom Up/Down character to address 0x00 of the CGRAM
/*		set_cgram_address (0x00);
		write_cgram (0x04);
		write_cgram (0x0e);
		write_cgram (0x15);
		write_cgram (0x00);
		write_cgram (0x15);
		write_cgram (0x0e);	
		write_cgram (0x04);
		write_cgram (0x00);
*/
		set_cgram_address (0x00);
		write_cgram (0x04);
		write_cgram (0x0e);
		write_cgram (0x1f);
		write_cgram (0x15);
		write_cgram (0x04);
		write_cgram (0x04);	
		write_cgram (0x00);
		write_cgram (0x00);


		set_cgram_address (0x08);
		write_cgram (0x00);
		write_cgram (0x00);
		write_cgram (0x04);
		write_cgram (0x04);
		write_cgram (0x15);
		write_cgram (0x1f);	
		write_cgram (0x0e);
		write_cgram (0x04);

	}




//;**************************************************
//;**************************************************
//;**************************************************
//;**************************************************
//;**************************************************
/*	LCD header functions for Boiler Controller
	
	USER must define sbit externally for LCD_ECLK, LCD_RS and LCD_RW lines

*/
//;**************************************************

//;**************************************************
//	command line necessary for LCD controls
// as well need to wire serial ports to P0
/*
sbit	LCD_ECLK     = P3^4 ;	// clock line
sbit	LCD_RS       = P3^5 ;	// data line
sbit	LCD_RW     	 = P2^1 ;	// uually grounded given always write
*/
//;**************************************************
/*
#include "LCD_Display.h"

#define FIRMWARE_VERSION 18


unsigned char const code menu_lcd_table[61][16]= 
{	
	"Outside Tmp Cal" ,		//0    single point calibration of temperature
	"Solar Tmp Cal" ,		//1    single point calibration of temperature
	"Pool Tmp Cal" ,		//2    single point calibration of temperature
// 	"Cooling PTerm" ,		//3     proportional term coolling  
// 	"Cooling ITerm" ,		//4    integral term coolling  
// 	"Heating PTerm" ,		//5     proportional term heating  
// 	"Heating ITerm" ,		//6    integral term heatin
//    "Heating Deadbnd" ,   	//9 heating deadbandg
//    "Cooling Deadbnd" ,     //8 cooling deadband

	"Solar Diff." ,   		//10
	"Metric or Imp.?" ,		//7  Degrees C or Deg F
	"Daylight Savings" , 	//11
	"Flow Rate" ,			//12 flow rate
	"$/GJ" ,				//13
	"Heat Efficiency", 		//14
	"Aux. Relay", 		
	"Pump", 				
	"Reset THERMS?" ,		//15
	"Factory Defaults" ,	//16

	"SETPOINT",				//17  heating setpoint

	"TIME", 				//18  set clock
	"ADJUST DATE", 			//19  set clock
	"PUMP SCHEDULE", 		//20   
	"AUX SCHEDULE",
	"DISPLAY MODE", 		//21 set temperature display mode

	"ON",  					//22
	"OFF",  				//23
	"SOLAR    ",			//24		display temperature
	"POOLTEMP ",			//25		display temperature
	"RETURN   ",			//26		display temperature
	"OUTSIDE  ",			//27		display temperature
	"rotating ",   			//28
	"Are you sure?Y",   	//29

	"Are you sure?N",   	//30
	"Metric",  				//31
	"Imperial",  			//32
	"USD", 					//33
	"Dollars Saved", 		//34
	"THERMS Saved", 		//35
	"Copy to all days",		//36
	"Version",		 		//37
	"EEPROM Error",	 		//38
	"adjust",	 			//39

	"SENSOR",	 			//39
	"DISCONNECTED!!",		//39

	"ADJUST MIN",			//40	
	"ADJUST HRS",	 		//41
	"solar temp", 			//42
	"pool temp",			//43
	"return temp",			//43
	"outside temp",			//43	
	"therms to date",		//44
	"gpm",					//45
	"/therm",				//46
	"/GJ",					//47
	"settings",				//49
	"pump on",				//50
	"if solar on    ",		//51
	"if solar on    ",		//52
	"timer only     ",		//53
	"solar off      ",		//54
	"Aux. on   ",			//55
	"engaged sched. ",		//56

	"loading...",			//57
	"FLOW RATE",			//58
	"flow rate",			//59
	"lpm"					//60


};
	

unsigned char const code set_day_table[ 8][4]= 
{

	"SUN",  //1
	"MON",  //2
	"TUE",  //3
	"WED",  //3
	"THU",  //5
	"FRI",  //6
	"SAT",  //7
	"ALL"  //0	

	
};

unsigned char const code set_month_table[ 13][4]= 
{
	"   ",  //0
	"JAN",  //1
	"FEB",  //2	
	"MAR",  //3
	"APR",  //4
	"MAY",  //5
	"JUN",  //6
	"JUL",  //7
	"AUG",  //8
	"SEP",  //9
	"OCT",  //10
	"NOV",  //11
	"DEC",  //12
};

unsigned char const code set_on_off[ 2][9]= 
{
	"      ON",  //0
	"     OFF",  //1
};

//;**************************************************
void clear_lcd_line1(void)
	{
		unsigned char i;
		for(i=0;i<16;i++)
			{	
				write_ddram(0x20,i);
				delay_us(30);
	    	 }
	}

void clear_lcd_line2(void)
	{
		unsigned char i;
		for(i=0;i<16;i++)
			{	
				write_ddram(0x20,i+0x40);
				delay_us(30);
	    	 }
	}



//;**************************************************
void show_lcd_menu(unsigned char menu,unsigned char address)
	{
		unsigned char i=0;

		do{	
			write_ddram(menu_lcd_table[menu][i],address+i);
			delay_us(30);
			i++;
		}while(menu_lcd_table[menu][i]!='\0');
	}


//;**************************************************
void show_month(unsigned char month,unsigned char address)
	{
		unsigned char i=0;
		
		do{	
			write_ddram(set_month_table[month][i],i+address);
			delay_us(30);
			i++;
		}while(set_month_table[month][i]!='\0');
	}

//;**************************************************
void show_day(unsigned char day,unsigned char address)
	{
		unsigned char i=0;
		
		do{	
			write_ddram(set_day_table[day][i],i+address);
			delay_us(30);
			i++;
		}while(set_day_table[day+1][i]!='\0');
	}

//;**************************************************
void show_on_off(void)
	{
		unsigned char i=0;
		
		do{	
			write_ddram(set_on_off[0][i],i);
			delay_us(30);
			write_ddram(set_on_off[1][i],i+0x40);
			delay_us(30);
			i++;
		}while(set_on_off[0][i]!='\0');
	}


void show_schedule_time(void)
	{
		unsigned char buf;

		buf=on_minute ;
		write_ddram((buf%10)+0x30,LCD_MINUTE-0x40);
		delay_us(30);
		write_ddram((buf/10)+0x30,LCD_MINUTE-0x40-1);
		delay_us(30);
    	write_ddram(0x3a,0x4c-0x40);
		delay_us(30);

		buf=on_hour;
		write_ddram((buf%10)+0x30,LCD_HOUR-0x40);
		delay_us(30);
		write_ddram((buf/10)+0x30,LCD_HOUR-0x40-1);
		delay_us(30);
		write_ddram(0x20,0x49-0x40);
		delay_us(30);

		buf=off_minute ;
		write_ddram((buf%10)+0x30,LCD_MINUTE);
		delay_us(30);
		write_ddram((buf/10)+0x30,LCD_MINUTE-1);
		delay_us(30);
    	write_ddram(0x3a,0x4c);
		delay_us(30);

		buf=off_hour;
		write_ddram((buf%10)+0x30,LCD_HOUR);
		delay_us(30);
		write_ddram((buf/10)+0x30,LCD_HOUR-1);
		delay_us(30);
		write_ddram(0x20,0x49);
		delay_us(30);

	}


void display_time(unsigned char address)
	{
		unsigned char buf;
  			buf=read_clock(MINUTE) ;
			buf=buf&0x7f;
			write_ddram((buf&0x0f)+0x30,address+4);
    		delay_us(30);
			buf=buf>>4 ;
			write_ddram((buf&0x0f)+0x30,address+3);
			delay_us(30);
        	write_ddram(0x3a,address+2);
			delay_us(30);

			buf=read_clock(HOUR) ;
			buf=buf&0x3f;
			write_ddram((buf&0x0f)+0x30,address+1);
			delay_us(30);
			buf=buf>>4 ;
			write_ddram((buf&0x0f)+0x30,address);
			delay_us(30);
	}


void display_date(unsigned char address)
	{
			unsigned char buf;
			buf=read_clock(MONTH) ;
			buf=buf&0x1f;
			buf=(buf/16)*10+buf%16;
			if(buf>12)
			buf = 1 ;
			show_month(buf,address);
			buf=read_clock(DATE) ;
			buf=buf&0x3f;
			write_ddram((buf&0x0f)+0x30,address+4);
			delay_us(30);
			buf=buf>>4 ;
			write_ddram((buf&0x0f)+0x30,address+3);
			delay_us(30);
 	}


void display_year(unsigned char address)
	{
			unsigned char buf;
			buf=read_clock(YEAR) ;
			write_ddram(0x32,address);
			delay_us(30);
			write_ddram(0x30,address+1);
			delay_us(30);
			write_ddram((buf&0x0f)+0x30,address+3);
			delay_us(30);
			buf=buf>>4 ;
			write_ddram((buf&0x0f)+0x30,address+2);
			delay_us(30);
	}


void display_day(unsigned char address)
	{
			unsigned char buf;
			buf=read_clock(DAY) ;
			buf=buf&0x07;
			show_day(buf,address);
	}


	
// --------------display_ver ----------------------------
//   show version and init rand value.
void display_ver()
	{
		unsigned char rval = 0;	
		unsigned char DispBuf[3];
										
		DispBuf[0]=(FIRMWARE_VERSION%100)/10;
		DispBuf[1]=FIRMWARE_VERSION%10;

		clear_lcd_line1();
		clear_lcd_line2();

			
		delay_us(50000);	// pause to allow viewer to see
		watchdog();
		delay_us(50000);
		watchdog();
		delay_us(50000);
		watchdog();
		delay_us(50000);
		watchdog();

		show_lcd_menu(VERSION, 0x00);
		
		write_ddram(DispBuf[0] + 0x30,0x08);
		write_ddram(0x2e,0x09);		
		write_ddram(DispBuf[1] + 0x30,0x0A);

		watchdog();
		delay_us(50000);	// pause to allow viewer to see
		watchdog();
		delay_us(50000);
		watchdog();
		delay_us(50000);
		watchdog();

	}

*/

