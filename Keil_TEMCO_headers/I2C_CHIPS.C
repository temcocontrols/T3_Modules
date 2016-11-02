//;**************************************************
/*	
	Eventually we would want to have all our chips using 
	the same I2C communication protocol.

	clock chip header functions
	pic chip header function
	E2chip header function
	
	usefull routines:
		- initialize_clock();		// must call to set clock chip

		- read_clock(arg1);		// where arguement must be one of the defines listed below
								// routine will return an unsigned char
			example: buffer = read_clock(MINUTE)
		- write_clock(arg1, arg2);	// arg1 is on of the degines listed below
									// arg2 is the value we want to write in
			example: write_clock(MONTH, 2)	to write in month of february

	USER must define sbit externally for clock and data lines
sbit	I2C_CLOCK 	= P2^6;
sbit	I2C_DATA    = P2^7;


*/
//;**************************************************


#define	  MINUTE  0x03
#define	  HOUR	  0x04
#define	  DATE 	  0x05	
#define	  MONTH   0x07
#define	  DAY     0x06      
#define	  YEAR    0x08   


#include "I2C_chips.h"




/*;***********************************/
/*;i2c_startup sequence of 24Cxx*/
void i2c_start()
{
	I2C_CLOCK=0;
	I2C_DATA = 1;
	I2C_CLOCK = 1;
	I2C_DATA = 0;
	I2C_CLOCK=0;
}



/*;************************************/
/*;i2c_stop sequence of 24Cxx*/
void i2c_stop()
{
	I2C_DATA = 0;
	I2C_CLOCK = 1;
	I2C_DATA = 1;
}

/*;**************************************/
/*;detect the ACK signal to 24Cxx*/
bit ACK( void )
{
   bit c;
   I2C_DATA=1;
   I2C_CLOCK=1;
   c=I2C_DATA;
   I2C_CLOCK=0;
   return c;
}



/*;************************************/
/*;send a 8-bit data to 24Cxx*/
void i2c_write( unsigned char ch )
{
	unsigned char i = 8;
	do
	{    
		I2C_DATA = ( ch & 0x80 );
		I2C_CLOCK=1;
		I2C_CLOCK=0;
		ch<<=1;

	} while( --i != 0 );
}


/*;**************************************/
/*;receive a 8-bit data from 24Cxx*/
unsigned char i2c_read( void )
{
	unsigned char i, data1 = 0;

	for( i=0; i<8; i++ )
	{
		I2C_CLOCK = 1;
		data1 = ( data1 << 1 ) | I2C_DATA;
		I2C_CLOCK = 0;
		delay_us(1);  // this is to slow down all i2c communication so that the pic chip 
					  // is able to run better. MDF 12/01/04
	}

	return data1;
}






//;**************************************************
//;**************************************************
//;**************************************************
unsigned char read_clock(unsigned char address)
{
	unsigned char clock;

	i2c_start();

	i2c_write( 0xa2 );
	ACK();
	i2c_write( address );
	ACK();

	i2c_start();

	i2c_write( 0xa3  );
	ACK();

	clock = i2c_read();

	i2c_stop();

	return(clock) ;
}


void write_clock(unsigned char address ,unsigned char value)
{
	i2c_start();
	
	i2c_write( 0xa2  );
	ACK();
	
	i2c_write( address );
	ACK();
	
	i2c_write(value);
	ACK();
	
	i2c_stop();
}


void initialize_clock(void)
{

	unsigned char buf;

	buf=read_clock(YEAR);
	
	if(buf>0x99)
	{
		write_clock(YEAR,0);
		delay_us( 30 ) ; 
	}

	buf=read_clock(MONTH);
	buf=buf&0x1f;
	
	if((buf>0x12)||(buf<0x01))
	{
		write_clock(MONTH,1);
		delay_us( 30 ) ; 
	}

	buf=read_clock(DATE);
	buf=buf&0x3f;
	
	if((buf>0x31)||(buf<0x01))
	{
		write_clock(DATE,1);
		delay_us( 30 ) ; 
	}

	buf=read_clock(HOUR);
	buf=buf&0x3f;
	
	if(buf>0x23)
	{
		write_clock(HOUR,0);
		delay_us( 30 ) ; 
	}

	buf=read_clock(MINUTE);
	buf=buf&0x7f;
	
	if(buf>0x59)
	{
		write_clock(MINUTE,0);
		delay_us( 30 ) ; 
	}

}


//;**************************************************
//;**************************************************
//;**************************************************


























