/*-----------------pic.h---------------------*/


#define READING_THRESHOLD	150

// PIC commands protocol
#define PIC_MUX1_CHANNEL 		0xB1
#define PIC_MUX2_CHANNEL   		0xB5
#define PIC16_PULSE    	 		0xB7
#define PIC16_PULSE_CLEAR		0xB8
#define PIC16_VERSION			0xB9
#define PIC16_CHANNEL_TYPE      0xBA

void pic_detect(void);

bit read_pic_original( unsigned char store_location, bit set );
void analog_input_buffer (unsigned int *raw_data, unsigned char length, unsigned char start);

bit read_pic(void);
void store_to_registers (unsigned int pic_data, unsigned char start);
 
void read_pulse_pic(void);
void store_pulse_registers(unsigned long data_buffer, unsigned char start);
 


void write_pic( unsigned char addr, unsigned char value );

unsigned int GET_ACK( void );
bit GIVE_PIC_ACK( void );
void i2c_pic_start();
void i2c_pic_write( unsigned char ch );
unsigned int i2c_pic_read( void );
void i2c_stop(void);
unsigned char  ReadPicVersion(void);
void delay_us(unsigned int);
	
extern void SetBit(unsigned char bit_number,unsigned char *byte);
extern bit GetBit(unsigned char bit_number,unsigned char *byte);

//extern unsigned int analog_buf;
//extern unsigned int output_feedback;


unsigned char xdata input_channel_select = EEP_INPUT1;
//unsigned char input_bank_select = 1;

 
unsigned char pic_checksum;
#ifdef T3_8IN13OUT //MHF 20010_07 COMBINE TWO IFDEFS INTO ONE

extern unsigned char channel_type;
union {
			unsigned long pic_pulse;
			unsigned char pic_buffer[4];
	}number;
typedef union   pulse_number_link {
		unsigned long      number_long[8];
		unsigned int	 half_number[16];
		unsigned char    pulse_number[32];
	};
extern unsigned char xdata pulse_number[40];// 
extern union   pulse_number_link xdata   pic,flash;

typedef union   dual_reading {
		unsigned int      dual_word[8];
		unsigned char    dual_byte[16];
	};
union dual_reading xdata adam;

unsigned int xdata adam_buffer[8];
unsigned int xdata adam_old_reading[8];

unsigned long xdata previous_pulse_number[8];

#endif

unsigned char xdata  reading_filter_bypass;
#ifdef T3_32IN
unsigned int xdata  old_reading[32]; //MHF 20010_07 FILTER TO DETECT PIC GARBAGE READINGS
unsigned char xdata filter[32];
unsigned char xdata gucFilterCounter[32]; //MHF 20010_07 PIC GARBAGE DETECTOR FILTER
#else
unsigned long xdata  old_reading[8];
unsigned char xdata filter[8];
unsigned char xdata gucFilterCounter[8];
unsigned char xdata fliter_slow_high[8],fliter_slow_low[8];
#endif
unsigned int xdata reading_counter;

unsigned char pic_type=0;	// 0 - no PIC
						// 1 - original PIC
						// 2 - PIC with interrupt, and new start condition
						// 3 - PIC with 8 AD



// --- transfer pic readings data to register data --------------------
extern xdata  struct link modbus;
//signed char analog_input_filter[MAX_INPUT_CHANNELS] ; 	// ron noticed must be in data memory or else pic and communication will not function properly
 


// --- timer SOP stuff -------------------------------------------------//
//extern unsigned int xdata error_check[16];


// --- FLASH MANAGEMENT -------------------------------------------------
extern bit flash_write_int(unsigned char id, unsigned int value, unsigned char block_select);


// --- flexdriver feature -----------------------------------------------
#ifdef FLEXDRIVER_FEATURE
	extern void master_com_serial(unsigned char Flex_id, unsigned int FlexDriver_reg_addr, unsigned int FlexDriver_reg_data);
#endif





