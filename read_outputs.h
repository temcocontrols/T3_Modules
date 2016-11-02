/*-----------------read_outputs.h---------------------*/


// --- function prototype ----------------------------
void refresh_outputs (void);

unsigned int look_up_table(unsigned int count );



extern void delay_us(unsigned int time);



// --- calibration ------------------------------------
extern unsigned int rectify_reading(unsigned char IO_port_select, unsigned int pic_reading);
extern bit LUT_present;


// --- variables ----------------------------


unsigned char xdata output_channel = 0; // WHEN I PUT THIS IN XDATA IT SCREWED EVERYTHING UP>>> COULD NOT BE INCREMENTED PROPERLY
unsigned char xdata output_sequence_count	= 0;
unsigned char output_sequence_channel	= 0;
bit incrementing	= 0;
unsigned int xdata testing_increment_init	= 300;


extern xdata struct link modbus;

extern unsigned char xdata output_sequence;		// SOP sequence
extern bit digital_mode;
extern bit Enable_set_output;
extern bit Enable_read_input;
extern unsigned char xdata startup_flag;	// for testing SOP
extern bit reverse_logic_output;
extern unsigned char xdata switch_state[MAX_OUTPUT_CHANNELS];
extern unsigned char xdata output_calibration;
extern bit out3;
unsigned char 				bdata gucPreviousRelayLO = 0xFF;
unsigned char 				bdata gucPreviousRelayHI = 0xFF;
unsigned char 				bdata gucCurrentRelayLO  = 0xFF;
unsigned char 				bdata gucCurrentRelayHI  = 0xFF;
sbit					gbRelay1		= gucPreviousRelayLO^0;
sbit					gbRelay2		= gucPreviousRelayLO^1;
sbit					gbRelay3		= gucPreviousRelayLO^2;
sbit					gbRelay4		= gucPreviousRelayLO^3;
sbit					gbRelay5		= gucPreviousRelayLO^4;
sbit					gbRelay6		= gucPreviousRelayLO^5;
sbit					gbRelay7		= gucPreviousRelayLO^6;
sbit					gbRelay8		= gucPreviousRelayLO^7;
sbit					gbRelay9		= gucPreviousRelayHI^0;
sbit					gbRelay10		= gucPreviousRelayHI^1;
sbit					gbRelay11		= gucPreviousRelayHI^2;
sbit					gbRelay12		= gucPreviousRelayHI^3;
sbit					gbRelay13		= gucPreviousRelayHI^4;

sbit					gbcRelay1		= gucCurrentRelayLO^0;
sbit					gbcRelay2		= gucCurrentRelayLO^1;
sbit					gbcRelay3		= gucCurrentRelayLO^2;
sbit					gbcRelay4		= gucCurrentRelayLO^3;
sbit					gbcRelay5		= gucCurrentRelayLO^4;
sbit					gbcRelay6		= gucCurrentRelayLO^5;
sbit					gbcRelay7		= gucCurrentRelayLO^6;
sbit					gbcRelay8		= gucCurrentRelayLO^7;
sbit					gbcRelay9		= gucCurrentRelayHI^0;
sbit					gbcRelay10		= gucCurrentRelayHI^1;
sbit					gbcRelay11		= gucCurrentRelayHI^2;
sbit					gbcRelay12		= gucCurrentRelayHI^3;
sbit					gbcRelay13		= gucCurrentRelayHI^4;
