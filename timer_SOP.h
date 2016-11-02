// --- timer_SOP.h file --- //
// [note] this file is an extension of serial1.c files, added feature where the device can send a signal 
//			on the serial line as well rather than just responding to messages from the master

#define SEND_POOL_LENGTH         8

// --- function prototype for serial communications ---------------------
extern void initSend_COM(void);
extern void InitCRC16(void);
extern void CRC16_Tstat(unsigned char ch);

extern void serial_restart(void);

// --- variables for serial communications ------------------------------
extern unsigned char xdata data_buffer[DATABUFLEN]; //MHF 20010_07 MOVE SERIAL TO XDATA TO SAVE RAM
extern bit transmit_finished;
extern unsigned char idata CRClo,CRChi;   // CRC low byte ,high byte

// --- function prototypes for IO module --------------------------------
extern void delay_us(unsigned int time);
extern void watchdog();




// --- function prototype for timer SOP mode-----------------------------
void timer_serial_signal(unsigned char start_id, unsigned char end_id, unsigned int tstat_reg_addr, unsigned int tstat_reg_data);

// --- variables for timer SOP mode -------------------------------------
extern bit Serial_Master;	// if set to 1, notify I am master and will want to RECEIVE a response
extern unsigned char xdata send_buffer_verify[SEND_POOL_LENGTH];
extern bit response_receive_finished;
extern unsigned char xdata timer_id;

// error_check used bit wise for every ID (total amount 256)
// note given ID 255 is not used, we use as control bit
// look at error_check[15] the MSB, if 0 thus performing operation for first time
// if MSB is 1, thus did operation before, only need to do on error check failed
//extern unsigned int xdata error_check[16];










