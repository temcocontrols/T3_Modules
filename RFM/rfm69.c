#include "rfm69.h" 
#include "rfm69registers.h"
#include "spi.h"
#include "timerx.h"
#include <stdbool.h>
#include <stdio.h>
#include "modbus.h"

#if 1//T36CTA
#define	SIMULATE_SPI_CS	PDout(2) //RFM69片选引脚	
#define SIMULATE_SPI_CLK    PBout(13)
#define SIMULATE_DELAY_US	delay_us(1)
#define SIMULATE_SPI_MOSI	PBout(15)
#define SIMULATE_MISO    PBin(14)

uint16_t rfm69_deadMaster = RFM69_DEFAULT_DEADMASTER;
bool rfm69_deadmaster_enable = false;
uint16_t rfm69_set_deadMaster = RFM69_DEFAULT_DEADMASTER;
uint8_t rfm69_size;
uint8_t rfm69_id;
bool rfm69_send_flag = false;
uint8_t rfm69_sendBuf[RF69_MAX_DATA_LEN];
static uint8_t data[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
static volatile uint8_t datalen;
static volatile uint8_t senderID;
static volatile uint8_t targetID;                // should match _address
static volatile uint8_t payloadLen;
static volatile uint8_t ACK_Requested;
static volatile uint8_t ACK_RECEIVED;           // should be polled immediately after sending a packet with ACK request
//static volatile uint8_t _mode;
int16_t rssi;                   // most accurate RSSI during reception (closest to the reception)
int16_t rcv_rssi = 0;

static uint8_t _address;
static uint8_t _powerLevel = 31;
bool _promiscuousMode = false;
uint8_t _mode = RF69_MODE_STANDBY;
char rfm69_key[16] = "ABCDEFGHIJKLMNOP";
uint16_t RFM69_networkID;
uint8_t RFM69_nodeID;
uint32_t RFM69_freq;
uint16_t RFM69_biterate;
bool RFM69_enable = true;

extern uint16_t rfm69_count;

extern u8 rfm69_rx_count;
extern u8 rfm69_tx_count;

// used function prototypes

uint8_t RFM69_readReg(uint8_t addr);


void RFM69_setHighPowerRegs(bool onOff);
void RFM69_sleep(void);
void RFM69_setPowerLevel(uint8_t level); // reduce/increase transmit power level
bool RFM69_canSend(void);

bool RFM69_ACKReceived(uint8_t fromNodeID);
bool RFM69_receiveDone(void);
bool RFM69_ACKRequested(void);
void RFM69_sendACK(const void* buffer, uint8_t bufferSize);
void RFM69_receiveBegin(void);
void RFM69_promiscuous(bool onOff);
void RFM69_readAllRegs(void);

void RFM69_rcCalibration(void); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]



void RFM69_setHighPower(bool onOff);

// extern functions
void noInterrupts();                // function to disable interrupts
void interrupts();                  // function to enable interrupts
void RFM69_SetCSPin(bool);          // function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
bool RFM69_ReadDIO0Pin(void);       // function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
uint8_t SPI_transfer8(uint8_t);     // function to transfer 1byte on SPI with readback
//void Serialprint(char*);            // function to print to serial port a string
bool Timeout_IsTimeout1(void);      // function for timeout handling, checks if previously set timeout expired
void Timeout_SetTimeout1(uint16_t); // function for timeout handling, sets a timeout, parameter is in milliseconds (ms)

void simulate_spi_init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOC时钟 
    //spi_clk
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13 | GPIO_Pin_15; 
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;     
    GPIO_Init(GPIOB,&GPIO_InitStructure);   
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15);  

    //spi_miso 
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14; 
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;     
    GPIO_Init(GPIOB,&GPIO_InitStructure);     

//    //spi_mosi
//    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15; 
//    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;  
//    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;    
//    GPIO_Init(GPIOB,&GPIO_InitStructure);     
//	
//	printf("simulate_spi_init..\r\n\r\n");
}


void simulate_spi_write_byte(u8 data)
{
    u8 kk;

    //SIMULATE_SPI_CS = 0;

   // SIMULATE_SPI_CLK = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	SIMULATE_DELAY_US;
    //delay_us(10);     


    for(kk=0;kk<8;kk++)
    {
     
		if((data&0x80)==0x80) 
			GPIO_SetBits(GPIOB, GPIO_Pin_15);//SIMULATE_SPI_MOSI = 1;
		else         
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);//SIMULATE_SPI_MOSI = 0;
		SIMULATE_DELAY_US;      
		//delay_us(10);
		GPIO_SetBits(GPIOB, GPIO_Pin_13);//SIMULATE_SPI_CLK = 1;
		SIMULATE_DELAY_US;
		//delay_us(10);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);//SIMULATE_SPI_CLK = 0; 
		data = data<<1;
    }

    //SIMULATE_SPI_CS = 1;
}

u8 simulate_spi_read_byte(void)
{
    u8 kk=0, ret=0;

    //SIMULATE_SPI_CS = 0;

    //SIMULATE_SPI_CLK = 0;
//	simulate_spi_write_byte(0);
//	SIMULATE_DELAY_US;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    SIMULATE_DELAY_US;
	//delay_us(10);  

    
    for(kk=0;kk<8;kk++)
    {
		ret = ret<<1; 
		GPIO_SetBits(GPIOB, GPIO_Pin_13);//SIMULATE_SPI_CLK = 1; 
		//if(SIMULATE_MISO) 
		if(GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_14))
			ret |= 0x01;
		SIMULATE_DELAY_US;
		//delay_us(10);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);//SIMULATE_SPI_CLK = 0;
		SIMULATE_DELAY_US; 
		//delay_us(10);
    }

    //SIMULATE_SPI_CS = 1;

    return ret;
}

// internal
static void RFM69_sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK, bool sendACK);




void RFM69_TIMER_init(void)
{
	//TIM3_Int_Init(100, 719);
}



void RFM69_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//	GPIO_InitTypeDef GPIO_InitStructure;
	//SPI_Cmd(SPI2, DISABLE); 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOC时钟 
    //spi_clk
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13 | GPIO_Pin_15; 
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;     
    GPIO_Init(GPIOB,&GPIO_InitStructure);   
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15);  

    //spi_miso 
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14; 
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;     
    GPIO_Init(GPIOB,&GPIO_InitStructure);     

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	 //使能PB端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 //PD2推挽 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOD,GPIO_Pin_2);						 //PD2上拉
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//使能GPIOC时钟 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //SET PC11 AS INPUT PULL UP
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOC,GPIO_Pin_11);						 //PC11上拉
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//使能GPIOC时钟 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //SET PC11 AS INPUT PULL UP
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_12);						 //PC12上拉

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11 ); 
	//EXTI_InitStructure.EXTI_Line = (uint16_t)1<<GPIO_Pin ;
	EXTI_InitStructure.EXTI_Line  = EXTI_Line11 ; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	RFM_CS = 1;
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & (1 << 11))	//是11线的中断
	{      
		EXTI->PR  = (1 << 11);	//清除LINE11上的中断标志位
//		printf( "EXTI15_10_IRQHandler..\r\n");
		RFM69_interruptHandler();
	}

}

bool RFM69_initialize(uint8_t freqBand, uint8_t nodeID, uint16_t networkID)
{
	
  uint8_t i;
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_9600}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_9600},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

//    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
//    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
//    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

	/* 0x07 */ { REG_FRFMSB, RF_FRFMSB_915 },
    /* 0x08 */ { REG_FRFMID, RF_FRFMID_915 },
    /* 0x09 */ { REG_FRFLSB, RF_FRFLSB_915 },
    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
//    /* 0x2F */ { REG_SYNCVALUE1, (uint8_t)(networkID & 0x00FF) },  // NETWORK ID lower 8 bits
//    /* 0x30 */ { REG_SYNCVALUE2, (uint8_t)(networkID >> 8) },      // NETWORK ID higher 8 bits
    /* 0x2F */ { REG_SYNCVALUE1, 0xaa},  // NETWORK ID lower 8 bits
    /* 0x30 */ { REG_SYNCVALUE2, 0x55 },      // NETWORK ID higher 8 bits
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };
  
	RFM69_SetCSPin(HIGH);
	//delay_ms(1000);
  
  while(0)
  {
	  RFM69_writeReg(REG_SYNCVALUE1, 0xAA); 
	  //RFM69_readReg(REG_SYNCVALUE1);
	  delay_ms(100);
	  if( 0xaa == RFM69_readReg(REG_SYNCVALUE1))
		  printf("0xaa == RFM69_readReg(REG_SYNCVALUE1)\r\n");
	  else
		  printf( "!= != != \r\n\r\n");
  }
  Timeout_SetTimeout1(50);
  do
  {
    RFM69_writeReg(REG_SYNCVALUE1, 0xAA); 
//	  GPIO_ResetBits(GPIOB, GPIO_Pin_13);
//	  delay_ms(20);
//	  GPIO_SetBits(GPIOB, GPIO_Pin_13);
//	  delay_ms(20);
//	  printf( "RFM69_writeReg(REG_SYNCVALUE1, 0xAA);\r\n\r\n\r\n");
  }while (RFM69_readReg(REG_SYNCVALUE1) != 0xaa);
  //while (RFM69_readReg(REG_SYNCVALUE1) != 0xaa && !Timeout_IsTimeout1());
  
  Timeout_SetTimeout1(50);
  do
  {
    RFM69_writeReg(REG_SYNCVALUE1, 0x55); 
  }while (RFM69_readReg(REG_SYNCVALUE1) != 0x55);
  //while (RFM69_readReg(REG_SYNCVALUE1) != 0x55 && !Timeout_IsTimeout1());

  for (i = 0; CONFIG[i][0] != 255; i++)
  {
    RFM69_writeReg(CONFIG[i][0], CONFIG[i][1]);
  }
  RFM69_setNetwork(RFM69_networkID);
  RFM69_setFrequency(RFM69_freq);
  _address = nodeID;
  RFM69_setAddress(nodeID);
  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  RFM69_encrypt(0);

  RFM69_setHighPower(ISRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  RFM69_setMode(RF69_MODE_STANDBY);
  Timeout_SetTimeout1(50);
  //RFM69_promiscuous(true);
  while (((RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && !Timeout_IsTimeout1()); // wait for ModeReady
  if (Timeout_IsTimeout1())
  {
    return false;
  }
  return true;
}

// return the frequency (in Hz)
uint32_t RFM69_getFrequency()
{
  return RF69_FSTEP * (((uint32_t) RFM69_readReg(REG_FRFMSB) << 16) + ((uint16_t) RFM69_readReg(REG_FRFMID) << 8) + RFM69_readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void RFM69_setFrequency(uint32_t freqHz)
{
  uint8_t oldMode = _mode;
  if (oldMode == RF69_MODE_TX) {
    RFM69_setMode(RF69_MODE_RX);
  }
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  RFM69_writeReg(REG_FRFMSB, freqHz >> 16);
  RFM69_writeReg(REG_FRFMID, freqHz >> 8);
  RFM69_writeReg(REG_FRFLSB, freqHz);
  if (oldMode == RF69_MODE_RX) {
    RFM69_setMode(RF69_MODE_SYNTH);
  }
  RFM69_setMode(oldMode);
}

uint16_t RFM69_getBitRate(void)
{
	return ((uint16_t) RFM69_readReg(REG_BITRATEMSB) << 8) + RFM69_readReg(REG_BITRATELSB);
}

void RFM69_setBitRate(uint16_t bitRate)
{
  RFM69_writeReg(REG_BITRATEMSB, bitRate >> 8);
  RFM69_writeReg(REG_BITRATELSB, bitRate);
}

void RFM69_setMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (ISRFM69HW) RFM69_setHighPowerRegs(true);
      break;
    case RF69_MODE_RX:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (ISRFM69HW) RFM69_setHighPowerRegs(false);
      break;
    case RF69_MODE_SYNTH:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)
	  printf("RFM69_setMode\r\n"); // wait for ModeReady
  //delay_ms(50);

  _mode = newMode;
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call RFM69_receiveDone()
void RFM69_sleep() {
  RFM69_setMode(RF69_MODE_SLEEP);
}

uint8_t RFM69_getAddress()
{
  return RFM69_readReg(REG_NODEADRS);
}

//set this node's address
void RFM69_setAddress(uint8_t addr)
{
  _address = addr;
  RFM69_writeReg(REG_NODEADRS, _address);
}

uint16_t RFM69_getNetwork()
{
  return ((uint16_t) RFM69_readReg(REG_SYNCVALUE2) << 8) + RFM69_readReg(REG_SYNCVALUE1);
}


//set this node's network id
void RFM69_setNetwork(uint16_t networkID)
{
  RFM69_writeReg(REG_SYNCVALUE1, (uint8_t)(networkID & 0x00FF));
  RFM69_writeReg(REG_SYNCVALUE2, (uint8_t)(networkID >> 8));
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void RFM69_setPowerLevel(uint8_t powerLevel)
{
  _powerLevel = (powerLevel > 31 ? 31 : powerLevel);
  if (ISRFM69HW)
  {
    _powerLevel /= 2;
  }
  RFM69_writeReg(REG_PALEVEL, (RFM69_readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
}

bool RFM69_canSend()
{
//	printf( "mode = %d, payloadLen = %d, rssi = %d\r\n", _mode,payloadLen,RFM69_readRSSI(0));
  if (_mode == RF69_MODE_RX && payloadLen == 0 && RFM69_readRSSI(0) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    RFM69_setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void RFM69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK)
{
	rfm69_tx_count = 7;
//	printf("go into RFM69_send\r\n");
  RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
//	printf("after RFM69_writeReg\r\n");
  //uint32_t now = millis();
	Timeout_SetTimeout1(50);
  while (!RFM69_canSend() && !Timeout_IsTimeout1())/*&& millis() - now < RF69_CSMA_LIMIT_MS*/ RFM69_receiveDone();
//	printf("after RFM69_canSend\r\n");
  RFM69_sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

// to increase the chance of getting a packet across, call this function instead of send
// and it handles all the ACK requesting/retrying for you :)
// The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
// The reason for the semi-automaton is that the lib is interrupt driven and
// requires user action to read the received data and decide what to do with it
// replies usually take only 5..8ms at 50kbps@915MHz
bool RFM69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) 
{
	uint8_t i;
	
  for ( i = 0; i <= retries; i++)
  {
//	  printf("go into RFM69_sendWithRetry\r\n");
    RFM69_send(toAddress, buffer, bufferSize, false);
    Timeout_SetTimeout1(retryWaitTime);
    while (!Timeout_IsTimeout1())
    {
      if (RFM69_ACKReceived(toAddress))
      {
        //Serial.print(" ~ms:"); Serial.print(millis() - sentTime);
//		  printf("RFM69_ACKReceived and still have %d ms\r\n",rfm69_count);
		  delay_ms(5);
        return true;
      }
    }
    //Serial.print(" RETRY#"); Serial.println(i + 1);
//	printf("RETRY, time %d\r\n", i+1);
	delay_ms(3);
	RFM69_setMode(RF69_MODE_RX);
//	delay_ms(20);
  }
  return false;
}

// should be polled immediately after sending a packet with ACK request
bool RFM69_ACKReceived(uint8_t fromNodeID) 
{
  if (RFM69_receiveDone())
    return (senderID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
  return false;
}

// check whether an ACK was requested in the last received packet (non-broadcasted packet)
bool RFM69_ACKRequested() 
{
  return ACK_Requested && (targetID != RF69_BROADCAST_ADDR);
}

// should be called immediately after reception in case sender wants ACK
void RFM69_sendACK(const void* buffer, uint8_t bufferSize) 
{
	
  uint8_t sender ;
  int16_t l_rssi; // save payload received RSSI value
  ACK_Requested = 0;   // TWS added to make sure we don't end up in a timing race and infinite loop sending Acks
  sender = senderID;	
  l_rssi = rssi;	

  RFM69_sendFrame(sender, buffer, bufferSize, false, true);
  rssi = l_rssi; // restore payload RSSI
}

// internal function
static void RFM69_sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
	  // control byte
  uint8_t CTLbyte = 0x00;
	uint8_t i;
//	uint32_t freq;
//	printf("go into RFM69_sendFrame\r\n");
  RFM69_setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
	Timeout_SetTimeout1(RF69_TX_LIMIT_MS);
  while (((RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)&& !Timeout_IsTimeout1()); // wait for ModeReady
//	printf("after RFM69_readReg(REG_IRQFLAGS1)\r\n");
  RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;


  if (sendACK)
    CTLbyte = RFM69_CTL_SENDACK;
  else if (requestACK)
    CTLbyte = RFM69_CTL_REQACK;

  // write to FIFO
  RFM69_select();
  SPI_transfer8(REG_FIFO | 0x80);
  SPI_transfer8(bufferSize + 3);
  SPI_transfer8(toAddress);
  SPI_transfer8(_address);
  SPI_transfer8(CTLbyte);

  for (i = 0; i < bufferSize; i++)
    SPI_transfer8(((uint8_t*) buffer)[i]);
  RFM69_unselect();

 // freq = RFM69_getFrequency();
//  printf( "Freq is %ud, in RFM69_sendFrame\r\n", freq);
  // no need to wait for transmit mode to be ready since its handled by the radio
  RFM69_setMode(RF69_MODE_TX);
  //freq = RFM69_getFrequency();
//  printf( "Freq is %ud, in RFM69_sendFrame after set mode\r\n", freq);
  
  Timeout_SetTimeout1(RF69_TX_LIMIT_MS);
  while (RFM69_ReadDIO0Pin() == 0 && !Timeout_IsTimeout1()); // wait for DIO0 to turn HIGH signalling transmission finish
  //while (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
  RFM69_setMode(RF69_MODE_STANDBY);
}

//=============================================================================
// interruptHook() - gets called by the base class interrupt handler right after the header is fetched.
//=============================================================================
void interruptHook(uint8_t CTLbyte) {
//  ACK_RSSI_REQUESTED = CTLbyte & RFM69_CTL_RESERVE1; // TomWS1: extract the ACK RSSI request bit (could potentially merge with ACK_REQUESTED)
  // TomWS1: now see if this was an ACK with an ACK_RSSI response
	
	
  //printf("send ACK:%d, \r\n\r\n",ACK_Requested);
  if (ACK_Requested) //&& ACK_RSSI_REQUESTED) {
  {
	  RFM69_sendACK((void*)&rssi, sizeof(rssi));

  }
}

// internal function - interrupt gets called when a packet is received
void RFM69_interruptHandler() {


  if (_mode == RF69_MODE_RX && (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    uint8_t CTLbyte;
	  uint8_t i;
    //rssi = RFM69_readRSSI();
	  rfm69_rx_count = 7;
	 // printf("RFM69_interruptHandler\n\r");
    RFM69_setMode(RF69_MODE_STANDBY);
    RFM69_select();
    SPI_transfer8(REG_FIFO & 0x7F);
	  
    payloadLen = simulate_spi_read_byte();//SPI_transfer8(0);
    payloadLen = payloadLen > 66 ? 66 : payloadLen; // precaution
    targetID = simulate_spi_read_byte();//SPI_transfer8(0);
//	printf("payloadLen= %d\r\n, targetID= %d", payloadLen, targetID);  
    if(!(_promiscuousMode || targetID == _address || targetID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in promiscuous mode
       || payloadLen < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
    {
      payloadLen = 0;
      RFM69_unselect();
      RFM69_receiveBegin();
      return;
    }

    datalen = payloadLen - 3;
    senderID = simulate_spi_read_byte();//SPI_transfer8(0);
    CTLbyte = simulate_spi_read_byte();//SPI_transfer8(0);

    ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
    ACK_Requested = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag
    

    for (i = 0; i < datalen; i++)
    {
      data[i] = simulate_spi_read_byte();//SPI_transfer8(0);
	  //rfm69_sendBuf[i] = data[i];	
    }
	rfm69_size = datalen;
	rfm69_id = senderID;
//	init_crc16(); 
	memcpy(rfm69_sendBuf, data, datalen);
//	printf("payloadLen= %d\r\n, senderID= %d\r\n, datalen = %d\r\n\r\n", payloadLen, senderID, datalen);  
//	printf("%d,%d,%d,%d,%d,%d,%d,%d,\r\n\r\n", data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
    interruptHook(CTLbyte);     // TWS: hook to derived class interrupt function
	rfm69_send_flag = true;
	//RFM69_sendWithRetry(senderID, data, 8, 0, 25);
//	if(data[0] == 255 || data[0] == modbus.address || data[0] == 0)
//		responseCmd(10, data);
//	memcpy( uart_send, data, rfm69_size);
//	TXEN = SEND;
//	USART_SendDataString(rfm69_size);
//    if (datalen < RF69_MAX_DATA_LEN) data[datalen] = 0; // add null at end of string
	//SerialPrint((char*) data);
	//printf("senderID= %d\r\n, targetID= %d", senderID, targetID);
	//printf(data);
    RFM69_unselect();
    RFM69_setMode(RF69_MODE_RX);
  }
  rssi = RFM69_readRSSI(0);
  rcv_rssi = rssi;
}


// internal function
void RFM69_receiveBegin() 
{
  datalen = 0;
  senderID = 0;
  targetID = 0;
  payloadLen = 0;
  ACK_Requested = 0;
  ACK_RECEIVED = 0;
  rssi = 0;
  if (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  RFM69_setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool RFM69_receiveDone() 
{
//ATOMIC_BLOCK(ATOMIC_FORCEON)
  noInterrupts(); // re-enabled in RFM69_unselect() via setMode() or via RFM69_receiveBegin()
  if (_mode == RF69_MODE_RX && payloadLen > 0)
  {
    RFM69_setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    interrupts(); // explicitly re-enable interrupts
    return false;
  }
  RFM69_receiveBegin();
  return false;
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void RFM69_encrypt(const char* key) 
{
	uint8_t i;
  RFM69_setMode(RF69_MODE_STANDBY);
  if (key != 0)
  {
    RFM69_select();
    SPI_transfer8(REG_AESKEY1 | 0x80);
    for (i = 0; i < 16; i++)
      SPI_transfer8(key[i]);
    RFM69_unselect();
  }
  RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
}

// get the received signal strength indicator (RSSI)
int16_t RFM69_readRSSI(bool forceTrigger) 
{
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    RFM69_writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((RFM69_readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00)
		printf("RFM69_readRSSI\r\n");
		//printf("while ((RFM69_readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00)\r\n"); // wait for RSSI_Ready
  }
  rssi = -RFM69_readReg(REG_RSSIVALUE);
  rssi >>= 1;
  //if(rssi != 0)
	return rssi;
}

// true  = disable filtering to capture all frames on network
// false = enable node/broadcast filtering to capture only frames sent to this/broadcast address
void RFM69_promiscuous(bool onOff) 
{
  _promiscuousMode = onOff;
  RFM69_writeReg(REG_PACKETCONFIG1, (RFM69_readReg(REG_PACKETCONFIG1) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

// for RFM69HW only: you must call RFM69_setHighPower(true) after initialize() or else transmission won't work
void RFM69_setHighPower(bool onOff) {
  RFM69_writeReg(REG_OCP, ISRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (ISRFM69HW) // turning ON
    RFM69_writeReg(REG_PALEVEL, (RFM69_readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    RFM69_writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

// internal function
void RFM69_setHighPowerRegs(bool onOff) 
{
  RFM69_writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  RFM69_writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

//for debugging

void RFM69_readAllRegs()
{
  uint8_t regVal;

#if REGISTER_DETAIL 
  int capVal;

  //... State Variables for intelligent decoding
  uint8_t modeFSK = 0;
  int bitRate = 0;
  int freqDev = 0;
  long freqCenter = 0;
#endif
  char pcBuf[16];
  
//  Serial.println("Address - HEX");
	uint8_t regAddr;
  for (regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    RFM69_select();
    SPI_transfer8(regAddr & 0x7F); // send address + r/w bit
    regVal = SPI_transfer8(0);
    RFM69_unselect();

//    usprintf(pcBuf, "%0X",regAddr);
    SerialPrint(pcBuf);  //hex
    SerialPrint(" - ");
//    usprintf(pcBuf, "%02X",regVal);
    SerialPrint(pcBuf);   //hex

#if 0//REGISTER_DETAIL 
    switch ( regAddr ) 
    {
        case 0x1 : {
            SerialPrint ( "Controls the automatic Sequencer ( see section 4.2 )\nSequencerOff : " );
            if ( 0x80 & regVal ) {
                SerialPrint ( "1 -> Mode is forced by the user\n" );
            } else {
                SerialPrint ( "0 -> Operating mode as selected with Mode bits in RegOpMode is automatically reached with the Sequencer\n" );
            }
            
            SerialPrint( "\nEnables Listen mode, should be enabled whilst in Standby mode:\nListenOn : " );
            if ( 0x40 & regVal ) {
                SerialPrint ( "1 -> On\n" );
            } else {
                SerialPrint ( "0 -> Off ( see section 4.3)\n" );
            }
            
            SerialPrint( "\nAborts Listen mode when set together with ListenOn=0 See section 4.3.4 for details (Always reads 0.)\n" );
            if ( 0x20 & regVal ) {
                SerialPrint ( "ERROR - ListenAbort should NEVER return 1 this is a write only register\n" );
            }
            
            SerialPrint("\nTransceiver's operating modes:\nMode : ");
            capVal = (regVal >> 2) & 0x7;
            if ( capVal == 0b000 ) {
                SerialPrint ( "000 -> Sleep mode (SLEEP)\n" );
            } else if ( capVal == 0b001 ) {
                SerialPrint ( "001 -> Standby mode (STDBY)\n" );
            } else if ( capVal == 0b010 ) {
                SerialPrint ( "010 -> Frequency Synthesizer mode (FS)\n" );
            } else if ( capVal == 0b011 ) {
                SerialPrint ( "011 -> Transmitter mode (TX)\n" );
            } else if ( capVal == 0b100 ) {
                SerialPrint ( "100 -> Receiver Mode (RX)\n" );
            } else {
                //Serial.print( capVal, BIN );
                SerialPrint ( " -> RESERVED\n" );
            }
            SerialPrint ( "\n" );
            break;
        }
        
        case 0x2 : {
        
            SerialPrint("Data Processing mode:\nDataMode : ");
            capVal = (regVal >> 5) & 0x3;
            if ( capVal == 0b00 ) {
                SerialPrint ( "00 -> Packet mode\n" );
            } else if ( capVal == 0b01 ) {
                SerialPrint ( "01 -> reserved\n" );
            } else if ( capVal == 0b10 ) {
                SerialPrint ( "10 -> Continuous mode with bit synchronizer\n" );
            } else if ( capVal == 0b11 ) {
                SerialPrint ( "11 -> Continuous mode without bit synchronizer\n" );
            }
            
            SerialPrint("\nModulation scheme:\nModulation Type : ");
            capVal = (regVal >> 3) & 0x3;
            if ( capVal == 0b00 ) {
                SerialPrint ( "00 -> FSK\n" );
                modeFSK = 1;
            } else if ( capVal == 0b01 ) {
                SerialPrint ( "01 -> OOK\n" );
            } else if ( capVal == 0b10 ) {
                SerialPrint ( "10 -> reserved\n" );
            } else if ( capVal == 0b11 ) {
                SerialPrint ( "11 -> reserved\n" );
            }
            
            SerialPrint("\nData shaping: ");
            if ( modeFSK ) {
                SerialPrint( "in FSK:\n" );
            } else {
                SerialPrint( "in OOK:\n" );
            }
            SerialPrint ("ModulationShaping : ");
            capVal = regVal & 0x3;
            if ( modeFSK ) {
                if ( capVal == 0b00 ) {
                    SerialPrint ( "00 -> no shaping\n" );
                } else if ( capVal == 0b01 ) {
                    SerialPrint ( "01 -> Gaussian filter, BT = 1.0\n" );
                } else if ( capVal == 0b10 ) {
                    SerialPrint ( "10 -> Gaussian filter, BT = 0.5\n" );
                } else if ( capVal == 0b11 ) {
                    SerialPrint ( "11 -> Gaussian filter, BT = 0.3\n" );
                }
            } else {
                if ( capVal == 0b00 ) {
                    SerialPrint ( "00 -> no shaping\n" );
                } else if ( capVal == 0b01 ) {
                    SerialPrint ( "01 -> filtering with f(cutoff) = BR\n" );
                } else if ( capVal == 0b10 ) {
                    SerialPrint ( "10 -> filtering with f(cutoff) = 2*BR\n" );
                } else if ( capVal == 0b11 ) {
                    SerialPrint ( "ERROR - 11 is reserved\n" );
                }
            }
            
            SerialPrint ( "\n" );
            break;
        }
        
        case 0x3 : {
            bitRate = (regVal << 8);
            break;
        }
        
        case 0x4 : {
            bitRate |= regVal;
            SerialPrint ( "Bit Rate (Chip Rate when Manchester encoding is enabled)\nBitRate : ");
            unsigned long val = 32UL * 1000UL * 1000UL / bitRate;
            //Serial.println( val );
            SerialPrint( "\n" );
            break;
        }
        
        case 0x5 : {
            freqDev = ( (regVal & 0x3f) << 8 );
            break;
        }
        
        case 0x6 : {
            freqDev |= regVal;
            SerialPrint( "Frequency deviation\nFdev : " );
            unsigned long val = 61UL * freqDev;
            //Serial.println( val );
            SerialPrint ( "\n" );
            break;
        }
        
        case 0x7 : {
            unsigned long tempVal = regVal;
            freqCenter = ( tempVal << 16 );
            break;
        }
       
        case 0x8 : {
            unsigned long tempVal = regVal;
            freqCenter = freqCenter | ( tempVal << 8 );
            break;
        }

        case 0x9 : {        
            freqCenter = freqCenter | regVal;
            SerialPrint ( "RF Carrier frequency\nFRF : " );
            unsigned long val = 61UL * freqCenter;
            //Serial.println( val );
            SerialPrint( "\n" );
            break;
        }

        case 0xa : {
            SerialPrint ( "RC calibration control & status\nRcCalDone : " );
            if ( 0x40 & regVal ) {
                SerialPrint ( "1 -> RC calibration is over\n" );
            } else {
                SerialPrint ( "0 -> RC calibration is in progress\n" );
            }
        
            SerialPrint ( "\n" );
            break;
        }

        case 0xb : {
            SerialPrint ( "Improved AFC routine for signals with modulation index lower than 2.  Refer to section 3.4.16 for details\nAfcLowBetaOn : " );
            if ( 0x20 & regVal ) {
                SerialPrint ( "1 -> Improved AFC routine\n" );
            } else {
                SerialPrint ( "0 -> Standard AFC routine\n" );
            }
            SerialPrint ( "\n" );
            break;
        }
        
        case 0xc : {
            SerialPrint ( "Reserved\n\n" );
            break;
        }

        case 0xd : {
            u8 val;
            SerialPrint ( "Resolution of Listen mode Idle time (calibrated RC osc):\nListenResolIdle : " );
            val = regVal >> 6;
            if ( val == 0b00 ) {
                SerialPrint ( "00 -> reserved\n" );
            } else if ( val == 0b01 ) {
                SerialPrint ( "01 -> 64 us\n" );
            } else if ( val == 0b10 ) {
                SerialPrint ( "10 -> 4.1 ms\n" );
            } else if ( val == 0b11 ) {
                SerialPrint ( "11 -> 262 ms\n" );
            }
            
            SerialPrint ( "\nResolution of Listen mode Rx time (calibrated RC osc):\nListenResolRx : " );
            val = (regVal >> 4) & 0x3;
            if ( val == 0b00 ) {
                SerialPrint ( "00 -> reserved\n" );
            } else if ( val == 0b01 ) {
                SerialPrint ( "01 -> 64 us\n" );
            } else if ( val == 0b10 ) {
                SerialPrint ( "10 -> 4.1 ms\n" );
            } else if ( val == 0b11 ) {
                SerialPrint ( "11 -> 262 ms\n" );
            }

            SerialPrint ( "\nCriteria for packet acceptance in Listen mode:\nListenCriteria : " );
            if ( 0x8 & regVal ) {
                SerialPrint ( "1 -> signal strength is above RssiThreshold and SyncAddress matched\n" );
            } else {
                SerialPrint ( "0 -> signal strength is above RssiThreshold\n" );
            }
            
            SerialPrint ( "\nAction taken after acceptance of a packet in Listen mode:\nListenEnd : " );
            val = (regVal >> 1 ) & 0x3;
            if ( val == 0b00 ) {
                SerialPrint ( "00 -> chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3)\n" );
            } else if ( val == 0b01 ) {
                SerialPrint ( "01 -> chip stays in Rx mode until PayloadReady or Timeout interrupt occurs.  It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3)\n" );
            } else if ( val == 0b10 ) {
                SerialPrint ( "10 -> chip stays in Rx mode until PayloadReady or Timeout occurs.  Listen mode then resumes in Idle state.  FIFO content is lost at next Rx wakeup.\n" );
            } else if ( val == 0b11 ) {
                SerialPrint ( "11 -> Reserved\n" );
            }
            
            
            SerialPrint ( "\n" );
            break;
        }
        
        default : {
        }
    }
#endif
  }
  RFM69_unselect();
}

uint8_t RFM69_readTemperature(uint8_t calFactor) // returns centigrade
{
  RFM69_setMode(RF69_MODE_STANDBY);
  RFM69_writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((RFM69_readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
	
//	RFM69_setMode(RF69_MODE_RX);
  return ~RFM69_readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void RFM69_rcCalibration()
{
  RFM69_writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((RFM69_readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

uint8_t RFM69_readReg(uint8_t addr)
{
  uint8_t regval;
  RFM69_select();
  SPI_transfer8(addr & 0x7F);
  regval = simulate_spi_read_byte();//SPI_transfer8(0);
  RFM69_unselect();
  return regval;
}

void RFM69_writeReg(uint8_t addr, uint8_t value)
{
  RFM69_select();
  SPI_transfer8(addr | 0x80);
  SPI_transfer8(value);
  RFM69_unselect();
}

// select the RFM69 transceiver (save SPI settings, set CS low)
void RFM69_select() 
{
  noInterrupts();
  RFM69_SetCSPin(LOW);
}

// unselect the RFM69 transceiver (set CS high, restore SPI settings)
void RFM69_unselect() 
{
  RFM69_SetCSPin(HIGH);
  interrupts();
}

void SerialPrint(char* string)
{
	printf("%s",string);
}


void noInterrupts()               // function to disable interrupts
{
//	__disable_irq(); 
}


void interrupts()                  // function to enable interrupts
{
//	__enable_irq();
}


void RFM69_SetCSPin(bool onOff)          // function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
{
	if(onOff == HIGH)
		RFM_CS = 1;
	else
		RFM_CS = 0;
}

bool RFM69_ReadDIO0Pin(void)       // function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
{
	//return true;
	return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11);
}

uint8_t SPI_transfer8(uint8_t onByte)    // function to transfer 1byte on SPI with readback
{
//	return SPI2_ReadWriteByte(onByte);
	simulate_spi_write_byte(onByte);
	return 0;
}




bool Timeout_IsTimeout1(void)      // function for timeout handling, checks if previously set timeout expired
{
	if(rfm69_count==0)
		return true;
	else
		return false;
}

void Timeout_SetTimeout1(uint16_t timeout)
{
	rfm69_count = timeout;
}

#endif