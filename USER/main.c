#include <string.h>
#include "stm32f10x.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
//#include "key.h"
#include "24cxx.h"
#include "spi.h"
#include "lcd.h"
#include "touch.h"
#include "flash.h"
#include "stmflash.h"
//#include "sdcard.h"
#include "mmc_sd.h"
#include "dma.h"
#include "vmalloc.h"
#include "ff.h"
#include "fattester.h"
#include "exfuns.h"
#include "enc28j60.h"
#include "timerx.h"
#include "uip.h"
#include "uip_arp.h"
#include "tapdev.h"
#include "usb_app.h"
//#include "ai.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "modbus.h"
#include "define.h"
#include "inputs.h"
#include "../output/output.h"
#include "dlmstp.h"
#include "rs485.h"
#include "datalink.h"
#include "config.h"
//#include "handlers.h"
#include "device.h"	
#include "registerlist.h"
#include "../filter/filter.h"
#include "../KEY/key.h"
#include "bacnet.h"
#include "t3-pt12.h" 
#include "read_pt.h"
#include "pt12_i2c.h"
#include "store.h"
#include "watchdog.h"
#include "rfm69.h"
#include "accelero_meter.h"

#if (defined T38AI8AO6DO)||(defined T322AI)||(defined T36CTA)
void vSTORE_EEPTask(void *pvParameters ) ;
static void vINPUTSTask( void *pvParameters );
#endif
static void vLED0Task( void *pvParameters );
static void vCOMMTask( void *pvParameters );
//static void vUSBTask( void *pvParameters );

static void vNETTask( void *pvParameters );
#ifdef T38AI8AO6DO
void vKEYTask( void *pvParameters );
void vOUTPUTSTask( void *pvParameters );
#endif
#ifdef T36CTA
void vKEYTask( void *pvParameters );
void vOUTPUTSTask( void *pvParameters );
void vAcceleroTask( void *pvParameters);
void vRFMTask(void *pvParameters);
void vAirFlowTask( void *pvParameters);
#endif
#ifdef T3PT12
void vI2C_READ(void *pvParameters) ;
#endif
static void vMSTP_TASK(void *pvParameters ) ;
void uip_polling(void);

#define	BUF	((struct uip_eth_hdr *)&uip_buf[0])	
	
u8 update = 0 ;

u32 Instance = 0x0c;
uint8_t  PDUBuffer[MAX_APDU];

u8 global_key = KEY_NON;

static void debug_config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
}

int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x8008000);
	debug_config();
	//ram_test = 0 ;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
 	delay_init(72);	
//	uart1_init(38400);
//	modbus.baudrate = 38400 ;
	//KEY_Init();
	beeper_gpio_init();
	beeper_on();
	beeper_off();	
	EEP_Dat_Init();
	mass_flash_init() ;
	//Lcd_Initial();
	SPI1_Init();
	SPI2_Init();
	watchdog_init();
//	mem_init(SRAMIN);
//	TIM3_Int_Init(5000, 7199);
//	TIM6_Int_Init(100, 7199);
//	TIM3_Int_Init(50, 7199);//5ms
	printf("T3_series\n\r");
	#if (defined T38AI8AO6DO)||(defined T322AI) ||(defined T36CTA)
		xTaskCreate( vINPUTSTask, ( signed portCHAR * ) "INPUTS", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL );
		xTaskCreate( vSTORE_EEPTask, ( signed portCHAR * ) "STOREEEP", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
	#endif
	xTaskCreate( vLED0Task, ( signed portCHAR * ) "LED0", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
 	xTaskCreate( vCOMMTask, ( signed portCHAR * ) "COMM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL );
	xTaskCreate( vNETTask, ( signed portCHAR * ) "NET",  configMINIMAL_STACK_SIZE+256, NULL, tskIDLE_PRIORITY + 1, NULL );
//	xTaskCreate( vUSBTask, ( signed portCHAR * ) "USB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
	//xTaskCreate( vUSBTask, ( signed portCHAR * ) "USB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
	/* Start the scheduler. */
 	#if (defined T38AI8AO6DO) || (defined T36CTA)
	xTaskCreate( vOUTPUTSTask, ( signed portCHAR * ) "OUTPUTS", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
	xTaskCreate( vKEYTask, ( signed portCHAR * ) "KEY", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
	#endif
	#if defined T36CTA
	xTaskCreate( vRFMTask, ( signed portCHAR * ) "RFM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
	xTaskCreate( vAcceleroTask, ( signed portCHAR * ) "ACCELERO", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
		#if T36CTA_REV2
			xTaskCreate( vAirFlowTask, ( signed portCHAR * ) "AIRFLOW", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
		#endif
	#endif
	#ifdef T3PT12
	xTaskCreate( vI2C_READ, ( signed portCHAR * ) "READ_I2C", configMINIMAL_STACK_SIZE+256, NULL, tskIDLE_PRIORITY + 2, NULL );
	#endif
	xTaskCreate( vMSTP_TASK, ( signed portCHAR * ) "MSTP", configMINIMAL_STACK_SIZE + 256  , NULL, tskIDLE_PRIORITY + 4, NULL );
	vTaskStartScheduler();
}
#if defined T36CTA
void vAirFlowTask( void *pvParameters)
{
	for(;;)
	{
		air_flow_ad = ADC_getChannal(ADC2,ADC_Channel_12);
		AD_Value[8] = air_flow_ad;
		delay_ms(10);
	}
}

extern u8 rfm69_tx_count;
extern bool rfm69_send_flag;
extern u8 rfm69_length;
extern u8 RFM69_SEND[];
bool rfm_exsit = false;
void vRFMTask( void *pvParameters)
{
	u8 temp;
	
	RFM69_GPIO_init();
	RFM69_TIMER_init();

	rfm_exsit = RFM69_initialize(0, RFM69_nodeID, 0);
	RFM69_encrypt(rfm69_key);

	RFM69_setMode(RF69_MODE_RX);
	for( ;; )
	{
		if(RFM69_getFrequency() == 0)
		{
			rfm_exsit = RFM69_initialize(0, RFM69_nodeID, 0);
		}
		delay_ms(300);

		RFM69_setMode(RF69_MODE_RX);
		if(rfm69_send_flag)
		{
			if(rfm69_sendBuf[0] == 255 || rfm69_sendBuf[0] == modbus.address || rfm69_sendBuf[0] == 0)
			{
				responseCmd(10, rfm69_sendBuf);
				internalDeal(10, USART_RX_BUF);
				RFM69_sendWithRetry(rfm69_id, RFM69_SEND, rfm69_length, 0, 1);
			}
			rfm69_send_flag = false;
		}
		RFM69_setMode(RF69_MODE_RX);

	}
}

void vAcceleroTask(void *pvParameters)
{
	ACCELERO_IO_Init();
	ACCELERO_I2C_init();
	/* Write CTL REG1 register, set ACTIVE mode */
    ACCELERO_Write_Data(0x2a, 0x01);
	for( ;; )
	{
		axis_value[0]=ACCELERO_Read_Data(0x2a);
		delay_ms(10);
	}
}
#endif
void vLED0Task( void *pvParameters )
{
	static u8 table_led_count =0 ;
	
	LED_Init();
	for( ;; )
	{
		table_led_count++ ;
		if(table_led_count>= 2)
		{
			table_led_count= 0 ;
			tabulate_LED_STATE();
		}
		
		delay_ms(10);
	}
}
void vCOMMTask(void *pvParameters )
{
//	uint16_t ram_left = 0 ; 
	
	modbus_init();
	for( ;; )
	{
		if (dealwithTag)
		{  
		  dealwithTag--;
		  if(dealwithTag == 1)//&& !Serial_Master )	
			dealwithData();
		}
		if(serial_receive_timeout_count>0)  
		{
				serial_receive_timeout_count -- ; 
				if(serial_receive_timeout_count == 0)
				{
					serial_restart();
				}
		}
//		ram_left = uxTaskGetStackHighWaterMark(NULL);
//		printf("R%u", ram_left);
		delay_ms(5) ;
		
	}	
}

#ifdef T3PT12
void vI2C_READ(void *pvParameters)
{
	static u8 wait_ad_stable = 0 ;
	u8		i;
	float Rc[4] = {1118284 ,966354,895032,676148};
	float Rc1[4] = {1118284 ,966354,895032,676148};
		modbus.cal_flag	= 0 ;
	PT12_IO_Init();

	for( ;; )
	{
		if(modbus.cal_flag == 0)
		{
			wait_ad_stable ++ ;
			if(wait_ad_stable >3)
			{
				if(init_celpoint()) 
				{
					modbus.cal_flag = 1;
					for(i=0; i<4; i++)
					{
						Rc[i] = (float)rs_data[i] ;
						Rc1[i] = (float)rs_data[i+4] ;
					}
					min2method(&linear_K.temp_C, &linear_B.temp_C, 4, Rc, Rs);
					min2method(&linear_K_1.temp_C, &linear_B_1.temp_C, 4, Rc1, Rs1);
				}
			}
		
		}
		else
		{
			if(read_rtd_data())
			{
				update_temperature();
				control_input();
			}
			Flash_Write_Mass();
		}
		delay_ms(2000) ;				//if pulse changes ,store the data
	}	

}

#endif



 #if (defined T322AI)||(defined T38AI8AO6DO) ||(defined T36CTA)
void vSTORE_EEPTask(void *pvParameters )
{
	uint8_t  loop ;
	for( ;; )
	{
		
		for(loop=0; loop<MAX_AI_CHANNEL; loop++)
		{
			if(data_change[loop] == 1)
			{
				AT24CXX_WriteOneByte(EEP_PLUSE0_HI_HI+4*loop, modbus.pulse[loop].quarter[0]);
				AT24CXX_WriteOneByte(EEP_PLUSE0_HI_LO+4*loop, modbus.pulse[loop].quarter[1]);
				AT24CXX_WriteOneByte(EEP_PLUSE0_LO_HI+4*loop, modbus.pulse[loop].quarter[2]);
				AT24CXX_WriteOneByte(EEP_PLUSE0_LO_LO+4*loop, modbus.pulse[loop].quarter[3]);
				data_change[loop] = 0 ;
			}
		}
		
		delay_ms(3000) ;				//if pulse changes ,store the data

	}
	
}
#endif
#ifndef T3PT12
void vINPUTSTask( void *pvParameters )
{
	static u16 record_count = 0 ;

	inputs_init();
	for( ;; )
	{
		portDISABLE_INTERRUPTS() ;
		inputs_scan();	
		record_count ++ ;
		if(record_count == 10)
		{
			record_count= 0 ;
			control_input();
			Flash_Write_Mass();

		}
		portDISABLE_INTERRUPTS() ;
		delay_ms(200);
	}	
}
#endif
#if (defined T38AI8AO6DO) || (defined T36CTA)
void vOUTPUTSTask( void *pvParameters )
{

	output_init();
	for( ;; )
	{
//		
//		update_digit_output();		
		control_output();
		output_refresh();
		delay_ms(100);
	}	
}

void vKEYTask( void *pvParameters )
{
	
	KEY_IO_Init();
	for( ;; )
	{
		KEY_Status_Scan();
		delay_ms(100);
	}	
}

#endif


void Inital_Bacnet_Server(void)
{
//	u32 Instance = 0x0c;
//	Device_Init();
//	Device_Set_Object_Instance_Number(Instance);
		
 Device_Init();
 Device_Set_Object_Instance_Number(Instance);  
 address_init();
 bip_set_broadcast_addr(0xffffffff);

#if  READ_WRITE_PROPERTY   
#ifdef T322AI
  AIS = MAX_AIS;
  AVS = MAX_AVS;
#endif	
#if (defined T38AI8AO6DO) || (defined T36CTA)
  AIS = MAX_AIS;
  AOS = MAX_AO;
  BOS = MAX_DO;
  AVS = MAX_AVS;
#endif	



#endif	
}
void vMSTP_TASK(void *pvParameters )
{
	uint16_t pdu_len = 0; 
	BACNET_ADDRESS  src;
	Inital_Bacnet_Server();
	dlmstp_init(NULL);
	Recievebuf_Initialize(0);
	for (;;)
    {
		if(modbus.protocal == BAC_MSTP)
		{
					pdu_len = datalink_receive(&src, &PDUBuffer[0], sizeof(PDUBuffer), 0,	BAC_MSTP);
					if(pdu_len) 
						{
							npdu_handler(&src, &PDUBuffer[0], pdu_len, BAC_MSTP);	
						} 
						
		}
//			modbus.stack[1] = uxTaskGetStackHighWaterMark(NULL);
			delay_ms(5);
	}
	
}

void vNETTask( void *pvParameters )
{
	//uip_ipaddr_t ipaddr;
	
	//u8 tcp_server_tsta = 0XFF;
	//u8 tcp_client_tsta = 0XFF;
	//printf("Temco ARM Demo\r\n");
//	delay_ms(500);
	u8 count = 0 ;
	while(tapdev_init())	//初始化ENC28J60错误
	{								   
		delay_ms(50);
		printf("tapdev_init() failed ...\r\n");
	}
	
    for( ;; )
	{
		uip_polling();	//处理uip事件，必须插入到用户程序的循环体中 
		
		if((IP_Change == 1)||(update == 1))
		{
			count++ ;
			if(count == 10)
			{
				count = 0 ;
				IP_Change = 0 ;	
//				//if(!tapdev_init()) printf("Init fail\n\r");				
//				while(tapdev_init())	//初始化ENC28J60错误
//				{								   
//				//	printf("ENC28J60 Init Error!\r\n");
//				delay_ms(50);
//				};	
				SoftReset();
			}
			
		}
		IWDG_ReloadCounter(); 
//		modbus.stack[0] = uxTaskGetStackHighWaterMark(NULL);
		delay_ms(2);
    }
}



//uip事件处理函数
//必须将该函数插入用户主循环,循环调用.
void uip_polling(void)
{
	u8 i;
	static struct timer periodic_timer, arp_timer;
	static u8 timer_ok = 0;	 
	if(timer_ok == 0)		//仅初始化一次
	{
		timer_ok = 1;
		timer_set(&periodic_timer, CLOCK_SECOND / 2); 	//创建1个0.5秒的定时器 
		timer_set(&arp_timer, CLOCK_SECOND * 10);	   	//创建1个10秒的定时器 
	}
	
	uip_len = tapdev_read();							//从网络设备读取一个IP包,得到数据长度.uip_len在uip.c中定义
	if(uip_len > 0)							 			//有数据
	{   
//		printf("uip_len");
		//处理IP数据包(只有校验通过的IP包才会被接收) 
		if(BUF->type == htons(UIP_ETHTYPE_IP))			//是否是IP包? 
		{
			uip_arp_ipin();								//去除以太网头结构，更新ARP表
			uip_input();   								//IP包处理			
			//当上面的函数执行后，如果需要发送数据，则全局变量 uip_len > 0
			//需要发送的数据在uip_buf, 长度是uip_len  (这是2个全局变量)		    
			if(uip_len > 0)								//需要回应数据
			{
				uip_arp_out();							//加以太网头结构，在主动连接时可能要构造ARP请求
				tapdev_send();							//发送数据到以太网
			}
		}
		else if (BUF->type == htons(UIP_ETHTYPE_ARP))	//处理arp报文,是否是ARP请求包?
		{
			uip_arp_arpin();
			
 			//当上面的函数执行后，如果需要发送数据，则全局变量uip_len>0
			//需要发送的数据在uip_buf, 长度是uip_len(这是2个全局变量)
 			if(uip_len > 0)
				tapdev_send();							//需要发送数据,则通过tapdev_send发送	 
		}
	}
	else if(timer_expired(&periodic_timer))				//0.5秒定时器超时
	{
		timer_reset(&periodic_timer);					//复位0.5秒定时器 
		
		//轮流处理每个TCP连接, UIP_CONNS缺省是40个  
		for(i = 0; i < UIP_CONNS; i++)
		{
			 uip_periodic(i);							//处理TCP通信事件
			
	 		//当上面的函数执行后，如果需要发送数据，则全局变量uip_len>0
			//需要发送的数据在uip_buf, 长度是uip_len (这是2个全局变量)
	 		if(uip_len > 0)
			{
				uip_arp_out();							//加以太网头结构，在主动连接时可能要构造ARP请求
				tapdev_send();							//发送数据到以太网
			}
		}
		
#if UIP_UDP	//UIP_UDP 
		//轮流处理每个UDP连接, UIP_UDP_CONNS缺省是10个
		for(i = 0; i < UIP_UDP_CONNS; i++)
		{
			uip_udp_periodic(i);						//处理UDP通信事件
			
	 		//当上面的函数执行后，如果需要发送数据，则全局变量uip_len>0
			//需要发送的数据在uip_buf, 长度是uip_len (这是2个全局变量)
			if(uip_len > 0)
			{
				uip_arp_out();							//加以太网头结构，在主动连接时可能要构造ARP请求
				tapdev_send();							//发送数据到以太网
			}
		}
#endif 
		//每隔10秒调用1次ARP定时器函数 用于定期ARP处理,ARP表10秒更新一次，旧的条目会被抛弃
		if(timer_expired(&arp_timer))
		{
			timer_reset(&arp_timer);
			uip_arp_timer();
		}
	}
}


void EEP_Dat_Init(void)
{
		u8 loop ;
		u8 temp[6]; 
		AT24CXX_Init();
//		panelname[0] = 'T' ;
//		panelname[1] = 'e' ;
//		panelname[2] = 's' ;
//		panelname[3] = 't' ;
	
//		AT24CXX_Read(EEP_PANEL_NAME1, panelname, 20); 
//		if((panelname[0] ==0xff)&&(panelname[1] ==0xff)&&(panelname[2] ==0xff)&&(panelname[3] ==0xff)&&(panelname[4] ==0xff))
		{			
			#ifdef T322AI	
			Set_Object_Name("T3_22AI");
			#endif
			#ifdef T38AI8AO6DO	
			Set_Object_Name("T3_8AO6DO");
			#endif	
			#ifdef T36CTA
			Set_Object_Name("T3_6CTA");
			#endif
			#ifdef 	T3PT12
			Set_Object_Name("T3_PT12");
			#endif
		}
		modbus.serial_Num[0] = AT24CXX_ReadOneByte(EEP_SERIALNUMBER_LOWORD);
		modbus.serial_Num[1] = AT24CXX_ReadOneByte(EEP_SERIALNUMBER_LOWORD+1);
		modbus.serial_Num[2] = AT24CXX_ReadOneByte(EEP_SERIALNUMBER_HIWORD);
		modbus.serial_Num[3] = AT24CXX_ReadOneByte(EEP_SERIALNUMBER_HIWORD+1);
	
		if((modbus.serial_Num[0]==0xff)&&(modbus.serial_Num[1]== 0xff)&&(modbus.serial_Num[2] == 0xff)&&(modbus.serial_Num[3] == 0xff))
		{
					modbus.serial_Num[0] = 1 ;
					modbus.serial_Num[1] = 1 ;
					modbus.serial_Num[2] = 2 ;
					modbus.serial_Num[3] = 2 ;
//					AT24CXX_WriteOneByte(EEP_SERIALNUMBER_LOWORD, modbus.serial_Num[0]);
//					AT24CXX_WriteOneByte(EEP_SERIALNUMBER_LOWORD+1, modbus.serial_Num[1]);
//					AT24CXX_WriteOneByte(EEP_SERIALNUMBER_LOWORD+2, modbus.serial_Num[2]);
//					AT24CXX_WriteOneByte(EEP_SERIALNUMBER_LOWORD+3, modbus.serial_Num[3]);
		}
		Instance = modbus.serial_Num[0] + (U16_T)(modbus.serial_Num[1] << 8);
		AT24CXX_WriteOneByte(EEP_VERSION_NUMBER_LO, SOFTREV&0XFF);
		AT24CXX_WriteOneByte(EEP_VERSION_NUMBER_HI, (SOFTREV>>8)&0XFF);
		modbus.address = AT24CXX_ReadOneByte(EEP_ADDRESS);
		if((modbus.address == 255)||(modbus.address == 0))
		{
					modbus.address = 254 ;
					
					AT24CXX_WriteOneByte(EEP_ADDRESS, modbus.address);
		}
		 
		modbus.product = PRODUCT_ID ;
		Station_NUM = AT24CXX_ReadOneByte(EEP_STATION_NUM);
		if(Station_NUM > 127)
		{
					Station_NUM = 3 ;
		}
		temp[0] = AT24CXX_ReadOneByte(EEP_BACNET_PORT_HI);
		temp[1] = AT24CXX_ReadOneByte(EEP_BACNET_PORT_LO);
		if(temp[0]== 0xff & temp[1] == 0xff)
		{
			modbus.bacnet_port = 47808 ; 
		}
		else 
		{
			modbus.bacnet_port = (temp[0]<<8)|temp[1] ;
		}
		modbus.hardware_Rev = AT24CXX_ReadOneByte(EEP_HARDWARE_REV);
		if((modbus.hardware_Rev == 255)||(modbus.hardware_Rev == 0))
		{
					modbus.hardware_Rev = HW_VER ;
		//			AT24CXX_WriteOneByte(EEP_HARDWARE_REV, modbus.hardware_Rev);
		}
		modbus.update = AT24CXX_ReadOneByte(EEP_UPDATE_STATUS);
		modbus.SNWriteflag = AT24CXX_ReadOneByte(EEP_SERIALNUMBER_WRITE_FLAG);
		
		modbus.baud = AT24CXX_ReadOneByte(EEP_BAUDRATE);
		if(modbus.baud > 5) 
		{	
			modbus.baud = 1 ;
//			AT24CXX_WriteOneByte(EEP_BAUDRATE, modbus.baud);
		}
		switch(modbus.baud)
				{
					case 0:
						modbus.baudrate = BAUDRATE_9600 ;
						SERIAL_RECEIVE_TIMEOUT = 6;
					break ;
					case 1:
						modbus.baudrate = BAUDRATE_19200 ;
						SERIAL_RECEIVE_TIMEOUT = 3;
					break;
					case 2:
						modbus.baudrate = BAUDRATE_38400 ;
						SERIAL_RECEIVE_TIMEOUT = 2;
					break;
					case 3:
						modbus.baudrate = BAUDRATE_57600 ;
						SERIAL_RECEIVE_TIMEOUT = 1;
					break;
					case 4:
						modbus.baudrate = BAUDRATE_115200 ;
						SERIAL_RECEIVE_TIMEOUT = 1;
					break;
					case 5:
						modbus.baudrate = BAUDRATE_76800 ;	
						SERIAL_RECEIVE_TIMEOUT = 1;
					break;
					default:
					break ;				
				}
				uart1_init(modbus.baudrate);

				
				#if T36CTA
				temp[0] = AT24CXX_ReadOneByte(EEP_RFM12_ENCRYPT_KEY1);
				if((temp[0]>=0x21)&&(temp[0]<=0x7f))
				{
					for(loop = 0; loop <16 ;loop++)
					{
						rfm69_key[loop] = AT24CXX_ReadOneByte(EEP_RFM12_ENCRYPT_KEY1+loop);
					}
				}
				RFM69_networkID = (AT24CXX_ReadOneByte(EEP_RFM69_NETWORK_ID_HI)<<8)|AT24CXX_ReadOneByte(EEP_RFM69_NETWORK_ID_LO);
				if((RFM69_networkID == 0xffff)||(RFM69_networkID == 0))
				{
					RFM69_networkID = 0x55AA;
				}
				
				RFM69_nodeID = AT24CXX_ReadOneByte(EEP_RFM69_NODE_ID);
				if((RFM69_nodeID == 0xff)|| (RFM69_nodeID == 0))
				{
					RFM69_nodeID = 10;
				}
				RFM69_freq = ((AT24CXX_ReadOneByte(EEP_RFM69_FREQ_1)<<24)|(AT24CXX_ReadOneByte(EEP_RFM69_FREQ_2)<<16)
								|(AT24CXX_ReadOneByte(EEP_RFM69_FREQ_3)<<8)|(AT24CXX_ReadOneByte(EEP_RFM69_FREQ_4)));
				if((RFM69_freq == 0xffffffff) || (RFM69_freq == 0))
				{
					RFM69_freq = 433000000;
				}
				#endif
				
				modbus.protocal = AT24CXX_ReadOneByte(EEP_MODBUS_COM_CONFIG); 
				if((modbus.protocal != MODBUS)&&(modbus.protocal != BAC_MSTP))
				{
					modbus.protocal = MODBUS;
				}
				for(loop = 0 ; loop<6; loop++)
				{
					temp[loop] = AT24CXX_ReadOneByte(EEP_MAC_ADDRESS_1+loop); 
				}
				if((temp[0]== 0xff)&&(temp[1]== 0xff)&&(temp[2]== 0xff)&&(temp[3]== 0xff)&&(temp[4]== 0xff)&&(temp[5]== 0xff) )
				{
					temp[0] = 0x04 ;
					temp[1] = 0x02 ;
					temp[2] = 0x35 ;
					temp[3] = 0xaF ;
					temp[4] = 0x00 ;
					temp[5] = 0x01 ;
//					AT24CXX_WriteOneByte(EEP_MAC_ADDRESS_1, temp[0]);
//					AT24CXX_WriteOneByte(EEP_MAC_ADDRESS_2, temp[1]);
//					AT24CXX_WriteOneByte(EEP_MAC_ADDRESS_3, temp[2]);
//					AT24CXX_WriteOneByte(EEP_MAC_ADDRESS_4, temp[3]);
//					AT24CXX_WriteOneByte(EEP_MAC_ADDRESS_5, temp[4]);
//					AT24CXX_WriteOneByte(EEP_MAC_ADDRESS_6, temp[5]);		
				}
				for(loop =0; loop<6; loop++)
				{
					modbus.mac_addr[loop] =  temp[loop]	;
				}
				
				for(loop = 0 ; loop<4; loop++)
				{
					temp[loop] = AT24CXX_ReadOneByte(EEP_IP_ADDRESS_1+loop); 
				}
				if((temp[0]== 0xff)&&(temp[1]== 0xff)&&(temp[2]== 0xff)&&(temp[3]== 0xff) )
				{
					temp[0] = 192 ;
					temp[1] = 168 ;
					temp[2] = 0 ;
					temp[3] = 3 ;
					AT24CXX_WriteOneByte(EEP_IP_ADDRESS_1, temp[0]);
					AT24CXX_WriteOneByte(EEP_IP_ADDRESS_2, temp[1]);
					AT24CXX_WriteOneByte(EEP_IP_ADDRESS_3, temp[2]);
					AT24CXX_WriteOneByte(EEP_IP_ADDRESS_4, temp[3]);
				}
				for(loop = 0 ; loop<4; loop++)
				{
					modbus.ip_addr[loop] = 	temp[loop] ;
					modbus.ghost_ip_addr[loop] = modbus.ip_addr[loop] ;
					//printf("%u,%u,%u,%u,%u,%u,%u,%u,",  modbus.ip_addr[0], modbus.ip_addr[1], modbus.ip_addr[2], modbus.ip_addr[3],temp[0],temp[1],temp[2],temp[3]);
				}
				
				temp[0] = AT24CXX_ReadOneByte(EEP_IP_MODE);
				if(temp[0] >1)
				{
					temp[0] = 0 ;
					AT24CXX_WriteOneByte(EEP_IP_MODE, temp[0]);	
				}
				modbus.ip_mode = temp[0] ;
				modbus.ghost_ip_mode = modbus.ip_mode ;
				
				
				for(loop = 0 ; loop<4; loop++)
				{
					temp[loop] = AT24CXX_ReadOneByte(EEP_SUB_MASK_ADDRESS_1+loop); 
				}
				if((temp[0]== 0xff)&&(temp[1]== 0xff)&&(temp[2]== 0xff)&&(temp[3]== 0xff) )
				{
					temp[0] = 0xff ;
					temp[1] = 0xff ;
					temp[2] = 0xff ;
					temp[3] = 0 ;
					AT24CXX_WriteOneByte(EEP_SUB_MASK_ADDRESS_1, temp[0]);
					AT24CXX_WriteOneByte(EEP_SUB_MASK_ADDRESS_2, temp[1]);
					AT24CXX_WriteOneByte(EEP_SUB_MASK_ADDRESS_3, temp[2]);
					AT24CXX_WriteOneByte(EEP_SUB_MASK_ADDRESS_4, temp[3]);
				
				}				
				for(loop = 0 ; loop<4; loop++)
				{
					modbus.mask_addr[loop] = 	temp[loop] ;
					modbus.ghost_mask_addr[loop] = modbus.mask_addr[loop] ;
				}
				
				for(loop = 0 ; loop<4; loop++)
				{
					temp[loop] = AT24CXX_ReadOneByte(EEP_GATEWAY_ADDRESS_1+loop); 
				}
				if((temp[0]== 0xff)&&(temp[1]== 0xff)&&(temp[2]== 0xff)&&(temp[3]== 0xff) )
				{
					temp[0] = 192 ;
					temp[1] = 168 ;
					temp[2] = 0 ;
					temp[3] = 4 ;
					AT24CXX_WriteOneByte(EEP_GATEWAY_ADDRESS_1, temp[0]);
					AT24CXX_WriteOneByte(EEP_GATEWAY_ADDRESS_2, temp[1]);
					AT24CXX_WriteOneByte(EEP_GATEWAY_ADDRESS_3, temp[2]);
					AT24CXX_WriteOneByte(EEP_GATEWAY_ADDRESS_4, temp[3]);
				
				}				
				for(loop = 0 ; loop<4; loop++)
				{
					modbus.gate_addr[loop] = 	temp[loop] ;
					modbus.ghost_gate_addr[loop] = modbus.gate_addr[loop] ;
				}
				
				temp[0] = AT24CXX_ReadOneByte(EEP_TCP_SERVER);
				if(temp[0] == 0xff)
				{
					temp[0] = 0 ;
					AT24CXX_WriteOneByte(EEP_TCP_SERVER, temp[0]);
				}
				modbus.tcp_server = temp[0];
				modbus.ghost_tcp_server = modbus.tcp_server  ;
				
				temp[0] =AT24CXX_ReadOneByte(EEP_LISTEN_PORT_HI);
				temp[1] =AT24CXX_ReadOneByte(EEP_LISTEN_PORT_LO);
				if(temp[0] == 0xff && temp[1] == 0xff )
				{
					modbus.listen_port = 502 ;
					temp[0] = (modbus.listen_port>>8)&0xff ;
					temp[1] = modbus.listen_port&0xff ;				
				}
				modbus.listen_port = (temp[0]<<8)|temp[1] ;
				modbus.ghost_listen_port = modbus.listen_port ;
				
				modbus.write_ghost_system = 0 ;
				modbus.reset = 0 ;
//		#ifndef T3PT12
//		for(loop = 0; loop < 11; loop++)
//		{
//			temp[0] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+4*loop);
//			temp[1] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+1+4*loop);
//			temp[2] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+2+4*loop);
//			temp[3] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+3+4*loop);
//			if((temp[0] == 0xff)&&(temp[1]==0xff)&&(temp[2]==0xff)&&(temp[3]==0xff))
//			{
//				temp[0] = 0 ;
//				temp[1]= 0 ;
//				temp[2] = 0 ;
//				temp[3] = 0 ;
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+4*loop, temp[0]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+4*loop+1, temp[1]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+4*loop+2, temp[2]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE1_VOL_HI_0+4*loop+3, temp[3]);
//				modbus.customer_table1_val[loop] = 0 ;
//				modbus.customer_table1_vol[loop] = 0 ;
//			}
//			else
//			{
//				modbus.customer_table1_vol[loop] = (temp[0]<<8)+temp[1] ;
//				modbus.customer_table1_val[loop] = (temp[2]<<8)+temp[3] ;
//			}
//			
//			temp[0] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+4*loop);
//			temp[1] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+1+4*loop);
//			temp[2] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+2+4*loop);
//			temp[3] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+3+4*loop);
//			if((temp[0] == 0xff)&&(temp[1]==0xff)&&(temp[2]==0xff)&&(temp[3]==0xff))
//			{
//				temp[0] = 0 ;
//				temp[1]= 0 ;
//				temp[2] = 0 ;
//				temp[3] = 0 ;
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+4*loop, temp[0]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+4*loop+1, temp[1]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+4*loop+2, temp[2]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE2_VOL_HI_0+4*loop+3, temp[3]);
//				modbus.customer_table2_val[loop] = 0 ;
//				modbus.customer_table2_vol[loop] = 0 ;
//			}
//			else
//			{
//				modbus.customer_table2_vol[loop] = (temp[0]<<8)+temp[1] ;
//				modbus.customer_table2_val[loop] = (temp[2]<<8)+temp[3] ;
//			}
//			
//			temp[0] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+4*loop);
//			temp[1] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+1+4*loop);
//			temp[2] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+2+4*loop);
//			temp[3] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+3+4*loop);
//			if((temp[0] == 0xff)&&(temp[1]==0xff)&&(temp[2]==0xff)&&(temp[3]==0xff))
//			{
//				temp[0] = 0 ;
//				temp[1]= 0 ;
//				temp[2] = 0 ;
//				temp[3] = 0 ;
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+4*loop, temp[0]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+4*loop+1, temp[1]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+4*loop+2, temp[2]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE3_VOL_HI_0+4*loop+3, temp[3]);
//				modbus.customer_table3_val[loop] = 0 ;
//				modbus.customer_table3_vol[loop] = 0 ;
//			}
//			else
//			{
//				modbus.customer_table3_vol[loop] = (temp[0]<<8)+temp[1] ;
//				modbus.customer_table3_val[loop] = (temp[2]<<8)+temp[3] ;
//			}
//			
//			temp[0] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+4*loop);
//			temp[1] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+1+4*loop);
//			temp[2] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+2+4*loop);
//			temp[3] =AT24CXX_ReadOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+3+4*loop);
//			if((temp[0] == 0xff)&&(temp[1]==0xff)&&(temp[2]==0xff)&&(temp[3]==0xff))
//			{
//				temp[0] = 0 ;
//				temp[1]= 0 ;
//				temp[2] = 0 ;
//				temp[3] = 0 ;
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+4*loop, temp[0]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+4*loop+1, temp[1]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+4*loop+2, temp[2]);
////				AT24CXX_WriteOneByte(EEP_CUSTOMER_TABLE4_VOL_HI_0+4*loop+3, temp[3]);
//				modbus.customer_table4_val[loop] = 0 ;
//				modbus.customer_table4_vol[loop] = 0 ;
//			}
//			else
//			{
//				modbus.customer_table4_vol[loop] = (temp[0]<<8)+temp[1] ;
//				modbus.customer_table4_val[loop] = (temp[2]<<8)+temp[3] ;
//			}
//			

//		}
//#endif		
		#ifdef T322AI
		for(loop = 0; loop < 22; loop++)
		{
				
				modbus.pulse[loop].quarter[0] = AT24CXX_ReadOneByte(EEP_PLUSE0_HI_HI+4*loop) ;
				modbus.pulse[loop].quarter[1] = AT24CXX_ReadOneByte(EEP_PLUSE0_HI_LO+4*loop);
				modbus.pulse[loop].quarter[2] = AT24CXX_ReadOneByte(EEP_PLUSE0_LO_HI+4*loop) ;
				modbus.pulse[loop].quarter[3] = AT24CXX_ReadOneByte(EEP_PLUSE0_LO_LO+4*loop);
				if((modbus.pulse[loop].quarter[0] == 0xff)&&(modbus.pulse[loop].quarter[1]== 0xff)&&(modbus.pulse[loop].quarter[2]== 0xff)&& (modbus.pulse[loop].quarter[3] == 0xff))
				{
					modbus.pulse[loop].word = 0 ;
//					AT24CXX_WriteOneByte(EEP_PLUSE0_HI_HI+4*loop, modbus.pulse[loop].quarter[0]);
//					AT24CXX_WriteOneByte(EEP_PLUSE0_HI_LO+4*loop, modbus.pulse[loop].quarter[1]);
//					AT24CXX_WriteOneByte(EEP_PLUSE0_LO_HI+4*loop, modbus.pulse[loop].quarter[2]);
//					AT24CXX_WriteOneByte(EEP_PLUSE0_LO_LO+4*loop, modbus.pulse[loop].quarter[3]);
				}
				
		
		}
		#endif
		#if (defined T38AI8AO6DO) || (defined T36CTA)
//		for(loop=0; loop<MAX_AO; loop++)
//		{				
//				temp[1]	= AT24CXX_ReadOneByte(EEP_AO_CHANNLE0+2*loop);
//				temp[2]	= AT24CXX_ReadOneByte(EEP_AO_CHANNLE0+1+2*loop);
//				if((temp[1]== 0xff)&&(temp[2] == 0xff))
//				{
//					outputs[loop].value = 0 ;
//					AT24CXX_WriteOneByte(EEP_AO_CHANNLE0+2*loop+1, 0);
//					AT24CXX_WriteOneByte(EEP_AO_CHANNLE0+2*loop, 0);
//				}
//				else
//				{
//						outputs[loop].value = (temp[2]<<8)| temp[1] ;		
//				}
//		}
//		for(loop=MAX_AO; loop<MAX_DO+MAX_AO; loop++)
//		{
//			outputs[loop].value = AT24CXX_ReadOneByte(EEP_DO_CHANNLE0+loop);
//			if(outputs[loop].value> 1) 
//			{
//					outputs[loop].value = 1 ;
//					AT24CXX_WriteOneByte(EEP_DO_CHANNLE0+loop, outputs[loop].value);
//			}
//		}
		for(loop=0; loop<MAX_AI_CHANNEL; loop++)
		{
				inputs[loop].filter = AT24CXX_ReadOneByte(EEP_AI_FILTER0+loop);
				if(inputs[loop].filter == 0xff)
				{
						inputs[loop].filter = DEFAULT_FILTER ;
//						AT24CXX_WriteOneByte(EEP_AI_FILTER0+loop, inputs[loop].filter);
				}
				inputs[loop].range = AT24CXX_ReadOneByte(EEP_AI_RANGE0+loop) ;
				if(inputs[loop].range == 0xff)
				{
						inputs[loop].range = 0 ;
//						AT24CXX_WriteOneByte(EEP_AI_RANGE0+loop, inputs[loop].range);
				}
				
				inputs[loop].calibration_hi = AT24CXX_ReadOneByte(EEP_AI_OFFSET0+2*loop) ;
				inputs[loop].calibration_lo = AT24CXX_ReadOneByte(EEP_AI_OFFSET0+2*loop +1) ;
				if((temp[1]== 0xff)&&(temp[2] == 0xff))
				{
						inputs[loop].calibration_hi = (500>>8)&0xff ;
						inputs[loop].calibration_lo = 500&0xff ;
						temp[1] = inputs[loop].calibration_hi ;
						temp[2] = inputs[loop].calibration_lo ;
//						AT24CXX_WriteOneByte(EEP_AI_OFFSET0+2*loop, temp[1]);
//						AT24CXX_WriteOneByte(EEP_AI_OFFSET0+2*loop+1, temp[2]);
				}

				modbus.pulse[loop].quarter[0] = AT24CXX_ReadOneByte(EEP_PLUSE0_HI_HI+4*loop) ;
				modbus.pulse[loop].quarter[1] = AT24CXX_ReadOneByte(EEP_PLUSE0_HI_LO+4*loop);
				modbus.pulse[loop].quarter[2] = AT24CXX_ReadOneByte(EEP_PLUSE0_LO_HI+4*loop) ;
				modbus.pulse[loop].quarter[3] = AT24CXX_ReadOneByte(EEP_PLUSE0_LO_LO+4*loop);
				if((modbus.pulse[loop].quarter[0] == 0xff)&&(modbus.pulse[loop].quarter[1]== 0xff)&&(modbus.pulse[loop].quarter[2]== 0xff)&& (modbus.pulse[loop].quarter[3] == 0xff))
				{
					modbus.pulse[loop].word = 0 ;
					AT24CXX_WriteOneByte(EEP_PLUSE0_HI_HI+4*loop, modbus.pulse[loop].quarter[0]);
					AT24CXX_WriteOneByte(EEP_PLUSE0_HI_LO+4*loop, modbus.pulse[loop].quarter[1]);
					AT24CXX_WriteOneByte(EEP_PLUSE0_LO_HI+4*loop, modbus.pulse[loop].quarter[2]);
					AT24CXX_WriteOneByte(EEP_PLUSE0_LO_LO+4*loop, modbus.pulse[loop].quarter[3]);
				}
				
		}
		#endif
	
}

//u16 swap_int16( u16 value)
//{
//	u8 temp1, temp2 ;
//	temp1 = value &0xff ;
//	temp2 = (value>>8)&0xff ;
//	
//	return  (temp1<<8)|temp2 ;
//}

//u32 swap_int32( u32 value)
//{
//	u8 temp1, temp2, temp3, temp4 ;
//	temp1 = value &0xff ;
//	temp2 = (value>>8)&0xff ;
//	temp3 = (value>>16)&0xff ;
//	temp4 = (value>>24)&0xff ;
//	
//	return  ((u32)temp1<<24)|((u32)temp2<<16)|((u32)temp3<<8)|temp4 ;
//}
