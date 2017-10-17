//T3-6CTA RF Link Test
//Feather M0 RHW69 using this firmware reads the T3-6CTA register 20020

#include "RFM69.h"
#include <SPI.h>


#define NETWORKID     1
#define NODEID        250
#define RECEIVER      1

#define RFNUMBEROFRETRIES 10
#define RFTIMEOUT     25
#define dWhileTimeout 2000 //While-loop timeout in ms

#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "ABCDEFGHIJKLMNOP"
#define IS_RFM69HCW   true

#define SERIAL_BAUD   9600

#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM69_IRQN    3 
#define RFM69_RST     4

#define LED           13

int16_t packetnumTX = 0;
int16_t packetnumRX = 0;
int16_t numOfFailsTX = 0;
int16_t numOfFailsRX = 0;

const char radiopacket_readModubusRegister20020[8] ={
  1,
  //Read instruction
  3,
  //Address 20020
  0x4e,
  0x34,
  //No. of Register to read
  0,
  1,         
  //CRC (calculated manually)
  0xD2,       
  0xEC  
};



RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

void setup() {
	Serial.begin(SERIAL_BAUD);
	//while (!Serial); // wait until serial console is open, remove if not tethered to computer
	Serial.println("RFM69HCW Transmitter");
	pinMode(LED, OUTPUT); 
	digitalWrite(LED,HIGH); 
	// Hard Reset the RFM module
	pinMode(RFM69_RST, OUTPUT);
	digitalWrite(RFM69_RST, HIGH);
	delay(100);
	digitalWrite(RFM69_RST, LOW);
	delay(100);

	// Initialize radio

   
	if(radio.initialize(FREQUENCY,NODEID,NETWORKID)){
		Serial.println("init ok");
	}else{
		Serial.println("init fail");
	}

	if (IS_RFM69HCW) {
		radio.setHighPower();
	}
	radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

	radio.encrypt(ENCRYPTKEY);
  radio.setAddress(NODEID);
  radio.setNetwork(NETWORKID);

  // Set bitrate 9600 bps
  radio.writeReg(0x03,0x0D);
  radio.writeReg(0x04,0x05);

	Serial.print("\nTransmitting at ");
	Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
	Serial.println(" MHz");



}


void loop() {
	  
  Blink(LED, 20, 1);
  
  Serial.print("\nSending Modbus packet "); Serial.print(++packetnumTX);  Serial.print(" (");Serial.print(numOfFailsTX);Serial.print(" TX failed)\n");
  
 if (radio.sendWithRetry(RECEIVER, radiopacket_readModubusRegister20020, sizeof(radiopacket_readModubusRegister20020),RFNUMBEROFRETRIES,RFTIMEOUT)) 
  { //target node Id, message as string or byte array, message length
    Serial.println("TX OK. Waiting for RX...");
  }else{
    Serial.println("TX Fail");
    numOfFailsTX++;
  }
  unsigned char RadioReceiveBuff[7];
  receive(RadioReceiveBuff, 7);

}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
	for (byte i=0; i<loops; i++)  {
		digitalWrite(PIN,HIGH);
		delay(DELAY_MS);
		digitalWrite(PIN,LOW);
		delay(DELAY_MS);
	}
}

bool receive(unsigned char* receivedStr, unsigned char length)
  {
    unsigned long while1InMillis = millis();
    //Wait for the radio dWhileTimeout ms to receive the string
    while (!radio.receiveDone())
    {     
      if ((unsigned long)(millis()-while1InMillis)>dWhileTimeout)
      {
        Serial.print("Breaking receiveDone while loop. ");Serial.print(" (");Serial.print(++numOfFailsRX);Serial.println(" RX failed).");
        Serial.println("RX failed");
        return false;
      }
    }
    unsigned char i=0;

    unsigned long while2InMillis = millis();
    //Wait for dWhileTimeout ms to read the string from the radio
    while(i<radio.DATALEN && radio.DATALEN<=RF69_MAX_DATA_LEN)
    {
      if ((unsigned long)(millis()-while2InMillis)>dWhileTimeout) 
      {
        Serial.print("Breaking DATALEN while loop. DATALEN=");Serial.print(" (");Serial.print(++numOfFailsRX);Serial.println(" RX failed).");
        Serial.println("RX failed");
        return false;
      }
      receivedStr[i]=(char)radio.DATA[i];
      i++;
  
    }

    Serial.print("Received Modbus packet "); Serial.print(++packetnumRX);  Serial.print(" (");Serial.print(numOfFailsRX);Serial.println(" RX failed):");
    Serial.print(receivedStr[0]);Serial.print(",");
    Serial.print(receivedStr[1]);Serial.print(",");
    Serial.print(receivedStr[2]);Serial.print(",");
    Serial.print(receivedStr[3]);Serial.print(",");
    Serial.print(receivedStr[4]);Serial.print(",");
    Serial.print(receivedStr[5]);Serial.print(",");
    Serial.print(receivedStr[6]);Serial.print("\n");
    
    if (radio.ACKRequested())//If sender asks for the acknowledgement,
    {
      //send the acknowledgement
      Serial.println("Sending ACK\n");
      radio.sendACK();
    }
    else
    {
      Serial.println("ACK not requested\n");
    }
    return true;
  }
