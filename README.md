# T3_Modules
Modbus/Bacnet I/O modules

Generate a hex file for T3 6CTA
1)	Make sure application address is 0x8008000 
 
2)	Create a Hex file firmware
 
3)	Set vector table 0x8008000
 
4)	write a character string “TemcoT36CTA” to address 0x08008200, ISP tool use this to confirm it’s a right firmware for the device. 
