#ifndef 	_UDP_SCAN_H
#define  	_UDP_SCAN_H


#include "dhcpc.h"
#include "modbus.h"


extern u8 InformationStr[60];
extern u32 Instance ;
extern uint8_t panelname[20] ;
void udp_scan_init(void) ;

void udp_appcall(void) ;


void dhcpc_configured(const struct dhcpc_state *s);


#define UIP_UDP_APPCALL		udp_appcall



extern u8 IP_Change ;



//void UdpData(u8 type) ;

void udp_scan_reply(u8 type) ;
extern u8 update ;





#endif
