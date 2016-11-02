/************************************************************
/* KEIL_VRS1000_regs.H                                      *
/*                                                          *
/* REV 1.0  - NOVEMBER 2003                                 *
/*                                                          *
/* Header file for the VRS1000 VERSA MCU microcontroller.   *
/* For Keil C Compiler.                                     *
/************************************************************/

#ifndef __VRS1000REGS_H__
#define __VRS1000REGS__


/*-----------------------*/
/*--- SFR REGISTERS -----*/
/*-----------------------*/

sfr P0       = 0x80;
sfr SP       = 0x81;
sfr DPL      = 0x82;
sfr DPH      = 0x83;
sfr RCON     = 0x85;
sfr DBANK    = 0x86;
sfr PCON     = 0x87;
sfr TCON     = 0x88;
sfr TMOD     = 0x89;
sfr TL0      = 0x8A;
sfr TL1      = 0x8B;
sfr TH0      = 0x8C;
sfr TH1      = 0x8D;
sfr P1       = 0x90;
sfr SCON     = 0x98;
sfr SBUF     = 0x99;
sfr	PWME     = 0x9B;
sfr WDTC     = 0x9f;
sfr P2       = 0xA0;
sfr PWMC     = 0xA3;
sfr PWMD0    = 0xA4;
sfr PWMD1    = 0xA5;
sfr PWMD2    = 0xA6;
sfr PWMD3    = 0xA7;
sfr IE       = 0xA8;
sfr PWMD4    = 0xAC;
sfr P3       = 0xB0;
sfr IP       = 0xB8;
sfr SYSCON   = 0xBF;
sfr T2CON    = 0xC8;
sfr T2MOD    = 0xC9;
sfr RCAP2L   = 0xCA;
sfr RCAP2H   = 0xCB;
sfr TL2      = 0xCC;
sfr TH2      = 0xCD;
sfr PSW      = 0xD0;
sfr P4       = 0xD8;
sfr ACC      = 0xE0;
sfr B        = 0xF0;
sfr IAPFADHI = 0xF4;
sfr IAPFADLO = 0xF5;
sfr IAPFDATA = 0xF6;
sfr ISPFCTRL = 0xF7;



/*--------------------------------------*/
/*---BIT ADDRESSABLE SFR REGISTERS -----*/
/*--------------------------------------*/

/*---TCON register's bit assignation---*/
sbit TF1   = TCON^7;
sbit TR1   = TCON^6;
sbit TF0   = TCON^5;
sbit TR0   = TCON^4;
sbit IE1   = TCON^3;
sbit IT1   = TCON^2;
sbit IE0   = TCON^1;
sbit IT0   = TCON^0;

/*---P1 register's bit assignation---*/
sbit T2EX  = P1^1;
sbit T2    = P1^0;

/*---SCON register's bit assignation---*/
sbit SM0   = SCON^7;
sbit SM1   = SCON^6;
sbit SM2   = SCON^5;
sbit REN   = SCON^4;
sbit TB8   = SCON^3;
sbit RB8   = SCON^2;
sbit TI    = SCON^1;
sbit RI    = SCON^0;

/*---IE register's bit assignation---*/
sbit EA    = IE^7;
sbit ET2   = IE^5;
sbit ES    = IE^4;
sbit ET1   = IE^3;
sbit EX1   = IE^2;
sbit ET0   = IE^1;
sbit EX0   = IE^0;

/*---IP register's bit assignation---*/
sbit PT2   = IP^5;
sbit PS    = IP^4;
sbit PT1   = IP^3;
sbit PX1   = IP^2;
sbit PT0   = IP^1;
sbit PX0   = IP^0;

/*---P3 register's bit assignation---*/
sbit RD    = P3^7;
sbit WR    = P3^6;
sbit T1    = P3^5;
sbit T0    = P3^4;
sbit INT1  = P3^3;
sbit INT0  = P3^2;
sbit TXD   = P3^1;
sbit RXD   = P3^0;

/*---T2CON register's bit assignation---*/
sbit TF2    = T2CON^7;
sbit EXF2   = T2CON^6;
sbit RCLK   = T2CON^5;
sbit TCLK   = T2CON^4;
sbit EXEN2  = T2CON^3;
sbit TR2    = T2CON^2;
sbit C_T2   = T2CON^1;
sbit CP_RL2 = T2CON^0;

/*---PSW register's bit assignation---*/
sbit CY    = PSW^7;
sbit AC    = PSW^6;
sbit F0    = PSW^5;
sbit RS1   = PSW^4;
sbit RS0   = PSW^3;
sbit OV    = PSW^2;
sbit P     = PSW^0;


#endif

