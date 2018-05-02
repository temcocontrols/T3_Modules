
#include <12F675.h>
 
#define CLK_IN  PIN_A3
#define CPU_RST PIN_A0

#define TARGET_ASIX         0
#define TARGET_SYNCMOS      1   
#define T3_6CTA            2
//#define RESET_TARGET_CHIP      TARGET_ASIX 
//#define RESET_TARGET_CHIP      TARGET_SYNCMOS
#define RESET_TARGET_CHIP       T3_6CTA
                                       
#fuses INTRC_IO,WDT,NOPROTECT,NOMCLR
#use delay(clock = 4000000)  //Adjusts the accuracy of the delay functions

#define CLK     4000000
#define T_MS    10  
#define T_CTR   (65536 - T_MS * CLK / 4 / 8 / 1000)

#define TIMER_OVERFLOW_SET  30000 //30000 * 10ms = 300s = 5min
unsigned int16 timer_overflow_ctr = TIMER_OVERFLOW_SET;

BOOLEAN new_clk;
BOOLEAN old_clk;                 

#int_TIMER1     
void TIMER1_isr()
{                                               
    if(timer_overflow_ctr)
    {
        timer_overflow_ctr--;
    }
    
    SET_TIMER1(T_CTR);
}

/*
 *--------------------------------------------------------------------------------
 * void initial (void)
 * Purpose :
 * Params  :
 * Returns :
 * Note    :
 *--------------------------------------------------------------------------------
 */
void initial (void)
{
   //Set Chip oscillator to 4MHz
   #ASM ASIS
      BSF 0x3, 0x5
      MOVLW 0xFF   // need to keep clock at fastest or else cannot read
      MOVWF 0x90
      BCF 0x3, 0x5
      MOVLW 0x07
      MOVWF 0x19
   #ENDASM                      
   //setup_oscillator(OSC_4MHZ); //the internal 4M.
   // Setup timer1 
   timer_overflow_ctr = TIMER_OVERFLOW_SET;
   setup_timer_1( T1_INTERNAL | T1_DIV_BY_8 );
   set_timer1(T_CTR);                                             
   
   // set all interrupts
   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);

   // setup the watchdog for the PIC chip
   // the time can be adjusted by looking at the .h file
   SETUP_WDT(WDT_576MS);
   
   SET_TRIS_A(0xfe);  
   PORT_A_PULLUPS(0xfe);

#if (RESET_TARGET_CHIP == TARGET_ASIX) 
   output_low(CPU_RST);
   delay_ms(50);           
   output_high(CPU_RST);   
#elif (RESET_TARGET_CHIP == TARGET_SYNCMOS)
   output_high(CPU_RST);
   delay_ms(50);           
   output_low(CPU_RST);
#elif (  RESET_TARGET_CHIP == T3_6CTA) 
   output_low(CPU_RST);
   delay_ms(50);           
   output_high(CPU_RST);    
#endif 

   set_timer1(T_CTR);  
   
   delay_ms(50);
   new_clk = input(CLK_IN);
   old_clk = new_clk;      
}

/*
 *--------------------------------------------------------------------------------
 * void main(void)
 * Purpose :
 * Params  :
 * Returns :
 * Note    :
 *--------------------------------------------------------------------------------
 */
void main(void)
{
    initial();
   
    for(;;)
    {
        RESTART_WDT();
        new_clk = input(CLK_IN);
        if(new_clk != old_clk)
        {
            old_clk = new_clk;
            timer_overflow_ctr = TIMER_OVERFLOW_SET;
        }
        
        if(timer_overflow_ctr == 0)
        {
         #if (RESET_TARGET_CHIP == TARGET_ASIX) 
            output_low(CPU_RST);
            delay_ms(50);           
            output_high(CPU_RST);   
         #elif (RESET_TARGET_CHIP == TARGET_SYNCMOS)
            output_high(CPU_RST);
            delay_ms(50);           
            output_low(CPU_RST);
         #elif(RESET_TARGET_CHIP == T3_6CTA)
            output_low(CPU_RST);
            delay_ms(50);           
            output_high(CPU_RST); 
         #endif  
                    
            timer_overflow_ctr = TIMER_OVERFLOW_SET;
        }
    }        
}
/* End of piccore.c */



