/***
         ____ _            _    _        _  
        / ___| | ___   ___| | _| |_ __ _| | 
       | |   | |/ _ \ / __| |/ / __/ _` | | 
       | |___| | (_) | (__|   <| || (_| | | 
        \____|_|\___/ \___|_|\_\\__\__,_|_| 
                                            
  
    Octal Clock implementation by Danjovic
	(c) Daniel Jose Viana, 2021 - danjovic@hotmail.com
	Release under GPL V2.0

    V0.1  - August 25, 2021 - basic implementation
    Vo.2  - August 28, 2021 - Added auto brightness control

*/


#include <pic16f688.h>
#include <stdint.h>
#include <stdbool.h>

uint16_t __at _CONFIG configWord = _HS_OSC & _MCLRE_OFF & _CPD_OFF &  _CP_OFF & _WDT_OFF & _PWRTE_ON;  

/*
   PIC16F688
                +--___--+ 
          +5V --|1    14|-- GND                          
     Xtal RA5 --|2    13|-- RA0 LDR (Analog)
     Xtal RA4 --|3    12|-- RA1 Button2       
  Button1 RA3 --|4    11|-- RA2 Digit0      
     Bit0 RC5 --|5    10|-- RC0 Digit3          
     Bit1 RC4 --|6     9|-- RC1 Digit2               
     Bit2 RC3 --|7     8|-- RC2 Digit1     
                +-------+  
*/


//       _      __ _      _ _   _             
//    __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
//   / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
//   \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
//                                            
#define _tresholdTime 10  // 4ms * 10 ~40ms

#define BUTTON1 !RA3 // ((PORTA & (1<<3)))  // RA3
#define BUTTON2 !RA1 // ((PORTA & (1<<1)))  // RA1


//                 _      _    _        
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//                                      

static volatile uint8_t hours   = 23;
static volatile uint8_t minutes = 59;
static uint8_t seconds = 0;
static uint32_t bres=0;

static volatile uint8_t go=0;
static volatile uint8_t tickCounter=0;
static volatile uint8_t tOFF = 32;   
 
uint8_t digit=0;
uint8_t timeButton1pressed,timeButton2pressed=0;


//                 _       _                     
//    _ __ _ _ ___| |_ ___| |_ _  _ _ __  ___ ___
//   | '_ \ '_/ _ \  _/ _ \  _| || | '_ \/ -_|_-<
//   | .__/_| \___/\__\___/\__|\_, | .__/\___/__/
//   |_|                       |__/|_|           

inline void advanceSeconds(void);
inline void advanceMinutes(void);
inline void advanceHours(void);
void init_hw(void);


//    _     _                         _      
//   (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
//   | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
//   |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
//                              |_|          
	
static void irqHandler(void) __interrupt 0 {
	// gets here every TMR0 int (every 256 ticks)
	bres+=256;
	if (bres>=2000000) // 8MHz cpu = 2000000 cycles per second
    {    
		bres=bres-2000000;
		advanceSeconds();
	}
    
    tickCounter++;
    tickCounter &= 31;
	if (tickCounter == 0) 
       go=1; // ~4ms per digit
    else if (tickCounter == tOFF ) {   // rudimentary PWM
       RA2=1;                          // Turns off all displays before
       RC2=1;                          // the next cycle
       RC1=1;
       RC0=1;
    }	
    
	T0IF = 0;
}



//     __              _   _             
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//                                       

inline void advanceSeconds(void) {
  if (++seconds > 59 ) {
     seconds = 0;
     if (++minutes > 59 ) {
        minutes = 0;
        if (++hours > 23 ) {
           hours = 0;
        }
     }
  }
}


inline void advanceMinutes(void) {
   if (++minutes > 59 ) {
      minutes = 0;
   }
}

inline void advanceHours(void) {
   if (++hours  > 23 ) {
      hours = 0;
   }
}

inline void initADC(void) {
// These steps should be followed for an A/D conversion:
// 1. Configure the A/D module:
// • Configure analog/digital I/O (ANSEL)
   ANSEL = (1<<0);
// • Configure voltage reference (ADCON0)
// • Select A/D input channel (ADCON0)
   ADCON0 = 0b00000000;
//            |||||||+--- ADON  0 = A/D converter module is not operating            
//            ||||||+---- GO/DONE 1 command new conversion, clear when ready
//            |||||+----- CHS0 
//            ||||+------ CHS1  000 = Channel 00 (AN0)
//            |||+------- CHS2  
//            ||+-------- unimplemented
//            |+--------- VCFG 0 = VDD    
//            +---------- ADFM 0 = Left justified
// • Select A/D conversion clock (ADCON1)
   ADCON1 = 0b01110000;	
//            |||||||+--- unimplemented
//            ||||||+---- unimplemented
//            |||||+----- unimplemented
//            ||||+------ unimplemented
//            |||+------- ADCS0
//            ||+-------- ADCS1 x11 = FRC Internal 500KHz oscillator
//            |+--------- ADCS2 
//            +---------- unimplemented
// • Turn on A/D module (ADCON0)
   ADON = 1;
// 2. Configure A/D interrupt (if desired):
// • Clear ADIF bit (PIR1<6>)
// • Set ADIE bit (PIE1<6>)
// • Set PEIE and GIE bits (INTCON<7:6>)
// 3. Wait the required acquisition time.
// 4. Start conversion:
// • Set GO/DONE bit (ADCON0<0>)
   GO_NOT_DONE = 1; 
// 5. Wait for A/D conversion to complete, by either:
// • Polling for the GO/DONE bit to be cleared
//while (GO_NOT_DONE);
// (with interrupts disabled); OR
// • Waiting for the A/D interrupt
// 6. Read A/D Result register pair
//value = ADRESH;
// (ADRESH:ADRESL), clear bit ADIF if required.
// 7. For next conversion, go to step 1 or step 2 as
// required. The A/D conversion time per bit is
// defined as TAD. A minimum wait of 2 TAD is
// required before the next acquisition starts.



}

void init_hw(void){
   // initialize I/O ports   
   TRISC=0b00000000; // All as outputs (segments) 
   PORTC=0b00000111; // digits 1,2,3 unactive
    
   TRISA=0b11111011; // RA2 as output
   PORTA=0b00000110; // digit 0 unactive, pull-up on RA0 (LDR) and RA1 (button2)

   CMCON0 = ( _CM2 | _CM1 | _CM0); // analog comparators off, pins as I/O 

   // Initialize A/D converter
   initADC();

   // Program Timer 0 interrupt every 256 instruction cycles 
   OPTION_REG = _PSA; 
   // _PSA       1 = Prescaler is assigned to the WDT (timer 0-> 1:1)
   //  _T0CS     0 = Internal instruction cycle clock 
   //  _NOT_RAPU 0 = PORTA pull-ups enabled by port latch values
 
	TMR0  = 0;  // clear timer 0 counter
	T0IF  = 0;  // clear any pending timer 0 flag
    T0IE  = 1;  // enable timer 0 interrupts
    GIE   = 1;  // enable global interrupts

}



//               _         __              _   _          
//    _ __  __ _(_)_ _    / _|_  _ _ _  __| |_(_)___ _ _  
//   | '  \/ _` | | ' \  |  _| || | ' \/ _|  _| / _ \ ' \ 
//   |_|_|_\__,_|_|_||_| |_|  \_,_|_||_\__|\__|_\___/_||_|
//                                                        

void main (void) {
    uint8_t adcValue,lighSenseCounter=0;
    
	init_hw(); // Initialize hardware
		
	// main loop, run once at every ~4ms
	for (;;){
		while (!go); // wait for 4ms flag
		go=0;        // reset go flag
		

		// update display
        switch (digit) {
        
        case 0: // least significant digit;
           RC0 = 1; // erase last display
           if (minutes & (1<<0) ) RC5 = 1; else RC5 = 0;  
           if (minutes & (1<<1) ) RC4 = 1; else RC4 = 0;  
           if (minutes & (1<<2) ) RC3 = 1; else RC3 = 0;  
           RA2 = 0; // Turn on this display
           break;
           
        case 1:
           RA2 = 1; // erase last display
           if (minutes & (1<<3) ) RC5 = 1; else RC5 = 0;  
           if (minutes & (1<<4) ) RC4 = 1; else RC4 = 0;  
           if (minutes & (1<<5) ) RC3 = 1; else RC3 = 0;
           RC2 = 0; // Turn on this display        
           break; 
       
        case 2:
           RC2 = 1; // erase last display
           if (hours   & (1<<0) ) RC5 = 1; else RC5 = 0;  
           if (hours   & (1<<1) ) RC4 = 1; else RC4 = 0;  
           if (hours   & (1<<2) ) RC3 = 1; else RC3 = 0; 
           RC1 = 0; // Turn on this display        
           break;   
     
        case 3:  // most significant digit
           RC1 = 1; // erase last display
           if (hours   & (1<<3) ) RC5 = 1; else RC5 = 0;  
           if (hours   & (1<<4) ) RC4 = 1; else RC4 = 0;  
           if (hours   & (1<<5) ) RC3 = 1; else RC3 = 0;  
           RC0 = 0; // Turn on this display 
           break;        
        }
        
        if (++digit > 3) digit = 0;
        

		// Test for buttons
		if (BUTTON1) { // button pressed
			if (timeButton1pressed<255) timeButton1pressed++;	
		} else {       // Release
			if (timeButton1pressed > _tresholdTime) {
				advanceHours();
			}
			timeButton1pressed =0;
		}

		if (BUTTON2) { // button pressed
			if (timeButton2pressed<255) timeButton2pressed++;	
		} else {       // Release
			if (timeButton2pressed > _tresholdTime) {
				advanceMinutes();
			}
			timeButton2pressed =0;
		}	
        
        // Read the LDR
        #define VOFFSET 87
        #define VRANGE  31
        if (++lighSenseCounter ==0)  {  // overflow @ every ~1 second

            if (GO_NOT_DONE==0) {   // New value available

               adcValue = ADRESH;   // read ADC value 
               GO_NOT_DONE = 1;     // start the next conversion
                
               // LDR voltage maximum range shall be in the range 82 to 250 (typycally 87 to 150)
               // this function limits the tOFF to 1 to 32
               if (adcValue < (VOFFSET) )
                  tOFF = 1 ;  // minimum brightness
               else if (adcValue > ( VOFFSET + VRANGE) ) 
                       tOFF = 1 + VRANGE;    
               else 
                   tOFF = 1 + adcValue - VOFFSET;  
                      
            }
        }
	} // end of main loop
}