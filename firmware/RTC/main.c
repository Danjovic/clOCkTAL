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
    V0.2  - August 28, 2021 - Added auto brightness control
    V2.0  - May    16, 2023 - Changed engine to use DS3231 RTC    

*/


#include <pic16f688.h>
#include <stdint.h>
#include <stdbool.h>

//uint16_t __at _CONFIG configWord = _HS_OSC & _MCLRE_OFF & _CPD_OFF &  _CP_OFF & _WDT_OFF & _PWRTE_ON;  
uint16_t __at _CONFIG configWord = _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_OFF &  _CP_OFF & _WDT_OFF & _PWRTE_ON; // watchdog off

/*
   PIC16F688
                +--___--+ 
          +5V --|1    14|-- GND                          
      SCL RA5 --|2    13|-- RA0 LDR (Analog)
      SDA RA4 --|3    12|-- RA1 Button2       
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
#define _tresholdTime 3  // 16ms * 3 ~48ms

#define BUTTON1 !RA3 // ((PORTA & (1<<3)))  // RA3
#define BUTTON2 !RA1 // ((PORTA & (1<<1)))  // RA1


#define OUT 0
#define INP 1



// I2C Stuff
#define SDA  RA4
#define SCL  RA5
#define sendACK     true
#define sendNACK    false
#define RTC_ADDRESS 0x68

#define DIR_SDA TRISA4
#define DIR_SCL TRISA5

// I2C macros
#define sclHigh()   do { DIR_SCL = INP;  SCL = 1; } while (0)
#define sclLow()    do { DIR_SCL = OUT;  SCL = 0; } while (0)
#define sdaHigh()   do { DIR_SDA = INP;  SDA = 1; } while (0)
#define sdaLow()    do { DIR_SDA = OUT;  SDA = 0; } while (0)
#define sdaGet()    SDA 
#define I2Cdelay()  do {for (uint8_t i=1;i>0;i--) __asm__ ("nop\n\t"); } while (0)


//               _               _
//    __ _  _ __| |_ ___ _ __   | |_ _  _ _ __  ___ ___
//   / _| || (_-<  _/ _ \ '  \  |  _| || | '_ \/ -_|_-<
//   \__|\_,_/__/\__\___/_|_|_|  \__|\_, | .__/\___/__/
//                                   |__/|_|

typedef struct {
  unsigned units : 4;
  unsigned tens  : 3;
  unsigned       : 1;
} t_seconds;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 3;
  unsigned       : 1;
} t_minutes;

typedef struct {
  unsigned units   : 4 ;
  unsigned tens    : 2 ;
  unsigned op12_24 : 1 ;
  unsigned         : 1 ;
} t_hours;

typedef struct {
  unsigned dow     : 3 ;  // day of week
  unsigned         : 5 ;
} t_wkDays;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 2;
  unsigned       : 2;
} t_dates;

typedef struct {
  unsigned units   : 4 ;
  unsigned tens    : 1 ;
  unsigned         : 2 ;
  unsigned century : 1 ;
} t_monthsCty;

typedef struct {
  unsigned units : 4;
  unsigned tens  : 4;
} t_years;


typedef struct {
  t_seconds    seconds ;
  t_minutes    minutes ;
  t_hours        hours ;
  t_wkDays       wkDay ;
  t_dates         date ;
  t_monthsCty monthCty ;
  t_years         year ;
} t_timeAndDate;

typedef union  {
  uint8_t rawdata[7];
  t_timeAndDate datetime;
} t_ds3231records;
//
 

//                 _      _    _        
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//                                      

t_ds3231records rtc;
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


// I2C Stuff
void I2Cstart(void);
void I2Cstop(void);
bool I2Cwrite(uint8_t d);
uint8_t I2Cread(bool nack);

// RTC stuff
bool readRtc (void); // t_ds3231records *t);
bool writeRtc (void); // t_ds3231records *t);

void advanceMinutes(void);
void advanceHours(void);

// other
void init_hw(void);
void animateError(void);

//    _     _                         _      
//   (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
//   | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
//   |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
//                              |_|          
	
static void irqHandler(void) __interrupt 0 {

	// gets here every TMR0 int (every 256 ticks -> 128us) 
  
    tickCounter++;
    tickCounter &= 31;    // further division by 32 -> 4.096ms 
	if (tickCounter == 0) {
      
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
        } // end switch
        
        if (++digit > 3) {
			digit = 0;
            go = 1;                        // main tick every 16.384 ms 
	    }
    } else if (tickCounter == tOFF ) {   // rudimentary PWM
       RA2=1;                            // Turns off all displays before
       RC2=1;                            // the next cycle
       RC1=1;
       RC0=1;
    }	
    
	T0IF = 0;
}
//



//               _         __              _   _          
//    _ __  __ _(_)_ _    / _|_  _ _ _  __| |_(_)___ _ _  
//   | '  \/ _` | | ' \  |  _| || | ' \/ _|  _| / _ \ ' \.
//   |_|_|_\__,_|_|_||_| |_|  \_,_|_||_\__|\__|_\___/_||_|
//                                                        

void main (void) {
    uint8_t adcValue,lighSenseCounter=0;
    
	init_hw(); // Initialize hardware
		
	// main loop, run once at every ~4ms
	for (;;){
		while (!go); // wait for 4ms flag
		go=0;        // reset go flag


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
        if (++lighSenseCounter == 63 )  {  // overflow @ every ~1 second (63 * 16ms)
            lighSenseCounter = 0;
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


		
		if ( readRtc() ) {
			minutes = rtc.datetime.minutes.tens * 10 + rtc.datetime.minutes.units;
			hours   = rtc.datetime.hours.tens   * 10 + rtc.datetime.hours.units;
			}  else { // make display error pattern
				animateError();  
				}

        
	} // end of main loop
}
//




//     __              _   _             
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//

// ******************************************************************************************************************
//
// Hardware Initialization
//
void init_hw(void){

   // Initialize clock
   IRCF0 = 1 ;  // 8MHz -> IRCF[2..0] = 111 ( default 110 on startup )
   
   // initialize I/O ports   
   TRISC=0b00000000; // All as outputs (segments) 
   PORTC=0b00000111; // digits 1,2,3 unactive
    
   TRISA=0b11111011; // RA2 as output
   PORTA=0b00000110; // digit 0 unactive, pull-up on RA0 (LDR) and RA1 (button2)

   CMCON0 = ( _CM2 | _CM1 | _CM0); // analog comparators off, pins as I/O 

   // Initialize A/D converter
   ANSEL = (1<<0); // Configure analog/digital I/O (ANSEL)
   ADCON0 = 0b00000000; // CHS[2..0] = 000 -> Channel AN0 (@pin RA0)
   ADCON1 = 0b01110000;	 //  ADCS1 [2..0] = x11 -> FRC Internal 500KHz oscillator
   ADON = 1;
   GO_NOT_DONE = 1;   // Set GO/DONE bit to start conversions

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
//


// ******************************************************************************************************************
//
//  Error Animation
//
void animateError(void) {  // called in main loop at every 4ms
	static uint8_t timingCounter = 0;
	static uint8_t errorSequence = 0;


	switch (errorSequence) {
		case 0:  hours   = 0x38; minutes = 0x00; break;
		case 1:  hours   = 0x07; minutes = 0x00; break;
		case 2:  hours   = 0x00; minutes = 0x38; break;
		case 3:  hours   = 0x00; minutes = 0x07; break;
		default: hours   = 0x70; minutes = 0x07; break; // just in case
	}

	if (++timingCounter >= 13)  {  // change pattern every 13*16ms => 200ms
      timingCounter = 0;
      if (++errorSequence >= 4 )  errorSequence=0;
	
	}
}
//


// ******************************************************************************************************************
//
//  Time handling  
//

// advance current hour
void advanceHours (void) {
  uint8_t d = ((1
                + 10 * (uint8_t)rtc.datetime.hours.tens
                + (uint8_t)rtc.datetime.hours.units )   ) % 24;

  rtc.datetime.hours.tens = ( (uint8_t) d / 10 )  & 0b00000011;
  rtc.datetime.hours.units = ( (uint8_t) d % 10 ) & 0b00001111;
  writeRtc();

}
//

// advance current minute
void advanceMinutes (void) {
  uint8_t d = (1
               + 10 * (uint8_t)rtc.datetime.minutes.tens
               + (uint8_t)rtc.datetime.minutes.units ) % 60;

  rtc.datetime.minutes.tens = ( (uint8_t) d / 10 )  & 0b00000111;
  rtc.datetime.minutes.units = ( (uint8_t) d % 10 ) & 0b00001111;
  writeRtc();
}
//


// ******************************************************************************************************************
//
// RTC chip handling
//


// write time data on clock Chip using I2C
bool writeRtc ( void) {
  uint8_t i;
  I2Cstart();
  if (!I2Cwrite((uint8_t)(RTC_ADDRESS << 1)) ) {
    I2Cwrite( 0x00 ); // register address, 1st clock register
    for ( i = 0 ; i < 7 ; i++)
      I2Cwrite(rtc.rawdata[i]);
    I2Cstop();
    return true;
  } else {
    I2Cstop(); I2Cstop();
  } return false;
}
//

// read time data from clock Chip using I2C
bool readRtc ( void ) {
  uint8_t i;
  I2Cstart();
  if ( !I2Cwrite((uint8_t)(RTC_ADDRESS << 1)) ) {
    I2Cwrite( 0x00); // register address, 1st clock register
    I2Cstart();  // repeated start
    I2Cwrite((uint8_t)(RTC_ADDRESS << 1)  | 1);
    for ( i = 0 ; i < 6 ; i++) {
      rtc.rawdata[i] = I2Cread ( sendACK );
    }
    rtc.rawdata[i] = I2Cread ( sendNACK );       // NACK on last bit
    I2Cstop();
    return true;
  } else {
    I2Cstop(); I2Cstop();
  } return false;
}
//


// ******************************************************************************************************************
//
// Soft I2C
//

void I2Cstart(void) {
  sdaHigh(); I2Cdelay();
  sclHigh(); I2Cdelay(); // sda = 1;  scl = 1;
  sdaLow();  I2Cdelay(); // sda = 0;
  sclLow();
}
//

void I2Cstop(void) {
  sdaLow();  I2Cdelay(); // sda = 0;  sda = 0;
  sclHigh(); I2Cdelay(); // scl = 1;  scl = 1;
  sdaHigh();             // sda = 1;
}
//

bool I2Cwrite(uint8_t d) {
  uint8_t i;
  bool nack;
  for (i = 0; i < 8; i++) {
    if (d & 0x80)   // write data bit, msb first
      sdaHigh();
    else 
      sdaLow();
    I2Cdelay(); // give time to settle data
    sclHigh(); I2Cdelay();  sclLow(); // pulse clock
    d = d << 1; // next bit
  }
  // now get the ack
  sdaHigh(); I2Cdelay();  // release data line
  sclHigh(); I2Cdelay();  // release clock line
  nack = sdaGet();  // get nack bit
  sclLow();// clock low
  return nack;
}
//

uint8_t I2Cread(bool nack) {
  uint8_t i, d;

  d = 0;
  sdaHigh();             // release data line and
  sclLow(); I2Cdelay();  // pull down clock line and wait to write a bit
  for (i = 0; i < 8; i++)  {
    sclHigh(); I2Cdelay(); // release clock line to read the data
    d = d << 1;
    if (sdaGet() ) d |= 1; // read data bit, msb first
    sclLow(); I2Cdelay();  // pull clock down to allow next bit
  }
  // give ACK / NACK
  if ( nack ) sdaLow(); else sdaHigh();

  sclHigh(); I2Cdelay(); // Pulse clock
  sclLow(); I2Cdelay();  //

  sdaHigh(); // release the data line
  return d;
}
//



