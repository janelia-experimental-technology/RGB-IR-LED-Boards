// Fly Bowl Board Code
// 20140521 sws ID&F Janelia, HHMI
// based on Olfactory arena code
// Target: Teensy 2.0 controller
// Built with: Arduino 1.6.4 with Teensy extensions


// CHR pc  - set chrimson brightness
// IR pc – set IR brightness
// ON x,y  - turn on Chr LEDs
// OFF x,y – turn off Chr LEDs/
//
// Where:
// -	 pc is percent of full brightness (0-100). A 0 value will be off
// -	x is the column of the area to turn on, 1-4 , 0 will turn on (or off) all areas in the row set by ‘y’
// -	y is the row to of the area to turn on, 1-4 , 0 will turn on (or off) all areas in the column set by ‘x’.
// -	a 0,0 value for x and y will change all areas
//
// There is no separate on/off for the IR LEDs.

/*
Janelia Open-Source Software (3-clause BSD License)

Copyright 2022 Howard Hughes Medical Institute

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
*/

//#define USE_SCALING  // define this to allow log brightness control
#define LINEAR_DRIVER  //define this if linear driver
//#define USE_EXT_TRIG  // enable this to use ext trigger
//#define MARKER_ACTIVE_LOW  // marker pin is active low 

// #define DEBUG // enable to send debug info out serial USB

#define VERSION "20200224ENV"

// 20200224ENV sws
// - force state to HOLD when STOP is sent to be sure marker LED is turned off

// 20170222ENV sws
// - special for Salk Institute - have marker envelope the pulses since short pulses would not be caught at low frame rates

// 20170228 sws
// - add DEBUG define to stop debug info all the time

// 20170209 sws
// - increase on time and period to 300 seconds

//  20170126 sws
//  - add ext trig note on ???

// 20160120 sws
// - add CNT command to enter in Chrimson setting as 0-4095 DAC counts
//    note that most controllers have a 10 bit DAC so only will change every 4th value
//     but a drop-in DAC can be substituted that is 12 bits (0-4095)


// 20160115 sws
// - change intensity to float to utilize DAC resolution better

// 20150615 sws
// - use define to turn ext trig capability on and off
// - set max offtime to 60 secs not 30

// 20150606 sws
// - add in calibartion for Chr
//    "CAL val" or "CAL ON" or "CAL OFF"

// 20150602 sws
// - changes for Tihana
//       - set setting to 0xffff in setup
//       -  change ext trig to turn LEDs on with high level 

// 20150218 sws
// - lengthen max pulse width and period from 1000 to 30000 msecs
// - change STOP to PAUSE and make STOP reset the timer

// 20150120 sws
// - at RUN with no wait time, the timer would wait out whatever remaining counts
//    there were from the previous run; add line to force counrt to 0 at run with no wait
// - add LIN and LOG commands to set scaling - default to LIN (linear)
// - add iterations to pulse mode to set number of loops of pulses

// 20141206 sws
// - move setup info from setup to ??? cmd

// 20141116 sws
// - Skip any non-printable chars at end of input line; CR after LF was messing things up

//20141026
// - Marker LED setting is opposite for Linear vs Switched code

//20141025 sws
// - add IR enable pin

//20141023 sws
// - EXT command with READY and DONE ouputs
// - turn off LEDS when STOP cmd is sent

// 20140927 sws
// - print out mode at start up
// - fine tune offset on non-linear to allow lowest brightness (may not light all LEDS at lowest settings)

// 20140923 sws
// combine all flavors into one - use the defines above to set proper behavior

// 20140923 sws
// allow disabling of scaling (USE_SCALING)

// 20140917 sws
// turn off print pulse state

// 20140914 sws
//  - add PATT command and use setting to hold pattern
//      modify the pulse interrupt to use pattern

// 20140807 sws 
// - modify timing
//  Old:
//  %Single pulse width (Light on period) : 1ms-1000ms
//  %Single pulse period (Light on period plus light off period): 25ms-1000ms
//  %Number of pulse in one pulse train (1 second/B): 1-40 times
//  %Interval time between two pulse trains : 0-10000ms
//  New: 
//  %Single pulse width (Light on period) : 1ms-1000ms
//  %Single pulse period (Light on period plus light off period): 1ms-1000ms
//  %Number of pulse in one pulse train (1 second/B): 1-1000 times
//  %Interval time between two pulse trains : 0-30000ms

// 20140805 sws
// - add linear control with non-linear brightness

//20140710 - add marker LED on pin 11, active low

// 20140521 sws

// allow steady on/off using onflag

// V5 14nov2013 sws
// - add IR led as indicator lamp - use spare pin (col 4) on row 1
//    500 ohm resistor in series with side looking IR emitter
//    can set up brightness with LED 1,4, n (n = 0-100%)

// V4
// 22oct2013 sws
// - extend light on to 1ms to 1000msec (from 5 to 500)
// - allow 0 interval time (continuous) (from 0.5 to 10)
// - if pulse width = pulse period - stay on




//=========== COMMAND INTERFACE =======================

#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include <TimerThree.h>
#include <TimerOne.h>
#include <EEPROM.h>

static uint16_t setting = 0;

int alarmPin = 13;  // high temperature alarm - and now will also be envelope out (OC)
#define ext_trig_pin 12 

#define IR_ON_PIN 13

uint8_t trig_status;
uint8_t ext_trig_en;
//uint8_t rowenPin[] = { 0, 19, 18, 17, 16, 15, 14, 8, 7 , 4, 3, 2, 1, 0 };

uint8_t onflag = 0; // allow steady on/off

#define PIN00  1
#define PIN10  0
#define PIN20 10
#define PIN30  8
#define PIN01  3
#define PIN11  2
#define PIN21  9
#define PIN31  7
#define PIN02 17
#define PIN12 15
#define PIN22 19
#define PIN32 18
#define PIN03 16
#define PIN13 14
#define PIN23 21
#define PIN33 20

#define MARKERPIN 11

#define TESTPIN 24


#define DACADR0 1
#define AD73x5_MAX_DEVICE 2

#define CR 0x0a
#define BUFFLEN 64

#define MINWIDTH     1   //5
#define MAXWIDTH  300000  //1000  //500
#define MINPERIOD    1   //25
#define MAXPERIOD 300000  //1000
#define MINPULSES    1
#define MAXPULSES 1000   //40
#define MINOFF       0
#define MAXOFF   600000  //30000 // 10000
#define MINWAIT      0
#define MAXWAIT    120
#define MINRATE      1
#define MAXRATE   1000
#define MINITERATIONS  0
#define MAXITERATIONS 30000

#define LINEAR 0
#define LOG    1


static char cmdbuf[BUFFLEN];
int cbp;
int cmderr;
int zeroA;
uint8_t i;


uint16_t pulse_on_msec;
uint16_t pulse_off_msec;
uint16_t off_time_msec;
uint16_t pulse_count;
uint16_t wait_sec;
uint16_t wait_cnt;
uint16_t frame_on_msec;
uint16_t frame_off_msec;
uint16_t iteration_loops = 0;

uint16_t iteration;

//elapsedMillis pulse_timer;
uint16_t pulse_counter;
int pulse_enabled;

uint16_t frame_on_cnt;
uint16_t frame_off_cnt;

#define START    0
#define WAIT     1
#define PULSEON  2
#define PULSEOFF 3
#define OFFTIME  4
#define CONTINUOUS 5
#define HOLD    6


int pulse_state = START;
int last_state = 6;

int bright_mode = LINEAR;

// timer variables

volatile int16_t next_timer;
volatile int8_t flag = 0;
volatile int8_t ledon = 0;
volatile int16_t pulse_timer;
volatile int8_t  sync_on;

// ==========================
// === P U L S E   C M D ====
//  PULSE a,b,c,d,e
//    a - pulse width
//    b - pulse period
//    c - # pulses
//    d - off time
//    e - wait time
//    f - number of iterations; 0 = continuous
// ==========================

int  PULSEcmd(char * cmdp)
{
uint16_t val[5];
int i;

Serial.println("P");

  for( i = 0; i < 5; i++ )
  {
    while ( isdigit(*cmdp) == 0 )
    {
       if( *cmdp == '\0') 
         return -1;
       else  
         cmdp++;  // get past any non-digit
    }     
    val[i] = atoi(cmdp);  // get value
   
    while( isdigit(*cmdp) ) cmdp++;
  }
  
  // get the optional iteration value
   while ( isdigit(*cmdp) == 0 )
   {
       if( *cmdp == '\0') 
       {
         val[5] = 0;
         break;
       }  
       else  
         cmdp++;  // get past any non-digit
    }     
    val[5] = atoi(cmdp);  // get value
     
 
  if( (val[0] < MINWIDTH)  || (val[0] > MAXWIDTH)  ) return -2;
  if( (val[1] < MINPERIOD) || (val[1] > MAXPERIOD) ) return -3;
  if( (val[2] < MINPULSES) || (val[2] > MAXPULSES) ) return -4;
  if( (val[3] < MINOFF)    || (val[3] > MAXOFF)    ) return -5;
  if( (val[4] < MINWAIT)   || (val[4] > MAXWAIT)   ) return -6;
  if( (val[5] < MINITERATIONS) || (val[5] > MAXITERATIONS)   ) return -7;
  
  if( val[1] < val[0] ) return -8;
  
  pulse_on_msec = val[0];
  pulse_off_msec = val[1] - val[0];
  pulse_count = val[2];
  off_time_msec = val[3];
  wait_sec  = val[4];
  iteration_loops = val[5];
  
//  Serial.println(pulse_on_msec);
//  Serial.println(pulse_off_msec);
//  Serial.println(off_time_msec);
//  Serial.println(pulse_count);
  
  return 0;
} 

// =========================
// === F R A M E  C M D ====
// =========================

int  FRAMEcmd(char * cmdp)
{
uint16_t val[2];
int i;


  for( i = 0; i < 2; i++ )
  {
    while ( isdigit(*cmdp) == 0 )
    {
       if( *cmdp == '\0') 
         return -1;
       else  
         cmdp++;  // get past any non-digit
    }     
    val[i] = atoi(cmdp);  // get quad
   
    while( isdigit(*cmdp) ) cmdp++;
  }
  
  if( (val[0] < MINRATE)   || (val[0] > MAXRATE)   ) return -1;
  if( (val[1] < 1)  || (val[1] > 100)  ) return -2;
  val[1] = (10 * (100-val[1])); 
  
  Timer3.setPeriod( 1e6/val[0] );
  Timer3.pwm(9,val[1]);

  return 0;
}   

// =========================
// === P A R S E  C M D ====
// =========================

int ParseCmd()
{
//int i;
char * cmdp;

    // upcase everything
   cmdp = cmdbuf;
   while( *cmdp != '\0' )
   {
       *cmdp = toupper(*cmdp);
       cmdp++;
   } 

#ifdef DEBUG
   Serial.println(cmdbuf);
#endif   
//   if( strncmp(cmdbuf, "CHR", 2) != 0 )
//     Serial.println( strncmp(cmdbuf, "CHR", 2));
      
   if( strncmp(cmdbuf, "CHR", 3) == 0 )
   {    
      Serial.print("C");
      cmdp = cmdbuf + 3;
      return(BRIGHTcmd(cmdp, 1));
   }
   if( strncmp(cmdbuf, "CNT", 3) == 0 )
   {    
      Serial.print("c");
      cmdp = cmdbuf + 3;
      uint16_t val = atoi(cmdp);
      if( val > 4095 ) val = 4095;
      SetIntensity(val);
      return(0);
   }
   else if( strncmp(cmdbuf, "IR", 2) == 0 )
   {    
      Serial.print("I");
      cmdp = cmdbuf + 2;
      return(BRIGHTcmd(cmdp, 0));
   }     
   else if( strncmp(cmdbuf, "RUN", 3) == 0 )
   {
      pulse_enabled = 1;
      pulse_state = START;

// Serial.println("RUN");
      digitalWrite(TESTPIN, 1);
      return(0);
   } 
   else if( strncmp(cmdbuf, "STOP", 4) == 0 )
   {
      pulse_enabled = 0;
      pulse_state = HOLD;  // 20200224 - need non-pulse state to tun off marker
 //     Timer3.stop();
      SetLEDs(0x0000);
      digitalWrite(TESTPIN, 0);
      pulse_timer = 0; 
       
      return(0);
   }  
   else if( strncmp(cmdbuf, "PAUSE", 5) == 0 )
   {
      pulse_enabled = 0;
//      Timer3.stop();
      SetLEDs(0x0000);
      digitalWrite(TESTPIN, 0);
      return(0);
   }   
   else if( strncmp(cmdbuf, "TST", 3) == 0 )
   {
      cmdp = cmdbuf + 3;
      return(TSTcmd(cmdp));
   } 
   else if( strncmp(cmdbuf, "OFF", 3) == 0 )
   {
      cmdp = cmdbuf + 3;
      return(LEDcmd(cmdp, 0) );
   }
   else if ( strncmp(cmdbuf, "ON", 2 ) == 0)
   {
      cmdp = cmdbuf + 2;
      return(LEDcmd(cmdp, 1) );
   }  
   else if ( strncmp(cmdbuf, "PATT", 4 ) == 0)
   {
      cmdp = cmdbuf + 2;
      return(PATTcmd(cmdp) );
   }  
   
   else if ( strncmp(cmdbuf, "EXT", 3 ) == 0)
   {
      cmdp = cmdbuf + 3;
      while ( isdigit(*cmdp) == 0 )
      {
         if( *cmdp == '\0') 
           return -1;
         else  
           cmdp++;  // get past any non-digit          
      }

      ext_trig_en = atoi(cmdp);  // get new trigger status  
      Serial.print("trig ");
      if( ext_trig_en == 1 )
         Serial.println("Enabled");
      else
         Serial.println("Disabled");   
      return(0);
   } 
   else if( strncmp(cmdbuf, "PULSE", 5) == 0 )
   {
      cmdp = cmdbuf + 5;
      return(PULSEcmd(cmdp));
   } 
   else if( strncmp(cmdbuf, "FRAME", 5) == 0 )
   {
      cmdp = cmdbuf + 5;
      return(FRAMEcmd(cmdp));
   }
   else if( strncmp(cmdbuf, "LIN", 3) == 0 )
   {
      bright_mode = LINEAR;
   }
   else if( strncmp(cmdbuf, "LOG", 3) == 0 )
   {
      bright_mode = LOG;
   }  
   else if ( strncmp(cmdbuf, "CAL ON", 6) == 0 )
   {
       EEPROM.write(0, 0x55);
       return(0);
   }
   else if ( strncmp(cmdbuf, "CAL OFF", 7) == 0 )
   {
       EEPROM.write(0, 0xff);
       return(0);
   }
   else if ( strncmp(cmdbuf, "CAL ?", 5) == 0 )
   {   if( EEPROM.read(0) == 0x55 )
         Serial.print("Cal on: ");
       else
         Serial.print("Cal off: ");   
       Serial.println(EEPROM_readFloat(1));
       return(0);
   }
   else if ( strncmp(cmdbuf, "CAL", 3) == 0 )
   {
      cmdp = cmdbuf + 3;
//      Serial.println(cmdp);
      while ( isdigit(*cmdp) == 0 )
      {
         if( *cmdp == '\0') 
           return -1;
         else  
           cmdp++;  // get past any non-digit         
      }     
      EEPROM_writeFloat(1, atof(cmdp) );
      EEPROM.write(0, 0x55); 
      return 0;
   }   
   else if( strncmp(cmdbuf, "???", 3) == 0 )
   {
      Serial.print("Fly Bowl interface ");
      Serial.println(VERSION);
      #ifdef USE_EXT_TRIG
          Serial.println("Ext Trig Enabled");
      #endif    

    if( bright_mode == LOG ) //#ifdef USE_SCALING 
      Serial.print("Log brightness, ");
    else  //#else
      Serial.print("Linear brightness, ");
    //#endif

    #ifdef LINEAR_DRIVER 
      Serial.println("linear drive");
    #else
      Serial.println("Switched drive");
    #endif
      Serial.println(pulse_on_msec);
      Serial.println(pulse_off_msec);
      Serial.println(pulse_count);
      Serial.println(off_time_msec);
      Serial.println(wait_sec);     
      
      return(0);
   }   
   else
   {
     Serial.print(cmdbuf);
     return -1;
   }
   return 0;   
}

// =========================
// === P U L S E   I N T ===
// =========================

// Timer 1 Interrupt 
// 1 msec timer

void PulseInt(void)
{  
   if( flag )
   {
     pulse_timer--;
     if( pulse_timer <= 0 )
     {
        if( ledon )
        {
          SetLEDs(setting);  //0xffff);
 //         digitalWrite(TESTPIN,0 );       
        }  
        else
        {
           SetLEDs(0x0000);  
        }
        pulse_timer = next_timer;
        flag = 0;
     }   
   }  
}  

// ====================
// === M A R K E R ====
// ====================
// turn marker pin on/off
// a define is used to determine if 'on' is active low

void marker(uint8_t state)
{
#ifdef MARKER_ACTIVE_LOW
  if( state  )
  {
     digitalWrite(MARKERPIN, LOW);
  }    
  else
  {
     if( (pulse_state != PULSEON) && (pulse_state != PULSEOFF) )
     {
          digitalWrite(MARKERPIN, HIGH);
     }     
  }   
#else
  if( state )
  {
     digitalWrite(MARKERPIN, HIGH);
     
  }   
  else
  { 
      if( (pulse_state != PULSEON) && (pulse_state != PULSEOFF) )
      {
        digitalWrite(MARKERPIN, LOW);
      }   
  }   
#endif
}

// ==================
// === S E T U P ====
// ==================

void setup()
{
//int pin; 
  
  Wire.begin();
 
  Serial.begin(115200);
  
   digitalWrite(PIN00, LOW);
   digitalWrite(PIN01, LOW);  
   digitalWrite(PIN02, LOW);
   digitalWrite(PIN03, LOW);      
   digitalWrite(PIN10, LOW);
   digitalWrite(PIN11, LOW);  
   digitalWrite(PIN12, LOW);
   digitalWrite(PIN13, LOW);    
   digitalWrite(PIN20, LOW);
   digitalWrite(PIN21, LOW);  
   digitalWrite(PIN22, LOW);
   digitalWrite(PIN23, LOW);      
   digitalWrite(PIN30, LOW);
   digitalWrite(PIN31, LOW);  
   digitalWrite(PIN32, LOW);
   digitalWrite(PIN33, LOW);       
   
   marker(0); // marker is initailly off
   
   pinMode(PIN00, OUTPUT);
   pinMode(PIN01, OUTPUT);
   pinMode(PIN02, OUTPUT);
   pinMode(PIN03, OUTPUT);
   pinMode(PIN10, OUTPUT);
   pinMode(PIN11, OUTPUT);
   pinMode(PIN12, OUTPUT);
   pinMode(PIN13, OUTPUT);  
   pinMode(PIN20, OUTPUT);
   pinMode(PIN21, OUTPUT);
   pinMode(PIN22, OUTPUT);   
   pinMode(PIN23, OUTPUT);
   pinMode(PIN30, OUTPUT);
   pinMode(PIN31, OUTPUT);
   pinMode(PIN32, OUTPUT);  
   pinMode(PIN33, OUTPUT);  
   
   pinMode(TESTPIN, OUTPUT);
   digitalWrite(TESTPIN,0);
   
   pinMode(MARKERPIN, OUTPUT);
   
   LEDsOff() ; // all off
   setting = 0xffff;    // assume all enabled
   
   
   cbp = 0;

   ext_trig_en = 0;  // start with external trigger disabled
      
//   pinMode( ext_trig_pin, INPUT);
  
//   trig_status = digitalRead(ext_trig_pin);

//  dacv = 1200; // .75V

  pulse_on_msec = 20;
  pulse_off_msec = 100;
  off_time_msec = 300;
  pulse_count = 10;

  pulse_counter = pulse_count;
  pulse_enabled = 0;
  
  ledon = 0;
 
  flag = 0;  // don't test anthing in timer interrupt  
  
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(PulseInt);
  
 // pinMode( alarmPin, OUTPUT);    
  
  // camera sync
//  Timer3.initialize(33333);  // 30 Hz 
//  Timer3.pwm(alarmPin, 900); // ~10% on time
//  Timer3.stop();
  
  sync_on = 0;
  
///  digitalWrite( alarmPin, LOW); 
//   pinMode( alarmPin, INPUT);    // fake an OC output, normally high
   
}

// ================
// === L O O P ====
// ================

void loop()
{

  if( pulse_enabled )
  { 
            
     if( pulse_state != last_state )
     {
 
#ifdef DEBUG     
        Serial.println(pulse_state);
#endif        
        last_state = pulse_state;
     }       
      
      switch (pulse_state)
      {            
        case START:  // new experiment
           pulse_counter = pulse_count;  // init pulse counter
//           pinMode( alarmPin, OUTPUT); // pull down envelope pin
//           Timer3.start();   // use alarmPin for camera trigger
           frame_on_cnt = frame_on_msec;
           frame_off_cnt = 0;
           sync_on = 1;
           iteration = 0;
           if( wait_sec > 0 )  // wait time request 
           {
             noInterrupts();
             ledon = 0;
             next_timer = 1000;
             flag = 1;             
             interrupts();
             wait_cnt = wait_sec;  // init wait time
             pulse_state = WAIT;    
           }
           else   // no wait - jump to pulses
           {
             noInterrupts();
             ledon = 1;
             flag = 1;
             pulse_timer = 0;  // force update
             next_timer = pulse_on_msec;
             interrupts();
             pulse_state = PULSEON;
           }                   
           break;
     
        case WAIT:     
           if( flag == 0 ) // wait time up
           {
             wait_cnt--;  // count down another second
// Serial.println(wait_cnt);
             if( wait_cnt == 0 )
             {
               noInterrupts();
               ledon = 1;
               flag = 1;
               next_timer = pulse_on_msec;
               interrupts();
               pulse_state = PULSEON;
             } 
             else
             {
               flag = 1;
             }  
           }             
           break;
           
        case PULSEON:  
           if( flag == 0 )  // wait for ON params to 'take'
           {         
             if( pulse_off_msec == 0 ) // continuous on mode?
             {  // if so, we will 'fake' continuous on by staying in on mode for N x pulse_width msecs

                if( off_time_msec != 0 )  // only count down have an off time
                    pulse_counter--;
                
                if( pulse_counter == 0 )  // ok we are done, go to off time
                {  
                  noInterrupts();
                  ledon = 0;
                  flag = 1;
                  next_timer = off_time_msec;
                  interrupts();                
                  pulse_state = OFFTIME;  
                  pulse_counter = pulse_count;    // reset pulse counter for next go round  
                }
                else   // still continuous mode, rerun pulse time 
                {
                  noInterrupts(); 
                  ledon = 1;
                  flag = 1;
                  next_timer = pulse_on_msec;
                  interrupts();
               }            
             }   // endif check if cont on
             else  // not continuous, so go on to pulse off time
             {
               noInterrupts(); 
               ledon = 0;
               flag = 1;
               next_timer = pulse_off_msec;              
               pulse_state = PULSEOFF;  
    //         pinMode( alarmPin, OUTPUT);    // set OC low (value is already LOW from setup() )           
               sync_on = 1;
               interrupts();
             }  
            
           }  
           break;
        case PULSEOFF:
           if( flag == 0 )  // wait for OFF params to 'take'         
           {
             pulse_counter--;
             // if we are still doing a string of pulses OR we have no off time
             // then swing around
             if( pulse_counter > 0 )
             {
               noInterrupts();
               ledon = 1;
               flag = 1;
               next_timer = pulse_on_msec;
               interrupts();                
               pulse_state = PULSEON;                 
             }   
             else  // done counting and need off time
             {
                if( off_time_msec != 0 )
                {
                  noInterrupts();
                  ledon = 0;
                  flag = 1;
                  next_timer = off_time_msec;
                  interrupts();                
                  pulse_state = OFFTIME;  
                  pulse_counter = pulse_count;    // reset pulse counter for next go round  
                }
                else
                {
                  pulse_state = OFFTIME;
                }  
             } // endif counting or offtime
           }             
           break;      
       case OFFTIME:
   
           if( flag == 0 )  // wait for OFF params to 'take'
           {
              iteration++;   // count number of iterations of pulse trains
              if( (iteration_loops == 0) || (iteration < iteration_loops) )  // still have more to do, loop back to pulse on
              {
                 noInterrupts();
                 ledon = 1;
                 flag = 1;
                 next_timer = pulse_on_msec;
                 interrupts();         
                 pulse_counter = pulse_count;         
                 pulse_state = PULSEON;  
              }   
              else
              {
                 pulse_state = HOLD;
              }   
          }
      
          break; 
      
       case HOLD: // single pulse mode leaves us here to hang out until next RUN command
           break;
       
       default:
          break;
      } // endswitch              
      
  } // endif pulses enabled
  else
  {
/*  
    if( onflag) 
        LEDsOn(); // turn on leds   
     else   
        LEDsOff();  // turn off leds    
*/        
     flag = 0;  // don't test anthing in timer interrupt 
     sync_on = 0; 
  //   pinMode( alarmPin, INPUT);  // set pin high by turning it off and using ext pullup 
     frame_on_cnt = frame_on_msec;
     frame_off_cnt = 0;
     sync_on = 1;     
  }  
   
  if( Serial.available() > 0 )
  {
    if( cbp >= (BUFFLEN - 1) )  cbp = 0;
    cmdbuf[cbp] = Serial.read();
    if( cmdbuf[cbp] == '\r' )
    {
        
        cmdbuf[++cbp] = '\0';
//        Serial.println(cmdbuf);
        if( (cmderr = ParseCmd()) < 0 ) 
        {
            Serial.print("cmderr " );
            Serial.println(cmderr);
        }
        else
        {
            Serial.println("ok");
        }    
        cbp = 0;
    }  
    else if(cmdbuf[cbp] >= ' ') //only keep printables
    {
      cbp++;      
    }  
    
  }
   else  
  {
#ifdef USE_EXT_TRIG   
     if( digitalRead(ext_trig_pin) == HIGH)
     {
          LEDsOn();
     } 
     else
     {
          LEDsOff(); 
     }
#endif     
//     if( ext_trig_en == 1 )
//     {
//        i = digitalRead(ext_trig_pin); // get trigger value; a '0' means we want on (power into optocoupler)
//        if(  (i == 0) && (trig_status == 1) ) // was off now on
//        {  
//           trig_status = 0;
//           Serial.println("READY"); 
//        }
//        else if( (i == 1 ) && (trig_status == 0) )  // was on now off
//        {
//           trig_status = 1;
//           Serial.println("DONE");
//        }   
//       
//     }  
  }  
   
}
