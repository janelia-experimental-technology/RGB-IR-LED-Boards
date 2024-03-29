// RGB LED board with individual DAC power quadrant (RGB7x7)
// HHMI Janelia jET SWS 
// 20200728

// processor: Teensy 3.2
// Arduinio IDE: 1.8.310
// 

//  PULSE  a,b,c,d,e, f, C
//    a - pulse width
//    b - pulse period
//    c - # pulses
//    d - off time
//    e - wait time
//    f - number of iterations; 0 = continuous
//    C - color channel ('R', 'G', and/or 'B')
 
//#define DEBUG
#define SYNC_PIN_OUT    // if defined, then SYNC pin is set up as camera trigger out,else it is a trigger in
 
// VERSIONS
//
#define VERSION 20210226

// 20210226 sws
// - extend gain to do all colors (now command is "GAIN color quad value"
 
// 20210223 sws
// - add IRgain for gain control of IR to even out IR backlight if needed

// 20210221 sws
// - add getdacs command to see settings for each quadrant

// 20210214 sws 
// - add watchdog timer; needs modified version of Peter Polidoro's watchdog code to get 'triggered'

// 20210212 sws
// - enables 5-8 were set LOW initially - must be HIGH
// - add pin setup for doutPin

// 20210129 sws
// - add digital board control. 
//    Use pin23 to identify board type (floating is LED, grounded is digital)
//    Use pin22 for chip select of IO expander

// 20210127 sws
// - use analogWrite for markers to allow varying intensity, default to constant on, MARKER cmd to change
//    display current Marker Intensity with MARKER cmd

// 20210117 sws
// - for all on and all off, call ledsOn (Off) with '0' not '0x0f'. The ledsOn(Off) then has the value
//    to turn them all on or off (was 0x0f, now 0xff for 8 'quadrant' setup).
// - don't enable serial2. hasn't been used, and want it for enable 7

// 20201213 sws
// - adding in new form of addOneStep for just duration

// 20201205 sws
// - for colorsSet, check that DAC valeu is less than 0x800 (max allowed) so we can use higher values for 
//      shock and.or aux ouputs
// - save DAC value of 1 for TTL out for shock, so promote a '1' intensity value to '2'

// 20201126 sws
// - add DAC command to set an individual DAC output for testing

// 202011223 sws
// - only send intensity values for quads 1-4 to downstream panels
// - save intensityPC not as percent but as dacvalue and move from newIntensity to setIntensity so
//      that downstream panels save it
// NOTE!!! - we can set intensities for each quadrant, but we only save the last one set (per color)
//   that may cause trouble...

// 20201109 sws
// - add SYNC_PIN_OUT to make sync pin usable as camera trigger out for original RGB board

// 20201029 sws
// - finalize adding in 4 extra chip selects so panel 1 has eight 'quadrants'

// 20201026 sws
// - uee power down at 0% setting. This required a fair amount of change, as when one goes from some brightness to 0
//    or 0% to some value, as power down is also used for Off. Added colorsAreOn boolean to differentiate off from a 
//    0% setting 

// 20201019 sws
// - force SYNC to start when starting experiment
// - only run channels with an on time with RUN command
// - remove default channel setups
// - 

// 20201018 sws
// - allow a 0 sync rate to stop the trigger out, use begin to update or restart the timer

// 20201017 sws
// - when stopping an experiment, need to force markers off. add to HOLD state

// 20201016 sws
// - fixed problem with LED coming on during second step wait time in experiment 

// 20201014 sws
// - add pattern control in Experiment protocol
// - StopExperiment now turns off LEDS and forces state to HOLD

// 20201012 sws
// - added quadrant 'memory' to allow on/off of a pattern (onQuadrants)

// 20200919 sws
//  - wrong index varaible when ending experiment

// 20200910 sws
// - set pulse_counter to 0 at start of experiment to dorec loading of first timer value

// 20200909 sws
// - add elapsed time to getExperimentStatus

// 20200904 sws
// - remove delay secs,msec just use msecs
// - add LED intensity to getexperimentstatus response
// - reset step index to 0 when experiment is done

// 20200903 sws
// - implemented DELAY in experiment 
// - changed duration to seconds

// 20200832 sws
//  - fixed several errors in running multiple steps

// 20200831 sws
//  - iterations was being used as pulse count in experiments
//  - iterations was not being set in experiments 
//  - when epxeriment is not running, getExperimentStatus returned -1, should be 0 

// 20200827 sws
// - experiment commands basically working
// - addExperimentStep returns number of valid steps 

// 20200826 sws
// - added HELP comamd to list comamnds
// - RGB marker LEDs working


// 20200728 sws
// - from RGB_J007017 VER: 20180703 

#include <Cmd.h>
#include <SPI.h>
#include <Watchdog.h>
#include <EEPROM.h>

IntervalTimer pulseTimer;
IntervalTimer syncTimer;

Watchdog watchdog;
    
#define BLUDAC 1
#define GRNDAC 2
#define REDDAC 4
#define IRDAC  8

#define BLUE 0
#define BLU 0
#define GREEN 1
#define GRN 1
#define RED 2
#define IR 3


#define LED_BOARD 1     // if pin23 is floating (existing LED boards) we get this
#define DIGITAL_BOARD 0  // digital boards nust ground pin 23

char colors[4][5] = {"BLU ","GRN ","RED ","IR  "};  // text to show what color number is
uint8_t color2DAC[] = {BLUDAC, GRNDAC, REDDAC, IRDAC}; // translate color number to DAC bit position

#define MAXIRDAC 4095.0    // IR drive can be 0-2.5V (max DAC range)
#define MAXRGBDAC 2047.0   // RGB drive should only go to 1.25V (1/2 DAC range)
 
// --- PINS
//      B2B_RX   0
//      B2B_TX   1
#define DIRpin   2
#define triggerPin  3
#define redPin   4
#define grnPin   5
#define bluPin   6
#define en5Pin   7
#define en6Pin   8
#define en7Pin   9
#define fanPin  10
#define MOSIpin 11
#define SCLKpin 13
#define OTMPpin 14
#define LLpin   15
#define URpin   16
#define LRpin   17
#define ULpin   18
#define SYNCpin 19
#define LEDpin  20
#define en8Pin  21
#define doutPin 22      // cs for digital board
#define boardTypePin 23


#define SYNC_ACTIVE_LOW  // marker pin is active low 

//--- pulse states---
#define OFFSTATE 0
#define START    1
#define WAIT     2
#define PULSEON  3
#define PULSEOFF 4
#define OFFTIME  5
#define CONTINUOUS 6
#define STEPWAIT  7
#define HOLD    8
// ------------------

char states[10][5] = {"off ","strt", "wait","pon ","poff","offt","cont","step","hold"};

#define MINWIDTH     1   //5
#define MAXWIDTH  1800000  //  30 minutes 120000 //30000  //1000  //500
#define MINPERIOD    1   //25
#define MAXPERIOD 1800000  //  30 minutes 120000 //30000  //1000
#define MINPULSES    1
#define MAXPULSES 1000   //40
#define MINOFF       0
#define MAXOFF   1800000  //  30 minutes  120000 //60000  //30000 // 10000
#define MINWAIT      0
#define MAXWAIT    120
#define MINRATE      1
#define MAXRATE   1000
#define MINITERATIONS  0
#define MAXITERATIONS 30000
#define MINSYNC 1
#define MAXSYNC 60

float syncRate = 30;        

int32_t pulse_on_msec[3];
int32_t pulse_off_msec[3];
int32_t off_time_msec[3];
int32_t pulse_count[3];
int32_t wait_sec[3];
int32_t wait_msec[3];
int32_t wait_cnt[3];
int32_t frame_on_msec[3];
int32_t frame_off_msec[3];
uint16_t iteration_loops[3] = {0,0,0};
uint32_t stepDuration;
uint32_t stepDelay;

uint32_t experimentMsecs;

uint8_t bowls =0;

uint16_t intensityPC[4] = {0,0,0,0};

uint16_t iteration[3];

int xIdx = 0;  // index into experiment table

//elapsedMillis pulse_timer;
int32_t pulse_counter[3];
int pulse_enabled[3];

uint16_t frame_on_cnt[3];
uint16_t frame_off_cnt[3];

volatile int32_t next_timer[3];
volatile int8_t flag[3] = {0,0,0};
volatile int8_t ledon[3] = {0,0,0};
volatile int32_t pulse_timer[3];
volatile int8_t  sync_on[3];

boolean markerState[] = {false, false, false};

uint8_t onQuadrants = 0x00;  // track which quadrants are turned on - default to none; bit pattern

uint8_t onColors[] = {0x00, 0x00, 0x00 };  // which colors are set to 0% - Blue, Green, Red , bit pattern for quadrants (up to 8)

boolean colorsAreOn = false;

int pulse_set[3] = {0,0,0};
int pulse_state[3] = {START, START, START };
int last_state[3] = { HOLD, HOLD, HOLD };
boolean markerPin[3] = { bluPin, grnPin, redPin };

int8_t err = 0;

//volatile uint16_t pattern[3] = { 0, 0 , 0 };
volatile uint16_t pattern =  0;

static uint8_t inited = 0;
static uint8_t thisPanel = 0;
// intensity values for each color [4] by number of quads (was 4 now 8)
uint16_t intensity[4][8] =
    {0, 0, 0, 0, 0, 0, 0, 0, 
     0, 0, 0, 0, 0, 0, 0, 0, 
     0, 0, 0, 0, 0, 0, 0, 0, 
     0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t intensityLON = 0;
uint16_t intensityMON = 0;
uint16_t intensityHON = 0;

uint8_t markerIntensity = 255;

float LEDgain[4][4] = 
{
   1.0, 1.0, 1.0, 1.0,  // blue
   1.0, 1.0, 1.0, 1.0,  // green
   1.0, 1.0, 1.0, 1.0,  // red
   1.0, 1.0, 1.0, 1.0   // IR   
};
int EE_INITED = 64;  // address of init code - 4 * 4 * 4 bytes used for gains

boolean bowlEnabled[4] = {false, false, false, false };

boolean wdtReset = false;
          
SPISettings DAC(2000000, MSBFIRST, SPI_MODE2);  
SPISettings DOUT(1000000, MSBFIRST, SPI_MODE0);

float readFloat(unsigned int addr)
{
  union
 {
   byte b[4];
  float f;
 }  data;
 for(int i = 0; i < 4; i++)
 {
  data.b[i] = EEPROM.read(addr+i);
 }
 return data.f;
}

void writeFloat(unsigned int addr, float x)
{
  union
  {
   byte b[4];
   float f;
  } data;
  data.f = x;
  for(int i = 0; i < 4; i++)
  {
    EEPROM.write(addr+i, data.b[i]);
  }
}
 

// =========== S E R I A L    C O M M A N D S =========

#define SETOFF      0x00
#define SETON       0x40
#define SETCOLOR    0x80
#define INTENSITYLON    0xC0
#define INTENSITYMON    0xD0
#define INTENSITYHON    0xE0
#define ENUMERATE1   0xF1  
#define ENUMERATE2   0xF2  
#define ENUMERATE3   0xF3
#define SERIES      0xF4
#define PARALLEL    0xF5
#define COLORSET    0xF8

#define PANELBITS  0x30
#define COLORBITS  0x03
#define QUADBITS   0x0c

#define ALLPANELS 0x00
#define ALLQUADS  0x0f

// ===========================
// ===  C O L O R   S E T  ===
// ===========================

// Turn on selected colors, turn others off

void colorSet(int8_t colors)
{  

#ifdef DEBUG
  Serial.print("COLORSET ");
  Serial.println(colors);
#endif

   // First turn everything (but IR) off
   SPI.beginTransaction(DAC);
   digitalWriteFast(ULpin,LOW);  // select quadrant DACs
   digitalWriteFast(URpin,LOW);    
   digitalWriteFast(LRpin,LOW);    
   digitalWriteFast(LLpin,LOW);      
   digitalWriteFast(en5Pin, LOW);
   digitalWriteFast(en6Pin, LOW);
   digitalWriteFast(en7Pin, LOW);   
   digitalWriteFast(en8Pin, LOW); 
   SPI.transfer(0x20);            // pwr up/down command
   SPI.transfer(0x00); 
   SPI.transfer(0x17);            // power down RGB with 1K to GND, but not IR
   delayMicroseconds(1);
   digitalWriteFast(ULpin,HIGH);  // stop transmitting   
   digitalWriteFast(URpin,HIGH);       
   digitalWriteFast(LLpin,HIGH);    
   digitalWriteFast(LRpin,HIGH);
   digitalWriteFast(en5Pin, HIGH);
   digitalWriteFast(en6Pin, HIGH);
   digitalWriteFast(en7Pin, HIGH);
   digitalWriteFast(en8Pin, HIGH);      
   SPI.endTransaction();
   colorsAreOn = false;  // assume we want them off

   if( colors != 0 )  // only turn colors on if any are asked for
   {
       uint8_t colorOn = 0x00;    
       colors |= 0x08; // always have IR on
       // now set the selected quadrants on but only if the brightness is > 0.01%    
       SPI.beginTransaction(DAC);
       // do on colors
       if( onQuadrants & 0x01 ) digitalWriteFast(ULpin,LOW);  // select quadrant DACs
       if( onQuadrants & 0x02 ) digitalWriteFast(URpin,LOW);    
       if( onQuadrants & 0x08 ) digitalWriteFast(LRpin,LOW);    
       if( onQuadrants & 0x04 ) digitalWriteFast(LLpin,LOW);      
       if( onQuadrants & 0x10 ) digitalWriteFast(en5Pin, LOW);
       if( onQuadrants & 0x20 ) digitalWriteFast(en6Pin, LOW);
       if( onQuadrants & 0x40 ) digitalWriteFast(en7Pin, LOW);   
       if( onQuadrants & 0x80 ) digitalWriteFast(en8Pin, LOW);
       delayMicroseconds(1);
       SPI.transfer(0x20);           // pwr up/down command
       SPI.transfer(0x00);       
       if( (intensityPC[RED] > 1) && (intensityPC[RED] < 0x800) ) colorOn |= 0x04;  // > 0x7ff fro colods is aux
       if( (intensityPC[BLU] > 1) && (intensityPC[BLU] < 0x800) ) colorOn |= 0x01;
       if( (intensityPC[GRN] > 1) && (intensityPC[GRN] < 0x800) ) colorOn |= 0x02;   
       if( intensityPC[IR]  > 1 ) colorOn |= 0x08;   
       colorOn = colorOn & colors;  // only enable desired colors that have intensity set
#ifdef DEBUG
  Serial.print(" on ");
  Serial.println(colorOn);
#endif          
       SPI.transfer(colorOn); 
       delayMicroseconds(1); 
       digitalWriteFast(ULpin,HIGH); // stop transmitting   
       digitalWriteFast(URpin,HIGH);     
       digitalWriteFast(LLpin,HIGH);     
       digitalWriteFast(LRpin,HIGH); 
       digitalWriteFast(en5Pin, HIGH);
       digitalWriteFast(en6Pin, HIGH);
       digitalWriteFast(en7Pin, HIGH);
       digitalWriteFast(en8Pin, HIGH);       
       SPI.endTransaction();

       colorsAreOn = true;
   } 
}


// ========================
// ===  L E D S   O N   ===
// ========================

void ledsOn(uint8_t quadrant)
{  
  
#ifdef DEBUG  
  Serial.print(millis()); 
  Serial.print(" LEDSON q:");  
  Serial.print(quadrant, HEX);  
  Serial.println(" on"); 
  Serial.println(intensityPC[RED]);
#endif  


   if( quadrant == 0 ) quadrant = 0xff;  

//   if(quadrant == 0) quadrant = 0x0f; // 0 is all

   onQuadrants = quadrant;  

   colorSet(0x07); // turn on selected quadrants; all colors > 0%
}

// ==========================
// ===  L E D S   O F F   ===
// ==========================

void ledsOff(int8_t quadrant)
{    
#ifdef DEBUG  
  Serial.print(millis()); 
  Serial.print(" LEDSOFF q:");  
  Serial.print(quadrant, HEX);  
  Serial.println(" off");  
#endif  

   if( quadrant == 0 ) quadrant = 0xff;  

//  if(quadrant == 0) quadrant = 0x0f; // 0 is all
   
   colorSet(0x00);  // turn off all colors 

}


// ===================================
// ===  A L L   P A N E L S  O N   ===
// ===================================

void allPanelsOn(void)  // for now ignore rgb 
{
 //   sendCmd(SETON | ALLPANELS | ALLQUADS);
    sendCmd(COLORSET | 0x07 );
    ledsOn(0);  // all on
}

// =====================================
// ===  A L L   P A N E L S  O F F   ===
// =====================================

void allPanelsOff(void)  // for now ignore rgb
{
//    sendCmd(SETOFF | ALLPANELS | ALLQUADS);
    sendCmd(COLORSET);
    ledsOff(0);  // all off   
}

// ====================================================================================
// === M C P 2 3 S 1 7    I N T E R F A C E   ( D I G I T A L   O U T   B O A R D ) ===
// ====================================================================================

#define DIRA_REG  0x00
#define DIRB_REG  0x01
#define OUTA_REG  0x12
#define OUTB_REG  0x13
#define CONFG_REG 0x0A
#define MCP23S17_ADR 0x40
#define MCP23S17_RD  0x41
#define MCP23S17_WR  0x40


// ===========================
// ===  W R I T E   R E G  ===
// ===========================

void writeReg(uint8_t reg, uint8_t data )
{
    SPI.beginTransaction(DOUT);
    digitalWrite( doutPin, LOW);
    SPI.transfer(MCP23S17_WR);
    SPI.transfer(reg);
    SPI.transfer(data);
    digitalWrite( doutPin, HIGH);
    SPI.endTransaction();
}


// ===========================
// ===  I N I T   D O U T  ===
// ===========================


void initDOUT( void)
{
    writeReg(CONFG_REG,0x78);  // 0111 1000 
    writeReg(DIRA_REG, 0x00);  // all outputs
    writeReg(DIRB_REG, 0x00); 
    writeReg(OUTA_REG, 0x00);  // all set low
    writeReg(OUTB_REG, 0x00);
}

// ==== END MCP32S17 ==========


// ===========================
// ===  I N I T   D A C S  ===
// ===========================

void initDACs( void)
{
   // sw reset
   SPI.beginTransaction(DAC);
   digitalWriteFast(ULpin,LOW);   // select quadrant DACs
   digitalWriteFast(URpin,LOW);    
   digitalWriteFast(LRpin,LOW);    
   digitalWriteFast(LLpin,LOW);  
   digitalWriteFast(en5Pin, LOW);
   digitalWriteFast(en6Pin, LOW);
   digitalWriteFast(en7Pin, LOW);   
   digitalWriteFast(en8Pin, LOW);    
   SPI.transfer(0x28);            // sw power-on reset
   SPI.transfer16(0x0001);   
   delayMicroseconds(1);   
   digitalWriteFast(ULpin,HIGH);  // stop transmitting   
   digitalWriteFast(URpin,HIGH);       
   digitalWriteFast(LLpin,HIGH);    
   digitalWriteFast(LRpin,HIGH); 
   digitalWriteFast(en5Pin, HIGH);
   digitalWriteFast(en6Pin, HIGH);
   digitalWriteFast(en7Pin, HIGH);
   digitalWriteFast(en8Pin, HIGH);     
   SPI.endTransaction();
  
   // turn on internal references on DACs
   SPI.beginTransaction(DAC);
   digitalWriteFast(ULpin,LOW);   // select quadrant DACs
   digitalWriteFast(URpin,LOW);    
   digitalWriteFast(LRpin,LOW);    
   digitalWriteFast(LLpin,LOW); 
   digitalWriteFast(en5Pin, LOW);
   digitalWriteFast(en6Pin, LOW);
   digitalWriteFast(en7Pin, LOW);   
   digitalWriteFast(en8Pin, LOW);        
   SPI.transfer(0x38);            // enable reference  xx11 1xxx xxxx xxx1
   SPI.transfer16(0x0001);  
   delayMicroseconds(1);    
   digitalWriteFast(ULpin,HIGH);  // stop transmitting   
   digitalWriteFast(URpin,HIGH);       
   digitalWriteFast(LLpin,HIGH);    
   digitalWriteFast(LRpin,HIGH); 
   digitalWriteFast(en5Pin, HIGH);
   digitalWriteFast(en6Pin, HIGH);
   digitalWriteFast(en7Pin, HIGH);
   digitalWriteFast(en8Pin, HIGH);   
   SPI.endTransaction();

    
   // update DAC when new DAC value is received
   SPI.beginTransaction(DAC);
   digitalWriteFast(ULpin,LOW);   // select quadrant DACs
   digitalWriteFast(URpin,LOW);    
   digitalWriteFast(LRpin,LOW);    
   digitalWriteFast(LLpin,LOW); 
   digitalWriteFast(en5Pin, LOW);
   digitalWriteFast(en6Pin, LOW);
   digitalWriteFast(en7Pin, LOW);   
   digitalWriteFast(en8Pin, LOW);        
   SPI.transfer(0x30);            // update DAC 
   SPI.transfer16(0x000f);        // on load
   delayMicroseconds(1);   
   digitalWriteFast(ULpin,HIGH);  // stop transmitting   
   digitalWriteFast(URpin,HIGH);       
   digitalWriteFast(LLpin,HIGH);    
   digitalWriteFast(LRpin,HIGH); 
   digitalWriteFast(en5Pin, HIGH);
   digitalWriteFast(en6Pin, HIGH);
   digitalWriteFast(en7Pin, HIGH);
   digitalWriteFast(en8Pin, HIGH);   
   SPI.endTransaction();
}


// =============================
// ===  C H E C K   I N I T  ===
// ==============================

void checkInit(void)
{
  if( inited == 0 )
  {  
     initBoards();
  }  
}

// =============================================================================================
// ================================== H O S T   C O M M A N D S ================================
// =============================================================================================
// --- command structure
//  - Host computer sends ASCII commands to set intensity, pulse setup, on, off, and pulse control to
//       the main panel. That panel acts on commands for itself and sends commands on to the other three 
//       downstream panels if needed 
//

int8_t quad2cspin[] = {ULpin, URpin, LRpin, LLpin, en5Pin, en6Pin, en7Pin, en8Pin };

// ====================================
// ===  S E T   I N T E N S I T Y   ===
// ====================================

// set new color for one quadrant: 0-3 now 0-7
// if needing to change all quadrants, need to call four times

void setIntensity(uint8_t color, uint8_t quadrant, uint16_t DACvalue)
{

   uint8_t cspin = quad2cspin[quadrant];
   uint8_t colorBit = color2DAC[color];

   intensityPC[color] = DACvalue;
   
#ifdef DEBUG
 Serial.print("SETINTENSITY q");
 Serial.print( quadrant, HEX);
 Serial.print(" c");
 Serial.print(color, HEX);
 Serial.print(" i");
 Serial.print(intensity[color][quadrant]);
 Serial.print(" cs");
 Serial.print(cspin, HEX);
 Serial.print(" ad");
 Serial.print( colorBit);
 Serial.print(" DAC 0x");
 Serial.println(DACvalue, HEX);
#endif

   if( digitalReadFast(boardTypePin) == LED_BOARD )
   {
     if( quadrant < 4 )
        DACvalue = int( (float)DACvalue * LEDgain[color][quadrant] );
     if( color == IR) 
     {
        if ( DACvalue > MAXIRDAC ) DACvalue = MAXIRDAC;
     }
     else
     {
         if ( DACvalue > MAXRGBDAC ) DACvalue = MAXRGBDAC;
     }
 
     SPI.beginTransaction(DAC);
     digitalWriteFast( cspin,LOW);    
     SPI.transfer( 0x18 | color);   // write and update DAC xx01 1aaa                     
     SPI.transfer16( DACvalue << 4);    
     delayMicroseconds(1);      
     digitalWriteFast( cspin,HIGH);    // stop transmitting   
     SPI.endTransaction();
    
  
     if( DACvalue == 0 )  // if we set to 0% , use power down to turn off
     {
  #ifdef DEBUG
   Serial.println("turn off");
  #endif    
         SPI.beginTransaction(DAC);
         digitalWriteFast( cspin,LOW);    
         SPI.transfer(0x20);            // pwr up/down command
         SPI.transfer(0x00); 
         SPI.transfer(0x10 | colorBit);  // power down this color
         delayMicroseconds(1);       
         digitalWriteFast( cspin,HIGH);  // stop transmitting   
         SPI.endTransaction();
     }
     else
     {  
        uint8_t quad = (1 << quadrant);
        boolean turnOn = false;  
        // if it was zero, and it is enabled, we need to turn it on , OR if IR, always turn on            
        if( (intensity[color][quadrant] == 0) && (onQuadrants & quad) && (colorsAreOn == true) ) turnOn = true;
        if( color == IR ) turnOn = true;   
        if( turnOn) 
        {
  #ifdef DEBUG
   Serial.print("turn on ");
   Serial.println(colorsAreOn);
  #endif           
           SPI.beginTransaction(DAC);
           digitalWriteFast( cspin,LOW);    
           SPI.transfer(0x20);             // pwr up/down command
           SPI.transfer(0x00); 
           SPI.transfer(colorBit);         // power up this color
           delayMicroseconds(1);         
           digitalWriteFast( cspin,HIGH);  // stop transmitting   
           SPI.endTransaction();
        }     
     }  
   } // end LED board 
   else
   {
      setDigitalOut(color, quadrant, DACvalue);       
   }
   intensity[color][quadrant] = DACvalue;  // update saved DACvalue
}



// =====================================
// ===  N E W    I N T E N S I T Y  ===
// =====================================

// set and/or send out new RGB intensity 

void newIntensity( float percent, uint8_t color, uint8_t panel, uint8_t quadrant)
{     
uint16_t DACval;

//    intensityPC[color] = percent;
//    if( percent >= 0 )  // actual brightness, not digital
//    {
      if (color == IR ) 
      { 
        DACval = (uint16_t) ( (percent * MAXIRDAC) / 100.0);
        if( DACval > MAXIRDAC ) DACval = MAXIRDAC;
      }
      else
      {
         DACval = (uint16_t) ( (percent * MAXRGBDAC) / 100.0);
         if( DACval > MAXRGBDAC ) DACval = MAXRGBDAC;
      } 
//      if( DACval == 1 ) DACval = 2;  // reserve 1 for digital out
//    }
//    else
//    {
//       DACval = 1; // digital setup
//    }
#ifdef DEBUG   
    Serial.print(colors[color]);
    Serial.print(" Intensity %,panel,quad,DAC: ");  
    Serial.print(percent);
    Serial.print(", ");
    Serial.print(panel, HEX);
    Serial.print(", ");
    Serial.print(quadrant, HEX);
    Serial.print(", ");
    Serial.println(DACval, BIN);   
    Serial.print("onQuads=");
    Serial.println(onQuadrants,BIN);
#endif
    sendCmd(INTENSITYHON | ((DACval & 0x0f00)>>8) ); 
    sendCmd(INTENSITYMON | ((DACval & 0x00f0)>>4) ); 
    sendCmd(INTENSITYLON | ( DACval & 0x000f));
 
    for( uint8_t quad = 0; quad < 8; quad++)  // host can have 8 'quadrants'
    {
      if( quadrant & (1 << quad))  // check each  quadrant
      {
          if( (panel != 1) && (quad < 4) ) // don't relay commands that are only for host panel (#1)
          {                                // also, remote panels can only have 4 quadrants
             uint8_t tpanel = panel;  
             if( tpanel != 0 ) tpanel--;  // translate 2-4 to 1-3 for remote cmds
             sendCmd(SETCOLOR | (tpanel << 4) | (quad << 2) | color);
          }        
          if( panel <= 1 )  // host panel or all - so do local update
          {

             // if( digitalReadFast(boardTypePin) == LED_BOARD )
                  setIntensity(color, quad, DACval); 
            //  else
               //   setDigitalOut(color, quad, DACval);    
          }  
      }     
    }  // next quadrant 
} 

// ======================================
// ===  S E T   D I G I T A L   O U T ===
// ======================================


void setDigitalOut( uint8_t color, uint8_t quadrant, uint16_t DACvalue)
{
static uint8_t BGbits = 0;
static uint8_t RIbits = 0;

  uint8_t quad = 1 << quadrant; // get bit position
   
  if( (color == GRN) || (color == IR) )  // upper nibble colors
      quad = quad << 4;
      
  if( (color == BLU) || (color == GRN) )
  {
     if( DACvalue > 0 ) 
        BGbits |= quad;
     else
        BGbits &= ~quad;  
     writeReg( OUTA_REG, BGbits);    
  }
  else
  {  
     if( DACvalue > 0 ) 
        RIbits |= quad;
     else
        RIbits &= ~quad; 
     writeReg( OUTB_REG, RIbits);    
  }    
  
}

// =======================
// ===  D A C   C M D  ===
// =======================

// set a DAC to a value
// DAC color quadrant value

 void DACCmd( int arg_cnt, char **args)
 {    

  uint8_t quadrant; 
  uint8_t color;
  uint16_t DACval;
  
 
    if( arg_cnt > 3 )  
    {
      if( strchr(args[1], 'R') )   
      {
        color = RED;;      
      }
      else if( strchr(args[1], 'B') ) 
      {
        color = BLUE;       
      }
      else if (strchr(args[1], 'G') ) 
      {
        color = GREEN;        
      }     
      else 
      {
         return;
      }  
        
      quadrant = cmdStr2Num(args[2], 2);  
      
      DACval = cmdStr2Num(args[3], 10);

      if( digitalReadFast( boardTypePin) == LED_BOARD)
      {
         setIntensity( color, quadrant, DACval);
      }
      else
      {
         setDigitalOut( color, quadrant, DACval);
      }
    }

 }


// ==================================
// ===  D O    I N T E N S I T Y  ===
// ==================================

 int8_t doIntensity( int arg_cnt, char **args, uint8_t color)
 {    
  // assume we want it all
  uint8_t quadrant = 0xff; // assume all quads
  uint8_t panel = 0;       // assume all panels
  float percent = 0;
//  uint16_t DACval;
  
    checkInit(); 
  
    if( arg_cnt > 1 )  // then we have an intensity
    {
//       if( strchr( args[1], 'D') != NULL )
//       {
//          percent = -1;  // flag if digtal setup
//       }
//       else
//       {
//          percent = cmdStr2Float(args[1]);
//       } 
  
       percent = cmdStr2Float(args[1]); 

       if( arg_cnt > 2 )  // then we have a panel
       {
          panel = cmdStr2Num(args[2], 10);  
          if( arg_cnt > 3 )  // then we have a quadrant
          {
             uint8_t tquad = cmdStr2Num(args[3], 2);
         //    if( tquad < 0x10 ) 
                quadrant = tquad;
         //    else
         //        return -3;    
          }  // endif quadrants chosen    
       }  // endif panel chosen   
       newIntensity( percent, color, panel, quadrant);
           
    } // endif percent 
    else 
    {
       return -1;      
    }

 //   newIntensity( percent, color, panel, quadrant);

    return 0;
 }



// =============================
// ===  S E T   O N   O F F  ===
// =============================   

// does the actual comamnds to turn quadrants on/off
// panel: 0 = all or 1-4  
// quadrant: bit pattern 0000 - 1111
// onoff: 0 = off, 1 = on

void setOnOff( uint8_t panel, uint8_t quadrant, uint8_t onoff)
{
#ifdef DEBUG
    Serial.print("panel:");
    Serial.print(panel);
    Serial.print("  quad:");
    Serial.print(quadrant, BIN);
    if( onoff)
      Serial.println(" ON");
    else
      Serial.println(" OFF");   
#endif
 
    if (onoff == 0 )  // ---- 0 = 'off' command
    {

        if( panel != 1 ) // only relay commands that include non-host panels
        {
            uint8_t tpanel = panel;  
            if( tpanel != 0 ) tpanel--;  // translate 2-4 to 1-3 for remote cmds
            sendCmd(SETOFF | (tpanel << 4) | quadrant);
        }
        if( panel <= 1 ) ledsOff(quadrant);  // 0 or 1 includes main panel
      
    }    
    else   //------ otherwise 'on' command
    {
     
        if( panel != 1 ) //  only relay commands that include non-host panels
        {
            uint8_t tpanel = panel;  
            if( tpanel != 0 ) tpanel--;  // translate 2-4 to 1-3 for remote cmds
            sendCmd(SETON | (tpanel << 4) | quadrant);
        }
        if( panel <= 1 ) ledsOn(quadrant);    // 0 or 1 includes main panel         
    }    
}

// ===========================
// ===  D O   O N   O F F  ===
// ===========================

// parse and setup to turn quadrant(s) on panel(s) on or off
// format:  panel, quad   - panel -> 0 = all, 1 = main, then 2,3, 4  quad -> bit pattern 0000 -1111

int8_t doOnOff(int arg_cnt, char **args, uint8_t onoff)
{
  // assume we want it all
  uint8_t quadrant = 0xff; // assume all quads
  uint8_t panel = 0;
  
    checkInit();

#ifdef DEBUG
    Stream *s = cmdGetStream();
    s->print("\r\n DOONOFF Parsed: c: ");
    s->print(arg_cnt);
    s->print(", p ");
#endif  

    if( arg_cnt > 1 )
    {
       uint8_t tpanel = cmdStr2Num(args[1], 10); 
       
#ifdef DEBUG       
       s->print(tpanel); 
#endif  

       if( tpanel < 5 )  // user sees panels as 0 = all, 1 is host, then 2,3,4
          panel = tpanel;  
       else
          return -2; 
          
 #ifdef DEBUG             
       s->print(", q "); 
 #endif
       
       if( arg_cnt > 2 )
       {
          //uint8_t tquad = cmdStr2Num(args[2], 2); 
          uint8_t tquad = 0;
          char * qp = args[2];
          while( (*qp == '1') || (*qp == '0') )
          {
            //  s->print(*qp);
              tquad = tquad << 1;
              if( *qp == '1') tquad += 1;
              qp++;           
          }
#ifdef DEBUG            
          s->println(tquad, HEX);
#endif          
 //         if( tquad < 0x10 ) 
              quadrant = tquad;
//          else
//              return -3;    
       }                         
    }
    
#ifdef DEBUG  
     s->print("pan=");
     s->println(panel);
#endif

    setOnOff(panel, quadrant, onoff);

    return 0;
}

// =======================
// ===  R E D   C M D  ===
// =======================

void redCmd(int arg_cnt, char **args) 
{
  //  Serial.println("red");
    doIntensity(arg_cnt, args, RED);
}

// ===========================
// ===  G R E E N   C M D  ===
// ===========================

void greenCmd(int arg_cnt, char **args)
{
    doIntensity(arg_cnt, args, GRN);
}

// ========================
// === B L U E   C M D  ===
// ========================

void blueCmd(int arg_cnt, char **args) 
{
   doIntensity(arg_cnt, args, BLU);
}

// ====================
// ===  IR   C M D  ===
// ====================

void irCmd(int arg_cnt, char **args) 
{
   doIntensity(arg_cnt, args, IR);
}

// ============================
// ===  G E T D A C S    C M D  ===
// ============================

void getdacsCmd(int arg_cnt, char **args) 
{
    Stream *s = cmdGetStream();
    s->print("RED :");
    for( int i = 0; i < 8; i++ )
    {
        s->print(intensity[RED][i]);
        s->print(" ");
    }
    s->print("\r\nGRN :");
    for( int i = 0; i < 8; i++ )
    {
        s->print(intensity[GRN][i]);
        s->print(" ");
    }   
    s->print("\r\nBLU :");
    for( int i = 0; i < 8; i++ )
    {
        s->print(intensity[BLU][i]);
        s->print(" ");
    } 
    s->print("\r\nIR  :");
    for( int i = 0; i < 8; i++ )
    {
        s->print(intensity[IR][i]);
        s->print(" ");
    }
    s->println();   
}

// =========================
// ===  G A I N   C M D  ===
// =========================

void gainCmd(int arg_cnt, char **args) 
{
    Stream *s = cmdGetStream();
    int clr;

//    s->println( arg_cnt);
//    s->println( cmdStr2Num(args[1], 10) );
//    s->println( cmdStr2Float(args[2]) );
    
    if( arg_cnt == 4)
    {
       if( args[1][0] == 'B' ) clr = BLU;
       else if (args[1][0] == 'G') clr = GRN;
       else if (args[1][0] == 'R') clr = RED;
       else if (args[1][0] == 'I') clr = IR;
       else 
       { 
          s->println("color?"); 
          return;
       }
      
       int idx = cmdStr2Num(args[2], 10);
       if( (idx >= 0) && (idx <= 3) )
       {
          float tgain = cmdStr2Float(args[3]);
          if( (tgain >= 0.8) && (tgain <= 1.2) )
          {
//    s->println( clr * 16 + idx * 4);
//    s->println( tgain); 
             writeFloat( clr * 16 + idx * 4, tgain);  // 4 bytes/float, 4 quads per color 
             LEDgain[clr][idx] = tgain;
//    s->println( LEDgain[clr][idx] );
          }
          else
          {
              s->println("gain?"); 
              return;           
          }
       }   
       else
       {
          s->println("quad?"); 
          return;
       }

       
    }
    else  // show gains
    {
      for ( int color = 0; color < 4; color++)
      {
         s->println();
         s->print(colors[color]);
      
         for ( int quad = 0; quad  < 4; quad++ )
         {
            s->print(" ");
            s->print( LEDgain[color][quad] ); 
         }        
      } 
      s->println();  
    }
}    

// =====================
// ===  O N   C M D  ===
// =====================

// turn on quadrants
// format: ON pan quad   - panel -> 0 = all, 1 = main, then 2,3, 4  quad -> bit pattern 0000 -1111
// if no parameters, then all on


void onCmd(int arg_cnt, char **args) 
{
    if( arg_cnt > 1) 
        doOnOff(arg_cnt, args, 1);
    else
        allPanelsOn();
        //doPattern(1);    
}


// ======================
// === O F F   C M D  ===
// ======================

void offCmd(int arg_cnt, char **args) 
{
    if( arg_cnt > 1)
        doOnOff(arg_cnt, args, 0);
    else
        allPanelsOff();
        //doPattern(0);    
}

// =======================
// ===  R U N   C M D  ===
// =======================

void runCmd(int arg_cnt, char **args) 
{

#ifdef DEBUG
    Serial.println("RUN");
#endif
      
   if( arg_cnt > 1 )
   {
      if( strchr(args[1], 'R') ) 
      {
        pulse_enabled[RED] = 1;
        pulse_state[RED] = START;       
      }
      if( strchr(args[1], 'B') ) 
      {
        pulse_enabled[BLUE] = 1;
        pulse_state[BLUE] = START;       
      }
      if( strchr(args[1], 'G') ) 
      {
        pulse_enabled[GREEN] = 1;
        pulse_state[GREEN] = START;       
      }     
   }
   else
   {
     for( int i = 0; i < 3; i++)
     {
        if( pulse_set[i] > 0 )  // only run channels with an on time
        {
           pulse_enabled[i] = 1;
           pulse_state[i] = START;
        }   
     }   
   }  
}

// =========================
// ===  S T O P   C M D  ===
// ========================

void stopCmd(int arg_cnt, char **args) 
{

  digitalWrite(LEDpin, LOW);
  
  if( arg_cnt > 1 )
   {
      if( strchr(args[1], 'R') ) 
      {
        pulse_enabled[RED] = 0;
        pulse_state[RED] = OFFSTATE;  
        last_state[RED] = OFFSTATE;  
        pulse_timer[RED] = 0;   
        //digitalWrite( markerPin[RED], LOW);
        analogWrite( markerPin[RED], 0);
      }
      if( strchr(args[1], 'B') ) 
      {
        pulse_enabled[BLUE] = 1;
        pulse_state[BLUE] = OFFSTATE;  
        last_state[BLUE] = OFFSTATE;  
        pulse_timer[BLUE] = 0;  
        //digitalWrite( markerPin[BLUE], LOW);   
        analogWrite( markerPin[BLUE], 0);     
      }
      if( strchr(args[1], 'G') ) 
      {
        pulse_enabled[GREEN] = 1;
        pulse_state[GREEN] = OFFSTATE;  
        last_state[GREEN] = OFFSTATE;  
        pulse_timer[GREEN] = 0;    
       //digitalWrite( markerPin[GREEN], LOW);
        analogWrite( markerPin[GREEN], 0);        
      }     
   }
   else
   {
  
       for( int i = 0; i < 3; i++)
       {  
          pulse_enabled[i] = 0;
          pulse_timer[i] = 0; 
          pulse_state[i] = OFFSTATE;
          last_state[i] = OFFSTATE;
          //digitalWrite( markerPin[i], LOW);
          analogWrite( markerPin[i],0);
       }   
        allPanelsOff();
   }     
//     digitalWrite(TESTPIN, 0);        
}

// ============================
// ===  P A U S E    C M D  ===
// ============================

void pauseCmd(int arg_cnt, char **args) 
{
    for( int i = 0; i < 3; i++)
       pulse_enabled[i] = 0;    
     allPanelsOff();
//     digitalWrite(TESTPIN, 0);  
}

// =========================
// ===  H E L P   C M D  ===
// =========================

void helpCmd(int arg_cnt, char **args) 
{
    Stream *s = cmdGetStream();
    s->println("--------------");   
    s->print("RGB LED V:");
    s->println(VERSION);
    if( digitalReadFast( boardTypePin ) == HIGH )
      s->println("LED board");
    else
      s->println("Digital Output Board");  
#ifdef DEBUG       
    s->println("Debug ON");    
#endif
    s->println("\r\nPARAMETER      BLU  GRN  RED");
   
    s->print("intensity   %: ");
    for( int rgb = 0; rgb < 3; rgb++ )
    {
         s->print(intensityPC[rgb]);
         s->print(" ");
    } 
    
    s->print("\npulse on   ms: ");
    for( int rgb = 0; rgb < 3; rgb++ )
    {
         s->print(pulse_on_msec[rgb]);
         s->print(" ");
    }
    s->print("\nperiod     ms: ");    
    for( int rgb = 0; rgb < 3; rgb++ )
    {
        s->print(pulse_off_msec[rgb] + pulse_on_msec[rgb]);
        s->print(" ");
    }
    s->print("\npulse   count: ");    
    for( int rgb = 0; rgb < 3; rgb++ )
    {
        s->print(pulse_count[rgb]);
        s->print(" ");
    }
    s->print("\noff time   ms: ");    
    for( int rgb = 0; rgb < 3; rgb++ )
    {
        s->print(off_time_msec[rgb]);
        s->print(" ");
    }
    s->print("\nwait time sec: ");    
    for( int rgb = 0; rgb < 3; rgb++ )
    {       
       s->print(wait_sec[rgb]);    
       s->print(" "); 
    }  
    s->print("\nwait time msec: ");    
    for( int rgb = 0; rgb < 3; rgb++ )
    {       
       s->print(wait_msec[rgb]);    
       s->print(" "); 
    }     
    s->print("\nnumber loops : ");    
    for( int rgb = 0; rgb < 3; rgb++ )
    {       
       s->print(iteration_loops[rgb]);    
       s->print(" "); 
    }      
    s->println(' ');    
}

// ==============================
// ===  I N I T  B O A R D S  ===
// ==============================


void initBoards(void)
{
   sendCmd(SERIES);     // enumerate passes the command from micro to micro 
   delay(100);           // allow for downstream micros to read cmd and set to series mode
   watchdog.reset(); // nice doggy!
   sendCmd(ENUMERATE1); // enumerate so each board knows where it is in daisy-chain
   delay(100);           // allow time for command to propogate
   watchdog.reset(); // nice doggy!
   sendCmd(PARALLEL);   // normal mode sends commnds to all at the same time
   delay(500);          //  allow time for propogatiion and change over
   watchdog.reset(); // nice doggy!
   sendCmd(SETOFF);     // turn all LEDS off on others
   if( digitalReadFast( boardTypePin) == LED_BOARD )
      initDACs();          // reset DACs and enable reference
   else
      initDOUT();   
   ledsOff(0x0f);       // and myself  
   for( uint8_t color = 0; color < 4; color++ ) // reset all intensity values
   {
       for (uint8_t quad = 0; quad < 7; quad++)
          intensity[color][quad] = 0;
   }  
   inited = 1;
}

// ---  R E S E T   C M D ---
void resetCmd(int arg_cnt, char **args) 
{   
    initBoards();
}

// ==========================
// === R E A D   R G B   ====
// ==========================

int8_t readRGB(char * cmdp)
{
int8_t colors = 0;

  if( strchr(cmdp, 'R')) colors |= REDDAC;  
  if( strchr(cmdp, 'B')) colors |= BLUDAC;
  if( strchr(cmdp, 'G')) colors |= GRNDAC;
  if( colors == 0 ) colors--;  // if no colors return -1 error
  return colors;
}

// ==========================
// === P U L S E   C M D ====
// ==========================


//  PULSE  a,b,c,d,e, f, C

//    a - pulse width
//    b - pulse period
//    c - # pulses
//    d - off time
//    e - wait time
//    f - number of iterations; 0 = continuous
//    C - color channel ('R', 'G', and/or 'B')
// ==========================

void pulseCmd(int arg_cnt, char **args)
{
int32_t val[6];
//int rgb = -1;
int rgb = 0;     // for all colors turn on and off together, so use the 0th array element
int errflag = 0;

   if( arg_cnt >= 6 )
   {
       for( int i = 0; i < 4; i++ )
       {
          val[i] = cmdStr2Num(args[i+1], 10); 
       }
       
       float fwait = cmdStr2Float(args[5]);  // wait time, floating point. divide into integer secs and turn fractu=ion into msec
       val[4] = (int32_t) fwait;
       int32_t waitMsec = (fwait - val[4]) * 1000;
       
       if( arg_cnt >= 7)  // iterations
          val[5] = cmdStr2Num(args[6], 10); 
       else
          val[5] = 0;

       if( arg_cnt == 8 ) // color!
       {
          int color = readRGB(args[7]); 
          if( color > -1 )  // good color
             rgb = color;
       }  
       else
       {
          rgb = 0x07;  // if no color set,  then set all 
       }
             
        if( (val[0] < MINWIDTH)  || (val[0] > MAXWIDTH)  ) errflag = -2;
        if( (val[1] < MINPERIOD) || (val[1] > MAXPERIOD) ) errflag = -3;
        if( (val[2] < MINPULSES) || (val[2] > MAXPULSES) ) errflag = -4;
        if( (val[3] < MINOFF)    || (val[3] > MAXOFF)    ) errflag = -5;
        if( (val[4] < MINWAIT)   || (val[4] > MAXWAIT)   ) errflag = -6;
        if( (val[5] < MINITERATIONS) || (val[5] > MAXITERATIONS)   ) errflag = -7;
        
        if( val[1] < val[0] ) errflag = -8;

        if( errflag == 0 ) 
        {
            for( uint8_t ch = 0; ch < 3; ch++ )
            {
               if( rgb & (1 << ch) ) // if this channel was selected
               {
                  pulse_on_msec[ch] = val[0];
                  pulse_off_msec[ch] = val[1] - val[0];
                  pulse_count[ch] = val[2];
                  off_time_msec[ch] = val[3];
                  wait_sec[ch]  = val[4];
                  wait_msec[ch] = waitMsec;
                  iteration_loops[ch] = val[5];
                
                 // if pulse on is 0, then turn off this channel
                 // else enable it
                  if( pulse_on_msec[ch] == 0 ) 
                    pulse_set[ch] = 0;
                  else
                    pulse_set[ch] = 1;   
    
    #ifdef DEBUG
      Serial.print("for: ");
      Serial.println(ch);
      Serial.println(pulse_on_msec[ch]);
      Serial.println(pulse_off_msec[ch]);
      Serial.println(off_time_msec[ch]);
      Serial.println(pulse_count[ch]);
      Serial.println(off_time_msec[ch]);
      Serial.println(wait_sec[ch]);  
      Serial.println(wait_msec[ch]);
      Serial.println(iteration_loops[ch]);      
    #endif
               }  // endif this channel is to be set
            } // check next channel
        }  // endif no error    
   } 
   else  // not enough arguments
   {
        errflag = -1;
   }
   if( errflag != 0 ) { Serial.print("err="); Serial.println(errflag); }
}

void patternCmd(int arg_cnt, char **args)
{  
  // depracated
}  

// =======================================
// === E X P E R I M E N T    C M D S  ===
// =======================================

//  addOnTimeStep stepNumber, Duration, RedIntensity, RedPattern, GrnIntensity, GrnPattern, BluIntensity, BluPattern
//
//1.  addOneStep(parameters): upload one experiment step to the buffer. Each step includes the following parameters: 
//      StepNumber, 
//      RedIntensity, RedPulsePeriod, RedPulseWidth, RedPulseNum, RedOffTime, RedIteration, …
//      GrnIntensity, GrnPulsePeriod, GrnPulseWidth, GrnPulseNum, GrnOffTime, GrnIteration,…
//      BluIntensity, BluPulsePeriod, BluPulseWidth, BluPulseNum, BluOffTime, BluIteration,…
//      DelayTime, Duration(step), [optional] pattern )
// OR
//  addOneStep(parameters): upload one experiment step to the buffer. Each step includes the following parameters: 
//      StepNumber, 
//      RedIntensity, RedPattern, …
//      GrnIntensity, GrnPattern, …
//      BluIntensity, BluPattern,…
//      Duration(step)

//2.  protocol = getExperimentsteps(): return all experimental steps for debugging;
//3.  removeAllSteps():  remove all steps from the buffer; 
//4.  runExperiment(): start experiment; 
//5.  StopExperiment(): stop experiment;
//6.  flyBowlEnabled(‘true/false’, ‘true/false’, ‘true/false’, ‘true/false’): choose which bowls are enabled for experiments;
//7.  status = getExperimentStatus: return the status of the current experiment;

#define MAXSTEPS 50
#define NO_TYPE 0
#define PULSE_TYPE 1
#define DURATION_TYPE 2


struct stepValues 
{
    uint8_t type;           // 0 = not set, 1 = pulse, 2 = duration 
    float intensity[3];     // intensity in %
    int32_t pulseOff[3];    // pulse off time
    int32_t pulseOn[3];     // pulse on time
    int32_t pulseCount[3];  // number of pulses 
    int32_t offTime[3];     // off time
    uint16_t iterations[3]; // how many times to repeat pulse train
    uint16_t onOff[3];      // on/off settings for duration type
    uint16_t aOrD[3];       // anaolg or digtal output
    float delayTime;        // delay 
    float duration;         // total duration of this step
    uint16_t pattern;       // binary pattern of what quads are on (1) or off (0)
    boolean usePattern;     // true if pattern was appended to command
};

// 


struct stepValues xStep[MAXSTEPS];

// addonestep 1  5 500 1000 3 1000 2  0   0   0 0    0  0 0 0   0   0 0    0 0 30
// addonestep 1  5 500 1000 3 1000 1  0   0   0 0    0  0 0 0   0   0 0    0 2 7
// addonestep 2  0   0    0 0    0 0  3 200 500 4 1000  1 0 0   0   0 0    0 0 3
// addonestep 2 10 200  500 5 1000 4  0   0   0 0    0  0 0 0   0   0 0    0 0 30
// addonestep 2  0   0    0 0    0 0  3 200 500 5 1000  4 0 0   0   0 0    0 0 30
// addonestep 3  0   0    0 0    0 0  0   0   0 0    0  0 0 4 200 500 5 1000 4 30


// addonestep 1, 0.00, 500,1000,1,2000,10,  8.00, 500,1000,1,2000,10,  0.00, 500,1000,1,2000,10, 5,60
// addonestep 2, 8.00,1000,1000,1,2000,10,  0.00,1000,1000,1,2000,10,  0.00,1000,1000,1,2000,10, 0,60

// addonestep 1, 0.00,500,1000,1,2000,10, 3.00,500,1000,1,2000,3, 0.00,500,1000,1,2000,10,5,20


// addonestep 1, 0.00, 500,1000,1,2000,5,  8.00, 100,1000,1,2000,5,  0.00, 500,1000,1,2000,5, 5,25, 11001010 
// addonestep 2, 8.00,1000,1000,1,2000,5,  0.00,1000,1000,1,2000,5,  0.00,1000,1000,1,2000,5, 5,30, 00110101
// addonestep 2, 0.00, 500,1000,1,2000,5,  0.00, 500,1000,1,2000,5,  8.00, 300,1000,1,2000,5, 5,25, 11000011 

//  runExperiment
//  getExperimentStatus
//  getExperimentsteps
// stopExperiment


// ================================
// === A D D   O N E   S T E P  ===
// ================================


void addOneStep(int arg_cnt, char **args)
{ 

    Stream *s = cmdGetStream();

#ifdef DEBUG
 Serial.println( arg_cnt);
 for( int i = 0; i < arg_cnt; i++)
 {
    Serial.print(i);
    Serial.print( "=");
    Serial.print( args[i]);
    Serial.print( " ");
 }
 Serial.println();
#endif

  if( arg_cnt >= 22 ) // command + 21 values must be present, 22 if pattern
  {
      int idx =  cmdStr2Num(args[1],10);  // get step number

//#ifdef DEBUG
//      Serial.print(idx);
//      Serial.print(" ");
//      Serial.println(MAXSTEPS);
//#endif
     
      if( idx < MAXSTEPS-1)
      {
          xStep[idx].intensity[RED] = cmdStr2Float(args[2]);
          xStep[idx].pulseOn[RED] =  cmdStr2Num(args[3], 10);
          xStep[idx].pulseOff[RED] = cmdStr2Num(args[4], 10) - xStep[idx].pulseOn[RED];  // send pulse perios, but we want off time to run it
          xStep[idx].pulseCount[RED] = cmdStr2Num(args[5], 10);
          xStep[idx].offTime[RED] = cmdStr2Num(args[6], 10);
          xStep[idx].iterations[RED] = cmdStr2Num(args[7], 10);      
          xStep[idx].intensity[GREEN] = cmdStr2Float(args[8]);
          xStep[idx].pulseOn[GREEN] =  cmdStr2Num(args[9], 10);
          xStep[idx].pulseOff[GREEN] = cmdStr2Num(args[10], 10) - xStep[idx].pulseOn[GREEN];  // send pulse perios, but we want off time to run it
          xStep[idx].pulseCount[GREEN] = cmdStr2Num(args[11], 10);
          xStep[idx].offTime[GREEN] = cmdStr2Num(args[12], 10);
          xStep[idx].iterations[GREEN] = cmdStr2Num(args[13], 10);        
          xStep[idx].intensity[BLUE] = cmdStr2Float(args[14]);
          xStep[idx].pulseOn[BLUE] =  cmdStr2Num(args[15], 10);
          xStep[idx].pulseOff[BLUE] = cmdStr2Num(args[16], 10) - xStep[idx].pulseOn[GREEN];  // send pulse perios, but we want off time to run it
          xStep[idx].pulseCount[BLUE] = cmdStr2Num(args[17], 10);
          xStep[idx].offTime[BLUE] = cmdStr2Num(args[18], 10);
          xStep[idx].iterations[BLUE] = cmdStr2Num(args[19], 10);       
          xStep[idx].delayTime = cmdStr2Float(args[20]);   // cmdStr2Num(args[20], 10);      
          xStep[idx].duration = cmdStr2Float(args[21]);   // cmdStr2Num(args[21], 10);   
          if( arg_cnt == 23 ) 
          {
             xStep[idx].pattern = cmdStr2Num(args[22],2); // get optional pattern
             xStep[idx].usePattern = true;
          }
          else
          {
            xStep[idx].usePattern = false;
          }

          xStep[idx].type  = PULSE_TYPE;
          

#ifdef DEBUG
  Serial.print(idx);
  Serial.print(" ");
  Serial.println(xStep[idx].type);
#endif         
      }   
  }   
  else if( arg_cnt == 8 ) // duration command
  {
      int idx =  cmdStr2Num(args[1],10);  // get step number
    
    //#ifdef DEBUG
    //      Serial.print(idx);
    //      Serial.print(" ");
    //      Serial.println(MAXSTEPS);
    //#endif

 
      if (idx < MAXSTEPS-1)
      {
          xStep[idx].intensity[RED] = cmdStr2Float(args[2]);    
          xStep[idx].onOff[RED] = 0;
          xStep[idx].aOrD[RED] = 0;
          int i = 0;
          while( (args[3][i] >= '0') && (i < 16) )
          {
             if( args[3][i] == '1' )
             {
                 xStep[idx].onOff[RED] |= 1; 
             }
             else if( args[3][i] == 'D')
             {                 
                 xStep[idx].onOff[RED] |= 1;
                 xStep[idx].aOrD[RED] |= 1;
             }
             xStep[idx].onOff[RED] = xStep[idx].onOff[RED] << 1;
             xStep[idx].aOrD[RED] =xStep[idx].aOrD[RED] << 1;
             i++;
          }
          
          xStep[idx].intensity[GREEN] = cmdStr2Float(args[4]);
          xStep[idx].onOff[GREEN] = 0;
          xStep[idx].aOrD[GREEN] = 0;
          i = 0;
          while( (args[3][i] >= '0') && (i < 16) )
          {
             if( args[3][i] == '1' )
             {
                 xStep[idx].onOff[GREEN] |= 1; 
             }
             else if( args[3][i] == 'D')
             {                 
                 xStep[idx].onOff[GREEN] |= 1;
                 xStep[idx].aOrD[GREEN] |= 1;
             }
             xStep[idx].onOff[GREEN] = xStep[idx].onOff[GREEN] << 1;
             xStep[idx].aOrD[GREEN] = xStep[idx].aOrD[GREEN] << 1;
             i++;
          } 
     
          xStep[idx].intensity[BLUE] = cmdStr2Float(args[6]);
          xStep[idx].onOff[BLUE] = 0;
          xStep[idx].aOrD[BLUE] = 0;
          i = 0;
          while( (args[3][i] >= '0') && (i < 16) )
          {
             if( args[3][i] == '1' )
             {
                 xStep[idx].onOff[BLUE] |= 1; 
             }
             else if( args[3][i] == 'D')
             {                 
                 xStep[idx].onOff[BLUE] |= 1;
                 xStep[idx].aOrD[BLUE] |= 1;
             }
             xStep[idx].onOff[BLUE] = xStep[idx].onOff[BLUE] << 1;
             xStep[idx].aOrD[BLUE] = xStep[idx].aOrD[BLUE] << 1;
             i++;
          } 
    
          xStep[idx].duration = cmdStr2Float(args[8]);   // cmdStr2Num(args[21], 10);   
          xStep[idx].type  = DURATION_TYPE;
      }    

#ifdef DEBUG
  Serial.print(idx);
  Serial.print(" ");
  Serial.println(xStep[idx].type);
#endif         
   }    
        
  int cnt = 0;
  // return number of steps so far 
  for ( int i = 1; i < MAXSTEPS; i++ )
  { 
    if( xStep[i].type > 0) 
       cnt++;
  }
  s->println(cnt);
  
}


// =================================================
// === G E T   E X P E R I M E N T    S T E P S  ===
// =================================================

void getExperimentsteps(int arg_cnt, char **args)
{ 
     
   Stream *s = cmdGetStream();
   for( int idx = 0; idx < MAXSTEPS; idx++ )
   {
      if( xStep[idx].type == PULSE_TYPE )
      {
          s->print(idx);
          s->print(",");
          s->print(xStep[idx].intensity[RED]);
          s->print(",");       
          s->print(xStep[idx].pulseOff[RED] + xStep[idx].pulseOn[RED]);
          s->print(",");     
          s->print(xStep[idx].pulseOn[RED]);
          s->print(",");  
          s->print(xStep[idx].pulseCount[RED]);
          s->print(",");  
          s->print(xStep[idx].offTime[RED]);
          s->print(",");                
          s->print(xStep[idx].iterations[RED]);
          s->print(",");     
          s->print(xStep[idx].intensity[GREEN]);
          s->print(",");       
          s->print(xStep[idx].pulseOff[GREEN] + xStep[idx].pulseOn[GREEN]);
          s->print(",");     
          s->print(xStep[idx].pulseOn[GREEN]);
          s->print(",");  
          s->print(xStep[idx].pulseCount[GREEN]);
          s->print(",");  
          s->print(xStep[idx].offTime[GREEN]);
          s->print(",");                
          s->print(xStep[idx].iterations[GREEN]);
          s->print(",");        
          s->print(xStep[idx].intensity[BLUE]);
          s->print(",");       
          s->print(xStep[idx].pulseOff[BLUE] + xStep[idx].pulseOn[BLUE]);
          s->print(",");     
          s->print(xStep[idx].pulseOn[BLUE]);
          s->print(",");  
          s->print(xStep[idx].pulseCount[BLUE]);
          s->print(",");  
          s->print(xStep[idx].offTime[BLUE]);
          s->print(",");                
          s->print(xStep[idx].iterations[BLUE]);
          s->print(",");       
          s->print(xStep[idx].delayTime);      
          s->print(",");              
          s->print(xStep[idx].duration);  
          if( xStep[idx].usePattern == true )
          {          
            s->print(",");              
            s->println(xStep[idx].pattern, BIN);  
          }
          else
          {
            s->println();  
          }       
      }  //   pulse step 
      else if ( xStep[idx].type == DURATION_TYPE )
      {
          s->print(idx);
          s->print(",");
          s->print(xStep[idx].intensity[RED]);
          s->print(","); 
          for( uint16_t i = 1; i <= 0x8000; i = i << 1)
          {
            if( xStep[idx].onOff[RED] & i ) 
            {
                if( xStep[idx].aOrD[RED] & i )       
                   s->print('D');
                else
                   s->print('1');
            }
            else
            {
                s->print('0');
            }                 
          }
          s->print(",");     

          s->print(xStep[idx].intensity[GREEN]);
          s->print(",");       
          for( uint16_t i = 1; i <= 0x8000; i = i << 1)
          {
            if( xStep[idx].onOff[GREEN] & i ) 
            {
                if( xStep[idx].aOrD[GREEN] & i )       
                   s->print('D');
                else
                   s->print('1');
            }
            else
            {
                s->print('0');
            }                 
          }
          s->print(",");      
 
          s->print(xStep[idx].intensity[BLUE]);
          for( uint16_t i = 1; i <= 0x8000; i = i << 1)
          {
            if( xStep[idx].onOff[BLUE] & i ) 
            {
                if( xStep[idx].aOrD[BLUE] & i )       
                   s->print('D');
                else
                   s->print('1');
            }
            else
            {
                s->print('0');
            }                 
          }          
          s->println();      
          
      }    
   }  // next step
}  // end getExperimentsteps


// =======================================
// === R E M O V E   A L L   S T E P S ===
// =======================================

void removeAllSteps(int arg_cnt, char **args)
{    
   for( int idx = 0; idx < MAXSTEPS; idx++ )
   {
      xStep[idx].type = 0;
   }
}


// ====================================
// === R U N   E X P E R I M E N T  ===
// ====================================
      
void runExperiment(int arg_cnt, char **args) // start experiment; 
{
#ifdef DEBUG
   Serial.println("runx "); 
#endif 
 
     xIdx = 0;
     for( int i = 0; i < 4; i++)
     {
        if( bowlEnabled[i] == true ) bowls |= (1 < i);
     }

// skip over any unfilled steps 
     while( (xStep[xIdx].type == 0) && (xIdx < MAXSTEPS) ) xIdx++;

     for ( uint8_t rgb = 0; rgb < 3; rgb++)
        newIntensity( 0.0, rgb, 0, 0x0f);  // turn off all colors before setting pattern

     if( xStep[xIdx].usePattern )  // set the pattern - this requires enabling active quadrants
     {
        uint16_t bitpat = xStep[xIdx].pattern;
        uint8_t quadbits;
        for( uint8_t pan = 1; pan < 5; pan++)
        {
            quadbits = bitpat & 0x0f; 
            bitpat = bitpat >> 4;      
            setOnOff( pan, quadbits, 1 );
        }    
     } 
     else
     {
           setOnOff( 0, 0x0f, 1 );   // set all active   
     }

     setOnOff(0,0,0); // now turn everything off before setting intensities  

    

     for( int i = 0; i < 3; i++)
     {      
        newIntensity( xStep[xIdx].intensity[i], i, bowls,0x0f);
        pulse_count[i] =  xStep[xIdx].pulseCount[i]; //iterations[i];
        pulse_on_msec[i] = xStep[xIdx].pulseOn[i];
        pulse_off_msec[i] = xStep[xIdx].pulseOff[i]; 
        off_time_msec[i] = xStep[xIdx].offTime[i];
        iteration_loops[i] = xStep[xIdx].iterations[i];
        wait_msec[i] = uint32_t (xStep[xIdx].delayTime * 1000.0) ;
        wait_cnt[i] = 0;
        frame_on_msec[i] = 0;
        frame_off_msec[i] = 0;
        if( xStep[xIdx].intensity[i] > 0.001)
        {
          pulse_enabled[i] = 1;
          pulse_state[i] = START;
          syncTimer.begin(syncInt, 500000/syncRate);  // change immediately (or start if ended)
        }   
        else
        {
          markerOff(i);
          pulse_enabled[i] = 0;
          pulse_state[i] = HOLD;
        }

        experimentMsecs = 0;

#ifdef DEBUG
   Serial.print("RGB "); 
   Serial.print(i);
   Serial.print(" WMS "); 
   Serial.println(wait_msec[i]);
#endif 
   
        
     }
 //    stepDelay = xStep[xIdx].delayTime; 
     stepDuration = uint32_t( xStep[xIdx].duration * 1000);


              
 #ifdef DEBUG        
  Serial.println("==== XRUN");
#endif  
         
  //   xIdx++; // point to next set of values and >0 indicates we are are doing an experiment  
}


// =======================================
// === S T O P    E X P E R I M E N T  ===
// =======================================

void StopExperiment(int arg_cnt, char **args) // stop experiment;
{
      xIdx = 0;  // this stops experiment after current step
      setOnOff(0,0,0);  // set all panels and quadrants off     
      for( uint8_t rgb = 0; rgb < 3; rgb++)  // all colors
      {
          markerOff(rgb);
          pulse_state[rgb] = HOLD;   // skip to HOLD state 
      }  

#ifdef DEBUG        
  Serial.println("=== XSTOP");
#endif           
}

// =======================================
// === F L Y   B O W L   E N A B L E D ===
// =======================================

void flyBowlEnabled(int arg_cnt, char **args) // (‘true/false’, ‘true/false’, ‘true/false’, ‘true/false’): choose which bowls are enabled for experiments;
{

 //  Serial.println(arg_cnt);
   
 //    Stream *s = cmdGetStream();
     if ( arg_cnt > 4 )
     {
         for( int i = 0; i < 4; i++)
         {

// Serial.print( args[i+1] );
 
            if( !strcmp( args[i+1], "TRUE" ) )
               bowlEnabled[i] = true;
            else 
               bowlEnabled[i] = false;
#ifdef DEBUG
  Serial.print(bowlEnabled[i]);
  if( i < 3 )  Serial.print(" "); else Serial.println();
#endif
         }                
     }  
}

// ===================================================
// === G E T   E X P E R I M E N T    S T A T U S  ===
// ===================================================

void getExperimentStatus(int arg_cnt, char **args) // getExperimentStatus: return the status of the current experiment;
{
    Stream *s = cmdGetStream();
    s->print(experimentMsecs);
    s->print(',');
    s->print(xIdx);    // xIdx points to next step 
    s->print(',');
    if( markerState[RED] )
        s->print( xStep[xIdx].intensity[RED]);
    else
        s->print('0');      
    s->print(',');
    if( markerState[GREEN] )
        s->print( xStep[xIdx].intensity[GREEN]);
    else
        s->print('0');  
    s->print(',');
    if( markerState[BLUE] )
        s->println( xStep[xIdx].intensity[BLUE]);
    else
        s->println('0');  
}


// ==============================
// === M A R K E R   C M D S  ===
// ==============================

void markerCmd(int arg_cnt, char **args)  //marker test
{
    Stream *s = cmdGetStream();
    if ( arg_cnt > 1 )
    {  // comes in as %, change to 0-255
       markerIntensity =(uint8_t) (cmdStr2Num(args[1], 10) * 255 / 100);
    }
    s->println((uint32_t)markerIntensity * 100 / 255); // go out as %
    //digitalWrite(redPin, HIGH);
    analogWrite(redPin, markerIntensity);
    digitalWrite(LEDpin, HIGH);
    for (int i = 0; i < 5; i++)
    {
      delay(1000);
      watchdog.reset(); // nice doggy!
    }    
    
    //digitalWrite(redPin, LOW);
    analogWrite(redPin, 0);
    digitalWrite(LEDpin, LOW);
    // digitalWrite(bluPin, HIGH);
    analogWrite(bluPin, markerIntensity);
    for (int i = 0; i < 5; i++)
    {
      delay(1000);
      watchdog.reset(); // nice doggy!
    }   
    
    // digitalWrite(bluPin, LOW);
    analogWrite(bluPin, 0);
    //digitalWrite(grnPin, HIGH);  
    analogWrite(grnPin, markerIntensity); 
    for (int i = 0; i < 5; i++)
    {
      delay(1000);
      watchdog.reset(); // nice doggy!
    }   
   //digitalWrite(grnPin, LOW);  
    analogWrite(grnPin, 0); 
}


// ==================================
// === L I S T   C O M M A N D S  ===
// ==================================

void listCommands(int arg_cnt, char **args) 
{
     cmdList();
}

// ==================================
// === S Y N C   R A T E   C M D  ===
// ==================================

// set the sync output rate for triggering camera

void syncRateCmd(int arg_cnt, char **args)
{  
  float tsync;

    if( arg_cnt > 1 )
    {
        tsync = cmdStr2Float(args[1]);
        if( (tsync > MINSYNC) && (tsync < MAXSYNC) )
        {
           syncRate = tsync;
           syncTimer.begin(syncInt, 500000/syncRate);  // change immediately (or start if ended)
           //syncTimer.update( 500000/syncRate);
        }
        else
        {
          if( (tsync > -0.01) && (tsync < 0.01) ) // if '0' then turn sync off
          {
             syncTimer.end();
          }
        }
    }
    else
    {
       Stream *s = cmdGetStream();
       s->println(syncRate);   
    }
}

// =====================
// === W D T  C M D  ===
// =====================
void wdtCmd(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  if( arg_cnt > 1 )  // any arguamnet will cause a test
    while(1);
  else  // see if last reset was WDT
  { 
    s->println( (RCM_SRS0 & 0x20) >> 5 ); 
    s->println( watchdog.tripped() );
  }   
  
}
// -- End USB  commnds ------

// =================================== I N T E R    B O A R D    C O M M A N D S ===============================
// --- command struture
//  - pp = panel (0b00 is all, 0b01 is 1, etc
//  - bbbb = quadrant bits(0b0001 is upper left, 0b0010 upper right, 0b1000 lower right, 0b0100 lower left
//  NOTE: with multiple panels the actual hardware quadrant will vary depending on where the panel is located 
//  - qq   = quadrant number 00 is upper left, etc
//  - cc  = color, 00 = blue, 01 = green, 10 = red, 11 = IR
//  - iiii = intensity value nibble
//  - ddd = dac bits for colors: bit 0 = blue, bit 1 = green, bit 2 = red

// Set Quadrants Off:   00ppbbbb     // turn off selected quadrants on selected panel
// Set Quadrants On:    01ppbbbb     // turn on  selected quadrants
// Set Intensity:       10ppqqcc     // update the selected panel(s), quadrant, color to the current intensity value
// Intensity low 4 bits 1100iiii     // send lowest 4 bits of 12 bit intensity DAC value
// Intensity mid 4 bits 1101iiii     // send middle 4 bits of 12 bit intensity DAC value
// Intensity high4 bits 1110iiii     // send highest 4 bits of 12 bit intensity DAC value
// Enumerate command:   111100xx     // enumerate the boards. Main sends 0xf1, next panel sees this and sends F2 to next, etc
// Series Mode:         11110100     // set intraboard comms to serial - needed to do enumeration
// Parallel Mode:       11110101     // set intraboard comms to parallel - used for all other commands
// color set            11111ddd     // turn colors on and off for all panels (mostly for pulse) 1= on, 0 = off

// ========================
// === S E N D   C M D  ===
// ========================
void sendCmd(uint8_t cmd)
{
//#ifdef DEBUG  
//  Serial.print("cmd: ");
//  Serial.println(cmd,BIN);
//#endif  
  Serial1.write(cmd);
}

// panels positions change when grouped, but want quads to make sense
uint8_t MYQUAD[] = { 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0 };

uint8_t getMyQuad(uint8_t myAdr, uint8_t quad)
{
   return(MYQUAD[myAdr*3+quad]); 
}

// ==========================================
// ===  I N T E R   B O A R D   P O L L   ===
// ==========================================

void interBoardPoll(void)
{
   while( Serial1.available() )
   {     
       uint8_t cmd = Serial1.read(); 
#ifdef DEBUG
    Stream *s = cmdGetStream();
    s->print("in:");
    s->print(cmd, BIN);
    s->print(" ");
#endif   
       if( (cmd & COLORSET) == COLORSET) // set colors on or off
       {  // ====== COLORSET =====       
          colorSet(cmd & 0x07);  // send RGB settings to DACs
       }
       // ======= ON OFF COMMAND =======
       else if( (cmd & 0x80) == 0 )  // bit 7 = 0; so, on or off
       {
          if( ((cmd & PANELBITS) == thisPanel) || ((cmd & PANELBITS) == 0) )
          {
#ifdef DEBUG          
    s->print(" On/Off");
#endif      
              if( cmd & SETON )  // on
              {
                 ledsOn(cmd & 0x0f);  // turn on selected quadrants               
              }                 
              else
              {
                 ledsOff(cmd & 0x0f);  // turn off selected quadrants     
              }
          } // endif this panel 
       } // endif on/off
       // =======  SET INTENSITY COMMAND =========
       else if( (cmd & 0x40) == 0 )  // bit 7 = 1, bit 6 = 0; so, intensity set
       {
           if( ((cmd & PANELBITS) == thisPanel) || ((cmd & PANELBITS) == 0) )
          {  
              uint8_t color = cmd & 0x03;
              uint8_t quad = (cmd & 0x0c) >> 2; 
              uint16_t DACval = (intensityHON << 8) + (intensityMON << 4) + intensityLON;   
              setIntensity(color, quad, DACval );     
#ifdef DEBUG
    s->print("panel:");
    s->print( (cmd & PANELBITS)>>4 );
    s->print(" quad:");
    s->print(quad, HEX);
    s->print(" color:");
    s->print(color, HEX);
    s->print(" to:");
    s->print(intensity[color][quad], HEX);   
#endif                   
          }
       }
       // ======= INTENSITY LON ===========
       else if( (cmd & 0xf0) == INTENSITYLON )  
       {
            intensityLON = uint16_t(cmd & 0x0f);  // intensity
#ifdef DEBUG
   s->print("LON:");
   s->print(intensityLON, HEX);   
#endif    
       }   
       // ======= INTENSITY MON  =========
       else if( (cmd & 0xf0) == INTENSITYMON )  
       {
            intensityMON = uint16_t(cmd & 0x0f);  // intensity  
#ifdef DEBUG
   s->print("MON:");
   s->print(intensityMON, HEX);   
#endif 
       }      
       // ======= INTENSITY HON ========
       else if( (cmd & 0xf0) == INTENSITYHON )  
       {
            intensityHON = uint16_t(cmd & 0x0f);  // intensity
#ifdef DEBUG
   s->print("HON:");
   s->print(intensityHON, HEX);   
#endif 
       }     
       // ======== ENUMERATE 1 ========
       else if( cmd == ENUMERATE1 )  // from host board - I'm #1
       { 
            thisPanel = 0x10;
            if( digitalReadFast( boardTypePin) == LED_BOARD)
              initDACs();          // reset DACs and enable reference
            else
              initDOUT();  
            ledsOff(0x0f); // basically a reset - so be sure we start off
            sendCmd(ENUMERATE2);
#ifdef DEBUG
   s->println("--- I'm board #1");  
#endif             
       }    
       // ====== ENUMERATE 2 ========
       else if( cmd == ENUMERATE2 )   // from board next to host - I'm #2
       { 
            thisPanel = 0x20;
            if( digitalReadFast( boardTypePin) == LED_BOARD)
              initDACs();          // reset DACs and enable reference
            else
              initDOUT();  
            ledsOff(0x0f); // basically a reset - so be sure we start off
            sendCmd(ENUMERATE3);
#ifdef DEBUG
   s->println("--- I'm board #2");  
#endif              
       }      
       // ====== ENUMERATE 3 ========
       else if( cmd == ENUMERATE3 )   // from 2nd board from host - I'm last (#3)
       {
            thisPanel = 0x30;   
            if( digitalReadFast( boardTypePin) == LED_BOARD)
              initDACs();          // reset DACs and enable reference
            else
              initDOUT();     
            ledsOff(0x0f); // basically a reset - so be sure we start off
#ifdef DEBUG
   s->println("--- I'm board #3");  
#endif              
       }      
       // ======= SERIES =======
       else if( cmd == SERIES)
       {
            digitalWriteFast( DIRpin, HIGH);
#ifdef DEBUG
    s->print(" serial-------------- ");
#endif            
       }
       // ======= PARALLEL ========
       else if( cmd == PARALLEL)
       {
            sendCmd(PARALLEL);  // since we are probably in serial mode, we need to manually propogate the cmd
            Serial1.flush();    // wait until it's satrting to send
            delay(1);           // and then time to finish
            digitalWriteFast( DIRpin, LOW);   // before changing direction mode
            
#ifdef DEBUG
    s->print(" parallel------------- ");
#endif
       }
#ifdef DEBUG
       s->print(" me:");
       s->println(thisPanel, HEX);
#endif       
   }  // endif serial1 available
} // end interboard poll

// --- End Inter Board Commands ---

// =================================
// === O V E R   T E M P   I N T ===
// =================================

void OverTempInt()
{
   ledsOff(0x0f);                // force RGB LEDs off
   newIntensity( 0.0, IR, 0, 0x0f );
  // setIntensity(IRDAC, 0x0f, 0x00);   // and IR
   Serial.println("OVERTEMP!"); // and let host know
}


// ====================
// === M A R K E R ====
// ====================
// turn sync pin pin on/off
// a define is used to determine if 'on' is active low

void marker(uint8_t state)
{
#ifndef SYNC_PIN_OUT

#ifdef SYNC_ACTIVE_LOW
  if( state  )
     digitalWrite(SYNCpin, LOW);
  else
     digitalWrite(SYNCpin, HIGH);
#else
  if( state )
     digitalWrite(SYNCpin, HIGH);
  else
     digitalWrite(SYNCpin, LOW);
#endif

#endif 
}

// =======================================
// === M A R K E R   L E D  O N  O F F === 
// =======================================

void markerOn( int rgb)
{
   markerState[rgb] = true;
   switch( rgb)
   {
       case RED:
          //digitalWriteFast( redPin, HIGH);
          analogWrite(redPin, markerIntensity);
          break;
       case BLU:
          //digitalWriteFast( bluPin, HIGH);
          analogWrite(bluPin, markerIntensity);
          break;
       case GRN:
          //digitalWriteFast( grnPin, HIGH);
          analogWrite(grnPin, markerIntensity);  
          break;
   }
#ifdef DEBUG
     Serial.print('^');
#endif     
}

void markerOff( int rgb)
{
   markerState[rgb] = false;
   switch( rgb)
   {
       case RED:
          //digitalWriteFast( redPin, LOW);
          analogWrite(redPin, 0);
          break;
       case BLU:
          //digitalWriteFast( bluPin, LOW);
          analogWrite(bluPin, 0);
          break;
       case GRN:
          //digitalWriteFast( grnPin, LOW); 
          analogWrite(grnPin, 0); 
          break;
   }

#ifdef DEBUG 
     Serial.print('v');
#endif
}


// =========================
// === P U L S E   I N T ===
// =========================

// 1 msec timer

void pulseInt(void)
{ 
static int8_t rgbset;
int8_t newrgb = rgbset;  // assume no change

  experimentMsecs++;

//#ifdef DEBUG
//  if( (pulse_state[GRN] > 0) && (pulse_state[GRN] < 7) )
//  {
//    if( (experimentMsecs % 100) == 0)
//    {
//       Serial.print( experimentMsecs);
//      Serial.print(" ");
//      Serial.println(states[pulse_state[GRN]]);
//    }   
//  }  
//#endif
  
  for( int rgb = 0; rgb < 3; rgb++ ) 
  {    
     if( flag[rgb] )  // still running a step
     {
        pulse_timer[rgb]--;   // count down a msec
        if( pulse_timer[rgb] <= 0 )  // if done, load next step info
        {
          if( ledon[rgb] )   // set LED bits on or off
          { 
              bitSet(newrgb, rgb);
          }  
          else
          {
              bitClear(newrgb, rgb); 
          }
          pulse_timer[rgb] = next_timer[rgb];  // new timer value
          flag[rgb] = 0;   // let main loop know we are done with last step
        }   
      }  
   }  // endif check each color
   
   if( newrgb != rgbset)  // only update LEDs if changes
   {   
//Serial.print('*'); 
      rgbset = newrgb;
      sendCmd(COLORSET | rgbset);   // send the command out to others first
      colorSet(rgbset);  // now update all at once  
 
   }   
}

// ========================
// === S Y N C    I N T ===
// ========================

// camera sync

void syncInt(void)
{ 

  static boolean state = false;
  if( state )
  {
    digitalWriteFast(triggerPin, LOW);
#ifdef SYNC_PIN_OUT    
     digitalWriteFast(SYNCpin, LOW);
#endif     
    state = false;
  }
  else
  {
     digitalWriteFast(triggerPin, HIGH);
#ifdef SYNC_PIN_OUT    
     digitalWriteFast(SYNCpin, HIGH);
#endif     
     state = true;
  }
}


// ==================
// ===  S E T U P ===
// ==================

void setup()
{
    pinMode( DIRpin, OUTPUT);  
    pinMode( OTMPpin, INPUT_PULLUP);
    pinMode( LLpin, OUTPUT);
    pinMode( URpin, OUTPUT);
    pinMode( LRpin, OUTPUT);
    pinMode( ULpin, OUTPUT);
#ifdef SYNC_PIN_OUT
    digitalWrite( SYNCpin, LOW);
    pinMode( SYNCpin, OUTPUT);
#else    
    pinMode( SYNCpin, INPUT_PULLUP);
#endif    
    pinMode( LEDpin, OUTPUT);
    pinMode( triggerPin, OUTPUT);
    pinMode( redPin, OUTPUT);
    pinMode( bluPin, OUTPUT);
    pinMode( grnPin, OUTPUT);

    pinMode( en5Pin, OUTPUT);
    pinMode( en6Pin, OUTPUT);
    pinMode( en7Pin, OUTPUT);
    pinMode( en8Pin, OUTPUT); 
    pinMode( doutPin, OUTPUT);

    pinMode( boardTypePin, INPUT_PULLUP);
  
    digitalWrite(DIRpin, HIGH);     // assume series mode
    digitalWrite(LLpin, HIGH);
    digitalWrite(URpin, HIGH);
    digitalWrite(LRpin, HIGH);
    digitalWrite(ULpin, HIGH);
    digitalWrite(LEDpin, LOW);
    digitalWrite(triggerPin, LOW);
    //digitalWrite(redPin, LOW);
    analogWrite(redPin, 0);
    //digitalWrite(bluPin, LOW);
    analogWrite(bluPin,0);
    // digitalWrite(grnPin, LOW);
    analogWrite(grnPin, 0); 
    digitalWrite( en5Pin, HIGH);
    digitalWrite( en6Pin, HIGH);  
    digitalWrite( en7Pin, HIGH);
    digitalWrite( en8Pin, HIGH); 
    digitalWrite( doutPin, HIGH); 
    
    
    SPI.begin();
  
    ledsOff(0x0F);   // quadrants off
    
    Serial.begin(19200);   // USB comms
    cmdInit(&Serial);
    cmdAdd("RED",redCmd);     // set RED to % brightness, update actual LEDs if they are on, 0 = off
    cmdAdd("CHR",redCmd);     //  "
    cmdAdd("BLUE",blueCmd);   // set BLUE to % brightness
    cmdAdd("BLU",blueCmd);    //  " 
    cmdAdd("GREEN",greenCmd); // set GREEN to % brightness
    cmdAdd("GRN",greenCmd);   //  "
    cmdAdd("IR",irCmd);       // set IR to % brightness
    cmdAdd("ON",onCmd);       // turn ON selected quadrant(s) on selected panel(s) 0 = all
    cmdAdd("OFF",offCmd);     // same but OFF
    cmdAdd("RESET", resetCmd);
    cmdAdd("PULSE", pulseCmd);
    cmdAdd("RUN",runCmd);
    cmdAdd("STOP", stopCmd);
    cmdAdd("PAUSE", pauseCmd);
    cmdAdd("PATT", patternCmd);
    cmdAdd("SYNC", syncRateCmd);
    cmdAdd("ADDONESTEP", addOneStep); // upload one experiment step to the buffer.
    cmdAdd("GETEXPERIMENTSTEPS", getExperimentsteps); // return all experimental steps for debugging;
    cmdAdd("REMOVEALLSTEPS", removeAllSteps); //  remove all steps from the buffer; 
    cmdAdd("RX", runExperiment); // start experiment;
    cmdAdd("RUNEXPERIMENT", runExperiment); // start experiment; 
    cmdAdd("STOPEXPERIMENT", StopExperiment); // stop experiment;
    cmdAdd("FLYBOWLENABLED", flyBowlEnabled);  // choose which bowls are enabled for experiments;
    cmdAdd("GETEXPERIMENTSTATUS", getExperimentStatus); // return the status of the current experiment;  
    cmdAdd("MARKER", markerCmd);
    cmdAdd("DAC", DACCmd);
    cmdAdd("???",helpCmd);
    cmdAdd("HELP", listCommands);
    cmdAdd("WDT", wdtCmd);
    cmdAdd("GETDACS", getdacsCmd);
    cmdAdd("GAIN", gainCmd);
  
    Serial1.begin(250000); // inter board comms
//    Serial2.begin(19200);  // modular board comms 
   
    for( int rgb = 0; rgb < 3; rgb++)
    {
        pulse_on_msec[rgb] =  0;
        pulse_off_msec[rgb] = 0;
        off_time_msec[rgb] = 0;
        wait_sec[rgb] = 0; 
        wait_msec[rgb] = 0;
        pulse_count[rgb] = 0;
        iteration_loops[rgb] = 0;
        pulse_counter[rgb] = pulse_count[rgb];
        pulse_enabled[rgb] = 0;            
        ledon[rgb] = 0;
        flag[rgb] = 0;  // don't test anything in timer interrupt  
        sync_on[rgb] = 0;
        pulse_state[rgb] = OFFSTATE;
        last_state[rgb] = OFFSTATE;
//#ifdef DEBUG
//    pulse_set[rgb] = 1;
//#endif
    } 
    
    initBoards();

//    attachInterrupt(OTMPpin, OverTempInt, LOW );

    pulseTimer.begin(pulseInt, 1000);  // run every 1 msec

    syncTimer.begin(syncInt, 500000/syncRate);  // 1e6 counts/sec  / rate

   for( int idx = 0; idx < MAXSTEPS; idx++ )  // start with no set protocol
   {
      xStep[idx].type = 0;
   } 

   // if gains not initialized, then do it

   if( EEPROM.read(EE_INITED) != 0x55 )
   {
       for ( int i = 0; i < 16; i++ )
          writeFloat( i*4, 1.0);
       EEPROM.write(EE_INITED, 0x55);
   }     
   for ( int clr = 0; clr < 4; clr++)
   {  
      for ( int quad = 0; quad < 4; quad++ )
           LEDgain[clr][quad] = readFloat(clr * 16 +  quad * 4);    
   }
   
  // Setup WDT
  watchdog.enable(Watchdog::TIMEOUT_4S);  

} // END SETUP

uint32_t stepStart;

// ===================
// ===== L O O P =====
// ===================

void loop() 
{

  watchdog.reset(); // nice doggy!

  cmdPoll();
  interBoardPoll();

  // ----  do pulse stuff if enabled
  for( int rgb = 0; rgb < 3; rgb++ )
//  int rgb = 0;   // have not implemented separte color on and off with pulses
  {
    if( pulse_enabled[rgb] )
    {              
       if( pulse_state[rgb] != last_state[rgb] )
       {
          #ifdef DEBUG
            if( pulse_state[rgb] == START) Serial.println("=================");
            Serial.print(millis()-stepStart);
            Serial.print(" ");
            Serial.print(pulse_state[rgb],BIN);
            Serial.print(" ");
            Serial.print(colors[rgb]);
            Serial.print(":");
            Serial.print(states[last_state[rgb]]);
            Serial.print("->");
            Serial.println(states[pulse_state[rgb]]);
          #endif
          last_state[rgb] = pulse_state[rgb];
//
//          if( pulse_state[rgb] & 0x01 )
//              digitalWrite( state0Pin, HIGH);
//          else 
//              digitalWrite( state0Pin, LOW); 
//                 
//          if( pulse_state[rgb] & 0x02 )
//              digitalWrite( state1Pin, HIGH);
//          else 
//              digitalWrite( state1Pin, LOW); 
//              
//          if( pulse_state[rgb] & 0x04 )
//              digitalWrite( state2Pin, HIGH);
//          else 
//              digitalWrite( state2Pin, LOW); 
//              
//          if( pulse_state[rgb] & 0x08 )
//              digitalWrite( state3Pin, HIGH);
//          else 
//              digitalWrite( state3Pin, LOW);                          
       }       
        
        switch (pulse_state[rgb])
        {       
          case OFFSTATE: // default when nothing is going on

             break;
               
          case START:  // new experiment
             pulse_counter[rgb] = pulse_count[rgb];  // init pulse counter
  //           pinMode( alarmPin, OUTPUT); // pull down envelope pin
  //           Timer3.start();   // use alarmPin for camera trigger
             frame_on_cnt[rgb]  = frame_on_msec[rgb];
             frame_off_cnt[rgb] = 0;
             sync_on[rgb] = 1;
             iteration[rgb] = 0;
             stepStart = millis();
             noInterrupts();
             pulse_timer[rgb] = 0; // be sure we load next value right away
             interrupts();              
             
             if( wait_msec[rgb] > 0 )  // wait time request 
             {
               noInterrupts();
               ledon[rgb] = 0;
               next_timer[rgb] = wait_msec[rgb];     
               flag[rgb] = 1;             
               interrupts();       
               pulse_state[rgb] = WAIT;                 
             }
             else   // no wait - jump to pulses
             {
               noInterrupts();
               ledon[rgb] = 1;
               flag[rgb] = 1;
               pulse_timer[rgb] = 0;  // force update
               next_timer[rgb] = pulse_on_msec[rgb];
               interrupts();
               pulse_state[rgb] = PULSEON;
             }                   
             break;
       
          case WAIT:     
             if( flag[rgb] == 0 ) // wait for WAIT params to take
             {            
                 noInterrupts();
                 ledon[rgb] = 1;
                 flag[rgb] = 1;
                 next_timer[rgb] = pulse_on_msec[rgb];
                 interrupts();
                 pulse_state[rgb] = PULSEON;
             }             
             break;
             
          case PULSEON:  
             if( flag[rgb] == 0 )  // wait for ON params to 'take'
             {         
#ifdef DEBUG
Serial.println(millis()-stepStart);
#endif
               digitalWrite(LEDpin, HIGH);
               markerOn(rgb);
               
               if( pulse_off_msec[rgb] == 0 ) // continuous on mode?
               {  // if so, we will 'fake' continuous on by staying in on mode for N x pulse_width msecs
  
                  if( off_time_msec[rgb] != 0 )  // only count down have an off time
                      pulse_counter[rgb]--;
                  
                  if( pulse_counter[rgb] == 0 )  // ok we are done, go to off time
                  {  
                    noInterrupts();
                    ledon[rgb] = 0;
                    flag[rgb] = 1;
                    next_timer[rgb] = off_time_msec[rgb];
                    interrupts();                
                    pulse_state[rgb] = OFFTIME;  
                    pulse_counter[rgb] = pulse_count[rgb];    // reset pulse counter for next go round  
                  }
                  else   // still continuous mode, rerun pulse time 
                  {
                    noInterrupts(); 
                    ledon[rgb] = 1;
                    flag[rgb] = 1;
                    next_timer[rgb] = pulse_on_msec[rgb];
                    interrupts();
                 }            
               }   // endif check if cont on
               else  // not continuous, so go on to pulse off time
               {
                 noInterrupts(); 
                 ledon[rgb] = 0;
                 flag[rgb] = 1;
                 next_timer[rgb] = pulse_off_msec[rgb];              
                 pulse_state[rgb] = PULSEOFF;  
      //         pinMode( alarmPin, OUTPUT);    // set OC low (value is already LOW from setup() )           
                 sync_on[rgb] = 1;
                 interrupts();
               }                
             }  
             break;
             
          case PULSEOFF:
             if( flag[rgb] == 0 )  // wait for OFF params to 'take'         
             {
               pulse_counter[rgb]--;
               // if we are still doing a string of pulses OR we have no off time
               // then swing around
               if( pulse_counter[rgb] > 0 )
               {
                 noInterrupts();
                 ledon[rgb] = 1;
                 flag[rgb] = 1;
                 next_timer[rgb]= pulse_on_msec[rgb];
                 interrupts();                
                 pulse_state[rgb] = PULSEON;                 
               }   
               else  // done counting and need off time
               {
                  if( off_time_msec != 0 )
                  {
                    noInterrupts();
                    ledon[rgb]= 0;
                    flag[rgb] = 1;
                    next_timer[rgb] = off_time_msec[rgb];
                    interrupts();                
                    pulse_state[rgb] = OFFTIME;  
                    pulse_counter[rgb] = pulse_count[rgb];    // reset pulse counter for next go round  
                  }
                  else
                  {
                    pulse_state[rgb] = OFFTIME;
                  }  
               } // endif counting or offtime
             }             
             break;      
         
         case OFFTIME:
             if( flag[rgb] == 0 )  // wait for OFF params to 'take'
             {
                digitalWrite(LEDpin, LOW);             
                markerOff(rgb);
              
                iteration[rgb]++;   // count number of iterations of pulse trains
                if( (iteration_loops[rgb] == 0) || (iteration[rgb] < iteration_loops[rgb]) )  // still have more to do, loop back to pulse on
                {
                   noInterrupts();
                   ledon[rgb] = 1;
                   flag[rgb] = 1;
                   next_timer[rgb] = pulse_on_msec[rgb];
                   interrupts();         
                   pulse_counter[rgb] = pulse_count[rgb];         
                   pulse_state[rgb] = PULSEON;  
                }   
                else  // pulses done - go to step off time or hold if end of experiment
                {
                   if(xIdx == 0) //  || (xStep[xIdx].type == 0) )   // xIdx is 0 if not running an experiment (or it is over)                
                   {
                     //  xIdx  = 0;  // if end of experiment, reset pointer 
                       pulse_state[rgb] = HOLD;
                   }     
                   else
                   {
                      pulse_state[rgb] = STEPWAIT;
                   }
                }   
            }       
            break; 
         
         case STEPWAIT:
            if( (millis() - stepStart ) >= stepDuration )
            {   
               
#ifdef DEBUG               
    Serial.println("++++++++++++++++++++++++");
     Serial.println( millis() - stepStart);
    Serial.print("Step:");
    Serial.print(xIdx);
    Serial.print(" is ");
    Serial.println(xStep[xIdx].type);
#endif  
              
               xIdx++; //  point to next  
               if(xStep[xIdx].type == PULSE_TYPE) 
               {
                   for ( uint8_t rgb = 0; rgb < 3; rgb++)
                      newIntensity( 0.0, rgb, 0, 0x0f );  // turn off all colors before setting pattern                 
              
                   if( xStep[xIdx].usePattern )  // set the pattern - this requires enabling active quadrants
                   {
                        uint16_t bitpat = xStep[xIdx].pattern;
                        uint8_t quadbits;
                        for( uint8_t pan = 1; pan < 5; pan++)
                        {
                            quadbits = bitpat & 0x0f; 
                            bitpat = bitpat >> 4;      
                            setOnOff( pan, quadbits, 1 );
                        }    
                   } 
                   else
                   {
                         setOnOff( 0, 0x0f, 1 );   // set all active   
                   }
                   
                   setOnOff(0,0,0); // now turn everything off before setting intensities             
                   
                   for( int i = 0; i < 3; i++)
                   {
                      newIntensity( xStep[xIdx].intensity[i], i, bowls,0x0f);
                      pulse_count[i] =  xStep[xIdx].pulseCount[i];
                      pulse_on_msec[i] = xStep[xIdx].pulseOn[i];
                      pulse_off_msec[i] = xStep[xIdx].pulseOff[i]; 
                      off_time_msec[i] = xStep[xIdx].offTime[i];
                      iteration_loops[i] = xStep[xIdx].iterations[i];
//                      wait_sec[i] = uint32_t (xStep[xIdx].delayTime);
//                      wait_msec[i] = uint32_t( (xStep[xIdx].delayTime - wait_sec[i])*1000 );   
                      wait_msec[i] = uint32_t (xStep[xIdx].delayTime * 1000.0) ;                  
                      wait_cnt[i] = 0;
                      frame_on_msec[i] = 0;
                      frame_off_msec[i] = 0;
                      
                      if( xStep[xIdx].intensity[i] > 0.001)
                      {
                        pulse_enabled[i] = 1;
                        pulse_state[i] = START;
                      }   
                      else
                      {
                        pulse_enabled[i] = 0;
                        markerOff(i);
                        pulse_state[i] = HOLD;
                      }
                      
                   }  
                   stepDuration = xStep[xIdx].duration * 1000;                      
               }
               else  // id valid == false, then we are done
               {
                  for( int i = 0; i < 3; i++)
                  {
                     markerOff(i);
                     pulse_state[i] = HOLD;
                  }
                  xIdx = 0;  // show we are done   
                 // syncTimer.end();        // if auto trigger off is wanted      
               }         
            }                    
            break;
                   
        
         case HOLD: // single pulse mode leaves us here to hang out until next RUN command
            // markerOff(rgb); // for( uint8_t rgb = 0; rgb < 3; rgb++)  markerOff(rgb);  // leds and markerState should be off here
             break;
         
         default:
            break;
        } // endswitch              
        
    } // endif pulses enabled
    else
    {
   
     flag[rgb] = 0;  // don't test anthing in timer interrupt 
     sync_on[rgb] = 0; 
     frame_on_cnt[rgb] = frame_on_msec[rgb];
     frame_off_cnt[rgb] = 0;
     sync_on[rgb] = 1;     
    }  
  }//  next rgb 


#ifndef SYNC_PIN_OUT

#ifdef USE_EXT_TRIG   
     if( digitalRead(SYNCpin) == HIGH)
     {
          allPanelsOn();
         // LEDsOn();
     } 
     else
     {
          allPanelsOff();
          LEDsOff(); 
     }
#endif     

#endif

}

/*
addonestep 1,0.00,500,1000,1,2000,10,3.00,500,1000,1,2000,3,0.00,500,1000,1,2000,10,5,20

addonestep 1,60.00,1000,1000,1,2000,10,60.00,1000,1000,1,2000,10,60.00,1000,1000,1,2000,10,2.00,35.00
addonestep 2,0.00,2000,2000,1,4000,2,60.00,2000,2000,1,4000,2,0.00,2000,2000,1,4000,2,5.00,30.00
addonestep 3,60.00,100,200,10,3000,6,0.00,1000,2000,1,3000,1,60.00,1000,2000,1,3000,1,0.00,30.00


runExperiment
getexperimentstatus
StopExperiment
*/
