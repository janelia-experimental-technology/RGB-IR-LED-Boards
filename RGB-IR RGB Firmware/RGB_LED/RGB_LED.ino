// RGB LED board with individual DAC per quadrant 
// HHMI Janelia jET SWS
// 20200728

// processor: Teensy 3.2 or 4.0 or LC, though LC will have limited memory for experiment steps
// LC and 4.0 require 3.3V RGB board 
// Arduinio IDE: 1.8.13 with Teensyduino

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
 // ADDONESTEP step#, red%, redOn, redPrd, redCnt, redOff, redIter, grn%, grnOn, grnPrd, grnCnt, grnOff, grnIter, blu%, bluOn, bluPrd, bluCnt, bluOff, bluIter, delay, dur

// addonestep 1  0   0    0 0    0 0  3 200 500 4 1000  1 0 0   0   0 0    0 2 5
// addonestep 1  3 200 500  4 1000 1  0  0   0  0 0     1 0 0   0   0 0    0 2 5
// addonestep 1  0   0   0  0    0 0  0  0   0  0 0     0 3 200 500 4 1000 1 2 5
// addonestep 1,1.00,5000,5000,1,20000,1,0.00,0,0,0,0,0,0.00,0,0,0,0,0,20.00,50000.00

// addonestep 1 0 30000 30000 1 0 1  0 0 0 0 0 0 0 0 0 0 0 0 20 50
// addonestep 2 2 20000 20000 1 0 1  0 0 0 0 0 0 0 0 0 0 0 0 0 20


//  PULSE  a,b,c,d,e, f, C
//    a - pulse width - msec
//    b - pulse period - msec
//    c - # pulses -
//    d - off time
//    e - wait time - floating seconds
//    f - number of iterations; 0 = continuous
//    C - color channel ('R', 'G', 'B' and/or 'S' or 'D')

//#define DEBUG   // print out debug info
//#define SYNC_PIN_OUT    // if defined, then SYNC pin is set up as an output,else it is a trigger in
//#define SYNC_ACTIVE_LOW  // marker pin is active low
//#define EN5to8  // creates 8 'quadrants' on main board - used for training rig for Yoshi - interferes with digital out 
//#define PWMCONTROL   // use external PWM control to run brightness
//#define ANALOGCONTROL
//#define SERVOCONTROL  // fro non-daisy chain rigs, can use TX out for servo control

// VERSIONS
//
#define VERSION 20240813

// 20240813 sws
// - add servo control as alternate use of interboard poll TX pin. Can only be used in stand alone application

// 20240624 sws
// - MARKER cmd didn't actually update the digital pins if 'D' used

// 20240606 sws
// - UPDATE - MARKER colors ON or OFF
//   add On or OFF to turn specific marker colors (and/or digital) on and off withour affecting the others
//   no arguments will return current on markers
// -xxx change MARKER from blinking each marker for one second to turning markers on and off 
//   xxx   example: MARKER RG  will turn on red and green and turn off blue

// 20240430 sws
// - initial setting of cmdMode PARALLEL not SERIES to easier work with FPGA control 

// 20231218 sws
// - a step with no pulse widths > 0 would go on forever. So now, if a pulse period is specified and width is 0, run
//   the pulse steps to make the experiment step work correctly

// 20231120 sws
// - add set flag to 0 in stopExperiment - before, after stop, would still LEDs on

// 2023115 sws
// - commented out filter stuff
// - add license info

// 20230531 sws
// - broaden sync cmd to 5000 to 0.01 hz
// - use SYNCON to start it or it will start with a runexperiment
// - change so it starts with a high

// 20230425 sws
//  - only run clock out if set up via the syncRate command with value > 0
//    this allows us to use it alternatively as a start of experiment trigger 
//  - if IR is manually set to 0 ('IR 0') then set a flag to have IR envelope the experiment (on at start, off at end)

// 20221018 sws
// add conditional compiles for LC (__MKL26Z64__) : 
//    no WDT for now, 
//    low RAM requires limits on number of experimental steps and step order

// 20220831 sws
// - PWMCONTROL not reliable, green misses rising edge interrupt at times
//   go back to analog control

// 20220830 sws
// - add external PWM control - use #define PWMCONTROL to compile this feature in

// 20220615 sws
//  - in initBoards, reset onQuadrants to 0
//  - in runCmd , add onQuadrants = 0x0f
//  - in colorset , don't use intensityPC, use intensity so we can check each quadrant, add loop to do that

// 20220614  sws
// - in newIntensity, do not propogate intensity if just main panel

// 20220606 sws
// - forgot to make the change from 0601 in the initial runexperiment for first step

// 20220601 sws
// - when running Steps, instead of using intensity > 0 as a criterion to run a color, 
//   use pulse width > 0. This allows a step to run with no color at all. 

// 20200518 sws
// - added STATE commamd to view current pulse state for each color
// - after last pulse on time, turn off markers (don't envelope the pulse period)

// 20220510 sws
// - blue offtime in addExperimentStep was using green ontime, not blue
// - add GS command to get experiment steps


// 20220509 sws
// - add STEPORDER command to allow custom running of experiment steps

// 20220429 sws
// - MAXBLINKTIME 32 secs not 5 secs
// - compile for Teensy 4.0
//     need to comment out: s->println( (RCM_SRS0 & 0x20) >> 5 );
// - add SX and GX commands (short forms for stopExperiment and getExperimentStatus)
// - had wrong arg for blink green and blue
// - if blink sent before runExperiment, then that blink will happen at the start of each step

// 20220413 sws
// - added PMWin command - allows SYNC pin (SMA under CPU) to be used to change brightness of the chosen
//    color using PWM signal - set to work at 48Khz rate 0-95%
//    NOTE : using analog in for now - need to figure out how to stop LEDs when no PWM if using interrupt in
// - to do the above I no longer set SYNC pin as output at boot (undefine SYNC_PIN_OUT) and set it 
//   in the SYNC command

// 20220405 sws
// - runexperiment was setting up 4 colors not 3 (i<4)
// 20220314 sws
// - add BLINK command

// 20220202 sws
// - add LED offset to get even LED brightness at low levels

// 20211117 sws
// - clean up and getting code working properly with changes. 
// - Markers now work with ON and OFF, not just pulse
// - if number of argumenst in PULSE command too few, show argument list

// 20211027 sws
// - finished first shot at digital code - need to test
// - removed PARALLEL intra board cmd. After enumeration, each board sets itself back to Parallel automatically
//     this frees up a command needed for digital marker off
// - added DIGITAL (or DIG) command followed by bit pattern for digital outs
// - added DON and DOFF commands to turn digitals on and off


// 20211020 sws
// - making changes to support new digital board design
// - need to share enable 7 and 8 pins with digital out 1 and 3
//  EN5to8 used to allow them as chip selects


// 20211014 sws
// - add digital output pins using unused Teensy pins
//  the digital board will sandwich between the Teensy and the RGB board
//  redo the marker commands to free up three digital commands - set digital, digital on, digital off
// - add 'D' for digital; will work the same as 'S' for shock


// 20210325 sws
// - add 'S' as fourth 'color' to PULSE command for shock setup
// - add STARTSHOCK and STOPSHOCK comamnds to start and stop shock

// 20210318 sws
// - fixed more problems with pulse in 100% duty cycle - was running twice as long as it should ( needed count check <= 0 not  <0 )
// - Pulse command delay not working - only was counting msec part of value - so 5 seconds gave 0 msec, add sec * 1000 to value

// 20210316 sws
// - fixed problems in pulses with 100% duty cycle

// 20210314 sws
// - remove analogWrite for markers
// - extend marker commands out to downstream panels - use unused panel comamnds

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
// - for colorsSet, check that DAC value is less than 0x800 (max allowed) so we can use higher values for
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
#ifndef __MKL26Z64__
 #include <Watchdog.h>
#endif

#include <EEPROM.h>
#ifdef SERVOCONTROL
#include <PWMServo.h>
PWMServo rgbServo;
#endif
//#include <Filters.h>
//#include <Filters/MedianFilter.hpp>        // MedianFilter

IntervalTimer pulseTimer;
IntervalTimer syncTimer;
IntervalTimer blinkTimer;



#ifndef __MKL26Z64__
  Watchdog watchdog;
#endif  

//MedianFilter<7,uint16_t> medfilt = {0};

#define BLUDAC 1
#define GRNDAC 2
#define REDDAC 4
#define IRDAC  8
#define DIGOUT 8

#define BLUE 0
#define BLU 0
#define GREEN 1
#define GRN 1
#define RED 2
#define IR 3
#define SHOCK 3
#define DIG 3
#define DIGITAL 3

#define REDBIT 4
#define GRNBIT 2
#define BLUBIT 1

int8_t blinkColor = 0;


//#define LED_BOARD 1     // if pin23 is floating (existing LED boards) we get this
//#define DIGITAL_BOARD 0  // digital boards nust ground pin 23

char colors[4][5] = {"BLU ", "GRN ", "RED ", "IR  "}; // text to show what color number is
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
#define redPWMpin 7
#define grnPWMpin 8
#define en5Pin   7
#define en6Pin   8
#define en7Pin   9
#define fanPin  10   // depracted
#define Dout2Pin 9
#define Dout1Pin 10
#define Dout3Pin 21
#define Dout4Pin 22
#define MOSIpin 11
#define MISOpin 12 // not used, but reserved for SPI
#define SCLKpin 13
#define OTMPpin 14
#define LLpin   15
#define URpin   16
#define LRpin   17
#define ULpin   18
#define SYNCpin 19
#define LEDpin  20
#define en8Pin  21
// #define boardTypePin 23
// #define shockPin 24  // shock marker
#define digitalPin 23   // digital marker 


//--- pulse states---
#define OFFSTATE 0
#define START    1
#define WAIT     2
#define PULSEON  3
#define PULSEOFF 4
#define OFFTIME  5
#define CONTINUOUS 6
#define STEPWAIT 7
#define HOLD     8
#define TURNOFF  9
// ------------------

char states[10][5] = {"off ", "strt", "wait", "pon ", "poff", "offt", "cont", "step", "hold","toff"};

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
#define MINSYNC 0.01 // 1
#define MAXSYNC 5000

float syncRate = 30;

int32_t pulse_on_msec[4];
int32_t pulse_off_msec[4];
int32_t off_time_msec[4];
int32_t pulse_count[4];
int32_t wait_sec[4];
int32_t wait_msec[4];
int32_t wait_cnt[4];
int32_t frame_on_msec[4];
int32_t frame_off_msec[4];
uint16_t iteration_loops[4] = {0, 0, 0, 0};
uint32_t stepDuration;
uint32_t stepDelay;

uint32_t experimentMsecs;

uint8_t bowls = 0;

uint16_t intensityPC[4] = {0, 0, 0, 0};

// intensity values for each color [4] by number of quads (was 4 now 8)
uint16_t intensity[4][8] =
{ 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};

// need intensity for each quadrant - save as DAC value
//uint16_t intensityDAC[4,4] = 
//  { 
//    0,0,0,0,
//    0,0,0,0,
//    0,0,0,0,
//    0,0,0,0
//  };
//
//uint16_t lastIntensityDAC[4,4];  

uint16_t iteration[4];

uint8_t digitalBits;  // bit pattern for digital on this board

int xIdx = 0;  // index into experiment table

//elapsedMillis pulse_timer;
int32_t pulse_counter[4];
int pulse_enabled[4];

uint16_t frame_on_cnt[4];
uint16_t frame_off_cnt[4];

volatile int32_t next_timer[4];
volatile int8_t flag[4] = {0, 0, 0, 0};
volatile int8_t ledon[4] = {0, 0, 0, 0};
volatile int32_t pulse_timer[4];
volatile int8_t  sync_on[4];

boolean markerState[] = {false, false, false, false};

uint8_t onQuadrants = 0x00;  // track which quadrants are turned on - default to none; bit pattern

uint8_t onColors[] = {0x00, 0x00, 0x00 };  // which colors are set to 0% - Blue, Green, Red , bit pattern for quadrants (up to 8)

boolean colorsAreOn = false;

int pwmStatus = -1;   // -1 == off, else which color

int pulse_set[4] = {0, 0, 0, 0};
int pulse_state[4] = {START, START, START, START };
int last_state[4] = { HOLD, HOLD, HOLD, HOLD };
int markerPin[4] = { bluPin, grnPin, redPin, digitalPin };

int8_t err = 0;

//volatile uint16_t pattern[4] = { 0, 0 , 0 };
volatile uint16_t pattern =  0;

boolean digitalState = false;

static uint8_t inited = 0;
static uint8_t thisPanel = 0;

volatile boolean state = false;

uint16_t intensityLON = 0;
uint16_t intensityMON = 0;
uint16_t intensityHON = 0;

//uint8_t markerIntensity = 255;

float LEDgain[4][4] =
{
  1.0, 1.0, 1.0, 1.0,  // blue
  1.0, 1.0, 1.0, 1.0,  // green
  1.0, 1.0, 1.0, 1.0,  // red
  1.0, 1.0, 1.0, 1.0   // IR
};

uint16_t LEDoffset[4][4] =
{
  0, 0, 0, 0,  // blue
  0, 0, 0, 0,  // green
  0, 0, 0, 0,  // red
  0, 0, 0, 0   // IR
};

#define OFFSET_ADR sizeof(LEDgain)

int EE_INITED = sizeof(LEDgain) + sizeof(LEDoffset);  // address of init code 

boolean bowlEnabled[4] = {false, false, false, false };

boolean wdtReset = false;

boolean clockOutEnabled = false;  // true clock out on trigger Pin enabled via SyncCmd
boolean IRwasOff = false;   // track if IR was set to off before running an experiment
float lastIRpercent = 0;     // keep track of last non-zero IR brightness setting

SPISettings DAC(2000000, MSBFIRST, SPI_MODE2);
SPISettings DOUT(1000000, MSBFIRST, SPI_MODE0);

//elapsedMillis blinkTimer;
uint16_t blinkTime = 0;
#define MAXBLINKTIME 32000

float readFloat(unsigned int addr)
{
  union
  {
    byte b[4];
    float f;
  }  data;
  for (int i = 0; i < 4; i++)
  {
    data.b[i] = EEPROM.read(addr + i);
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
  for (int i = 0; i < 4; i++)
  {
    EEPROM.write(addr + i, data.b[i]);
  }
}

uint16_t readUint(unsigned int addr)
{
  union
  {
    byte b[2];
    uint16_t u;
  }  data;

  data.b[0] = EEPROM.read( addr);
  data.b[1] = EEPROM.read( addr+1);
  return data.u;
}

void writeUint(unsigned int addr, uint16_t x)
{
  union
  {
    byte b[2];
    uint16_t u;
  } data;
  data.u = x;
  EEPROM.write( addr, data.b[0]);
  EEPROM.write( addr+1, data.b[1]);
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
#define PARALLEL    0xF5   // command not used, but indicates comm state
#define COLORSET    0xF8
#define RMARKOFF    0xF0
#define GMARKOFF    0xF2
#define BMARKOFF    0xF6
#define DMARKOFF    0xF5
#define DIGITALON   0xF1
#define DIGITALOFF  0xF3
#define SETDIGITAL  0xF7


uint8_t cmdMode = PARALLEL;

//uint8_t markerOnCmd[] = { BMARKON, GMARKON, RMARKON };
uint8_t markerOffCmd[] = { BMARKOFF, GMARKOFF, RMARKOFF, DMARKOFF};

#define PANELBITS  0x30
#define COLORBITS  0x03
#define QUADBITS   0x0c

#define ALLPANELS 0x00
#define ALLQUADS  0x0f


// ===============================
// ===  D I G I T A L   S E T  ===
// ===============================

// Turn digital pins on and off

void digitalSet(boolean state)
{

  digitalState = state;

  if ( state )
  {
    sendCmd(DIGITALON);
#ifdef DEBUG
    Serial.print("bits:");
    Serial.println(digitalBits, BIN);
#endif
    if ( digitalBits & 0x01 ) digitalWriteFast(Dout1Pin, HIGH); else digitalWriteFast(Dout1Pin, LOW);
    if ( digitalBits & 0x02 ) digitalWriteFast(Dout2Pin, HIGH); else digitalWriteFast(Dout2Pin, LOW);
    if ( digitalBits & 0x04 ) digitalWriteFast(Dout3Pin, HIGH); else digitalWriteFast(Dout3Pin, LOW);
    if ( digitalBits & 0x08 ) digitalWriteFast(Dout4Pin, HIGH); else digitalWriteFast(Dout4Pin, LOW);
    markerOn(DIGITAL);
  }
  else
  {
    sendCmd(DIGITALOFF);
    digitalWriteFast(Dout1Pin, LOW);
    digitalWriteFast(Dout2Pin, LOW);
    digitalWriteFast(Dout3Pin, LOW);
    digitalWriteFast(Dout4Pin, LOW);
    //    markerOff(DIGITAL);
    
  }
}

int8_t quad2cspin[] = {ULpin, URpin, LRpin, LLpin, en5Pin, en6Pin, en7Pin, en8Pin };

// ===========================
// ===  C O L O R   S E T  ===
// ===========================

// Turn on selected colors, turn others off

void colorSet(int8_t colors)
{

#ifdef DEBUG
  Serial.print("    COLORSET ");
  Serial.print(colors);
  Serial.print(" onquads= ");
  Serial.println(onQuadrants);
#endif

  // First turn everything (but IR) off
  SPI.beginTransaction(DAC);
  digitalWriteFast(ULpin, LOW); // select quadrant DACs
  digitalWriteFast(URpin, LOW);
  digitalWriteFast(LRpin, LOW);
  digitalWriteFast(LLpin, LOW);
#ifdef EN5to8
  digitalWriteFast(en5Pin, LOW);
  digitalWriteFast(en6Pin, LOW);
  digitalWriteFast(en7Pin, LOW);
  digitalWriteFast(en8Pin, LOW);
#endif
  SPI.transfer(0x20);            // pwr up/down command
  SPI.transfer(0x00);
  SPI.transfer(0x17);            // power down RGB with 1K to GND, but not IR
  delayMicroseconds(1);
  digitalWriteFast(ULpin, HIGH); // stop transmitting
  digitalWriteFast(URpin, HIGH);
  digitalWriteFast(LLpin, HIGH);
  digitalWriteFast(LRpin, HIGH);
#ifdef EN5to8
  digitalWriteFast(en5Pin, HIGH);
  digitalWriteFast(en6Pin, HIGH);
  digitalWriteFast(en7Pin, HIGH);
  digitalWriteFast(en8Pin, HIGH);
#endif
  SPI.endTransaction();
  colorsAreOn = false;  // assume we want them off

  if ( colors != 0 ) // only turn colors on if any are asked for
  {
    for( int quad = 0; quad < 4; quad++ )  // must set each quadrant separately as they may have different intensities
    {
        uint8_t colorOn = 0x00;
        colors |= 0x08; // always have IR on
        // now set the selected quadrants on but only if the brightness is > 0.01%
        SPI.beginTransaction(DAC);
        // do on colors
        if( onQuadrants & (1 << quad))
        {
            digitalWriteFast(quad2cspin[quad], LOW); // select quadrant DACs
        }
        delayMicroseconds(1);
        SPI.transfer(0x20);           // pwr up/down command
        SPI.transfer(0x00);
        if ( (intensity[RED][quad] > 1) && (intensity[RED][quad] < 0x800) ) colorOn |= 0x04; // > 0x7ff for colors is aux
        if ( (intensity[BLU][quad] > 1) && (intensity[BLU][quad] < 0x800) ) colorOn |= 0x01;
        if ( (intensity[GRN][quad] > 1) && (intensity[GRN][quad] < 0x800) ) colorOn |= 0x02;
        if ( intensity[IR][quad]  > 1 ) colorOn |= 0x08;
        colorOn = colorOn & colors;  // only enable desired colors that have intensity set
    #ifdef DEBUG
        Serial.print(" CSon ");
        Serial.print(quad2cspin[quad]);
        Serial.print(" coloron ");
        Serial.println(colorOn, BIN);    
    #endif
        SPI.transfer(colorOn);
        delayMicroseconds(1);
        digitalWriteFast(quad2cspin[quad], HIGH); // stop transmitting
        SPI.endTransaction();
    
        if(colorOn & 0x01) markerOn(BLU);
        if(colorOn & 0x02) markerOn(GRN);
        if(colorOn & 0x04) markerOn(RED);
    
        colorsAreOn = true;
    } // end quad for loop  
  }
}


// ========================
// ===  L E D S   O N   ===
// ========================

void ledsOn(uint8_t quadrant)
{

#ifdef DEBUG
  Serial.println("    LEDS ON");
  Serial.print(millis());
  Serial.print(" LEDSON q: 0b");
  Serial.print(quadrant, BIN);
  Serial.print(" on ");
  Serial.println(intensityPC[RED]);
#endif


  if ( quadrant == 0 ) quadrant = 0xff;

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

  if ( quadrant == 0 ) quadrant = 0xff;

  //  if(quadrant == 0) quadrant = 0x0f; // 0 is all

  colorSet(0x00);  // turn off all colors

}


// ===================================
// ===  A L L   P A N E L S  O N   ===
// ===================================

void allPanelsOn(void)  // for now ignore rgb
{
  //   sendCmd(SETON | ALLPANELS | ALLQUADS);
  #ifdef DEBUG
     Serial.println("   ALL PANELS ON");
  #endif   
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



// ===========================
// ===  I N I T   D O U T  ===
// ===========================


void initDOUT( void)
{
  digitalWriteFast( Dout1Pin, LOW);
  digitalWriteFast( Dout2Pin, LOW);
  digitalWriteFast( Dout3Pin, LOW);
  digitalWriteFast( Dout4Pin, LOW);
}

// ==== END MCP32S17 ==========


// ===========================
// ===  I N I T   D A C S  ===
// ===========================

void initDACs( void)
{
  // sw reset
  SPI.beginTransaction(DAC);
  digitalWriteFast(ULpin, LOW);  // select quadrant DACs
  digitalWriteFast(URpin, LOW);
  digitalWriteFast(LRpin, LOW);
  digitalWriteFast(LLpin, LOW);
#ifdef EN5to8
  digitalWriteFast(en5Pin, LOW);
  digitalWriteFast(en6Pin, LOW);
  digitalWriteFast(en7Pin, LOW);
  digitalWriteFast(en8Pin, LOW);
#endif
  SPI.transfer(0x28);            // sw power-on reset
  SPI.transfer16(0x0001);
  delayMicroseconds(1);
  digitalWriteFast(ULpin, HIGH); // stop transmitting
  digitalWriteFast(URpin, HIGH);
  digitalWriteFast(LLpin, HIGH);
  digitalWriteFast(LRpin, HIGH);
#ifdef EN5to8
  digitalWriteFast(en5Pin, HIGH);
  digitalWriteFast(en6Pin, HIGH);
  digitalWriteFast(en7Pin, HIGH);
  digitalWriteFast(en8Pin, HIGH);
#endif
  SPI.endTransaction();

  // turn on internal references on DACs
  SPI.beginTransaction(DAC);
  digitalWriteFast(ULpin, LOW);  // select quadrant DACs
  digitalWriteFast(URpin, LOW);
  digitalWriteFast(LRpin, LOW);
  digitalWriteFast(LLpin, LOW);
#ifdef EN5to8
  digitalWriteFast(en5Pin, LOW);
  digitalWriteFast(en6Pin, LOW);
  digitalWriteFast(en7Pin, LOW);
  digitalWriteFast(en8Pin, LOW);
#endif
  SPI.transfer(0x38);            // enable reference  xx11 1xxx xxxx xxx1
  SPI.transfer16(0x0001);
  delayMicroseconds(1);
  digitalWriteFast(ULpin, HIGH); // stop transmitting
  digitalWriteFast(URpin, HIGH);
  digitalWriteFast(LLpin, HIGH);
  digitalWriteFast(LRpin, HIGH);
#ifdef EN5to8
  digitalWriteFast(en5Pin, HIGH);
  digitalWriteFast(en6Pin, HIGH);
  digitalWriteFast(en7Pin, HIGH);
  digitalWriteFast(en8Pin, HIGH);
#endif

  SPI.endTransaction();
  // update DAC when new DAC value is received
  SPI.beginTransaction(DAC);
  digitalWriteFast(ULpin, LOW);  // select quadrant DACs
  digitalWriteFast(URpin, LOW);
  digitalWriteFast(LRpin, LOW);
  digitalWriteFast(LLpin, LOW);
#ifdef EN5to8
  digitalWriteFast(en5Pin, LOW);
  digitalWriteFast(en6Pin, LOW);
  digitalWriteFast(en7Pin, LOW);
  digitalWriteFast(en8Pin, LOW);
#endif
  SPI.transfer(0x30);            // update DAC
  SPI.transfer16(0x000f);        // on load
  delayMicroseconds(1);
  digitalWriteFast(ULpin, HIGH); // stop transmitting
  digitalWriteFast(URpin, HIGH);
  digitalWriteFast(LLpin, HIGH);
  digitalWriteFast(LRpin, HIGH);
#ifdef EN5to8
  digitalWriteFast(en5Pin, HIGH);
  digitalWriteFast(en6Pin, HIGH);
  digitalWriteFast(en7Pin, HIGH);
  digitalWriteFast(en8Pin, HIGH);
#endif
  SPI.endTransaction();
}


// =============================
// ===  C H E C K   I N I T  ===
// ==============================

void checkInit(void)
{
  if ( inited == 0 )
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

// ====================================
// ===  S E T   I N T E N S I T Y   ===
// ====================================

// set new color for one quadrant: 0-3 now 0-7
// if needing to change all quadrants, need to call four times

void setIntensity(uint8_t color, uint8_t quadrant, uint16_t DACvalue)
{

  uint8_t cspin = quad2cspin[quadrant];
  uint8_t colorBit = color2DAC[color];
  
  // add gain and offset
  if ( quadrant < 4 )
    DACvalue = int( (float)DACvalue * LEDgain[color][quadrant] + LEDoffset[color][quadrant]);
  if ( color == IR)
  {
    if ( DACvalue > MAXIRDAC ) DACvalue = MAXIRDAC;
  }
  else
  {
    if ( DACvalue > MAXRGBDAC ) DACvalue = MAXRGBDAC;
  }
  
  intensityPC[color] = DACvalue;
//  intensity[color][quadrant] = DACvalue;

#ifdef DEBUG
  Serial.print("---- SET INTENSITY q:");
  Serial.print( quadrant);
  Serial.print(" c:");
  Serial.print(color, HEX);
  Serial.print(" was:");
  Serial.print(intensity[color][quadrant]);
  Serial.print(" cs:");
  Serial.print(cspin);
  Serial.print(" ad:");
  Serial.print( colorBit);
  Serial.print(" DAC: 0x");
  Serial.println(DACvalue, HEX);
#endif

  SPI.beginTransaction(DAC);
  digitalWriteFast( cspin, LOW);
  SPI.transfer( 0x18 | color);   // write and update DAC xx01 1aaa
  SPI.transfer16( DACvalue << 4);
  delayMicroseconds(1);
  digitalWriteFast( cspin, HIGH);   // stop transmitting
  SPI.endTransaction();

  if ( DACvalue == 0 ) // if we set to 0% , use power down to turn off
  {
#ifdef DEBUG
    Serial.println("turn off");
#endif
    SPI.beginTransaction(DAC);
    digitalWriteFast( cspin, LOW);
    SPI.transfer(0x20);            // pwr up/down command
    SPI.transfer(0x00);
    SPI.transfer(0x10 | colorBit);  // power down this color
    delayMicroseconds(1);
    digitalWriteFast( cspin, HIGH); // stop transmitting
    SPI.endTransaction();
  }
  else
  {
    uint8_t quad = (1 << quadrant);
    boolean turnOn = false;
    // if it was zero, and it is enabled, we need to turn it on , OR if IR, always turn on
    if ( (intensity[color][quadrant] == 0) && (onQuadrants & quad) && (colorsAreOn == true) ) turnOn = true;
    if ( color == IR ) turnOn = true;
    if ( turnOn)
    {
#ifdef DEBUG
      Serial.print("turn on ");
      Serial.println(colorsAreOn);
#endif
      SPI.beginTransaction(DAC);
      digitalWriteFast( cspin, LOW);
      SPI.transfer(0x20);             // pwr up/down command
      SPI.transfer(0x00);
      SPI.transfer(colorBit);         // power up this color
      delayMicroseconds(1);
      digitalWriteFast( cspin, HIGH); // stop transmitting
      SPI.endTransaction();
    }
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

  if (color == IR )
  {
    DACval = (uint16_t) ( (percent * MAXIRDAC) / 100.0);
    if ( DACval > MAXIRDAC ) DACval = MAXIRDAC;
  }
  else
  {
    DACval = (uint16_t) ( (percent * MAXRGBDAC) / 100.0);
    if ( DACval > MAXRGBDAC ) DACval = MAXRGBDAC;
  }
  //      if( DACval == 1 ) DACval = 2;  // reserve 1 for digital out
  //    }
  //    else
  //    {
  //       DACval = 1; // digital setup
  //    }
#ifdef DEBUG
  Serial.println("--- newIntensity");
  Serial.print(colors[color]);
  Serial.print(" Intensity %");
  Serial.print(percent);
  Serial.print(", pan:");
  Serial.print(panel);
  Serial.print(", quad: 0b");
  Serial.print(quadrant, BIN);
  Serial.print(", DAC");
  Serial.println(DACval);
  Serial.print("onQuads= 0b");
  Serial.println(onQuadrants, BIN);
#endif

  if( panel != 1 )  // if only this panel, don't propogate
  {
    sendCmd(INTENSITYHON | ((DACval & 0x0f00) >> 8) );
    sendCmd(INTENSITYMON | ((DACval & 0x00f0) >> 4) );
    sendCmd(INTENSITYLON | ( DACval & 0x000f));
  }  

#ifdef EN5to8
  int maxQuad = 8;
#else 
  int maxQuad = 4;
#endif

  for ( uint8_t quad = 0; quad < maxQuad; quad++) // host can have 8 'quadrants' in some setups
  {
    if ( quadrant & (1 << quad)) // check each  quadrant
    {
      if ( (panel != 1) && (quad < 4) ) // don't relay commands that are only for host panel (#1)
      { // also, remote panels can only have 4 quadrants
        uint8_t tpanel = panel;
        if ( tpanel != 0 ) tpanel--; // translate 2-4 to 1-3 for remote cmds
        sendCmd(SETCOLOR | (tpanel << 4) | (quad << 2) | color);
      }
      if ( panel <= 1 ) // host panel or all - so do local update
      {
        setIntensity(color, quad, DACval);
      }
    }
  }  // next quadrant
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


  if ( arg_cnt > 3 )
  {
    if ( strchr(args[1], 'R') )
    {
      color = RED;;
    }
    else if ( strchr(args[1], 'B') )
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

    quadrant = cmdStr2Num(args[2], 10);

    DACval = cmdStr2Num(args[3], 10);

    //    if ( digitalReadFast( boardTypePin) == LED_BOARD)
    //    {
    setIntensity( color, quadrant, DACval);
    //    }
    //    else
    //    {
    //      setDigitalOut( color, quadrant, DACval);
    //    }
  }

}


// ==================================
// ===  D O    I N T E N S I T Y  ===
// ==================================

//  given a color, read rest of comamnd line to get brightness, panels, and quads

int8_t doIntensity( int arg_cnt, char **args, uint8_t color)
{
  // assume we want it all
  uint8_t quadrant = 0xff; // assume all quads
  uint8_t panel = 0;       // assume all panels
  float percent = 0;
  //  uint16_t DACval;

  #ifdef DEBUG
     Serial.println("---- doIntensity");
  #endif

  checkInit();

  if ( arg_cnt > 1 ) // then we have an intensity
  {

    percent = cmdStr2Float(args[1]);

    if( color == IR)
    {
      if (percent > 1) 
          lastIRpercent = percent;  // remember last 'on' value
      else
          IRwasOff = true; // note that an IR 0 command was sent- this will trigger IR on during experiment
    }
    
    if ( arg_cnt > 2 ) // then we have a panel
    {
      panel = cmdStr2Num(args[2], 10);
      if ( arg_cnt > 3 ) // then we have a quadrant
      {
        uint8_t tquad = cmdStr2Num(args[3], 2);
        //    if( tquad < 0x10 )
        quadrant = tquad;
        //    else
        //        return -3;
      }  // endif quadrants chosen
    }  // endif panel chosen
    // now - set the intensity
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
  if ( onoff)
    Serial.println(" ON");
  else
    Serial.println(" OFF");
#endif

  if (onoff == 0 )  // ---- 0 = 'off' command
  {

    if ( panel != 1 ) // only relay commands that include non-host panels
    {
      uint8_t tpanel = panel;
      if ( tpanel != 0 ) tpanel--; // translate 2-4 to 1-3 for remote cmds
      sendCmd(SETOFF | (tpanel << 4) | quadrant);
    }
    if ( panel <= 1 ) ledsOff(quadrant); // 0 or 1 includes main panel

  }
  else   //------ otherwise 'on' command
  {

    if ( panel != 1 ) //  only relay commands that include non-host panels
    {
      uint8_t tpanel = panel;
      if ( tpanel != 0 ) tpanel--; // translate 2-4 to 1-3 for remote cmds
      sendCmd(SETON | (tpanel << 4) | quadrant);
    }
    if ( panel <= 1 ) ledsOn(quadrant);   // 0 or 1 includes main panel
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
  s->print("\r\n DO ON OFF Parsed: c: ");
  s->print(arg_cnt);
  s->print(", p ");
#endif

  if ( arg_cnt > 1 )
  {
    uint8_t tpanel = cmdStr2Num(args[1], 10);

#ifdef DEBUG
    s->print(tpanel);
#endif

    if ( tpanel < 5 ) // user sees panels as 0 = all, 1 is host, then 2,3,4
      panel = tpanel;
    else
      return -2;

#ifdef DEBUG
    s->print(", q ");
#endif

    if ( arg_cnt > 2 )
    {
      //uint8_t tquad = cmdStr2Num(args[2], 2);
      uint8_t tquad = 0;
      char * qp = args[2];
      while ( (*qp == '1') || (*qp == '0') )
      {
        //  s->print(*qp);
        tquad = tquad << 1;
        if ( *qp == '1') tquad += 1;
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
  #ifdef DEBUG
    Serial.print("==== RED CMD ====");  
    for( int i = 1; i < arg_cnt; i++ )
    {
       Serial.print(" ");
       Serial.print(args[i]);   
    }   
    Serial.println();
  #endif  
  
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
  #ifdef DEBUG
  Serial.println("******** IR CMD *********");
  #endif
  doIntensity(arg_cnt, args, IR);
}

// ===============================
// ===  D I G I T A L   C M D  ===
// ===============================

// set new digital pattern

void digitalCmd(int arg_cnt, char **args)
{

  uint16_t pattern;

  checkInit();

  if ( arg_cnt > 1 ) // then we have a pattern
  {
    pattern = cmdStr2Num(args[1], 2);

    // set up main pattern
    digitalBits = pattern & 0x000f;
#ifdef DEBUG
    Serial.println( pattern, BIN);
#endif
    // then send out to downstream panels
    // bits are encoded in the color intensity slots
    pattern = pattern >> 4;
    sendCmd( INTENSITYLON | (pattern & 0x000f));
    pattern = pattern >> 4;
    sendCmd( INTENSITYMON | (pattern & 0x000f));
    pattern = pattern >> 4;
    sendCmd( INTENSITYHON | (pattern & 0x000f));
    // and tell the panels these are digital
    sendCmd( SETDIGITAL);

    if ( digitalState ) digitalSet(true); // update outputs to new pattern if on
  }

}


// ================================
// ===  G E T D A C S    C M D  ===
// ================================

void getdacsCmd(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("RED :");
  for ( int i = 0; i < 8; i++ )
  {
    s->print(intensity[RED][i]);
    s->print(" ");
  }
  s->print("\r\nGRN :");
  for ( int i = 0; i < 8; i++ )
  {
    s->print(intensity[GRN][i]);
    s->print(" ");
  }
  s->print("\r\nBLU :");
  for ( int i = 0; i < 8; i++ )
  {
    s->print(intensity[BLU][i]);
    s->print(" ");
  }
  s->print("\r\nIR  :");
  for ( int i = 0; i < 8; i++ )
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

  if ( arg_cnt == 4)
  {
    if ( args[1][0] == 'B' ) clr = BLU;
    else if (args[1][0] == 'G') clr = GRN;
    else if (args[1][0] == 'R') clr = RED;
    else if (args[1][0] == 'I') clr = IR;
    else
    {
      s->println("color?");
      return;
    }

    int idx = cmdStr2Num(args[2], 10);
    if ( (idx >= 0) && (idx <= 3) )
    {
      float tgain = cmdStr2Float(args[3]);
      if ( (tgain >= 0.8) && (tgain <= 1.2) )
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

// =========================
// ===  O F F S E T   C M D  ===
// =========================

void offsetCmd(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  int clr;

  //    s->println( arg_cnt);
  //    s->println( cmdStr2Num(args[1], 10) );
  //    s->println( cmdStr2Float(args[2]) );

  if ( arg_cnt == 4)
  {
    if ( args[1][0] == 'B' ) clr = BLU;
    else if (args[1][0] == 'G') clr = GRN;
    else if (args[1][0] == 'R') clr = RED;
    else if (args[1][0] == 'I') clr = IR;
    else
    {
      s->println("color?");
      return;
    }

    int idx = cmdStr2Num(args[2], 10);
    if ( (idx >= 0) && (idx <= 3) )
    {
      uint16_t toff = cmdStr2Num(args[3], 10);
      if ( (toff >= 0) && (toff <= 20) )
      {
        //    s->println( clr * 16 + idx * 4);
        //    s->println( tgain);
        writeUint( OFFSET_ADR + clr * 16 + idx * 4, toff);  // 4 bytes/float, 4 quads per color
        LEDoffset[clr][idx] = toff;
        //    s->println( LEDgain[clr][idx] );
      }
      else
      {
        s->println("offset?");
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
        s->print( LEDoffset[color][quad] );
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
  #ifdef DEBUG
    Serial.println("==== ON CMD");
  #endif  
  
  if ( arg_cnt > 1)
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
  #ifdef DEBUG
    Serial.println("==== OFF CMD");
  #endif  
  
  if ( arg_cnt > 1)
    doOnOff(arg_cnt, args, 0);
  else
    allPanelsOff(); 
  for( int color = 0; color < 3; color++)
     markerOff(color);
    
  //doPattern(0);
}

// ========================
// === D O N     C M D  ===
// ========================

void digitalOnCmd(int arg_cnt, char **args)
{
  digitalSet(true);
}


// ============================
// === E N A B L E   C M D  ===
// ============================

void enableCmd(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();

  if ( arg_cnt == 1)
  {
    onQuadrants = 0x0f;
  }
 
}


// ==========================
// === D O F F     C M D  ===
// ==========================

void digitalOffCmd(int arg_cnt, char **args)
{
  digitalSet(false);
  sendCmd( DMARKOFF); 
  markerOff(DIGITAL); 
}

// =======================
// ===  R U N   C M D  ===
// =======================

void runCmd(int arg_cnt, char **args)
{

#ifdef DEBUG
  Serial.println("==== RUN");
#endif

  if ( arg_cnt > 1 )
  {
    if ( strchr(args[1], 'R') )
    {
      pulse_enabled[RED] = 1;
      pulse_state[RED] = START;
    }
    if ( strchr(args[1], 'B') )
    {
      pulse_enabled[BLUE] = 1;
      pulse_state[BLUE] = START;
    }
    if ( strchr(args[1], 'G') )
    {
      pulse_enabled[GREEN] = 1;
      pulse_state[GREEN] = START;
    }
    if ( strchr(args[1], 'S') || strchr(args[1], 'D') )
    {
      pulse_enabled[DIGITAL] = 1;
      pulse_state[DIGITAL] = START;
    }
  }
  else
  {
    for ( int i = 0; i < 4; i++)
    {
      if ( pulse_set[i] > 0 ) // only run channels with an on time
      {
        pulse_enabled[i] = 1;
        pulse_state[i] = START;
      }
    }
  }

  onQuadrants = 0x0f;

  experimentMsecs = 0;

}

// =========================
// ===  S T O P   C M D  ===
// ========================

void stopCmd(int arg_cnt, char **args)
{

#ifdef DEBUG
  Serial.println("==== STOP");
#endif

  digitalWrite(LEDpin, LOW);

  if ( arg_cnt > 1 )
  {
    if ( strchr(args[1], 'R') )
    {
      pulse_enabled[RED] = 0;
      pulse_state[RED] = TURNOFF;
      //     last_state[RED] = OFFSTATE;
      pulse_timer[RED] = 0;
      digitalWrite( markerPin[RED], LOW);
      sendCmd( RMARKOFF);
    }
    if ( strchr(args[1], 'B') )
    {
      pulse_enabled[BLUE] = 1;
      pulse_state[BLUE] = TURNOFF;
      //     last_state[BLUE] = OFFSTATE;
      pulse_timer[BLUE] = 0;
      digitalWrite( markerPin[BLUE], LOW);
      sendCmd( BMARKOFF);
    }
    if ( strchr(args[1], 'G') )
    {
      pulse_enabled[GREEN] = 1;
      pulse_state[GREEN] = TURNOFF;
      //      last_state[GREEN] = OFFSTATE;
      pulse_timer[GREEN] = 0;
      digitalWrite( markerPin[GREEN], LOW);
      sendCmd( GMARKOFF);
    }
    if ( strchr(args[1], 'S') || strchr(args[1], 'D') )
    {
      pulse_enabled[DIGITAL] = 1;
      pulse_state[DIGITAL] = TURNOFF;
      //     last_state[DIGITAL] = OFFSTATE;
      pulse_timer[DIGITAL] = 0;
      digitalWrite( markerPin[DIGITAL], LOW);
      sendCmd( DMARKOFF);
    }
  }
  else
  {
    for ( int i = 0; i < 4; i++)
    {
      pulse_enabled[i] = 0;
      pulse_timer[i] = 0;
      pulse_state[i] = TURNOFF;
      //      last_state[i] = OFFSTATE;
      digitalWrite( markerPin[i], LOW);
      sendCmd( markerOffCmd[i]);
    }
    allPanelsOff();
  }
  //     digitalWrite(TESTPIN, 0);
}

// =======================================
// ===  S T A R T   S H O C K   C M D  ===
// =======================================

void startShockCmd(int arg_cnt, char **args)
{

#ifdef DEBUG
  Serial.println("SS");
#endif
  if ( pulse_set[SHOCK] > 0 ) // only run channels with an on time
  {
    pulse_enabled[SHOCK] = 1;
    pulse_state[SHOCK] = START;
  }
}

// =======================================
// ===  S T O P   S H O C K   C M D  ===
// =======================================

void stopShockCmd(int arg_cnt, char **args)
{
  pulse_enabled[SHOCK] = 0;
  pulse_timer[SHOCK] = 0;
  pulse_state[SHOCK] = TURNOFF; //OFFSTATE;
  //  last_state[SHOCK] = OFFSTATE;
  digitalWrite( markerPin[SHOCK], LOW);
  shock(false);
  //    sendCmd( markerOffCmd[SHOCK]);   // can't send SHOCK or IR on to other boards
}

// ============================
// ===  P A U S E    C M D  ===
// ============================

void pauseCmd(int arg_cnt, char **args)
{
  for ( int i = 0; i < 3; i++)
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

  #ifdef PWMCONTROL
     s->println("External PWM Control");
  #endif

#ifdef DEBUG
  s->println("Debug ON");
#endif
  s->println("\r\nPARAMETER      BLU  GRN  RED DIG");

  s->print("intensity   %: ");
  for ( int rgb = 0; rgb < 4; rgb++ )
  {
    s->print(intensityPC[rgb]);
    s->print(" ");
  }

  s->print("\npulse on   ms: ");
  for ( int rgb = 0; rgb < 4; rgb++ )
  {
    s->print(pulse_on_msec[rgb]);
    s->print(" ");
  }
  s->print("\nperiod     ms: ");
  for ( int rgb = 0; rgb < 4; rgb++ )
  {
    s->print(pulse_off_msec[rgb] + pulse_on_msec[rgb]);
    s->print(" ");
  }
  s->print("\npulse   count: ");
  for ( int rgb = 0; rgb < 4; rgb++ )
  {
    s->print(pulse_count[rgb]);
    s->print(" ");
  }
  s->print("\noff time   ms: ");
  for ( int rgb = 0; rgb < 4; rgb++ )
  {
    s->print(off_time_msec[rgb]);
    s->print(" ");
  }
  s->print("\nwait time sec: ");
  for ( int rgb = 0; rgb < 4; rgb++ )
  {
    s->print(wait_sec[rgb]);
    s->print(" ");
  }
  s->print("\nwait time msec: ");
  for ( int rgb = 0; rgb < 4; rgb++ )
  {
    s->print(wait_msec[rgb]);
    s->print(" ");
  }
  s->print("\nnumber loops : ");
  for ( int rgb = 0; rgb < 4; rgb++ )
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
#ifndef __MKL26Z64__  
  watchdog.reset(); // nice doggy!
#endif  
  sendCmd(ENUMERATE1); // enumerate so each board knows where it is in daisy-chain
  delay(100);           // allow time for command to propogate
#ifndef __MKL26Z64__  
  watchdog.reset(); // nice doggy!
#endif  
//  sendCmd(PARALLEL);   // normal mode sends commnds to all at the same time
//  delay(500);          //  allow time for propogatiion and change over
// #ifndef __MKL26Z64__  
//  watchdog.reset(); // nice doggy!
// #endif  
  sendCmd(SETOFF);     // turn all LEDS off on others

  initDACs();          // reset DACs and enable reference

  ledsOff(0x0f);       // and myself
  for ( uint8_t color = 0; color < 4; color++ ) // reset all intensity values
  {
      
    for (uint8_t quad = 0; quad < 8; quad++)
      intensity[color][quad] = 0;

  }
  for ( int rgb = 0 ; rgb < 3 ; rgb++) // make sure marker LEDs are off
    markerOff(rgb);

  digitalBits = 0x00;  // set digitals off
  digitalSet(false);
  onQuadrants = 0x00;  // reset which quadrants are on 

  inited = 1;

  IRwasOff = false;  // reset IR set off flag
}

// ---  R E S E T   C M D ---
void resetCmd(int arg_cnt, char **args)
{
  #ifdef DEBUG
  Serial.println("     resetCmd");
  #endif
  initBoards();
}

// ==========================
// === R E A D   R G B   ====
// ==========================

int8_t readRGB(char * cmdp)
{
  int8_t colors = 0;

  if ( strchr(cmdp, 'R')) colors |= REDDAC;
  if ( strchr(cmdp, 'B')) colors |= BLUDAC;
  if ( strchr(cmdp, 'G')) colors |= GRNDAC;
  if ( strchr(cmdp, 'S')) colors |= DIGOUT;  // ???
  if ( strchr(cmdp, 'D')) colors |= DIGOUT;

  if ( colors == 0 ) colors--; // if no colors return -1 error
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

  if ( arg_cnt >= 6 )
  {
    for ( int i = 0; i < 4; i++ )
    {
      val[i] = cmdStr2Num(args[i + 1], 10);
    }

    float fwait = cmdStr2Float(args[5]);  // wait time, floating point. divide into integer secs and turn fraction into msec
    val[4] = (int32_t) fwait;
    int32_t waitMsec = (fwait - val[4]) * 1000;

    if ( arg_cnt >= 7) // iterations
      val[5] = cmdStr2Num(args[6], 10);
    else
      val[5] = 0;

    if ( arg_cnt == 8 ) // color!
    {
      int color = readRGB(args[7]);
      if ( color > -1 ) // good color
        rgb = color;
    }
    else
    {
      rgb = 0x0f;  // if no color set,  then set all
    }

    if ( (val[0] < MINWIDTH)  || (val[0] > MAXWIDTH)  ) errflag = -2;
    if ( (val[1] < MINPERIOD) || (val[1] > MAXPERIOD) ) errflag = -3;
    if ( (val[2] < MINPULSES) || (val[2] > MAXPULSES) ) errflag = -4;
    if ( (val[3] < MINOFF)    || (val[3] > MAXOFF)    ) errflag = -5;
    if ( (val[4] < MINWAIT)   || (val[4] > MAXWAIT)   ) errflag = -6;
    if ( (val[5] < MINITERATIONS) || (val[5] > MAXITERATIONS)   ) errflag = -7;

    if ( val[1] < val[0] ) errflag = -8;

    if ( errflag == 0 )
    {
      for ( uint8_t ch = 0; ch < 4; ch++ )
      {
        if ( rgb & (1 << ch) ) // if this channel was selected
        {
          pulse_on_msec[ch] = val[0];
          pulse_off_msec[ch] = val[1] - val[0];
          pulse_count[ch] = val[2];
          off_time_msec[ch] = val[3];
          wait_sec[ch]  = val[4];
          wait_msec[ch] = wait_sec[ch] * 1000 + waitMsec;
          iteration_loops[ch] = val[5];

          // if pulse on is 0, then turn off this channel
          // else enable it
          if ( pulse_on_msec[ch] == 0 )
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
    
Serial.println(" PULSE  a,b,c,d,e, f, C");
Serial.println(" a - pulse width - msec");
Serial.println(" b - pulse period - msec");
Serial.println(" c - # pulses");
Serial.println(" d - off time - msec");
Serial.println(" e - wait time - floating seconds");
Serial.println(" f - number of iterations; 0 = continuous");
Serial.println(" C - color channel ('R', 'G', 'B' and/or 'S' or 'D')");
  }
}

void patternCmd(int arg_cnt, char **args)
{
  // depracated
}

// ==========================
// === S E R V O   C M D ====
// ==========================

int16_t servoVal = 0;

#ifdef SERVOCONTROL
void servoCmd(int arg_cnt, char **args)
{
  if ( arg_cnt > 1 )
  {
      uint16_t tservo = cmdStr2Num(args[1], 10);
      if( (tservo >= 0) && (tservo <= 720) )
      {
        servoVal = tservo;
        rgbServo.write(servoVal);
      }      
  }
  else
  {
    Serial.println(servoVal);
  }

}
#endif

// =================================================================
// ================= E X P E R I M E N T    C M D S  ===============
// =================================================================

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


#define NO_TYPE 0
#define PULSE_TYPE 1
#define DURATION_TYPE 2

#ifndef __MKL26Z64__
#define MAXSTEPS 50
#define MAX_STEP_ORDER  2000
#else
#define MAXSTEPS 10
#define MAX_STEP_ORDER  100
#endif

static uint8_t stepOrder[MAX_STEP_ORDER];
static uint16_t orderCount = 0;
static uint16_t nextOrderedStep = 0;


struct stepValues
{
  uint8_t type;           // 0 = not set, 1 = pulse, 2 = duration
  float intensity[4];     // intensity in %
  int32_t pulseOff[4];    // pulse off time
  int32_t pulseOn[4];     // pulse on time
  int32_t pulseCount[4];  // number of pulses
  int32_t offTime[4];     // off time
  uint16_t iterations[4]; // how many times to repeat pulse train
  uint16_t onOff[4];      // on/off settings for duration type
  uint16_t aOrD[4];       // anaolg or digtal output
  float delayTime;        // delay
  float duration;         // total duration of this step
  uint16_t pattern;       // binary pattern of what quads are on (1) or off (0)
  boolean usePattern;     // true if pattern was appended to command
};

//


struct stepValues xStep[MAXSTEPS];

// ADDONESTEP step#, red%, redOn, redPrd, redCnt, redOff, redIter, grn%, grnOn, grnPrd, grnCnt, grnOff, grnIter, blu%, bluOn, bluPrd, bluCnt, bluOff, bluIter, delay, dur

// addonestep 1, 0.00, 30000, 30000,1,0,1,  3.00, 30000,30000,1,0,1,  0.00, 30000,30000,1,0,1, 0,

// addonestep 1  5 500 1000 3 1000 2  0   0   0 0    0  0 0 0   0   0 0    0 0 30
// addonestep 2  5 500 1000 3 1000 1  0   0   0 0    0  0 0 0   0   0 0    0 2 7
// addonestep 3  0   0    0 0    0 0  3 200 500 4 1000  1 0 0   0   0 0    0 2 5
// addonestep 2 3 200  500 5 1000 4  0   0   0 0    0  0 0 0   0   0 0    0 0 30
// addonestep 2  0   0    0 0    0 0  3 200 500 5 1000  4 0 0   0   0 0    0 0 30
// addonestep 3  0   0    0 0    0 0  0   0   0 0    0  0 0 4 200 500 5 1000 4 30

// addonestep 1  20 10000 30000 3 5000 1  0 0 0 0 0 0  0 0 0 0 0 0 5 100
// addonestep 2  0 0 0 0 0 0  30 5000 15000 30 5000 1  0 0 0 0 0 0 0 50
// addonestep 3  0 0 0 0 0 0  0 0 0 0 0 0  40 5000 10000 3 0 1  0 30


// addonestep 1, 0.00, 000,1000,1,2000,10,  8.00, 500,1000,1,2000,10,  0.00, 000,1000,1,2000,10, 5,60
// addonestep 2, 8.00,1000,1000,1,2000,10,  0.00,1000,1000,1,2000,10,  0.00,1000,1000,1,2000,10, 0,60

// addonestep 1, 0.00,500,1000,1,2000,10, 3.00,500,1000,1,2000,3, 0.00,500,1000,1,2000,10,5,20


// addonestep 1, 0.00, 500,1000,1,2000,5,  8.00, 100,1000,1,2000,5,  0.00, 500,1000,1,2000,5, 5,25, 11001010
// addonestep 2, 8.00,1000,1000,1,2000,5,  0.00,1000,1000,1,2000,5,  0.00,1000,1000,1,2000,5, 5,30, 00110101
// addonestep 2, 0.00, 500,1000,1,2000,5,  0.00, 500,1000,1,2000,5,  8.00, 300,1000,1,2000,5, 5,25, 11000011
// Jin 20220601
// addonestep 1, 00.00,1000,2000,0,2000,1,  0.00,0,0,0,0,0, 0.00,0,0,0,0,0, 3.9, 10, 00000000
// addonestep 2, 16.00,1000,2000,0,2000,1,  0.00,0,0,0,0,0, 0.00,0,0,0,0,0, 3.9, 10, 10011001
// addonestep 3, 16.00,1000,2000,0,2000,1,  0.00,0,0,0,0,0, 0.00,0,0,0,0,0, 3.9, 10, 01100110
// addonestep 4, 16.00,1000,2000,0,2000,1,  0.00,0,0,0,0,0, 0.00,0,0,0,0,0, 3.9, 10, 11111111
// order 4 1 2 3

//addonestep 1, 5.00,1000,1000,1,2000,2, 2.00,1000,1000,1,2000,2, 2.00,1000,1000,1,2000,2, 2.00,10
//addonestep 2, 0.00, 000,2000,1,4000,2,  3.00,2000,2000,1,4000,2,  0.00,0000,2000,1,4000,2,  2.00,15
//addonestep 3, 3.00,100,200,10,3000,3,   0.00,0000,2000,1,3000,1,  3.00,1000,2000,1,3000,1,  2.00,15




//  runExperiment -RX
//  getExperimentStatus - GX
//  getExperimentsteps - GS
//  stopExperiment - SX


// ================================
// === A D D   O N E   S T E P  ===
// ================================


void addOneStep(int arg_cnt, char **args)
{

  Stream *s = cmdGetStream();

#ifdef DEBUG
  Serial.println( arg_cnt);
  for ( int i = 0; i < arg_cnt; i++)
  {
    Serial.print(i);
    Serial.print( "=");
    Serial.print( args[i]);
    Serial.print( " ");
  }
  Serial.println();
#endif

  if ( arg_cnt >= 22 ) // command + 21 values must be present, 22 if pattern
  {
    int idx =  cmdStr2Num(args[1], 10); // get step number

    //#ifdef DEBUG
    //      Serial.print(idx);
    //      Serial.print(" ");
    //      Serial.println(MAXSTEPS);
    //#endif

    if ( idx < MAXSTEPS - 1)
    {
      xStep[idx].intensity[RED] = cmdStr2Float(args[2]);
      xStep[idx].pulseOn[RED] =  cmdStr2Num(args[3], 10);
      xStep[idx].pulseOff[RED] = cmdStr2Num(args[4], 10) - xStep[idx].pulseOn[RED];  // sent pulse period, but we want off time to run it
      xStep[idx].pulseCount[RED] = cmdStr2Num(args[5], 10);
      xStep[idx].offTime[RED] = cmdStr2Num(args[6], 10);
      xStep[idx].iterations[RED] = cmdStr2Num(args[7], 10);
      
      xStep[idx].intensity[GREEN] = cmdStr2Float(args[8]);
      xStep[idx].pulseOn[GREEN] =  cmdStr2Num(args[9], 10);
      xStep[idx].pulseOff[GREEN] = cmdStr2Num(args[10], 10) - xStep[idx].pulseOn[GREEN];  // sent pulse period, but we want off time to run it
      xStep[idx].pulseCount[GREEN] = cmdStr2Num(args[11], 10);
      xStep[idx].offTime[GREEN] = cmdStr2Num(args[12], 10);
      xStep[idx].iterations[GREEN] = cmdStr2Num(args[13], 10);
      
      xStep[idx].intensity[BLUE] = cmdStr2Float(args[14]);
      xStep[idx].pulseOn[BLUE] =  cmdStr2Num(args[15], 10);
      xStep[idx].pulseOff[BLUE] = cmdStr2Num(args[16], 10) - xStep[idx].pulseOn[BLUE];  // sent pulse period, but we want off time to run it
      xStep[idx].pulseCount[BLUE] = cmdStr2Num(args[17], 10);
      xStep[idx].offTime[BLUE] = cmdStr2Num(args[18], 10);
      xStep[idx].iterations[BLUE] = cmdStr2Num(args[19], 10);
      xStep[idx].delayTime = cmdStr2Float(args[20]);   // cmdStr2Num(args[20], 10);
      xStep[idx].duration = cmdStr2Float(args[21]);   // cmdStr2Num(args[21], 10);
      if ( arg_cnt == 23 )
      {
        xStep[idx].pattern = cmdStr2Num(args[22], 2); // get optional pattern
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
  else if ( arg_cnt == 8 ) // duration command
  {
    int idx =  cmdStr2Num(args[1], 10); // get step number

    //#ifdef DEBUG
    //      Serial.print(idx);
    //      Serial.print(" ");
    //      Serial.println(MAXSTEPS);
    //#endif


    if (idx < MAXSTEPS - 1)
    {
      xStep[idx].intensity[RED] = cmdStr2Float(args[2]);
      xStep[idx].onOff[RED] = 0;
      xStep[idx].aOrD[RED] = 0;
      int i = 0;
      while ( (args[3][i] >= '0') && (i < 16) )
      {
        if ( args[3][i] == '1' )
        {
          xStep[idx].onOff[RED] |= 1;
        }
        else if ( args[3][i] == 'D')
        {
          xStep[idx].onOff[RED] |= 1;
          xStep[idx].aOrD[RED] |= 1;
        }
        xStep[idx].onOff[RED] = xStep[idx].onOff[RED] << 1;
        xStep[idx].aOrD[RED] = xStep[idx].aOrD[RED] << 1;
        i++;
      }

      xStep[idx].intensity[GREEN] = cmdStr2Float(args[4]);
      xStep[idx].onOff[GREEN] = 0;
      xStep[idx].aOrD[GREEN] = 0;
      i = 0;
      while ( (args[3][i] >= '0') && (i < 16) )
      {
        if ( args[3][i] == '1' )
        {
          xStep[idx].onOff[GREEN] |= 1;
        }
        else if ( args[3][i] == 'D')
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
      while ( (args[3][i] >= '0') && (i < 16) )
      {
        if ( args[3][i] == '1' )
        {
          xStep[idx].onOff[BLUE] |= 1;
        }
        else if ( args[3][i] == 'D')
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
    if ( xStep[i].type > 0)
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
  for ( int idx = 0; idx < MAXSTEPS; idx++ )
  {
    if ( xStep[idx].type == PULSE_TYPE )
    {
      s->print(idx);
      s->print(",");
      s->print(xStep[idx].intensity[RED]);
      s->print(",");
      s->print(xStep[idx].pulseOn[RED]);
      s->print(",");    
      s->print(xStep[idx].pulseOff[RED] + xStep[idx].pulseOn[RED]);
      s->print(",");
      s->print(xStep[idx].pulseCount[RED]);
      s->print(",");
      s->print(xStep[idx].offTime[RED]);
      s->print(",");
      s->print(xStep[idx].iterations[RED]);
      s->print(",");
      
      s->print(xStep[idx].intensity[GREEN]);
      s->print(",");
      s->print(xStep[idx].pulseOn[GREEN]);
      s->print(",");    
      s->print(xStep[idx].pulseOff[GREEN] + xStep[idx].pulseOn[GREEN]);
      s->print(",");
      s->print(xStep[idx].pulseCount[GREEN]);
      s->print(",");
      s->print(xStep[idx].offTime[GREEN]);
      s->print(",");
      s->print(xStep[idx].iterations[GREEN]);
      s->print(",");
      
      s->print(xStep[idx].intensity[BLUE]);
      s->print(",");
      s->print(xStep[idx].pulseOn[BLUE]);
      s->print(",");
      s->print(xStep[idx].pulseOff[BLUE] + xStep[idx].pulseOn[BLUE]);
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
      if ( xStep[idx].usePattern == true )
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
      for ( uint16_t i = 1; i <= 0x8000; i = i << 1)
      {
        if ( xStep[idx].onOff[RED] & i )
        {
          if ( xStep[idx].aOrD[RED] & i )
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
      for ( uint16_t i = 1; i <= 0x8000; i = i << 1)
      {
        if ( xStep[idx].onOff[GREEN] & i )
        {
          if ( xStep[idx].aOrD[GREEN] & i )
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
      for ( uint16_t i = 1; i <= 0x8000; i = i << 1)
      {
        if ( xStep[idx].onOff[BLUE] & i )
        {
          if ( xStep[idx].aOrD[BLUE] & i )
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
  for ( int idx = 0; idx < MAXSTEPS; idx++ )
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
  for ( int i = 0; i < 4; i++)
  {
    if ( bowlEnabled[i] == true ) bowls |= (1 < i);
  }

  // skip over any unfilled steps
  while ( (xStep[xIdx].type == 0) && (xIdx < MAXSTEPS) ) xIdx++;

  if( orderCount > 0 )
  {
    nextOrderedStep = 0;
    xIdx = stepOrder[nextOrderedStep++];
  }

  for ( uint8_t rgb = 0; rgb < 3; rgb++)
    newIntensity( 0.0, rgb, 0, 0x0f);  // turn off all colors before setting pattern

  if ( xStep[xIdx].usePattern ) // set the pattern - this requires enabling active quadrants
  {
    uint16_t bitpat = xStep[xIdx].pattern;
    uint8_t quadbits;
    for ( uint8_t pan = 1; pan < 5; pan++)
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

  setOnOff(0, 0, 0); // now turn everything off before setting intensities

  for ( int i = 0; i < 3; i++) // go through colors
  {
    newIntensity( xStep[xIdx].intensity[i], i, bowls, 0x0f);
    pulse_count[i] =  xStep[xIdx].pulseCount[i]; //iterations[i];
    pulse_on_msec[i] = xStep[xIdx].pulseOn[i];
    pulse_off_msec[i] = xStep[xIdx].pulseOff[i];
    off_time_msec[i] = xStep[xIdx].offTime[i];
    iteration_loops[i] = xStep[xIdx].iterations[i];
    wait_msec[i] = uint32_t (xStep[xIdx].delayTime * 1000.0) ;
    wait_cnt[i] = 0;
    frame_on_msec[i] = 0;
    frame_off_msec[i] = 0;
    if ( pulse_on_msec[i] > 0 ) //xStep[xIdx].intensity[i] > 0.001)
    {
      pulse_enabled[i] = 1;
      pulse_state[i] = START;
    }
    else
    {
      markerOff(i);
      if( pulse_off_msec[i] == 0 )  
        pulse_enabled[i] = 0;
      else         // if we have no on time but an off time we need to enable pulse so it goes through the state mnachine
        pulse_enabled[i] = 1;  
      pulse_state[i] = START; // STEPWAIT; //HOLD;
    }

    experimentMsecs = 0;

#ifdef DEBUG
    Serial.print("RGB ");
    Serial.print(i);
    Serial.print(" WMS ");
    Serial.print(wait_msec[i]);
    Serial.print(" dur ");
    Serial.println(xStep[xIdx].duration * 1000);
#endif


  }
  //    stepDelay = xStep[xIdx].delayTime;
  stepDuration = uint32_t( xStep[xIdx].duration * 1000);
  
  state = false;
  digitalWrite( triggerPin, HIGH);
  if( clockOutEnabled) 
     syncTimer.begin(syncInt, 500000 / syncRate); // start clock out
      
//  if( clockOutEnabled == false) digitalWrite( triggerPin, HIGH); 

//  Serial.print( IRwasOff);
//  Serial.print(' ');
//  Serial.println( lastIRpercent);

  if( IRwasOff ) newIntensity( lastIRpercent, IR, 0, 0x0f);
  
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
#ifdef DEBUG
  Serial.println("=== XSTOP");
#endif

  xIdx = 0;  // this stops experiment after current step
  setOnOff(0, 0, 0); // set all panels and quadrants off
  for ( uint8_t rgb = 0; rgb < 4; rgb++) // all colors
  {
    markerOff(rgb);
    pulse_state[rgb] = HOLD;   // skip to HOLD state
    flag[rgb] = 0;   // don't allow any more setups in msec timer
  }
  
  digitalWrite( triggerPin, LOW);
  if( clockOutEnabled) 
     syncTimer.end(); //begin(syncInt, 500000 / syncRate); // start clock out
  
  if( IRwasOff ) newIntensity( 0.0, IR, 0, 0x0f); // turn IR off


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
    for ( int i = 0; i < 4; i++)
    {

      // Serial.print( args[i+1] );

      if ( !strcmp( args[i + 1], "TRUE" ) )
        bowlEnabled[i] = true;
      else
        bowlEnabled[i] = false;
#ifdef DEBUG
      Serial.print(bowlEnabled[i]);
      if ( i < 3 )  Serial.print(" "); else Serial.println();
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
  if ( markerState[RED] )
    s->print( xStep[xIdx].intensity[RED]);
  else
    s->print('0');
  s->print(',');
  if ( markerState[GREEN] )
    s->print( xStep[xIdx].intensity[GREEN]);
  else
    s->print('0');
  s->print(',');
  if ( markerState[BLUE] )
    s->println( xStep[xIdx].intensity[BLUE]);
  else
    s->println('0');
}



// ===========================
// === S T E P  O R D E R  ===
// ===========================

void stepOrderCmd(int arg_cnt, char **args) // add a step order - 0 resets it
{
  Stream *s = cmdGetStream();
  if ( arg_cnt > 1 ) // parameter?
  {
    uint8_t nextStep =  cmdStr2Num(args[1], 10); // get step number
    if( nextStep == 0)  // reset table
    {
      orderCount = 0;
      nextOrderedStep = 0;
    }
    else if( nextStep < MAXSTEPS )  // limit to number of steps allocated
    {
      stepOrder[orderCount++] = nextStep;
    }
  }
  else // list steps
  {    
     for( uint16_t i = 0; i < orderCount; i++)
     {
       s->print( i+1 );
       s->print(" ");
       s->println(stepOrder[i]);
     }  
  }
}


// ==============================
// === M A R K E R   C M D S  ===
// ==============================

void markerCmd(int arg_cnt, char **args)  //marker test
{
   if ( arg_cnt > 2 )
   { 
      if( strchr(args[2], 'N') )
      {
        if ( strchr(args[1], 'R') )
          markerOn(RED);
        if ( strchr(args[1], 'G') )
          markerOn(GRN);
        if ( strchr(args[1], 'B') )
          markerOn(BLU);
        if ( strchr(args[1], 'D') )
          markerOn(DIG);  
      }
      else if( strchr(args[2], 'F') )
      {
        if ( strchr(args[1], 'R') )
          markerOff(RED);
        if ( strchr(args[1], 'G') )
          markerOff(GRN);
        if ( strchr(args[1], 'B') )
          markerOff(BLU); 
        if ( strchr(args[1], 'D') )
          markerOff(DIG);            
      }
   }
   else
   {
      if( markerState[RED]) Serial.print('R');
      if( markerState[BLU]) Serial.print('B');
      if( markerState[GRN]) Serial.print('G');
      if( markerState[DIG]) Serial.print('D');
      Serial.println();      
   }
}

// ===========================
// ===  B L I N K   C M D  ===
// ===========================

void blinkEnd()
{
   blinkTimer.end();
   digitalWriteFast(redPin, LOW);  
   digitalWriteFast(grnPin, LOW); 
   digitalWriteFast(bluPin, LOW); 
   digitalWrite(LEDpin, LOW);
}       

void blinkCmd(int arg_cnt, char **args)  // blink Marker(s)
{
  
   if ( arg_cnt > 1 )
   {
      blinkTime = cmdStr2Num(args[1], 10);
      if( (blinkTime < 0) || (blinkTime > MAXBLINKTIME) )
      {
         blinkTime = 0;
         blinkColor = 0;
      }
   }
   else
   {
      blinkTime = 100;
   } 
    
   if( blinkTime > 0 ) 
   { 
     if( arg_cnt > 2 )
     {
        blinkColor = 0;
        if ( strchr(args[2], 'R') )
        {
           digitalWriteFast(redPin, HIGH);   
           digitalWriteFast(LEDpin, HIGH); 
           blinkColor |= REDBIT;
        }
        if ( strchr(args[2], 'B') )
        {
          digitalWriteFast(bluPin, HIGH);
          blinkColor |= BLUBIT;
        }
        if ( strchr(args[2], 'G') )
        {
          digitalWriteFast(grnPin, HIGH);
          blinkColor |= GRNBIT;
        }
     }
     else
     {
        digitalWriteFast(LEDpin, HIGH);
     }

     blinkTimer.begin( blinkEnd, blinkTime * 1000);
   }
}


void blink(void)
{
   if( (blinkTime > 0) && (blinkColor > 0) )
   {
      if( blinkColor & REDBIT ) 
      {
           digitalWriteFast(redPin, HIGH);   
           digitalWriteFast(LEDpin, HIGH); 
      }
      if( blinkColor & BLUBIT )
         digitalWriteFast(bluPin, HIGH);
      if( blinkColor & GRNBIT )
         digitalWriteFast(grnPin, HIGH);
      blinkTimer.begin( blinkEnd, blinkTime * 1000);     
   }
}

// ===================================
// === S T A T E   C O M M A N D   ===
// ===================================

void stateCmd(int arg_cnt, char **args)
{
   Serial.print(states[pulse_state[RED]]);
   Serial.print(" ");
   Serial.print(states[pulse_state[GRN]]);
   Serial.print(" ");  
   Serial.println(states[pulse_state[BLU]]);    
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

  if ( arg_cnt > 1 )
  {
    tsync = cmdStr2Float(args[1]);
    if ( (tsync > MINSYNC) && (tsync < MAXSYNC) )
    {
      detachInterrupt(SYNCpin);
      pinMode( SYNCpin, OUTPUT);
      digitalWrite( SYNCpin, LOW);
      syncRate = tsync;
      digitalWrite(triggerPin, LOW);
      clockOutEnabled = true;
  //    syncTimer.begin(syncInt, 500000 / syncRate); // change immediately (or start if ended)
      //syncTimer.update( 500000/syncRate);
    }
    else
    {
      if ( (tsync > -0.001) && (tsync < 0.001) ) // if '0' then turn sync off
      {
        clockOutEnabled = false;
        syncTimer.end();
        pinMode( SYNCpin, INPUT_PULLUP);
        digitalWrite(triggerPin, LOW);
      }
    }
  }
  else
  {
    Stream *s = cmdGetStream();
    s->println(syncRate);
  }
}

// ======================
// === S Y N C   O N   C M D  ===
// ======================

void SyncOnCmd(int arg_cnt, char **args)
{
  syncTimer.begin(syncInt, 500000 / syncRate); // change immediately (or start if ended)
}

// ======================
// === P W M   C M D  ===
// ======================

void PWMcmd(int arg_cnt, char **args)
{
   if( arg_cnt > 1 )
   {
      if ( strchr(args[1], 'R') )
      {
         pwmStatus = RED;
      }
      if ( strchr(args[1], 'B') )
      {
         pwmStatus  = BLU;
      }
      if ( strchr(args[1], 'G') )
      {
         pwmStatus = GRN;
      }
      
 //     attachInterrupt(digitalPinToInterrupt(SYNCpin), PWMint, CHANGE);
 //     NVIC_SET_PRIORITY(IRQ_PORTB, 0);  // fastest priority for sync pin
   }    
}

// =====================
// === W D T  C M D  ===
// =====================
void wdtCmd(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  if ( arg_cnt > 1 ) // any argument will cause a test
    while (1);
  else  // see if last reset was WDT
  {
//    s->println( (RCM_SRS0 & 0x20) >> 5 );
#ifndef __MKL26Z64__
    s->println( watchdog.tripped() );
#endif    
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
// Intensity low 4 bits 1100iiii     // send lowest 4 bits of 12 bit intensity DAC value or panel 1 digital
// Intensity mid 4 bits 1101iiii     // send middle 4 bits of 12 bit intensity DAC value or panel 2 digital
// Intensity high4 bits 1110iiii     // send highest 4 bits of 12 bit intensity DAC value or panel 3 digital
// Enumerate command:   111100xx     // enumerate the boards when in series mode. Main sends 0xf1, next panel sees this and sends F2 to next, etc
// Series Mode:         11110100     // set intraboard comms to serial - needed to do enumeration
// - depracted--- Parallel Mode:       11110101     // set intraboard comms to parallel - used for all other commands
// color set            11111ddd     // turn colors on and off for all panels (mostly for pulse) 1= on, 0 = off

// MARKER commands (in parallel mode)
// red marker off       11110000
// green marker off     11110010   // also enumerate 2
// blue marker off      11110110
// set digital pins     11110111   // uses bit pattern from Intensity bits (was blue on)
// digital on           11110001   // also enumerate 1 (was red on)
// digital off          11110011   // also enumerate 3 (was green on)
// digital marker off   11110101   // was parallel mode

// ========================
// === S E N D   C M D  ===
// ========================
void sendCmd(uint8_t cmd)
{
  #ifdef DEBUG
    Serial.print("cmd: ");
    Serial.print(cmd,BIN);
    if( (cmd & 0xc0) == 0x00 )  Serial.println(" OFF");
    else if( (cmd & 0xc0) == 0x40 )  Serial.println(" ON");
    else if( (cmd & 0xc0) == 0x80 )  Serial.println(" SET I");
    else if( (cmd & 0xf0) == 0xc0 )  Serial.println(" LOW");
    else if( (cmd & 0xf0) == 0xd0 )  Serial.println(" MID");
    else if( (cmd & 0xf0) == 0xe0 )  Serial.println(" HIGH");
//    else if( (cmd & 0xfc) == 0xf0 )  Serial.println(" ENUM");
    else if( (cmd & 0xfc) == 0xfc )  Serial.println(" CSET");
    else if(  cmd == 0xf4 ) Serial.println(" SERIES");
//    else if(  cmd == 0xf5 ) Serial.println(" PAREL");
    else if(  cmd == 0xf0 ) Serial.println(" RMOFF");
    else if(  cmd == 0xf6 ) Serial.println(" BMOFF");
    else if(  cmd == 0xf7 ) Serial.println(" SET D");
    else if(  cmd == 0xf3 ) Serial.println(" D OFF");
    else if(  cmd == 0xf1 ) Serial.println(" D ON");
    else if(  cmd == 0xf5 ) Serial.println(" DMOFF");
  #endif
#ifndef SERVOCONTROL
  Serial1.write(cmd);
#endif  
}

// panels positions change when grouped, but want quads to make sense
uint8_t MYQUAD[] = { 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0 };

uint8_t getMyQuad(uint8_t myAdr, uint8_t quad)
{
  return (MYQUAD[myAdr * 3 + quad]);
}

// ==========================================
// ===  I N T E R   B O A R D   P O L L   ===
// ==========================================

void interBoardPoll(void)
{
  while ( Serial1.available() )
  {
    uint8_t cmd = Serial1.read();
#ifdef DEBUG
    Stream *s = cmdGetStream();
    s->print("in:");
    s->print(cmd, BIN);
    s->print(" mode:");
    if( cmdMode == SERIES ) 
      s->print("series ");
    else
      s->print("parallel");  
#endif
    if ( (cmd & COLORSET) == COLORSET) // set colors on or off
    { // ====== COLORSET =====
      colorSet(cmd & 0x07);  // set selected colors on (1) or off (0)
      if( cmd & 0x01 ) markerOn(BLU);  //else markerOff(BLUE);    // if 'on', turn on markers
      if( cmd & 0x02 ) markerOn(GRN); // else markerOff(GREEN);  //   don't turn off here as we may be pulsing
      if( cmd & 0x04 ) markerOn(RED); //else markerOff(RED);
    }
    // ======= ON OFF COMMAND =======
    else if ( (cmd & 0x80) == 0 ) // bit 7 = 0; so, on or off
    {
      if ( ((cmd & PANELBITS) == thisPanel) || ((cmd & PANELBITS) == 0) )
      {
#ifdef DEBUG
        s->print(" On/Off");
#endif
        if ( cmd & SETON ) // on
        {
          ledsOn(cmd & 0x0f);  // turn on selected quadrants
        }
        else
        {
          ledsOff(cmd & 0x0f);  // turn off selected quadrants
        }
      } // endif this panel
    } // endif on/off
    else if( (cmdMode == PARALLEL) && (cmd == DIGITALON) )
    {
      #ifdef DEBUG
         s->println("DON");
      #endif   
      digitalSet(true);
      markerOn(DIG);
    }
    else if( (cmdMode == PARALLEL) && (cmd == DIGITALOFF) )
    {
      #ifdef DEBUG
         s->println("DOFF");
      #endif      
      digitalSet(false);
    }
    // =======  SET INTENSITY COMMAND =========
    else if ( (cmd & 0x40) == 0 ) // bit 7 = 1, bit 6 = 0; so, intensity set
    {
      if ( ((cmd & PANELBITS) == thisPanel) || ((cmd & PANELBITS) == 0) )
      {
        uint8_t color = cmd & 0x03;
        uint8_t quad = (cmd & 0x0c) >> 2;
        uint16_t DACval = (intensityHON << 8) + (intensityMON << 4) + intensityLON;
        setIntensity(color, quad, DACval );
#ifdef DEBUG
        s->print("panel:");
        s->print( (cmd & PANELBITS) >> 4 );
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
    else if ( (cmd & 0xf0) == INTENSITYLON )
    {
      intensityLON = uint16_t(cmd & 0x0f);  // intensity
#ifdef DEBUG
      s->print("LON:");
      s->print(intensityLON, HEX);
#endif
    }
    // ======= INTENSITY MON  =========
    else if ( (cmd & 0xf0) == INTENSITYMON )
    {
      intensityMON = uint16_t(cmd & 0x0f);  // intensity
#ifdef DEBUG
      s->print("MON:");
      s->print(intensityMON, HEX);
#endif
    }
    // ======= INTENSITY HON ========
    else if ( (cmd & 0xf0) == INTENSITYHON )
    {
      intensityHON = uint16_t(cmd & 0x0f);  // intensity
#ifdef DEBUG
      s->print("HON:");
      s->print(intensityHON, HEX);
#endif
    }
    // ======== ENUMERATE 1 ========
    else if( (cmdMode == SERIES) && (cmd == ENUMERATE1) ) // from host board - I'm #1 or digital on
    {
        sendCmd(ENUMERATE2);
        Serial1.flush();    // wait until it's starting to send
        thisPanel = 0x10;
        initDACs();          // reset DACs and enable reference
        initDOUT();
        ledsOff(0x0f); // basically a reset - so be sure we start off
#ifdef DEBUG
        digitalWriteFast(Dout1Pin, HIGH);
#endif   
   
        digitalWriteFast( DIRpin, LOW);   // change direction mode
        cmdMode = PARALLEL;

#ifdef DEBUG
        s->println("--- I'm board #1");
#endif
    }
    // ====== ENUMERATE 2 ========
    else if( (cmdMode == SERIES) && (cmd == ENUMERATE2) )  // from board next to host - I'm #2
    {
        sendCmd(ENUMERATE3);
        Serial1.flush();    // wait until it's starting to send
        thisPanel = 0x20;
        initDACs();          // reset DACs and enable reference
        initDOUT();
        ledsOff(0x0f); // basically a reset - so be sure we start off
#ifdef DEBUG
        digitalWriteFast(Dout2Pin, HIGH);
#endif        
        digitalWriteFast( DIRpin, LOW);   // change direction mode     
        cmdMode = PARALLEL;
#ifdef DEBUG
        s->println("--- I'm board #2");
#endif
    }
    // ====== ENUMERATE 3 ========
    else if( (cmdMode == SERIES) && (cmd == ENUMERATE3) )  // from 2nd board from host - I'm last (#3)
    {
        thisPanel = 0x30;
        initDACs();          // reset DACs and enable reference
        initDOUT();
        ledsOff(0x0f); // basically a reset - so be sure we start off
#ifdef DEBUG
        digitalWriteFast(Dout3Pin, HIGH);
#endif         
        digitalWriteFast( DIRpin, LOW);   // change direction mode
        cmdMode = PARALLEL;
#ifdef DEBUG
        s->println("--- I'm board #3");
#endif
    }
    // ======= SERIES =======
    else if ( cmd == SERIES)
    {
      digitalWriteFast( DIRpin, HIGH); // daisy chain serial connetions so we can enumerate the boards
      cmdMode = SERIES;
#ifdef DEBUG
      s->print(" -> serial ");
#endif
    }
    else if (cmd == RMARKOFF )
    {
      markerOff(RED);
    }
    else if ( cmd == BMARKOFF)
    {
      markerOff(BLUE);
    }
    else if( (cmdMode == PARALLEL) && (cmd == GMARKOFF) )
    {
      markerOff(GREEN);
    }
    else if ( cmd == DMARKOFF)
    {
      markerOff(DIGITAL);
    }
    else if (cmd == SETDIGITAL)
    {
#ifdef DEBUG     
       s->print(" DIGON ");
#endif       
       if ( thisPanel == 0x10 )
       {
          digitalBits = intensityLON;
       }
       else if (thisPanel == 0x20 )
       {
          digitalBits = intensityMON;
       }
       else if ( thisPanel == 0x30 )
       {
          digitalBits = intensityHON;
       }
      // markerOn(DIGITAL);
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
// === digitalOut  ====
// ====================
// turn digital Out pins on/off

void digitalOut( boolean state)
{

}


// ====================
// === S H O C K  ====
// ====================
// turn shock pin pin on/off
// a define is used to determine if 'on' is active low

void shock(boolean state)
{
#ifdef SYNC_PIN_OUT

#ifdef SYNC_ACTIVE_LOW
  if ( state  )
    digitalWriteFast(SYNCpin, LOW);
  else
    digitalWriteFast(SYNCpin, HIGH);
#else
  if ( state )
    digitalWriteFast(SYNCpin, HIGH);
  else
    digitalWriteFast(SYNCpin, LOW);
#endif

#endif
}

// =======================================
// === M A R K E R   L E D  O N  O F F ===
// =======================================

void markerOn( int rgb)
{
  markerState[rgb] = true;
  switch ( rgb)
  {
    case RED:
      digitalWriteFast( redPin, HIGH);
      break;
    case BLU:
      digitalWriteFast( bluPin, HIGH);
      break;
    case GRN:
      digitalWriteFast( grnPin, HIGH);
      break;
    case DIG:
      digitalWriteFast( digitalPin, HIGH);
      break;
  }
  //   sendCmd( markerOnCmd[rgb]);
#ifdef DEBUG
  Serial.print('^');
  //     Serial.print(experimentMsecs);
#endif
}

void markerOff( int rgb)
{
  markerState[rgb] = false;
  if ( thisPanel == 0 )  sendCmd( markerOffCmd[rgb]); // board one must tell the others
  switch ( rgb)
  {
    case RED:
      digitalWriteFast( redPin, LOW);
      break;
    case BLU:
      digitalWriteFast( bluPin, LOW);
      break;
    case GRN:
      digitalWriteFast( grnPin, LOW);
      break;
    case DIG:
      digitalWriteFast( digitalPin, LOW);
      break;
  }

#ifdef DEBUG
  Serial.print('v');
#endif
}

// ===================================
//  PWM INT
//=========================================

//int width, lastWidth;
volatile long static PWstart[4], PWend[4], PW[4];
volatile boolean newWidth[4] = {false, false, false, false};
//volatile boolean newStart[4] = {false, false, false, false};
volatile long static lastWidth[4], width[4], oldWidth[4];

void PWMintR()   // as is, will not work at 100% brightness
{
 
   PWend[RED] = ARM_DWT_CYCCNT;
   if( digitalReadFast(redPWMpin) == HIGH )
   {
      PWstart[RED] = PWend[RED]; 
//      newStart[RED] = true; 
   }
   else
   {
      PW[RED] = PWend[RED] - PWstart[RED];  
      newWidth[RED] = true;
   } 
//   Serial.write('r');
 //  Serial.print(newWidth[RED]);
}

void PWMintG()   // as is, will not work at 100% brightness
{
   PWend[GRN] = ARM_DWT_CYCCNT;
   if( digitalReadFast(grnPWMpin) == HIGH )
   {
      PWstart[GRN] = PWend[GRN];  
//      newStart[GRN] = true;
   }
   else
   {
      PW[GRN] = PWend[GRN] - PWstart[GRN];
      newWidth[GRN] = true;
   }
//              Serial.write('G');
}



// =========================
// === P U L S E   I N T ===
// =========================

// 1 msec timer

void pulseInt(void)
{
  
  static int8_t rgbset;
  int8_t newrgb = rgbset;  // assume no change
  static uint16_t cnt;

  experimentMsecs++;

  for ( int rgb = 0; rgb < 4; rgb++ )
  {
#ifdef ANALOGCONTROL
     if( (rgb == RED) || (rgb == GRN) )
     {
        lastWidth[rgb] = width[rgb];
        switch(rgb)
        {
          case RED:
            width[RED] = analogRead(A10);
//            Serial.println(width[RED]);
            break;
          case GRN:
            width[GRN] = analogRead(A11);
 //            Serial.println(width[GRN]);
            break;  
          default: 
            break;          
        }
        if( width[rgb] < 8 ) width[rgb] = 0;
//      medfilt(width);
        if( abs(width[rgb] - lastWidth[rgb]) > 5 )
        {
            newIntensity( (float)width[rgb]/10, rgb, 0, 0xff);
//          #ifdef DEBUG
//           Serial.print(rgb );
 //          Serial.print(" ");
//           Serial.println(width[rgb]);
//          #endif
        }   
        if( width[rgb] > 0 ) 
             bitSet(newrgb, rgb);       
         else 
             bitClear(newrgb, rgb); 
     }
#else  
    
// PWM was commented out - save code at end of this file
    
    if ( flag[rgb] ) // still running a step
    {     
      pulse_timer[rgb]--;   // count down a msec
      if ( pulse_timer[rgb] <= 0 ) // if done, load next step info
      {
        if ( ledon[rgb] )  // set LED bits on
        {
          if ( rgb < 3 ) // colors
            bitSet(newrgb, rgb);
          else // shock
            digitalSet(true);   // shock(true);
        }
        else              // or off
        {
          if ( rgb < 3 ) // colors
            bitClear(newrgb, rgb);
          else
            digitalSet(false); // shock(false);
        }
        pulse_timer[rgb] = next_timer[rgb];  // new timer value
        flag[rgb] = 0;   // let main loop know we are done with last step
      }
    } 
    #endif
  }  // next  color

  if ( newrgb != rgbset) // only update LEDs if changes
  {
    #ifdef DEBUG
       Serial.print(experimentMsecs);
       Serial.print(" ");
       Serial.print(newrgb);
       Serial.print(" != ");
       Serial.println(rgbset);
    #endif              
    rgbset = newrgb;
    #ifndef ANALOGCONTROL
    sendCmd(COLORSET | rgbset);   // send the command out to others first
    #endif
    colorSet(rgbset);  // now update all at once
 //   Serial.println(rgbset);
  }

}

// ========================
// === S Y N C    I N T ===
// ========================

// camera sync

  
void syncInt(void)
{


  if ( state )
  {
    digitalWriteFast(triggerPin, HIGH);
//    #ifdef SYNC_PIN_OUT
//       digitalWriteFast(SYNCpin, LOW);
//    #endif
    state = false;
  }
  else
  {
    digitalWriteFast(triggerPin, LOW);
//    #ifdef SYNC_PIN_OUT
//       digitalWriteFast(SYNCpin, HIGH);
//    #endif
    state = true;
  }
}


// ==================
// ===  S E T U P ===
// ==================

void setup()
{
 #ifndef __MKL26Z64__
  // The following 2 lines are only necessary for T3.0, T3.1 and T3.2
  ARM_DEMCR    |= ARM_DEMCR_TRCENA;         // enable debug/trace
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;   // enable cycle counte
 #endif
  pinMode( DIRpin, OUTPUT);
  pinMode( OTMPpin, INPUT_PULLUP);
  pinMode( LLpin, OUTPUT);
  pinMode( URpin, OUTPUT);
  pinMode( LRpin, OUTPUT);
  pinMode( ULpin, OUTPUT);
#ifdef SYNC_PIN_OUT
  pinMode( SYNCpin, OUTPUT);
  digitalWrite( SYNCpin, LOW);
#else
//  pinMode( SYNCpin, INPUT_PULLUP);
#endif
  pinMode( LEDpin, OUTPUT);
  pinMode( triggerPin, OUTPUT);
  pinMode( redPin, OUTPUT);
  pinMode( bluPin, OUTPUT);
  pinMode( grnPin, OUTPUT);


  //  pinMode( doutPin, OUTPUT);
  pinMode (Dout1Pin, OUTPUT);
  pinMode (Dout2Pin, OUTPUT);
  pinMode (Dout3Pin, OUTPUT);
  pinMode (Dout4Pin, OUTPUT);
  pinMode (digitalPin, OUTPUT);

  //  pinMode( boardTypePin, INPUT_PULLUP);
#ifndef SERVOCONTROL
  digitalWrite(DIRpin, LOW);    // assume paralllel mode for FPGA interface // assume series mode
#else
  digitalWrite(DIRpin, HIGH);   // for servo mode, feed 'TX' out to servo pin
#endif
  digitalWrite(LLpin, HIGH);
  digitalWrite(URpin, HIGH);
  digitalWrite(LRpin, HIGH);
  digitalWrite(ULpin, HIGH);
  digitalWrite(LEDpin, LOW);
  digitalWrite(triggerPin, LOW);
  digitalWrite(redPin, LOW);
  digitalWrite(bluPin, LOW);
  digitalWrite(grnPin, LOW);
#ifdef EN5to8
  pinMode( en5Pin, OUTPUT);
  pinMode( en6Pin, OUTPUT);
  pinMode( en7Pin, OUTPUT);
  pinMode( en8Pin, OUTPUT);
  digitalWrite( en5Pin, HIGH);
  digitalWrite( en6Pin, HIGH);
  digitalWrite( en7Pin, HIGH);
  digitalWrite( en8Pin, HIGH);
#endif

#ifdef PWMCONTROL
  pinMode( redPWMpin, INPUT);
  pinMode( grnPWMpin, INPUT);
#endif  

  //  digitalWrite( doutPin, HIGH);
  digitalWrite( Dout1Pin, LOW);
  digitalWrite( Dout2Pin, LOW);
  digitalWrite( Dout3Pin, LOW);
  digitalWrite( Dout4Pin, LOW);
  digitalWrite( digitalPin, LOW);

  SPI.begin();

  Serial.begin(19200);   // USB comms
#ifndef SERVOCONTROL 
  Serial1.begin(250000); // inter board comms
#else  
  rgbServo.attach(1);  
#endif
  
  ledsOff(0x0F);   // quadrants off

  cmdInit(&Serial);
  cmdAdd("RED", redCmd);    // set RED to % brightness, update actual LEDs if they are on, 0 = off
  cmdAdd("CHR", redCmd);    //  "
  cmdAdd("BLUE", blueCmd);  // set BLUE to % brightness
  cmdAdd("BLU", blueCmd);   //  "
  cmdAdd("GREEN", greenCmd); // set GREEN to % brightness
  cmdAdd("GRN", greenCmd);  //  "
  cmdAdd("IR", irCmd);      // set IR to % brightness
  cmdAdd("DIGITAL", digitalCmd);
  cmdAdd("DIG", digitalCmd);
  cmdAdd("SHOCK", digitalCmd);
  cmdAdd("DON", digitalOnCmd);
  cmdAdd("DOFF", digitalOffCmd);
  cmdAdd("ON", onCmd);      // turn ON selected quadrant(s) on selected panel(s) 0 = all
  cmdAdd("OFF", offCmd);    // same but OFF
  cmdAdd("RESET", resetCmd);
  cmdAdd("PULSE", pulseCmd);
  cmdAdd("RUN", runCmd);
  cmdAdd("STOP", stopCmd);
  cmdAdd("PAUSE", pauseCmd);
  cmdAdd("PATT", patternCmd);  // depracated
  cmdAdd("SYNC", syncRateCmd);
  cmdAdd("ADDONESTEP", addOneStep); // upload one experiment step to the buffer.
  cmdAdd("GETEXPERIMENTSTEPS", getExperimentsteps); // return all experimental steps for debugging;
  cmdAdd("GS", getExperimentsteps);
  cmdAdd("REMOVEALLSTEPS", removeAllSteps); //  remove all steps from the buffer;
  cmdAdd("RX", runExperiment); // start experiment;
  cmdAdd("RUNEXPERIMENT", runExperiment); // start experiment;
  cmdAdd("STOPEXPERIMENT", StopExperiment); // stop experiment;
  cmdAdd("SX", StopExperiment);
  cmdAdd("FLYBOWLENABLED", flyBowlEnabled);  // choose which bowls are enabled for experiments;
  cmdAdd("GETEXPERIMENTSTATUS", getExperimentStatus); // return the status of the current experiment;
  cmdAdd("GX", getExperimentStatus);
  cmdAdd("MARKER", markerCmd);
  cmdAdd("DAC", DACCmd);
  cmdAdd("???", helpCmd);
  cmdAdd("HELP", listCommands);
  cmdAdd("WDT", wdtCmd);
  cmdAdd("GETDACS", getdacsCmd);
  cmdAdd("GAIN", gainCmd);
  cmdAdd("OFFSET", offsetCmd);
  cmdAdd("STARTSHOCKER", startShockCmd);
  cmdAdd("STOPSHOCKER", stopShockCmd);
  cmdAdd("RS", startShockCmd);
  cmdAdd("SS", stopShockCmd);
  cmdAdd("BLINK", blinkCmd);
  cmdAdd("PWM", PWMcmd);
  cmdAdd("STEPORDER", stepOrderCmd);
  cmdAdd("STATE", stateCmd); // return state machine status
  cmdAdd("ENABLE", enableCmd); // enable quadrants
  cmdAdd("SYNCON", SyncOnCmd); // start sync output
  #ifdef SERVOCONTROL
  cmdAdd("SERVO", servoCmd);
  #endif


  //    Serial2.begin(19200);  // modular board comms

  for ( int rgb = 0; rgb < 4; rgb++)
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

//  syncTimer.begin(syncInt, 500000 / syncRate); // 1e6 counts/sec  / rate

  for ( int idx = 0; idx < MAXSTEPS; idx++ ) // start with no set protocol
  {
    xStep[idx].type = 0;
  }

  // if gains not initialized, then do it

  if ( EEPROM.read(EE_INITED) != 0x55 )
  {
    for ( int i = 0; i < 16; i++ )
    {
      writeFloat( i * 4, 1.0);
      writeUint( OFFSET_ADR + i * 2, 0);
    }  
    EEPROM.write(EE_INITED, 0x55);
  }
  for ( int clr = 0; clr < 4; clr++)
  {
    for ( int quad = 0; quad < 4; quad++ )
    {
      LEDgain[clr][quad] = readFloat(clr * 16 +  quad * 4);
      LEDoffset[clr][quad] =  readUint( OFFSET_ADR + clr * 8 + quad * 2);
    }  
  }
  
//
//  Serial.println(OFFSET_ADR);
//
//  for( int i = 0; i < 100; i++ )
//  {
//     Serial.print(i);
//     Serial.print(" ");
//     Serial.println(EEPROM.read(i));
//  }  

 
#ifdef PWMCONTROL
  onQuadrants = 0xff;
#endif

#ifdef ANALOGCONTROL
  onQuadrants = 0xff;
#endif

#ifndef __MKL26Z64__
  // Setup WDT
  watchdog.enable(Watchdog::TIMEOUT_4S);
#endif

  #ifdef PWMCONTROL
     attachInterrupt(digitalPinToInterrupt(redPWMpin), PWMintR, CHANGE);
     attachInterrupt(digitalPinToInterrupt(grnPWMpin), PWMintG, CHANGE);
//     NVIC_SET_PRIORITY(IRQ_PORTD, 0);  // fastest priority for sync pin
  #endif    

} // END SETUP

uint32_t stepStart;



// ===================
// ===== L O O P =====
// ===================

void loop()
{
#ifndef __MKL26Z64__
  watchdog.reset(); // nice doggy!
#endif

  cmdPoll();

#ifndef SERVOCONTROL
  interBoardPoll();
#endif
  
  for( int rgb = 0; rgb < 4; rgb++ )
  {
    if ( pulse_enabled[rgb] )
    {
      if ( pulse_state[rgb] != last_state[rgb] )
      {
#ifdef DEBUG
        if ( pulse_state[rgb] == START) Serial.println("=================");
        Serial.print("NEW STATE ");
        Serial.print(experimentMsecs); //millis()-stepStart);
        Serial.print(" ");
        Serial.print(pulse_state[rgb], BIN);
        Serial.print(" ");
        Serial.print(colors[rgb]);
        Serial.print(":");
        Serial.print(states[last_state[rgb]]);
        Serial.print("->");
        Serial.println(states[pulse_state[rgb]]);
#endif
        last_state[rgb] = pulse_state[rgb];
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

          if ( wait_msec[rgb] > 0 ) // wait time request
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
          blink(); // see if blinking is enabled, and if so, do it
          break;

        case WAIT:
          if ( flag[rgb] == 0 ) // wait for WAIT params to take
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
          if ( flag[rgb] == 0 ) // wait for ON params to 'take'
          {
            digitalWrite(LEDpin, HIGH);
            markerOn(rgb);

            if ( pulse_off_msec[rgb] == 0 ) // continuous on mode?
            { // if so, we will 'fake' continuous on by staying in on mode for N x pulse_width msecs
              pulse_counter[rgb]--;   // one more pulse done
              if ( pulse_counter[rgb] <= 0 )  // this pulse set done
              {
                pulse_counter[rgb] = pulse_count[rgb];    // reset pulse counter for next go round
                noInterrupts();
                ledon[rgb] = 0;
                flag[rgb] = 1;
                next_timer[rgb] = off_time_msec[rgb];
                interrupts();
                pulse_state[rgb] = OFFTIME;
              }
              else // more pulses
              {
                //pulse_state[rgb] = STEPWAIT;
                noInterrupts();
                ledon[rgb] = 1;
                flag[rgb] = 1;
                next_timer[rgb] = pulse_on_msec[rgb];
                interrupts();
                pulse_state[rgb] = PULSEON;  // stay in pulse on
              } // endif counting pulses
            }
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
          if ( flag[rgb] == 0 ) // wait for OFF params to 'take'
          {
            pulse_counter[rgb]--;
            // if we are still doing a string of pulses OR we have no off time
            // then swing around
            if ( pulse_counter[rgb] > 0 )
            {
              noInterrupts();
              ledon[rgb] = 1;
              flag[rgb] = 1;
              next_timer[rgb] = pulse_on_msec[rgb];
              interrupts();
              pulse_state[rgb] = PULSEON;
            }
            else  // done counting and need off time
            {
              //                  if( off_time_msec != 0 )
              //                  {
              noInterrupts();
              ledon[rgb] = 0;
              flag[rgb] = 1;
              next_timer[rgb] = off_time_msec[rgb];
              interrupts();
              pulse_state[rgb] = OFFTIME;
              pulse_counter[rgb] = pulse_count[rgb];    // reset pulse counter for next go round
              markerOff(rgb);
            } // endif counting or offtime
          }
          break;

        case OFFTIME:
          if ( flag[rgb] == 0 ) // wait for OFF params to 'take'
          {
            digitalWrite(LEDpin, LOW);
            markerOff(rgb);

            iteration[rgb]++;   // count number of iterations of pulse trains
            if ( (iteration_loops[rgb] == 0) || (iteration[rgb] < iteration_loops[rgb]) ) // still have more to do, loop back to pulse on
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
              if (xIdx == 0) //  || (xStep[xIdx].type == 0) )   // xIdx is 0 if not running an experiment (or it is over)
              {
                //  xIdx  = 0;  // if end of experiment, reset pointer
                pulse_state[rgb] = HOLD;
              }
              else
              {
                pulse_state[rgb] = STEPWAIT;
#ifdef DEBUG
Serial.print("STEPWAIT");
Serial.print(millis());
Serial.print(" ");
Serial.print(stepStart);
Serial.print(" ");
Serial.println(stepDuration);
#endif
              }
            }
          }
          break;

        case STEPWAIT:  // wait for step end, 
          // this step is common to all colors, first one to get here 
          // will set things up for all colors 
          if ( (millis() - stepStart ) >= stepDuration )  
          {
            if( orderCount > 0 )
            {
                xIdx = stepOrder[nextOrderedStep++];
            }    
            else 
            {   
                xIdx++; //  point to next
            }    
            
#ifdef DEBUG
            Serial.println("++++++++++++++++++++++++");
            Serial.println( millis() - stepStart);
            Serial.print("dur ");
            Serial.println(stepDuration);
            Serial.print("Step:");
            Serial.print(xIdx);
            Serial.print(" is ");
            Serial.println(xStep[xIdx].type);
#endif            
          
            if (xStep[xIdx].type == PULSE_TYPE)
            {

              // If a pattern was set for this step then set it up
              for ( uint8_t color = 0; color < 3; color++)
                newIntensity( 0.0, color, 0, 0x0f );  // turn off all colors before setting pattern

              if ( xStep[xIdx].usePattern ) // set the pattern - this requires enabling active quadrants
              {
                uint16_t bitpat = xStep[xIdx].pattern;
                uint8_t quadbits;
                for ( uint8_t pan = 1; pan < 5; pan++)
                {
                  quadbits = bitpat & 0x0f;
                  bitpat = bitpat >> 4;
                  setOnOff( pan, quadbits, 1 );
                }
              }
              else // not a pattern
              {
                setOnOff( 0, 0x0f, 1 );   // set all active
              }

              setOnOff(0, 0, 0); // now turn everything off before setting intensities

              for ( int i = 0; i < 3; i++)
              {
                newIntensity( xStep[xIdx].intensity[i], i, bowls, 0x0f);
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

                #ifdef DEBUG
                  Serial.print("++++++++++ S T E P ");
                  Serial.print(xIdx);
                  Serial.println("++++++++++++++++++");
                #endif  

                if ( pulse_on_msec[i] > 0 ) //xStep[xIdx].intensity[i] > 0.001)
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
              }  // end loop setting intensities
              stepDuration = xStep[xIdx].duration * 1000;
            }
            else  // id valid == false, then we are done
            {             
              for ( int i = 0; i < 3; i++)
              {
                markerOff(i);
                pulse_state[i] = HOLD;             
              }
              
              if( clockOutEnabled ) 
                 syncTimer.end();        // if auto trigger off is wanted
              digitalWrite( triggerPin, LOW);  // trigger out low at end of experiment
              state = false;
              if( IRwasOff ) newIntensity( 0.0, IR, 0, 0x0f); // turn IR off
              xIdx = 0;  // show we are done

            }  // endif check ID
          }
          break;


        case HOLD: // single pulse mode leaves us here to hang out until next RUN command
          // markerOff(rgb); // for( uint8_t rgb = 0; rgb < 3; rgb++)  markerOff(rgb);  // leds and markerState should be off here
//          if( !clockOutEnabled ) digitalWrite( triggerPin, LOW);  // trigger out low at end of experiment
//          if( IRwasOff ) newIntensity( 0.0, IR, 0, 0x0f); // turn IR off
          break;

        case TURNOFF:
          ledsOff(0);
          markerOff(rgb);
          pulse_state[rgb] = OFFSTATE;
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
  if ( digitalRead(SYNCpin) == HIGH)
  {
    allPanelsOn();
    // LEDsOn(0);
  }
  else
  {
    allPanelsOff();
    LEDsOff(0);
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

//    #ifdef PWMCONTROL
//    if( rgb < 3 )
//    { 
//      noInterrupts();
//      if( newWidth[rgb] == false )
//      {
//         PW[rgb] = 0;
////         if(rgb == RED) Serial.println(rgb);
//      }   
//      long testWidth = PW[rgb];  // don't commit new value until we test it's within bounds
//      interrupts();
//
//      if( cnt >= 100 )
//      {
//         Serial.print(testWidth);
//         Serial.print(" ");
//          
//      }
//      if( rgb == 2 )
//      {
//         cnt++;
//         if( cnt > 100 )
//         {  
//           cnt = 0;
//           Serial.println();
//         }        
//      }    
//     
//      if( (testWidth >= 0) && (testWidth <= 95*20) )
//      {
//         width[rgb] = testWidth;
//         if( (abs(width[rgb] - lastWidth[rgb])) > 20) 
//         {
// //           Serial.print(rgb);
//            newIntensity( (float)width[rgb]/20, rgb, 0, 0xff);
//         }
//         lastWidth[rgb] = width[rgb];  
//         if( width[rgb] > 0 ) 
//             bitSet(newrgb, rgb);       
//         else 
//             bitClear(newrgb, rgb);
//       }
//       newWidth[rgb]= false;    
//    }  
//    #endif

  
