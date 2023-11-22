#define MAXQUAD 4

#define CHRIMSON 0
#define IR 1


// percents over 46 are look up - quasi-log scale
static uint16_t scaling[] = 
{
49,
54,
58,
64,
69,
75,
82,
89,
97,
105,
114,
124,
135,
147,
159,
173,
188,
205,
222,
242,
263,
286,
310,
337,
367,
398,
433,
471,
512,
556,
604,
657,
714,
776,
843,
916,
995,
1082,
1176,
1278,
1389,
1509,
1640,
1782,
1937,
2105,
2288,
2486,
2702,
2936,
3191,
3468,
3769,
4095,
};



void SetLEDs(uint16_t state)
{
  
 // Serial.print("set ");
 // Serial.println(state);                                                       // col, row
   if( state & 0x0001 ) digitalWrite(PIN00, HIGH); else digitalWrite(PIN00, LOW); // 1,1
   if( state & 0x0002 ) digitalWrite(PIN10, HIGH); else digitalWrite(PIN10, LOW); // 2,1
   if( state & 0x0004 ) digitalWrite(PIN20, HIGH); else digitalWrite(PIN20, LOW); // 3,1
   if( state & 0x0008 ) digitalWrite(PIN30, HIGH); else digitalWrite(PIN30, LOW);
   
   if( state & 0x0010 ) digitalWrite(PIN01, HIGH); else digitalWrite(PIN01, LOW);
   if( state & 0x0020 ) digitalWrite(PIN11, HIGH); else digitalWrite(PIN11, LOW);
   if( state & 0x0040 ) digitalWrite(PIN21, HIGH); else digitalWrite(PIN21, LOW);
   if( state & 0x0080 ) digitalWrite(PIN31, HIGH); else digitalWrite(PIN31, LOW); 
 
   if( state & 0x0100 ) digitalWrite(PIN02, HIGH); else digitalWrite(PIN02, LOW);
   if( state & 0x0200 ) digitalWrite(PIN12, HIGH); else digitalWrite(PIN12, LOW);
   if( state & 0x0400 ) digitalWrite(PIN22, HIGH); else digitalWrite(PIN22, LOW);
   if( state & 0x0800 ) digitalWrite(PIN32, HIGH); else digitalWrite(PIN32, LOW);
   
   if( state & 0x1000 ) digitalWrite(PIN03, HIGH); else digitalWrite(PIN03, LOW);
   if( state & 0x2000 ) digitalWrite(PIN13, HIGH); else digitalWrite(PIN13, LOW);
   if( state & 0x4000 ) digitalWrite(PIN23, HIGH); else digitalWrite(PIN23, LOW);
   if( state & 0x8000 ) digitalWrite(PIN33, HIGH); else digitalWrite(PIN33, LOW);

    marker(state); // if any state is active turn on marker pin

//
//#ifdef LINEAR_DRIVER   // linear driver sources LED current
//   if( state )
//     digitalWrite(MARKERPIN, HIGH);
//   else  
//     digitalWrite(MARKERPIN, LOW);
//#else
//   if( state )
//     digitalWrite(MARKERPIN, LOW);
//   else  
//     digitalWrite(MARKERPIN, HIGH);
//#endif
     
}

void LEDsOn(void)
{
    SetLEDs(setting);
}

void LEDsOff(void)
{
    SetLEDs(0x00); 
}  

void SetIntensity(uint16_t val)
{
  
  if(val > 4095) val = 4095;
  
   Wire.beginTransmission(0x0d); // transmit to device #44 (0x2c)
                              // device address is specified in datasheet
  Wire.write(0x01);             // pointer byte - sends value byte  
//  Wire.endTransmission(0);
//  Wire.beginTransmission(0x0d);
  Wire.write(0x20 | (val >> 8) );  
  Wire.write(val & 0xff );    
  Wire.endTransmission(1);     // stop transmitting
  Serial.print(val);
  Serial.println(" I-ok");
}

void SetBacklight(uint16_t val)
{
   Wire.beginTransmission(0x0d); // transmit to device #44 (0x2c)
                              // device address is specified in datasheet
  Wire.write(0x02);             // sends value byte  
//  Wire.endTransmission(0);
//  Wire.beginTransmission(0x0d);
  Wire.write(0x20 | (val >> 8) );  
  Wire.write(val & 0xff );    
  Wire.endTransmission(1);     // stop transmitting
 Serial.print(val); 
  Serial.println("B-ok");
}


int BRIGHTcmd(char * cmdp, int chrimson)
{
// int quad;
float percent;

//  while ( isdigit(*cmdp) == 0 )
//  {
//     if( *cmdp == '\0') 
//       return -1;
//     else  
//       cmdp++;  // get past any non-digit
//  }     
  
//  quad = atoi(cmdp);  // get quad 
 
//  if( (quad < CHRIMSON) || (quad > IR ) ) return -2;

//Serial.println(cmdp);

  while( isdigit(*cmdp) ) cmdp++;  
  while ( isdigit(*cmdp) == 0 )
  {
     if( *cmdp == '\0') 
     {
       return 0;
     }  
     else  
       cmdp++;  // get past any non-digit
  }   
  
  percent = atof(cmdp);  // get percentage
  if( (percent < 0) || (percent > 100) ) return -5;
 
   // we set the led here
  
 // Serial.println(quad); 
//  Serial.println(percent);
  
  if( percent == 0 ) 
  {
     if( chrimson ) 
        SetIntensity(0);
     else 
        SetBacklight(0);   
  }
  else
  {    
       // set min to ~.5 V = ~ 900 counts
    // max = 4000, diff = 3100 
    // so 900 + 31 * percent
    if(chrimson)
    {
   

#ifdef LINEAR_DRIVER 
  if( bright_mode == LOG )  //#ifdef USE_SCALING   
  {
    if( percent <= 46 ) 
        SetIntensity((int)percent);
    else
        SetIntensity( scaling[(int)percent-47]);         
  }      
  else //#else
  {
    percent *= 40.95;
    if( EEPROM.read(0) == 0x55 )
    {   
       float fcal = 1.00;
       fcal = EEPROM_readFloat( 1);
       percent = (uint16_t) (percent * fcal);
       if (percent > 4095 ) percent = 4095;
    } 
     
    SetIntensity(percent); //0-100 -> 0-4000
  } // #endif // use scaling
#else // switching driver
  if( bright_mode == LOG )
  { //#ifdef USE_SCALING
   if( percent <= 46 )
        SetIntensity(750+(int)percent);
    else       
        SetIntensity(750+scaling[(int)percent-47]);
  }
  else  //  #else
  {
        SetIntensity(750 + ((int)percent * 34));
  } //#endif   //log scaling    
#endif  // linear drive
    }   
    else
    {
#ifdef LINEAR_DRIVER
        SetBacklight((uint16_t)(percent * 40.95));
#else
        SetBacklight(900 + ((int)percent * 31)); 
#endif        
    }     

         
  }
  
  return(0);  
}


int  LEDcmd(char * cmdp, int onoff)
{
int row;
int col;
uint16_t val;


  while ( isdigit(*cmdp) == 0 )
  {
     if( *cmdp == '\0') 
       break;
     else  
       cmdp++;  // get past any non-digit
  }     
  
  if( *cmdp != '\0' ) // if not empty then get col and row info
  {
  
    col = atoi(cmdp);  // get column
    
    if( (col < 0) || (col > 4 ) ) return -2;
   
    Serial.print(col);  
    
    while( isdigit(*cmdp) ) cmdp++;
   
    while ( isdigit(*cmdp) == 0 )
    {
       if( *cmdp == '\0') 
         return -1;
       else  
         cmdp++;  // get past any non-digit
    }     
    
    row = atoi(cmdp);  // get column
    if( (row < 0) || (row > 4 ) ) return -3;
  
     Serial.print(",");  
     Serial.println(row);  
    
   // while( isdigit(*cmdp) ) cmdp++;
   
  //  while ( isdigit(*cmdp) == 0 )
  //  {
  //     if( *cmdp == '\0') 
  //       return -1;
  //     else  
  //       cmdp++;  // get past any non-digit
  //  }     
    
  //  onoff = atoi(cmdp);  // get column
  
    if( (onoff < 0) || (onoff > 1) ) return -3;
  //  Serial.println(col); 
  
     if( row == 0 )
     {
        if( col == 0 )
        {
            if( onoff == 0 ) 
              setting = 0x0000;
            else
              setting = 0xffff;  
        }
        else 
        {
            val = 0x1111 << (col-1);
            if( onoff == 0 )
            {
                val ^= 0xffff;         
                setting &= val;
            }    
            else
            {
                setting |= val;
            }   
        }
     }
     else if( col == 0 )
     {
        if( row == 0 )
        {
            if( onoff == 0 ) 
              setting = 0x0000;
            else
              setting = 0xffff;  
        }
        else 
        {
            val = 0x000f << (4*(row-1));
            if( onoff == 0 )
            {
                val ^= 0xffff;         
                setting &= val;
            }    
            else
            {
                setting |= val;
            }   
        }
     }
     else   
     {     
  //       row = 4 - row;
  //       col = 4 - col;
     row--;
     col--;
         
         if( onoff == 0)
         {
             val = 0xffff;
             val ^= 1 << ((row * 4) + col);
             setting &= val;
             
         }
         else
         {
  
             setting |= 1 << ((row * 4) + col);
            
         }
     }   
     
     SetLEDs(setting);
      
  }  // end read col and row info 
  else // use existing settings
  {
      if( onoff == 1 ) // on
          SetLEDs(setting);
      else
          SetLEDs(0x0000); //turn off    
  }
  
  Serial.println(setting, HEX);
   
  return 0;
} 

int PATTcmd(char * cmdp)
{  
uint16_t tmpsetting;
uint8_t count = 0;

  while ( isdigit(*cmdp) == 0 )
  {
     if( *cmdp == '\0') 
       return -1;
     else  
       cmdp++;  // get past any non-digit
  }     
  
  tmpsetting = 0;  // assume off

  for (count = 0; count < 16; count++ )
  {
    
      tmpsetting = tmpsetting << 1;  // make room for next
      if( *cmdp == '1')
      {   
          tmpsetting |= 1;               // only need to handle a '1' bit   
      }
      else if (*cmdp != '0' )
      {
          return -2; // either EOL or unexpected value
      }
      cmdp++;
  }    

  setting = tmpsetting;
  Serial.print("patt=");
  Serial.println(setting, HEX);

  return 0;
 
}  

int TSTcmd(char * cmdp)
{
int col;
int row;

   for( row = 1; row < 5; row ++ )
   {
         for( col = 1; col < 5; col++ )
         {
  
             setting |= 1 << (((row-1) * 4) + col-1);
             SetLEDs(setting);
                                      
             Serial.print(row);
             Serial.print(" ");
             Serial.println(col);
               
             delay(1000);    // wait a  sec  
        }  
   }
    return 0;
 
}  
