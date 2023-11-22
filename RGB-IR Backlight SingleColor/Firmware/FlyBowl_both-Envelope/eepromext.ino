
#include <EEPROM.h>

void EEPROM_writeFloat(int ee, float fval)
{
   const byte* p = (const byte*)&fval;
   uint8_t i;
   for (i = 0; i < sizeof(fval); i++)
       EEPROM.write(ee++, *p++);
}

float EEPROM_readFloat(int ee)
{
   float fval;
   byte* p = (byte*)&fval;
   uint8_t i;
   for (i = 0; i < sizeof(fval); i++)
       *p++ = EEPROM.read(ee++);
   return fval;
}

