// ETM MODULE:  READ DOSE PERSONALITY MODULE
#include "A36043.h"

unsigned char ReadDosePersonality() {
      unsigned int data;
      unsigned char i, data1, data2;

      PIN_ID_CLK_OUT   = !OLL_ID_CLK;
      PIN_ID_SHIFT_OUT = !OLL_ID_SHIFT; // load the reg
      __delay32(1); // 100ns for 10M TCY
      PIN_ID_SHIFT_OUT = OLL_ID_SHIFT;  // enable shift
      __delay32(1); // 100ns for 10M TCY

      data = PIN_ID_DATA_IN;

      for (i = 0; i < 8; i++)
      {
      	PIN_ID_CLK_OUT = OLL_ID_CLK;
        data <<= 1;
        data |= PIN_ID_DATA_IN;
      	PIN_ID_CLK_OUT = !OLL_ID_CLK;
        __delay32(1); // 100ns for 10M TCY
      }

      //if bits do not match then bad module
      data1 = data & 0x01;
      data2 = data & 0x10;
      if (data1 != (data2 >> 4))
          return 0xFF;
      data1 = data & 0x02;
      data2 = data & 0x20;
      if (data1 != (data2 >> 4))
          return 0xFF;
      data1 = data & 0x04;
      data2 = data & 0x40;
      if (data1 != (data2 >> 4))
          return 0xFF;
      data1 = data & 0x08;
      data2 = data & 0x80;
      if (data1 != (data2 >> 4))
          return 0xFF;

      //bit 3 is 1 except when 0,1,2 are 1
      data1 = data & 0x08;
      data2 = data & 0x07;
      if (data1 != data2)
            return 0xFF;

      if (data == ULTRA_LOW_DOSE)
          return 0x02;
      else if (data == LOW_DOSE)
          return 0x04;
      else if (data == MEDIUM_DOSE)
          return 0x08;
      else if (data == HIGH_DOSE)
          return 0x10;
      else if (data == 0xFF)
          return data;
      else
        return 0;
}