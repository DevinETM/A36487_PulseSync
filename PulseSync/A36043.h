// ETM MODULE:  READ DOSE PERSONALITY MODULE
//
// MODULE REQUIREMENTS:
//  1. Hardware: A36043-XXX installed as a daugter board
//  2. Software:  Pin requirements (see defines below)
//
//
// MODULE DESCRIPTION:
//      Call ReadDosePersonailty() to obtain a char representing the dose
//      personality module installed
//
// MODULE AUTHOR: Devin Henderlong 11/7/2014
//

#ifndef __A36043_H
#define __A36043_H
#include <p30f6014A.h>
#include <libpic30.h>

unsigned char ReadDosePersonality(void);
//
//  Return:
//      0x00 = No dose personailty installed
//      0x02 = Ultra Low Dose Personality Installed
//      0x04 = Low Dose Personailty Installed
//      0x08 = Medium Dose Personality Installed
//      0x10 = High Dose Personailty Installed
//      0xFF = Problem reading personailty module
//

// These defines are what is actually read from the shift register
#define HIGH_DOSE       0x77
#define MEDIUM_DOSE     0xCC
#define LOW_DOSE        0xAA
#define ULTRA_LOW_DOSE  0x99

//Pin requirements
#define PIN_ID_SHIFT_OUT	_LATC2
#define TRIS_PIN_ID_SHIFT_OUT   _TRISC2
#define OLL_ID_SHIFT            1

#define PIN_ID_CLK_OUT 		_LATC3
#define TRIS_PIN_ID_CLK_OUT	_TRISC3
#define OLL_ID_CLK   		1

#define PIN_ID_DATA_IN          _RC4
#define TRIS_PIN_ID_DATA_IN     _TRISC4
#define ILL_ID_DATA  		1

#endif