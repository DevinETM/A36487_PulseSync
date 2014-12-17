// PULSE SYNCHRONIZATION BOARD FIRMWARE
//
// DESCRIPTION:
//      1. Sample trigger input
//      2. Calculate the desired gun width/delay from the trigger width
//      3. Setup all the energy levels (for 6-4 linac only)
//      4. Manage states to customer from CANbus
//      5. Ensure the trigger does not stay high
//
// SOFTWARE FLOW:
//      1. Most of the functionality is in A36487.c
//      2. Using module BufferByte for UART transmission
//      3. Using module A36043 for personality module reading
//
// AUTHOR: Devin Henderlong 11/10/2014
//



#ifndef __main_h
#define __main_h
#include <p30f6014A.h>
#include <dsp.h>
#include <libpic30.h>
#include "A36487.h"


//Oscillator Setup
_FOSC(EC & CSW_FSCM_OFF); // Primary Oscillator without PLL and Startup with User Selected Oscillator Source, CLKOUT 10MHz is used to measure trigger width.
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_2);  // Watchdog Timer is enabled, 1024ms TIMEOUT
_FBORPOR(PWRT_64 & BORV_27 & PBOR_ON & MCLR_EN); // Brown out and Power on Timer settings
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


int main(void);

#endif