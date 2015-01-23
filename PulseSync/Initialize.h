#ifndef  __INITIALIZE_H
#define __INITIALIZE_H
#include <p30f6014A.h>
#include <timer.h>


//#include "ETM_BUFFER_BYTE_64.h"

// BufferByte Setup
#define COMMAND_BUFFER_EMPTY  0x00
#define COMMAND_BUFFER_FULL   0x02

// Various definitions
#define TRIS_OUTPUT_MODE 0
#define TRIS_INPUT_MODE  1

// ***Digital Pin Definitions***

// Personality module
#define PIN_ID_SHIFT_OUT                    _LATC2
#define TRIS_PIN_ID_SHIFT_OUT               _TRISC2
#define OLL_ID_SHIFT                        1
#define PIN_ID_CLK_OUT                      _LATC3
#define TRIS_PIN_ID_CLK_OUT                 _TRISC3
#define OLL_ID_CLK                          1
#define PIN_ID_DATA_IN                      _RC4
#define TRIS_PIN_ID_DATA_IN                 _TRISC4
#define ILL_ID_DATA                         1

// Spare (not used in current application)
#define TRIS_PIN_PACKAGE_ID1_IN             _TRISF3
#define PIN_PACKAGE_ID1_IN                  _RF3
#define ILL_PACKAGE_ID1_OK                  0
#define TRIS_PIN_READY_FOR_ANALOG_OUT       _TRISD15	//READY / !ADJUSTING
#define PIN_READY_FOR_ANALOG_OUT            _LATD15
#define OLL_READY_FOR_ANALOG                1

//Control to PFN control board for Gantry/Portal Selection
#define TRIS_PIN_MODE_OUT                   _TRISF2
#define PIN_MODE_OUT                        _RF2
#define OLL_MODE_GANTRY                     1
#define OLL_MODE_PORTAL                     0

// STATUS from board A35487
#define PIN_XRAY_CMD_MISMATCH_IN            _RD14
#define TRIS_PIN_XRAY_CMD_MISMATCH_IN       _TRISD14
#define ILL_XRAY_CMD_MISMATCH               1
#define PIN_LOW_MODE_IN                     _RB3
#define TRIS_PIN_LOW_MODE_IN                _TRISB3
#define ILL_LOW_MODE                        1
#define PIN_HIGH_MODE_IN                    _RB2
#define TRIS_PIN_HIGH_MODE_IN               _TRISB2
#define ILL_HIGH_MODE                       1
#define TRIS_PIN_KEY_LOCK_IN                _TRISF7	 
#define PIN_KEY_LOCK_IN                     _RF7
#define ILL_KEY_LOCK_ACTIVE                 1
#define TRIS_PIN_PANEL_IN                   _TRISF8	
#define PIN_PANEL_IN                        _RF8
#define ILL_PANEL_OPEN                      1

// CONTROL to board A35487
#define TRIS_PIN_CUSTOMER_BEAM_ENABLE_IN    _TRISG2
#define PIN_CUSTOMER_BEAM_ENABLE_IN         _RG2
#define ILL_CUSTOMER_BEAM_ENABLE            1
#define TRIS_PIN_CUSTOMER_XRAY_ON_IN        _TRISG3	
#define PIN_CUSTOMER_XRAY_ON_IN             _RG3
#define ILL_CUSTOMER_XRAY_ON                1
#define TRIS_PIN_CPU_XRAY_ENABLE_OUT        _TRISC13
#define TRIS_PIN_CPU_HV_ENABLE_OUT          _TRISD8
#define PIN_CPU_HV_ENABLE_OUT               _RD8
#define OLL_CPU_HV_ENABLE                   1
#define PIN_CPU_XRAY_ENABLE_OUT             _RC13
#define OLL_CPU_XRAY_ENABLE                 1
#define TRIS_PIN_CPU_WARNING_LAMP_OUT       _TRISD9
#define PIN_CPU_WARNING_LAMP_OUT            _RD9
#define OLL_CPU_WARNING_LAMP                1

// Customer Status pins
#define PIN_CPU_STANDBY_OUT                 _RB5
#define TRIS_PIN_CPU_STANDBY_OUT            _TRISB5
#define OLL_CPU_STANDBY                     1
#define PIN_CPU_READY_OUT                   _RB4
#define TRIS_PIN_CPU_READY_OUT              _TRISB4
#define OLL_CPU_READY                       1
#define TRIS_PIN_CPU_SUMFLT_OUT             _TRISD0
#define PIN_CPU_SUMFLT_OUT                  _RD0
#define OLL_CPU_SUMFLT                      1
#define TRIS_PIN_CPU_WARMUP_OUT             _TRISD10
#define PIN_CPU_WARMUP_OUT                  _RD10
#define OLL_CPU_WARMUP                      1

//     LEDS
#define PIN_LED_READY                       _LATG13
#define TRIS_PIN_LED_READY                  _TRISG13
#define OLL_LED_ON                          0
#define PIN_LED_STANDBY                     _LATG12
#define TRIS_PIN_LED_STANDBY                _TRISG12
//#define PIN_LED_WARMUP                      _LATG14  //This is used for CAN status
#define TRIS_PIN_LED_WARMUP                 _TRISG14
#define PIN_LED_XRAY_ON                     _LATG15
#define TRIS_PIN_LED_XRAY_ON                _TRISG15
#define PIN_LED_SUMFLT                      _LATC1
#define TRIS_PIN_LED_SUMFLT                 _TRISC1

//Energy Pins
#define TRIS_PIN_ENERGY_CPU_OUT             _TRISC14
#define PIN_ENERGY_CPU_OUT                  _LATC14
#define OLL_ENERGY_CPU                      0
#define TRIS_PIN_AFC_TRIGGER_OK_OUT         _TRISD7
#define PIN_AFC_TRIGGER_OK_OUT              _LATD7
#define OLL_AFC_TRIGGER_OK                  1
#define TRIS_PIN_RF_POLARITY_OUT            _TRISD1
#define PIN_RF_POLARITY_OUT                 _LATD1
#define OLL_RF_POLARITY                     0
#define TRIS_PIN_HVPS_POLARITY_OUT          _TRISD2
#define PIN_HVPS_POLARITY_OUT		    _LATD2
#define OLL_HVPS_POLARITY                   0
#define TRIS_PIN_GUN_POLARITY_OUT           _TRISD3
#define PIN_GUN_POLARITY_OUT		    _LATD3
#define OLL_GUN_POLARITY                    0

// Pins for the delay line shift registers
#define TRIS_PIN_SPI_CLK_OUT                _TRISG6
#define PIN_SPI_CLK_OUT                     _LATG6
#define TRIS_PIN_SPI_DATA_OUT               _TRISG8
#define PIN_SPI_DATA_OUT                    _LATG8
#define TRIS_PIN_SPI_DATA_IN                _TRISG7
#define PIN_SPI_DATA_IN                     _RG7
#define TRIS_PIN_LD_DELAY_PFN_OUT           _TRISD12
#define PIN_LD_DELAY_PFN_OUT		    _LATD12
#define TRIS_PIN_LD_DELAY_AFC_OUT           _TRISD13
#define PIN_LD_DELAY_AFC_OUT		    _LATD13
#define TRIS_PIN_LD_DELAY_GUN_OUT           _TRISD11
#define PIN_LD_DELAY_GUN_OUT		    _LATD11

// Trigger Pulse width measure
#define TRIS_PIN_PW_SHIFT_OUT               _TRISD4
#define PIN_PW_SHIFT_OUT                    _LATD4
#define OLL_PW_SHIFT                        1
#define TRIS_PIN_PW_CLR_CNT_OUT             _TRISD5
#define PIN_PW_CLR_CNT_OUT                  _LATD5
#define OLL_PW_CLR_CNT                      1
#define TRIS_PIN_PW_HOLD_LOWRESET_OUT       _TRISD6
#define PIN_PW_HOLD_LOWRESET_OUT            _LATD6
#define OLL_PW_HOLD_LOWRESET                1		 /* 1, hold VALID_PULSE, 0, reset the START and VALID_PULSE	*/
#define TRIS_PIN_TRIG_INPUT                 _TRISF6
#define PIN_TRIG_INPUT                      _RF6
#define ILL_TRIG_ON                         1

//Interrupt pins
#define TRIS_PIN_ENERGY_CMD_IN1             _TRISA12	//INT1
#define PIN_ENERGY_CMD_IN1		    _RA12
#define TRIS_PIN_ENERGY_CMD_IN2             _TRISA13	//INT2 tied to INT1
#define PIN_ENERGY_CMD_IN2		    _RA13
#define TRIS_PIN_40US_IN2                   _TRISA14	//INT3
#define PIN_40US_IN2                        _RA14
#define TRIS_PIN_40US_IN1                   _TRISA15	//INT4 tied to INT3
#define PIN_40US_IN1                        _RA15
#define COMM_DRIVER_ENABLE_TRIS             _TRISG1		//Enable the communications driver
#define COMM_DRIVER_ENABLE_PIN              _RG1

//Bypass these to allow xray on
#define TRIS_PIN_RF_OK                      _TRISA7
#define PIN_RF_OK                           _RA7
#define TRIS_PIN_GUN_OK                     _TRISA6
#define PIN_GUN_OK                          _RA6
#define TRIS_PIN_PFN_OK                     _TRISG0
#define PIN_PFN_OK                          _RG0

//Communications
#define COMM_RX_TRIS                        _TRISF4		//U2RX
#define COMM_RX_PIN                         _RF4
#define COMM_TX_TRIS                        _TRISF5		//U2TX
#define COMM_TX_PIN                         _RF5


void Initialize(void);

#endif
