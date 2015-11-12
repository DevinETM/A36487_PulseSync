// PULSE SYNCHRONIZATION BOARD FIRMWARE
//
// DESCRIPTION:
//      1. All the action happens here
//      2. Calling the initialize routine
//      3. Sampling the trigger width
//      4. Interpreting the customer energy command
//      5. Setting the appropriate energy levels
//      6. Programing the delay lines for the trigger delays/widths
//      7. Reading the personality module
//

#ifndef __A36487_h
#define __A36487_h
//#include "A36043.h"
//#include "Initialize.h"

#include <XC.h>
#include <libpic30.h>
#include <timer.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"




//These values are received from the ethernet control board
typedef struct{
    unsigned char grid_delay_high3;
    unsigned char grid_delay_high2;
    unsigned char grid_delay_high1;
    unsigned char grid_delay_high0;
    unsigned char grid_delay_low3;
    unsigned char grid_delay_low2;
    unsigned char grid_delay_low1;
    unsigned char grid_delay_low0;
    unsigned char grid_width_high3;
    unsigned char grid_width_high2;
    unsigned char grid_width_high1;
    unsigned char grid_width_high0;
    unsigned char grid_width_low3;
    unsigned char grid_width_low2;
    unsigned char grid_width_low1;
    unsigned char grid_width_low0;
    unsigned char pfn_delay_high;
    unsigned char pfn_delay_low;
    unsigned char afc_delay_high;
    unsigned char afc_delay_low;
    unsigned char dose_sample_delay_high;
    unsigned char dose_sample_delay_low;
    unsigned char magnetron_current_sample_delay_high;
    unsigned char magnetron_current_sample_delay_low;
} PULSE_PARAMETERS;




//These values are calculated or measured by the pulse sync board
typedef struct{
  //unsigned int can_counter_ms;
    unsigned int led_flash_counter;
    unsigned int counter_config_received;
    unsigned int state_machine;
    unsigned int led_state;        
    unsigned int last_period;
    unsigned int period_filtered;
    unsigned int pulses_on;
    unsigned int trigger_complete;
    unsigned char trigger_input;    //measured trigger
    unsigned char trigger_filtered; //filtered trigger
    unsigned char grid_delay;       //interpolated delay
    unsigned char grid_width;       //interpolated width
    unsigned char dose_sample_delay;         //calculated RF PCB Delay (target current)
    unsigned char pfn_delay;
    unsigned char afc_delay;
    unsigned char magnetron_current_sample_delay;
    
    unsigned char personality;      //1=UL, 2=L, 3=M, 4=H
    unsigned char last_trigger_filtered;
    unsigned char energy;

    unsigned int rep_rate_deci_herz;
} PSB_DATA;

typedef struct  {
  unsigned char command_byte;
  unsigned char register_byte;
  unsigned char data_high_byte;
  unsigned char data_low_byte;
  unsigned char data_state;
} CommandStringStruct;


//Global Variables
extern CommandStringStruct command_string;
extern BUFFERBYTE64 uart2_input_buffer;
extern BUFFERBYTE64 uart2_output_buffer;
extern PULSE_PARAMETERS psb_params;
extern PSB_DATA psb_data;


//Definitions
#define DOSE_LEVELS     11   //sets the amount of bits to converge into single dose level

#define HI              1
#define LOW             0
#define MAX_FREQUENCY   410 // Hz

#define STATE_INIT              10
#define STATE_WAIT_FOR_CONFIG   20
#define STATE_HV_OFF            30
#define STATE_HV_ENABLE         40
#define STATE_X_RAY_ENABLE      50
#define STATE_FAULT             60
#define STATE_UNKNOWN           70


//State bits in the customer LED register
#define SUM_FAULT       0b00010000
#define XRAY_ON         0b00001000
#define READY           0b00000100
#define STANDBY         0b00000010
#define WARMING_UP      0b00000001


//Oscillator Setup
#define FCY             10000000    //40MHz clock / 4 = 10MHz


/*
  --- UART Setup ---
  See uart.h and Microchip documentation for more information about the condfiguration
  // DPARKER cleanup this uart configuration
  */
//#define UART1_BAUDRATE             303000        // U1 Baud Rate
//#define UART2_BAUDRATE             38400
//#define A35997_U2MODE_VALUE        (UART_DIS & UART_IDLE_STOP & UART_RX_TX & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT)
  //#define A35997_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
//#define A35997_U2STA_VALUE         (UART_INT_TX & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
//#define A35997_U2BRG_VALUE         (unsigned int)(((FCY/UART2_BAUDRATE)/16)-1)


//#define _PERSONALITY_SELECT_BIT_0                  _CONTROL_3
//#define _PERSONALITY_SELECT_BIT_1                  _CONTROL_4


#define _STATUS_CUSTOMER_HV_DISABLE                _WARNING_0
#define _STATUS_CUSTOMER_X_RAY_DISABLE             _WARNING_1
#define _STATUS_LOW_MODE_OVERRIDE                  _WARNING_2
#define _STATUS_HIGH_MODE_OVERRIDE                 _WARNING_3
#define _STATUS_PERSONALITY_READ_COMPLETE          _WARNING_4

#define _PERSONALITY_BIT_0                         _NOT_LOGGED_0
#define _PERSONALITY_BIT_1                         _NOT_LOGGED_1
#define _PERSONALITY_BIT_2                         _NOT_LOGGED_2
#define _PERSONALITY_BIT_3                         _NOT_LOGGED_3


#define _FAULT_PANEL                               _FAULT_0
#define _FAULT_KEYLOCK                             _FAULT_1
#define _FAULT_TIMING_MISMATCH                     _FAULT_2
#define _FAULT_TRIGGER_STAYED_ON                   _FAULT_3
#define _FAULT_X_RAY_ON_WIHTOUT_HV                 _FAULT_4
#define _FAULT_SYNC_TIMEOUT                        _FAULT_5
#define _FAULT_PFN_STATUS                          _FAULT_6
#define _FAULT_RF_STATUS                           _FAULT_7

#define LED_WARMUP_STATUS                         (psb_data.led_state & 0x0001)
#define LED_STANDBY_STATUS                        (psb_data.led_state & 0x0002)
#define LED_READY_STATUS                          (psb_data.led_state & 0x0004)
#define LED_SUM_FAULT_STATUS                      (psb_data.led_state & 0x0008)

#define LED_STARTUP_FLASH_TIME                    300 // 3 Seconds
































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
#define ILL_KEY_LOCK_FAULT                  0

#define TRIS_PIN_PANEL_IN                   _TRISF8	
#define PIN_PANEL_IN                        _RF8
#define ILL_PANEL_OPEN                      0

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
//#define TRIS_PIN_LED_WARMUP                 _TRISG14
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
#define ILL_PIN_RF_FAULT                    0

#define TRIS_PIN_GUN_OK                     _TRISA6
#define PIN_GUN_OK                          _RA6

#define TRIS_PIN_PFN_OK                     _TRISG0
#define PIN_PFN_OK                          _RG0
#define ILL_PIN_PFN_FAULT                   0

//Communications
#define COMM_RX_TRIS                        _TRISF4		//U2RX
#define COMM_RX_PIN                         _RF4
#define COMM_TX_TRIS                        _TRISF5		//U2TX
#define COMM_TX_PIN                         _RF5























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
