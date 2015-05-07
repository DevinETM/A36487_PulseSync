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
#include "A36043.h"
#include "Initialize.h"
#include "ETM_BUFFER_BYTE_64.h"
#include "P1395_CAN_SLAVE.h"


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
    unsigned int can_counter_ms;
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
    
    //unsigned int pulses_off;
    //unsigned int prf_pulse_counter;
    //unsigned int prf_counter_100ms;
    //unsigned int can_counter_100ms;
    //unsigned int heartbeat;
    //unsigned int can_comm_ok;
    //unsigned char prf;
    //unsigned char prf_ok_to_pulse;  //Limits the prf to 2.4ms period
    unsigned char personality;      //1=UL, 2=L, 3=M, 4=H
    unsigned char last_trigger_filtered;
    unsigned char energy;
    //unsigned char enable_high_voltage;
    //unsigned char enable_pulses;
    //unsigned char local_state;      //same definitions as system state

    unsigned int rep_rate_deci_herz;
} PSB_DATA;

//typedef struct{
    //unsigned char trigger_fault;
    //unsigned char mismatch_fault;
    //unsigned char keylock_fault;
    //unsigned char panel_fault;
    //unsigned char prf_fault;
    // unsigned char can_comm_fault;
    // unsigned char inhibit_pulsing;  //Inhibit all output pulses
    // unsigned char reset_faults;     //From ECB

//} PSB_FAULTS;

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
//extern PSB_FAULTS psb_faults;
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
#define UART2_BAUDRATE             38400
#define A35997_U2MODE_VALUE        (UART_DIS & UART_IDLE_STOP & UART_RX_TX & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT)
  //#define A35997_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35997_U2STA_VALUE         (UART_INT_TX & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35997_U2BRG_VALUE         (unsigned int)(((FCY/UART2_BAUDRATE)/16)-1)




#define _STATUS_CUSTOMER_HV_DISABLE                _STATUS_0
#define _STATUS_CUSTOMER_X_RAY_DISABLE             _STATUS_1

#define _STATUS_LOW_MODE_OVERRIDE                  _STATUS_5
#define _STATUS_HIGH_MODE_OVERRIDE                 _STATUS_6

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

#endif
