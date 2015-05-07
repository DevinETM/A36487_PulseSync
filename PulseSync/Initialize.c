#include "Initialize.h"
#include "P1395_MODULE_CONFIG.h"
#include "FIRMWARE_VERSION.h"

void Initialize(void);
void InitPins(void);
void InitTimer4(void);
void InitINT3(void);
void InitTimer1(void);
void Initialize(void)
{
    InitPins();
    InitINT3(); //Trigger Interrupt
    InitTimer4();
    InitTimer1();

    etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
    etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
    etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;
}

void InitPins()
{
    //Trigger Measurement Pins
    TRIS_PIN_TRIG_INPUT             = TRIS_INPUT_MODE;
    PIN_PW_SHIFT_OUT                = !OLL_PW_SHIFT;
    PIN_PW_CLR_CNT_OUT              = !OLL_PW_CLR_CNT;                // clear active
    PIN_PW_HOLD_LOWRESET_OUT        = !OLL_PW_HOLD_LOWRESET;	 // reset active
    TRIS_PIN_PW_SHIFT_OUT           = TRIS_OUTPUT_MODE;
    TRIS_PIN_PW_CLR_CNT_OUT         = TRIS_OUTPUT_MODE;
    TRIS_PIN_PW_HOLD_LOWRESET_OUT   = TRIS_OUTPUT_MODE;
    TRIS_PIN_40US_IN1               = TRIS_INPUT_MODE;
    TRIS_PIN_40US_IN2               = TRIS_INPUT_MODE;
    TRIS_PIN_TRIG_INPUT             = TRIS_INPUT_MODE;

    // Personality ID Pins
    PIN_ID_SHIFT_OUT            = !OLL_ID_SHIFT;
    TRIS_PIN_ID_SHIFT_OUT       = TRIS_OUTPUT_MODE;
    PIN_ID_CLK_OUT              = !OLL_ID_CLK;
    TRIS_PIN_ID_CLK_OUT         = TRIS_OUTPUT_MODE;
    TRIS_PIN_ID_DATA_IN         = TRIS_INPUT_MODE;

    //Spare pins (not used in current application)
    TRIS_PIN_PACKAGE_ID1_IN         = TRIS_INPUT_MODE;
    TRIS_PIN_READY_FOR_ANALOG_OUT   = TRIS_OUTPUT_MODE;
    PIN_READY_FOR_ANALOG_OUT        = OLL_READY_FOR_ANALOG;

    //Control to PFN control board for Gantry/Portal Selection
    TRIS_PIN_MODE_OUT           = TRIS_OUTPUT_MODE;

    //Hardware Status
    TRIS_PIN_KEY_LOCK_IN            = TRIS_INPUT_MODE;
    TRIS_PIN_PANEL_IN               = TRIS_INPUT_MODE;
    TRIS_PIN_XRAY_CMD_MISMATCH_IN   = TRIS_INPUT_MODE;
//    PIN_CUSTOMER_BEAM_ENABLE_IN     = !ILL_CUSTOMER_BEAM_ENABLE;
    TRIS_PIN_CUSTOMER_BEAM_ENABLE_IN = TRIS_INPUT_MODE;
    PIN_CUSTOMER_XRAY_ON_IN         = !ILL_CUSTOMER_XRAY_ON;
    TRIS_PIN_CUSTOMER_XRAY_ON_IN    = TRIS_INPUT_MODE;

    //Energy Select Pins
    TRIS_PIN_LOW_MODE_IN        = TRIS_INPUT_MODE;
    TRIS_PIN_HIGH_MODE_IN 	= TRIS_INPUT_MODE;
    PIN_ENERGY_CPU_OUT          = !OLL_ENERGY_CPU;
    TRIS_PIN_ENERGY_CPU_OUT     = TRIS_OUTPUT_MODE;
    TRIS_PIN_AFC_TRIGGER_OK_OUT = TRIS_OUTPUT_MODE;
    PIN_AFC_TRIGGER_OK_OUT      = OLL_AFC_TRIGGER_OK;
    PIN_RF_POLARITY_OUT         = OLL_RF_POLARITY;
    TRIS_PIN_RF_POLARITY_OUT    = TRIS_OUTPUT_MODE;
    PIN_HVPS_POLARITY_OUT       = !OLL_HVPS_POLARITY;
    TRIS_PIN_HVPS_POLARITY_OUT  = TRIS_OUTPUT_MODE;
    PIN_GUN_POLARITY_OUT        = !OLL_GUN_POLARITY;
    TRIS_PIN_GUN_POLARITY_OUT   = TRIS_OUTPUT_MODE;
    TRIS_PIN_ENERGY_CMD_IN1     = TRIS_INPUT_MODE;
    TRIS_PIN_ENERGY_CMD_IN2     = TRIS_INPUT_MODE;
   
    //State Hardware Control
    TRIS_PIN_CPU_HV_ENABLE_OUT      = TRIS_OUTPUT_MODE;
    PIN_CPU_HV_ENABLE_OUT           = !OLL_CPU_HV_ENABLE;
    TRIS_PIN_CPU_XRAY_ENABLE_OUT    = TRIS_OUTPUT_MODE;
    PIN_CPU_XRAY_ENABLE_OUT         = !OLL_CPU_XRAY_ENABLE;
    TRIS_PIN_CPU_WARNING_LAMP_OUT   = TRIS_OUTPUT_MODE;
    PIN_CPU_WARNING_LAMP_OUT        = !OLL_CPU_WARNING_LAMP;
    TRIS_PIN_CPU_STANDBY_OUT        = TRIS_OUTPUT_MODE;
    PIN_CPU_STANDBY_OUT             = !OLL_CPU_STANDBY;
    TRIS_PIN_CPU_READY_OUT          = TRIS_OUTPUT_MODE;
    PIN_CPU_READY_OUT               = !OLL_CPU_READY;
    TRIS_PIN_CPU_SUMFLT_OUT         = TRIS_OUTPUT_MODE;
    PIN_CPU_SUMFLT_OUT              = !OLL_CPU_SUMFLT;
    TRIS_PIN_CPU_WARMUP_OUT         = TRIS_OUTPUT_MODE;
//    PIN_CPU_WARMUP_OUT              = !OLL_CPU_WARMUP;
    
    //LEDs
    TRIS_PIN_LED_READY          = TRIS_OUTPUT_MODE;
    PIN_LED_READY               = !OLL_LED_ON;
    TRIS_PIN_LED_STANDBY        = TRIS_OUTPUT_MODE;
    PIN_LED_STANDBY             = !OLL_LED_ON;
    TRIS_PIN_LED_WARMUP         = TRIS_OUTPUT_MODE;
//    PIN_LED_WARMUP              = !OLL_LED_ON;
    TRIS_PIN_LED_XRAY_ON        = TRIS_OUTPUT_MODE;
    PIN_LED_XRAY_ON             = !OLL_LED_ON;
    TRIS_PIN_LED_SUMFLT         = TRIS_OUTPUT_MODE;
    PIN_LED_SUMFLT              = !OLL_LED_ON;

    // Pins for loading the delay lines
    PIN_SPI_CLK_OUT             = 0;
    TRIS_PIN_SPI_CLK_OUT        = TRIS_OUTPUT_MODE;
    PIN_SPI_DATA_OUT            = 0;
    TRIS_PIN_SPI_DATA_OUT       = TRIS_OUTPUT_MODE;
    TRIS_PIN_SPI_DATA_IN        = TRIS_INPUT_MODE;
    TRIS_PIN_LD_DELAY_PFN_OUT   = TRIS_OUTPUT_MODE;
    PIN_LD_DELAY_PFN_OUT        = 0;
    TRIS_PIN_LD_DELAY_AFC_OUT   = TRIS_OUTPUT_MODE;
    PIN_LD_DELAY_AFC_OUT        = 0;
    TRIS_PIN_LD_DELAY_GUN_OUT   = TRIS_OUTPUT_MODE;
    PIN_LD_DELAY_GUN_OUT        = 0;

    //Bypass these to allow xray on
    TRIS_PIN_RF_OK             = TRIS_INPUT_MODE;
    TRIS_PIN_GUN_OK            = TRIS_INPUT_MODE;
    TRIS_PIN_PFN_OK            = TRIS_INPUT_MODE;

    //Communications
    COMM_DRIVER_ENABLE_TRIS = TRIS_OUTPUT_MODE;
    COMM_DRIVER_ENABLE_PIN = 0;
    COMM_RX_TRIS = TRIS_INPUT_MODE;
    COMM_TX_TRIS = TRIS_OUTPUT_MODE;

    ADPCFG = 0b0000000011111111;
    ADCON1 = 0x0000;
}

 void InitINT3()
 {
  // Set up Interrupts
  // Set up external INT3 */
  // This is the trigger interrupt
  _INT3IF = 0;		// Clear Interrupt flag
  _INT3IE = 1;		// Enable INT3 Interrupt
  _INT3EP = 1; 	        // Interrupt on falling edge
  _INT3IP = 7;		// Set interrupt to highest priority
 }

void InitTimer4(void)
{
#define T4_PERIOD_US 10000   // 10mS Period
  T4CON = (T4_ON & T4_IDLE_CON & T4_GATE_OFF & T4_PS_1_8 & T4_SOURCE_INT & T4_32BIT_MODE_OFF);
  PR4 = (FCY_CLK_MHZ*T4_PERIOD_US/8);
  TMR4 = 0;
  _T4IF = 0;

}
void InitTimer1(void) {
  // Setup Timer 1 to measure interpulse period.
  T1CON = (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT);
  PR1 = 62500;  // 400mS
}

