
#include "Initialize.h"
//#include "ETM_BUFFER_BYTE_64.h"



void Initialize(void);
void InitPins(void);
void InitADC(void);
void LEDCheck(void);
void InitTimer4(void);
void InitTimer5(void);
void InitUART(void);
void InitINT3(void);
void InitINT4(void);

void Initialize(void)
{
    InitPins();
    MACRO_ClrWdt()
    LEDCheck();
    MACRO_ClrWdt()
    LEDCheck();
    MACRO_ClrWdt()
    //InitUART();
    InitINT3(); //Trigger Interrupt
    InitTimer4();
    InitTimer5();
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
    PIN_CUSTOMER_BEAM_ENABLE_IN     = !ILL_CUSTOMER_BEAM_ENABLE;
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
    TRIS_PIN_RF_OK             = TRIS_OUTPUT_MODE;
    PIN_RF_OK                  = 1;
//    TRIS_PIN_GUN_OK            = TRIS_OUTPUT_MODE;
//    PIN_GUN_OK                 = 1;
    TRIS_PIN_PFN_OK            = TRIS_OUTPUT_MODE;
    PIN_PFN_OK                 = 1;

    //Communications
    COMM_DRIVER_ENABLE_TRIS = TRIS_OUTPUT_MODE;
    COMM_DRIVER_ENABLE_PIN = 0;
    COMM_RX_TRIS = TRIS_INPUT_MODE;
    COMM_TX_TRIS = TRIS_OUTPUT_MODE;
}

void LEDCheck(void)
{
	unsigned int y;
	unsigned int z = 0xFFFF;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;
	
	while (y != z)
		y++;

        PIN_LED_WARMUP = OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = OLL_LED_ON;
        PIN_LED_STANDBY = OLL_LED_ON;
	PIN_LED_READY = OLL_LED_ON;
	PIN_LED_XRAY_ON = OLL_LED_ON;
	PIN_LED_SUMFLT = OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = OLL_LED_ON;
        PIN_LED_STANDBY = OLL_LED_ON;
	PIN_LED_READY = OLL_LED_ON;
	PIN_LED_XRAY_ON = OLL_LED_ON;
	PIN_LED_SUMFLT = OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = OLL_LED_ON;
        PIN_LED_STANDBY = OLL_LED_ON;
	PIN_LED_READY = OLL_LED_ON;
	PIN_LED_XRAY_ON = OLL_LED_ON;
	PIN_LED_SUMFLT = OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = !OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;

	y = 0;
	while (y != z)
		y++;

        PIN_LED_WARMUP = OLL_LED_ON;
        PIN_LED_STANDBY = !OLL_LED_ON;
	PIN_LED_READY = !OLL_LED_ON;
	PIN_LED_XRAY_ON = !OLL_LED_ON;
	PIN_LED_SUMFLT = !OLL_LED_ON;
}


 void InitUART()
 {
  // Initialize the UART

  // ----------------- UART #2 Setup and Data Buffer -------------------------//
  // Setup the UART input and output buffers
  BufferByte64Initialize(&uart2_input_buffer);
  BufferByte64Initialize(&uart2_output_buffer);

  //U1MODE = A35997_U1MODE_VALUE;
  // U1STA = A35997_U1STA_VALUE;
  U2BRG = A35997_U2BRG_VALUE;
  U2MODE = 0b0000000000000000;

  // Begin UART operation
  command_string.data_state = COMMAND_BUFFER_EMPTY;  // The command buffer is empty

  _U2TXIF = 0;	// Clear the Transmit Interrupt Flag
  _U2RXIF = 0;	// Clear the Recieve Interrupt Flag
  _U2TXIE = 1;	// Enable Transmit Interrupts
  _U2TXIP = 1;  // Low Priority
  _U2RXIE = 1;	// Enable Recieve Interrupts
  _U2RXIP = 1;  //Low Priority
  U2MODEbits.UARTEN = 1;	// And turn the peripheral on
  U2STA = 0b0000010000000000;   // The U1STA register must be set AFTER the module is enabled for some reason
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

 void InitINT4()
 {
  // Set up Interrupts
  // Set up external INT4 */
  // This is the trigger interrupt
  _INT4IF = 0;		// Clear Interrupt flag
  _INT4IE = 1;		// Enable INT3 Interrupt
  _INT4EP = 0; 	        // Interrupt on rising edge
  _INT4IP = 7;		// Set interrupt to highest priority
 }

void InitTimer4(void)
{
    T4CONbits.TCKPS = 0b10;     //64 prescale
    T4CONbits.TCS = 0;          //Disable Tsync from external clock output
    PR4 = (FCY / 64 / 10);      //This produces a 100ms interrupt timer (15625)
    TMR4 = 0;
    _T4IF = 0;
    _T4IE = 1;
    _T4IP = 5;
    T4CONbits.TON = 1;
}

void InitTimer5(void)
{
    T5CONbits.TCKPS = 0b01;     //8 prescale
    T5CONbits.TCS = 0;          //Disable Tsync from external clock output
    PR5 = 3000;                 //This produces a 2.4ms interrupt timer
    TMR5 = 0;
    _T5IF = 0;
    _T5IE = 1;
    _T5IP = 6;
    T5CONbits.TON = 1;
}