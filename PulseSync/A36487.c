#include "A36487.h"
#include "ETM_RC_FILTER.h"


//Global Variables
CommandStringStruct command_string;
BUFFERBYTE64 uart2_input_buffer;
BUFFERBYTE64 uart2_output_buffer;
PULSE_PARAMETERS psb_params;
PSB_DATA psb_data;

//Limited scope variables
unsigned char change_pulse_width_counter;
//unsigned char state;
const unsigned int  dose_intensities[4] = {15, 95, 175, 255};  // fixed constants

void ReadTrigPulseWidth(void);
unsigned char FilterTrigger(unsigned char param);
void ReadAndSetEnergy(void);
void ProgramShiftRegisters(void);
unsigned int GetInterpolationValue(unsigned int low_point, unsigned int high_point, unsigned low_value, unsigned high_value, unsigned point);
void PulseSyncStateMachine(void);
void DoA36487(void);

//Processor Setup
_FOSC(EC & CSW_FSCM_OFF); // Primary Oscillator without PLL and Startup with User Selected Oscillator Source, CLKOUT 10MHz is used to measure trigger width.
//_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_2);  // Watchdog Timer is enabled, 1024ms TIMEOUT
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_64 & BORV_27 & PBOR_ON & MCLR_EN); // Brown out and Power on Timer settings
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


int main(void) {
    psb_data.state_machine = STATE_INIT;
    while (1) {
      PulseSyncStateMachine();
    }
}



void PulseSyncStateMachine(void) {
  
  switch (psb_data.state_machine) {

  case STATE_INIT:
    psb_data.personality = 0;
    Initialize();
    ETMCanInitialize();    
    psb_data.personality = ReadDosePersonality();
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    psb_data.state_machine = STATE_WAIT_FOR_CONFIG;
    break;

    // DPARKER - Need to create a "FLASH LED" state so that we can communicate over can bus while flashing LEDs at startup

  case STATE_WAIT_FOR_CONFIG:
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    while (psb_data.state_machine == STATE_WAIT_FOR_CONFIG) {
      DoA36487();
      ETMCanDoCan();
      if (psb_data.counter_config_received == 0b1111) {
	psb_data.state_machine = STATE_HV_OFF;
      }
    }
    break;
      

  case STATE_HV_OFF:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    while (psb_data.state_machine == STATE_HV_OFF) {
      DoA36487();
      ETMCanDoCan();
      
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV == 0) {
	psb_data.state_machine = STATE_HV_ENABLE;
      }
      
      if (_FAULT_REGISTER) {
	psb_data.state_machine = STATE_FAULT;
      }
    }
    break;


  case STATE_HV_ENABLE:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    while (psb_data.state_machine == STATE_HV_ENABLE) {
      DoA36487();
      ETMCanDoCan();
      
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY == 0) {
	psb_data.state_machine = STATE_X_RAY_ENABLE;
      }
      
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	psb_data.state_machine = STATE_HV_OFF;
      }

      if (_FAULT_REGISTER) {
	psb_data.state_machine = STATE_FAULT;
      }
    }
    break;


  case STATE_X_RAY_ENABLE:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = OLL_CPU_XRAY_ENABLE;
    while (psb_data.state_machine == STATE_X_RAY_ENABLE) {
      DoA36487();
      ETMCanDoCan();
      
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY || _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	psb_data.state_machine = STATE_HV_ENABLE;
      }
      
      if (_FAULT_REGISTER) {
	psb_data.state_machine = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    while (psb_data.state_machine == STATE_FAULT) {
      DoA36487();
      ETMCanDoCan();
      
      if (_FAULT_REGISTER == 0) {
	psb_data.state_machine = STATE_WAIT_FOR_CONFIG;
      }
    }
    break;

      
  default:
    _CONTROL_NOT_CONFIGURED = 1;
    _CONTROL_NOT_READY = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    psb_data.state_machine = STATE_UNKNOWN;
    while (1) {
      DoA36487();
      ETMCanDoCan();
    }
    break;
    
  } // switch (psb_data.state_machine) {

}


void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _INT3Interrupt(void) {

  _INT3IF = 0;		// Clear Interrupt flag
  
  // A trigger was recieved.
  // THIS DOES NOT MEAN THAT A PULSE WAS GENERATED
  // If (PIN_CPU_XRAY_ENABLE_OUT == OLL_CPU_XRAY_ENABLE)  && (PIN_CUSTOMER_XRAY_ON_IN == ILL_CUSTOMER_BEAM_ENABLE) then we "Probably" generated a pulse

  
  // Calculate the Trigger PRF
  // TMR1 is used to time the time between INT3 interrupts
  psb_data.last_period = TMR1;
  if (_T1IF) {
    // The timer exceed it's period of 400mS - (Will happen if the PRF is less than 2.5Hz)
    psb_data.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
  }
  TMR1 = 0;
  _T1IF = 0;


  // This is used to detect if the trigger is high (which would cause constant pulses to the system)
  if (PIN_TRIG_INPUT != ILL_TRIG_ON) {
    ReadTrigPulseWidth();
    ReadAndSetEnergy();
  } else {  // if pulse trig stays on, set to minimum dose and flag fault
    _FAULT_TRIGGER_STAYED_ON = 1;
    psb_data.trigger_filtered = 0;
    
  }
  
  ProgramShiftRegisters();
  psb_data.period_filtered = RCFilterNTau(psb_data.period_filtered, psb_data.last_period, RC_FILTER_16_TAU);

  if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
    ETMCanLogCustomPacketC();
  }
  
  psb_data.pulses_on++;       // This counts the pulses
  ETMCanPulseSyncSendNextPulseLevel(psb_data.energy, (psb_data.pulses_on));
}

void ReadTrigPulseWidth(void)
{
      unsigned int data;
      unsigned char i;

      PIN_SPI_CLK_OUT  = 0;
      Nop();
      PIN_PW_SHIFT_OUT = !OLL_PW_SHIFT; // load the reg
      Nop();
      __delay32(1); // 100ns for 10M TCY
      PIN_PW_SHIFT_OUT = OLL_PW_SHIFT;  // enable shift
      Nop();
      __delay32(1); // 100ns for 10M TCY

      data = PIN_SPI_DATA_IN;

      for (i = 0; i < 8; i++)
      {
      	PIN_SPI_CLK_OUT = 1;
        Nop();
        data <<= 1;
        data |= PIN_SPI_DATA_IN;
      	PIN_SPI_CLK_OUT = 0;
        Nop();
        __delay32(1); // 100ns for 10M TCY
      }

      PIN_PW_SHIFT_OUT = !OLL_PW_SHIFT; // make load active when idle
      Nop();

      if (data & 0x0100)  // counter overflow
      {
          psb_data.trigger_input = 0xFF;
      }
      else
      {
          psb_data.trigger_input = data & 0xFF;
      }
      psb_data.trigger_filtered = FilterTrigger(psb_data.trigger_input);

      if (psb_data.trigger_filtered < 245)   //signify to pfn control board what mode to expect
          PIN_MODE_OUT = OLL_MODE_PORTAL;   //so it can use a different target
      else                                  //current setpoint for low energy
          PIN_MODE_OUT = OLL_MODE_GANTRY;
}

unsigned char FilterTrigger(unsigned char param)
{
    int x;

    //Establish Dose Levels to reduce jitter and provide consistent dose vs trigger width
    //Every bit represents 20ns pulse width change on the electron gun
    //Every bit also represents a 200ns pulse width change from the customer
    if (param > (DOSE_LEVELS - 1))
    {
        for (x = 0; x <= (param % DOSE_LEVELS); x++)
            param--;
    }
    else
        param = 0;

    //Ensure that at least 15 of the same width pulses in a row only change the sampled width
    if (param != psb_data.last_trigger_filtered)
    {
        change_pulse_width_counter++;
        if (change_pulse_width_counter < 15)
            param = psb_data.last_trigger_filtered;
        else
            psb_data.last_trigger_filtered = param;
    }
    else
        change_pulse_width_counter = 0;

    return param;
}

void ReadAndSetEnergy()
{
    if ((PIN_LOW_MODE_IN == HI) && (PIN_HIGH_MODE_IN == HI))
    {
        if (PIN_ENERGY_CMD_IN1 == HI)
        {
            Nop();
            PIN_AFC_TRIGGER_OK_OUT = OLL_AFC_TRIGGER_OK;    //Trigger the AFC in high energy only
            Nop();
            PIN_GUN_POLARITY_OUT = !OLL_GUN_POLARITY;
            Nop();
            PIN_ENERGY_CPU_OUT = !OLL_ENERGY_CPU;
            Nop();
            psb_data.energy = HI;
        }
        else
        {
            Nop();
            PIN_AFC_TRIGGER_OK_OUT = !OLL_AFC_TRIGGER_OK;   //Do not trigger the AFC in low energy
            Nop();
            PIN_GUN_POLARITY_OUT = !OLL_GUN_POLARITY;
            Nop();
            PIN_ENERGY_CPU_OUT = OLL_ENERGY_CPU;
            Nop();
            psb_data.energy = LOW;
        }
    }
    else
    {
        if (PIN_HIGH_MODE_IN == HI)
        {
            Nop();
            PIN_AFC_TRIGGER_OK_OUT = OLL_AFC_TRIGGER_OK;    //Trigger the AFC in single energy mode
            Nop();
            PIN_GUN_POLARITY_OUT = OLL_GUN_POLARITY;
            Nop();
            PIN_ENERGY_CPU_OUT = OLL_ENERGY_CPU;
            Nop();
            psb_data.energy = LOW;
        }
        else
        {
            Nop();
            PIN_AFC_TRIGGER_OK_OUT = OLL_AFC_TRIGGER_OK;    //Trigger the AFC in single energy mode
            Nop();
            PIN_GUN_POLARITY_OUT = OLL_GUN_POLARITY;
            Nop();
            PIN_ENERGY_CPU_OUT = !OLL_ENERGY_CPU;
            Nop();
            psb_data.energy = HI;
        }
    }
}

void ProgramShiftRegisters(void)
{
    unsigned int p;
    unsigned int q;
    unsigned long temp;
    unsigned long bittemp;

    PIN_PW_CLR_CNT_OUT = OLL_PW_CLR_CNT;			 // clear width count
    Nop();
    PIN_PW_HOLD_LOWRESET_OUT = !OLL_PW_HOLD_LOWRESET;	 // reset start to disable pulse
    Nop();

    // do inteplation for grid delay and grid width
    for (p = 0; p < 4; p++)
    {
    	if (psb_data.trigger_filtered <= dose_intensities[p]) break;
    }
    
    if (p == 0)
    {
        if (psb_data.energy == HI) {
            psb_data.grid_delay = psb_params.grid_delay_high0;
            psb_data.grid_width = psb_params.grid_width_high0;
        }
        else {
            psb_data.grid_delay = psb_params.grid_delay_low0;
            psb_data.grid_width = psb_params.grid_width_low0;
        }
    }
    else if (p >= 4)
    {
        if (psb_data.energy == HI) {
            psb_data.grid_delay = psb_params.grid_delay_high3;
            psb_data.grid_width = psb_params.grid_width_high3;
        }
        else {
            psb_data.grid_delay = psb_params.grid_delay_low3;
            psb_data.grid_width = psb_params.grid_width_low3;
        }
    }
    else // interpolation
    {
        if (p == 1) {
            if (psb_data.energy == HI) {
                psb_data.grid_delay = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_delay_high0, psb_params.grid_delay_high1, psb_data.trigger_filtered);
                psb_data.grid_width = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_width_high0, psb_params.grid_width_high1, psb_data.trigger_filtered);
            }
            else {
                psb_data.grid_delay = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_delay_low0, psb_params.grid_delay_low1, psb_data.trigger_filtered);
                psb_data.grid_width = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_width_low0, psb_params.grid_width_low1, psb_data.trigger_filtered);
            }
        }
        else if (p == 2) {
            if (psb_data.energy == HI) {
                psb_data.grid_delay = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_delay_high1, psb_params.grid_delay_high2, psb_data.trigger_filtered);
                psb_data.grid_width = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_width_high1, psb_params.grid_width_high2, psb_data.trigger_filtered);
            }
            else {
                psb_data.grid_delay = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_delay_low1, psb_params.grid_delay_low2, psb_data.trigger_filtered);
                psb_data.grid_width = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_width_low1, psb_params.grid_width_low2, psb_data.trigger_filtered);
            }
        }
        else if (p == 3) {
            if (psb_data.energy == HI) {
                psb_data.grid_delay = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_delay_high2, psb_params.grid_delay_high3, psb_data.trigger_filtered);
                psb_data.grid_width = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_width_high2, psb_params.grid_width_high3, psb_data.trigger_filtered);
            }
            else {
                psb_data.grid_delay = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_delay_low2, psb_params.grid_delay_low3, psb_data.trigger_filtered);
                psb_data.grid_width = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], psb_params.grid_width_low2, psb_params.grid_width_low3, psb_data.trigger_filtered);
            }
        }
    }

    for (p = 0; p < 6; p++)
    {
    	if (p == 0)
            temp = psb_data.grid_width;     //Grid Width
        else if (p == 1)
            temp = psb_data.grid_delay;     //Grid Delay
        else if (p == 2) {
            psb_data.rf_delay = psb_data.grid_delay - 9;
            temp = psb_data.rf_delay;       //RF PCB Delay
        }
        else if (p == 3)
            temp = psb_params.pfn_delay_high;   //PFN Delay
        else if (p == 4)
            temp = 0;                           //Dosimeter delay (not used)
        else if (p == 5)
            temp = psb_params.afc_delay_high;   //AFC Delay
        else
            temp = 0;

        for (q = 0; q < 8; q++)
        {
            PIN_SPI_CLK_OUT = 0;
            Nop();

            bittemp = temp & 0x80;
            temp = temp << 1;

            if (bittemp == 0x80)
            {
                PIN_SPI_DATA_OUT = 1;
                Nop();
            }
            else
            {
                PIN_SPI_DATA_OUT = 0;
                Nop();
            }

            PIN_SPI_CLK_OUT = 1;
            Nop();
        }

        if (p == 1)						//Latch Gun delay and width data into shift registers
        {
            PIN_LD_DELAY_GUN_OUT = 0;
            Nop();
            PIN_LD_DELAY_GUN_OUT = 1;
            Nop();
        }
        else if (p == 3)				//Latch PFN/RF delay data into shift registers
        {
            PIN_LD_DELAY_PFN_OUT = 0;
            Nop();
            PIN_LD_DELAY_PFN_OUT = 1;
            Nop();
        }
        else if (p == 5)				//Latch AFC/Dose delay data into shift registers
        {
            PIN_LD_DELAY_AFC_OUT = 0;
            Nop();
            PIN_LD_DELAY_AFC_OUT = 1;
            Nop();
        }
    }

    PIN_PW_CLR_CNT_OUT = !OLL_PW_CLR_CNT;			 // enable width count
    Nop();
    if (PIN_TRIG_INPUT != ILL_TRIG_ON)
    {
    	PIN_PW_HOLD_LOWRESET_OUT = OLL_PW_HOLD_LOWRESET;   // clear reset only when trig pulse is low
        Nop();
    	_FAULT_TRIGGER_STAYED_ON = 0;
    }
    else
    	_FAULT_TRIGGER_STAYED_ON = 1;
}

// calculate the interpolation value
unsigned int GetInterpolationValue(unsigned int low_point, unsigned int high_point, unsigned low_value, unsigned high_value, unsigned point)
{
   double dtemp, dslope;
   unsigned int ret = low_value;

   if (high_point > low_point)  // high point has to be bigger
   {
   	dslope = ((double)high_value - (double)low_value) / ((double)high_point - (double)low_point);
        dtemp = (double)point - (double)low_point;
        dtemp *= dslope;
        dtemp += low_value;
        ret = (unsigned)dtemp;
   }
   return (ret);
}


void DoA36487(void) {
  

  local_debug_data.debug_0 = psb_data.grid_delay;
  local_debug_data.debug_1 = psb_data.grid_width;
  local_debug_data.debug_2 = psb_data.rf_delay;
  local_debug_data.debug_3 = psb_data.energy;

  local_debug_data.debug_4 = psb_data.trigger_input;
  local_debug_data.debug_5 = psb_data.trigger_filtered;
  local_debug_data.debug_6 = psb_data.last_trigger_filtered;
  local_debug_data.debug_7 = psb_data.personality;

  local_debug_data.debug_8 = psb_data.state_machine;
  local_debug_data.debug_9 = psb_data.pulses_on;
  local_debug_data.debug_A = psb_data.last_period;
  local_debug_data.debug_A = psb_data.period_filtered;



  // ---------- UPDATE LOCAL FAULTS ------------------- //

  if ((psb_data.state_machine == STATE_FAULT) && (_SYNC_CONTROL_RESET_ENABLE)) {
    _FAULT_REGISTER = 0;
  }
  
  if (PIN_PANEL_IN == 0) {
    _FAULT_PANEL = 1;
  }
  
  if (PIN_KEY_LOCK_IN == 0) {
    _FAULT_KEYLOCK = 1;
  }
  
  if (PIN_XRAY_CMD_MISMATCH_IN == !ILL_XRAY_CMD_MISMATCH) {
    _FAULT_TIMING_MISMATCH = 1;
  }

  // _FAULT_TRIGGER_STAYED_ON is set by INT3 Interrupt

  if ((PIN_CUSTOMER_XRAY_ON_IN) && (!PIN_CUSTOMER_BEAM_ENABLE_IN)) {
    _FAULT_X_RAY_ON_WIHTOUT_HV = 1;
  }
  
  // ---------- END UPDATE LOCAL FAULTS -------------- //




  // ------------- UPDATE STATUS -------------------- //

  _STATUS_CUSTOMER_HV_DISABLE = !PIN_CUSTOMER_BEAM_ENABLE_IN;

  _STATUS_CUSTOMER_X_RAY_DISABLE = !PIN_CUSTOMER_XRAY_ON_IN;

  _STATUS_LOW_MODE_OVERRIDE = PIN_LOW_MODE_IN;
  
  _STATUS_HIGH_MODE_OVERRIDE = PIN_HIGH_MODE_IN;
  
  etm_can_status_register.data_word_A = psb_data.period_filtered;
  etm_can_status_register.data_word_B = psb_data.personality;
  
  // ------------- END UPDATE STATUS --------------------- //



  if (_T4IF) {
    // Run these once every 10ms
    _T4IF = 0;

    // -------------- UPDATE LED OUTPUTS ---------------- //
    if (LED_WARMUP_STATUS) {
      //PIN_LED_WARMUP = OLL_LED_ON;
      PIN_CPU_WARMUP_OUT = OLL_CPU_WARMUP;
    } else {
      //PIN_LED_WARMUP = !OLL_LED_ON;
      PIN_CPU_WARMUP_OUT = !OLL_CPU_WARMUP;
    }
    
    if (LED_STANDBY_STATUS) {
      PIN_LED_STANDBY = OLL_LED_ON;
      PIN_CPU_STANDBY_OUT = OLL_CPU_STANDBY;
    } else {
      PIN_LED_STANDBY = !OLL_LED_ON;
      PIN_CPU_STANDBY_OUT = !OLL_CPU_STANDBY;
    }
    
    if (LED_READY_STATUS) {
      PIN_LED_READY = OLL_LED_ON;
      PIN_CPU_READY_OUT = OLL_CPU_READY;
    } else {
      PIN_LED_READY = !OLL_LED_ON;
      PIN_CPU_READY_OUT = !OLL_CPU_READY;
    }
  
    if (LED_SUM_FAULT_STATUS) {
      PIN_LED_SUMFLT = OLL_LED_ON;
    PIN_CPU_SUMFLT_OUT = OLL_CPU_SUMFLT;
    } else {
      PIN_LED_SUMFLT = !OLL_LED_ON;
      PIN_CPU_SUMFLT_OUT = !OLL_CPU_SUMFLT;
    }
    
    // -------------- END UPDATE LED OUTPUTS ---------------- //
    


    // This is not needed as the CAN module will generate 
    psb_data.can_counter_ms += 10;
    if (_CONTROL_CAN_SYNC_REC) {
      psb_data.can_counter_ms = 0;
      _CONTROL_CAN_SYNC_REC = 0;
    }
    if (psb_data.can_counter_ms >= 150) {
      _FAULT_SYNC_TIMEOUT = 1;
      PIN_LED_SUMFLT = OLL_LED_ON;
    }
    
    //Heartbeat the standby LED
    psb_data.heartbeat++;
    if (psb_data.heartbeat >= 50) {
      psb_data.heartbeat = 0;
      if (PIN_LED_STANDBY == OLL_LED_ON) {
	PIN_LED_STANDBY = !OLL_LED_ON;
      }
      else {
	PIN_LED_STANDBY = OLL_LED_ON;
      }
    }
  }
}

void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();

}
