#include "A36717.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"
#include <spi.h>

// This is the firmware for modulator- HV section

/*

*/

/* -------------------- Device configuration --------------------------  */

_FOSC(EC & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);
//------------------------------------------------------------------------//


void DoStateMachine(void);
void DoA36717(void);
void InitializeA36717(void);
void DoControlLoop(TYPE_UC2827_CONTROL* ptr);
void CheckAnalogFaults(void);
void A36717TransmitData(void); 
void A36717ReceiveData(void); 
void A36717DownloadData(unsigned char *msg_data);
void UpdateBias (void);

unsigned int heater_set_point;
unsigned int top_1_set_point;
unsigned int top_2_set_point;

unsigned int dac_return_value;
unsigned int return_data;
unsigned int last_PRF_sample;

ControlData global_data_A36717;
LTC265X U10_LTC2654;
BUFFERBYTE64 uart1_input_buffer;
BUFFERBYTE64 uart1_output_buffer;

AnalogInput top_raw_vmon;  
AnalogInput bias_vmon;

AnalogInput top_1_vmon;      
AnalogInput top_2_vmon;
AnalogInput heater_vmon;
AnalogInput heater_1_imon;
AnalogInput heater_2_imon;

AnalogOutput top_1_set;
AnalogOutput top_2_set;




#define drive_per_PRF_values 26500,24327,23157,22369,21781,21315,20930,20604,20322,20074,19853,19654,19473,19308,19155,19014,18883,18760,18645,18536,18434,18337,18245,18158,18074,17995,17918,17845,17775,17708,17643,17581,17521,17462,17406,17352,17299,17248,17198,17150,17103,17058,17014,16971,16929,16888,16848,16809,16771,16734,16697,16662,16627,16593,16560,16528,16496,16464,16434,16404,16374,16345,16317,16289,16262,16235,16208,16182,16157,16132,16107,16083,16059,16035,16012,15989,15966,15944,15922,15901,15880,15859,15838,15818,15798,15778,15758,15739,15720,15701,15683,15664,15646,15628,15611,15593,15576,15559,15542,15526,15509,15493,15477,15461,15445,15430,15414,15399,15384,15369,15354,15340,15325,15311,15297,15283,15269,15255,15242,15228,15215,15202,15188,15176,15163,15150
#define top_drive_per_PRF_values 21480,21012,19100,17000,16100,14486,13839,13250,12800,12600,12450,12200,12000,11900,11870,11823,11787,11660,11543,11434,11332,11236,11146,11061,10981,10906,10834,10766,10702,10640,10582,10526,10473,10422,10373,10326,10281,10238,10197,10158,10119,10083,10048,10014,9981,9950,9919,9890,9862,9834,9808,9783,9758,9734,9711,9689,9668,9647,9627,9608,9589,9571,9553,9536,9519,9503,9488,9473,9458,9444,9431,9418,9405,9393,9381,9369,9358,9347,9337,9327,9317,9307,9298,9289,9281,9272,9264,9257,9249,9242,9235,9229,9222,9216,9210,9204,9199,9193,9188,9183,9179,9174,9170,9166,9162,9158,9155,9151,9148,9145,9142,9140,9137,9135,9133,9130,9129,9127,9125,9124,9122,9121,9120,9119,9118,9117

const unsigned int drive_per_PRF[126] = {drive_per_PRF_values};
const unsigned int top_drive_per_PRF[126] = {top_drive_per_PRF_values};


unsigned int do_control;

TYPE_UC2827_CONTROL bias_supply;
TYPE_UC2827_CONTROL top_supply;
unsigned int detected_PRF_log[10];
unsigned int datalog_counter;

int main(void) {

  global_data_A36717.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}

void DoStateMachine(void) {
  switch (global_data_A36717.control_state) {

  case STATE_STARTUP:
    InitializeA36717();
    global_data_A36717.control_state = STATE_OPERATE;

    break;
	



  case STATE_OPERATE:
    PIN_BIAS_ENABLE = ENABLE_SUPPLY;
    PIN_TOP_ENABLE  = ENABLE_SUPPLY;
    top_supply.dac_setting = 0x2000;
    bias_supply.dac_setting = 0x3000;
    WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, bias_supply.dac_setting);
    WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, top_supply.dac_setting);
    while(global_data_A36717.control_state == STATE_OPERATE) {
      DoA36717();
    }
    
    break;
    
    
 case STATE_COLD_FAULT:

	break;
	
  default:
    global_data_A36717.control_state = STATE_COLD_FAULT;

    break;
  }
}


void DoA36717(void) {
  A36717ReceiveData();
  bias_supply.reading = bias_vmon.reading_scaled_and_calibrated;
  top_supply.reading = top_raw_vmon.reading_scaled_and_calibrated;

  ETMCanSlaveDoCan();

  if ((PIN_PIC_COLD_FLT == 1) || (PIN_PIC_HOT_FLT == 1))
  {
      //PIN_PIC_PULSE_ENABLE_NOT = 1; // ***This should be uncommented later ***
  }
  else
  {
      PIN_PIC_PULSE_ENABLE_NOT = 0;
  }
  

  if (do_control) {
    // A new set of data has been received from the high side
    // Run an iteration of the control loop
    do_control = 0;
    DoControlLoop(&bias_supply);
    DoControlLoop(&top_supply);
    dac_return_value= WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, top_supply.dac_setting);
    dac_return_value= WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, bias_supply.dac_setting);
    

    CheckAnalogFaults();

  }
  
  if (_T2IF == 1)
  {
    _T2IF = 0;
    global_data_A36717.input_capture_sample = 0;
    if (global_data_A36717.detected_PRF != (PR2_VALUE_1_6_MS >> 4))
    {
        global_data_A36717.detected_PRF = PR2_VALUE_1_6_MS >> 4;
        datalog_counter %= 10;
        detected_PRF_log[++datalog_counter]= PR2_VALUE_1_6_MS >> 4;
        UpdateBias();
    }
  }

   
    dac_return_value= WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, bias_supply.dac_setting);


  if (_T3IF == 1 ) {
    // This happens once every 100uS
    _T3IF = 0;
    
    if (PIN_PIC_KICK == 1) 
    { 
      //kick pic external watchdog every 200us ( times out if 5 consecutive kicks are missed) 
      PIN_PIC_KICK = 0;
    } 
    else 
    {
      PIN_PIC_KICK = 1;
    }

    // --------------------- CHECK FOR CAN COMM LOSS -------------------- //
    if (ETMCanSlaveGetComFaultStatus()) 
    {
      _FAULT_CAN_COMM_LOSS = 1;
      PIN_PIC_COLD_FLT = 1;
    }
    else 
    {
      if (ETMCanSlaveGetSyncMsgResetEnable()) 
      {
	       _FAULT_CAN_COMM_LOSS = 0;
               PIN_PIC_COLD_FLT = 0;
      }
    }
    
    // -------------------- CHECK FOR HIGH SIDE CAN COMM LOSS --------- //
    #define HIGH_SIDE_TIMEOUT  10// 1msec
    
    if (global_data_A36717.counter_100us_high_side_loss > HIGH_SIDE_TIMEOUT) 
    {
      _FAULT_HIGH_SIDE_COMM_LOSS = 1;
      //PIN_PIC_COLD_FLT = 1; // ***This should be uncommented later ***

    } 
    else 
    {
      if (ETMCanSlaveGetSyncMsgResetEnable())
      {
	       _FAULT_HIGH_SIDE_COMM_LOSS = 0;
               PIN_PIC_COLD_FLT = 0;
      }
    }
    global_data_A36717.counter_100us_high_side_loss++;
    
    global_data_A36717.counter_100us++;

    if (global_data_A36717.counter_100us >= 10) {
      // This is true every 1ms
      global_data_A36717.counter_100us = 0;
      if (ETMCanSlaveGetSyncMsgResetEnable()) //update the status clear bit.
      {
        global_data_A36717.status |= 0x80;
      }
      else
      {
        global_data_A36717.status &= 0x7F;  
      }

      A36717TransmitData();
      
      slave_board_data.log_data[0] = bias_supply.target;
      slave_board_data.log_data[1] =  top_raw_vmon.reading_scaled_and_calibrated;
      slave_board_data.log_data[2] =  top_1_vmon.reading_scaled_and_calibrated;
      slave_board_data.log_data[3] = top_1_set.set_point;
      
      slave_board_data.log_data[4] =  bias_vmon.reading_scaled_and_calibrated;
      slave_board_data.log_data[5] =  top_raw_vmon.reading_scaled_and_calibrated;
      slave_board_data.log_data[6] =  top_2_vmon.reading_scaled_and_calibrated;
      slave_board_data.log_data[7] = top_2_set.set_point;
      
      slave_board_data.log_data[8] =  heater_1_imon.reading_scaled_and_calibrated;
      slave_board_data.log_data[9] =  heater_2_imon.reading_scaled_and_calibrated;
      slave_board_data.log_data[10] =  heater_vmon.reading_scaled_and_calibrated;
      slave_board_data.log_data[11] = heater_set_point;
      
      ETMCanSlaveSetDebugRegister(0x0, bias_supply.dac_setting);
      ETMCanSlaveSetDebugRegister(0x1,top_supply.dac_setting );
      ETMCanSlaveSetDebugRegister(0x2, global_data_A36717.detected_PRF);
      ETMCanSlaveSetDebugRegister(0x3, detected_PRF_log[2]);
      ETMCanSlaveSetDebugRegister(0x4, detected_PRF_log[3]);
      ETMCanSlaveSetDebugRegister(0x5, detected_PRF_log[4]);
      ETMCanSlaveSetDebugRegister(0x6, detected_PRF_log[5]);
      ETMCanSlaveSetDebugRegister(0x7, detected_PRF_log[6]);
      ETMCanSlaveSetDebugRegister(0x8, detected_PRF_log[7]);
      ETMCanSlaveSetDebugRegister(0x9, detected_PRF_log[8]);
      ETMCanSlaveSetDebugRegister(0xA, detected_PRF_log[9]);
      ETMCanSlaveSetDebugRegister(0xB, detected_PRF_log[10]);
      ETMCanSlaveSetDebugRegister(0xC, 0xFF);
      ETMCanSlaveSetDebugRegister(0xD, return_data);
      ETMCanSlaveSetDebugRegister(0xE, LTC265X_single_channel_error_count);
      ETMCanSlaveSetDebugRegister(0xF, top_supply.dac_setting);
    }
    
    global_data_A36717.led_counter++;
    global_data_A36717.led_counter &= 0x7FFF;
    
    if ((global_data_A36717.led_counter & 0x03FF) == 0) // this will be true every ~100mS
    {      
      if(PIN_LED_OPERATIONAL_GREEN == 1) 
      {
	       PIN_LED_OPERATIONAL_GREEN = 0;
      } 
      else 
      {
	       PIN_LED_OPERATIONAL_GREEN = 1;
      } 
    }
  }
}


void InitializeA36717(void) {
  //Timer setup
  T3CON = T3CON_VALUE;
  PR3 = PR3_VALUE_100_US;
  _T3IF = 0;
  
  PIN_PIC_KICK = 0;
  PIN_BIAS_FLT = 0;
  PIN_TOP_FLT = 0;
  PIN_BIAS_ENABLE = 0;
  PIN_TOP_ENABLE = 0;
  PIN_PIC_COLD_FLT = 0;
  PIN_PIC_HOT_FLT = 0;
  PIN_LED_OPERATIONAL_GREEN = 0;
  PIN_PIC_HTR_FLT = 0;
  PIN_LED_TEST_POINT_A;
  PIN_PIC_PULSE_ENABLE_NOT = 0;
  PIN_LED_TEST_POINT_A = 0;
  

#define PS_MAX_DAC_OUTPUT       0x8000
#define PS_MIN_DAC_OUTPUT       0x1A00
#define BIAS_MIN_DAC_OUTPUT     10000
#define DAC_FAST_STEP           0x0004
#define DAC_SLOW_STEP           0x0001

#define BIAS_TARGET             40000   // 400V
#define BIAS_WINDOW              5000   // 50V

#define TOP_TARGET              3000   // 30V
#define TOP_WINDOW              1500   // 15V

  // Set up the control loops
  bias_supply.max_dac_setting = PS_MAX_DAC_OUTPUT;
  bias_supply.min_dac_setting = BIAS_MIN_DAC_OUTPUT;
  bias_supply.dac_setting = 0x3000;
  bias_supply.target = BIAS_TARGET;
  bias_supply.min_window = BIAS_TARGET - BIAS_WINDOW;
  bias_supply.max_window = BIAS_TARGET + BIAS_WINDOW;
  bias_supply.fast_step_more_power = DAC_FAST_STEP;
  bias_supply.slow_step_more_power = DAC_SLOW_STEP;
  bias_supply.fast_step_less_power = DAC_FAST_STEP;
  bias_supply.slow_step_less_power = DAC_SLOW_STEP;

  

  // Set up the control loops
  top_supply.max_dac_setting = PS_MAX_DAC_OUTPUT;
  top_supply.min_dac_setting = PS_MIN_DAC_OUTPUT;
  top_supply.dac_setting = top_supply.min_dac_setting;
  top_supply.target = TOP_TARGET;
  top_supply.min_window = TOP_TARGET - TOP_WINDOW;
  top_supply.max_window = TOP_TARGET + TOP_WINDOW;
  top_supply.fast_step_more_power = DAC_FAST_STEP;
  top_supply.slow_step_more_power = DAC_SLOW_STEP;
  top_supply.fast_step_less_power = DAC_FAST_STEP;
  top_supply.slow_step_less_power = DAC_SLOW_STEP;


  // Set up PRF detection (input capture)
  IC4CON= IC4CON_SETTING;
  T2CON = T2CON_VALUE;
   _T2IF = 0;
  PR2 = PR2_VALUE_1_6_MS;
  _T2IF = 0;
  _IC4IF = 0;
  _IC4IE = 1;
  _IC4IP = 5;

  global_data_A36717.input_capture_sample = 0;
  global_data_A36717.detected_PRF = PR2_VALUE_1_6_MS >> 4;
  global_data_A36717.last_detected_PRF = PR2_VALUE_1_6_MS >> 4;

  global_data_A36717.status = 0x10;


  //set tris
  TRISA = A36717_TRISA_VALUE;
  TRISB = A36717_TRISB_VALUE;
  TRISC = A36717_TRISC_VALUE;
  TRISD = A36717_TRISD_VALUE;
  TRISF = A36717_TRISF_VALUE;
  TRISG = A36717_TRISG_VALUE;
  
  SetupLTC265X(&U10_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
  

  // Set up the UART
  _U1RXIP = 6;
  _U1TXIP = 6;
  
  _U1RXIF = 0;
  _U1TXIF = 0;
  _U1RXIE = 1;
  _U1TXIE = 1;
    
  U1BRG  = A36717_U1BRG_VALUE;  //A36717_SERIAL_UART_BRG_VALUE;
  U1MODE = A36717_U1MODE_VALUE; //A36717_SERIAL_UART_MODE_VALUE;
  U1STA  = A36717_U1STA_VALUE;  //A36717_SERIAL_UART_STA_VALUE;

#define AGILE_REV 77
#define SERIAL_NUMBER 100
  
  
  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RB6, 4, _PIN_NOT_CONNECTED, _PIN_NOT_CONNECTED);
  ETMCanSlaveLoadConfiguration(36717, 0, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);

#define TOP_RAW_VMON_SCALE_FACTOR        0.08247
#define TOP_RAW_VMON_OFFSET              -3792
#define HEATER_VMON_SCALE_FACTOR         1
#define HEATER_IMON_SCALE_FACTOR         1

#define TOP_VMON_SCALE_FACTOR            0.50354 //based on 1V per 100V sensed * ADC conversion (1024/3.3) * accumulation of 64 samples.
#define TOP_OVER_TRIP_POINT_ABSOLUTE     25000 // 250 Volts
#define TOP_UNDER_TRIP_POINT_ABSOLUTE    10000 // 100 Volts
#define TOP_ABSOLUTE_TRIP_COUNTER        2


#define BIAS_VMON_SCALE_FACTOR           0.7455
#define BIAS_OVER_TRIP_POINT_ABSOLUTE    50000 // 500 Volts
#define BIAS_UNDER_TRIP_POINT_ABSOLUTE   35000 // 350 Volts
#define BIAS_ABSOLUTE_TRIP_COUNTER       2

#define TOP_MIN_SET_POINT                10000  //100V
#define TOP_MAX_SET_POINT                25000  //250V
#define TOP_SET_SCALE_FACTOR             1.6

  // Initialize the analog input module
  
  if (!ETMAnalogCheckEEPromInitialized()) {
    ETMAnalogLoadDefaultCalibration();
  }
  
  ETMAnalogInitializeInput(&top_raw_vmon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TOP_RAW_VMON_SCALE_FACTOR),
			   OFFSET_ZERO,
			   ANALOG_INPUT_0,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER
			   );

  ETMAnalogInitializeInput(&bias_vmon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(BIAS_VMON_SCALE_FACTOR),
			   OFFSET_ZERO,
			   ANALOG_INPUT_2,
			   BIAS_OVER_TRIP_POINT_ABSOLUTE,
			   BIAS_UNDER_TRIP_POINT_ABSOLUTE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   BIAS_ABSOLUTE_TRIP_COUNTER
			   );

  ETMAnalogInitializeInput(&top_1_vmon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TOP_VMON_SCALE_FACTOR),
			   OFFSET_ZERO,
			   ANALOG_INPUT_3,
			   TOP_OVER_TRIP_POINT_ABSOLUTE,
			   TOP_UNDER_TRIP_POINT_ABSOLUTE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   TOP_ABSOLUTE_TRIP_COUNTER
			   );

  ETMAnalogInitializeInput(&top_2_vmon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TOP_VMON_SCALE_FACTOR),
			   OFFSET_ZERO,
			   ANALOG_INPUT_4,
			   TOP_OVER_TRIP_POINT_ABSOLUTE,
			   TOP_UNDER_TRIP_POINT_ABSOLUTE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   TOP_ABSOLUTE_TRIP_COUNTER
			   );

  ETMAnalogInitializeInput(&heater_vmon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(HEATER_VMON_SCALE_FACTOR),
			   OFFSET_ZERO,
			   ANALOG_INPUT_5,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER
			   );

  ETMAnalogInitializeInput(&heater_1_imon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(HEATER_IMON_SCALE_FACTOR),
			   OFFSET_ZERO,
			   ANALOG_INPUT_6,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER
			   );

  ETMAnalogInitializeInput(&heater_2_imon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(HEATER_IMON_SCALE_FACTOR),
			   OFFSET_ZERO,
			   ANALOG_INPUT_7,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER
			   );

  ETMAnalogInitializeOutput(&top_1_set,
             MACRO_DEC_TO_SCALE_FACTOR_16(TOP_SET_SCALE_FACTOR),
             OFFSET_ZERO,
             ANALOG_OUTPUT_0,
             TOP_MAX_SET_POINT,
             TOP_MIN_SET_POINT,
             0);

  ETMAnalogInitializeOutput(&top_2_set,
             MACRO_DEC_TO_SCALE_FACTOR_16(TOP_SET_SCALE_FACTOR),
             OFFSET_ZERO,
             ANALOG_OUTPUT_1,
             TOP_MAX_SET_POINT,
             TOP_MIN_SET_POINT,
             0);

  top_1_set.enabled = 0xFF;
  top_2_set.enabled = 0xFF;

  datalog_counter= 1;
  last_PRF_sample=125;
  while (datalog_counter<=10)
  {
  detected_PRF_log[datalog_counter++]=0;
  }
  datalog_counter= 1;
}




void DoControlLoop(TYPE_UC2827_CONTROL* ptr) {
  // To decrease the output power of the UC2827, reduce to program voltage



  // First check for over voltage conditions.  If voltage is too high reduce the drive voltage
  if (ptr->reading > ptr->max_window)
  {
    // The voltage is very high reduce the drive voltage quickly
    
    if (ptr->fast_step_less_power < ptr->dac_setting) 
    {

      ptr->dac_setting -= ptr->fast_step_less_power;
    } 
    else 
    {
      ptr->dac_setting = 0;
    }
    if (ptr->dac_setting < ptr->min_dac_setting) 
    {
      ptr->dac_setting = ptr->min_dac_setting;
    }
    
  } 
  else if (ptr->reading > ptr->target) {
    
    // The voltage is greater than target - reduce drive voltage slowly
      if (ptr->slow_step_less_power < ptr->dac_setting) 
      {
        ptr->dac_setting -= ptr->slow_step_less_power;
      } 
      else 
      {
        ptr->dac_setting = 0;
      }
      if (ptr->dac_setting < ptr->min_dac_setting) {
        ptr->dac_setting = ptr->min_dac_setting;
      }
    
  
    
  }

  else if (ptr->reading < ptr->min_window) 
  {
    // The voltage is very low - increase the drive voltage quickly
    
    if ((0xFFFF - ptr->fast_step_more_power) > ptr->dac_setting) 
    {
      ptr->dac_setting += ptr->fast_step_more_power;
    } 
    else 
    {
      ptr->dac_setting = 0xFFFF;
    }
    if (ptr->dac_setting > ptr->max_dac_setting) {
      ptr->dac_setting = ptr->max_dac_setting;
    }
  } 
  
  else if (ptr->reading < ptr->target) {
    // The voltage is less than target - increase the drive voltage slowly
    
    if ((0xFFFF - ptr->slow_step_more_power) > ptr->dac_setting) {
      ptr->dac_setting += ptr->slow_step_more_power;
    } else {
      ptr->dac_setting = 0xFFFF;
    }
    if (ptr->dac_setting > ptr->max_dac_setting) {
      ptr->dac_setting = ptr->max_dac_setting;
    }
  }
}


void CheckAnalogFaults(void) {
  // ------------------- CHECK BIAS VOLTAGE FAULTS -------------------- //
  if (ETMAnalogCheckOverAbsolute(&bias_vmon)) {
    _FAULT_BIAS_OVER_VOLTAGE_ABSOLUTE = 1;
   // PIN_PIC_HOT_FLT = 1; // ***This should be uncommented later ***
    PIN_BIAS_FLT = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_BIAS_OVER_VOLTAGE_ABSOLUTE = 0;
      PIN_PIC_HOT_FLT = 0;
      PIN_BIAS_FLT = 0;
    }
  }
  
  if (ETMAnalogCheckUnderAbsolute(&bias_vmon)) {
    if (global_data_A36717.control_state > STATE_BIAS_SUPPLY_RAMP_UP) {
      _FAULT_BIAS_UNDER_VOLTAGE_ABSOLUTE = 1;
    //  PIN_PIC_COLD_FLT = 1; // ***This should be uncommented later ***
      PIN_BIAS_FLT = 1;

    }
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_BIAS_UNDER_VOLTAGE_ABSOLUTE = 0;
      PIN_PIC_COLD_FLT = 0;
      PIN_BIAS_FLT = 0;
    }
  }
  

  // ---------------------- CHECK TOP 1 VOLTAGE FAULTS ------------------- //
  if (ETMAnalogCheckOverAbsolute(&top_1_vmon)) {
    _FAULT_TOP_1_OVER_VOLTAGE_ABSOLUTE = 1;
   // PIN_PIC_HOT_FLT = 1; // ***This should be uncommented later ***
    PIN_TOP_FLT = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_TOP_1_OVER_VOLTAGE_ABSOLUTE = 0;
      PIN_PIC_HOT_FLT = 0;
      PIN_TOP_FLT = 0;
    }
  }

  if (ETMAnalogCheckUnderAbsolute(&top_1_vmon)) {
    if (global_data_A36717.control_state > STATE_TOP_RAMP_UP) {
      _FAULT_TOP_1_UNDER_VOLTAGE_ABSOLUTE = 1;
    //  PIN_PIC_HOT_FLT = 1; // ***This should be uncommented later ***
      PIN_TOP_FLT = 1;
    }
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_TOP_1_UNDER_VOLTAGE_ABSOLUTE = 0;
      PIN_PIC_HOT_FLT = 0;
      PIN_TOP_FLT = 0;
    }
  }


  // ---------------------- CHECK TOP 2 VOLTAGE FAULTS ------------------- //
  if (ETMAnalogCheckOverAbsolute(&top_2_vmon)) {
    _FAULT_TOP_2_OVER_VOLTAGE_ABSOLUTE = 1;
    //PIN_PIC_HOT_FLT = 1; // ***This should be uncommented later ***
    PIN_TOP_FLT = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_TOP_2_OVER_VOLTAGE_ABSOLUTE = 0;
      PIN_PIC_HOT_FLT = 0;
      PIN_TOP_FLT = 0;
    }
  }

  if (ETMAnalogCheckUnderAbsolute(&top_2_vmon)) {
    if (global_data_A36717.control_state > STATE_TOP_RAMP_UP) {
      _FAULT_TOP_2_UNDER_VOLTAGE_ABSOLUTE = 1;
    //  PIN_PIC_HOT_FLT = 1; // ***This should be uncommented later ***
      PIN_TOP_FLT = 1;
    }
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_TOP_2_UNDER_VOLTAGE_ABSOLUTE = 0;
      PIN_PIC_HOT_FLT = 0;
      PIN_TOP_FLT = 0;
    }
  }

// ------------------------ HEATER FAULTS ------------------------ //
if (global_data_A36717.status & 0x01)
{
  _FAULT_HEATER_OVER_VOLTAGE_ABSOLUTE = 1;
}
else
{
  _FAULT_HEATER_OVER_VOLTAGE_ABSOLUTE = 0;
}

if (global_data_A36717.status & 0x02)
{
  _FAULT_HEATER_UNDER_VOLTAGE_ABSOLUTE = 1;
}
else
{
  _FAULT_HEATER_UNDER_VOLTAGE_ABSOLUTE = 0;
}

if (global_data_A36717.status & 0x04)
{
  _FAULT_HEATER_OVER_CURRENT_ABSOLUTE  = 1;
}
else
{
  _FAULT_HEATER_OVER_CURRENT_ABSOLUTE = 0;
}

if (global_data_A36717.status & 0x08)
{
  _FAULT_HEATER_UNDER_CURRENT_ABSOLUTE = 1;
}
else
{
  _FAULT_HEATER_UNDER_CURRENT_ABSOLUTE = 0;
}

if (global_data_A36717.status & 0x10)
{
  _FAULT_HEATER_NOT_READY = 1;
  //PIN_PIC_COLD_FLT = 1; // ***This should be uncommented later ***
  //PIN_PIC_HTR_FLT = 1; // ***This should be uncommented later ***
}
else
{
  _FAULT_HEATER_NOT_READY = 0;
  PIN_PIC_COLD_FLT = 0;
  PIN_PIC_HTR_FLT = 0;
}

  
}
  

void UpdateBias (void) {
    unsigned int temp;
    temp= drive_per_PRF[global_data_A36717.detected_PRF];
    if ((temp >= bias_supply.min_dac_setting) && (temp <= bias_supply.max_dac_setting))
    {
        if(temp >= global_data_A36717.last_detected_PRF)
        {
      if ((temp - global_data_A36717.last_detected_PRF >=200))
      {
        global_data_A36717.last_detected_PRF = temp;
        bias_supply.dac_setting = temp;
        top_supply.dac_setting = top_drive_per_PRF[global_data_A36717.detected_PRF];
        
        dac_return_value= WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, bias_supply.dac_setting);
        WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, top_supply.dac_setting);
      }
        }
         if(temp <= global_data_A36717.last_detected_PRF)
        {
      if ((global_data_A36717.last_detected_PRF -temp >=200))
      {
        global_data_A36717.last_detected_PRF = temp;
        bias_supply.dac_setting = temp;
        top_supply.dac_setting = top_drive_per_PRF[global_data_A36717.detected_PRF];
        
        dac_return_value= WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, bias_supply.dac_setting);
        WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, top_supply.dac_setting);
      }
    }
    }

}

void A36717TransmitData(void) {
  unsigned char transmitMessage[transmitMessageLength];
  transmitMessage[0] = 0xFF; //sync
  transmitMessage[1] = global_data_A36717.status;
  transmitMessage[2] = top_1_set.dac_setting_scaled_and_calibrated >>8; // Top 1 set voltage hi byte
  transmitMessage[3] = top_1_set.dac_setting_scaled_and_calibrated & 0x00FF; // Top 1 set voltage lo byte
  transmitMessage[4] = top_2_set.dac_setting_scaled_and_calibrated >>8; // Top 2 set voltage hi byte
  transmitMessage[5] = top_2_set.dac_setting_scaled_and_calibrated & 0x00FF; // Top 2 set voltage lo byte
  transmitMessage[6] = heater_set_point >>8; // Heater set voltage hi byte
  transmitMessage[7] = heater_set_point & 0x00FF; // Heater set voltage lo byte
  unsigned int crc = ETMCRC16( &transmitMessage[0], transmitMessageLength);

  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[0]); // Sync
  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[1]); // Status
  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[2]);      // Top 1 Set High Byte
  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[3]);  // Top 1 Set High Byte
  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[4]);      // Top 2 Set High Byte
  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[5]);  // Top 2 Set High Byte
  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[6]);     // Heater Set High Byte
  BufferByte64WriteByte(&uart1_output_buffer, transmitMessage[7]); // Heater Set High Byte
  BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8));
  BufferByte64WriteByte(&uart1_output_buffer, (crc & 0xFF));
 

  if ((!U1STAbits.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer))) { 
    //fill TX REG and then wait for interrupt to fill the rest.
    U1TXREG =  BufferByte64ReadByte(&uart1_output_buffer);
  }
}



void A36717ReceiveData(void) {
  unsigned char message_data[8];
  unsigned char read_byte;
  unsigned int crc;

  // Look for a command
  while ( (BufferByte64BytesInBuffer(&uart1_input_buffer)) >= COMMAND_LENGTH) {
    read_byte = BufferByte64ReadByte(&uart1_input_buffer);
    if ((read_byte <= 0xF5) && (read_byte >= 0xF0)) {
      // All of the sync bytes matched, this should be a valid command
      message_data[0] = read_byte;
      message_data[1] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[2] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[3] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[4] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[5] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[6] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[7] = BufferByte64ReadByte(&uart1_input_buffer);
      crc = BufferByte64ReadByte(&uart1_input_buffer);
      crc <<= 8;
      crc += BufferByte64ReadByte(&uart1_input_buffer);
      if (crc == ETMCRC16(&message_data[0],8)) {
	       A36717DownloadData(message_data);
	       global_data_A36717.counter_100us_high_side_loss = 0;
	       return;  // Stop after one successful message
      }
    }
  }
}

#define TOP_1_VMON_SELECT           0
#define TOP_2_VMON_SELECT           1
#define HEATER_VMON_SELECT          2
#define HEATER_1_IMON_SELECT        3
#define HEATER_2_IMON_SELECT        4


void A36717DownloadData(unsigned char *msg_data) {
  unsigned int temp;

  global_data_A36717.status = msg_data[1];

  temp = msg_data[2];
  temp <<= 8;
  temp += msg_data[3];
  top_raw_vmon.filtered_adc_reading = 0xFFFF-temp;
  ETMAnalogScaleCalibrateADCReading(&top_raw_vmon);

  temp = msg_data[4];
  temp <<= 8;
  temp += msg_data[5];
  bias_vmon.filtered_adc_reading = temp;
  ETMAnalogScaleCalibrateADCReading(&bias_vmon);

  temp = msg_data[6];
  temp <<= 8;
  temp += msg_data[7];
  switch ((msg_data[0] & 0x0F)) 
    {
    case TOP_1_VMON_SELECT:
      top_1_vmon.filtered_adc_reading = temp;
      ETMAnalogScaleCalibrateADCReading(&top_1_vmon);
      break;
      
    case TOP_2_VMON_SELECT:
      top_2_vmon.filtered_adc_reading = temp;
      ETMAnalogScaleCalibrateADCReading(&top_2_vmon);
      break;
      
    case HEATER_VMON_SELECT:
      heater_vmon.filtered_adc_reading = temp;
      ETMAnalogScaleCalibrateADCReading(&heater_vmon);
      break;
      
    case HEATER_1_IMON_SELECT:
      heater_1_imon.filtered_adc_reading = temp;
      ETMAnalogScaleCalibrateADCReading(&heater_1_imon);
      break;
      
    case HEATER_2_IMON_SELECT:
      heater_2_imon.filtered_adc_reading = temp;
      ETMAnalogScaleCalibrateADCReading(&heater_2_imon);
      break;
    }
  
  do_control = 1;
  
}


/* 
   Faults
   Cold Faults
   Use Pic to disable trigger pulses - Send Cold fault discrete line
   Bias absolute Over/under voltage - 1st reading
   No message from high side of N uSeconds
   
   
   Warm Faults
   Use Pic to disable trigger pulses
   High Side pic sends - Heater Warm Fault 
   Over/under voltage on Top1/Top2 Feedback - 1st reading
   
   
   When do you shut down the two power supplies
   Bias never shuts down due to a fault
   Top supply should shut down if there is an over voltage (on Top1 and Top 2 Feedback)
   
   Top Raw 1 & 2 - Never Shut down based on these
   Bias V  - Over/Under - 
   
*/






void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U1RXInterrupt(void) {
  _U1RXIF = 0;
  while (U1STAbits.URXDA) {
    BufferByte64WriteByte(&uart1_input_buffer, U1RXREG);
  }
}



void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U1TXInterrupt(void) {
  _U1TXIF = 0;
  if ((!U1STAbits.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer) )) { 
    //fill TX REG and then wait for interrupt to fill the rest.
    U1TXREG =  BufferByte64ReadByte(&uart1_output_buffer);
  }
}




void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();
    Nop();
    __asm__ ("Reset");
}


void __attribute__((interrupt, no_auto_psv)) _IC4Interrupt(void) {
    unsigned int temp;
    TMR2 =0x0000;
    _IC4IF = 0;
    
    if (global_data_A36717.input_capture_sample == 1)
    {
        temp = IC4BUF;
        temp= temp >> 4;
        if (temp >= last_PRF_sample)
        {
            if (temp- last_PRF_sample >=1)
            {
                last_PRF_sample = temp;
            }
            else
            {
                last_PRF_sample = temp;
                global_data_A36717.detected_PRF = temp;
                datalog_counter %= 10;
                detected_PRF_log[++datalog_counter]= temp;
                UpdateBias();
            }
        }
        else
        {
            if ( last_PRF_sample -temp >=1)
            {
                last_PRF_sample = temp;

            }
            else
            {
                last_PRF_sample = temp;
                global_data_A36717.detected_PRF = temp;
                datalog_counter %= 10;
                detected_PRF_log[++datalog_counter]= temp;
                UpdateBias();
            }
        }
    }
    else
    {
      global_data_A36717.input_capture_sample = 1;
      temp = IC4BUF;
    }
}



void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
      
    case ETM_CAN_REGISTER_GUN_DRIVER_SET_1_GRID_TOP_SET_POINT:
      top_2_set.set_point = message_ptr->word0;
      ETMAnalogScaleCalibrateDACSetting(&top_2_set);
      top_1_set.set_point = message_ptr->word1;
      ETMAnalogScaleCalibrateDACSetting(&top_1_set);
      _CONTROL_NOT_CONFIGURED = 0;
      break;
      
    case ETM_CAN_REGISTER_GUN_DRIVER_SET_1_HEATER_CATHODE_SET_POINT:
      heater_set_point = message_ptr->word0;
      //_CONTROL_NOT_CONFIGURED = AreAnyReferenceNotConfigured(); // DPARKER fix this
      break;
       
    default:
      //local_can_errors.invalid_index++;
      break;
    }
}
