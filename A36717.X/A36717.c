#include "A36717.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"

// This is the firmware for modulator- HV section

/*modules that need to be developed:
 * Serial communicationi with LV section (Uart) - need to rewrite the recieve (how do we know start of messsage?), needs CRC
 *bootloader

	Other open items:
	* how do we want to respond to HTR faults - as of now, heater fault conditions will be chacked by LV section only.
	* how do we respond to xmit faults?
	* CRC?
	* watchdog?
	* average current mode? peak current mode?
  * UpdateTopVoltage(); // change this to be done every 1 sec?
  * A36717TransmitData(); //does this need to be gated? Do I need to check that the previous message has been sent?

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
void InitializeA36717(void);
void DoA36717(void);

ControlData global_data_A36717;
LTC265X U10_LTC2654;



BUFFERBYTE64 uart1_input_buffer;
BUFFERBYTE64 uart1_output_buffer;


//void A36717MakeCRC(unsigned OutputData* data);
//int A36717CheckCRC(unsigned InputData* data);
void A36717LoadData(void); //Moves data from main global structure to output buffer and generates CRC.
//void A36717DownloadData(void); //Checks CRC and if good - moves data from input buffer to main global variable.
void A36717DownloadData(unsigned char *msg_data);

void A36717TransmitData(void); 
void A36717ReceiveData(void); //ReceiveData is used by the Serial module once the RX message has been fully received. This functions checks CRC and if good, moves data from input buffer to global data structure.

//OutputBuffer A36717_output_buffer;
InputData A36717inputdata;



AnalogInput top_1_raw_vmon;  
AnalogInput top_2_raw_vmon;
AnalogInput bias_vmon;

AnalogInput top_1_vmon;      
AnalogInput top_2_vmon;
AnalogInput heater_vmon;
AnalogInput heater_1_imon;
AnalogInput heater_2_imon;

unsigned int accumulator_counter;

unsigned int do_control;

unsigned int heater_set_point;
unsigned int top_1_set_point;
unsigned int top_2_set_point;


/*
  This is a non-linear integral-ish compensation scheme.
  It is not stable and will never reach a steady state response but will osciallate around the target.
  It should be adjusted so that the osciallations are small
  
  If the target reading is within the min/max window.
  The dac_setting is adjusted by slow_step.

  If the target reading is outsode the min/max window.
  The dac_setting is adjusted by fast_step.
  
*/


typedef struct {
  unsigned int dac_setting;
  unsigned int reading;
  unsigned int target;
  unsigned int min_window;
  unsigned int max_window;
  unsigned int margin_target;
  unsigned int fast_step;
  unsigned int slow_step;
  unsigned int step_gain_sign;
} TYPE_CONTROL;






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
    global_data_A36717.control_state = STATE_READY;

    break;
	



  case STATE_READY:
    PIN_BIAS_ENABLE = !ENABLE_SUPPLY;
    PIN_TOP_ENABLE  = !ENABLE_SUPPLY;
    WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, 0x1F00);
    WriteLTC265X(&U10_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, 0x1F00);
    while(global_data_A36717.control_state == STATE_READY) {
      DoA36717();
    }
    
    break;
    
    
 case STATE_FAULT:
     
	break;
	
  default:
    global_data_A36717.control_state = STATE_READY;

    break;
  }
}


void DoA36717(void) {
  A36717ReceiveData();
  ETMCanSlaveDoCan();
  if (do_control) {
    // A new set of data has been received from the high side
    // Run an iteration of the control loop
    do_control = 0;
    //DoControlLoops(); DPARKER write this
  }


  if (_T3IF == 1 ) {
    // This happens once every 100uS
    _T3IF = 0;

    slave_board_data.log_data[0] = 6500;  // BIAS_SET_POINT
    slave_board_data.log_data[1] = 500;   // top_1_raw_vmon.reading_scaled_and_calibrated;
    slave_board_data.log_data[2] = 12750; // top_1_vmon.reading_scaled_and_calibrated;
    slave_board_data.log_data[3] = top_1_set_point;

    slave_board_data.log_data[4] = 6600;  // bias_vmon.reading_scaled_and_calibrated;
    slave_board_data.log_data[5] = 550;   // top_2_raw_vmon.reading_scaled_and_calibrated;
    slave_board_data.log_data[6] = 13000; // top_2_vmon.reading_scaled_and_calibrated;
    slave_board_data.log_data[7] = top_2_set_point;

    slave_board_data.log_data[8] = 1000;  // heater_1_imon.reading_scaled_and_calibrated;
    slave_board_data.log_data[9] = 1200;  // heater_2_imon.reading_scaled_and_calibrated;
    slave_board_data.log_data[10] = 13000; // heater_vmon.reading_scaled_and_calibrated
    slave_board_data.log_data[11] = heater_set_point;

    ETMCanSlaveSetDebugRegister(0x0, 0);
    ETMCanSlaveSetDebugRegister(0x1, 11);
    ETMCanSlaveSetDebugRegister(0x2, 22);
    ETMCanSlaveSetDebugRegister(0x3, 33);
    ETMCanSlaveSetDebugRegister(0x4, 44);
    ETMCanSlaveSetDebugRegister(0x5, 55);
    ETMCanSlaveSetDebugRegister(0x6, 66);
    ETMCanSlaveSetDebugRegister(0x7, 77);
    ETMCanSlaveSetDebugRegister(0x8, 88);
    ETMCanSlaveSetDebugRegister(0x9, 99);
    ETMCanSlaveSetDebugRegister(0xA, 101);
    ETMCanSlaveSetDebugRegister(0xB, 102);
    ETMCanSlaveSetDebugRegister(0xC, 103);
    ETMCanSlaveSetDebugRegister(0xD, 104);
    ETMCanSlaveSetDebugRegister(0xE, 105);
    ETMCanSlaveSetDebugRegister(0xF, 106);
  


    global_data_A36717.led_counter++;
    global_data_A36717.led_counter &= 0x7FFF;

    if ((global_data_A36717.led_counter & 0x03FF) == 0) {
      // this will be true every ~100mS
      if(PIN_LED_OPERATIONAL_GREEN == 1) {
	PIN_LED_OPERATIONAL_GREEN = 0;
      } else {
	PIN_LED_OPERATIONAL_GREEN = 1;
      } 
    }


    if (global_data_A36717.led_counter == 0) {
      // this will be true once every 3.27 Seconds
      if (global_data_A36717.heater_set_voltage != global_data_A36717.top_feedback) {
	global_data_A36717.heater_set_voltage = global_data_A36717.top_feedback;
	A36717TransmitData();
      }
            
    }

    if (PIN_PIC_KICK == 1) { 
      //kick pic external watchdog every 200us ( times out if 5 consecutive kicks are missed) 
      PIN_PIC_KICK = 0;
    } else {
      PIN_PIC_KICK = 1;
    }
    if (global_data_A36717.heater_set_voltage != global_data_A36717.top_feedback) {
      global_data_A36717.heater_set_voltage = global_data_A36717.top_feedback;
    }
    
    A36717TransmitData();
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
  PIN_PIC_PULSE_ENABLE_NOT = 1;
  PIN_LED_TEST_POINT_A = 0;
  
  //init global variables
  global_data_A36717.heater_set_voltage = 0x03;//624; //10V heater output
  global_data_A36717.heater_output_voltage = 0;
  global_data_A36717.top_set_voltage = 2496; //2V from DAC
  global_data_A36717.top_dac_setting_scaled = 0;
  global_data_A36717.bias_set_voltage = 2496; //2V from DAC
  global_data_A36717.bias_dac_setting_scaled = 0;
  global_data_A36717.top1_voltage_monitor = 0;
  global_data_A36717.top2_voltage_monitor = 0;
  global_data_A36717.top1_set_voltage = 0x01;
  global_data_A36717.top2_set_voltage = 0x02;
  global_data_A36717.heater1_current_monitor = 0;
  global_data_A36717.heater2_current_monitor = 0;
  global_data_A36717.bias_feedback = 0;
  global_data_A36717.top_feedback = 0;
  global_data_A36717.status = 1;
  

  //set tris
  TRISA = A36717_TRISA_VALUE;
  TRISB = A36717_TRISB_VALUE;
  TRISC = A36717_TRISC_VALUE;
  TRISD = A36717_TRISD_VALUE;
  TRISF = A36717_TRISF_VALUE;
  TRISG = A36717_TRISG_VALUE;
  
  SetupLTC265X(&U10_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
  

  // Set up the UART
  _U1RXIP = 5;
  _U1TXIP = 5;
  
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
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RB6, 4);
  ETMCanSlaveLoadConfiguration(36717, 0, AGILE_REV, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV, SERIAL_NUMBER);
}



void A36717LoadData(void) {
  unsigned int crc= 0x5555;
  BufferByte64WriteByte(&uart1_output_buffer,SETTINGS_MSG);                         // Sync
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.top1_set_voltage); // 2
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.top2_set_voltage); // 2
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.heater_set_voltage); // 2
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.heater_enable);    
  BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8)); //should be crc hi      // 2
  BufferByte64WriteByte(&uart1_output_buffer, (crc & 0xFF)); //should be crc lo
}


void A36717TransmitData(void) {
  //A36717LoadData();
  

  if ((!U1STAbits.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer))) { 
    //fill TX REG and then wait for interrupt to fill the rest.
    U1TXREG =  BufferByte64ReadByte(&uart1_output_buffer);
  }
}



void A36717ReceiveData(void) {
  unsigned char message_data[10];
  unsigned char read_byte;
  unsigned int crc;

  // Look for a command
  while ( (BufferByte64BytesInBuffer(&uart1_input_buffer)) >= COMMAND_LENGTH) {
    read_byte = BufferByte64ReadByte(&uart1_input_buffer);
    if (read_byte == FEEDBACK_MSG) {
      // All of the sync bytes matched, this should be a valid command
      message_data[0] = read_byte;
      message_data[1] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[2] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[3] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[4] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[5] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[6] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[7] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[8] = BufferByte64ReadByte(&uart1_input_buffer);
      message_data[9] = BufferByte64ReadByte(&uart1_input_buffer);
      crc = BufferByte64ReadByte(&uart1_input_buffer);
      crc <<= 8;
      crc += BufferByte64ReadByte(&uart1_input_buffer);
      if (crc == 0x5555) {
	//A36717DownloadData(&message_data); DPARKER fix this
	return;  // Stop after one successful message
      }
    }
  }
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





#define TOP_1_VMON_SELECT           0
#define TOP_2_VMON_SELECT           1
#define HEATER_VMON_SELECT          2
#define HEATER_1_IMON_SELECT        3
#define HEATER_2_IMON_SELECT        4


void A36717DownloadData(unsigned char *msg_data) {
  unsigned int temp;

  temp = msg_data[2];
  temp <<= 8;
  temp += msg_data[3];
  top_1_raw_vmon.filtered_adc_reading = temp;
  ETMAnalogScaleCalibrateADCReading(&top_1_raw_vmon);

  temp = msg_data[4];
  temp <<= 8;
  temp += msg_data[5];
  top_2_raw_vmon.filtered_adc_reading = temp;
  ETMAnalogScaleCalibrateADCReading(&top_2_raw_vmon);

  temp = msg_data[6];
  temp <<= 8;
  temp += msg_data[7];
  bias_vmon.filtered_adc_reading = temp;
  ETMAnalogScaleCalibrateADCReading(&bias_vmon);

  temp = msg_data[8];
  temp <<= 8;
  temp += msg_data[9];
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



void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
      
    case ETM_CAN_REGISTER_GUN_DRIVER_SET_1_GRID_TOP_SET_POINT:
      top_2_set_point = message_ptr->word0;
      top_1_set_point = message_ptr->word1;
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
