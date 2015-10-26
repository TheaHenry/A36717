// Header file
#ifndef __A36717_H
#define __A36717_H


#define FCY_CLK 31000000

#include <xc.h>
#include <timer.h>
#include "Buffer64.h"


#include "ETM.h"
#include "A36717SERIAL.h"

/*
  
  Hardware Module Resource Usage

  SPI2   - Used/Configured by LTC265X Module
 Timer 4 - UART RX
 * Timer5 - UART TX
 * Timer3 - 10ms timer
  
  
*/


// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// Pins to be configured as inputs
/*

None

*/

//#define A36717_TRISA_VALUE 0x0000 
#define A36717_TRISB_VALUE 0x0000 
#define A36717_TRISC_VALUE 0x0000 
#define A36717_TRISD_VALUE 0x0000 
#define A36717_TRISE_VALUE 0x0000
#define A36717_TRISF_VALUE 0x0000 
#define A36717_TRISG_VALUE 0x0000


// ------------- PIN DEFINITIONS ------------------- ///

#define PIN_PIC_KICK                    _LATB0
#define PIN_LED_TEST_POINT_A            _LATB6
#define PIN_BIAS_FLT                    _LATB9
#define PIN_TOP_FLT                     _LATB10
#define PIN_PIC_PULSE_ENABLE_NOT        _LATB13
#define PIN_PIC_COLD_FLT                _LATB14
#define PIN_PIC_HOT_FLT                 _LATB15
//#define PIN_PIC_HTR_WARMUP              _LATF5
#define PIN_BIAS_ENABLE                 _LATD11
#define PIN_TOP_ENABLE                  _LATD4
#define PIN_PIC_HTR_FLT                 _LATD5
#define PIN_LED_OPERATIONAL_GREEN       _LATD9



// ---------------- Timing Configuration Values ------------- //


/* 
   TMR3 Configuration
   Timer3 - Used for 100usTicToc
   Period should be set to 10mS
*/
#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_8 & T3_SOURCE_INT)
#define PR3_PERIOD_US                  100   // 100uS
#define PR3_VALUE_100_US      (unsigned int)((FCY_CLK / 1000000)*PR3_PERIOD_US/8)


#define STATE_STARTUP   0x10
#define STATE_READY     0x20
#define STATE_FAULT  0x30


typedef struct {
  unsigned int control_state;
  unsigned int heater_set_voltage;
  unsigned int heater_output_voltage; //needs to be an 8bit number
  unsigned int heater_enable;
  unsigned int top_set_voltage;
  unsigned int top_dac_setting_scaled;
  unsigned int bias_set_voltage;
  unsigned int bias_dac_setting_scaled;
  unsigned int top1_voltage_monitor; //needs to be an 8bit number
  unsigned int top2_voltage_monitor; //needs to be an 8bit number
  unsigned int heater1_current_monitor; //needs to be an 8bit number
  unsigned int heater2_current_monitor; //needs to be an 8bit number
  unsigned int bias_feedback;
  unsigned int top_feedback;
  unsigned int top1_set_voltage; //needs to be an 8bit number
  unsigned int top2_set_voltage; //needs to be an 8bit number
  unsigned char status;


} ControlData;

extern ControlData global_data_A36717;
extern BUFFER64BYTE uart1_input_buffer;
extern BUFFER64BYTE uart1_output_buffer;

#endif
