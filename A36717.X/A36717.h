// Header file
#ifndef __A36717_H
#define __A36717_H



#include <xc.h>
#include <timer.h>
#include <libpic30.h>
#include <uart.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"


#define FCY_CLK     10000000

/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI2   - Used/Configured by LTC265X Module

  Timer3 - Used to time 100us

  ADC Module - NOT USED

*/






/*
  
  Hardware Module Resource Usage
  
  SPI2   - Used/Configured by LTC265X Module
  Timer 4 - UART RX
  * Timer5 - UART TX
  * Timer3 - 10ms timer
  
  
*/


// ------------- PIN DEFINITIONS ------------------- ///
#define PIN_PIC_KICK                    _LATB2
#define PIN_LED_TEST_POINT_A            _LATB6
#define PIN_BIAS_FLT                    _LATB9
#define PIN_TOP_FLT                     _LATB8
#define PIN_PIC_PULSE_ENABLE_NOT        _LATB13
#define PIN_PIC_COLD_FLT                _LATB15
#define PIN_PIC_HOT_FLT                 _LATD14
#define PIN_PIC_HTR_WARMUP              _LATF4
#define PIN_LED_OPERATIONAL_GREEN       _LATD8
#define PIN_BIAS_ENABLE                 _LATD10
#define PIN_TOP_ENABLE                  _LATD12
#define PIN_PIC_HTR_FLT                 _LATD13


#define ENABLE_SUPPLY                   1

/*
  All of the above must be configured as outputs
  B2, B6, B8, B9, B13, B15
  D8, D10, D12, D13, D14
  F4

  
*/

#define A36717_TRISA_VALUE 0b1111111111111111
#define A36717_TRISB_VALUE 0b0101110010111011
#define A36717_TRISC_VALUE 0b1111111111111111
#define A36717_TRISD_VALUE 0b1000101011111111
#define A36717_TRISF_VALUE 0b1111111111101111
#define A36717_TRISG_VALUE 0b1111111111111111






// ---------------- Timing Configuration Values ------------- //


/* 
   TMR3 Configuration
   Timer3 - Used for 100usTicToc
*/
#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT)
#define PR3_VALUE_100_US               1000



// ------------ ADC CONFIGURATION ---------------- //
// ADC is off, all pins are digital I/0

#define ADCON1_SETTING  0x0000
#define ADCON2_SETTING  0x0000
#define ADCON3_SETTING  0x0000
#define ADCHS_SETTING   0x0000
#define ADPCFG_SETTING  0xFFFF
#define ADCSSL_SETTING  0x0000




typedef struct {
  unsigned int control_state;
  unsigned int heater_enable;
  unsigned int counter_100us;
  unsigned int counter_100us_high_side_loss;
  unsigned char status;
  unsigned int led_counter;
} ControlData;



// Serial Communication

#define COMMAND_LENGTH 12

#define COMMAND_BUFFER_EMPTY  0x00
#define COMMAND_BUFFER_FULL   0x02
#define FEEDBACK_MSG  0xF1 
#define SETTINGS_MSG  0xF2

#define UART1_BAUDRATE             312500//19200// 460800//625000
#define A36717_U1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36717_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A36717_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)

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
  unsigned int max_dac_setting;
  unsigned int min_dac_setting;
  unsigned int reading;
  unsigned int target;
  unsigned int min_window;
  unsigned int max_window;
  unsigned int fast_step_more_power;
  unsigned int slow_step_more_power;
  unsigned int fast_step_less_power;
  unsigned int slow_step_less_power;
} TYPE_UC2827_CONTROL;



#define STATE_STARTUP              0x10
#define STATE_WAIT_FOR_CONFIG      0x20

#define STATE_START_UP_FAULT       0x30
#define STATE_PERMA_FAULT          0x32
#define STATE_COLD_FAULT           0x35

#define STATE_BIAS_SUPPLY_RAMP_UP  0x40
#define STATE_HEATER_RAMP_UP       0x50
#define STATE_TOP_SUPPLY_RAMP_UP   0x60
#define STATE_TOP_RAMP_UP          0x70
#define STATE_OPERATE              0x80
#define STATE_WARM_FAULT           0x90


#define _FAULT_CAN_COMM_LOSS                                    _FAULT_0
#define _FAULT_BIAS_OVER_VOLTAGE_ABSOLUTE                       _FAULT_1
#define _FAULT_BIAS_UNDER_VOLTAGE_ABSOLUTE                      _FAULT_2
#define _FAULT_HIGH_SIDE_COMM_LOSS                              _FAULT_3
#define _FAULT_TOP_1_OVER_VOLTAGE_ABSOLUTE                      _FAULT_4
#define _FAULT_TOP_1_UNDER_VOLTAGE_ABSOLUTE                     _FAULT_5
#define _FAULT_TOP_2_OVER_VOLTAGE_ABSOLUTE                      _FAULT_6
#define _FAULT_TOP_2_UNDER_VOLTAGE_ABSOLUTE                     _FAULT_7


#endif
