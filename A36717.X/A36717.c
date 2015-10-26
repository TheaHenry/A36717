#include "A36717.h"

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


// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (General Segment may be written)
#pragma config GSS = OFF                // General Segment Code Protection (General Segment Code protect is disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Internal Fast RC with PLL (FRCPLL))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 is clock output)
#pragma config FCKSM = CSECME           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR32            // WDT Prescaler (1:32)
#pragma config WINDIS = ON              // Watchdog Timer Window (Watchdog Timer in Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTSS1 = OFF             // Enable Alternate SS1 pin bit (SS1 is selected as the I/O pin for SPI1)
#pragma config ALTQIO = OFF             // Enable Alternate QEI1 pin bit (QEA1, QEB1, INDX1 are selected as inputs to QEI1)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is disabled)

// FCMP
#pragma config HYST0 = HYST45           // Even Comparator Hysteresis Select (45 mV Hysteresis)
#pragma config CMPPOL0 = POL_FALL       // Comparator Hysteresis Polarity (for even numbered comparators) (Hysteresis is applied to falling edge)
#pragma config HYST1 = HYST45           // Odd Comparator Hysteresis Select (45 mV Hysteresis)
#pragma config CMPPOL1 = POL_FALL       // Comparator Hysteresis Polarity (for odd numbered comparators) (Hysteresis is applied to falling edge)


//------------------------------------------------------------------------//


void DoStateMachine(void);
void InitializeA36717(void);
void ConfigureClock(void);

ControlData global_data_A36717;
LTC265X U10_LTC2654;


int main(void) {

  ConfigureClock();

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
    PIN_LED_OPERATIONAL_GREEN = 1;
    PIN_LED_TEST_POINT_A = 1;
    unsigned int flashDuration = 5000;
    while(global_data_A36717.control_state == STATE_READY)
    {
        
        if (_T3IF ==1 )
        {
            _T3IF = 0;
            flashDuration--;
            if(PIN_PIC_KICK ==1) //kick pic external watchdog every 200us ( times out if 5 consecutive kicks are missed) 
              PIN_PIC_KICK = 0;
            else
              PIN_PIC_KICK = 1;

            if (global_data_A36717.heater_set_voltage != global_data_A36717.top_feedback)
            {
              global_data_A36717.heater_set_voltage = global_data_A36717.top_feedback;
              
            }
           A36717TransmitData();
        }

        if (flashDuration ==0)
        {
            if(PIN_LED_TEST_POINT_A==1)
                PIN_LED_TEST_POINT_A=0;
            else
                PIN_LED_TEST_POINT_A=1;

            flashDuration = 50000;
            if (global_data_A36717.heater_set_voltage != global_data_A36717.top_feedback)
            {
              global_data_A36717.heater_set_voltage = global_data_A36717.top_feedback;
              A36717TransmitData();
            }
        }

      
      A36717ReceiveData();
      
      

    }
    break;
     
    
 case STATE_FAULT:
     
	break;
	
  default:
    global_data_A36717.control_state = STATE_READY;

    break;

  }
}



void ConfigureClock(void)
{

  //**********************Setup Clock speeds*********************************//
  //   Fin=7.3MHz
  //   Fosc = Fin*M/(N1+N2), Fcy=Fosc/2
  //   Fosc= 7.49 * 50 / (2 * 3) = 62.4MHz, Fcy= 31.2MIPS
  //*************************************************************************//
  // Configure PLL prescaler, PLL postscaler, PLL divisor
  //_TUN = 4;//tune FRC to 7.49MHz
  PLLFBD = 48;// M = 50
  CLKDIVbits.PLLPOST=0;// N2 = 2
  CLKDIVbits.PLLPRE=1;// N1 = 3
  // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
  __builtin_write_OSCCONH(0x01);
  __builtin_write_OSCCONL(OSCCON | 0x01);
  // Wait for Clock switch to occur
  while (OSCCONbits.COSC != 0b001);
  // Wait for PLL to lock
  while(OSCCONbits.LOCK!=1) {};


// CLKDIVbits.PLLPOST = 0b00; // Set PLL Postscaler (N2) to 2.
//  CLKDIVbits.PLLPRE = 0b00000; // Set PLL Prescaler (N1) to 2.
//  PLLFBD = 58; // Set PLL Divider (M) to 60.


  //Auxilary clock configuration
//  _SELACLK = 0; //PLL output (FVCO) provides the source clock for the auxiliary clock divider
//  _APSTSCLR = 0b111; // Auxiliary Clock Output Divider 1:1

  ACLKCONbits.FRCSEL = 1; /* Internal FRC is clock source for auxiliary PLL */
  
  ACLKCONbits.SELACLK = 1;/* Auxiliary PLL provides the source clock for the */
  /* clock divider */
  ACLKCONbits.APSTSCLR = 7;/* Auxiliary Clock Output Divider is Divide-by-1 */
  ACLKCONbits.ENAPLL = 1; /* APLL is enabled */

  while(ACLKCONbits.APLLCK != 1){}; /* Wait for Auxiliary PLL to Lock */
  /* Given a 7.5MHz input from the FRC the Auxiliary Clock for the ADC and PWM */
  /* modules are 7.5MHz * 16 = 120MHz */

  /* Disable Watch Dog Timer */
        RCONbits.SWDTEN = 0;


}


void InitializeA36717(void) 
{
  //Timer setup
  T3CON = T3CON_VALUE;
  PR3 = PR3_VALUE_100_US;
  _T3IF = 0;
  DisableIntT3;

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
  TRISB = A36717_TRISB_VALUE;
  TRISC = A36717_TRISC_VALUE;
  TRISD = A36717_TRISD_VALUE;
  TRISE = A36717_TRISE_VALUE;
  TRISF = A36717_TRISF_VALUE;
  TRISG = A36717_TRISG_VALUE;

  SetupLTC265X(&U10_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RE6, _PIN_RE7);

  InitializeA36717Serial();

}



void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();
    Nop();

}

