
#include <xc.h>
#include <uart.h>
#include "A36717SERIAL.h"
#include "A36717.h"

/*
typedef struct  {
  //unsigned char command_byte;
  unsigned char top1_set;
  unsigned char top2_set;
  unsigned char heater_set;
  unsigned char heater_enable;
  unsigned char output_crc_hi;
  unsigned char output_crc_lo;
}OutputData;
*/

typedef struct
{
  //unsigned char command_byte;
  unsigned char top_fdbk_hi;
  unsigned char top_fdbk_lo;
  unsigned char bias_fdbk_hi;
  unsigned char bias_fdbk_lo;
  unsigned char top1_mon;
  unsigned char top2_mon;
  unsigned char heater_voltage_mon;
  unsigned char heater1_current_mon;
  unsigned char heater2_current_mon;
  unsigned char status;
}InputData;



//OutputBuffer A36717_output_buffer;
InputData A36717inputdata;


// possible commands (application specific):


#define COMMAND_LENGTH 12


#define A36717_SERIAL_UART_BRG_VALUE   (unsigned int)(((FCY_CLK/A36717_SERIAL_BAUDRATE)/16)-1)


#define SERIAL_UART_INT_PRI 5

//#define ETM_SERIAL_UART_MODE_VALUE  0b1010000000000000
//#define ETM_SERIAL_UART_STA_VALUE   0b0000110001000000



#define UART_STATS_BITS     U1STAbits
#define UART_TX_REG         U1TXREG
#define UART_RX_REG         U1RXREG
#define UART_BRG            U1BRG
#define UART_MODE           U1MODE
#define UART_STA            U1STA

#define UART_RX_IF         _U1RXIF
#define UART_TX_IF         _U1TXIF
#define UART_RX_IE         _U1RXIE
#define UART_TX_IE         _U1TXIE 
#define UART_RX_IP         _U1RXIP
#define UART_TX_IP         _U1TXIP


#define UART_RX_INTERRUPT  _U1RXInterrupt
#define UART_TX_INTERRUPT  _U1TXInterrupt



BUFFERBYTE64 uart1_input_buffer;
BUFFERBYTE64 uart1_output_buffer;


//void A36717MakeCRC(unsigned OutputData* data);
//int A36717CheckCRC(unsigned InputData* data);
void A36717LoadData(void); //Moves data from main global structure to output buffer and generates CRC.
void A36717DownloadData(void); //Checks CRC and if good - moves data from input buffer to main global variable.


void InitializeA36717Serial(void) 
{
  UART_RX_IP = SERIAL_UART_INT_PRI;
  UART_TX_IP = SERIAL_UART_INT_PRI;

  UART_RX_IF = 0;
  UART_TX_IF = 0;
  UART_RX_IE = 1;
  UART_TX_IE = 1;


  UART_BRG  = A36717_U1BRG_VALUE;  //A36717_SERIAL_UART_BRG_VALUE;
  UART_MODE = A36717_U1MODE_VALUE; //A36717_SERIAL_UART_MODE_VALUE;
  UART_STA  = A36717_U1STA_VALUE;  //A36717_SERIAL_UART_STA_VALUE;
 

}


void A36717LoadData(void)
{
  unsigned int crc= 0x5555;
  BufferByte64WriteByte(&uart1_output_buffer,SETTINGS_MSG);
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.top1_set_voltage);
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.top2_set_voltage);
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.heater_set_voltage);
  BufferByte64WriteByte(&uart1_output_buffer, global_data_A36717.heater_enable);
  BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8)); //should be crc hi
  BufferByte64WriteByte(&uart1_output_buffer, (crc & 0xFF)); //should be crc lo
};


void A36717TransmitData(void)
{
  A36717LoadData();
  if ((!UART_STATS_BITS.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer)) )
  { //fill TX REG and then wait for interrupt to fill the rest.
    U1TXREG =  BufferByte64ReadByte(&uart1_output_buffer);
  }
};

void A36717ReceiveData(void)
{
  int temp = BufferByte64BytesInBuffer(&uart1_input_buffer);
  while ( (BufferByte64BytesInBuffer(&uart1_input_buffer)) >= COMMAND_LENGTH) {
    // Look for a command
    unsigned char read_byte;
    unsigned int crc;
    read_byte = BufferByte64ReadByte(&uart1_input_buffer);
    if (read_byte == FEEDBACK_MSG) {
      // All of the sync bytes matched, this should be a valid command
    A36717inputdata.top_fdbk_hi   = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.top_fdbk_lo = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.bias_fdbk_hi  = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.bias_fdbk_lo  = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.top1_mon   = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.top2_mon = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.heater_voltage_mon  = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.heater1_current_mon  = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.heater2_current_mon  = BufferByte64ReadByte(&uart1_input_buffer);
    A36717inputdata.status  = BufferByte64ReadByte(&uart1_input_buffer);
    crc = BufferByte64ReadByte(&uart1_input_buffer);
    crc = (crc << 8) & 0xFF00;
    crc += BufferByte64ReadByte(&uart1_input_buffer);
    if (crc == 0x5555)
      A36717DownloadData();
    }
  }
}
    

void A36717DownloadData(void)
{
  unsigned int worddata = (int) A36717inputdata.top_fdbk_hi;
  worddata = (worddata << 8) ;
  worddata += (int) A36717inputdata.top_fdbk_lo;
  global_data_A36717.top_feedback =  worddata;
  worddata = (int) A36717inputdata.bias_fdbk_hi;
  worddata = (worddata << 8) ;
  worddata += (int) A36717inputdata.bias_fdbk_lo;
  global_data_A36717.bias_feedback =  worddata;
  global_data_A36717.top1_voltage_monitor = A36717inputdata.top1_mon;
  global_data_A36717.top2_voltage_monitor = A36717inputdata.top2_mon;
  global_data_A36717.heater1_current_monitor = A36717inputdata.heater1_current_mon;
  global_data_A36717.heater2_current_monitor = A36717inputdata.heater2_current_mon;
  global_data_A36717.heater_output_voltage = A36717inputdata.heater_voltage_mon;
  global_data_A36717.status = A36717inputdata.status;  
}


/*
unsigned int MakeCRC(unsigned OutputData* data) {
  data.output_crc_hi=0x55;
  data.output_crc_lo = 0x55;
 // crc = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_SEND;
  //crc += command_byte + register_byte;
  //crc += (data_word >> 8);
  //crc += (data_word & 0x00FF);
  
  //return crc;
  // DPAKRER Make real CRC
}


unsigned char CheckCRC(unsigned InputData* data) {
 // unsigned int crcCheck;
  // At the moment the CRC is just a checksum
 // crcCheck = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_RECEIVE;
 // crcCheck += command_string.command_byte + command_string.register_byte;
 // crcCheck += command_string.data_high_byte + command_string.data_low_byte;
//  if (crcCheck == crc) {
 //   return 1;
 // } else {
    return 1;
  }
  // DPARKER make Real CRC

}

*/

void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) UART_RX_INTERRUPT(void) {
  UART_RX_IF = 0;
  while (U1STAbits.URXDA) {
    BufferByte64WriteByte(&uart1_input_buffer, U1RXREG);
  }
}



void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) UART_TX_INTERRUPT(void) {
  UART_TX_IF = 0;
  if ((!UART_STATS_BITS.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer) ))
    { //fill TX REG and then wait for interrupt to fill the rest.
      U1TXREG =  BufferByte64ReadByte(&uart1_output_buffer);
    }
}
