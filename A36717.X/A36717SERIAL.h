/*
  Module Description
  Only one type of message can be sent and only one type of message can be received.
  Recieved data is 1 ID byte, 4 bytes of data and 2 bytes of crc.
  The uart module calculates the crc for transmitting, and checks it upon receiving.
  If crc does not match, message is ignored.
  Module assumes there is a defined amount of time allowed per message- period between 2 messages is checked using timer4 (receiving) and timer5 (transmitting).



  Assumptions
  FCY_CLK is defined (Fosc/2)

*/


#ifndef __SERIAL_H
#define __SERIAL_H





// User Configuration Parameters
#define A36717_SERIAL_BAUDRATE         1843200 // 9600//
#define A36717_SERIAL_UART_INT_PRI     4

#define COMMAND_BUFFER_EMPTY  0x00
#define COMMAND_BUFFER_FULL   0x02

#define A36717_SERIAL_UART_MODE_VALUE  (UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_SIMPLEX & UART_UEN_00 &  UART_EN_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36717_SERIAL_UART_STA_VALUE   (UART_INT_TX & UART_TX_ENABLE & UART_SYNC_BREAK_DISABLED & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_IrDA_POL_INV_ZERO)


#define FEEDBACK_MSG  0xF1 
#define SETTINGS_MSG  0xF2





void InitializeA36717Serial(void);

void A36717TransmitData(void); 
void A36717ReceiveData(void); //ReceiveData is used by the Serial module once the RX message has been fully received. This functions checks CRC and if good, moves data from input buffer to global data structure.


#endif