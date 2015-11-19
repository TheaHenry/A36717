#ifndef __BUFFER64
#define __BUFFER64

typedef struct {
  unsigned char data[64];
  unsigned char write_location;
  unsigned char read_location;
} BUFFER64BYTE;

#define Buffer64Mask 0b00111111



  
void Buffer64WriteByte(BUFFER64BYTE* ptr, unsigned char value);
/*
  Writes a byte to the buffer
  If the buffer is full the oldest byte will be overwritten
*/

unsigned char Buffer64ReadByte(BUFFER64BYTE* ptr);
/*
  Reads a single byte from the buffer.
  If the buffer is empty zero will be returned and the write/read location will not be changed
  Before calling Buffer64ReadByte the buffer should be checked with Buffer64BytesInBuffer or Buffer64IsNotEmpty
*/

unsigned char Buffer64BytesInBuffer(BUFFER64BYTE* ptr);
/*
  Returns the number of bytes stored in the buffer
*/

unsigned char Buffer64IsNotEmpty(BUFFER64BYTE* ptr);
/*
  Returns zero if the buffer is Empty
  Returns one if the buffer is not empty
*/






#endif
