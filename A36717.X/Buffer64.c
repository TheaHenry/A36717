#include "Buffer64.h"

void Buffer64WriteByte(BUFFER64BYTE* ptr, unsigned char value) {
  ptr->data[ptr->write_location] = value;
  ptr->write_location += 1;
  ptr->write_location &= Buffer64Mask;
  if (ptr->write_location == ptr->read_location) {
    ptr->read_location += 1;
    ptr->read_location &= Buffer64Mask;
  }
}

unsigned char Buffer64ReadByte(BUFFER64BYTE* ptr) {
  unsigned char local_read_location;
  unsigned char return_data;
						
  local_read_location = ptr->read_location;
  if (local_read_location != ptr->write_location) {
    // the buffer is not empty
    return_data = ptr->data[local_read_location];
    local_read_location += 1;
    local_read_location &= Buffer64Mask; 
    ptr->read_location = local_read_location;
  } else {
    // the buffer was empty
    // return zero and do not increment the read_location
    return_data = 0;
  }
  return return_data;
}

unsigned char Buffer64BytesInBuffer(BUFFER64BYTE* ptr) {
  return ((ptr->write_location - ptr->read_location) & Buffer64Mask);
}

unsigned char Buffer64IsNotEmpty(BUFFER64BYTE* ptr) {
  if (ptr->write_location == ptr->read_location) {
    return 0;
  } else {
    return 1;
  }
}



