// Written by Nick Gammon
// May 2012

#include <Arduino.h>
#include <Wire.h>

template <typename T> unsigned int I2C_writeAnything (const T& value)
  {
  Wire.write((byte *) &value, sizeof (value));
  return sizeof (value);
  }  // end of I2C_writeAnything

template <typename T> unsigned int I2C_readAnything(T& value)
  {
    byte * p = (byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
          *p++ = Wire.read();
    return i;
  }
  
template <typename T> int I2C_singleWriteAnything (const T& value) {
  int size = sizeof value;
  byte vals[size];
  const byte* p = (const byte*) &value;
  unsigned int i;
  for (i = 0; i < sizeof value; i++) {
    vals[i] = *p++;
  }
 
  Wire.write(vals, size);
  return i;
}
