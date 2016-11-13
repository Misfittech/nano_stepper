#ifndef __AS5047D_H__
#define __AS5047D_H__

#include <Arduino.h>
#define AS5047D_DEGREES_PER_BIT  (360.0/(float)(0x3FFF))

class AS5047D {
  private:
    int chipSelectPin;
    int16_t readAddress(uint16_t addr);
    
  public:
    boolean begin(int csPin);
    int16_t readEncoderAngle(void);
    void diagnostics(char *ptrStr);
    int16_t readEncoderAnglePipeLineRead(void);
};

#endif //__AS5047D_H__
