#ifndef USB_NZS_DATASTREAM_H
#define USB_NZS_DATASTREAM_H
#include "Arduino.h"
#include "syslog.h"
#include "board.h"
#include "stepper_controller.h"

struct __attribute__ ((packed)) motordata {
  int32_t error;
  int64_t angle;
  unsigned long Time;
};

class USB_stream
{
	private:
		StepperCtrl *ptrStepperCtrl;
		motordata getdata(void);
		void send(motordata);
		motordata mdata;

	public:
    bool on=false;
		void setup(StepperCtrl *ptrStepperCtrl); //sets up the USB stream
		void begin(void);
		void process(void);
		void process(motordata);
};

#endif
