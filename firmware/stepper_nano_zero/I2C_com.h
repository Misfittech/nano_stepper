#ifndef I2C_COM_H
#define I2C_COM_H
#include "Arduino.h"
#include "syslog.h"
#include "board.h"
#include "stepper_controller.h"

#define NZS_CNT 1                 //number of NZS (masters) on the bus
#define SEND_INTERVAL 2000
#define GET_INTERVAL 500
#define READ_LIM 100              //retries on read collision
#define I2C_CLOCK 100000
//set NZS_LABEL in board.h

struct __attribute__ ((packed)) i2c_com_data {
	char from=NZS_LABEL;		//NZS axis (X,Y,E...)
	char to='M';
	char action='e';//e: absolute max error stream, i: motor current setting (mA),h: hold current setting (mA),c: calibrate
	int32_t data;
};

class i2c_com
{
	private:
		StepperCtrl *ptrStepperCtrl;
		int32_t maxerr;
		int32_t geterr(void);
		void communicate(i2c_com_data);
		// void run_cmd_from_slave(void);

	public:
    bool on=false;
    bool print_=false;
		void setup(StepperCtrl *ptrStepperCtrl); //sets up the USB stream
		void begin(void);
		void process(void);
		void process(i2c_com_data);
    i2c_com_data data_to_master;
    i2c_com_data data_from_master;
};

void receiveEvent(int howMany);
void requestEvent();
#endif
