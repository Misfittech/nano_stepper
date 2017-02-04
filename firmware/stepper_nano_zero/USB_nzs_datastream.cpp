#include "USB_nzs_datastream.h"
#include <string.h>
#include <stdio.h>

motordata USB_stream::getdata(void){
//	int32_t err=ptrStepperCtrl->getLoopError();
//	int32_t deg=ptrStepperCtrl->getCurrentLocation();
	mdata={ptrStepperCtrl->getLoopError(),
			ptrStepperCtrl->getDesiredSteps(),
			millis()
	};
	return mdata;
}
void USB_stream::send(motordata err){
	static uint32_t tusb=0;
  //delay(2);
	if ((millis()-tusb)>2)
	{
		tusb=millis();
		Serial.write((uint8_t*) &err, sizeof(motordata));
	}
}


void USB_stream::setup(StepperCtrl *ptrsCtrl){
	ptrStepperCtrl=ptrsCtrl;
}

void USB_stream::process(void){
	motordata err=getdata();
	send(err);
}
void USB_stream::process(motordata err){
	send(err);
}
void USB_stream::begin(void){
	SerialUSB.end();
	Serial.begin(115200);//restart USB connection, to let user start data ploting app
//	while(!SerialUSB){
//		;
//	}
}

class USB_stream USB_stream;
