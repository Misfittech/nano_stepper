#include "I2C_com.h"
#include <string.h>
#include <stdio.h>
#include <Wire.h>
#include "I2C_Anything.h"

uint16_t read_cnt1 = 0;
uint16_t read_cnt = 0;
uint8_t status;
volatile bool data_ready=false;
volatile uint32_t ok1=0;

int32_t i2c_com::geterr(void){
	int32_t err = abs(ptrStepperCtrl->getLoopError());
	if (err > maxerr)
	{
		maxerr = err;
	}
	return maxerr;
}

void i2c_com::begin(void){
	Wire.begin(NZS_ADDRESS);        // join i2c bus
	Wire.onReceive(receiveEvent);
  Wire.onRequest (requestEvent);
}

void i2c_com::setup(StepperCtrl *ptrsCtrl){
	ptrStepperCtrl=ptrsCtrl;
}

void i2c_com::process(void){//call this in loop
	data_to_master.data = geterr();
	communicate(data_to_master);
}
void i2c_com::process(i2c_com_data data){
	communicate(data);
}


void i2c_com::communicate(i2c_com_data data){
  if(data_ready){
//    SerialUSB.println (data_from_master.from);
//    SerialUSB.println (data_from_master.to);
//    SerialUSB.println (data_from_master.action);
//    SerialUSB.println (data_from_master.data);
//    SerialUSB.println (data_from_master.from=='M');
//    SerialUSB.println (data_from_master.to==NZS_LABEL);
//    SerialUSB.println (data_from_master.action=='e');
//Do something with received data here

      //Check if valid data is transmitted (only for testing purpose)
      if(data_from_master.from=='M'&&data_from_master.to==NZS_LABEL&&data_from_master.action=='e')
      {
        ok1++;
//        SerialUSB.print (ok1);
//        SerialUSB.println (" ok1");
      // }else if(data_from_master.from=='Y'&&data_from_master.action=='e'&&data_from_master.data==2000)
      // {
        // ok2++;
      }
    data_ready = false;
  }
  
  //report current transfer results
  if(print_) {
    LOG("%d Valid transfers from m1",ok1);
    LOG("from %c",data_from_master.from);
    LOG("to %c",data_from_master.to);
    LOG("action %c",data_from_master.action);
    LOG("data %d",data_from_master.data);
    // Serial.print(ok1);
    // Serial.println (" Valid transfers from m1");
    // Serial.println (data_from_master.from);
    // Serial.println (data_from_master.to);
    // Serial.println (data_from_master.action);
    // Serial.println (data_from_master.data);    

    print_ = false;
  }
}

// void i2c_com::run_cmd_from_slave(void){
	// if(data_from_master.action=='t')//test time tick transfer
	// {
		// Serial.println(data_from_master.from);
		// Serial.println(data_from_master.to);
		// Serial.println(data_from_master.action);
		// Serial.println(data_from_master.data);
	// }else if(data_from_master.action=='c')
	// {
		// ptrStepperCtrl->calibrateEncoder();
	// }else{
		// Serial.println(data_from_master.from);
		// Serial.println(data_from_master.to);
		// Serial.println(data_from_master.action);
		// Serial.println(data_from_master.data);
	// }
// }

class i2c_com i2c_com;

void receiveEvent(int howMany) {
  if(howMany==sizeof(i2c_com_data)){           //check if the right data size is returned (filters out Wire library's misinterpreted requests)
    I2C_readAnything(i2c_com.data_from_master);
    data_ready = true;
  }
}

void requestEvent(){
  //set data for master 1
  i2c_com.data_to_master.data = 2500;
  i2c_com.data_to_master.action = 'M';
  I2C_singleWriteAnything(i2c_com.data_to_master);
}
