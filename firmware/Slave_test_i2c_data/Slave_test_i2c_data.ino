#include "Arduino.h"
#include <string.h>
#include <stdio.h>
#include <Wire.h>
#include "I2C_Anything.h"

#define CTRL_I2C_ADDR 12
#define READ_LIM 10       //retries on read collision

struct __attribute__ ((packed)) ctrlcomdata {
  char from='X';    //NZS axis (X,Y,E...)
  char to='M';
  char action='e';//e: absolute max error stream, i: motor current setting (mA),h: hold current setting (mA),c: calibrate
  int32_t data;
};

ctrlcomdata data_from_Master;
ctrlcomdata data_to_Master;

volatile bool data_ready=false;
volatile uint32_t ok1=0;
volatile uint32_t ok2=0;
String inputString = "";                      // a string to hold incoming data
boolean stringComplete = false;               // whether the string is complete

void setup()
{
//  pinMode(12, OUTPUT);                        //set pin for 12V power supply
//  digitalWrite(12, LOW);                      //enable 12V power supply
	SerialUSB.begin(115200);                       // start serial for output
  inputString.reserve(2);
	while (!SerialUSB) {
	; // wait for serial port to connect. Needed for native USB
	}
	SerialUSB.println ("Slave connected");
	Wire.begin(CTRL_I2C_ADDR);        // join i2c bus
	Wire.onReceive(receiveEvent);
  Wire.onRequest (requestEvent);
}

void loop(){
  if(data_ready){
//    Serial.println (data_from_Master.id);
//    Serial.println (data_from_Master.action);
//    Serial.println (data_from_Master.data);
//Do something with received data here

      //Check if valid data is transmitted (only for testing purpose)
      if(data_from_Master.from=='M'&&data_from_Master.action=='e')
      {
        ok1++;
        //data_to_Slave2=data_from_Master;
      // }else if(data_from_Master.id=='Y'&&data_from_Master.action=='e'&&data_from_Master.data==2000)
      // {
        // ok2++;
      }
    data_ready = false;
  }
  
  //report current transfer results when typing "r" + "return" keys in serial window
  if (stringComplete) {
    if(inputString=="r\r"){
      SerialUSB.print(ok1);
      SerialUSB.println (" Valid transfers from m1");
      // Serial.print(ok2);
      // Serial.println (" Valid transfers from m2"); 
//      Serial.println (data_from_Master.id);
//      Serial.println (data_from_Master.action);
//      Serial.println (data_from_Master.data);    
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

void receiveEvent(int howMany) {
  if(howMany==sizeof(ctrlcomdata)){           //check if the right data size is returned (filters out Wire library's misinterpreted requests)
    I2C_readAnything(data_from_Master);
    data_ready = true;
  }
}

void requestEvent(){
  static uint32_t t=0;
  //set data for Master 1
  data_to_Master.data = 2500;
  data_to_Master.action = 'Z';
  I2C_singleWriteAnything(data_to_Master);
//  Wire.write ((uint8_t*) &data_to_Master, sizeof(ctrlcomdata));
  //set data for Master 2
  // data_to_Master.data = 3500;
  // data_to_Master.action = 'Y';
  // I2C_singleWriteAnything(data_to_Master);
//  Wire.write ((uint8_t*) &data_to_Master, sizeof(ctrlcomdata));
//we don't know which master is requesting. Sending data fo both.
}

//get serial commands
void serialEvent() {
  while (SerialUSB.available()) {
    // get the new byte:
    char inChar = (char)SerialUSB.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\r') {
      stringComplete = true;
    }
  }
}

//serial event for SAMD
void serialEventRun(void){
  if(SerialUSB.available()) serialEvent();
}
