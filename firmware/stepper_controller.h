#ifndef __STEPPER_CONTROLLER_H__
#define __STEPPER_CONTROLLER_H__

#include "syslog.h"
#include "board.h"
#include "as5047d.h"
#include "calibration.h"
#include "A4954.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


class StepperCtrl 
{
  private:
	bool enableFeedback; //true if we are using PID control algorithm
    AS5047D encoder;
    A4954 stepperDriver;
    Adafruit_SSD1306 display;

    int32_t numMicroSteps=16;
    int32_t fullStepsPerRotation=200;


    int64_t numSteps; //this is the number of steps we have taken from our start angle


    int32_t currentLocation; //estimate of the current location from encoder feedback
    // the current location lower 16 bits is angle (0-360 degrees in 65536 steps) while upper
    // bits is the number of full rotations.

    //all angles are measured as 16 bit values ie degrees per bit is 360/(2^16)
    int32_t degreesPerFullStep=0;  //number of degrees per step for stepper motor
    							   //  this is estimated by the controller.


    bool forwardWiring; //determines if the wiring is for forward rotation or reverse
                        // This way forward motion is always clockwise.


    //does linear interpolation of the encoder calibration table
    int32_t getAngleCalibration(int32_t encoderAngle);

    //updates the currentMeasuredAngle with our best guess where we are 
    Angle sampleAngle(void);
    Angle sampleMeanEncoder(uint32_t numSamples);
    
    uint16_t measureStepSize(void); //steps motor and estimates step size
    uint32_t measureMaxCalibrationError(void);
    void setLocationFromEncoder(void);
    int64_t getDesiredLocation(void);
    void UpdateLcd(void);
    void  motorReset(void);
  public:

    CalibrationTable calTable;

    bool calibrateEncoder(void); //do manual calibration of the encoder
    Angle maxCalibrationError(void); //measures the maximum calibration error as an angle

    void moveToAngle(int32_t a, uint32_t ma);

    int begin(void);
    bool process(void);
    void requestStep(int dir, uint16_t steps); //requests a step, if feedback controller is off motor does not move


    int measure(void);
    int32_t measureMeanEncoder(void);
    

    bool changeMicrostep(uint16_t microSteps);
    void feedback(bool enable);
    bool getFeedback(void) {return enableFeedback;}

    void encoderDiagnostics(char *ptrStr);
    int32_t getMicroSteps(void) {return numMicroSteps;}
    int32_t measureError(void);
    void testRinging(void);
    int32_t getCurrentLocation(void);

    void move(int dir, uint16_t steps); //forces motor to move even if feedback controller is turned off.

};

#endif //__STEPPER_CONTROLLER_H__

