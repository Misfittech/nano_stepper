#ifndef __STEPPER_CONTROLLER_H__
#define __STEPPER_CONTROLLER_H__

#include "syslog.h"
#include "board.h"
#include "as5047d.h"
#include "calibration.h"
#include "A4954.h"

#ifndef MECHADUINO_HARDWARE
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif 

#define N_DATA (1024)

#define DISPLAY_TEXT_SIZE 1
#define DISPLAY_LINE_HEIGHT 10*DISPLAY_TEXT_SIZE

typedef enum {
	CTRL_OFF =0,
	CTRL_SIMPLE = 1, //simple error controller
	CTRL_POS_PID =2, //PID  Position controller
} feedbackCtrl_t;


typedef struct {
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
} PID_t;

//this scales the PID parameters from Flash to floating point
// to fixed point int32_t values
#define CTRL_PID_SCALING (1024)

class StepperCtrl 
{
  private:
	volatile bool enableFeedback; //true if we are using PID control algorithm
    AS5047D encoder;
    A4954 stepperDriver;
#ifndef MECHADUINO_HARDWARE //mech does not have LCD
    Adafruit_SSD1306 display;
#endif
    bool currentLimit;
    volatile bool enabled;

    volatile feedbackCtrl_t controlType=CTRL_SIMPLE;

    volatile int32_t loopTimeus; //time to run loop in microseconds

    volatile int32_t numMicroSteps=16;
    volatile int32_t fullStepsPerRotation=200;

    volatile PID_t sPID;
    volatile PID_t pPID;
    volatile PID_t vPID;

    volatile int64_t numSteps; //this is the number of steps we have taken from our start angle


    volatile int32_t currentLocation; //estimate of the current location from encoder feedback
    // the current location lower 16 bits is angle (0-360 degrees in 65536 steps) while upper
    // bits is the number of full rotations.

    //all angles are measured as 16 bit values ie degrees per bit is 360/(2^16)
    int32_t degreesPerFullStep=0;  //number of degrees per step for stepper motor
    							   //  this is estimated by the controller.


    bool forwardWiring; //determines if the wiring is for forward rotation or reverse
                        // This way forward motion is always clockwise.


    volatile int16_t data[N_DATA];

    //does linear interpolation of the encoder calibration table
    int32_t getAngleCalibration(int32_t encoderAngle);

    //updates the currentMeasuredAngle with our best guess where we are 
    Angle sampleAngle(void);
    Angle sampleMeanEncoder(uint32_t numSamples);
    
    uint16_t measureStepSize(void); //steps motor and estimates step size
    uint32_t measureMaxCalibrationError(void);
    void setLocationFromEncoder(void);
    int64_t getDesiredLocation(void);

    void  motorReset(void);
    void updateStep(int dir, uint16_t steps);

    //these are LCD functions
    void showSplash(void);
    void menu(void);
    void showCalError(void);
    int32_t measureMeanEncoder(void);
    int32_t getCurrentLocation(void);
    bool pidFeedback(void);
    bool simpleFeedback(void);
    void LCDShow(char *str);

  public:
    void updatePIDs(void);
    CalibrationTable calTable;
    void printData(void);

    bool calibrateEncoder(void); //do manual calibration of the encoder
    Angle maxCalibrationError(void); //measures the maximum calibration error as an angle

    void moveToAbsAngle(int32_t a);
    void moveToAngle(int32_t a, uint32_t ma);

    int begin(void);

    bool processFeedback(void); // does the feedback loop

    void setControlMode(feedbackCtrl_t mode);
    feedbackCtrl_t getControlMode(void) { return controlType;};

    void requestStep(int dir, uint16_t steps); //requests a step, if feedback controller is off motor does not move


    int measure(void);
    


    bool changeMicrostep(uint16_t microSteps);
    void feedback(bool enable);
    bool getFeedback(void) {return enableFeedback;}

    void encoderDiagnostics(char *ptrStr);
    int32_t getMicroSteps(void) {return numMicroSteps;}
    int32_t measureError(void);
    void testRinging(void);

    float getCurrentAngle(void);

    void move(int dir, uint16_t steps); //forces motor to move even if feedback controller is turned off.
    void UpdateLcd(void); //this can be called from outside class
    void enable(bool enable);

    int32_t getLoopTime(void) { return loopTimeus;}
    void PID_Autotune(void);

};

#endif //__STEPPER_CONTROLLER_H__

