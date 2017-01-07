# Smart Stepper (also known as the nano zero stepper)



Uses feedback to control stepper motor. 

License

All nano stepper related materials are released under the Creative Commons Attribution Share-Alike 4.0 License
Much of the work is based on Mechaduino project:
https://github.com/jcchurch13/Mechaduino-Firmware

If you want to support the work on the firmware and hardware consider buying hardware from www.misfittech.net or buying me a beer using the donation button. 

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=4JAEK4G24W2U4)

[Hardware install](http://misfittech.net/2016/11/29/installing-the-nano-zero-stepper/)

# Command List

## Smart Stepper Command Line Interface
The smart stepper uses a command line interface where the prompt is “:>” 

### help 
The help command will return a list of commands that the smart stepper supports. 

### getcal
This command will print out the 200 point calibration table.  This is useful if you are doing firmware development and do not want to calibrate each time you update firmware.  You can take this table and copy it into the nonvolatile.cpp file as shown below:
```//we use this so we can hard code calibration table
// be sure to set the last word as status flag
// this save time calibrating each time we do a code build
#ifdef NZS_FAST_CAL
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[16640]={  //allocates 33280 bytes
#else
__attribute__((__aligned__(FLASH_ROW_SIZE))) const uint16_t NVM_flash[256]={  //allocates 512 bytes
#endif
		//insert the getcal command calibration data here		22064,22378,22734,23065,23433,23756,24120,24456,24827,25150,25512,25841,26203,26520,26869,27191,27543,27853,28197,28513,28861,29169,29508,29823,30172,30477,30819,31136,31483,31790,32131,32447,32794,33099,33439,33755,34102,34409,34748,35061,35405,35709,36046,36357,36701,37005,37341,37655,37999,38302,38639,38952,39295,39599,39938,40251,40598,40906,41250,41566,41921,42230,42575,42896,43247,43557,43902,44223,44575,44881,45225,45541,45889,46192,46530,46843,47187,47489,47824,48138,48479,48783,49122,49436,49783,50088,50427,50739,51083,51384,51719,52028,52366,52668,53003,53314,53660,53965,54310,54626,54981,55292,55640,55966,56324,56640,56992,57319,57681,57998,58352,58685,59051,59371,59732,60062,60424,60739,61089,61408,61759,62066,62409,62725,63074,63381,63725,64042,64394,64701,65043,65357,167,467,803,1111,1450,1751,2087,2399,2745,3049,3392,3711,4063,4372,4720,5042,5396,5708,6059,6384,6744,7055,7407,7728,8081,8387,8727,9036,9375,9670,10000,10302,10635,10930,11255,11559,11892,12185,12513,12814,13147,13438,13762,14060,14389,14675,14996,15294,15621,15908,16228,16529,16858,17148,17478,17785,18124,18425,18762,19077,19425,19730,20073,20390,20739,21049,21393,21714,

		0xFFFF //for the valid flag

};
```

### calibrate
This will run a 200 point calibration of the encoder. 

### testcal 
This will test the calibration and report the maximum error in degrees. 

### step
This will move the motor one step clockwise, the step size is based on the current microstep setting.  To move the motor counterclockwise use “step 1”. To move the motor clockwise 16 steps used “step 0 16” to move motor counterclockwise 16 steps use “step 1 16”

### feedback 
This commands disable/enables feedback control loop. 
The plan is to discountinue this command in the future and use the “controlmode” command to put controller in open or one of the many closed loop operational modes. 

### readpos
Reads the current motor position and reports it as degrees.

### encoderdiag
This command will read and report the AS5047D internal registers for diagnostic purposes. 

### microsteps 
This command gets/sets the number of microsteps the smart stepper will use for the step command and the step pin.  The number of microsteps does not affect the resolution of the controller but rather how fine you can set the position. 

### spid
This command sets the Kp, Ki, and Kd terms for the simple positional PID controller. 

### ppid
This command sets the Kp, Ki, and Kd terms for the positional PID controller. 

### vpid
This command sets the Kp, Ki, and Kd terms for the velocity PID controller. 

### velocity 
This sets the velocity to rotate motor when unit is configured for velocity PID mode of operation. 

### boot
This command will put the microprocessor in the boot loader mode.  Alternatively this can be done by double pressing the reset button. 

### factoryreset 
This erases the calibration and other system and motor parameters such that unit is reset to the factory ship state.  After this command the unit will need to be calibrated to the motor again. 

### dirpin
This command sets which direction the motor will rotate when direction pin is pulled high. The direction pin is only sampled when the step pin has a rising edge. 
‘dirpin 0’ will set the motor to rotate clockwise when dir pin is high
‘dirpin 1’ will set the motor to rotate counter-clockwise when dir pin is high

### errorlimit
Gets set the maximum number of degrees of error that is acceptable, any posistioning error about the error limit will assert the error pin, when error pin is set as error output. 
For example:
:>errorlimit 1.8 
Will set the error limit to 1.8 degrees. 

### ctrlmode
Gets/Sets the feedback controller mode of operation. The command takes an integer from 0 through 4 to set the control mode per table below:
Controller off - 	0  -- this is not currently used
Open-Loop - 	1  -- this is open loop with no feedback
Simple PID -   2  -- simple positional PID, which is factory default 
Positional PID - 3 -- current based PID mode, requires tuning for your machine
Velocity PID - 4 -- velocity based PID, requires tuning for your machine and speed range

If you are unsure what you are doing leave unit in the Simple PID mode of operation. 

### maxcurrent
This sets the maximum current that will be pushed into the motor. To set the current for maximum of 2.0A you would use command “maxcurrent 2000” as the argument is in milliAmps. 

### holdcurrent
For the Simple Positional PID mode the minimal current (ie current with no positional error) is the hold current. You set this current based on the required holding torque you need for your application. The higher the hold current, the hotter and noisier the motor will be but also the larger the holding torque. 
For the Positional PID mode the PID tuning params have to be set correctly such that the control loop will dynamically determine the holding torque. This tuning of the PID can be difficult, hence the simple PID mode will work most of the time out of the box by setting maximum current and holding current. 

### motorwiring
The firmware always uses a positive angle as a clockwise rotation. A stepper motor however could have wiring done with one coil reversed wired, which will cause motor to normally operate in opposite direction. The Smart Stepper firmware will detect the motor wiring direction, using the encoder, and the firmware will compensate for a reverse wired motor.  The reverse or forward wiring of a motor is detected on first power up after factory reset. If the wiring changes after that you can compensate using this command. 
HOWEVER  it is better to do a factory reset and recalibrate motor if wiring changes. 

### stepsperrotation
The Smart Stepper firmware will with first power on after factory reset detect the number of full steps per rotation for the stepper motor and store in flash memory. This command will read this parameter from flash and allow user to change this parameter if motor is changed.
HOWEVER  it is better to do a factory reset and recalibrate motor if motorchanges. 

### move
The move command will request the motor to move to an absolute angle position.  Optionally the user can specify the rotational speed (RPMs) by which the move should happen. For example if the current motor position is at angle 0 and you issue ‘move 3600” the motor will turn 10 rotations clockwise to angle of 3600 degrees. If issue the ‘move 3600’ again nothing will happen as motor is already at angle 3600. 
If motor is at angle 0 and user issues the command ‘move 3600 20’ then motor will move to 10 rotations clockwise to angle of 3600 at a rate of 20 RPMs. 

### stop
If user issues a move command that takes a long time and wants to stop the move before completion then user can issue the stop command command which will stop a move operation. 

### setzero
This command will take the current motor position and set it to absolute angle of  zero. Note that if you are in the middle move it will take the position at the time of the command and use it, thus it is recommend a move be stopped or wait for completion before issuing the setzero. 



