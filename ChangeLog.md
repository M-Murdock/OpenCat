# Change Log

## Oct 26, 2022
* Move the detailed initializing functions in **OpenCat.ino**'s setup() to OpenCat.h. 
* Add a simple example to elaborate the integration of customized sensors. You need to enable "#define OTHER_MODULES" to see its effects. 

## Aug 31, 2022
* Add a watchdog to reset the board if the initialization for MPU or skills fails.
* At the last stage of the first configuration process (with #define MAIN_SKETCH commented out), provide an optional step to calibrate the PCA9685's PWM frequency. The calibration value is saved in the EEPROM table to be loaded by the main sketch. 
* Change the unit of the duration for music notes ('b' and 'B' token) from duration (miliseconds) to half notes (use 2, 4, 8 to represent 1/2, 1/4, 1/8).
* For Bittle: Add front flip, roll left, roll right, and tune back flip. It's better to remove its head to perform those tricks. 
* Shrink the size of codes for getting and saving MPU offsets.
* Won't repeat the last behavior after roll recover.
* ESP8266WiFiController: won't refresh the page after sending command to save time
* ESP8266WiFiController: Simplify the cases on the main command page with a dictionary. 
* Add some new moduleTest codes

## May 8, 2022
* pyUI/ui.py: Add names to trigger axis
* pyUI/ui.py: Optimize the layout for Windows machines
* pyUI/ui.py: Add add hover tips to some buttons

## May 4, 2022
* pyUI/ui.py: Import and export files

## May 2, 2022
* pyUI/ui.py: Improve performance with large angles
* pyUI/ui.py: Show connected ports
* pyUI/ui.py: Improve logic and workflow for connecting multiple robots
* pyUI/ui.py: Support connect to one or all robots dynamically
* pyUI/ui.py: Improve global orientation and translation sliders
* pyUI/ui.py: Change dial buttons from regular buttons to press-down buttons

## Apr 30, 2022
* pyUI/ui.py: Mirror single frame during editing
* pyUI/ui.py: Fix bug in the loop structure when exporting 

## Apr 29, 2022
* pyUI/ui.py: Add binders (both positive and negative) for joint sliders to control several joints together.
* pyUI/ui.py: Can control multiple robots at once. 
* ardSerial: Get return value from threads.

## Apr 24, 2022
* pyUI/ui.py: Support loading and sending large angles
* pyUI/ui.py: Optimize panel sizes

## Apr 22, 2022
* pyUI/ui.py: Support selecting different robot models. Update the center image for the model. 
* pyUI/ui.py: Support different lanuguages.
* pyUI/ui.py: Support importing and exporting a multi-frame skill as a behavior or gait. A single frame will always be exported as a posture.
* pyUI/ui.py: Update the skill frame's state for delete operations.
* pyUI/ui.py: Check the format of skill data that's manually entered. Pop up a warning window.  

## Apr 20, 2022
* pyUI/ui.py: Support trigger axis and angles for behaviors
* pyUI/ui.py: Allow stop when playing frames
* Add a button to connect and disconnect the robot in the "State Dials" block
* Change the texts of the state dials to indicate their states

## Apr 19, 2022
* pyUI/ui.py: Support importing existing skill array and editing them frame by frame. Speed and delay are included. The trigger axis of behaviors has not been implemented.
* pyUI/ui.py: Export the skill to the robot and print the skill data array in the terminal. 

## Apr 6, 2022
* Add ui.py in pyUI to allow changing the robot's postures and body tilts with sliders. 
* Improve the structure of ardSerial.py to allow continuous inputs when it's used as a commandline tool. 
* Fix a bug when a static posture is called after a gait by resetting the frame to 0.

## Mar 29, 2022
* Use dataBuffer to hold the incoming serial commands to allow a longer melody and other instructions. newCmd will only be used to hold skill names that are shorter. 
* Save the serial skill data from the dataBuffer to I2C EEPROM so that it can be recalled. 

## Mar 20, 2022
* A schedulerToSkill function in serialMaster to summerize the motion commands and generate a new skill array that can be directly used in the instinct.h.
* Design a convenient macro switch for regular walker, ultrasonic, and voice modes. 
```cpp
//you can activate the following modes (they will diable the gyro to save programming space)
//allowed combinations: RANDOM_MIND, ULTRASONIC, RANDOM_MIND + ULTRASONIC, VOICE
//#define RANDOM_MIND     //let the robot do random stuffs. use token 'z' to activate/deactivate
//#define ULTRASONIC      //for Nybble's ultrasonic sensor
//#define VOICE           //for LD3320 module
```

## Mar 17, 2022
### OpenCat2.0 is pushed to the branch 2.0 for beta testers.
### New Features:
* **Complete redesign of the code structure to make it easier to understand and insert 3rd party codes.**
* **Combine WriteInstinct.ino and OpenCat.ino into a single OpenCat.ino.** Use the ```#define MAIN_SKETCH``` macro to switch between the modes. 
* Fixed many bugs in the original workflow. For example, the robot no longer skips commands randomly.
* Improved the smoothness of motion and the speed of balance adaption. 
* A better documented serialMaster Python module and provide a command scheduler in example.py. 
* **Designd a new token "K"** to send a whole skill array from the Python serial master through serial in realtime then run it locally on the robot.
* Optimized the token definitions. Print the token after finishing one task as a confirmation to the master program. 
* Used a new MPU6050 function to avoid using the interrupt pin. **Accelerate the IMU calibration process.** 
* Enter the joint calibration state by booting up the robot with one side up. 
* Write a formalized servo class inherited from the AdafruitPWM servo library. Make it easier to change parameters of 3rd party servos.
* **Use the built-in tone function to generate clean beeps and melody.**
* Add random actions for the robot to make it more life-like. Use token ‘z’ to turn on/off the feature. 
* Add angle limits to avoid most cases of of self-collision.
* Put the skill-related functions into the Skill Class. Combine reused code blocks into single functions.
* Make more use of the EEPROM and PRGMEM memory to save resources for flash and SRAM.
* Mirror rightward gaits from the leftward gaits, to save static memory. 
* Allow both “newline” and “no line ending” for the serial monitor.
* Add support for LD3320 voice recognition module (the current code works for Chinese).

## Jan 30, 2022
* Fix type conversion warnings of skill data.
* Add demo for testBuzzer.ino using Arduino's built-in tone function. However, it cannot be used for the main program because ATmega328P has only two timers, used up by the IR and IMU. 

## Jan 11, 2022
* Create a mirror function to allow the robot pick random directions of behaviors. It makes the robot more unpredictable and helps it get rid of an infinite loop, such as failed fall-recovering against a wall.  

## Jan 3, 2022
* Put the dependent libraries within the OpenCat folder so that no more downloading is required to configure OpenCat's environment in Arduino. 

It's nasty to include the **src/** libraries here rather than Arduino's desinated library folder, especially when we have to duplicate the **src/** in two layers within the same project folder.  

We do this only to help those users who are new to Arduino and complained about the difficulties of configuring the Arduino environment. You may find more informative comments within those library files. 


## Dec 29, 2021
* Organinzed serialMaster/ and its demo codes
* fixed the divide by zero bug in calibratedPWM()

## Dec 11, 2021
* Fix bug of "wrong key!" messages after g, p

## Dec 3, 2021
* Modified the "hi" behavior of Nybble so its tail will not push Nybble over;
* Replaced the "look up" behavior with "stand". Make sure you hold Nybble when first trying it. 

## Nov 28, 2021
* Add the march gait that only works well with gyro off; 
* Fix many typos with the help of Grammarly.

## Sep 29, 2021
* Print the robot's state (paused and gyro) when "g" and "p" is entered:

| State  | True  | False |
|:----------|:----------|:----------|
| Pause    | P (paused)    | p (unpaused)    |
| Gyro   | G (on)    | g (off)   |

## Sep 26, 2021
* Move the codes for EEPROM to function configureEEPROM() in OpenCat.h;

## Sep 18, 2021
* Copied the required files of IRremote library to OpenCat/src/ to help new Arduino users config the environment. 
* Cancel "pause" of motion if other commands are received. 


## Sep 16, 2021
* Move the EEPROM related functions from WriteInstinct.ino to OpenCat.h.
* Added a "stand" behavior for Nybble so that it can stand up with its hind legs and tail. 

## Sep 9, 2021
* Print the list of calibration offsets before the joints' movements to help the App read the values.


## Aug 31, 2021
* Moved the device's model info (Nybble/Bittle) to the top of booting printouts.
* Added a shorter encoding for the IR remote to save flash by about 178 Bytes.

## Aug 25, 2021
* Added an 'M' token to move multiple indexed joints to angles (ASCII format entered in the serial monitor) simultaneously;
* Re-arranged the IR keymap for the new customized IR panel.

## Jun 29, 2021
* OpenCat.ino will print out the model name (Bittle/Nybble) at booting up.
* Moved #define I2C_EEPROM to the beginning OpenCat.h.

## Jun 7, 2021

* Adjusted the threshold voltage so that Bittle will keep beeping when the battery goes under 6.5V, a few moments before the battery power shuts off;
* Adjusted the gaits so that the elbow won't hit the body in the accelerated phase. 
* The trot on key "1" is tuned to be faster.
* Removed the running gait on the IR remote key "2" and replaced it with "check around" behavior previously on key "3";
* Assigned a "play dead" trick to IR remote key "3". Bittle will fall on its back then roll back (if the gyro is activated);
* Removed the behavior when the robot is tilted at a large angle to avoid an occasional bug. Will put the behavior back if the bug can be fixed; 
* Added an auto-detection code for a new sound&light sensor connected to the analog Grove pin. Some new automated behaviors are being developed. The code block won't be active if the sensor is not connected to the Analog Grove socket;


