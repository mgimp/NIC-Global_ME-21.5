/*
BRIEF EXPLANATION FOR CLASSMATES
I made this file so that we could construct the statemachine from the ground up rather than editing Greg's files.
This ought to give us a greater understanding of the structure of the program, even if we end up copy/pasting
a significant amount of the code.

Seattle University
    Mechanical Engineering Department
        Engineering Design Group 21.5: NIC Global
            Vision System for Silkscreen Inspection
        David Schulman, Linh Ngo, Matthew Gimpelevich, Robert Wheeler
27FEB2021

THIS CODE is meant to service the Vision System for Silkscreen Inspection as designed by group 21.5
It uses a basic state machine to control a camera, gantry systems, limit switch, and operating consol using an Arduino Mega and the SpeedyStepper library Copyright (c) 2014 Stanley Reifel & Co.
Camera: 
Actuator Model: FUYU FSL Series
Stepper Motor: Nema 23
Brain: Arduino Mega
*/

// -----EQUIPMENT NOTES-----
// Limit switches probably fail LOW
// xgantry is 350mm
// ygantry is 900mm
// wiper is 500mm
// tray is  1000mm

// -----MOVEMENT NOTES-----
// NEGATIVE VALUES = TOWARD MOTOR
// POSITIVE VALUES = AWAY FROM MOTOR

// -----LIBRARIES-----
// SETUP STEPPER MOTOR AND ARDUINO VARIABLES
#include "SpeedyStepper.h"
#include "Arduino.h"

// -----DEFINITIONS-----
// These definitions are used in place of pin numbers throughout the code
// The usefulness comes from setting all of the necessary pin numbers in a single place in the code
// definitions are not designed to be updated in the code

// digital IO pins
// DO = digital output
// DI = digital input

// DIR = direction
// PUL = pulse
#define DO_XGANTRY_DIR   ?
#define DO_XGANTRY_PUL   ?
#define DO_YGANTRY_DIR   ?
#define DO_YGANTRY_PUL   ?
#define DO_WIPER_DIR   ?
#define DO_WIPER_PUL   ?
#define DO_TRAY_DIR   ?
#define DO_TRAY_PUL  ?

// home switch should be setup so there are alway HIGH unless triggered
// this provide a better default in case the wiring fails
// HOME should always be at the motor side of the actuator
#define DI_HOME_XGANTRY  ?
#define DI_HOME_YGANTRY  ?
#define DI_HOME_WIPER  ?
#define DI_HOME_TRAY  ?   

#define DI_EMERGENCYSTOP ? // a locking switch that stops all processes
#define DI_START ?   // a momentary switch to start the inspection

// Lights
#define DO_Ready ?      //Green LED on for ready status
#define DO_WIP ?        //Yellow LED blinking for cycle in progress
#define DO_Part_Failure ?  //ORANGE LED Blinking for part failure
#define DO_System_Failure ? //RED LED Steady for system failure

// camera
#define DO_CAM_TAKEPICTURE ?
#define DI_CAM_GOTPICTURE ?
#define DI_CAM_FAILED ?


#define NPOS 8   // number of points the gantry must move to capture the pictures
#define Y 0    // the long gantry direction
#define X 1    // the short gantry direction

// -----ENUMERATION-----
// enumeration of possible cases
// these don't have to be listed in order in this list
// All variables created by this type of enumeration are int
typedef enum {
   
    INITIALIZE,             // home the motors and move the tray to the out position

    HOMING_CYCLE,           // Homing procedure; WARNING: This is a partially blocking case
    START_HOMING_CYCLE,     // initialize variables for homing
    END_HOMING_CYCLE,       // clean up after homing

    WAIT_TO_START,          // wait for the start button, then begin the inspection

    PAUSE_SYSTEM,           // not used
    RESUME_SYSTEM,          // not used

                            // the start and finish gantry states are call NPOS times
                            // each time the gantry is move to a new position, a picture is taken

    START_GANTRY_MOVE,      // prepare gantry x and y asix for movement
    FINISH_GANTRY_MOVE,     // complete the gantry moves.  

    START_PICTURE,
    WAIT_FOR_PICTURE,
    FINISH_PICTURE,

    START_TRAY_MOVE_OUT,
    FINISH_TRAY_MOVE_OUT,
    START_TRAY_MOVE_IN,
    FINISH_TRAY_MOVE_IN,

    START_WIPER_MOVE_OUT,
    FINISH_WIPER_MOVE_OUT,
    START_WIPER_MOVE_IN,    
    FINISH_WIPER_MOVE_IN,

    ERROR_CONDITION,            // error state.  currently there is no way to recover from this state.  This needs fixing.
    
    EMERGENCY_STOP_START,             // Button pushed E. Stop
    EMERGENCY_STOP_INPROGRESS,        // E. Stop in progress
    EMERGENCY_STOP_EXIT               // Release E. Stop button

} systemState;

systemState currState;  // holds current case

// -----SPEEDYSTEPPER VARIABLES-----
// Uses a typedef provided by SpeedyStepper
SpeedyStepper ss_gantryy;  // 900mm
SpeedyStepper ss_gantryx;  // 350mm
SpeedyStepper ss_wiper; // 500mm
SpeedyStepper ss_tray;  // 1000mm

// -----LED QUICK FUNCTIONS-----
// The point is to make control of LEDs easier to write
// Write 0 or 1 in place of x for LED functions to control output
void LED_Ready(x){
    digitalWrite(DO_Ready, x);
    return 1;
}
void LED_WIP(x){
    digitalWrite(DO_WIP, x);
    return 1;
}
void LED_Part_Failure(x){
    digitalWrite(DO_Part_Failure, x);
    return 1;
}
void LED_System_Failure(x){
    digitalWrite(DO_System_Failure, x);
    return 1;
}

// -----MOTOR SPEED SETUP-----
// initialize the motor speeds and accelerations
// these are the values used while the machine is running normally
int initializeMotorSpeeds(){

    ss_gantryx.setStepsPerMillimeter(17.5 * 2);
    ss_gantryy.setStepsPerMillimeter(17.5 * 2);
    ss_wiper.setStepsPerMillimeter(17.5 * 2);
    ss_tray.setStepsPerMillimeter(17.5 * 2);

    ss_gantryx.setSpeedInStepsPerSecond(8000);
    ss_gantryx.setAccelerationInStepsPerSecondPerSecond(9500);

    ss_gantryy.setSpeedInStepsPerSecond(8000);
    ss_gantryy.setAccelerationInStepsPerSecondPerSecond(9500);

    ss_wiper.setSpeedInStepsPerSecond(?);
    ss_wiper.setAccelerationInStepsPerSecondPerSecond(?);

    ss_tray.setSpeedInStepsPerSecond(?);
    ss_tray.setAccelerationInStepsPerSecondPerSecond(?);
    
}

// -----SETUP-----
Setup() {

    // SpeedyStepper() sets most default values
    SpeedyStepper.SpeedyStepper();

    // STEPPER MOTOR SETUP
    // connectToPins(PUL, DIR)
    ss_gantryy.connectToPins(DO_XGANTRY_PUL, DO_XGANTRY_DIR);
    ss_gantryx.connectToPins(DO_YGANTRY_PUL, DO_YGANTRY_DIR);
    ss_wiper.connectToPins(DO_WIPER_PUL, DO_WIPER_DIR);
    ss_tray.connectToPins(DO_TRAY_PUL, DO_TRAY_DIR);

    // assign the homing digital inputs
    pinMode(DI_HOME_XGANTRY,INPUT);
    pinMode(DI_HOME_YGANTRY,INPUT);
    pinMode(DI_HOME_WIPER,INPUT);
    pinMode(DI_HOME_TRAY,INPUT);

    // Set the LED pins to OUTPUT
    pinMode(DO_Ready, OUTPUT);
    pinMode(DO_WIP, OUTPUT);
    pinMode(DO_Part_Failure, OUTPUT);
    pinMode(DO_System_Failure, OUTPUT);

    // assign the start button digital inputs
    pinMode(DI_EMERGENCYSTOP,INPUT);
    pinMode(DI_START,INPUT)
    
    // Test LEDs (Flash all LEDs a few seconds, then turn all off)
    LED_Ready(1);
    LED_WIP(1);
    LED_Part_Failure(1);
    LED_System_Failure(1);

    // delay() operates in milliseconds
    delay(2000);

    LED_Ready(0);
    LED_WIP(0);
    LED_Part_Failure(0);
    LED_System_Failure(0);

    currState = INITIALIZE;
}   

// -----PRIMARY LOOP-----
Loop() {

    // Checks to see if the emergency stop button has been pressed
    if(digitalRead(DI_EMERGENCYSTOP)){
        currState = EMERGENCY_STOP_START;
    }

    // -----SWITCH-----
    // The bulk of the program is composed of states referenced by case names that go in the place of currState
    // To initiate a state, ensure that the currState value matches the state that you are trying to call
    switch (currState){
        case BLANK:

            break;
    }
}
