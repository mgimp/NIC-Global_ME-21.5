/* README
Written at NIC Global during the site tour.

This code is written to home the gantry x and y motors in order to measure the offset made by gantry movement constrints.
Once gantries are homed, we will measure the offset of the actuator block from the motor mount and subtract it from the planned scannign locations.
*/

#include "Arduino.h"
#include "SpeedyStepper.h"

// digital IO pins
// DO = digital output
// DI = digital input
#define DO_XGANTRY_DIR  3
#define DO_XGANTRY_PUL  4
#define DO_YGANTRY_DIR  1
#define DO_YGANTRY_PUL  2
#define DO_WIPER_DIR    69 // A15 
#define DO_WIPER_PUL    68 // A14
#define DO_TRAY_DIR     66 // A12
#define DO_TRAY_PUL     65 // A11

// home switch should be setup so there are alway HIGH unless triggered
// this provide a better default in case the wiring fails
#define DI_HOME_XGANTRY 58  // A4 // DOES NOT MATCH WIRING DIAGRAM 29APR2021
#define DI_HOME_YGANTRY 57  // A3 // DOES NOT MATCH WIRING DIAGRAM 29APR2021
#define DI_HOME_WIPER   56  // A2 // DOES NOT MATCH WIRING DIAGRAM 29APR2021
#define DI_HOME_TRAY    54  // A0 // the home should be on the inside of the inspection chamber

// button inputs
#define DI_EMERGENCYSTOP 55 // A1
#define DI_START         19 // a momentary switch to start the inspection

// LED inputs
#define DO_Ready          14    //Green LED on for ready status
#define DO_WIP            15    //Yellow LED blinking for cycle in progress
#define DO_Part_Failure   16    //RED LED Blinking for part failure
#define DO_System_Failure 17    //RED LED Steady for system failure
    
// camera
#define DO_CAM_TAKEPICTURE 11
#define DI_CAM_GOTPICTURE  10
#define DI_CAM_FAILED      9 
#define DI_CAM_MISPRINT    8 

    
#define NPOS 9  // number of points the gantry must move to capture the pictures
#define Y 1     // the long gantry direction
#define X 0     // the short gantry direction

// -----ORIGINAL XYPOSITIONS ----- //
// float xyposition[NPOS][2] = {
//     {262.5,112.5},               // First position
//     {262.5,337.5},               // Second position
//     {262.5,562.5},               // Third posistion
//     {262.5,787.5},               // Fourth position end of y axis and end of the x axis
//     {87.5,787.5},               // Fifth position retracts the x axis
//     {87.5,562.5},               // Sixth position
//     {87.5,337.5},               // Seventh position 
//     {87.5,112.5},               // Eighth position
//     {0,0}
//     };

// enumeration of possible states
// these don't have to be listed in order in this list
typedef enum  {
   
    HOMING_CYCLE,           // Homing procedure; WARNING: This is a partially blocking case
    START_HOMING_CYCLE,     // initialize variables for homing
    FINISH_HOMING_CYCLE,       // clean up after homing
    
    START_ERROR_CONDITION,
    ERROR_CONDITION,           // error currState.  currently there is no way to recover from this currState.  This needs fixing.
    FINISH_ERROR_CONDITION,

} systemState;

// -----VARIOUS GLOBAL VARIABLES-----
systemState currState;  // this holds the current system currState
int currPos;            // keeps track of where we are on the gantry

// added as global for consistency
int homingStep;         // A variable for keeping track of which gantry is being homed
int flag_homingError;       // A variable for keeping track of gantry homing success

// OOS Errors start as 1 so that an initial homing cycle will happen
int flag_OOS_tray = 1;      // out of step error for tray
int flag_OOS_wiper = 1;     // out of step error for wiper
int flag_OOS_gantryy = 1;   // out of step error for y gantry
int flag_OOS_gantryx = 1;   // out of step error for x gantry

// stepper motors
SpeedyStepper ss_gantryx;   // short travel direction
SpeedyStepper ss_gantryy;   // long travel direction
SpeedyStepper ss_wiper;
SpeedyStepper ss_tray;

int initializeMotorSpeeds(){
    // initialize the motor speeds and accelerations
    // these are the values used while the machine is running normally

    ss_gantryx.setStepsPerMillimeter(17.5 * 2);
    ss_gantryy.setStepsPerMillimeter(17.5 * 2);
    ss_wiper.setStepsPerMillimeter(17.5 * 2);
    ss_tray.setStepsPerMillimeter(17.5 * 2);
    
    ss_gantryx.setSpeedInMillimetersPerSecond(228.57); // TEST SCRIPT
    ss_gantryx.setAccelerationInMillimetersPerSecondPerSecond(271.43); // TEST SCRIPT
    
    ss_gantryy.setSpeedInMillimetersPerSecond(228.57); // TEST SCRIPT
    ss_gantryy.setAccelerationInMillimetersPerSecondPerSecond(271.43); // TEST SCRIPT

    ss_wiper.setSpeedInMillimetersPerSecond(228.57); // TEST SCRIPT
    ss_wiper.setAccelerationInMillimetersPerSecondPerSecond(271.43); // TEST SCRIPT

    ss_tray.setSpeedInMillimetersPerSecond(228.57); // TEST SCRIPT
    ss_tray.setAccelerationInMillimetersPerSecondPerSecond(271.43); // TEST SCRIPT
    
    ss_tray.setSpeedInMillimetersPerSecond(228.57);
    ss_tray.setAccelerationInMillimetersPerSecondPerSecond(271.43);

}

void setup(){
    Serial.begin(9600); // TEST SCRIPT

    initializeMotorSpeeds();

    // -----SETUP ALL DIGITAL I/O-----//
    // Assign IO pins used for Step and Direction
    ss_gantryx.connectToPins(DO_XGANTRY_PUL, DO_XGANTRY_DIR);
    ss_gantryy.connectToPins(DO_YGANTRY_PUL, DO_YGANTRY_DIR);
    ss_wiper.connectToPins(DO_WIPER_PUL, DO_WIPER_DIR);
    ss_tray.connectToPins(DO_TRAY_PUL, DO_TRAY_DIR);

    // assign the homing digital inputs
    pinMode(DI_HOME_XGANTRY,INPUT);
    pinMode(DI_HOME_YGANTRY,INPUT);
    pinMode(DI_HOME_WIPER,INPUT);
    pinMode(DI_HOME_TRAY,INPUT);

    // assign the start button digital inputs
    pinMode(DI_EMERGENCYSTOP,INPUT);
    pinMode(DI_START,INPUT);

    // assign the LED digital outputs
    pinMode(DO_Part_Failure,OUTPUT);
    pinMode(DO_WIP,OUTPUT);
    pinMode(DO_System_Failure,OUTPUT);
    pinMode(DO_Ready,OUTPUT);

    // assign LED states to low
    digitalWrite(DO_WIP,LOW);               // Turn off WIP light
    digitalWrite(DO_Ready,LOW);             // Turn off Ready light
    digitalWrite(DO_Part_Failure,LOW);      // Turn off Part Failure light
    digitalWrite(DO_System_Failure,LOW);    // Turn off System Failure light

     // initialize the currState;
    currState = START_HOMING_CYCLE;
    currPos = 0;  // Renamed to match code
    Serial.print("START_HOMING_CYCLE\n"); // TEST SCRIPT

}

// -----LOOP STRUCTURE----- //
// Loop structure uses a State-machine and non-blocking motor functions whenever possible.
// Cases should return to the beginning of loop() in between instructions to maintain the non-blocking quality.
// 'currState' is the variable that controls the working case.
// Cases contain internal parameters to determine which cases succeed each other.
void loop() {

    // -----EMERGENCY STOP BUTTON----- //
    // Checks to see if stop button has been pressed before continuing to the current case.
    // Switches to ERROR_CONDITION case if button pressed.

    // Button input depends on non-blocking structure of the program.
    // Each case needs to repeatedly return to the top of loop() in order to check if the button has been pressed.
    // The stop button will not work in blocking cases
    if (digitalRead(DI_EMERGENCYSTOP)) {
        // do something
        currState = START_ERROR_CONDITION;
        Serial.print("START_ERROR_CONDITION\n"); // TEST SCRIPT
    }
    
    // -----STATE MACHINE----- //
    // State machine calls whichever instructions are determined by variable 'currState'.
    // Cases will not change without specific instructions.
    // The order of cases as defined in the switch does not affect the order that they are called.
    // Most cases contain instructions to select the next case.

    // The non-blocking case structure needs each case to repeatedly exit and re-enter the switch in order to detect errors and certain inputs.
    // Blocking cases must include conditions that would ordinarily be detected at the top of loop().
    switch (currState){
        // -----ERROR_CONDITION----- //
        // This case is the general issue currState that is called in the following situations:
        // 1. Any internal error (as determined within individual cases)
        // 2. If the stop button is pressed.

        case START_ERROR_CONDITION:
            digitalWrite(DO_System_Failure,HIGH); // turn on the RED LED when E.stop button is pressed
            //update to new case and then break
            currState = ERROR_CONDITION; //go to emergency stop progress state
            Serial.print("ERROR_CONDITION\n"); // TEST SCRIPT
            break; 

        case ERROR_CONDITION:
            // Decelerating all four motors to zero velocity
            ss_gantryy.setupStop();
            ss_gantryx.setupStop();
            ss_wiper.setupStop();
            ss_tray.setupStop();

            //if all 4 motors stop, update the currState to exit state
            if (ss_gantryy.processMovement() && ss_gantryx.processMovement() && 
            ss_wiper.processMovement() && ss_tray.processMovement() && !digitalRead(DI_EMERGENCYSTOP))
            {
                // currState = FINISH_ERROR_CONDITION;
                // Serial.print("FINISH_ERROR_CONDITION\n"); // TEST SCRIPT 
            }
            break;
 
        // -----HOMING CYCLE CASES----- //
        case START_HOMING_CYCLE:
            digitalWrite(DO_WIP,HIGH);              // Turn on WIP light
            homingStep = 1;         // A variable for keeping track of which gantry is being homed
            flag_homingError=true;       // A variable for keeping track of gantry homing success
            currState = HOMING_CYCLE;
            Serial.print("HOMING_CYCLE\n"); // TEST SCRIPT
            break;
            
        case HOMING_CYCLE:
            // This is a partially BLOCKING CASE.  It blocks during the homing of each axis, but releases between moves.
            // case uses the SpeedyStepper pre-built blocking homing function
            // SpeedyStepper.ccp line[325]
            // moveToHomeInMillimeters(long directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin)

            // The basic function is that the homing cycle will cycle to the start of loop() after every motor is homed
            // moveToHomeInMillimeters() will return false if the homing limit switch is not pressed for a set distance
            // if flag_homingError==false when cycling then an error currState will be called

            // Homing case is called after each cycle, but only motors with a raised OOS flag will home.
            // All OOS flags are raised when the arduino is initialized.
               
            // directionTowardHome is set to -1, toward motor
            // speedInMillimetersPerSecond is set to half max speed
            // maxDistanceToMoveInMillimeters is set to 900mm, the length of gantry Y
            if ((homingStep == 1) && flag_OOS_gantryy) flag_homingError = ss_gantryy.moveToHomeInMillimeters(-1, 50, 900, DI_HOME_YGANTRY);

            // maxDistanceToMoveInMillimeters is set to 350mm, the length of gantry X
            if ((homingStep == 2) && flag_OOS_gantryx) flag_homingError == ss_gantryx.moveToHomeInMillimeters(-1, 50, 350, DI_HOME_XGANTRY);

            // maxDistanceToMoveInMillimeters is set to 500mm, the length of the wiper actuator
            // if ((homingStep == 3) && flag_OOS_wiper) flag_homingError = ss_wiper.moveToHomeInMillimeters(-1, 50, 500, DI_HOME_WIPER);

            // maxDistanceToMoveInMillimeters is set to 1000mm, the length of the tray actuator
            // if ((homingStep == 4) && flag_OOS_tray) flag_homingError = ss_tray.moveToHomeInMillimeters(-1, 50, 1000, DI_HOME_TRAY);
                
            if (homingStep >= 5){
                currState = FINISH_HOMING_CYCLE;
                Serial.print("FINISH_HOMING_CYCLE\n");  //TEST SCRIPT
            } 

            if (flag_homingError == false) currState = START_ERROR_CONDITION;

            homingStep++;          
            break;
            
        case FINISH_HOMING_CYCLE:
            digitalWrite(DO_WIP,LOW);                   // Turn off WIP light
            // currState = START_ERROR_CONDITION;          // WAIT_TO_START assumes tray is out
            // Serial.print("START_ERROR_CONDITION\n");    // TEST SCRIPT 

            //reset OOS flags
            flag_OOS_gantryy=0;
            flag_OOS_gantryx=0;
            flag_OOS_wiper=0;
            flag_OOS_tray=0; 
            break;

        default:    // This should never be called
            Serial.print("default\n"); // TEST SCRIPT
            delay(20);
            break;

    }



}
