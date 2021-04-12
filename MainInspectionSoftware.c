#include "Arduino.h"
#include "SpeedyStepper.h"

// digital IO pins
// DO = digital output
// DI = digital input
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
#define DI_HOME_XGANTRY  ?
#define DI_HOME_YGANTRY  ?
#define DI_HOME_WIPER  ?
#define DI_HOME_TRAY  ?   // the home should be on the inside of the inspection chamber

// home switch values
#define HOMESWITCH_PRESSED LOW      // exists to clarify home switch states
#define HOMESWITCH_RELEASED HIGH    // exists to clarify home switch states
// example:
// if (digitalRead(DI_HOME_YGANTRY) == HOMESWITCH_PRESSED)

// button inputs
#define DI_EMERGENCYSTOP ?
#define DI_START ?   // a momentary switch to start the inspection

// LED inputs
#define DO_Ready ?      //Green LED on for ready status
#define DO_WIP ?        //Yellow LED blinking for cycle in progress
#define DO_Part_Failure ?  //RED LED Blinking for part failure
#define DO_System_Failure ? //RED LED Steady for system failure
    
// camera
#define DO_CAM_TAKEPICTURE ?
#define DI_CAM_GOTPICTURE ?
#define DI_CAM_FAILED ?
#define DI_CAM_MISPRINT ?

    
#define NPOS 8   // number of points the gantry must move to capture the pictures
#define Y 0    // the long gantry direction
#define X 1    // the short gantry direction

// xy position to move to.  These are in steps from the home position
int xyposition[NPOS] = {
    {262.5,112.5},               // First position
    {262.5,337.5},               // Second position
    {262.5,562.5},               // Third posistion
    {262.5,787.5},               // Fourth position end of y axis and end of the x axis
    {87.5, 787.5},               // Fifth position retracts the x axis
    {87.5, 562.5},               // Sixth position
    {87.5, 337.5},               // Seventh position 
    {87.5, 112.5},               // Eighth position
    {0, 0}
    }

// enumeration of possible states
// these don't have to be listed in order in this list
typedef enum  {
   
    HOMING_CYCLE,           // Homing procedure; WARNING: This is a partially blocking case
    START_HOMING_CYCLE,     // initialize variables for homing
    FINISH_HOMING_CYCLE,       // clean up after homing

    START_TRAY_MOVE_OUT_WTS     // Case exists to extend tray between FINISH_HOMING_CYCLE and WAIT_TO_START
    FINISH_TRAY_MOVE_OUT_WTS    // Case exists to extend tray between FINISH_HOMING_CYCLE and WAIT_TO_START

    WAIT_TO_START,          // wait for the start button, then begin the inspection

    PAUSE_SYSTEM,           // not used
    RESUME_SYSTEM,          // not used

                            // the start and finish gantry states are call NPOS times
                            // each time the gantry is move to a new position, a picture is taken

    START_GANTRY_MOVE,      // prepare gantry x and y asix for movement
    FINISH_GANTRY_MOVE,     // complete the gantry moves.  

    START_PICTURE,
    WAIT_FOR_PICTURE,
    NEXT_PICTURE,
    FINISH_PICTURE,

    START_TRAY_MOVE_OUT,
    FINISH_TRAY_MOVE_OUT,
    START_TRAY_MOVE_IN,
    FINISH_TRAY_MOVE_IN,

    START_WIPER_MOVE_OUT,
    FINISH_WIPER_MOVE_OUT,
    START_WIPER_MOVE_IN,    
    FINISH_WIPER_MOVE_IN,

    PART_COMPLIANCE_FAILURE,
    
    ERROR_CONDITION,           // error state.  currently there is no way to recover from this state.  This needs fixing.

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
    
    ss_gantryx.setSpeedInStepsPerSecond(8000);
    ss_gantryx.setAccelerationInStepsPerSecondPerSecond(9500);

    ss_gantryy.setSpeedInStepsPerSecond(8000);
    ss_gantryy.setAccelerationInStepsPerSecondPerSecond(9500);

    ss_wiper.setSpeedInStepsPerSecond(?);
    ss_wiper.setAccelerationInStepsPerSecondPerSecond(?);

    ss_tray.setSpeedInStepsPerSecond(?);
    ss_tray.setAccelerationInStepsPerSecondPerSecond(?);
    
}

void setup(){
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
    pinMode(DI_START,INPUT)

    // assign the LED digital outputs
    pinMode(DO_Part_Failure,OUTPUT);
    pinMode(DO_WIP,OUTPUT);
    pinMODE(DO_System_Failure,OUTPUT);
    pinMODE(DO_Ready,OUTPUT);

    // assign LED states to low
    digitalWrite(DO_WIP,LOW);               // Turn off WIP light
    digitalWrite(DO_Ready,LOW);             // Turn off Ready light
    digitalWrite(DO_Part_Failure,LOW);      // Turn off Part Failure light
    digitalWrite(DO_System_Failure,LOW);    // Turn off System Failure light

    // setup the camera IO
    // ?
 
    // do any other camera initialization

     // initialize the currState;
    currState = START_HOMING_CYCLE;
    currPos = 0;  // Renamed to match code

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
        currState = ERROR_CONDITION;
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
        // This case is the general issue state that is called in the following situations:
        // 1. Any internal error (as determined within individual cases)
        // 2. If the stop button is pressed.

        // ERROR_CONDITION is subject to change at this time -28MAR2021
        case ERROR_CONDITION:
            // flash the LED until
            digitalWrite(DO_RUNNING,!digitalRead(DO_RUNNING);
            msDelay(500);

            // right now there is no way of exiting an error condition
            // this is something to be decided
            // for examlple could have the emergency stop button be latching and you need to undo it and then press start?
            break;

        // -----HOMING CYCLE CASES----- //
        case START_HOMING_CYCLE:
            digitalWrite(DO_WIP,HIGH);              // Turn on WIP light
            homingStep = 1;         // A variable for keeping track of which gantry is being homed
            flag_homingError=0;       // A variable for keeping track of gantry homing success
            currState = HOMING_CYCLE;
            break;
            
        case HOMING_CYCLE:
            // This is a partially BLOCKING CASE.  It blocks during the homing of each axis, but releases between moves.
            // case uses the SpeedyStepper pre-built blocking homing function
            // SpeedyStepper.ccp line[325]
            // moveToHomeInMillimeters(long directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin)

            // The basic function is that the homing cycle will cycle to the start of loop() after every motor is homed
            // moveToHomeInMillimeters() will return false if the homing limit switch is not pressed for a set distance
            // if flag_homingError==false when cycling then an error state will be called

            // Homing case is called after each cycle, but only motors with a raised OOS flag will home.
            // All OOS flags are raised when the arduino is initialized.
               
            // directionTowardHome is set to -1, toward motor
            // speedInMillimetersPerSecond is set to half max speed
            // maxDistanceToMoveInMillimeters is set to 900mm, the length of gantry Y
            if ((homingStep == 1) && flag_OOS_gantryy) flag_homingError = ss_gantryy.moveToHomeInMillimeters(-1, ?, 900, DI_HOME_YGANTRY);

            // maxDistanceToMoveInMillimeters is set to 350mm, the length of gantry X
            if ((homingStep == 2) && flag_OOS_gantryx) flag_homingError == ss_gantryx.moveToHomeInMillimeters(-1, ?, 350, DI_HOME_XGANTRY);

            // maxDistanceToMoveInMillimeters is set to 500mm, the length of the wiper actuator
            if ((homingStep == 3) && flag_OOS_wiper) flag_homingError = ss_wiper.moveToHomeInMillimeters(-1, ?, 500, DI_HOME_WIPER);

            // maxDistanceToMoveInMillimeters is set to 1000mm, the length of the tray actuator
            if ((homingStep == 4) && flag_OOS_tray) flag_homingError = ss_tray.moveToHomeInMillimeters(-1, ?, 1000, DI_HOME_TRAY);
                
            if (homingStep == 5) currState = FINISH_HOMING_CYCLE;

            if (flag_homingError == false) currState = ERROR_CONDITION;

            homingStep++;            
            break;
            
        case FINISH_HOMING_CYCLE:
            digitalWrite(DO_WIP,LOW);               // Turn off WIP light
            currState = START_TRAY_MOVE_OUT_WTS;    // WAIT_TO_START assumes tray is out
                         
            //reset OOS flags
            flag_OOS_gantryy=0;
            flag_OOS_gantryx=0;
            flag_OOS_wiper=0;
            flag_OOS_tray=0; 
                         
            break;

        case START_TRAY_MOVE_OUT_WTS:               // Case exists to extend tray between FINISH_HOMING_CYCLE and WAIT_TO_START
            digitalWrite(DO_WIP,HIGH);              // Turn on WIP light
            ss_tray.setupMoveInMilimeter(980);
            state = FINISH_TRAY_MOVE_OUT_WTS;
            break;
        
        case FINISH_TRAY_MOVE_OUT_WTS:              // Case exists to extend tray between FINISH_HOMING_CYCLE and WAIT_TO_START
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                digitalWrite(DO_WIP,LOW);           // Turn on WIP light
                currState = WAIT_TO_START; 
            }            
            break;

        case WAIT_TO_START:
            // this case assumes the tray is out in the home position
            // wait until the start button is pressed
            digitalWrite(DO_Ready,HIGH);
            if (digitalRead(DI_START)==HIGH){
                digitalWrite(DO_Ready,LOW);    
                digitalWrite(DO_Part_Failure,LOW);      // Turn off Part Failure light
                digitalWrite(DO_WIP,HIGH);              // Turn on WIP light
                currState = START_TRAY_MOVE_IN;
            }
            msDelay(10);
            break;

        case START_TRAY_MOVE_IN:
            ss_tray.setupMoveInMilimeter(0);
                         
            // also move the gantry to position 0,0. This lets you check the homing
            ss_gantryx.setupMoveInSteps(0,0);
            ss_gantryy.setupMoveInSteps(0,0);
                         
            state = FINISH_TRAY_MOVE_IN;   
            break;

        case FINISH_TRAY_MOVE_IN:   
            ss_tray.processMovement();
               
            // move the gantry to 0,0              
            ss_gantryx.processMovement();
            ss_gantryy.processMovement();
            
            // verify that the limit switches are in the correct state depending on if the motor is still moving or not
            // if moving switch should not be pressed.  if not moving switch should be pressed
            if (!ss_gantryy.processMovement() && !digitalRead(DI_HOME_YGANTRY)) flag_OOS_gantryy = 1;   // Check to see if home switch pressed too early
            if (!ss_gantryx.processMovement() && !digitalRead(DI_HOME_XGANTRY)) flag_OOS_gantryx = 1;   // Check to see if home switch pressed too early
            if (!ss_tray.motionComplete() && !digitalRead(DI_HOME_TRAY)) flag_OOS_tray = 1; // Check to see if home switch pressed too early
      
                         
            if (ss_tray.motionComplete() && ss_gantryx.processMovement() && ss_gantryy.processMovement()){
                if (digitalRead(DI_HOME_TRAY)) flag_OOS_tray = 1;           // Check if home switch pressed at (0,0) position
                if (digitalRead(DI_HOME_YGANTRY)) flag_OOS_gantryy = 1;    // Homing cycle flag in case misstep occurs
                if (digitalRead(DI_HOME_XGANTRY)) flag_OOS_gantryx = 1;    // Homing cycle flag in case misstep occurs
              
                currState = START_GANTRY_MOVE;
                currPos = 0; // set the starting position
               
            }
            break;

        case START_GANTRY_MOVE:
            ss_gantryx.setupMoveInMillimeter(xyposition[currPos][X]);
            ss_gantryy.setupMoveInMillimeter(xyposition[currPos][Y]);
            currState = FINISH_GANTRY_MOVE;
            break;

        case FINISH_GANTRY_MOVE:
            ss_gantryx.processMovement();
            ss_gantryy.processMovement();

            if (ss_gantryx.processMovement() && ss_gantryy.processMovement()){
                currState = START_PICTURE;
            } 
            break;    

        case START_PICTURE:
            // code to take a picture at position   currMove
            digitalWrite(DO_CAM_TAKEPICTURE,HIGH);
            delay(20);
            digitalWrite(DO_CAM_TAKEPICTURE,LOW);      
            currState= WAIT_FOR_PICTURE;
            break;

        case WAIT_FOR_PICTURE:
            if (digitalRead(DI_CAM_GOTPICTURE)==HIGH) {
                // record the data is needed
                currState=NEXT_PICTURE;
            }
            break;
        
        case NEXT_PICTURE:
                if (currPos < NPOS){
                    currPos++; //Update the next position for the gantry
                    currState = START_GANTRY_MOVE; //Goes into the next position for taking a picture picture
                } else { //After all 8 positions has been completed
                    currState = FINISH_PICTURE;
                }
            break;            

        case FINISH_PICTURE:
            if (digitalRead(DI_CAM_FAILED)==LOW){
                
                if (currPos >= NPOS) && digitalRead(DI_CAM_MISPRINT) ==LOW) {

                    // finished all of the inspections and they all passed
                    digitalWrite(DO_WIP,LOW);
                    currState = START_WIPER_MOVE_OUT;

                } else {
                    currPos++; // the next picture
                    currState = START_GANTRY_MOVE; // move to the next gantry position position 
                }
            } else {
                if (digitalRead(DI_CAM_MISPRINT)==HIGH){
                    digitalWrite(DO_WIP,LOW);
                    currState = PART_COMPLIANCE_FAILURE; // failed inspection
                // record a failure (? within compliance case)
                }
            }
            break;

        case PART_COMPLIANCE_FAILURE:
            // -----BAD PART----- //
            // This is the only case made specially for a part being unsatisfactory.
            // The basic system behavior has the part under observation returned to the operator when it fails inspection.
            // It's possible that
             digitalWrite(DO_Part_Failure,HIGH);    // This LED cleared when the start button is pressed in the WAIT_TO_START case
             state = START_TRAY_MOVE_OUT;
             break;
                         
        case START_WIPER_MOVE_OUT:
            digitalWrite(DO_WIP,HIGH);               // Turn on WIP light
            ss_wiper.setupMoveInMillimeter(480);
            state = FINISH_WIPER_MOVE_OUT;
            break;

        case FINISH_WIPER_MOVE_OUT:   
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                msDelay(200); // a little sloppy delaying here
                currState = START_WIPER_MOVE_IN;
            }
            break; 

        case START_WIPER_MOVE_IN:
            ss_wiper.setupMoveInMillimeter(0);
            state = FINISH_WIPER_MOVE_IN;
            break;

        case FINISH_WIPER_MOVE_IN:   
            ss_wiper.processMovement();
            
            if (!ss_wiper.motionComplete() && !digitalRead(DI_HOME_WIPER)) flag_OOS_wiper = 1; // Check to see if home switch pressed before movement complete
            
            if (ss_wiper.motionComplete()){
                if (digitalRead(DI_HOME_WIPER)) flag_OOS_wiper = 1;   // an extra check to see if a misstep occured
                currState = START_TRAY_MOVE_OUT;
            }
            break;

        case START_TRAY_MOVE_OUT:
            ss_tray.setupMoveInMillimeter(980);
            state = FINISH_TRAY_MOVE_OUT;
            break;

        case FINISH_TRAY_MOVE_OUT:
            digitalWrite(DO_Part_Failure,LOW);
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                digitalWrite(DO_WIP,LOW);               // Turn off WIP light
                currState = START_HOMING_CYCLE; //Homing cycle checks for misstep errors that may have occurred
            }
            break;

        default:    // This should never be called
            delay(20);
            break;

    }



}
