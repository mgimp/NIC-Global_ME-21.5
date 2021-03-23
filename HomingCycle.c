// -----LIBRARIES-----
// SETUP STEPPER MOTOR VARIABLES
#include "SpeedyStepper.h"

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

    WAIT_TO_START,  // Get into position for a part and wait for start button

    HOMING_CYCLE, // Homing procedure; WARNING: This is a partially blocking case
    START_HOMING_CYCLE, // initialize variables for homing
    END_HOMING_CYCLE, // clean up after homing

    ERROR_CONDITION,   // general for things gone wrong

    EMERGENCY_STOP, // Button pushed E. Stop

} systemState;

systemState currState;  // holds current case

// -----VARIOUS GLOBAL VARIABLES-----
bool flag_inStep = true;       // used as a flag for misstep errors; true means the actuators are in correct position

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

// -----MOTOR SPEED FUNCTIONS-----
int initializeMaxMotorSpeeds(){
    // initialize the motor speeds and accelerations at maximum for single motor control

    ss_gantryx.setMillimetersPerMillimeter(17.5 * 2);
    ss_gantryy.setMillimetersPerMillimeter(17.5 * 2);
    ss_wiper.setMillimetersPerMillimeter(17.5 * 2);
    ss_tray.setMillimetersPerMillimeter(17.5 * 2);

    ss_gantryy.setSpeedInMillimetersPerSecond(8000/17.5);
    ss_gantryy.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);

    ss_gantryx.setSpeedInMillimetersPerSecond(8000/17.5);
    ss_gantryx.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);

    ss_wiper.setSpeedInMillimetersPerSecond(8000/17.5);
    ss_wiper.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);

    ss_tray.setSpeedInMillimetersPerSecond(8000/17.5);
    ss_tray.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);
    
}

// -----SETUP-----
Setup() {

    // SpeedyStepper() sets most default values
    SpeedyStepper.SpeedyStepper()

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

    currState = HOMING_CYCLE;
}   

Loop() {

    // Checks to see if the emergency stop button has been pressed
    if (digitalRead(DI_EMERGENCYSTOP) || (flag_inStep == false)) {  // flag_inStep works checks for misstep
        currState = EMERGENCY_STOP;
    }

    switch (currState){
        case START_HOMING_CYCLE:
                    // LED STUFF
                    LED_Ready(0);
                    LED_InProgress(1);
                    LED_Failure(0);
                    LED_Error(0);
                    homingstep = 1
                   currState = HOMING_CYCLE:
            break;
            
            case HOMING_CYCLE:
            // This is a partially BLOCKING CASE.  It blocks during the homing of each axis, but releases between moves.
            // case uses the SpeedyStepper pre-build blocking homing function
            // SpeedyStepper.ccp line[325]
            // moveToHomeInMillimeters(long directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin)

            // The basic function is that the homing cycle will cycle to the start of loop() after every motor is homed
            // moveToHomeInMillimeters() will return false if the homing limit switch is not pressed for a set distance
            // if flag_inStep==false when cycling then an error state will be called

//   int step static or global, otherwise they will be reinitialized everytime through the Loop() function
//   it was renamed to homingstep.
//   int step;   // variable to track progress of homing and escape the while loop
                
                // directionTowardHome is set to -1, toward motor
                // speedInMillimetersPerSecond is set to half max speed
                // maxDistanceToMoveInMillimeters is set to 900mm, the length of gantry Y
                if (homingstep == 1){
                    flag_inStep = ss_gantryy.moveToHomeInMillimeters(-1, 4000/17.5, 900, DI_HOME_YGANTRY);
                }

                // maxDistanceToMoveInMillimeters is set to 350mm, the length of gantry X
                if (homingstep == 2){
                    flag_inStep == ss_gantryx.moveToHomeInMillimeters(-1, 4000/17.5, 350, DI_HOME_XGANTRY);
                }

                // maxDistanceToMoveInMillimeters is set to 500mm, the length of the wiper actuator
                if (homingstep == 3){
                    flag_inStep = ss_wiper.moveToHomeInMillimeters(-1, 4000/17.5, 500, DI_HOME_WIPER);
                }

                // maxDistanceToMoveInMillimeters is set to 1000mm, the length of the tray actuator
                if (homingstep == 4){
                    flag_inStep = ss_tray.moveToHomeInMillimeters(-1, 4000/17.5, 1000, DI_HOME_TRAY);
                    currState = WAIT_TO_START;  // Whatever case gets into ready position
                }

                if (flag_inStep == false){
                    currState = ERROR_CONDITION;
                }
                homingstep++;            
            break;
            
            case END_HOMING_CYCLE:
                    // LED STUFF and any other final homing procedures
                    LED_Ready(0);
                    LED_InProgress(0);
                    LED_Failure(0);
                    LED_Error(0);
                  currState = WAIT_TO_START;
            break;
    }
}
