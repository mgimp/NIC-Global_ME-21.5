include "SpeedyStepper.h"

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

#define DI_EMERGENCYSTOP ?
#define DI_START ?   // a momentary switch to start the inspection

#define DO_RUNNING  ?   // light is on if the process is running and flashing when there is an error.

// camera
#define DO_CAM_TAKEPICTURE ?
#define DI_CAM_GOTPICTURE ?
#define DI_CAM_FAILED ?


#define NPOS 4   // number of points the gantry must move to capture the pictures
#define Y 0    // the long gantry direction
#define X 1    // the short gantry direction


// Global Variables


// xy position to move to.  These are in steps from the home position
int xyposition[NPOS] = {
    {10,13},
    {34,23},
    {34,22},
    {1,123}
    }



// enumeration of possible states
// these don't have to be listed in order in this list
typedef enum {
   
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

    ERROR_CONDITION,           // error state.  currently there is no way to recover from this state.  This needs fixing.

} systemState;

// -----VARIOUS GLOBAL VARIABLES-----
bool flag_inStep = true;       // used as a flag for misstep errors; true means the actuators are in correct position
systemState  currState;  // this holds the current system state
int currPos;              // keeps track of where we are on the gantry

// stepper motors
SpeedyStepper ss_gantryx;  
SpeedyStepper ss_gantryy;   // long travel direction
SpeedyStepper ss_wiper;
SpeedyStepper ss_tray;


int initializeMotorSpeeds(){
    // initialize the motor speeds and accelerations
    // these are the values used while the machine is running normally

    ss_gantryx.setSpeedInStepsPerSecond(?);
    ss_gantryx.setAccelerationInStepsPerSecondPerSecond(?);

    ss_gantryy.setSpeedInStepsPerSecond(?);
    ss_gantryy.setAccelerationInStepsPerSecondPerSecond(?);

    ss_wiper.setSpeedInStepsPerSecond(?);
    ss_wiper.setAccelerationInStepsPerSecondPerSecond(?);

    ss_tray.setSpeedInStepsPerSecond(?);
    ss_tray.setAccelerationInStepsPerSecondPerSecond(?);
    
}

int homeMotor(SpeedyStepper* motor, int homeDI){
    // motor = pointer to speedystepper structure
    // homeDI = the digital pin for the home switch

    motor->setupStop();  // step in case something is running

    motor->setSpeedInStepsPerSecond(?); // something slow for homing
    motor->setAccelerationInStepsPerSecondPerSecond(?); // something slow for homing

    motor->setupRelativeMoveInSteps(?)  // something big that will force it to always move in one direction

    // homeIO can be the input pin of the home switch
    // switch is currently high = not touching
    while (digitalRead(homeDI)==HIGH){
        motor->processMovement();
    }
    
    motor->setupStop();

    // make this the new zero
    motor->setCurrentPositionInSteps(0);

    if (the stop button is pressed){
        return 1; // stopped because of the stop button.  This is an error condition
    }

    return 0; // no errors

}

void setup(){
    // setup all of the digital IO

   
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

    // assign the inprogress LED
    pinMode(DO_RUNNING,OUTPUT);

  
    // setup the camera IO
    // ?
 


    // do any other camera initialization

     // initialize the currState;
    currState = START_HOMING_CYCLE;
    currMove = 0;

}

void loop() {

    // loop uses a state-machine and nonblocking motor function

    // check for emergencies
    // currently only looking for the emergency stop
    // but could check other things here
    if (digitalRead(DI_EMERGENCYSTOP)) {
        // do something
        currState = ERROR_CONDITION;
    }
 

    switch (currState){
        case ERROR_CONDITION:
            // flash the LED until
            digitalWrite(DO_RUNNING,!digitalRead(DO_RUNNING);
            msDelay(500);

            // right now there is no way of exiting an error condition
            // this is something to be decided
            // for examlple could have the emergency stop button be latching and you need to undo it and then press start?
        break;

        // -----SCANNING CYCLE CASES-----
        case WAIT_TO_START:
            // this assume the tray is out in the home position
            // wait until the start button is pressed
            if (start button is pressed) {            
                currState=START_TRAY_MOVE_IN;
            }
            msDelay(10);
            break;

        case START_GANTRY_MOVE:
            ss_gantryx.setupMoveInSteps(xyposition[currPos][X]);
            ss_gantryy.setupMoveInSteps(xyposition[currPos][Y]);
            state = FINISH_GANTRY_MOVE;
            break;

        case FINISH_GANTRY_MOVE:
            ss_gantryx.processMovement();
            ss_gantryy.processMovement();

            if (ss_gantryx.motionComplete() &&  ss_gantryy.motionComplete()){
                state =  START_PICTURE;
            }
            break;    

        case START_PICTURE:
            // code to take a picture at position   currMove
          
            digitalWrite(DO_CAM_TAKEPICTURE,HIGH);
            delay(20);
            digitalWrite(DO_CAM_TAKEPICTURE,LOW);      
            state= WAIT_FOR_PICTURE;
            break;

        case WAIT_FOR_PICTURE:
            if (digitalRead(DI_CAM_GOTPICTURE)==HIGH) {
                // record the data is needed
                state=FINISH_PICTURE;
            }
            break;

        case FINISH_PICTURE:
            if (digitalRead(DI_CAM_FAILED)==LOW) {
                currMove++; // the next picture
                if (currMove > NPOS) {
                    // finished all of the inspections and they all passed
                    state = START_WIPER_MOVE;
                }
            } else {
                // failed inspection
                // record a failure
                // don't run the wiper
                state = START_TRAY_MOVE_OUT;
            }
            break;

        case START_WIPER_MOVE_OUT:
            ss_wiper.setupMoveInSteps(?);
            state = FINISH_WIPER_MOVE_OUT;
            break;

        case  FINISH_WIPER_MOVE_OUT:   
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                msDelay(200); // a little sloppy delaying here
                state =  START_WIPER_MOVE_IN;
            }
            break; 

        case START_WIPER_MOVE_IN:
            ss_wiper.setupMoveInSteps(?);
            state = FINISH_WIPER_MOVE_IN;
            break;

        case  FINISH_WIPER_MOVE_IN:   
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                state =  START_TRAY_MOVE_OUT;
            }
            break;

        case START_TRAY_MOVE_OUT:
            ss_tray.setupMoveInSteps(?);
            state = FINISH_TRAY_MOVE_OUT;
            break;

        case  FINISH_TRAY_MOVE_OUT:   
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                state =  WAIT_TO_START;
            }
            break;

         case START_TRAY_MOVE_IN:
                ss_tray.setupMoveInSteps(?);
                state = FINISH_TRAY_MOVE_IN;
           
            break;

        case  FINISH_TRAY_MOVE_IN:   
            ss_tray.processMovement();
            if (ss_tray.motionComplete()){
                state =  START_GANTRY_MOVE;
            }
            break;

        default:
            // error so do what every needs to be done
            delay(20);
            break;

        // -----HOMING CYCLE CASES-----
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