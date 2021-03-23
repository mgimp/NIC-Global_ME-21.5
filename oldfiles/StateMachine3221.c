include "SpeedyStepper.h"

// digital IO pins
// DO = digital output
// DI = digital input
#define DO_XGANTRY_DIR   3
#define DO_XGANTRY_PUL   4
#define DO_YGANTRY_DIR   1
#define DO_YGANTRY_PUL   2
#define DO_WIPER_DIR   ?
#define DO_WIPER_PUL   ?
#define DO_TRAY_DIR   ?
#define DO_TRAY_PUL  ?

// home switch should be setup so there are alway HIGH unless triggered
// this provide a better default in case the wiring fails
#define DI_HOME_XGANTRY  ?
#define DI_HOME_YGANTRY  33
#define DI_HOME_WIPER  ?
#define DI_HOME_TRAY  ?   // the home should be on the inside of the inspection chamber

#define DI_EMERGENCYSTOP ?
#define DI_START ?   // a momentary switch to start the inspection

#define DO_RUNNING  ?   // light is on if the process is running and flashing when there is an error.

// Lights
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

// Global Variables


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
typedef enum {
   
    INITIALIZE,             // home the motors and move the tray to the out position

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
    
    PART_COMPLIANCE_FAILURE,

    ERROR_CONDITION,           // error state.  currently there is no way to recover from this state.  This needs fixing.

} systemState;



systemState  currState;  // this holds the current system state
int currPos;              // keeps track of where we are on the gantry

// -----VARIOUS GLOBAL VARIABLES-----
bool flag_inStep = true;       // used as a flag for misstep errors; true means the actuators are in correct position

// stepper motors
SpeedyStepper ss_gantryx;  
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
    currState = INITIALIZE;
    currMove = 0;

}

void loop() {

    // loop uses a state-machine and nonblocking motor function

    // check for emergencies
    // currently only looking for the emergency stop
    // but could check other things here
    // MATT EDIT
    if (digitalRead(DI_EMERGENCYSTOP) || (flag_inStep == false)) {  // flag_inStep works checks for misstep
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

        case INITIALIZE:
            digitalWrite(DO_RUNNING,HIGH); // turn on the light
            // home the motors
            //  may need to have some process to make sure there isn't a part on the tray
            //
            // need to fill these in for each motor
            // moveToHomeInSteps(long directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps, int homeLimitSwitchPin)

            if (!ss_gantryx.moveToHomeInSteps(?,?,?, DI_HOME_XGANTRY) ){
                // home failed
                currState = ERROR_CONDITION;
            } 
            else if (!ss_gantryy.moveToHomeInSteps(?,?,?, DI_HOME_YGANTRY) ){
                // home failed
                currState = ERROR_CONDITION;
            } 
            else if (!ss_tray.moveToHomeInSteps(?,?,?, DI_HOME_TRAY) ){
                // home failed
                currState = ERROR_CONDITION;
            } 
            else if (!ss_wiper.moveToHomeInSteps(?,?,?, DI_HOME_WIPER) ){
                // home failed
                currState = ERROR_CONDITION;
            }

            if (currState == INITTIALIZE) {
                ss_tray.moveToPositionInSteps(?); // move the tray out
                currState= WAIT_TO_START;
            }
            break;


        case WAIT_TO_START:
            // this assume the tray is out in the home position
            // wait until the start button is pressed
            if (start button is pressed) {            
                currState=START_TRAY_MOVE_IN;
            }
            msDelay(10);
            break;

        case START_GANTRY_MOVE:
            ss_gantryx.setupMoveInMillimeters(xyposition[currPos][X]);
            ss_gantryy.setupMoveInMillimeters(xyposition[currPos][Y]);
            state = FINISH_GANTRY_MOVE;
            break;

        case FINISH_GANTRY_MOVE:
            ss_gantryx.processMovement();
            ss_gantryy.processMovement();

            if (ss_gantryx.motionComplete() && ss_gantryy.motionComplete()){
                // // MATT CODE
                // Is this where we return to (0,0)?
                if ((digitalRead(DI_HOME_XGANTRY) == true) || (digitalRead(DI_HOME_YGANTRY) == true)){   // an extra check to see if a misstep occured
                    flag_inStep == false;
                }

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
                currPos++; // the next picture
                if ((currPos > NPOS)  && digitalRead(DI_CAM_MISPRINT) ==LOW) {
                    // finished all of the inspections and they all passed
                    state = START_WIPER_MOVE;
                    }
                )else {
                    if (digitalRead(DI_CAM_MISPRINT)==HIGH){
                        state = PART_COMPLIANCE_FAILURE // failed inspection
                     // record a failure (? within compliance case)
               }     
            }
            break;
                         
        case PART_COMPLIANCE_FAILURE:
             digitalWrite(DO_Part_Failure == HIGH);
             state = START_TRAY_MOVE_OUT;
             break;
            

        case START_WIPER_MOVE_OUT:
            ss_wiper.setupMoveInMilimeter(480);
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
            ss_wiper.setupMoveInMilimeter(-480);
            state = FINISH_WIPER_MOVE_IN;
            break;

        case FINISH_WIPER_MOVE_IN:   
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                // MATT CODE
                if (digitalRead(DI_HOME_WIPER) == true){   // an extra check to see if a misstep occured
                    flag_inStep == false;
                }
                state =  START_TRAY_MOVE_OUT;
            }
            break;

        case START_TRAY_MOVE_OUT:
            ss_tray.setupMoveInMilimeter(980);
            state = FINISH_TRAY_MOVE_OUT;
            break;

        case FINISH_TRAY_MOVE_OUT:   
            ss_wiper.processMovement();
            if (ss_wiper.motionComplete()){
                state =  WAIT_TO_START;
            }
            break;

         case START_TRAY_MOVE_IN:
                ss_tray.setupMoveInMilimeter(-980);
                state = FINISH_TRAY_MOVE_IN;
            break;

        case FINISH_TRAY_MOVE_IN:   
            ss_tray.processMovement();
            if (ss_tray.motionComplete()){
                // MATT CODE
                if (digitalRead(DI_HOME_WIPER) == true){   // an extra check to see if a misstep occured
                    flag_inStep == false;
                }
                state =  START_GANTRY_MOVE;
            }
            break;

        default:
            // error so do what every needs to be done
            delay(20);
            break;

    }



}
