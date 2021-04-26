// this should be incorporated into the state machine
// what is the state that gets you into this state
// how do you exit this state

// do we need an EMERGENCY_STOP_INPROGRESS state that is where you land to wait for the E.S. to exit?

// REFERENCE OF VARIABLES FROM statemachine.c
// -----SPEEDYSTEPPER VARIABLES-----
// Uses a typedef provided by SpeedyStepper
//SpeedyStepper ss_gantryy;  // 900mm
//SpeedyStepper ss_gantryx;  // 350mm
//SpeedyStepper ss_wiper; // 500mm
//SpeedyStepper ss_tray;  // 1000mm

case EMERGENCY_STOP_START:
    LED_System_Failure(1); // turn on the RED LED when E.stop button is pressed
        // Assign IO pins used for Step and Direction
        // Stop command from Arduino to motors by setting all pinmode to output
    

    //update to new case and then break
    currState = EMERGENCY_STOP_INPROGRESS; //go to emergency stop progress state
    break; 

case EMERGENCY_STOP_INPROGRESS:
    // Decelerating all four motors to zero velocity
    ss_gantryy.setupStop();
    ss_gantryx.setupStop();
    ss_wiper.setupStop();
    ss_tray.setupStop();

    //if all 4 motors stop, update the currState to exit state
    if (ss_gantryy.processMovement() &&  ss_gantryx.processMovement() && 
    ss_wiper.processMovement() && ss_tray.processMovement())
    {
        currState = EMERGENCY_STOP_EXIT; 
    }
    break;

case EMERGENCY_STOP_EXIT
    // home the motors, update the currState to Homing Cycle and reset the whole system
    initializeMotorSpeeds();
    currState = START_HOMING_CYCLE; 

    break;







