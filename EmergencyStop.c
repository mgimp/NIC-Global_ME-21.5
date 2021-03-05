// this should be incorporated into the state machine
// what is the state that gets you into this state
// how do you exit this state

// do we need an EMERGENCY_STOP_INPROGRESS state that is where you land to wait for the E.S. to exit?

// REFERENCE OF VARIABLES FROM statemachine.c
// -----SPEEDYSTEPPER VARIABLES-----
// Uses a typedef provided by SpeedyStepper
SpeedyStepper ss_gantryy;  // 900mm
SpeedyStepper ss_gantryx;  // 350mm
SpeedyStepper ss_wiper; // 500mm
SpeedyStepper ss_tray;  // 1000mm

void EmergencyStop(void)
{
    // turn on RED LED indicating users have pressed emergency stop button
    LED_System_Failure(1);
    
    // setupStop() will move the target position 
    // so that the motor will begin deceleration now
    ss_gantryy.setupStop();
    ss_gantryx.setupStop();
    ss_wiper.setupStop();
    ss_tray.setupStop();

    // I don't believe the following code is necessary
    // the setupStop function should ensure the motors are stopped
        
    // processMovement() returns true if movement complete, 
    // and false not a final target 
    // position yet
    if (ss_gantryy.processMovement() == true)
    {
        // motor is completely stop
        // nothing is done in here so we don't need these checks
    }
    if (ss_gantryx.processMovement() == true)
    {
        // motor is completely stop
    }
    if (ss_wiper.processMovement() == true)
    {
        // motor is completely stop
    }
    if (ss_tray.processMovement() == true)
    {
        // motor is completely stop
    }
}
