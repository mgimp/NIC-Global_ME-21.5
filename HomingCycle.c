// SETUP STEPPER MOTOR VARIABLES
#include SpeedyStepper.h

// DEFINE GANTRY NAMES
SpeedyStepper gantryLong;  // The big one
SpeedyStepper gantryShort;  // The small one
SpeedyStepper pusher;
SpeedyStepper tray;

// SETUP CAMERA CONTROLS
// Assumes controlled by two pins
void cameraRead(){
    return 1;
}
void cameraWrite(){
    return 1;
}

// SETUP HOMING SWITCH VARIABLES
bool gantryLong_Switch = digitalRead(i);
bool gantryShort_Swtich = digitalRead(j);
bool pusher_Switch = digitalRead(k);
bool tray_Switch = digitalRead(l);

// SETUP CONTROLS
bool onButton = digitalRead(m);
bool stopButton = digitalRead(n);

// LEDs
// Write 0 or 1 for LED functions to control output
void LED_Ready(x){
    pinMode(o, OUTPUT);
    digitalWrite(o, x);
    return 1;
}
void LED_InProgress(x){
    pinMode(p, OUTPUT);
    digitalWrite(p, x);
    return 1;
}
void LED_Failure(x){
    pinMode(q, OUTPUT);
    digitalWrite(q, x);
    return 1;
}
void LED_Error(x){
    pinMode(r, OUTPUT);
    digitalWrite(r, x);
    return 1;
}

Setup() {
    SpeedyStepper.SpeedyStepper()

    // STEPPER MOTOR SETUP
    gantryLong.connectToPins(a,b);
    gantryShort.connectToPins(c,d);
    pusher.connectToPins(e,f);
    tray.connectToPins(g,h);

    // SETUP STEPPER MOTOR SPEEDS AND ACCELERATIONS
    // CONVERSION: 17.5 steps/mm
    // Vmax = 8000 steps/s
    // Amax = 9500 steps/s^2
    gantryLong.setStepsPerMillimeter(17.5);
    gantryShort.setStepsPerMillimeter(17.5);
    pusher.setStepsPerMillimeter(17.5);
    tray.setStepsPerMillimeter(17.5);

    gantryLong.setSpeedInMillimetersPerSecond(8000/17.5);
    gantryLong.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);

    gantryShort.setSpeedInMillimetersPerSecond(8000/17.5);
    gantryShort.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);

    pusher.setSpeedInMillimetersPerSecond(8000/17.5);
    pusher.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);

    tray.setSpeedInMillimetersPerSecond(8000/17.5);
    tray.setAccelerationInMillimetersPerSecondPerSecond(9500/17.5);

    // STARTING ALL LEDs OFF
    LED_Ready(0);
    LED_InProgress(0);
    LED_Failure(0);
    LED_Error(0);

}

Loop() {

    switch (currState){
            case HOMING_CYCLE:
            // Starts at max speed
            // ASSUMING that a value of 1 from a switch means that it IS being pressed
            // ASSUMING that relative motion has (positive = away from motor) (negative = toward motor)
            // QUESTION: Should we slow down the gantries prior to seeking the switch?

                // LED STUFF
                LED_Ready(0);
                LED_InProgress(1);
                LED_Failure(0);
                LED_Error(0);

                // MAKE SURE ALL GANTRIES ARE AT THE STARTING POSITION
                int mmCount = 0;
                while((gantryLong_Switch < 1) && (stopButton < 1)){
                    gantryLong.moveRelativeInMillimeters(-10);
                    // ASSUMES that we are able to catch motion complete every time
                    if(gantryLong.motionComplete()){
                        mmCount += 1;
                    }
                    elif (mmCount > 90){
                        currState = HOMING_ERROR;
                        // ASSUMES THIS WILL THIS IMMEDIATELY BREAK THE HOMING_CYCLE CASE
                        break;
                    }
                }
                gantryLong.setupStop();
                mmCount = 0;
            
                while((gantryShort_Switch < 1) && (stopButton < 1)){
                    gantryShort.moveRelativeInMillimeters(-10);
                    // ASSUMES that we are able to catch motion complete every time
                    if(gantryShort.motionComplete()){
                        mmCount += 1;
                    }
                    elif (mmCount > 35){
                        currState = HOMING_ERROR;
                        // ASSUMES THIS WILL THIS IMMEDIATELY BREAK THE HOMING_CYCLE CASE
                        break;
                    }
                }
                gantryShort.setupStop();
                mmCount = 0;

                while((pusher_Switch < 1) && (stopButton < 1)){
                    pusher.moveRelativeInMillimeters(-10);
                    // ASSUMES that we are able to catch motion complete every time
                    if(pusher.motionComplete()){
                        mmCount += 1;
                    }
                    elif (mmCount > 50){
                        currState = HOMING_ERROR;
                        // ASSUMES THIS WILL THIS IMMEDIATELY BREAK THE HOMING_CYCLE CASE
                        break;
                    }
                }
                pusher.setupStop();
                mmCount = 0;

                while((tray_Switch < 1) && (stopButton < 1)){
                    tray.moveRelativeInMillimeters(-10);
                    // ASSUMES that we are able to catch motion complete every time
                    if(tray.motionComplete()){
                        mmCount += 1;
                    }
                    elif (mmCount > 100){
                        currState = HOMING_ERROR;
                        // ASSUMES THIS WILL THIS IMMEDIATELY BREAK THE HOMING_CYCLE CASE
                        break;
                    }
                }
                tray.setupStop();

                // SET STARTING POSITION TO 0
                gantryLong.setCurrentPositionInMillimeters(0);
                gantryShort.setCurrentPositionInMillimeters(0);
                pusher.setCurrentPositionInMillimeters(0);
                tray.setCurrentPositionInMillimeters(0);

                currState = GET_READY;
            
            break;
    }
}