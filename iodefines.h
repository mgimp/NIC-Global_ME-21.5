// put all of the digital io defines here

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

