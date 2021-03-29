#include <SpeedyStepper.h>

//Pins
// First Actuator 900 mm
const int DIR1 = 1; //Initialize I/O pin 1 as DIR1
const int PUL1 = 2; //Initialize I/O pin 2 as PUL1
// Second Actuator 350 mm
const int DIR2 = 3; //Initialize I/O pin 3 as DIR2
const int PUL2 = 4; //Initialize I/O pin 4 as PUL2
// Limit Switch for 900 mm
const int LSwitch1 = 33;

SpeedyStepper stepperX; //Creates the object for the 900 mm
SpeedyStepper stepperY; //Creates the object for the 350 mm


void setup() {
  // put your setup code here, to run once:
  
  stepperX.connectToPins(PUL1,DIR1); //Set up 900 mm pins
  stepperY.connectToPins(PUL2,DIR2); //Set up 350 mm pins
  //Set up speed and acceleration for 900 mm
  stepperX.setSpeedInStepsPerSecond(8000);
  stepperX.setAccelerationInStepsPerSecondPerSecond(9500);
  //Set up speed and acceleration for 350 mm
  stepperY.setSpeedInStepsPerSecond(8000);
  stepperY.setAccelerationInStepsPerSecondPerSecond(9500);

//  pinMode(LSwitch1, INPUT_PULLUP);
//  Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:

  delay(20000);

  stepperY.moveRelativeInSteps(12250); //Extends 350 mm carriage
  
  for (int i=0; i < 3; i++){
    stepperX.moveRelativeInSteps(10500); //Extends 900mm carriage

    delay(100);
  }

  stepperY.moveRelativeInSteps(-8000);

  for (int i=0; i < 3; i++){
   stepperX.moveRelativeInSteps(-10500);

    delay(100);
  }

  stepperY.moveRelativeInSteps(-4250);

}
