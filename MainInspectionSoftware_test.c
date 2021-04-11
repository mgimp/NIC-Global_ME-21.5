#include "SpeedyStepper.h"
#include "Arduino.h"

#define Test 13
#define DO_Ready 14
#define DO_Part_Failure 15
#define DO_WIP 16
#define DO_System_Failure 17

void setup(){
  pinMode(Test,INPUT);
  pinMode(DO_Ready,OUTPUT);
  pinMode(DO_Part_Failure,OUTPUT);
  pinMode(DO_WIP,OUTPUT);
  pinMode(DO_System_Failure,OUTPUT);
}

void loop(){
  if(digitalRead(Test)){
    digitalWrite(DO_Ready,HIGH);
    delay(500);
    digitalWrite(DO_Part_Failure,HIGH);
    delay(500);
    digitalWrite(DO_WIP,HIGH);
    delay(500);
    digitalWrite(DO_System_Failure,HIGH);
    delay(500);
  }
  else{
    digitalWrite(DO_Ready,LOW);
    delay(500);
    digitalWrite(DO_Part_Failure,LOW);
    delay(500);
    digitalWrite(DO_WIP,LOW);
    delay(500);
    digitalWrite(DO_System_Failure,LOW);
    delay(500);
  }
}
