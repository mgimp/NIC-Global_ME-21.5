/*
BRIEF EXPLANATION FOR CLASSMATES
I made this file so that we could construct the statemachine from the ground up rather than editing Greg's files.
This ought to give us a greater understanding of the structure of the program, even if we end up copy/pasting
a significant amount of the code.

Seattle University
    Mechanical Engineering Department
        Engineering Design Group 21.5: NIC Global
            Vision System for Silkscreen Inspection
        David Schulman, Linh Ngo, Matthew Gimpelevich, Robert Wheeler
27FEB2021

THIS CODE is meant to service the Vision System for Silkscreen Inspection as designed by group 21.5
It uses a basic state machine to control a camera, gantry systems, limit switch, and operating consol using an Arduino Mega and the SpeedyStepper library Copyright (c) 2014 Stanley Reifel & Co.
Camera: 
Actuator Model: FUYU FSL Series
Stepper Motor: Nema 23
Brain: Arduino Mega
*/

// -----EQUIPMENT NOTES-----
// Limit switches probably fail LOW

// -----MOVEMENT-----
// LOW = AWAY FROM MOTOR
// HIGH = TOWARD MOTOR
// NEGATIVE VALUES = TOWARD MOTOR
// POSITIVE VALUES = AWAY FROM MOTOR