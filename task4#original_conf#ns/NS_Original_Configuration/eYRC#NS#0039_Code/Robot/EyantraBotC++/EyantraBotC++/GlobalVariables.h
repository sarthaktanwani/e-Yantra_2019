/*
* Team Id:#NS0039
* Author List: Madhur Raghav, Ayush Aggarwal, Prakhar Gupta, Sarthak Tanwani
* Filename: main.cpp
* Theme: Nutty Squirrel
* Functions: map(long, long, long, long, long)
* Global Variables: S0, S1, S2, S3, OUT, BLUE, GREEN, RED, BUILTIN_LED, LeftMotorPin1, LeftMotorPin2, RightMotorPin1, RightMotorPin2, LeftMotorPin1REG, LeftMotorPin2REG, 
					RightMotorPin1REG, RightMotorPin2REG, INT_MAX, palmPin, elbowPin, openPalm, closePalm, elbowUp, elbowDown, BuzzerPin, , PWM_MAX, V
*
*/


#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_

#define S0  0
#define S1  1
#define S2  2
#define S3  3
#define OUT 0
#define BLUE 5
#define GREEN 6
#define RED  7

#define  BUILTIN_LED     PB7           //PIN D13
#define LeftMotorPin1   PB5           //PIN D11
#define LeftMotorPin2   PB6           //PIN D12
#define RightMotorPin1    PE3           //PIN D5
#define RightMotorPin2    PE4           //PIN D2
#define LeftMotorPin1REG  OCR1A
#define LeftMotorPin2REG  OCR1B
#define RightMotorPin1REG OCR3A
#define RightMotorPin2REG OCR3B
#define	INT_MAX				65535

#define palmPin		PL3
#define elbowPin	PH3
#define openPalm	40
#define closePalm	90
#define elbowUp		0
#define elbowDown	90
#define BuzzerPin	PC1			//Buzzer pin PC1 or pin 54 of the Arduino Mega

#define PWM_MAX       1023

//Number of vertices in the graph
#define V 28

long map(long, long, long, long, long);

#endif /* GLOBALVARIABLES_H_ */