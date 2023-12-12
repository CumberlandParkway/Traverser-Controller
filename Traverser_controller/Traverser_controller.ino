// Include servo driver libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define the PCA9685 board address
Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40); //setup the first board address

// Define arduino pin connections
#define TrackPlus 2
#define TrackMinus 3
#define TrackButtonA 4
#define TrackButtonB 5
#define TrackButtonC 6
#define TrackButtonD 7
#define TrackButtonE 8
#define TrackButtonF 9
#define TrackButtonG 10
#define DecoupleButton 11

#define TraverserDir A0
#define TraverserStep A1
#define TraverserEnablePin A2
#define TraverserLimitSwitch A3

int TrackPos[] = {0,2075,4185,6285,8385,10465,12550}; // array to hold predefined track positions

int TraverserHomeSteps = 480;  // adjust this value to get initial start position
int TraverserNormalSpeed = 1000;
int TraverserSlowSpeed = 3000;
int CurrentTraverserPosition = 0; // home position
int DecouplerPosition = 0; // 0=closed/down 1=thrown/up
int ButtonDelay = 300;

//Common variables
int i = 0;
int x = 0;

void TraverserHome() {
// Enable the stepper motor
digitalWrite(TraverserEnablePin, LOW);

// Move motor at normal speed until rough home position reached
digitalWrite(TraverserDir, HIGH); // towards home position
while (digitalRead(TraverserLimitSwitch) == 1) {
digitalWrite(TraverserStep, HIGH);
delayMicroseconds(TraverserNormalSpeed);
digitalWrite(TraverserStep, LOW);
delayMicroseconds(TraverserNormalSpeed);
}
delay(1000);
// reverse stepper for 100 steps
digitalWrite(TraverserDir, LOW); // away from home position
for(i = 0; i < 400; i++) {
digitalWrite(TraverserStep,HIGH); 
delayMicroseconds(TraverserNormalSpeed); 
digitalWrite(TraverserStep,LOW); 
delayMicroseconds(TraverserNormalSpeed); 
}
delay(1000);
// move slowing towards home position
digitalWrite(TraverserDir, HIGH);
while (digitalRead(TraverserLimitSwitch) == 1) {
digitalWrite(TraverserStep, HIGH);
delayMicroseconds(TraverserSlowSpeed);
digitalWrite(TraverserStep, LOW);
delayMicroseconds(TraverserSlowSpeed);
}
delay(1000);
// back away from microswitch to set home position
digitalWrite(TraverserDir, LOW);
for(i = 0; i < TraverserHomeSteps; i++) {
digitalWrite(TraverserStep,HIGH); 
delayMicroseconds(TraverserNormalSpeed); 
digitalWrite(TraverserStep,LOW); 
delayMicroseconds(TraverserNormalSpeed); 
}
// Disable the stepper motor
digitalWrite(TraverserEnablePin, HIGH);
}

void GoToPos(int Position){
// Enable the stepper motor
digitalWrite(TraverserEnablePin, LOW);
if(Position == CurrentTraverserPosition){  //DO NOTHING
}
else{
if(Position > CurrentTraverserPosition){
digitalWrite(TraverserDir, LOW); // away from home position
}
else{
digitalWrite(TraverserDir, HIGH); // towards home position
}
int Diff = abs(Position - CurrentTraverserPosition);
for(x = 0; x < Diff; x++) {
digitalWrite(TraverserStep,HIGH); 
delayMicroseconds(1000); 
digitalWrite(TraverserStep,LOW); 
delayMicroseconds(1000);
}
CurrentTraverserPosition = Position;
//Serial.println(CurrentTraverserPosition);
}
// Disable the stepper motor
digitalWrite(TraverserEnablePin, HIGH);
}

void setup() {
// set pin modes for buttons, relays and stepper controls
pinMode(TraverserDir, OUTPUT);
pinMode(TraverserStep, OUTPUT);
pinMode(TraverserEnablePin, OUTPUT);
pinMode(TrackPlus, INPUT_PULLUP);
pinMode(TrackMinus, INPUT_PULLUP);
pinMode(TrackButtonA, INPUT_PULLUP);
pinMode(TrackButtonB, INPUT_PULLUP);
pinMode(TrackButtonC, INPUT_PULLUP);
pinMode(TrackButtonD, INPUT_PULLUP);
pinMode(TrackButtonE, INPUT_PULLUP);
pinMode(TrackButtonF, INPUT_PULLUP);
pinMode(TrackButtonG, INPUT_PULLUP);
pinMode(DecoupleButton, INPUT_PULLUP);
pinMode(TraverserLimitSwitch, INPUT_PULLUP);

//INITIALISE PCA9685 BOARD
Serial.begin(19200);
pwm0.begin();
pwm0.setPWMFreq(50);  // This is the maximum PWM frequency
// set servo to closed position
pwm0.writeMicroseconds(0, 1000);

// disable the stepper motors
digitalWrite(TraverserEnablePin, HIGH);

// Home the traverser
TraverserHome();
}

void loop() {
// Decoupler servo control
if(digitalRead(DecoupleButton) == 0){
if(DecouplerPosition == 0){
pwm0.writeMicroseconds(0,1450);
DecouplerPosition = 1;
delay(ButtonDelay);
}
else{
pwm0.writeMicroseconds(0,1000);
DecouplerPosition = 0;
delay(ButtonDelay);
}
}

// Traverser control
if(digitalRead(TrackMinus) == 0) {
GoToPos(CurrentTraverserPosition-5);
}
if(digitalRead(TrackPlus) == 0) {
GoToPos(CurrentTraverserPosition+5);
}
if(digitalRead(TrackButtonG) == 0) {
GoToPos(TrackPos[6]);
}
if(digitalRead(TrackButtonF) == 0) {
GoToPos(TrackPos[5]);
}
if(digitalRead(TrackButtonE) == 0) {
GoToPos(TrackPos[4]);
}
if(digitalRead(TrackButtonD) == 0) {
GoToPos(TrackPos[3]);
}
if(digitalRead(TrackButtonC) == 0) {
GoToPos(TrackPos[2]);
}
if(digitalRead(TrackButtonB) == 0) {
GoToPos(TrackPos[1]);
}
if(digitalRead(TrackButtonA) == 0) {
GoToPos(TrackPos[0]);
}
delay(ButtonDelay);
}
