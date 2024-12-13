#include <NewPing.h>
#include "CytronMotorDriver.h"
#include <QTRSensors.h>
#include <Servo.h>

// Distance PINS
#define FRONT_TRIGGER_PIN 13
#define RIGHT_TRIGGER_PIN 12
#define LEFT_TRIGGER_PIN 4
#define FRONT_ECHO_PIN 7
#define RIGHT_ECHO_PIN 8
#define LEFT_ECHO_PIN 2
#define MAX_DISTANCE 200

#define MOTOR1A 3
#define MOTOR1B 5
#define MOTOR2A 11
#define MOTOR2B 6

#define buttonPin 13

int distanceFront = 0;
int distanceRight = 0;
int distanceLeft = 0;

const int NUM_MEASUREMENTS = 5;
int distanceFrontAvg[NUM_MEASUREMENTS];
int distanceRightAvg[NUM_MEASUREMENTS];
int distanceLeftAvg[NUM_MEASUREMENTS];
uint8_t i_front = 0;
uint8_t i_right = 0;
uint8_t i_left = 0;
uint8_t temp = 0;
int error = 0;
int fAvg = 0;
int lAvg = 0;
int rAvg = 0;

int rescuedCylinders = 0;
int turnNR = 0; //6 for three way blind
uint16_t position;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// Line-following thresholds
const uint16_t LINE_THRESHOLD = 650; // Adjust based on calibration
const uint16_t CENTER_POSITION = 2500;

// Orientation
enum Orientation {FORWARD = 0, LEFT = 1, RIGHT = 2, BACKWARD = 3};
enum LineType { STRAIGHT, LEFT_TURN, RIGHT_TURN, INTERSECTION, NONE};

QTRSensors qtr;

// Motor configuration
CytronMD motorLeft(PWM_PWM, MOTOR1A, MOTOR1B);
CytronMD motorRight(PWM_PWM, MOTOR2A, MOTOR2B);

// Ultrasonic sensors
NewPing sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

// Arrays for turn indices 1 = left 2 = right 3 = straigth forward
Orientation movements[] = {LEFT, LEFT, LEFT, LEFT, FORWARD, FORWARD, LEFT, LEFT, LEFT, FORWARD, LEFT, LEFT, LEFT, RIGHT, LEFT, LEFT, LEFT, LEFT, FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, BACKWARD, LEFT, RIGHT}; 

// Setup
void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  int buttonState = digitalRead(buttonPin);
  while(buttonState != LOW){
    buttonState = digitalRead(buttonPin);
  }

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // configure the gripper
  gripperSetup();
  spin();
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  stopMotors();
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  int loopButtonState = digitalRead(buttonPin);
  while(loopButtonState != LOW){
    loopButtonState = digitalRead(buttonPin);
  }
}

// Main loop
void loop() {
  // Step 1: Cylinder detection and rescue
  if (detectCylinder()) { // && lineType != NONE
    rescueCylinder();
    delay(100);
  }

  // Step 2: Read QTR sensors
  position = qtr.readLineBlack(sensorValues); // 0 for sensor 0, 1000 for sensor 1, 2000 for sensor 2 etc.
  LineType lineType = detectLineType(sensorValues, LINE_THRESHOLD);

  measureFront();
  measureLeft();
  measureRight();

  fAvg = calcAvgFront();
  lAvg = calcAvgLeft();
  rAvg = calcAvgRight();

  Serial.print(fAvg);
  Serial.print("\t");
  Serial.print(lAvg);
  Serial.print("\t");
  Serial.println(rAvg);
  Serial.print(distanceFrontAvg[i_front]);
  Serial.print("\t");
  Serial.print(distanceLeftAvg[i_left]);
  Serial.print("\t");
  Serial.print(distanceRightAvg[i_right]);
  Serial.print("\t");

  if (turnNR >= 26 && lineType == NONE) {
    stopMotors();
    while (true) {
    }
  }
  
  // Step 3: Handle Line Type
  switch (lineType) {
    case STRAIGHT:
      followLine(position); // Continue line-following
      break;

    case LEFT_TURN:
      if (rAvg > 20) {
        break;
      }

      if(fAvg > 40){
        for (int i = 0; i < 5; i++) {
          measureFront();
        }
        fAvg = calcAvgFront();
        if (fAvg > 40) {
          handleIntersection();
          break;
        }
      }
      if (fAvg < 22){

      turnLeft(); // Perform left turn
              }
      break;

      case RIGHT_TURN:
        if (lAvg > 20) {
          break;
        }

        if(fAvg > 40){
        for (int i = 0; i < 5; i++) {
          measureFront();
        }
        fAvg = calcAvgFront();
        if (fAvg > 40) {
            handleIntersection();
            break;
          }
        }
        if (fAvg < 22){

        turnRight(); // Perform right turn
        } 
        break;

      case INTERSECTION:
        handleIntersection(); 
        break;

      case NONE:
        handleNoLine();
        break;
    }
} //End of loop



LineType detectLineType(uint16_t *sensorValues, uint16_t threshold) {
  int activeSensors = 0;        // Count of sensors detecting the line
  int firstActive = -1;         // Index of the first sensor detecting the line
  int lastActive = -1;          // Index of the last sensor detecting the line

  // Analyze sensor values to find active sensors
  for (uint8_t i = 0; i < 6; i++) { // Assuming 6 sensors
    if (sensorValues[i] > threshold) {
      activeSensors++;
      if (firstActive == -1) {
        firstActive = i; // Mark the first active sensor
      }
      lastActive = i; // Continuously update the last active sensor
    }
  }

  // Determine line type
  if (activeSensors == 0) {
    return NONE; // No line detected
  } else if (activeSensors <= 3) { // Straight line case (at most 3 sensors)
    return STRAIGHT;
  } else if (activeSensors >= 4) { // At least 4 sensors detecting the line
    if (firstActive <= 2 && lastActive <= 3) {
      return RIGHT_TURN; // Line on the left (left turn)
    } else if (firstActive >= 2 && lastActive >= 3) {
      return LEFT_TURN; // Line on the right (right turn)
    } else {
      return INTERSECTION; // Line spans across center and edges
    }
  }

  return NONE; // Default case (shouldn't happen)
}

// Line-following logic
void followLine(uint16_t position) {
  int prev_error = error;
  error = position - CENTER_POSITION; // Calculate error relative to the center
  int derivative_error = error - prev_error;
  int turnSpeed = error / 16 + derivative_error/4; // Adjust motor speed based on error

  // Adjust motors to stay on the line
  motorLeft.setSpeed(150 - turnSpeed);
  motorRight.setSpeed(150 + turnSpeed);
}

void handleIntersection() {
  switch(movements[turnNR]){
    case FORWARD:
      moveForwardIntersection();
      break;
    case LEFT:
      turnLeft();
      break;
    case RIGHT:
      turnRight();
      break;
    case BACKWARD:
      turnAround();
      break;
  }
  turnNR++;
}


// Rescue logic
bool detectCylinder() {
  int objectDistance = sonarFront.ping_cm();
  return (objectDistance > 0 && objectDistance < 5);
}

void rescueCylinder() {
  stopMotors();
  delay(100);
  rescuedCylinders++;
  gripAndRelease();
}


void handleNoLine(){//int distanceFront, int distanceRight, int distanceLeft){
  stopMotors();
  for (int i = 0; i < 5 ; i++){
  measureFront();
  measureLeft();
  measureRight();
}
  fAvg = calcAvgFront();
  lAvg = calcAvgLeft();
  rAvg = calcAvgRight();
  if(fAvg < 30 && lAvg < 30 && rAvg < 30){
    turnAround();
    return;
  }
  
  if(turnNR < 5){
    moveForwardBlind();
    turnRightBlind();
    moveForwardBlind();
    position = qtr.readLineBlack(sensorValues); // 0 for sensor 0, 1000 for sensor 1, 2000 for sensor 2 etc.
    followLine(position);
    return;
  }else{
    if (turnNR < 7) {
      moveForwardBlindLong();
      turnLeftBlind1();
      turnNR++;
    } else if (turnNR > 8) {
      turnNR++;
      moveForwardBlind();
      turnLeftBlind2();
    }

    moveForwardBlindShort();

    for (int i = 0; i < 7; i++) {
      position = qtr.readLineBlack(sensorValues); // 0 for sensor 0, 1000 for sensor 1, 2000 for sensor 2 etc
      followLine(position);
      measureFront();
      measureLeft();
      measureRight();
    }
  }
}

void delayOrLine(uint16_t time){
  long timer_0 = millis();
  LineType line = NONE;

  while (millis() < (timer_0 + time) && line != STRAIGHT ){  
  position = qtr.readLineBlack(sensorValues); // 0 for sensor 0, 1000 for sensor 1, 2000 for sensor 2 etc.
  line = detectLineType(sensorValues, LINE_THRESHOLD);
  delay(30);
  }
}

void moveForwardBlindShort() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delayOrLine(1000);
  followLine(position);

}

// Movement functions
void moveForward() {
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
}

void moveForwardIntersection() {
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
  delay(100);
   followLine(position);

}

void spin(){
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
}
void turnLeft() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(500);
}
void turnRight() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(500);
}
void turnAround() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delayOrLine(1700);
}

void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void moveForwardBlind() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delayOrLine(1400);
}

void moveForwardBlindLong() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(1600);
}

void turnLeftBlind1() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(800);
}

void turnLeftBlind2() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(750);
}

void turnRightBlind() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(800);
}

void measureFront(){

  i_front++;
  if (i_front == NUM_MEASUREMENTS){
    i_front = 0;
  }
  distanceFrontAvg[i_front] = sonarFront.ping_cm(); 
}

void measureLeft(){
  i_left++;
  if (i_left == NUM_MEASUREMENTS){
    i_left = 0;
  }
  distanceLeftAvg[i_left] = sonarLeft.ping_cm(); 
}

void measureRight(){


  i_right++;
  if (i_right == NUM_MEASUREMENTS){
    i_right = 0;
  }
  distanceRightAvg[i_right] = sonarRight.ping_cm(); 
}


int calcAvgFront(){

  int sortedDistances[NUM_MEASUREMENTS];
    
  // Copy the array to avoid modifying the original
  for (int k = 0; k < NUM_MEASUREMENTS; k++) {
    sortedDistances[k] = distanceFrontAvg[k];
  }
  
  // Sort the array
  for (int k = 0; k < NUM_MEASUREMENTS - 1; k++) {
    for (int l = k + 1; l < NUM_MEASUREMENTS; l++) {
      if (sortedDistances[k] > sortedDistances[l]) {
        int temp = sortedDistances[k];
        sortedDistances[k] = sortedDistances[l];
        sortedDistances[l] = temp;
      }
    }
  }
  
  // Select the median 3 values (middle three values in the sorted array)
  int medianValues[3];
  medianValues[0] = sortedDistances[1];
  medianValues[1] = sortedDistances[2];
  medianValues[2] = sortedDistances[3];
  
  // Calculate the average of the median 3 values
  int sum = medianValues[0] + medianValues[1] + medianValues[2];
  int average = sum / 3;

  return average;
}

int calcAvgLeft(){

  int sortedDistances[NUM_MEASUREMENTS];
    
  // Copy the array to avoid modifying the original
  for (int k = 0; k < NUM_MEASUREMENTS; k++) {
    sortedDistances[k] = distanceLeftAvg[k];
  }
  
  // Sort the array
  for (int k = 0; k < NUM_MEASUREMENTS - 1; k++) {
    for (int l = k + 1; l < NUM_MEASUREMENTS; l++) {
      if (sortedDistances[k] > sortedDistances[l]) {
        int temp = sortedDistances[k];
        sortedDistances[k] = sortedDistances[l];
        sortedDistances[l] = temp;
      }
    }
  }
  
  // Select the median 3 values (middle three values in the sorted array)
  int medianValues[3];
  medianValues[0] = sortedDistances[1];
  medianValues[1] = sortedDistances[2];
  medianValues[2] = sortedDistances[3];
  
  // Calculate the average of the median 3 values
  int sum = medianValues[0] + medianValues[1] + medianValues[2];
  int average = sum / 3;

  return average;
}

int calcAvgRight(){

  int sortedDistances[NUM_MEASUREMENTS];
    
  // Copy the array to avoid modifying the original
  for (int k = 0; k < NUM_MEASUREMENTS; k++) {
    sortedDistances[k] = distanceRightAvg[k];
  }
  
  // Sort the array
  for (int k = 0; k < NUM_MEASUREMENTS - 1; k++) {
    for (int l = k + 1; l < NUM_MEASUREMENTS; l++) {
      if (sortedDistances[k] > sortedDistances[l]) {
        int temp = sortedDistances[k];
        sortedDistances[k] = sortedDistances[l];
        sortedDistances[l] = temp;
      }
    }
  }
  
  // Select the median 3 values (middle three values in the sorted array)
  int medianValues[3];
  medianValues[0] = sortedDistances[1];
  medianValues[1] = sortedDistances[2];
  medianValues[2] = sortedDistances[3];
  
  // Calculate the average of the median 3 values
  int sum = medianValues[0] + medianValues[1] + medianValues[2];
  int average = sum / 3;

  return average;
}

//Servo objects for arm and gripper
Servo arm; 
Servo gripper;

int pos = 0;  //angle tracker
uint8_t standby_angle = 90;
uint8_t gripp_angle = 170;
uint8_t release_angle = 75;

void gripperSetup(){
  arm.attach(9);
  gripper.attach(10);
  arm.write(standby_angle);  //start angle arm, 170 degrees is straight forward and 0 degrees is straight backwards
  gripper.write(90);   //start angle gripper, 180 degrees is fully open and 0 is a little more than closed
}

//call the following function when a cylinder has been detected
void gripAndRelease() {
  for(pos = arm.read(); pos <= gripp_angle ; pos += 1) {
      arm.write(pos);  
      delay(10);
    }
    delay(100);
    gripper.write(5);
    delay(500);
    for(pos = arm.read(); pos >= release_angle ; pos -= 1) {
      arm.write(pos);  
      delay(10);
    }
    delay(500);
    gripper.write(90);
    delay(500);
    for(pos = arm.read(); pos <= standby_angle ; pos += 1) {
      arm.write(pos);  
      delay(10);
    }
}

