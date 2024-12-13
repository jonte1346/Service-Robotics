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

// int distanceFrontAvg 

int error = 0;

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
    Serial.println("WAIT FOR CALIBRATION");
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
  Serial.println("Robot Initialized");
  pinMode(buttonPin, INPUT_PULLUP);
  int loopButtonState = digitalRead(buttonPin);
  while(loopButtonState != LOW){
    Serial.println(loopButtonState);
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

  distanceFront = sonarFront.ping_cm();
  distanceRight = sonarRight.ping_cm();
  distanceLeft = sonarLeft.ping_cm();

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
      Serial.println("LEFT_TURN");
      if (distanceRight > 20) {
        break;
      }

      if(distanceFront > 40){
        int averageDistanceFront = 0;
        for (int i = 0; i < 3; i++) {
          distanceFront = sonarFront.ping_cm();
          averageDistanceFront += distanceFront;
        }
        if (averageDistanceFront/3 > 40) {
          Serial.println("INTERSECTION");
          handleIntersection();
          break;
        }
      }
      turnLeft(); // Perform left turn
      break;

      case RIGHT_TURN:
        Serial.println("RIGHT_TURN");
        if (distanceLeft > 20) {
          break;
        }

        if(distanceFront > 40){
          int averageDistanceFront = 0;
          for (int i = 0; i < 3; i++) {
            distanceFront = sonarFront.ping_cm();
            averageDistanceFront += distanceFront;
          }
          if (averageDistanceFront/3 > 40) {
            Serial.println("INTERSECTION");
            handleIntersection();
            break;
          }
        }
        turnRight(); // Perform right turn
        break;

      case INTERSECTION:
        Serial.println("INTERSECTION");
        handleIntersection(); 
        break;

      case NONE:
        handleNoLine();
        break;
    }

    

}



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
  int turnSpeed = error / 10 + derivative_error/4; // Adjust motor speed based on error

  // Adjust motors to stay on the line
  motorLeft.setSpeed(150 - turnSpeed);
  motorRight.setSpeed(150 + turnSpeed);
}

void handleIntersection() {
  Serial.println(turnNR);
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
  Serial.print("Cylinder rescued! Total rescued: ");
  Serial.println(rescuedCylinders);
  gripAndRelease();
}


void handleNoLine(){//int distanceFront, int distanceRight, int distanceLeft){
  stopMotors();
  int distanceFront = sonarFront.ping_cm();
  int distanceRight = sonarRight.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();

  if(distanceFront < 30 && distanceRight < 30 && distanceLeft < 30){
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
  }
  switch(movements[turnNR]){
    case LEFT:
      if (turnNR < 7) {
        moveForwardBlindLong();
        turnLeftBlind1();
      } else if (turnNR > 8) {
        moveForwardBlind();
        turnLeftBlind2();
      }
      moveForwardBlindShort();
      position = qtr.readLineBlack(sensorValues); // 0 for sensor 0, 1000 for sensor 1, 2000 for sensor 2 etc
      for (int i = 0; i < 3; i++) {
        followLine(position);
      }
      break;
  }
  if(turnNR > 5){
    turnNR++;
  }
}

void delayOrLine(uint16_t time){
  long timer_0 = millis();
  LineType line = NONE;

  while (millis() < (timer_0 + time) && line != STRAIGHT ){  
  //position = qtr.readLineBlack(sensorValues); // 0 for sensor 0, 1000 for sensor 1, 2000 for sensor 2 etc.
  line = detectLineType(sensorValues, LINE_THRESHOLD);
  delay(50);
  }
}

void moveForwardBlindShort() {
  Serial.println("moveForwardBlindShort");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delayOrLine(1000);
}

// Movement functions
void moveForward() {
  Serial.println("moveForward");
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
}

void moveForwardIntersection() {
  Serial.println("moveForwardIntersection");
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
  delay(100);
}

void spin(){
  Serial.println("spin");
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
}
void turnLeft() {
  Serial.println("turnLeft");
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(500);
}
void turnRight() {
  Serial.println("turnRight");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(500);
}
void turnAround() {
  Serial.println("Turn Around");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delayOrLine(1700);
}

void stopMotors() {
  Serial.println("stopMotor");
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void moveForwardBlind() {
  Serial.println("moveForwardBlind");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delayOrLine(1400);
}

void moveForwardBlindLong() {
  Serial.println("moveForwardBlindLong");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(1600);
}

void turnLeftBlind1() {
  Serial.println("turnLeftBlind");
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(800);
}

void turnLeftBlind2() {
  Serial.println("turnLeftBlind");
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(750);
}

void turnRightBlind() {
  Serial.println("turnRightBlind");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(800);
}