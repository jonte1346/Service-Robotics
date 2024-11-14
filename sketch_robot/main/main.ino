#include <NewPing.h>
#include "CytronMotorDriver.h"

// Ultrasonic sensor pins
#define FRONT_TRIGGER_PIN  12
#define FRONT_ECHO_PIN     11
#define RIGHT_TRIGGER_PIN  8
#define RIGHT_ECHO_PIN     7
#define LEFT_TRIGGER_PIN   4
#define LEFT_ECHO_PIN      2
#define MAX_DISTANCE       200

NewPing sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

// Motor configuration using CytronMotorDriver library
CytronMD motorLeft(PWM_PWM, 3, 9);   // PWM 1A = Pin 3, PWM 1B = Pin 9 (Left motor)
CytronMD motorRight(PWM_PWM, 10, 11); // PWM 2A = Pin 10, PWM 2B = Pin 11 (Right motor)

// Movement modes
enum MovementMode { FORWARD, TURN_LEFT, TURN_RIGHT, TURN_AROUND, STOP };
MovementMode currentMode = FORWARD;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read distances from the ultrasonic sensors
  int distanceFront = sonarFront.ping_cm();
  int distanceRight = sonarRight.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();

  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Right: ");
  Serial.print(distanceRight);
  Serial.print(" cm, Left: ");
  Serial.println(distanceLeft);
  Serial.println(" cm");

  // Decision-making based on sensor readings
  if (distanceFront > 0 && distanceFront < 30) {
    if (distanceRight > 0 && distanceRight < 30 && distanceLeft > 0 && distanceLeft < 30) {
      // Obstacles in all three directions
      setMovementMode(TURN_AROUND);
    } else if (distanceRight > 0 && distanceRight < 30) {
      // Obstacle in front and on the right
      setMovementMode(TURN_LEFT);
    } else if (distanceLeft > 0 && distanceLeft < 30) {
      // Obstacle in front and on the left
      setMovementMode(TURN_RIGHT);
    } else {
      // Obstacle in front only
      setMovementMode(TURN_LEFT);
    }
  } else {
    // No obstacle in front
    setMovementMode(FORWARD);
  }

  // Execute movement based on the current mode
  switch (currentMode) {
    case FORWARD:
      moveForward();
      break;
    case TURN_LEFT:
      turnLeft();
      break;
    case TURN_RIGHT:
      turnRight();
      break;
    case TURN_AROUND:
      turnAround();
      break;
    case STOP:
      stopMotors();
      break;
  }
}

// Movement functions using CytronMotorDriver library
void moveForward() {
  motorLeft.setSpeed(150);   // Move left motor forward at medium speed
  motorRight.setSpeed(150);  // Move right motor forward at medium speed
}

void turnLeft() {
  motorLeft.setSpeed(-100);  // Move left motor backward at low speed
  motorRight.setSpeed(100);  // Move right motor forward at low speed
}

void turnRight() {
  motorLeft.setSpeed(100);   // Move left motor forward at low speed
  motorRight.setSpeed(-100); // Move right motor backward at low speed
}

void turnAround() {
  motorLeft.setSpeed(-150);  // Move left motor backward at medium speed
  motorRight.setSpeed(150);  // Move right motor forward at medium speed
  delay(1000);               // Turn for 1 second (adjust as needed)
}

void stopMotors() {
  motorLeft.setSpeed(0);     // Stop left motor
  motorRight.setSpeed(0);    // Stop right motor
}

// Function to change the movement mode
void setMovementMode(MovementMode mode) {
  currentMode = mode;
}
