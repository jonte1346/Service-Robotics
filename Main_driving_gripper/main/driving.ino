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
  delay(200);
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
  delay(1700);
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
  delay(1600);
}
void moveForwardBlindAgain() {
  Serial.println("moveForwardBlind");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(1400);
}

void turnLeftBlind() {
  Serial.println("turnLeftBlind");
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(800);
}

void turnRightBlind() {
  Serial.println("turnRightBlind");
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(800);
}
