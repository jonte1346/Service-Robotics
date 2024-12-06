// Movement functions
void moveForward() {
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
}

void moveForwardIntersection() {
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
  delay(200);
}

void spin(){
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
}
void turnLeft() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  changeOrientation(true);
  delay(900);
}

void turnRight() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  changeOrientation(false);
  delay(900);
}

void turnAround() {
  Serial.println("Turn Around");
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  changeOrientation(true);
  changeOrientation(true);
  delay(1500);
}

void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void moveForwardBlind() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(2000);
}

void turnLeftBlind() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(500);
  motorLeft.setSpeed(-50);
  motorRight.setSpeed(50);
  changeOrientation(true);
  delay(800);
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(500);
}

void turnRightBlind() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(1000);
  motorLeft.setSpeed(50);
  motorRight.setSpeed(-50);
  changeOrientation(true);
  delay(800);
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100);
  delay(1000);
}

void turnAroundBlind() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  changeOrientation(true);
  changeOrientation(true);
  delay(1100);
}