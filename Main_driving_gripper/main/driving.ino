// Movement functions
void moveForward() {
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
}

void spin(){
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
}
void turnLeft() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  changeOrientation(true);
  delay(400);
}

void turnRight() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  changeOrientation(false);
  delay(400);
}

void turnAround() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  changeOrientation(true);
  changeOrientation(true);
  delay(900);
}

void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}


