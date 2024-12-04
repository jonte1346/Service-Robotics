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
  delay(900);
}

void turnRight() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  changeOrientation(false);
  delay(900);
}

void turnAround() {
  motorLeft.setSpeed(-150);
  motorRight.setSpeed(150);
  changeOrientation(true);
  changeOrientation(true);
  delay(1800);
}

void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}


