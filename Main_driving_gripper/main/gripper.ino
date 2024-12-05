//Servo objects for arm and gripper
Servo arm; 
Servo gripper;

int pos = 0;  //angle tracker
uint8_t standby_angle = 90;
uint8_t gripp_angle = 170;
uint8_t release_angle = 70;

void gripperSetup(){
  arm.attach(9);
  gripper.attach(10);
  arm.write(standby_angle);  //start angle arm, 170 degrees is straight forward and 0 degrees is straight backwards
  gripper.write(180);   //start angle gripper, 180 degrees is fully open and 0 is a little more than closed
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
    gripper.write(180);
    delay(500);
    for(pos = arm.read(); pos <= standby_angle ; pos += 1) {
      arm.write(pos);  
      delay(10);
    }
}