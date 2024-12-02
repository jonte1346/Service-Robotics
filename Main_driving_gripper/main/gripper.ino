//Servo objects for arm and gripper
Servo arm; 
Servo gripper;

int pos = 0;  //angle tracker

void gripperSetup(){
  arm.attach(9);
  gripper.attach(10);
  arm.write(125);  //start angle arm, 170 degrees is straight forward and 0 degrees is straight backwards
  gripper.write(180);   //start angle gripper, 180 degrees is fully open and 0 is a little more than closed
}

//call the following function when a cylinder has been detected
void gripAndRelease() {
  for(pos = arm.read(); pos <= 170 ; pos += 1) {
      arm.write(pos);  
      delay(10);
    }
    delay(100);
    gripper.write(5);
    delay(500);
    for(pos = arm.read(); pos >= 70 ; pos -= 1) {
      arm.write(pos);  
      delay(10);
    }
    delay(500);
    gripper.write(180);
    delay(500);
    for(pos = arm.read(); pos <= 125 ; pos += 1) {
      arm.write(pos);  
      delay(10);
    }
}