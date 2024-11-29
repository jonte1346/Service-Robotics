#include <Servo.h>

Servo arm; 
Servo gripper;
//Servo objects for digital output 9 and 10
int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  arm.attach(9);
  gripper.attach(10);
  arm.write(45);  //start angle arm
  gripper.write(180);   //start angle gripper
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.find("s")) {
    for(pos = arm.read(); pos >= 0 ; pos -= 1) {
      arm.write(pos);  
      delay(10);
    }
    gripper.write(30);
    delay(500);
    for(pos = arm.read(); pos <= 90 ; pos += 1) {
      arm.write(pos);  
      delay(15);
    }
    gripper.write(180);
    delay(500);
    arm.write(45);
  } 

}
