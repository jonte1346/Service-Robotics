#include <Servo.h>
#include <NewPing.h>

#define Front_Trigger_Pin 12
#define Front_Echo_Pin 7
#define MAX_DISTANCE 200

NewPing sonarFront(Front_Trigger_Pin, Front_Echo_Pin, MAX_DISTANCE);

Servo arm; 
Servo gripper;
//Servo objects for digital output 9 and 10
int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  arm.attach(9);
  gripper.attach(10);
  arm.write(125);  //start angle arm, 170 degrees is straight forward and 0 degrees is straight backwards
  gripper.write(90);   //start angle gripper, 180 degrees is fully open and 0 is a little more than closed
}

void loop() {
  // put your main code here, to run repeatedly:
  int distance = sonarFront.ping_cm();
  Serial.println(distance);
  if(Serial.find("c")) {
    gripAndRelease();
  }
}

void gripAndRelease() {
  for(pos = arm.read(); pos <= 170 ; pos += 1) {
      arm.write(pos);  
      delay(10);
    }
    delay(100);
    gripper.write(5);
    delay(500);
    for(pos = arm.read(); pos >= 75 ; pos -= 1) {
      arm.write(pos);  
      delay(10);
    }
    delay(500);
    gripper.write(90);
    delay(500);
    for(pos = arm.read(); pos <= 125 ; pos += 1) {
      arm.write(pos);  
      delay(10);
    }
}
