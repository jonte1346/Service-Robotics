#include <Servo.h>

Servo servoOutput9;
Servo servoOutput10;
int timer = 5000;
int prevTime = 0;
//Servo objects for digital output 9 and 10

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servoOutput9.attach(9);
  servoOutput10.attach(10);
  servoOutput9.write(0);
  servoOutput10.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - prevTime > timer && servoOutput9.read() == 0) {
    servoOutput9.write(90);
    servoOutput10.write(90);
    prevTime = millis();
  } 

  if(millis() - prevTime > timer && servoOutput9.read() == 90) {
    servoOutput9.write(0);
    servoOutput10.write(0);
    prevTime = millis();
  } 

}
