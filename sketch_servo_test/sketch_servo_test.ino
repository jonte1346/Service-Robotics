#include <Servo.h>

Servo servoOutput9;
Servo servoOutput10;
int pos = 0;
int topPos = 90;
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
  if(Serial.find("s")) {
    servoOutput9.write(80);
    delay(500);
    for(pos = 0; pos <= topPos ; pos += 1) {
      servoOutput10.write(pos);  
      delay(15);
    }
    delay(500);
    for(pos = topPos; pos >= 0 ; pos -= 1) {
      servoOutput10.write(pos);  
      delay(15);
    }
    servoOutput9.write(0);
  } 

}
