#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install

AF_DCMotor motorEsq(4, MOTOR12_64KHZ);
AF_DCMotor motorDir(3, MOTOR12_64KHZ);
void setup() {
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
  Serial.begin(9600);
}
  
void loop() {
 motorDir.setSpeed(60);
 motorEsq.setSpeed(60); 
 motorDir.run(FORWARD);
 motorEsq.run(FORWARD);
  
}
