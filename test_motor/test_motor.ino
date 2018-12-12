#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install

AF_DCMotor motorEsq(3, MOTOR12_64KHZ);
AF_DCMotor motorDir(4, MOTOR12_64KHZ);

void setup() {
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
  Serial.begin(9600);
}

void loop() {
 motorDir.setSpeed(20);
 motorEsq.setSpeed(20); 
 motorDir.run(FORWARD);
 motorEsq.run(FORWARD);
  
}
