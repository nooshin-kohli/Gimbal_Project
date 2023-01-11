// home gimbal and testing motors
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo_t;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo_t.attach(3);
}

void loop() {
  myservo.write(90);
//    myservo.write(90);
  myservo_t.write(82);
}
