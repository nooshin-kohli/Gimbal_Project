// home gimbal and testing motors
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo_t;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(5);  // attaches the servo on pin 9 to the servo object
  myservo_t.attach(3);
}

void loop() {
  myservo.write(90);
  myservo_t.write(90);
  
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    myservo_t.write(pos);
//    delay(15);                       // waits 15 ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    myservo_t.write(pos);
//    delay(15);                       // waits 15 ms for the servo to reach the position
//  }
}
