/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

Servo pitch;
Servo roll;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  pitch.attach(5); //servo motor for pitch
  roll.attach(3);  //servo motor for roll
  Serial.begin(9600);
  pitch.write(88);
  roll.write(80);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
  Serial.print("X : ");
  Serial.print(mpu.getAngleX());
  roll.write(80+mpu.getAngleX());
  Serial.print("\tY : ");
  Serial.print(mpu.getAngleY());
  pitch.write(88-mpu.getAngleY());
//  Serial.print("\tZ : ");
//  Serial.println(mpu.getAngleZ());
  timer = millis();  
  }
}
