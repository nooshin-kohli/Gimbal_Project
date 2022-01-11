/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */
#include <MadgwickAHRS.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
Madgwick filter;
long timer = 0;
float roll_angle,pitch_angle;
double gyrox,gyroy,gyroz;
double accx,accy,accz;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  filter.begin(50);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();
  gyrox = mpu.getGyroX();
  gyroy = mpu.getGyroY();
  gyroz = mpu.getGyroZ();
  accx = mpu.getAccX();
  accy = mpu.getAccY();
  accz = mpu.getAccZ();
  filter.updateIMU(gyrox,gyroy,gyroz,accx,accy,accz);
  roll_angle = filter.getRoll();
  pitch_angle = filter.getPitch();
  Serial.print("pitch:  ");
  Serial.print(pitch_angle);
  Serial.print("   |roll: ");
  Serial.println(roll_angle);

//  if(millis() - timer > 1000){ // print data every second
//    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
//    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
//    Serial.print("\tY: ");Serial.print(mpu.getAccY());
//    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
//  
//    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
//    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
//    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
//    
//    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
//    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
//    
//    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
//    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
//    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
//    Serial.println(F("=====================================================\n"));
//    timer = millis();
    
    
    
//  }

}
