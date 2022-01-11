///////////// simple calibration: drift in angle and gyro velocity

#include <Wire.h>
const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float angleXaccel, angleYaccel, velocityXgyro, velocityYgyro, velocityZgyro;

void ErrorCalibration()
{
  float accelConversionRatio = 16384.0;
  float gyroConversionRatio = 131.0;
  int attempts = 1000;

  for (int counter = 0; counter < attempts; counter++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / accelConversionRatio;
    AccY = (Wire.read() << 8 | Wire.read()) / accelConversionRatio;
    AccZ = (Wire.read() << 8 | Wire.read()) / accelConversionRatio;
    angleXaccel += atan(AccY / AccZ) * 180 / PI;
    angleYaccel += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
  }

  angleXaccel /= attempts;
  angleYaccel /= attempts;

  for (int counter = 0; counter < attempts; counter++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    velocityXgyro += ((Wire.read() << 8 | Wire.read()) / gyroConversionRatio);
    velocityYgyro += ((Wire.read() << 8 | Wire.read()) / gyroConversionRatio);
    velocityZgyro += ((Wire.read() << 8 | Wire.read()) / gyroConversionRatio);
  }

  velocityXgyro /= attempts;
  velocityYgyro /= attempts;
  velocityZgyro /= attempts;

  Serial.print("angleXaccel: ");
  Serial.println(angleXaccel,3);
  Serial.print("angleYaccel: ");
  Serial.println(angleYaccel,3);
  Serial.print("velocityXgyro: ");
  Serial.println(velocityXgyro,3);
  Serial.print("velocityYgyro: ");
  Serial.println(velocityYgyro,3);
  Serial.print("velocityZgyro: ");
  Serial.println(velocityZgyro,3);
  Serial.println();
}
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void loop()
{
  ErrorCalibration();
  Serial.println("=====================");
}
