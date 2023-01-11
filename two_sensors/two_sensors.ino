#include<Wire.h>
const int MPU_addr_1 = 0x68; // I2C address of the first MPU-6050
const int MPU_addr_2 = 0x69; // I2C address of the second MPU-6050
int16_t AcX1, AcY1, AcZ1, Tmp1, AcX2, AcY2, AcZ2, Tmp2; // definition of variables
double Acc_angle_x,Acc_angle_y;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr_1);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr_2);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(MPU_addr_1);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr_1, 14, true); // request a total of 14 registers
  float AcX1 = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  float AcY1 = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  float AcZ1 = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp1 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  Wire.beginTransmission(MPU_addr_2);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr_2, 14, true); // request a total of 14 registers
  float AcX2 = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  float AcY2= Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  float AcZ2 = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp2 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  Serial.print("IMU_one: ");
  Serial.print(AcX1 / 16384);
  Serial.print(" "); 
  Serial.print(AcY1 / 16384);
  Serial.print(" ");
  Serial.print(AcZ1 / 16384);
  Serial.print(" ");
  Serial.print(Tmp1 / 340.00 + 36.53);
  Serial.print(" ");
  Serial.print("IMU_two: ");
  Serial.print(AcX2 / 16384);
  Serial.print(" "); 
  Serial.print(AcY2 / 16384);
  Serial.print(" ");
  Serial.print(AcZ2 / 16384);
  Serial.print(" ");
  Serial.println(Tmp2 / 340.00 + 36.53);
//  Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) ;
// /*---Y---*/
//  Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) ;
//  delay(500);
}
