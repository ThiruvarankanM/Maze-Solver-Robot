#include <Wire.h>

#define MPU6050_ADDR 0x68

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);       // PWR_MGMT_1
  Wire.write(0x00);       // Wake up
  Wire.endTransmission();

  Serial.println("MPU6050 started");
}

void loop()
{
  int16_t AcX, AcY, AcZ;
  int16_t GyX, GyY, GyZ;
  int16_t Temp;

  // Start reading from ACCEL_XOUT_H
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // 14 bytes:
  // Accel X,Y,Z = 6
  // Temperature = 2
  // Gyro X,Y,Z = 6
  Wire.requestFrom(MPU6050_ADDR, 14);

  if (Wire.available() == 14)
  {
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    Temp = Wire.read() << 8 | Wire.read();

    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    Serial.print("ACC: ");
    Serial.print(AcX);
    Serial.print("  ");
    Serial.print(AcY);
    Serial.print("  ");
    Serial.print(AcZ);

    Serial.print(" | GYRO: ");
    Serial.print(GyX);
    Serial.print("  ");
    Serial.print(GyY);
    Serial.print("  ");
    Serial.println(GyZ);
  }

  delay(100);
}
