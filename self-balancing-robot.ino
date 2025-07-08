#include <Wire.h>

const int MPU_SDA = 17;
const int MPU_SCL = 18;

int16_t accXLSB, accYLSB, accZLSB;
float accX, accY, accZ;

int16_t gyroXLSB, gyroYLSB, gyroZLSB;
float gyroX, gyroY, gyroZ;

void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL);
  setupMPU();
}

void loop() {
  readAccel();
  readGyro();
  Serial.print("Gyro X = ");
  Serial.print(gyroX);
  Serial.print(" Gyro Y = ");
  Serial.print(gyroY);
  Serial.print(" Gyro Z = ");
  Serial.print(gyroZ);
  Serial.print(" Acc X = ");
  Serial.print(accX);
  Serial.print(" Acc Y = ");
  Serial.print(accY);
  Serial.print(" Acc Z = ");
  Serial.println(accZ);
  delay(1000);
}

void setupMPU() {
  // Wake up
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set gyro range to +/- 250 deg/s
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set accel range to +/- 2g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readAccel() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  accXLSB = Wire.read() << 8 | Wire.read();
  accYLSB = Wire.read() << 8 | Wire.read();
  accZLSB = Wire.read() << 8 | Wire.read();

  accX = accXLSB / 16384.0;
  accY = accYLSB / 16384.0;
  accZ = accZLSB / 16384.0;
}

void readGyro() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  gyroXLSB = Wire.read() << 8 | Wire.read();
  gyroYLSB = Wire.read() << 8 | Wire.read();
  gyroZLSB = Wire.read() << 8 | Wire.read();

  gyroX = gyroXLSB / 131.0;
  gyroY = gyroYLSB / 131.0;
  gyroZ = gyroZLSB / 131.0;
}
