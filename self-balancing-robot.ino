#include <Wire.h>

// MPU6050
const int MPU = 0x68;  // Default I2C address
const int MPU_SDA = 17;
const int MPU_SCL = 18;

// Variables
int16_t accXLSB, accYLSB, accZLSB;
float accX, accY, accZ;
float accBiasX = -0.06;
float accBiasY = 0.01;
float accBiasZ = 0.03;

int16_t gyroXLSB, gyroYLSB, gyroZLSB;
float gyroX, gyroY, gyroZ;

float gyroCalibrationX = 0;
float gyroCalibrationY = 0;
float gyroCalibrationZ = 0;

float gyroBiasX = -0.40;
float gyroBiasY = 0.44;
float gyroBiasZ = 1.12;

void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL, 400000); // Start I2C on fast mode (400 kHz)
  
  setupMPU();
  // calibrateGyro();
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
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set gyro range to +/- 250 deg/s
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set accel range to +/- 2g
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readAccel() {
  // Read accelerometer registers
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(MPU, 6);
  accXLSB = Wire.read() << 8 | Wire.read();
  accYLSB = Wire.read() << 8 | Wire.read();
  accZLSB = Wire.read() << 8 | Wire.read();

  // Convert to g
  accX = accXLSB / 16384.0 + accBiasX;
  accY = accYLSB / 16384.0 + accBiasY;
  accZ = accZLSB / 16384.0 + accBiasZ;
}

void readGyro() {
  // Read gyro registers
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(MPU, 6);
  gyroXLSB = Wire.read() << 8 | Wire.read();
  gyroYLSB = Wire.read() << 8 | Wire.read();
  gyroZLSB = Wire.read() << 8 | Wire.read();

  // Convert to deg/s
  gyroX = gyroXLSB / 131.0 + gyroBiasX;
  gyroY = gyroYLSB / 131.0 + gyroBiasY;
  gyroZ = gyroZLSB / 131.0 + gyroBiasZ;
}

void calibrateGyro() {
  // Average 2000 gyro readings while MPU6050 is stationary
  for (int i = 0; i < 2000; i++) {
    readGyro();
    gyroCalibrationX += gyroX;
    gyroCalibrationY += gyroY;
    gyroCalibrationZ += gyroZ;
    delay(1);
  }

  gyroCalibrationX /= 2000;
  gyroCalibrationY /= 2000;
  gyroCalibrationZ /= 2000;

  Serial.print("Gyro Bias X = ");
  Serial.print(-gyroCalibrationX);
  Serial.print(", Gyro Bias Y = ");
  Serial.print(-gyroCalibrationY);
  Serial.print(", Gyro Bias Z = ");
  Serial.println(-gyroCalibrationZ);
}
