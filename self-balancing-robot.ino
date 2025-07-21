#include <Wire.h>

// MPU6050 I2C
const int MPU = 0x68;  // Default I2C address
const int MPU_SDA = 17;
const int MPU_SCL = 18;

// TB6612FNG
const int PWM_A = 4;
const int AIN_2 = 5;
const int AIN_1 = 6;
const int STBY = 7;
const int BIN_1 = 15;
const int BIN_2 = 16;
const int PWM_B = 17;

// Motorrrr
const int MOTOR_A = 0;
const int MOTOR_B = 1;

// Accelerometer
int16_t accelXLSB, accelYLSB, accelZLSB;
float accelX, accelY, accelZ, accelAngle;

float accelBiasX = -0.06;
float accelBiasY = 0.01;
float accelBiasZ = 0.03;

// Gyroscope
int16_t gyroXLSB, gyroYLSB, gyroZLSB;
float gyroX, gyroY, gyroZ;

float gyroCalibrationX = 0;
float gyroCalibrationY = 0;
float gyroCalibrationZ = 0;

float gyroBiasX = -0.40;
float gyroBiasY = 0.44;
float gyroBiasZ = 1.12;

// Loop timing
uint32_t previousTime;
int timeDelta = 4000; // in microseconds, 250 Hz

// Kalman filter
float kalmanAngle = 0; // Inital angle estimate
float kalmanUncertainty = 2 * 2; // Variance of initial angle estimate
float gyroUncertainty = 4 * 4;
float accelUncertainty = 3 * 3;
float kalmanGain;

void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL, 400000); // Start I2C on fast mode (400 kHz)
  
  setupMPU();
  // calibrateGyro();

  setupTB6612FNG();

  previousTime = micros();
}

void loop() {
  readAccel();
  readGyro();

  // Pitch angle estimate from accelerometer
  accelAngle = atan2(-accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * (180 / 3.14159);
  
  // Kalman filter
  kalmanAngle = kalmanAngle + (timeDelta / 1e6) * gyroY;
  kalmanUncertainty = kalmanUncertainty + pow(timeDelta / 1e6, 2) * gyroUncertainty;
  kalmanGain = kalmanUncertainty / (kalmanUncertainty + accelUncertainty);
  kalmanAngle = kalmanAngle + kalmanGain * (accelAngle - kalmanAngle);
  kalmanUncertainty = (1 - kalmanGain) * kalmanUncertainty;

  Serial.print("Accel_angle:");
  Serial.print(accelAngle);
  Serial.print(" Kalman_angle:");
  Serial.println(kalmanAngle);

  while (micros() - previousTime < timeDelta); // Wait
  previousTime = micros();

  // Testing motors
  setMotorSpeed(0,-100);
  setMotorSpeed(1,100);
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

void setupTB6612FNG() {
  pinMode(PWM_A, OUTPUT);
  pinMode(AIN_2, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void readAccel() {
  // Read accelerometer registers
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(MPU, 6);
  accelXLSB = Wire.read() << 8 | Wire.read();
  accelYLSB = Wire.read() << 8 | Wire.read();
  accelZLSB = Wire.read() << 8 | Wire.read();

  // Convert to g
  accelX = accelXLSB / 16384.0 + accelBiasX;
  accelY = accelYLSB / 16384.0 + accelBiasY;
  accelZ = accelZLSB / 16384.0 + accelBiasZ;
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

void setMotorSpeed (int motor, int speed) {

  int IN_1;
  int IN_2;
  int PWM;

  if(motor == MOTOR_A) {
    IN_1 = AIN_1;
    IN_2 = AIN_2;
    PWM = PWM_A;
  } else {
    IN_1 = BIN_1;
    IN_2 = BIN_2;
    PWM = PWM_B;
  }

  constrain(speed, -255, 255);
  
  if (speed > 0) {
    // FORWARD!
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
    analogWrite(PWM, speed);
  } else if (speed < 0) {
    // BACKWARDS!
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
    analogWrite(PWM, -speed);
  } else {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, LOW);
  }

}








