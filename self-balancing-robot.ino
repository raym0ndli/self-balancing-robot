#include <Wire.h>
#include <LedControl.h>

// MPU6050 I2C
const int MPU = 0x68;  // Default I2C address
const int MPU_SDA = 12;
const int MPU_SCL = 13;

// LED pins
const int CS2 = 41;
const int CLK = 48;
const int CS1 = 47;
const int DIN = 21;

// LED matrix 1 - DIN, CLK, CS
LedControl lc1 = LedControl(DIN, CLK, CS1, 1);

// LED matrix 2
LedControl lc2 = LedControl(DIN, CLK, CS2, 1);

// Eye Modes
// R0 to R7
byte circle[8] = {0, 0, 0b00011000, 0b00111100, 0b00111100, 0b00011000, 0, 0};
byte blink[8] = {0, 0, 0, 0b00100100, 0b00011000, 0, 0, 0 };
byte heart[8] = {0b00011000, 0b00111100, 0b01111110, 0b11111111, 0b11111111, 0b11111111, 0b01100110, 0};
byte happyface[8] = {0b00111100, 0b01000010, 0b10011001, 0b10100101, 0b10000001, 0b10100101, 0b01000010, 0b00111100};

const int left = 0;
const int right = 1;

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
const int OFFSET_A = 1;
const int OFFSET_B = -1;

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

// PID controller
float kp = 60;
float ki = 0;
float kd = 0;

float desiredAngle = -2;
float error;
float prevError = 0;
float prevI = 0;
float motorInput;

void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL, 400000); // Start I2C on fast mode (400 kHz)
  
  setupMPU();
  // calibrateGyro();
  setupTB6612FNG();
  setupLed();
  previousTime = micros();
}

void loop() {
  updateKalman();
  updatePID();
  updateMotors();

  Serial.println(kalmanAngle);

  while (micros() - previousTime < timeDelta); // Wait
  previousTime = micros();

  eyeMode(blink, left);
  eyeMode(heart, right);
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

void updateKalman() {
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
}

void updatePID() {
  error = desiredAngle - kalmanAngle;
  float P = kp * error;
  float I = prevI + ki * (prevError + error) / 2 * (timeDelta / 1e6);
  float D = kd * (error - prevError) / (timeDelta / 1e6);
  motorInput = P + I + D;

  prevError = error;
  prevI = I;
}

void updateMotors() {
  if (abs(kalmanAngle) < 20 && abs(motorInput) > 50) {
    setMotorSpeed(MOTOR_A, motorInput);
    setMotorSpeed(MOTOR_B, motorInput);
  } else {
    setMotorSpeed(MOTOR_A, 0);
    setMotorSpeed(MOTOR_B, 0);
  }
}

void setMotorSpeed (int motor, int speed) {
  int IN_1;
  int IN_2;
  int PWM;
  int OFFSET;

  if(motor == MOTOR_A) {
    IN_1 = AIN_1;
    IN_2 = AIN_2;
    PWM = PWM_A;
    OFFSET = OFFSET_A;
  } else {
    IN_1 = BIN_1;
    IN_2 = BIN_2;
    PWM = PWM_B;
    OFFSET = OFFSET_B;
  }

  speed *= OFFSET;
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

void setupLed() {
  pinMode(DIN, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);

  lc1.shutdown(0,false);
  // Set brightness to a medium value
  lc1.setIntensity(0,1);
  // Clear the display
  lc1.clearDisplay(0);

  lc2.shutdown(0,false);
  // Set brightness to a medium value
  lc2.setIntensity(0,1);
  // Clear the display
  lc2.clearDisplay(0);
}

void eyeMode(byte eyeMode_values[], int side){
  for (int i = 0; i < 8; i += 1) {
    if (side == left) {
      lc1.setColumn(0, i, eyeMode_values[i]);
    } else {
      lc2.setColumn(0, i, eyeMode_values[i]);
    }
  }
}
