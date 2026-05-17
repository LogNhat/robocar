#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int numSensors = 8;
const int sensorPins[numSensors] = {26, 25, 33, 32, 35, 34, 39, 36};

// Giữ nguyên ngưỡng bạn đang dùng
const int thresholdValues[numSensors] = {3185, 3327, 3091, 3071,
                                         3192, 3113, 3450, 2787};

// TB6612FNG
const int PWMA = 27;
const int AIN1 = 18;
const int AIN2 = 19;

const int PWMB = 13;
const int BIN1 = 5;
const int BIN2 = 23;

// =====================================================
// Smooth PD + Gyro Damping + Continuous Adaptive Speed
// =====================================================
// Error range khoảng -3500..3500
// correction = Kp*error + Kd*dError - Kg*gyroZ
// gyroZ đơn vị rad/s, dùng để hãm xoay quá đà.

float lineKp = 0.070;       // tăng nếu xe vào cua thiếu, giảm nếu lắc mạnh
float lineKd = 0.360;       // tăng nếu xe văng cua, giảm nếu bị giật
float gyroDamping = 34.0;   // tăng nếu xe xoay tròn/quá đà, giảm nếu cua bị ì

float gyroZOffset = 0.0;

// Speed tuning
int maxSpeed = 255;         // đường thẳng
int minSpeed = 135;         // cua gắt vẫn giữ lực kéo
int searchSpeed = 95;       // tìm line chậm để không xoay quá đà

int currentSpeed = 160;
int speedStepUp = 7;        // tăng tốc mượt hơn
int speedStepDown = 35;     // giảm tốc nhanh khi gặp cua

// Giảm tốc liên tục theo error, đạo hàm và gyro
float curveErrorGain = 0.030;   // giảm tốc theo độ lệch line
float curveDGian = 0.018;       // giảm tốc theo tốc độ biến thiên error
float curveGyroGain = 18.0;     // giảm tốc khi xe đang xoay nhanh

// Giới hạn correction theo độ cua
int softTurnLimit = 85;
int midTurnLimit = 155;
int hardTurnLimit = 225;

// Lọc mượt error
float filteredError = 0;
float errorAlpha = 0.42;     // thấp hơn = mượt hơn, cao hơn = nhạy hơn

// Lọc mượt derivative để bớt giật motor
float filteredDerivative = 0;
float derivativeAlpha = 0.35;

int lastError = 0;
bool lineDetected = false;
bool sensorBlack[numSensors];

void setupMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
#else
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWMA, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(PWMB, 1);
#endif
}

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWrite(PWMB, speed);
#else
  ledcWrite(1, speed);
#endif
}

void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWrite(PWMA, speed);
#else
  ledcWrite(0, speed);
#endif
}

void rampSpeedTo(int targetSpeed) {
  if (currentSpeed < targetSpeed) {
    currentSpeed += speedStepUp;
    if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
  } else if (currentSpeed > targetSpeed) {
    currentSpeed -= speedStepDown;
    if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
  }
}

void calibrateGyro() {
  Serial.println("Calibrating gyro... keep robot still");

  sensors_event_t accel, gyro, temp;
  float sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z;
    delay(2);
  }

  gyroZOffset = sum / samples;

  Serial.print("gyroZOffset = ");
  Serial.println(gyroZOffset, 6);
}

float readGyroZ() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  return gyro.gyro.z - gyroZOffset; // rad/s
}

int readLineError() {
  long weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < numSensors; i++) {
    int value = analogRead(sensorPins[i]);
    sensorBlack[i] = value < thresholdValues[i];

    if (sensorBlack[i]) {
      int reversedIndex = numSensors - 1 - i;
      weightedSum += reversedIndex * 1000;
      activeCount++;
    }
  }

  lineDetected = activeCount > 0;

  if (lineDetected) {
    int position = weightedSum / activeCount;
    return position - 3500;
  }

  return lastError;
}

void searchLine() {
  currentSpeed = searchSpeed;

  // Dùng hướng lệch cuối cùng để tìm lại line, không đổi hướng liên tục
  if (lastError > 0) {
    setMotorLeft(searchSpeed);
    setMotorRight(-searchSpeed);
  } else {
    setMotorLeft(-searchSpeed);
    setMotorRight(searchSpeed);
  }
}

void followLineSmoothGyro() {
  int rawError = readLineError();

  if (!lineDetected) {
    searchLine();
    return;
  }

  // Lọc error để xe không giật theo nhiễu analog
  filteredError = filteredError * (1.0 - errorAlpha) + rawError * errorAlpha;
  int error = (int)filteredError;

  int rawDerivative = error - lastError;
  filteredDerivative = filteredDerivative * (1.0 - derivativeAlpha) + rawDerivative * derivativeAlpha;
  float dError = filteredDerivative;

  float gyroZ = readGyroZ();

  // PD bám line + gyro damping chống xoay quá đà
  float correctionFloat = lineKp * error + lineKd * dError - gyroDamping * gyroZ;
  int correction = (int)correctionFloat;

  int absError = abs(error);

  if (absError < 650) {
    correction = constrain(correction, -softTurnLimit, softTurnLimit);
  } else if (absError < 1700) {
    correction = constrain(correction, -midTurnLimit, midTurnLimit);
  } else {
    correction = constrain(correction, -hardTurnLimit, hardTurnLimit);
  }

  // Tốc độ giảm mượt theo độ cua, tốc độ đổi hướng và tốc độ xoay thật
  int targetSpeed = maxSpeed
                    - (int)(curveErrorGain * absError)
                    - (int)(curveDGian * abs(dError))
                    - (int)(curveGyroGain * abs(gyroZ));

  targetSpeed = constrain(targetSpeed, minSpeed, maxSpeed);
  rampSpeedTo(targetSpeed);

  int leftSpeed = currentSpeed + correction;
  int rightSpeed = currentSpeed - correction;

  // Cho phép active braking nhẹ khi cua rất gắt, nhưng không quá sâu để tránh xoay tròn
  leftSpeed = constrain(leftSpeed, -95, 255);
  rightSpeed = constrain(rightSpeed, -95, 255);

  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);

  lastError = error;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 NOT FOUND!");
    while (1) delay(100);
  }

  Serial.println("MPU6050 CONNECTED!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // nhanh hơn 21Hz nhưng vẫn lọc nhiễu tốt

  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  setupMotors();
  calibrateGyro();

  Serial.println("START SMOOTH PD + GYRO DAMPING + ADAPTIVE SPEED");
  delay(500);
}

void loop() {
  followLineSmoothGyro();
}
