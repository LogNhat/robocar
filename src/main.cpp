#include <Arduino.h>

// ================= PIN =================

// QTR-8A: mắt đang lắp ngược
const int sensorPins[8] = {36, 39, 34, 35, 32, 33, 25, 26};

int thresholds[8] = {2953, 3545, 3268, 3368, 3223, 3275, 3452, 3265};

const int weights[8] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000};

// TB6612
#define PWMA 27
#define AIN1 18
#define AIN2 19

#define PWMB 13
#define BIN1 5
#define BIN2 23

// Tắt laser
#define XSHUT_LEFT 15
#define XSHUT_FRONT 16
#define XSHUT_RIGHT 17

// ================= DEBUG =================

#define DEBUG 0 // 0 = chạy nhanh, 1 = in Serial để tune

// ================= THÔNG SỐ TINH CHỈNH =================

// Dùng integer cho nhanh
// Kp = 35 nghĩa là 0.035
// Kd = 80 nghĩa là 0.080
int Kp = 40;
int Kd = 80;

int maxCorrection = 130;

// tốc độ
int SPEED_MAX = 130;
int SPEED_FAST = 120;
int SPEED_MID = 110;
int SPEED_CURVE = 100;
int SPEED_SEARCH = 90;

// ================= BIẾN =================

int lastError = 0;
int currentSpeed = 100;
int lastDirection = 1;

// ================= MOTOR =================

void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  }
}

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  }
}

void setMotors(int left, int right) {
  motorLeft(left);
  motorRight(right);
}

// ================= LINE SENSOR =================

int readLine(int &error) {
  long weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < 8; i++) {
    int raw = analogRead(sensorPins[i]);

    // raw < threshold => thấy line đen
    if (raw < thresholds[i]) {
      weightedSum += weights[i];
      activeCount++;
    }
  }

  if (activeCount == 0)
    return 0; // Mất line (cả 8 mắt đều thấy trắng)
    
  if (activeCount == 8) // CHỈ khi đủ 8 mắt đều thấy đen thì mới coi là vạch đích
    return 2; // Vạch đích

  int position = weightedSum / activeCount;

  // Sensor mắt lắp ngược
  error = 3500 - position;

  if (error > 50)
    lastDirection = 1;
  else if (error < -50)
    lastDirection = -1;

  return 1; // Đang bám line bình thường
}

// ================= SPEED =================

int chooseTargetSpeed(int error) {
  int e = abs(error);

  if (e < 400)
    return SPEED_MAX;
  if (e < 1000)
    return SPEED_FAST;
  if (e < 2200)
    return SPEED_MID;

  return SPEED_CURVE;
}

// Không ramp để phản ứng nhanh nhất
void updateSpeed(int targetSpeed) { currentSpeed = targetSpeed; }

// ================= PD =================

void followLine() {
  int error = 0;
  int lineStatus = readLine(error);

  // Vạch đích (cả 8 mắt đều đen): Dừng hẳn
  if (lineStatus == 2) {
    setMotors(-currentSpeed, -currentSpeed); // Phanh nhẹ trước khi dừng hẳn
    delay(30);
    setMotors(0, 0);
    
    Serial.println("=== FINISH LINE DETECTED - STOPPED ===");
    while (true) {
      setMotors(0, 0); // Khóa xe dừng vĩnh viễn ở đây
      delay(100);
    }
  }

  static bool wasOnLine = true;

  // Mất line: hãm phanh và xoay tìm lại NGAY LẬP TỨC
  if (lineStatus == 0) {
    if (wasOnLine) {
      // Phanh phản lực cực gắt để dập quán tính trong 15ms
      setMotors(-255, -255);
      delay(15); 
      
      wasOnLine = false;
    }

    currentSpeed = SPEED_SEARCH;

    // Phản xạ xoay ngay lập tức không có độ trễ
    if (lastDirection > 0) {
      setMotors(-SPEED_SEARCH, SPEED_SEARCH);
    } else {
      setMotors(SPEED_SEARCH, -SPEED_SEARCH);
    }

    return;
  }

  wasOnLine = true;

  int dError = error - lastError;

  // PD integer:
  // Kp=35, Kd=80 tương đương 0.035 và 0.080
  int correction = (Kp * error + Kd * dError) / 1000;

  correction = constrain(correction, -maxCorrection, maxCorrection);

  int targetSpeed = chooseTargetSpeed(error);
  updateSpeed(targetSpeed);

  int leftSpeed = currentSpeed - correction;
  int rightSpeed = currentSpeed + correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotors(leftSpeed, rightSpeed);

  lastError = error;

#if DEBUG
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 200) {
    lastLog = millis();

    Serial.printf("Err:%5d | D:%5d | Corr:%4d | Spd:%3d | L:%3d | R:%3d\n",
                  error, dError, correction, currentSpeed, leftSpeed,
                  rightSpeed);
  }
#endif
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  delay(300);

  // ADC ESP32
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);

  // Tắt laser VL53L0X
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);

  // Motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  setMotors(0, 0);

  // Sensor
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

#if DEBUG
  Serial.println("PD ULTRA FAST READY");
#endif
}

// ================= LOOP =================

void loop() {
  // Gửi ký tự 'r' qua Serial Monitor để reset ESP32 từ xa
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      Serial.println("=== SOFTWARE RESET ===");
      setMotors(0, 0);
      delay(100);
      ESP.restart();
    }
  }

  followLine();
}