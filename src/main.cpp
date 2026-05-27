#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// 1. CẤU HÌNH CHÂN (PIN CONFIGURATION)
// ============================================================================

#define XSHUT_CENTER 15
#define XSHUT_LEFT 17
#define XSHUT_RIGHT 16

#define ADDR_CENTER 0x30
#define ADDR_LEFT 0x31
#define ADDR_RIGHT 0x32

// ============================================================================
// CẤU HÌNH SAI SỐ HIỆU CHUẨN CẢM BIẾN LASER (mm)
// ============================================================================
const int OFFSET_CENTER = -55;
const int OFFSET_LEFT = -20;  // Cảm biến TRÁI (lắp vuông góc với tường)
const int OFFSET_RIGHT = -25; // Cảm biến PHẢI (lắp vuông góc với tường)

// Động cơ TB6612FNG
const int PWMA = 27; // Tốc độ bánh Phải (Kênh A)
const int AIN1 = 18;
const int AIN2 = 19;
const int PWMB = 13; // Tốc độ bánh Trái (Kênh B)
const int BIN1 = 14;
const int BIN2 = 23; // Chiều bánh Trái1

const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ============================================================================
// 2. THAM SỐ THUẬT TOÁN BÁM TƯỜNG TRÁI (CASCADE PID: WALL & GYRO)
// ============================================================================

Adafruit_MPU6050 mpu;
Adafruit_VL53L0X lox_center = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();

// Kích thước vật lý
const float CAR_WIDTH = 120.0;  // mm
const float CAR_LENGTH = 140.0; // mm

// --- THAM SỐ BÁM TƯỜNG ---
// Khoảng cách mục tiêu tới tường trái (mm) - đo được ~40mm khi xe ở giữa
const int WALL_TARGET = 40;

// Ngưỡng dừng khẩn cấp khi tường trước quá gần (mm)
const int FRONT_STOP_DIST = 50;

// Tốc độ cơ bản (mặc định là 90 theo yêu cầu)
const int BASE_SPEED = 50;

// --- HỆ SỐ CASCADE PID (WALL & GYRO) ---
// Hệ số vòng ngoài (Wall Error -> Target Heading)
float wallKp = 1.2; // 1mm lệch tương đương 1.2 độ lệch hướng mục tiêu

// Hệ số vòng trong (Heading Error -> Motor Speed Correction)
float gyroKp = 3.5; // Lực phản hồi bám góc hướng
float gyroKd =
    0.5; // Lực cản tốc độ xoay (Damping) để xe đi siêu thẳng, không lắc hông

// --- BỘ LỌC EMA ---
const float EMA_ALPHA = 0.35;
float smoothLeft = 40.0;
float smoothCenter = 380.0;
float smoothRight = 40.0;

// Biến Gyro
float gyroZOffset = 0.0;
float gyroZDeg = 0.0; // Vận tốc góc deg/s hiện tại
float currentHeading = 0.0;
unsigned long lastGyroTime = 0;

// ============================================================================
// 3. ĐIỀU KHIỂN ĐỘNG CƠ (TB6612FNG) — MOTOR ENABLED
// ============================================================================

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

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

// ============================================================================
// 4. CẢM BIẾN LASER VL53L0X
// ============================================================================

bool isDevicePresent(byte address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

void initLaserSensors() {
  Serial.println(
      "\n[SYSTEM] Khởi động các cảm biến Laser (Chế độ Chủ động OUTPUT)...");

  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(150);

  // CENTER
  pinMode(XSHUT_CENTER, OUTPUT);
  digitalWrite(XSHUT_CENTER, HIGH);
  delay(450);
  if (!isDevicePresent(0x29)) {
    Serial.println("❌ Cảm biến CENTER không thức dậy ở 0x29!");
    while (1)
      delay(200);
  }
  if (!lox_center.begin(ADDR_CENTER, false, &Wire)) {
    Serial.println("❌ Không thể cấu hình Center ở 0x30");
    while (1)
      delay(100);
  }
  Serial.println("✓ CENTER (0x30)");

  // LEFT
  pinMode(XSHUT_LEFT, OUTPUT);
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(450);
  if (!isDevicePresent(0x29)) {
    Serial.println("❌ Cảm biến LEFT không thức dậy ở 0x29!");
    while (1)
      delay(200);
  }
  if (!lox_left.begin(ADDR_LEFT, false, &Wire)) {
    Serial.println("❌ Không thể cấu hình Left ở 0x31");
    while (1)
      delay(100);
  }
  Serial.println("✓ LEFT (0x31)");

  // RIGHT
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(450);
  if (!lox_right.begin(ADDR_RIGHT, false, &Wire)) {
    Serial.println("❌ Không thể cấu hình Right ở 0x32");
    while (1)
      delay(100);
  }
  Serial.println("✓ RIGHT (0x32)");

  Serial.println("[SYSTEM] Khởi động thành công 3 cảm biến! 🎉\n");
}

int readDistanceCenter() {
  VL53L0X_RangingMeasurementData_t measure;
  lox_center.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter + OFFSET_CENTER;
  }
  return 9999;
}

int readDistanceLeft() {
  VL53L0X_RangingMeasurementData_t measure;
  lox_left.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter + OFFSET_LEFT;
  }
  return 9999;
}

int readDistanceRight() {
  VL53L0X_RangingMeasurementData_t measure;
  lox_right.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter + OFFSET_RIGHT;
  }
  return 9999;
}

// ============================================================================
// 5. CẢM BIẾN GYRO MPU6050
// ============================================================================

void calibrateGyro() {
  Serial.println("[GYRO] Hiệu chuẩn... giữ robot đứng yên...");
  sensors_event_t accel, gyro, temp;
  float sum = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z;
    delay(2);
  }
  gyroZOffset = sum / samples;
  Serial.printf("[GYRO] Offset Z: %.6f\n", gyroZOffset);
}

void updateHeading() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  gyroZDeg = (gyro.gyro.z - gyroZOffset) * 57.2957795;
  currentHeading += gyroZDeg * dt;
}

// ============================================================================
// 6. SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==================================================");
  Serial.println("🤖 ROBOCAR — CHẾ ĐỘ BÁM TƯỜNG TRÁI (PD CONTROL)");
  Serial.printf("⚙️  wallKp=%.2f  gyroKp=%.2f  gyroKd=%.2f  BASE_SPEED=%d\n",
                wallKp, gyroKp, gyroKd, BASE_SPEED);
  Serial.println("==================================================");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(30000);
  delay(200);

  // MPU6050
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("❌ MPU6050 không tìm thấy!");
    while (1)
      delay(100);
  }
  Serial.println("✓ MPU6050 (0x68)");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Laser sensors
  initLaserSensors();

  // Motors
  setupMotors();
  stopMotors();

  // Gyro calibration
  calibrateGyro();
  lastGyroTime = millis();

  Serial.println("🎉 SẴN SÀNG! Xe sẽ bắt đầu chạy sau 3 giây...");
  delay(3000);
}

// ============================================================================
// 7. VÒNG LẶP CHÍNH — ĐI THẲNG + PD BÁM TƯỜNG TRÁI LIÊN TỤC
// ============================================================================
//
// Cảm biến 2 bên lắp VUÔNG GÓC → giá trị đo = khoảng cách vuông góc trực tiếp
//
// PD Control:
//   error = WALL_TARGET - smoothLeft
//   error > 0 → xe quá GẦN tường trái → lái sang PHẢI (bánh trái nhanh hơn)
//   error < 0 → xe quá XA tường trái  → lái sang TRÁI (bánh phải nhanh hơn)
//
//   motorLeft  = BASE_SPEED + correction
//   motorRight = BASE_SPEED - correction
// ============================================================================

void loop() {
  // Cập nhật hướng góc và vận tốc góc từ Gyro
  updateHeading();

  // 1. Đọc cảm biến thô
  int rawL = readDistanceLeft();
  int rawC = readDistanceCenter();
  int rawR = readDistanceRight();

  // 2. Bộ lọc EMA làm mượt tín hiệu
  if (rawL < 8000)
    smoothLeft = EMA_ALPHA * rawL + (1.0 - EMA_ALPHA) * smoothLeft;
  if (rawC < 8000)
    smoothCenter = EMA_ALPHA * rawC + (1.0 - EMA_ALPHA) * smoothCenter;
  if (rawR < 8000)
    smoothRight = EMA_ALPHA * rawR + (1.0 - EMA_ALPHA) * smoothRight;

  // 3. VÒNG NGOÀI (Wall Error -> Target Heading)
  float wallError = (float)WALL_TARGET - smoothLeft;
  // Giới hạn hướng mục tiêu tối đa +/- 20 độ để bám tường mượt mà
  float targetHeading = constrain(wallError * wallKp, -20.0, 20.0);

  // 4. VÒNG TRONG (Gyro Heading & Angular velocity -> Motor Correction)
  float headingError = targetHeading - currentHeading;

  // Cascade PD Controller:
  // - gyroKp * headingError giúp xe quay nhanh về góc hướng mong muốn
  // - gyroKd * gyroZDeg đóng vai trò giảm chấn (damping) triệt tiêu rung lắc
  // hông
  float correction = (gyroKp * headingError) - (gyroKd * gyroZDeg);

  // 5. Tính tốc độ motor
  int motorL = BASE_SPEED + (int)correction;
  int motorR = BASE_SPEED - (int)correction;
  motorL = constrain(motorL, 0, 255);
  motorR = constrain(motorR, 0, 255);

  // 6. An toàn: dừng nếu tường trước quá gần
  if (smoothCenter < FRONT_STOP_DIST) {
    stopMotors();
    // In cảnh báo (chỉ in 1 lần mỗi giây)
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 1000) {
      lastWarn = millis();
      Serial.printf("⚠️ TƯỜNG TRƯỚC! C=%.0fmm < %dmm → DỪNG\n", smoothCenter,
                    FRONT_STOP_DIST);
    }
  } else {
    setMotorLeft(motorL);
    setMotorRight(motorR);
  }

  // 7. Debug log (mỗi 150ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 150) {
    lastPrint = millis();
    Serial.printf("L:%3d R:%3d C:%3d | WErr:%+5.1f TgtH:%+5.1f | errH:%+5.1f "
                  "gyroZ:%+6.1f | ML:%3d MR:%3d | H:%.1f°\n",
                  (int)smoothLeft, (int)smoothRight, (int)smoothCenter,
                  wallError, targetHeading, headingError, gyroZDeg, motorL,
                  motorR, currentHeading);
  }

  delay(15); // ~50Hz vòng lặp điều khiển
}
