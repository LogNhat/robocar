#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// 1. CẤU HÌNH CHÂN (PIN CONFIGURATION) THEO MAPPINGTH.MD
// ============================================================================

// Định nghĩa chân XSHUT điều khiển độc lập cho 3 cảm biến VL53L0X
#define XSHUT_CENTER 17
#define XSHUT_LEFT   15
#define XSHUT_RIGHT  16

// Địa chỉ I2C gán độc lập sau khi khởi động
#define ADDR_CENTER  0x30
#define ADDR_LEFT    0x31
#define ADDR_RIGHT   0x32

// Định nghĩa các chân Driver Động cơ TB6612FNG (Lấy từ th.text)
const int PWMA = 27; // Điều khiển tốc độ bánh Phải (Kênh A)
const int AIN1 = 18; // Chiều bánh Phải
const int AIN2 = 19; // Chiều bánh Phải

const int PWMB = 13; // Điều khiển tốc độ bánh Trái (Kênh B)
const int BIN1 = 5;  // Chiều bánh Trái
const int BIN2 = 23; // Chiều bánh Trái

// Chân I2C sử dụng chung cho MPU6050 và 3 cảm biến Laser VL53L0X
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ============================================================================
// 2. CẤU HÌNH SAI SỐ HIỆU CHUẨN CẢM BIẾN LASER (CALIBRATION OFFSETS IN MM)
// ============================================================================
// Đơn vị: mm. Giá trị này được cộng vào kết quả đo thô (để bù trừ nếu cảm biến đo lệch).
// Đã cấu hình theo kết quả đo thực tế của bạn.
const int OFFSET_CENTER = -58;  // Sai số cảm biến GIỮA
const int OFFSET_LEFT   = -9;   // Sai số cảm biến TRÁI
const int OFFSET_RIGHT  = -20;  // Sai số cảm biến PHẢI

// ============================================================================
// 3. THAM SỐ CẤU HÌNH HỌC KHÔNG GIAN MAZE & XE
// ============================================================================
const float CAR_WIDTH  = 120.0; // Chiều rộng xe: 12cm = 120mm
const float CAR_LENGTH = 140.0; // Chiều dài xe: 14cm = 140mm

// Ngưỡng phát hiện tường hông và tường trước dựa trên hình học 45 độ
const int WALL_THRESHOLD_SIDE   = 150; // Ngưỡng phát hiện tường hông (45 độ)
const int WALL_THRESHOLD_CENTER = 150; // Ngưỡng phát hiện tường trước (0 độ)

// Các thông số điều khiển tốc độ di chuyển
const int BASE_SPEED = 150;   // Tốc độ chạy thẳng
const int TURN_SPEED = 140;   // Tốc độ xoay tại chỗ
const unsigned long MOVE_FORWARD_DURATION = 800; // ms (Thời gian chạy qua 1 ô 20cm)

// Hệ số PD của thuật toán giữ xe đi thẳng bằng Gyro
float gyroKp = 3.5;
float gyroKd = 0.6;

// Biến lưu trạng thái góc và điều hướng
float gyroZOffset = 0.0;
float currentHeading = 0.0;
unsigned long lastGyroTime = 0;

// Các trạng thái của xe trong mê cung
enum RobotState {
  STATE_DECIDE,
  STATE_MOVE_FORWARD,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT,
  STATE_TURN_AROUND
};

RobotState currentState = STATE_DECIDE;

// Khởi tạo các đối tượng cảm biến
Adafruit_MPU6050 mpu;
Adafruit_VL53L0X lox_center = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left   = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right  = Adafruit_VL53L0X();

// ============================================================================
// 4. ĐIỀU KHIỂN ĐỘNG CƠ THỰC TẾ (TB6612FNG)
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
// 5. QUẢN LÝ CẢM BIẾN LASER VL53L0X
// ============================================================================
bool isDevicePresent(byte address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

void recoverI2C() {
  Serial.print("[I2C RECOVERY] Đang kiểm tra và giải phóng Bus I2C... ");
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);

  for (int i = 0; i < 9; i++) {
    if (digitalRead(SDA_PIN) == HIGH) {
      break; 
    }
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);
  Serial.println("Đã hoàn thành!");
}

void initLaserSensors() {
  Serial.println("\n[SYSTEM] Bắt đầu khởi động tuần tự cảm biến...");

  // Bước 1: Đưa tất cả cảm biến vào trạng thái Standby
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(500); 

  // BƯỚC 2: Khởi động cảm biến CENTER
  Serial.println("\n-> [1/3] Đang đánh thức cảm biến CENTER...");
  pinMode(XSHUT_CENTER, OUTPUT);
  digitalWrite(XSHUT_CENTER, HIGH);
  delay(450); 

  if (!isDevicePresent(0x29)) {
    Serial.println("❌ LỖI VẬT LÝ: Cảm biến CENTER không thức dậy ở địa chỉ 0x29!");
    Serial.println("👉 Vui lòng kiểm tra chân XSHUT_CENTER (GPIO 17) hoặc nguồn!");
    while (1) delay(200);
  }

  if (!lox_center.begin(ADDR_CENTER, false, &Wire)) {
    Serial.println("❌ Lỗi phần mềm: Không thể khởi tạo Center ở địa chỉ 0x30");
    while (1) delay(100);
  }
  Serial.println("✓ Khởi động thành công cảm biến CENTER (Đã chuyển sang I2C: 0x30)");

  // BƯỚC 3: Khởi động cảm biến LEFT
  Serial.println("\n-> [2/3] Đang đánh thức cảm biến LEFT...");
  pinMode(XSHUT_LEFT, OUTPUT);
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(450); 

  if (!isDevicePresent(0x29)) {
    Serial.println("❌ LỖI VẬT LÝ: Cảm biến LEFT không thức dậy ở địa chỉ 0x29!");
    Serial.println("👉 Vui lòng kiểm tra chân XSHUT_LEFT (GPIO 15) hoặc nguồn!");
    while (1) delay(200);
  }

  if (!lox_left.begin(ADDR_LEFT, false, &Wire)) {
    Serial.println("❌ Lỗi phần mềm: Không thể khởi tạo Left ở địa chỉ 0x31");
    while (1) delay(100);
  }
  Serial.println("✓ Khởi động thành công cảm biến LEFT (Đã chuyển sang I2C: 0x31)");

  // BƯỚC 4: Khởi động cảm biến RIGHT
  Serial.println("\n-> [3/3] Đang đánh thức cảm biến RIGHT...");
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(450); 

  if (!isDevicePresent(0x29)) {
    Serial.println("❌ LỖI VẬT LÝ: Cảm biến RIGHT không thức dậy ở địa chỉ 0x29!");
    Serial.println("👉 Vui lòng kiểm tra chân XSHUT_RIGHT (GPIO 16) hoặc nguồn!");
    while (1) delay(200);
  }

  if (!lox_right.begin(ADDR_RIGHT, false, &Wire)) {
    Serial.println("❌ Lỗi phần mềm: Không thể khởi tạo Right ở địa chỉ 0x32");
    while (1) delay(100);
  }
  Serial.println("✓ Khởi động thành công cảm biến RIGHT (Đã chuyển sang I2C: 0x32)");
  
  Serial.println("\n🎉 [SYSTEM] Khởi động thành công toàn bộ 3 cảm biến! Xe sẵn sàng đo đạc.\n");
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
// 6. CẢM BIẾN GYRO MPU6050 & ĐIỀU HƯỚNG GÓC
// ============================================================================
void calibrateGyro() {
  Serial.println("[GYRO] Đang hiệu chuẩn con quay hồi chuyển, giữ robot đứng yên...");
  sensors_event_t accel, gyro, temp;
  float sum = 0;
  const int samples = 300; 

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z;
    delay(2);
  }
  gyroZOffset = sum / samples;
  Serial.print("[GYRO] Hoàn thành. Sai số Offset Z: ");
  Serial.println(gyroZOffset, 6);
}

float readGyroZ() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  return gyro.gyro.z - gyroZOffset; 
}

void updateHeading() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0; 
  lastGyroTime = now;
  
  float gyroZ = readGyroZ(); 
  float gyroZDeg = gyroZ * 57.2957795; 
  
  currentHeading += gyroZDeg * dt;
}

// ============================================================================
// 7. THUẬT TOÁN ĐIỀU KHIỂN HÀNH TRÌNH XE THỰC TẾ (CLOSED-LOOP CONTROL)
// ============================================================================

// Chạy thẳng giữ hướng bằng Gyro và phanh an toàn bằng Laser giữa
void moveForwardOneBlock() {
  Serial.println("\n>>> [PHYSICAL] TIẾN THẲNG 1 Ô (20cm) <<<");
  updateHeading();
  float targetHeading = currentHeading; 
  unsigned long startTime = millis();
  
  while (millis() - startTime < MOVE_FORWARD_DURATION) {
    updateHeading();
    
    int distL = readDistanceLeft();
    int distC = readDistanceCenter();
    int distR = readDistanceRight();
    
    // 1. Phanh an toàn va chạm
    if (distC != 9999 && distC < 55) { 
      Serial.println("  ⚠️ [SAFETY] Khoảng cách phía trước < 5.5cm. Dừng khẩn cấp!");
      break;
    }
    
    // 2. Điều khiển vi sai giữ thẳng bằng Gyro
    float error = currentHeading - targetHeading;
    float gyroZ = readGyroZ() * 57.2957795; // rad/s sang deg/s
    
    // PD Loop
    int correction = gyroKp * error + gyroKd * gyroZ;
    correction = constrain(correction, -60, 60); 
    
    int leftSpeed  = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;
    
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
    
    delay(10);
  }
  
  stopMotors();
  delay(300); // Trễ ngắn để xe ổn định
}

// Xoay Trái 90 độ khép kín (Closed-loop)
void turnLeft90() {
  Serial.println("\n>>> [PHYSICAL] XOAY TRÁI 90 ĐỘ <<<");
  updateHeading();
  float startHeading = currentHeading;
  float targetHeading = startHeading + 90.0;
  
  setMotorLeft(-TURN_SPEED);
  setMotorRight(TURN_SPEED);
  
  unsigned long timeout = millis();
  while (currentHeading < targetHeading - 2.5) { // Dừng sớm 2.5 độ chống trớn quán tính
    updateHeading();
    
    if (millis() - timeout > 2500) { // Chống kẹt xe
      Serial.println("  ⚠️ [TIMEOUT] Xoay quá lâu, dừng khẩn cấp!");
      break;
    }
    delay(5);
  }
  
  stopMotors();
  delay(300);
  updateHeading();
  Serial.printf("  ✓ Hoàn thành xoay TRÁI. Hướng mới: %.2f°\n", currentHeading);
}

// Xoay Phải 90 độ khép kín (Closed-loop)
void turnRight90() {
  Serial.println("\n>>> [PHYSICAL] XOAY PHẢI 90 ĐỘ <<<");
  updateHeading();
  float startHeading = currentHeading;
  float targetHeading = startHeading - 90.0;
  
  setMotorLeft(TURN_SPEED);
  setMotorRight(-TURN_SPEED);
  
  unsigned long timeout = millis();
  while (currentHeading > targetHeading + 2.5) {
    updateHeading();
    
    if (millis() - timeout > 2500) {
      Serial.println("  ⚠️ [TIMEOUT] Xoay quá lâu, dừng khẩn cấp!");
      break;
    }
    delay(5);
  }
  
  stopMotors();
  delay(300);
  updateHeading();
  Serial.printf("  ✓ Hoàn thành xoay PHẢI. Hướng mới: %.2f°\n", currentHeading);
}

// Quay đầu 180 độ khép kín (Closed-loop)
void turnAround180() {
  Serial.println("\n>>> [PHYSICAL] QUAY ĐẦU 180 ĐỘ <<<");
  updateHeading();
  float startHeading = currentHeading;
  float targetHeading = startHeading + 180.0;
  
  setMotorLeft(-TURN_SPEED);
  setMotorRight(TURN_SPEED);
  
  unsigned long timeout = millis();
  while (currentHeading < targetHeading - 3.5) {
    updateHeading();
    
    if (millis() - timeout > 4000) {
      Serial.println("  ⚠️ [TIMEOUT] Quay đầu quá lâu, dừng khẩn cấp!");
      break;
    }
    delay(5);
  }
  
  stopMotors();
  delay(300);
  updateHeading();
  Serial.printf("  ✓ Hoàn thành QUAY ĐẦU 180°. Hướng mới: %.2f°\n", currentHeading);
}

// ============================================================================
// 8. KHỞI TẠO & VÒNG LẶP CHẠY THỰC TẾ
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==================================================================");
  Serial.println("🤖 ROBOCAR GIẢI MÊ CUNG GIÁT THỂ 20x20CM (PHYSICAL MODE ACTIVE)");
  Serial.printf("📏 Thông số xe: Rộng %dmm (12cm) | Dài %dmm (14cm)\n", (int)CAR_WIDTH, (int)CAR_LENGTH);
  Serial.println("📐 Cảm biến Trái/Phải đặt xiên góc 45 độ");
  Serial.println("==================================================================");

  // 1. Đưa tất cả XSHUT về LOW ngay lập tức
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(200);

  // 2. Giải phóng Bus I2C
  recoverI2C();

  // 3. Khởi tạo Bus I2C và Gyro
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(30000); // 30kHz cực kỳ lì lợm và ổn định chống nhiễu
  delay(200);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("❌ Không tìm thấy cảm biến Gyro MPU6050!");
    while (1) delay(100);
  }
  Serial.println("✓ Khởi động thành công MPU6050 (0x68)");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 4. Khởi tạo 3 cảm biến VL53L0X
  initLaserSensors();
  
  // 5. Cấu hình Driver Động cơ
  setupMotors();
  
  // 6. Hiệu chuẩn Gyro
  calibrateGyro();
  
  lastGyroTime = millis();
  currentState = STATE_DECIDE;
  
  Serial.println("🎉 XE ĐÃ SẴN SÀNG! ĐẶT ROBOT VÀO MÊ CUNG ĐỂ BẮT ĐẦU CHẠY...");
  delay(2000);
}

void loop() {
  // Cập nhật tích phân góc liên tục
  updateHeading();

  // Đọc khoảng cách thực tế (đã bù trừ sai số OFFSET)
  int leftDist   = readDistanceLeft();
  int centerDist = readDistanceCenter();
  int rightDist  = readDistanceRight();

  // Tính khoảng cách vuông góc thực tế đến tường bên dựa theo góc xiên 45 độ
  // d_perp = d_sensor * sin(45°) = d_sensor * 0.7071
  float leftPerp  = (leftDist != 9999) ? (leftDist * 0.7071) : 9999;
  float rightPerp = (rightDist != 9999) ? (rightDist * 0.7071) : 9999;

  // Xác định tường bao quanh xe
  bool wallLeft   = (leftDist < WALL_THRESHOLD_SIDE);
  bool wallCenter = (centerDist < WALL_THRESHOLD_CENTER);
  bool wallRight  = (rightDist < WALL_THRESHOLD_SIDE);

  switch (currentState) {
    
    // RA QUYẾT ĐỊNH HƯỚNG ĐI (Thuật toán Bám Tường Trái - Left Hand Rule)
    case STATE_DECIDE: {
      Serial.println("\n========================================================");
      Serial.println("[STATE] TRẠNG THÁI QUYẾT ĐỊNH (STATE_DECIDE)");
      Serial.println("--------------------------------------------------------");
      Serial.printf("🔍 [CẢM BIẾN]  Trái(45°): %dmm | Giữa(0°): %dmm | Phải(45°): %dmm\n", leftDist, centerDist, rightDist);
      Serial.printf("📐 [TÍNH VUÔNG GÓC] Khoảng cách vuông góc hông | Trái: %.1fmm | Phải: %.1fmm\n", leftPerp, rightPerp);
      Serial.printf("🧱 [TƯỜNG VẬT LÝ]  Trái: %s | Giữa: %s | Phải: %s\n", 
                    wallLeft ? "Có Tường (🧱)" : "Trống (🟢)", 
                    wallCenter ? "Có Tường (🧱)" : "Trống (🟢)", 
                    wallRight ? "Có Tường (🧱)" : "Trống (🟢)");

      // Áp dụng thuật toán Bám Tường Trái:
      if (!wallLeft) {
        Serial.println("👉 [QUYẾT ĐỊNH] => Ưu tiên 1: Bên TRÁI TRỐNG. Xoay TRÁI và đi tới!");
        currentState = STATE_TURN_LEFT;
      } 
      else if (!wallCenter) {
        Serial.println("👉 [QUYẾT ĐỊNH] => Ưu tiên 2: Phía GIỮA TRỐNG. Đi THẲNG!");
        currentState = STATE_MOVE_FORWARD;
      } 
      else if (!wallRight) {
        Serial.println("👉 [QUYẾT ĐỊNH] => Ưu tiên 3: Bên PHẢI TRỐNG. Xoay PHẢI và đi tới!");
        currentState = STATE_TURN_RIGHT;
      } 
      else {
        Serial.println("👉 [QUYẾT ĐỊNH] => Ưu tiên 4: Cụt đường hoàn toàn! QUAY ĐẦU 180°!");
        currentState = STATE_TURN_AROUND;
      }
      break;
    }

    // THỰC HIỆN DI CHUYỂN
    case STATE_MOVE_FORWARD: {
      moveForwardOneBlock();
      currentState = STATE_DECIDE; 
      break;
    }

    case STATE_TURN_LEFT: {
      turnLeft90();
      moveForwardOneBlock();
      currentState = STATE_DECIDE;
      break;
    }

    case STATE_TURN_RIGHT: {
      turnRight90();
      moveForwardOneBlock();
      currentState = STATE_DECIDE;
      break;
    }

    case STATE_TURN_AROUND: {
      turnAround180();
      moveForwardOneBlock();
      currentState = STATE_DECIDE;
      break;
    }
  }

  delay(200); // Trễ ngắn 200ms giữa các trạng thái để dễ kiểm soát
}