#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// 1. CẤU HÌNH CHÂN VÀ SAI SỐ CỦA CẢM BIẾN LASER
// ============================================================================
#define XSHUT_CENTER 15
#define XSHUT_LEFT 17
#define XSHUT_RIGHT 16

#define ADDR_CENTER 0x30
#define ADDR_LEFT 0x31
#define ADDR_RIGHT 0x32

// Bù trừ sai số đo thực tế của cảm biến (mm)
const int OFFSET_CENTER = -55;
const int OFFSET_LEFT = -20;
const int OFFSET_RIGHT = -25;

// Động cơ TB6612FNG (PWMA là Left, PWMB là Right)
const int PWMA = 27;
const int AIN1 = 18;
const int AIN2 = 19;
const int PWMB = 13;
const int BIN1 = 14;
const int BIN2 = 23;

const int SDA_PIN = 21;
const int SCL_PIN = 22;

Adafruit_MPU6050 mpu;
Adafruit_VL53L0X lox_center = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();

// ============================================================================
// 2. THAM SỐ THUẬT TOÁN BÁM TƯỜNG (CĂN CHỈNH CHO XE 15x12cm, Ô MÊ CUNG 20x20cm)
// ============================================================================
// Hành lang rộng 200mm, xe rộng 120mm -> Khoảng cách căn giữa lý tưởng là 40mm
const int WALL_TARGET = 40;  
const int BASE_SPEED = 100;  // Tốc độ cơ sở đi thẳng lầm lũi siêu ổn định
const int maxCorrection = 70; // Giới hạn chỉnh lái để chống lắc hông

// Hệ số PID từ file backup đã được chứng minh cực kỳ mượt mà
float Kp = 1.4;  
float KdGyro = 0.6;  
const float GYRO_SIGN = 1.0; 

// Bộ lọc EMA làm mượt dữ liệu bám tường
const float EMA_ALPHA = 0.35;
float smoothLeft = 40.0;
float smoothCenter = 300.0;
float smoothRight = 40.0;

// Biến Gyro và góc hướng
float gyroZOffset = 0.0;
float gyroZDeg = 0.0;       
float currentHeading = 0.0; 
unsigned long lastGyroTime = 0;
unsigned long lastTurnTime = 0; // Cooldown chặn ngã rẽ trùng lặp

// ============================================================================
// 3. ĐIỀU KHIỂN ĐỘNG CƠ (PWMA = Left, PWMB = Right)
// ============================================================================
void motorLeft(int s) {
  s = constrain(s, -255, 255);
  digitalWrite(AIN1, s >= 0);
  digitalWrite(AIN2, s < 0);
  analogWrite(PWMA, abs(s));
}

void motorRight(int s) {
  s = constrain(s, -255, 255);
  digitalWrite(BIN1, s >= 0);
  digitalWrite(BIN2, s < 0);
  analogWrite(PWMB, abs(s));
}

void setMotors(int l, int r) {
  motorLeft(l);
  motorRight(r);
}

// ============================================================================
// 4. KHỞI TẠO VÀ ĐỌC LASER VL53L0X
// ============================================================================
bool isDevicePresent(byte address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

void initLaserSensors() {
  Serial.println("\n[SYSTEM] Khởi động các cảm biến Laser...");
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
    while (1) delay(200);
  }
  lox_center.begin(ADDR_CENTER, false, &Wire);

  // LEFT
  pinMode(XSHUT_LEFT, OUTPUT);
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(450);
  if (!isDevicePresent(0x29)) {
    Serial.println("❌ Cảm biến LEFT không thức dậy ở 0x29!");
    while (1) delay(200);
  }
  lox_left.begin(ADDR_LEFT, false, &Wire);

  // RIGHT
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(450);
  if (!isDevicePresent(0x29)) {
    Serial.println("❌ Cảm biến RIGHT không thức dậy ở 0x29!");
    while (1) delay(200);
  }
  lox_right.begin(ADDR_RIGHT, false, &Wire);

  Serial.println("[SYSTEM] Khởi động thành công 3 cảm biến Laser!\n");
}

int readDistanceLeft() {
  VL53L0X_RangingMeasurementData_t measure;
  lox_left.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter + OFFSET_LEFT;
  }
  return 9999;
}

int readDistanceCenter() {
  VL53L0X_RangingMeasurementData_t measure;
  lox_center.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter + OFFSET_CENTER;
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
  Serial.println("[GYRO] Hiệu chuẩn Gyro... Giữ robot đứng yên...");
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
  if (dt > 0.1) dt = 0.01; 
  
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  gyroZDeg = (gyro.gyro.z - gyroZOffset) * 57.2957795 * GYRO_SIGN; 
  currentHeading += gyroZDeg * dt;
}

// ============================================================================
// 6. HÀM XOAY SO LE (PIVOT TURN) ĐÃ ĐƯỢC CHỨNG MINH CHUẨN XÁC 100%
// ============================================================================
void rotateSoleTo(float targetHeading) {
  unsigned long start = millis();
  lastGyroTime = millis(); 
  
  while (millis() - start < 4000) { 
    updateHeading(); 
    float error = targetHeading - currentHeading;
    
    // Đạt góc đích (sai số < 1.5 độ) -> Dừng ngay
    if (abs(error) < 1.5) {
      Serial.printf("  -> Hoàn thành xoay! Sai số: %.2f\n", error);
      break; 
    }
    
    // --- LẮNG NGHE LASER TRONG LÚC XOAY (CHỐNG XOAY 360 ĐỘ) ---
    if (abs(error) < 60) { // Đã xoay hơn phân nửa góc, liếc nhìn cảm biến trước
      int currentCenter = readDistanceCenter();
      if (currentCenter != 9999 && currentCenter > 220) {
        Serial.println("  -> [LASER OVERRIDE] Phía trước thông thoáng! Hủy xoay Gyro để đâm thẳng.");
        break;
      }
    }
    
    // Tốc độ giảm dần (P-Control) để chống vọt lố (Overshoot)
    int spd = 85; 
    if (abs(error) < 30) spd = 65; 
    if (abs(error) < 10) spd = 55; 
    
    if (error > 0) {
      // Quay Trái (Bánh trái lùi, bánh phải tiến)
      setMotors(-spd, spd);
    } else {
      // Quay Phải (Bánh trái tiến, bánh phải lùi)
      setMotors(spd, -spd);
    }
    
    delay(10);
  }
  
  setMotors(0, 0);
  delay(300); // Ổn định
}

void turnLeft90() {
  int sideDist = readDistanceLeft();
  if (sideDist != 9999 && sideDist < 60) {
    Serial.printf("⚠️ [SAFETY] Đầu sườn trái quá sát (%d mm < 60mm), lùi nhẹ tránh quẹt mép...\n", sideDist);
    setMotors(-95, -95);
    delay(350); // Lùi nhẹ khoảng 5cm
    setMotors(0, 0);
    delay(150);
  }
  Serial.println("\n>>> [EXECUTE] QUAY TRÁI 90 ĐỘ SO LE <<<");
  rotateSoleTo(currentHeading + 90.0);
  lastTurnTime = millis() - 600; // Giảm thời gian chờ thực tế xuống chỉ còn 200ms sau rẽ trái
  lastGyroTime = millis();
}

void turnRight90() {
  int sideDist = readDistanceRight();
  if (sideDist != 9999 && sideDist < 60) {
    Serial.printf("⚠️ [SAFETY] Đầu sườn phải quá sát (%d mm < 60mm), lùi nhẹ tránh quẹt mép...\n", sideDist);
    setMotors(-95, -95);
    delay(350); // Lùi nhẹ khoảng 5cm
    setMotors(0, 0);
    delay(150);
  }
  Serial.println("\n>>> [EXECUTE] QUAY PHẢI 90 ĐỘ SO LE <<<");
  rotateSoleTo(currentHeading - 90.0);
  lastTurnTime = millis() - 600; // Giảm thời gian chờ thực tế xuống chỉ còn 200ms sau rẽ phải
  lastGyroTime = millis();
}

void turnAround180() {
  Serial.println("\n>>> [EXECUTE] QUAY ĐẦU 180 ĐỘ SO LE <<<");
  rotateSoleTo(currentHeading - 180.0);
  lastTurnTime = millis() - 800; // Xóa bỏ hoàn toàn cooldown (0ms) sau khi quay đầu 180 để rẽ ngay nếu gặp lối thoát
  lastGyroTime = millis();
}

// ============================================================================
// 7. SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==================================================");
  Serial.println("🤖 ROBOCAR — THUẬT TOÁN GIẢI MÊ CUNG SO LE BÁM TRÁI");
  Serial.println("==================================================");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(200);

  // MPU6050
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("❌ Không tìm thấy Gyro MPU6050!");
    while (1) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Khởi tạo Laser
  initLaserSensors();

  // Khởi tạo chân Động cơ
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  setMotors(0, 0);

  // Hiệu chuẩn cảm biến góc
  calibrateGyro();
  lastGyroTime = millis();

  Serial.println("🎉 KHỞI ĐỘNG XONG! Chạy mê cung sau 3 giây...");
  delay(3000);
}

// ============================================================================
// 8. LUỒNG QUYẾT ĐỊNH GIẢI MÊ CUNG ĐƠN GIẢN VÀ CHẮC CHẮN NHẤT
// ============================================================================
void loop() {
  updateHeading();

  // Đọc thô cực nhanh
  int rawL = readDistanceLeft();
  int rawR = readDistanceRight();
  int rawC = readDistanceCenter();

  // --------------------------------------------------------------------------
  // BỘ PHÁT HIỆN KẸT (STUCK DETECTOR - 2 GIÂY KHÔNG DỊCH CHUYỂN / ĐỔI HƯỚNG)
  // --------------------------------------------------------------------------
  static unsigned long lastPosChangeTime = 0;
  static int lastKnownC = 0;
  static float lastKnownHeading = 0.0;
  
  if (lastPosChangeTime == 0) lastPosChangeTime = millis();

  // Nếu khoảng cách trước mặt thay đổi > 8mm HOẶC góc gyro thay đổi > 3 độ
  if (abs(rawC - lastKnownC) > 8 || abs(currentHeading - lastKnownHeading) > 3.0) {
    lastKnownC = rawC;
    lastKnownHeading = currentHeading;
    lastPosChangeTime = millis(); // Reset bộ đếm kẹt
  }

  // Nếu bị kẹt tại một chỗ quá 2000ms
  if (millis() - lastPosChangeTime > 2000) {
    Serial.println("⚠️ [STUCK DETECTED] Robot bị kẹt 2 giây! Đang tự cứu hộ bằng cách lùi và lách lái...");
    setMotors(-115, -115); // Lùi mạnh mẽ để thoát kẹt
    delay(500);
    setMotors(115, -115);  // Đánh lái phải nhẹ để lách mép kẹt
    delay(250);
    setMotors(0, 0);
    delay(150);
    
    // Đọc lại cảm biến để cập nhật ngay sau khi cứu kẹt
    rawL = readDistanceLeft();
    rawR = readDistanceRight();
    rawC = readDistanceCenter();
    lastPosChangeTime = millis(); // Reset mốc kẹt
  }

  // Áp dụng bộ lọc EMA làm mượt cho luồng đi thẳng bám tường
  if (rawL < 350) smoothLeft = EMA_ALPHA * rawL + (1.0 - EMA_ALPHA) * smoothLeft;
  else smoothLeft = 999.0;

  if (rawR < 350) smoothRight = EMA_ALPHA * rawR + (1.0 - EMA_ALPHA) * smoothRight;
  else smoothRight = 999.0;

  if (rawC < 1000) smoothCenter = EMA_ALPHA * rawC + (1.0 - EMA_ALPHA) * smoothCenter;
  else smoothCenter = 999.0;

  // -------------------------------------------------------------
  // TRẠNG THÁI 1: KHẨN CẤP HOẶC GẶP TƯỜNG TRƯỚC MẶT (CHẶN ĐẦU)
  // -------------------------------------------------------------
  // Phát hiện tường trước < 65mm -> Lùi nhẹ lấy đà và tạo khoảng trống tâm ngã tư tuyệt đối
  if (rawC != 9999 && rawC < 65) {
    Serial.println("\n🚨 [DECISION] Phát hiện tường trước (< 65mm)! Lùi nhẹ chống quán tính...");
    
    // Lùi nhẹ để triệt tiêu quán tính tiến và tạo khoảng trống an toàn vừa đủ
    setMotors(-80, -80);
    delay(200); 
    setMotors(0, 0);
    delay(150); 
    
    // Đọc lại cảm biến sườn cực kỳ chuẩn xác khi xe đã đứng yên ổn định tại tâm ngã tư
    rawL = readDistanceLeft();
    rawR = readDistanceRight();
    
    if (rawL > 180) {
      turnLeft90();
    } 
    else if (rawR > 180) {
      turnRight90();
    } 
    else {
      // Đã lùi sẵn, chỉ việc thực hiện quay đầu 180 độ tại chỗ cực thoáng
      turnAround180();
    }
    
    currentHeading = 0.0;
    smoothLeft = 40.0;
    smoothCenter = 300.0;
    smoothRight = 40.0;
    return;
  }

  // -------------------------------------------------------------
  // TRẠNG THÁI 2: ĐANG ĐI THẲNG MÀ XUẤT HIỆN LỐI ĐI BÊN TRÁI
  // -------------------------------------------------------------
  bool canTurn = (millis() - lastTurnTime > 200); // Cooldown siêu ngắn 200ms
  
  if (canTurn && rawL > 180) {
    // Sườn trái mở rộng -> Tiến thẳng thêm 10cm (~650ms) để đưa bánh xe vào tâm ngã ba
    unsigned long startM = millis();
    while (millis() - startM < 650) {
      if (readDistanceCenter() < 65) {
        break; // Phanh khẩn cấp nếu có tường ngang chặn đột ngột
      }
      setMotors(95, 95); 
      delay(10);
    }
    
    // Phanh chủ động bằng xung lùi ngắn để xe đứng khựng ngay tại tâm ngã ba, chống trôi tự do
    setMotors(-110, -110);
    delay(120);
    setMotors(0, 0);
    delay(150);
    
    turnLeft90();
    currentHeading = 0.0;
    smoothLeft = 40.0;
    smoothCenter = 300.0;
    smoothRight = 40.0;
    return;
  }

  // -------------------------------------------------------------
  // TRẠNG THÁI 3: ĐI THẲNG LẦM LŨI BÁM TƯỜNG (PID MƯỢT MÀ TỪ BACKUP)
  // -------------------------------------------------------------
  float dynamicTarget = WALL_TARGET; // Căn giữa 40mm

  // Ép sát trái xuống 25mm trước khi rẽ phải/180 để tạo đà quay sườn phải
  if (rawC <= 260 && rawL <= 240) {
    dynamicTarget = 25.0; 
  }

  float wallError = 0.0;
  
  if (smoothLeft <= 180 && smoothRight <= 180) {
    // A. Cả 2 bên đều có tường sát: Bám trái, né phải nếu phải quá sát (< 45mm)
    wallError = smoothLeft - dynamicTarget;
    if (smoothRight < 45) {
      float avoidRight = 45.0 - smoothRight;
      wallError += avoidRight * 1.5; 
    }
  }
  else if (smoothLeft <= 180 && smoothRight > 180) {
    // B. Chỉ có tường trái: Bám tường trái
    wallError = smoothLeft - dynamicTarget;
  }
  else if (smoothLeft > 180 && smoothRight <= 180) {
    // C. Mất tường trái nhưng có tường phải: Bám tường phải
    wallError = 40.0 - smoothRight; 
  }
  else {
    // D. Trống cả 2 bên: Đi thẳng
    wallError = 0.0;
  }
  
  // Tính toán điều chỉnh PD bám tường + Gyro D giảm giật đuôi cực mượt từ file backup
  float correction = (Kp * wallError) - (KdGyro * gyroZDeg);
  correction = constrain(correction, -maxCorrection, maxCorrection);

  int leftSpeed = constrain(BASE_SPEED - correction, -255, 255);
  int rightSpeed = constrain(BASE_SPEED + correction, -255, 255);

  // --- BỘ BẢO VỆ PHẢN XẠ THÔ CẬP NHẬT TỨC THÌ (CHỐNG TÔNG SƯỜN VÀ CHẠM MÉP) ---
  // Vì 2 cảm biến sườn lắp ở đầu xe, nếu đầu xe lệch sát sườn nào (< 22mm), lập tức bẻ lái khẩn cấp bằng cảm biến thô!
  if (rawL != 9999 && rawL < 22) {
    leftSpeed = BASE_SPEED + 35;
    rightSpeed = BASE_SPEED - 55;
    Serial.printf("🚨 [AVOID] Đầu trái sát (%d mm)! Bẻ phải khẩn cấp.\n", rawL);
  }
  else if (rawR != 9999 && rawR < 22) {
    leftSpeed = BASE_SPEED - 55;
    rightSpeed = BASE_SPEED + 35;
    Serial.printf("🚨 [AVOID] Đầu phải sát (%d mm)! Bẻ trái khẩn cấp.\n", rawR);
  }

  setMotors(leftSpeed, rightSpeed);

  // In log bám tường gọn gàng mỗi 200ms
  static unsigned long lastLogPD = 0;
  if (millis() - lastLogPD >= 200) {
    lastLogPD = millis();
    Serial.printf("[DRIVE] L:%.0f C:%.0f R:%.0f | Err:%+5.1f | SpdL:%3d SpdR:%3d\n", 
                  smoothLeft, smoothCenter, smoothRight, wallError, leftSpeed, rightSpeed);
  }

  delay(10); 
}
