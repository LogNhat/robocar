#include <Arduino.h>   // BẮT BUỘC CÓ KHI DÙNG VS CODE
#include <Bluepad32.h> // Thư viện đọc tay cầm
#include <ESP32Servo.h> // Thư viện điều khiển servo cho ESP32
#include <Preferences.h> // Thư viện lưu trữ dữ liệu bền vững (Flash) của ESP32

ControllerPtr tayCam = nullptr; // Khai báo tên tay cầm
Preferences preferences;        // Đối tượng lưu vị trí cánh tay robot vào Flash
unsigned long connectionTime = 0; // Thời điểm tay cầm kết nối thành công

// --- SƠ ĐỒ CHÂN ĐỘNG CƠ DC THEO ĐÚNG ĐẤU NỐI THỰC TẾ ---
int IN1 = 27;
int IN2 = 26;
int ENA = 14; // OUT1 & OUT2 -> Cụm bánh TRÁI (Đỏ OUT1, Đen OUT2)

int IN3 = 25;
int IN4 = 33;
int ENB = 32; // OUT3 & OUT4 -> Cụm bánh PHẢI (Đỏ OUT3, Đen OUT4)

// --- SƠ ĐỒ CHÂN CÁC ĐỘNG CƠ SERVO CÁNH TAY ROBOT ---
const int PIN_BASE_LIFT1 = 15; // Servo nâng đế 1 (Base Lift 1)
const int PIN_BASE_LIFT2 = 4;  // Servo nâng đế 2 (Base Lift 2)
const int PIN_MID        = 16; // Servo khớp giữa (Elbow)
const int PIN_OUTER      = 17; // Servo khớp ngoài cùng, sát kẹp gắp (Wrist)
const int PIN_GRIPPER    = 5;  // Servo khớp kẹp gắp (Gripper)

// Khai báo các đối tượng Servo
Servo servoBaseLift1;
Servo servoBaseLift2;
Servo servoMid;
Servo servoOuter;
Servo servoGripper;

// Khai báo trạng thái kích hoạt của từng Servo (Lazy Attach)
bool baseAttached = false;
bool midAttached = false;
bool outerAttached = false;
bool gripperAttached = false;

// Góc hiện tại của các servo
float angleBaseLift;
float angleMid;
float angleOuter;
float angleGripper;

// Giới hạn góc quay để bảo vệ khớp cơ khí (Hãy tự căn chỉnh lại các số này sau khi test)
const float LIMIT_BASE_LIFT_MIN = 0.0;
const float LIMIT_BASE_LIFT_MAX = 360.0; 
const float LIMIT_MID_MIN       = 0.0;
const float LIMIT_MID_MAX       = 180.0; 
const float LIMIT_OUTER_MIN     = 0.0;
const float LIMIT_OUTER_MAX     = 180.0; 
const float LIMIT_GRIPPER_MIN   = 0.0;   // Giới hạn tối thiểu ban đầu (0 độ) để tìm góc khép/mở
const float LIMIT_GRIPPER_MAX   = 180.0; // Tăng thêm 90 độ nữa (từ 90.0 lên 180.0) để thử nghiệm toàn bộ dải servo

// Tốc độ di chuyển góc servo
const float SPEED_JOYSTICK = 1.2;
const float SPEED_BUTTONS  = 1.5;
const float SPEED_GRIPPER  = 2.5;

// KHAI BÁO TRƯỚC CÁC HÀM DI CHUYỂN
void diChuyenTien(int tocDo);
void diChuyenLui(int tocDo);
void quayTrai(int tocDo);
void quayPhai(int tocDo);
void dungLai();
void inLogTayCam(ControllerPtr ctl);
void setMotorLeft(int speed);
void setMotorRight(int speed);
void updateServos();
void attachServos();
void saveServosToFlash();

// Hàm tự động chạy khi tay cầm kết nối
void onConnectedController(ControllerPtr ctl) {
  if (tayCam == nullptr) {
    tayCam = ctl;
    connectionTime = millis();
    Serial.println("Đã kết nối tay cầm thành công!");
  }
}

// Hàm tự động chạy khi tay cầm ngắt kết nối
void onDisconnectedController(ControllerPtr ctl) {
  if (tayCam == ctl) {
    tayCam = nullptr;
    Serial.println("Tay cầm đã ngắt kết nối!");
    dungLai();
    
    if (baseAttached) { servoBaseLift1.detach(); servoBaseLift2.detach(); baseAttached = false; }
    if (midAttached) { servoMid.detach(); midAttached = false; }
    if (outerAttached) { servoOuter.detach(); outerAttached = false; }
    if (gripperAttached) { servoGripper.detach(); gripperAttached = false; }
    Serial.println("[SERVO] Đã nhả lực giữ toàn bộ Servo.");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // ==========================================
  // ĐỌC DỮ LIỆU TỪ FLASH KHI KHỞI ĐỘNG
  // ==========================================
  preferences.begin("robot_arm", false);
  // Nếu chưa có dữ liệu lưu (hoặc vừa bị reset), sẽ tự động lấy giá trị mặc định là 90, 90, 90, 0
  angleBaseLift = preferences.getFloat("base", 90.0);
  angleMid      = preferences.getFloat("mid", 90.0);
  angleOuter    = preferences.getFloat("outer", 90.0);
  angleGripper  = preferences.getFloat("grip", 0.0); // Mặc định khép hoàn toàn ở góc 0 độ
  preferences.end();
  
  Serial.println("\n--- THÔNG TIN KHỞI ĐỘNG ---");
  Serial.printf("✓ Góc đọc từ bộ nhớ Flash: Đế=%d°, Giữa=%d°, Ngoài=%d°, Kẹp=%d°\n", 
                (int)angleBaseLift, (int)angleMid, (int)angleOuter, (int)angleGripper);
  Serial.println("✓ Bấm nút [SELECT / -] để LƯU vị trí hiện tại.");
  Serial.println("✓ Bấm nút [START / +] để XÓA dữ liệu và khôi phục mặc định.");
  Serial.println("----------------------------\n");

  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("Hệ thống đã sẵn sàng! Đang tìm kiếm tay cầm...");
}

void attachBase() {
  if (baseAttached) return;
  ESP32PWM::allocateTimer(0);
  servoBaseLift1.setPeriodHertz(50);
  servoBaseLift2.setPeriodHertz(50);
  servoBaseLift1.write((int)angleBaseLift);
  servoBaseLift2.write(180 - (int)angleBaseLift);
  servoBaseLift1.attach(PIN_BASE_LIFT1, 500, 2400);
  servoBaseLift2.attach(PIN_BASE_LIFT2, 500, 2400);
  baseAttached = true;
  Serial.println("[SERVO] Kích hoạt Servo Đế (15 & 4).");
}

void attachMid() {
  if (midAttached) return;
  ESP32PWM::allocateTimer(1);
  servoMid.setPeriodHertz(50);
  servoMid.write((int)angleMid);
  servoMid.attach(PIN_MID, 500, 2400);
  midAttached = true;
  Serial.println("[SERVO] Kích hoạt Servo Giữa (16).");
}

void attachOuter() {
  if (outerAttached) return;
  ESP32PWM::allocateTimer(2);
  servoOuter.setPeriodHertz(50);
  servoOuter.write((int)angleOuter);
  servoOuter.attach(PIN_OUTER, 500, 2400);
  outerAttached = true;
  Serial.println("[SERVO] Kích hoạt Servo Cổ Tay (17).");
}

void attachGripper() {
  if (gripperAttached) return;
  ESP32PWM::allocateTimer(3);
  servoGripper.setPeriodHertz(50);
  servoGripper.write((int)angleGripper);
  servoGripper.attach(PIN_GRIPPER, 500, 2400);
  gripperAttached = true;
  Serial.println("[SERVO] Kích hoạt Servo Kẹp Gắp (5).");
}

void loop() {
  BP32.update();

  if (tayCam && tayCam->isConnected()) {

    // ==========================================
    // TÍNH NĂNG LƯU / XÓA BỘ NHỚ FLASH TỪ TAY CẦM
    // ==========================================
    
    // Bấm nút Share (Select / -) -> Lưu vị trí hiện tại
    if (tayCam->miscSelect()) {
      saveServosToFlash();
      delay(500); // Trễ 0.5s để chống dội phím
    }

    // Bấm nút Options (Start / +) -> Xóa sạch bộ nhớ & Reset mạch
    if (tayCam->miscStart()) {
      Serial.println("\n[CẢNH BÁO] Đang tiến hành xóa toàn bộ dữ liệu Flash...");
      preferences.begin("robot_arm", false);
      preferences.clear();
      preferences.end();
      Serial.println("[THÀNH CÔNG] Đã xóa Flash! Mạch sẽ khởi động lại sau 1 giây...");
      delay(1000);
      ESP.restart(); // Khởi động lại ESP32
    }

    // ==========================================
    // ĐIỀU KHIỂN ĐỘNG CƠ DC (BÁNH XE)
    // ==========================================
    if (tayCam->dpad() == DPAD_UP) {
      diChuyenTien(250);
    } else if (tayCam->dpad() == DPAD_DOWN) {
      diChuyenLui(250);
    } else if (tayCam->dpad() == DPAD_LEFT) {
      quayTrai(200);
    } else if (tayCam->dpad() == DPAD_RIGHT) {
      quayPhai(200);
    } 
    else {
      int trucY = tayCam->axisY(); 
      int trucX = tayCam->axisX(); 

      int deadzone = 40;
      if (abs(trucY) < deadzone) trucY = 0;
      if (abs(trucX) < deadzone) trucX = 0;

      if (trucY == 0 && trucX == 0) {
        dungLai();
      } else {
        float throttle = -trucY; 
        float steering = trucX;
        float v_throttle = (throttle / 512.0) * 255.0;
        float v_steering = (steering / 512.0) * 255.0;
        float leftSpeed = v_throttle + v_steering;
        float rightSpeed = v_throttle - v_steering;

        int speedL = constrain((int)leftSpeed, -255, 255);
        int speedR = constrain((int)rightSpeed, -255, 255);
        setMotorLeft(speedL);
        setMotorRight(speedR);
      }
    }

    // ==========================================
    // ĐIỀU KHIỂN CÁNH TAY ROBOT (SERVOS)
    // ==========================================
    updateServos();
  }
  delay(15);
}

void updateServos() {
  if (!tayCam || !tayCam->isConnected()) return;
  if (millis() - connectionTime < 1500) return;

  int ry = tayCam->axisRY(); 
  int deadzone = 70;
  if (abs(ry) < deadzone) ry = 0;

  if (ry != 0) {
    attachBase();
    angleBaseLift += (-ry / 512.0) * SPEED_JOYSTICK;
    angleBaseLift = constrain(angleBaseLift, LIMIT_BASE_LIFT_MIN, LIMIT_BASE_LIFT_MAX);
    servoBaseLift1.write((int)angleBaseLift);
    servoBaseLift2.write(180 - (int)angleBaseLift);
  }

  if (tayCam->l1() || tayCam->r1()) {
    attachMid();
    if (tayCam->l1()) angleMid -= SPEED_BUTTONS;
    if (tayCam->r1()) angleMid += SPEED_BUTTONS;
    angleMid = constrain(angleMid, LIMIT_MID_MIN, LIMIT_MID_MAX);
    servoMid.write((int)angleMid);
  }

  if (tayCam->x() || tayCam->y()) {
    attachOuter();
    if (tayCam->x()) angleOuter -= SPEED_BUTTONS;
    if (tayCam->y()) angleOuter += SPEED_BUTTONS;
    angleOuter = constrain(angleOuter, LIMIT_OUTER_MIN, LIMIT_OUTER_MAX);
    servoOuter.write((int)angleOuter);
  }

  if (tayCam->a() || tayCam->b()) {
    attachGripper();
    if (tayCam->b()) angleGripper += SPEED_GRIPPER; // Nút B -> Tăng góc (Mở hoặc đóng tùy cấu trúc lắp ráp)
    if (tayCam->a()) angleGripper -= SPEED_GRIPPER; // Nút A -> Giảm góc (Đóng hoặc mở tùy cấu trúc lắp ráp)
    angleGripper = constrain(angleGripper, LIMIT_GRIPPER_MIN, LIMIT_GRIPPER_MAX);
    servoGripper.write((int)angleGripper);
  }
}

// CẬP NHẬT HÀM LƯU VỊ TRÍ VÀO FLASH
void saveServosToFlash() {
  preferences.begin("robot_arm", false);
  preferences.putFloat("base", angleBaseLift);
  preferences.putFloat("mid", angleMid);
  preferences.putFloat("outer", angleOuter);
  preferences.putFloat("grip", angleGripper);
  preferences.end(); // Đóng lại để bảo vệ bộ nhớ
  Serial.println("[LƯU FLASH] Đã lưu thành công tư thế hiện tại của cánh tay!");
}

// --- CÁC HÀM XỬ LÝ DI CHUYỂN ĐỘNG CƠ DC ---
void diChuyenTien(int tocDo) { setMotorLeft(tocDo); setMotorRight(tocDo); }
void diChuyenLui(int tocDo) { setMotorLeft(-tocDo); setMotorRight(-tocDo); }
void quayTrai(int tocDo) { setMotorLeft(-tocDo); setMotorRight(tocDo); }
void quayPhai(int tocDo) { setMotorLeft(tocDo); setMotorRight(-tocDo); }
void dungLai() { setMotorLeft(0); setMotorRight(0); }

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed); } 
  else if (speed < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, -speed); } 
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0); }
}

void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed); } 
  else if (speed < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, -speed); } 
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0); }
}

// Hàm in log các tín hiệu từ tay cầm ra Serial để theo dõi
void inLogTayCam(ControllerPtr ctl) {
  static unsigned long lastLogTime = 0;
  if (millis() - lastLogTime < 250) {
    return;
  }
  lastLogTime = millis();

  int lx = ctl->axisX();
  int ly = ctl->axisY();
  int rx = ctl->axisRX();
  int ry = ctl->axisRY();

  int phanh = ctl->brake();
  int ga = ctl->throttle();

  int dpad = ctl->dpad();
  String dpadStr = "";
  if (dpad & DPAD_UP) dpadStr += "LEN ";
  if (dpad & DPAD_DOWN) dpadStr += "XUONG ";
  if (dpad & DPAD_LEFT) dpadStr += "TRAI ";
  if (dpad & DPAD_RIGHT) dpadStr += "PHAI ";
  if (dpadStr == "") dpadStr = "KHONG";

  String nutStr = "";
  if (ctl->a()) nutStr += "A ";
  if (ctl->b()) nutStr += "B ";
  if (ctl->x()) nutStr += "X ";
  if (ctl->y()) nutStr += "Y ";
  if (ctl->l1()) nutStr += "L1 ";
  if (ctl->r1()) nutStr += "R1 ";
  if (nutStr == "") nutStr = "KHONG";

  Serial.print("[LOG TAY CAM] ");
  Serial.print("JoyL: (X="); Serial.print(lx); Serial.print(", Y="); Serial.print(ly); Serial.print(") | ");
  Serial.print("JoyR: (X="); Serial.print(rx); Serial.print(", Y="); Serial.print(ry); Serial.print(") | ");
  Serial.print("Dpad: "); Serial.print(dpadStr); Serial.print(" | ");
  Serial.print("Nút: "); Serial.print(nutStr); Serial.print(" | ");
  Serial.print("Cò Trái: "); Serial.print(phanh); Serial.print(" | ");
  Serial.print("Cò Phải: "); Serial.println(ga);
}