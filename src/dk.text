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

// Góc hiện tại của các servo (sẽ được khôi phục từ bộ nhớ Flash khi khởi động)
float angleBaseLift  = 90.0;
float angleMid       = 90.0;
float angleOuter     = 90.0;
float angleGripper   = 90.0;

// Giới hạn góc quay để bảo vệ khớp cơ khí
const float LIMIT_BASE_LIFT_MIN = 0.0;
const float LIMIT_BASE_LIFT_MAX = 180.0;
const float LIMIT_MID_MIN       = 0.0;
const float LIMIT_MID_MAX       = 300.0;
const float LIMIT_OUTER_MIN     = 0.0;
const float LIMIT_OUTER_MAX     = 180.0;
const float LIMIT_GRIPPER_MIN   = 45.0;  // Giới hạn mở tối đa kẹp gắp ở góc 45 độ
const float LIMIT_GRIPPER_MAX   = 170.0;

// Tốc độ di chuyển góc servo (độ thay đổi góc ở mỗi chu kỳ loop)
const float SPEED_JOYSTICK = 1.2;
const float SPEED_BUTTONS  = 1.5;
const float SPEED_GRIPPER  = 4.5;  // Tốc độ đóng/mở kẹp gắp nhanh hơn theo yêu cầu

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
    connectionTime = millis(); // Lưu mốc thời gian kết nối
    Serial.println("Đã kết nối tay cầm thành công!");
  }
}

// Hàm tự động chạy khi tay cầm ngắt kết nối
void onDisconnectedController(ControllerPtr ctl) {
  if (tayCam == ctl) {
    tayCam = nullptr;
    Serial.println("Tay cầm đã ngắt kết nối!");
    dungLai(); // Xe tự phanh lại cho an toàn
  }
}

void setup() {
  Serial.begin(115200);

  // Khởi tạo các chân động cơ DC là đầu ra lệnh (OUTPUT)
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ép các chân động cơ về 0V ngay lập tức để khóa động cơ, chống nhiễu
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Khôi phục góc của các Servo từ Flash (mặc định là 90 độ nếu chạy lần đầu)
  preferences.begin("arm_pos", true);
  angleBaseLift = preferences.getFloat("base", 90.0);
  angleMid      = preferences.getFloat("mid", 90.0);
  angleOuter    = preferences.getFloat("outer", 90.0);
  angleGripper  = preferences.getFloat("gripper", 90.0);
  preferences.end();
  
  Serial.println("\n--- THÔNG TIN KHỞI ĐỘNG ---");
  Serial.printf("✓ Khôi phục góc cánh tay từ Flash: Đế=%d°, Giữa=%d°, Ngoài=%d°, Kẹp=%d°\n", 
                (int)angleBaseLift, (int)angleMid, (int)angleOuter, (int)angleGripper);
  Serial.println("✓ Chế độ Quiet Mode: Servo chưa được cấp điện. Sẽ chỉ bật khi tay cầm kết nối thành công.");
  Serial.println("----------------------------\n");

  // Khởi động trạm thu phát Bluetooth chờ tay cầm
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("Hệ thống đã sẵn sàng! Đang tìm kiếm tay cầm...");
}

bool servosAttached = false; // Cờ theo dõi trạng thái cấp nguồn servo

// Hàm cấp phát bộ định thời và cấp nguồn điều khiển cho các Servo
void attachServos() {
  if (servosAttached) return;

  Serial.println("\n[SERVO] Tay cầm đã kết nối. Đang kích hoạt và cấp nguồn cho toàn bộ các Servo...");

  // Cho phép cấp phát tài nguyên timer LEDC của ESP32 cho Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Thiết lập tần số 50Hz tiêu chuẩn cho Servo
  servoBaseLift1.setPeriodHertz(50);
  servoBaseLift2.setPeriodHertz(50);
  servoMid.setPeriodHertz(50);
  servoOuter.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  // Viết góc ban đầu được khôi phục từ Flash TRƯỚC khi attach để tránh giật/snap servo lúc khởi động
  servoBaseLift1.write((int)angleBaseLift);
  // - Áp dụng đúng chiều ngược cơ học cho Servo 4 (Chân 4)
  servoBaseLift2.write(180 - (int)angleBaseLift); 
  servoMid.write((int)angleMid);
  servoOuter.write((int)angleOuter);
  servoGripper.write((int)angleGripper);

  // Gắn chân điều khiển cho từng Servo sau khi đã thiết lập góc mặc định
  // SG90/MG996R hoạt động ổn định nhất trong khoảng độ rộng xung 500us - 2400us
  servoBaseLift1.attach(PIN_BASE_LIFT1, 500, 2400);
  servoBaseLift2.attach(PIN_BASE_LIFT2, 500, 2400);
  servoMid.attach(PIN_MID, 500, 2400);
  servoOuter.attach(PIN_OUTER, 500, 2400);
  servoGripper.attach(PIN_GRIPPER, 500, 2400);

  servosAttached = true;
  Serial.println("[SERVO] Đã hoàn thành! Toàn bộ Servo đang khóa giữ vị trí ổn định.");
}

void loop() {
  // Cập nhật liên tục trạng thái tay cầm
  BP32.update();

  if (tayCam && tayCam->isConnected()) {
    // 1. Kích hoạt và cấp nguồn Servo ngay khi tay cầm kết nối
    attachServos();

    // ==========================================
    // 1. ĐIỀU KHIỂN ĐỘNG CƠ DC (BÁNH XE)
    // ==========================================
    
    // Đọc phím chữ thập (D-pad) trước
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
      // Điều khiển trơn tru bánh xe bằng Joystick TRÁI (Analog Control)
      int trucY = tayCam->axisY(); // Trục dọc Joystick Trái (-512 đến 511)
      int trucX = tayCam->axisX(); // Trục ngang Joystick Trái (-512 đến 511)

      // Vùng chết (Deadzone) để tránh xe tự trôi khi thả tay gạt
      int deadzone = 40;
      if (abs(trucY) < deadzone) trucY = 0;
      if (abs(trucX) < deadzone) trucX = 0;

      if (trucY == 0 && trucX == 0) {
        dungLai();
        static unsigned long lastStopLog = 0;
        if (millis() - lastStopLog >= 1000) {
          lastStopLog = millis();
          Serial.println("[DONG CO LOG] XE DUNG (Không chạm Joystick Trái)");
        }
      } else {
        // Đảo chiều Y vì khi đẩy lên Joystick cho giá trị âm (-)
        float throttle = -trucY; 
        float steering = trucX;

        // Quy đổi từ [-512, 512] sang dải PWM động cơ [-255, 255]
        float v_throttle = (throttle / 512.0) * 255.0;
        float v_steering = (steering / 512.0) * 255.0;

        // Trộn kênh vi sai (Differential Steering)
        float leftSpeed = v_throttle + v_steering;
        float rightSpeed = v_throttle - v_steering;

        int speedL = constrain((int)leftSpeed, -255, 255);
        int speedR = constrain((int)rightSpeed, -255, 255);

        // Phát xung điều khiển
        setMotorLeft(speedL);
        setMotorRight(speedR);

        // In log tốc độ động cơ DC mỗi 500ms
        static unsigned long lastMotorLog = 0;
        if (millis() - lastMotorLog >= 500) {
          lastMotorLog = millis();
          String dirL = (speedL > 0) ? "TIEN" : ((speedL < 0) ? "LUI" : "DUNG");
          String dirR = (speedR > 0) ? "TIEN" : ((speedR < 0) ? "LUI" : "DUNG");
          Serial.printf("[DONG CO LOG] JoyL: (X=%d, Y=%d) | Bánh TRÁI: %s (PWM=%d) | Bánh PHẢI: %s (PWM=%d)\n",
                        trucX, trucY, dirL.c_str(), abs(speedL), dirR.c_str(), abs(speedR));
        }
      }
    }

    // ==========================================
    // 2. ĐIỀU KHIỂN CÁNH TAY ROBOT (SERVOS)
    // ==========================================
    updateServos();
  }
  delay(15); // Trễ ngắn để vòng lặp mượt mà và nhạy bén
}

// Hàm điều khiển cánh tay robot bằng Right Joystick và các nút bấm
void updateServos() {
  if (!tayCam || !tayCam->isConnected()) return;

  // Bảo vệ 1: Bỏ qua hoàn toàn lệnh trong 1.5 giây đầu kết nối tay cầm để tránh trôi nhiễu
  if (millis() - connectionTime < 1500) return;

  bool positionsChanged = false;

  // A. [ĐIỀU KHIỂN ĐỒNG THỜI 2 SERVO ĐẾ (BASE LIFT - CHÂN 15 & 4)]
  int ry = tayCam->axisRY(); 
  int deadzone = 70;
  if (abs(ry) < deadzone) ry = 0;

  if (ry != 0) {
    angleBaseLift += (-ry / 512.0) * SPEED_JOYSTICK;
    angleBaseLift = constrain(angleBaseLift, LIMIT_BASE_LIFT_MIN, LIMIT_BASE_LIFT_MAX);
    positionsChanged = true;
  }

  // B. [ĐIỀU KHIỂN SERVO GIỮA (KHỚP GIỮA - CHÂN 16)]
  // Yêu cầu: Ngược chiều kim đồng hồ (CCW) là đi lên.
  // -> Nút L1 (Giảm góc - CCW) -> Khớp giữa nâng lên.
  // -> Nút R1 (Tăng góc - CW) -> Khớp giữa hạ xuống.
  if (tayCam->l1()) { // Nút L1 -> Khớp giữa nâng lên (Giảm góc - CCW)
    angleMid -= SPEED_BUTTONS;
    angleMid = constrain(angleMid, LIMIT_MID_MIN, LIMIT_MID_MAX);
    positionsChanged = true;
  }
  if (tayCam->r1()) { // Nút R1 -> Khớp giữa hạ xuống (Tăng góc - CW)
    angleMid += SPEED_BUTTONS;
    angleMid = constrain(angleMid, LIMIT_MID_MIN, LIMIT_MID_MAX);
    positionsChanged = true;
  }

  // C. [ĐIỀU KHIỂN SERVO NGOÀI CÙNG (CỔ TAY - CHÂN 17)]
  // Yêu cầu: Quay cùng chiều kim đồng hồ (CW) là xuống. 
  // -> Nút Y (Tăng góc - CW) -> Hạ xuống.
  // -> Nút X (Giảm góc - CCW) -> Nâng lên.
  if (tayCam->x()) { // Nút X -> Cổ tay nâng lên (Giảm góc - CCW)
    angleOuter -= SPEED_BUTTONS;
    angleOuter = constrain(angleOuter, LIMIT_OUTER_MIN, LIMIT_OUTER_MAX);
    positionsChanged = true;
  }
  if (tayCam->y()) { // Nút Y -> Cổ tay hạ xuống (Tăng góc - CW - cùng chiều kim đồng hồ)
    angleOuter += SPEED_BUTTONS;
    angleOuter = constrain(angleOuter, LIMIT_OUTER_MIN, LIMIT_OUTER_MAX);
    positionsChanged = true;
  }

  // D. [ĐIỀU KHIỂN SERVO KẸP GẮP (CHÂN 5) - TỐC ĐỘ NHANH HƠN]
  // Yêu cầu: Ngược chiều kim đồng hồ (CCW) là mở, kẹp nhanh hơn.
  // -> Nút A (Giảm góc - CCW - tốc độ SPEED_GRIPPER) -> Mở kẹp gắp.
  // -> Nút B (Tăng góc - CW - tốc độ SPEED_GRIPPER) -> Đóng kẹp gắp.
  if (tayCam->a()) { // Nút A -> Mở kẹp gắp (Giảm góc - CCW)
    angleGripper -= SPEED_GRIPPER;
    angleGripper = constrain(angleGripper, LIMIT_GRIPPER_MIN, LIMIT_GRIPPER_MAX);
    positionsChanged = true;
  }
  if (tayCam->b()) { // Nút B -> Đóng kẹp gắp (Tăng góc - CW)
    angleGripper += SPEED_GRIPPER;
    angleGripper = constrain(angleGripper, LIMIT_GRIPPER_MIN, LIMIT_GRIPPER_MAX);
    positionsChanged = true;
  }

  // Chỉ ghi đè góc và xuất PWM khi có sự tương tác thực tế từ người dùng
  if (positionsChanged) {
    // Ghi góc điều khiển ra các Servo đang hoạt động
    servoBaseLift1.write((int)angleBaseLift);
    servoBaseLift2.write(180 - (int)angleBaseLift);
    servoMid.write((int)angleMid);
    servoOuter.write((int)angleOuter);
    servoGripper.write((int)angleGripper);

    // Lưu lại vị trí cánh tay vào Flash để khởi động lần sau không bị giật
    saveServosToFlash();
  }

  // In log góc Servo ra Serial monitor mỗi 500ms
  static unsigned long lastServoLog = 0;
  if (millis() - lastServoLog >= 500) {
    lastServoLog = millis();
    Serial.printf("[SERVO LOG] Đế(15,4): %d° | Giữa(16): %d° | Cổ tay(17): %d° | Kẹp(5): %d°\n",
                  (int)angleBaseLift, (int)angleMid, (int)angleOuter, (int)angleGripper);
  }
}

// Hàm lưu trữ thông số góc servo vào Flash
void saveServosToFlash() {
  static float lastSavedBase = -1;
  static float lastSavedMid = -1;
  static float lastSavedOuter = -1;
  static float lastSavedGripper = -1;
  
  // Chỉ lưu nếu thực sự có sự xê dịch góc so với lần lưu trước
  if (angleBaseLift == lastSavedBase && angleMid == lastSavedMid && angleOuter == lastSavedOuter && angleGripper == lastSavedGripper) {
    return;
  }
  
  // Giới hạn tần suất ghi tối đa 1 lần mỗi 2 giây để bảo vệ tuổi thọ bộ nhớ Flash
  static unsigned long lastSaveTime = 0;
  if (millis() - lastSaveTime < 2000) return;
  lastSaveTime = millis();
  
  preferences.begin("arm_pos", false);
  preferences.putFloat("base", angleBaseLift);
  preferences.putFloat("mid", angleMid);
  preferences.putFloat("outer", angleOuter);
  preferences.putFloat("gripper", angleGripper);
  preferences.end();
  
  lastSavedBase = angleBaseLift;
  lastSavedMid = angleMid;
  lastSavedOuter = angleOuter;
  lastSavedGripper = angleGripper;
  
  Serial.println("[PREFS] Đã tự động lưu vị trí Đế, Giữa, Cổ tay & Kẹp mới vào Flash!");
}

// --- CÁC HÀM XỬ LÝ DI CHUYỂN ĐỘNG CƠ DC ---
void diChuyenTien(int tocDo) {
  Serial.println("tiến");
  setMotorLeft(tocDo);
  setMotorRight(tocDo);
}
void diChuyenLui(int tocDo) {
  Serial.println("lùi");
  setMotorLeft(-tocDo);
  setMotorRight(-tocDo);
}
void quayTrai(int tocDo) {
  Serial.println("trái");
  setMotorLeft(-tocDo); // Trái lùi, phải tiến
  setMotorRight(tocDo);
}
void quayPhai(int tocDo) {
  Serial.println("phải");
  setMotorLeft(tocDo);  // Trái tiến, phải lùi
  setMotorRight(-tocDo);
}
void dungLai() {
  setMotorLeft(0);
  setMotorRight(0);
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

// Hàm điều khiển bánh bên trái (Cho phép chạy cả Tiến & Lùi trơn tru)
void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

// Hàm điều khiển bánh bên phải (Cho phép chạy cả Tiến & Lùi trơn tru)
void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}