#include <Arduino.h>   // BẮT BUỘC CÓ KHI DÙNG VS CODE
#include <Bluepad32.h> // Thư viện đọc tay cầm

ControllerPtr tayCam = nullptr; // Khai báo tên tay cầm

// --- SƠ ĐỒ CHÂN BẠN ĐÃ THIẾT KẾ ---
int IN1 = 27;
int IN2 = 26;
int ENA = 14; // Cụm bánh trái

int IN3 = 25;
int IN4 = 33;
int ENB = 32; // Cụm bánh phải

// KHAI BÁO TRƯỚC CÁC HÀM DI CHUYỂN (Dành cho VS Code/C++ chuẩn)
void diChuyenTien(int tocDo);
void diChuyenLui(int tocDo);
void quayTrai(int tocDo);
void quayPhai(int tocDo);
void dungLai();

// Hàm tự động chạy khi tay cầm kết nối
void onConnectedController(ControllerPtr ctl) {
  if (tayCam == nullptr) {
    tayCam = ctl;
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

  // Khởi tạo các chân là đầu ra lệnh (OUTPUT)
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ép các chân về 0V ngay lập tức để khóa động cơ, chống nhiễu
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Khởi động trạm thu phát Bluetooth chờ tay cầm
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("Hệ thống đã sẵn sàng! Đang tìm kiếm tay cầm...");
}

void loop() {
  // Cập nhật liên tục trạng thái tay cầm
  BP32.update();

  if (tayCam && tayCam->isConnected()) {

    // 1. ĐỌC PHÍM CHỮ THẬP (D-PAD)
    uint8_t dpad = tayCam->dpad();
    if (dpad == DPAD_UP) {
      Serial.println("D-PAD: UP -> Tiến");
      diChuyenTien(250);
    } else if (dpad == DPAD_DOWN) {
      Serial.println("D-PAD: DOWN -> Lùi");
      diChuyenLui(250);
    } else if (dpad == DPAD_LEFT) {
      Serial.println("D-PAD: LEFT -> Xoay trái");
      quayTrai(200);
    } else if (dpad == DPAD_RIGHT) {
      Serial.println("D-PAD: RIGHT -> Xoay phải");
      quayPhai(200);
    }

    else {
      // 2. NẾU KHÔNG BẤM D-PAD -> ĐỌC CẦN GẠT JOYSTICK TRÁI
      // Giá trị trục X, Y của Joystick chạy từ khoảng -511 đến 511
      int trucY = tayCam->axisY();
      int trucX = tayCam->axisX();

      if (trucY <= -150) {
        Serial.printf("JOYSTICK: Y=%d -> Tiến\n", trucY);
        diChuyenTien(250);
      } // Đẩy cần lên
      else if (trucY >= 150) {
        Serial.printf("JOYSTICK: Y=%d -> Lùi\n", trucY);
        diChuyenLui(250);
      } // Kéo cần xuống
      else if (trucX <= -150) {
        Serial.printf("JOYSTICK: X=%d -> Xoay trái\n", trucX);
        quayTrai(200);
      } // Gạt sang trái
      else if (trucX >= 150) {
        Serial.printf("JOYSTICK: X=%d -> Xoay phải\n", trucX);
        quayPhai(200);
      } // Gạt sang phải
      else {
        // Tránh in liên tục khi không làm gì
        // Serial.println("Dừng");
        dungLai();
      } // Không chạm vào tay cầm thì dừng xe
      
      // In giá trị trục (chỉ in khi có thay đổi đáng kể để tránh nghẽn Serial)
      static int lastX = 0, lastY = 0;
      if (abs(trucX - lastX) > 20 || abs(trucY - lastY) > 20) {
        Serial.printf("Truc X: %d | Truc Y: %d\n", trucX, trucY);
        lastX = trucX;
        lastY = trucY;
      }
    }
  }
  delay(10); // Nghỉ 10 mili-giây cho hệ thống mượt mà
}

// --- CÁC HÀM XỬ LÝ DI CHUYỂN ---
void diChuyenTien(int tocDo) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, tocDo);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, tocDo);
}
void diChuyenLui(int tocDo) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, tocDo);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, tocDo);
}
void quayTrai(int tocDo) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, tocDo);
}
void quayPhai(int tocDo) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, tocDo);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
void dungLai() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}