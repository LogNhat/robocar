# Robocar ESP32 Bluetooth Controller (L298N + Bluepad32) - Wiring (`mapingdk.md`)

Tài liệu này hướng dẫn chi tiết sơ đồ đấu nối phần cứng và giải thích cấu trúc thuật toán điều khiển cho xe Robocar điều khiển từ xa bằng Tay cầm Bluetooth (PS4, Xbox,...) qua thư viện **Bluepad32** và mạch cầu H **L298N** dựa theo cấu hình thực tế trong code `main.cpp`.

## Cấu hình phần cứng
- **ESP32 DevKit** (30 chân)
- **Mạch cầu H L298N** (Dual H-Bridge Motor Driver)
- **Tay cầm Bluetooth** (PS4, PS5, Xbox One, Nintendo Switch Pro,...) hỗ trợ kết nối BLE/Classic qua Bluepad32
- **2x N20 DC Motor** (Động cơ bánh TRÁI và bánh PHẢI)
- **LiPo 2S Battery** (Nguồn cấp chính 7.4V - 8.4V)
- **Buck Converter** (Hạ áp xuống 5V cấp nguồn cho ESP32 nếu không dùng đầu ra 5V của L298N)
- **Tụ 330uF** (Lọc nguồn bảo vệ vi điều khiển)

---

# 1. ESP32 ↔ L298N Motor Driver

Mạch L298N điều khiển hai cụm động cơ độc lập qua các chân Enable (PWM) và các chân Input điều khiển hướng (Logic).

| L298N | ESP32 (GPIO) | Loại tín hiệu | Chức năng đấu nối thực tế |
| :---: | :----------: | :----------: | :------------------------ |
| **ENA** | **GPIO 14**  | PWM (Output)  | Điều khiển tốc độ **Cụm bánh TRÁI** |
| **IN1** | **GPIO 27**  | Logic (Output)| Chiều quay **Cụm bánh TRÁI** |
| **IN2** | **GPIO 26**  | Logic (Output)| Chiều quay **Cụm bánh TRÁI** |
| **IN3** | **GPIO 25**  | Logic (Output)| Chiều quay **Cụm bánh PHẢI** |
| **IN4** | **GPIO 33**  | Logic (Output)| Chiều quay **Cụm bánh PHẢI** |
| **ENB** | **GPIO 32**  | PWM (Output)  | Điều khiển tốc độ **Cụm bánh PHẢI** |
| **GND** | GND          | Nguồn         | Mass chung (CỰC KỲ QUAN TRỌNG) |

---

# 2. L298N ↔ Động cơ (Motors)

Các chân đầu ra công suất của L298N kết nối trực tiếp với 2 cụm động cơ.

## Cụm bánh TRÁI (Left Motor)
| L298N Output | Dây Động cơ | Ghi chú đấu nối thực tế |
| :----------: | :---------: | :---------------------- |
| **OUT1**     | Dây Đỏ      | Phía cực dương động cơ Trái |
| **OUT2**     | Dây Đen     | Phía cực âm động cơ Trái |

## Cụm bánh PHẢI (Right Motor)
| L298N Output | Dây Động cơ | Ghi chú đấu nối thực tế |
| :----------: | :---------: | :---------------------- |
| **OUT3**     | Dây Đỏ      | Phía cực dương động cơ Phải |
| **OUT4**     | Dây Đen     | Phía cực âm động cơ Phải |

> [!TIP]
> Nếu robot di chuyển ngược với tay cầm điều khiển (ví dụ: đẩy cần tiến nhưng robot lùi), bạn có thể dễ dàng sửa bằng cách đảo vị trí 2 dây của động cơ trên cổng domino OUT tương ứng mà không cần sửa code.

---

# 3. Kết nối nguồn cấp & Mass chung

Do động cơ DC tiêu thụ dòng lớn và sinh ra xung nhiễu lớn, việc phân tách nguồn và thiết lập mass chung là bắt buộc.

```
                      +-------------------+
                      |      PIN LiPo     |
                      |    (7.4V - 8.4V)  |
                      +---+-----------+---+
                          |           |
                 (+) / VCC|           |(-) / GND
                          |           +-----------------------+
                          v           v                       |
                  +-------+-----------+-------+               |
                  |     Mạch Cầu H L298N      |               |
                  |  - Cổng 12V: Nhận Pin     |               |
                  |  - Cổng GND: Mass chung   |               |
                  +---+-------------------+---+               |
                      |                                       |
              5V Out  | (Nếu bật Jump 5V L298N)               |
              (Hoặc   v                                       |
             từ Buck) +-------------------+                   |
                      |   ESP32 VIN / 5V  |                   |
                      +---+---------------+                   |
                          |                                   |
                          v GND                               v
             =============+===================================+========= (GND Mass chung)
```

> [!WARNING]
> Phải kết nối chân **GND** của ESP32 với chân **GND** của mạch cầu H L298N và cực âm (-) của pin. Nếu không nối chung mass GND, tín hiệu điều khiển PWM và Logic sẽ bị nhiễu loạn và xe không hoạt động được!

---

# 4. Tóm tắt mã nguồn cấu hình chân (Pin Map Code)

Trích đoạn khai báo chân từ `main.cpp`:

```cpp
// --- SƠ ĐỒ CHÂN THEO ĐÚNG ĐẤU NỐI THỰC TẾ ---
int IN1 = 27;
int IN2 = 26;
int ENA = 14; // OUT1 & OUT2 -> Cụm bánh TRÁI (Đỏ OUT1, Đen OUT2)

int IN3 = 25;
int IN4 = 33;
int ENB = 32; // OUT3 & OUT4 -> Cụm bánh PHẢI (Đỏ OUT3, Đen OUT4)
```

---

# 5. Cấu trúc và Thuật toán điều khiển trong `main.cpp`

Hệ thống điều khiển qua tay cầm Bluetooth có hai chế độ hoạt động chính:

## A. Điều khiển bằng Phím Chữ Thập (D-pad) - Chế độ số
Khi nhấn các nút D-pad, xe di chuyển theo các hướng định sẵn với tốc độ cố định:
- **DPAD_UP (Tiến):** Cả hai bánh quay tiến với tốc độ **250** (`diChuyenTien`).
- **DPAD_DOWN (Lùi):** Cả hai bánh quay lùi với tốc độ **250** (`diChuyenLui`).
- **DPAD_LEFT (Quay trái tại chỗ):** Bánh trái lùi, bánh phải tiến với tốc độ **200** (`quayTrai`).
- **DPAD_RIGHT (Quay phải tại chỗ):** Bánh trái tiến, bánh phải lùi với tốc độ **200** (`quayPhai`).

## B. Điều khiển Trơn Tru bằng Analog Joystick (Cần gạt Trái) - Trộn kênh vi sai (Differential Steering)
Khi không nhấn D-pad, hệ thống sẽ đọc giá trị cần Analog trái (`axisX` và `axisY` có khoảng giá trị từ `-512` đến `511`):
1. **Lọc vùng chết (Deadzone = 40):** Để loại bỏ sai số nhỏ ở tâm cần gạt khi không chạm vào, giữ xe đứng im hoàn toàn.
2. **Quy đổi dải giá trị:** Chuyển đổi tuyến tính từ dải `[-512, 512]` sang dải điều khiển PWM động cơ `[-255, 255]`.
3. **Thuật toán trộn kênh vi sai:**
   - Trục Y điều khiển ga kéo (`throttle`): Đẩy lên là Tiến (+), kéo xuống là Lùi (-).
   - Trục X điều khiển hướng rẽ (`steering`): Gạt phải rẽ Phải (+), gạt trái rẽ Trái (-).
   - Công thức phối hợp tốc độ hai bánh:
     $$\text{leftSpeed} = \text{throttle} + \text{steering}$$
     $$\text{rightSpeed} = \text{throttle} - \text{steering}$$
   - **Giới hạn tốc độ (Constrain):** Giữ tốc độ bánh trong dải an toàn `[-255, 255]`.
   - Kết quả: Khi đẩy tiến và gạt nhẹ sang phải, bánh trái sẽ quay nhanh hơn bánh phải giúp xe cua cực kỳ mượt mà.

---

# 6. Quy ước hướng Robot và Motor

```
             [ Bánh TRÁI ]                 [ Bánh PHẢI ]
              (Cụm OUT1/2)                  (Cụm OUT3/4)
           (ENA, IN1, IN2)               (ENB, IN3, IN4)
       
                  ^                             ^
                  |                             |
                  +-------- HƯỚNG TIẾN ---------+
```
