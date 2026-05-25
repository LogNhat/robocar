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

# 2. ESP32 ↔ Động cơ Servo (Cánh tay Robot)

Cánh tay robot sử dụng 5 động cơ Servo được kết nối trực tiếp với các chân GPIO của ESP32 và được điều khiển bằng Tay cầm Bluetooth qua cần gạt Analog PHẢI (Right Joystick) và các nút phụ trợ.

| Động cơ Servo | ESP32 (GPIO) | Điều khiển tay cầm | Chức năng chi tiết |
| :--- | :---: | :--- | :--- |
| **Servo Đế 1 (Base Lift 1)** | **GPIO 15** | **Right Joystick Y** (Lên/Xuống) | Nâng/hạ khớp vai (Đế - Servo 1) |
| **Servo Đế 2 (Base Lift 2)** | **GPIO 4** | **Right Joystick Y** (Lên/Xuống) | Nâng/hạ khớp vai (Đế - Servo 2, đồng bộ cùng Servo 1) |
| **Servo Giữa (Elbow)** | **GPIO 16** | **Nút L1 / R1** | Co/duỗi khớp khuỷu tay (giữa cánh tay) |
| **Servo Ngoài Cùng (Wrist)** | **GPIO 17** | **Nút X / Y** | Nâng/hạ khớp cổ tay (sát kẹp gắp) |
| **Servo Kẹp Gắp (Gripper)** | **GPIO 5** | **Nút A / B** | Mở/đóng kẹp gắp vật thể |

> [!NOTE]
> Do 5 động cơ Servo tiêu thụ dòng điện tức thời rất lớn khi hoạt động đồng thời, **KHÔNG ĐƯỢC** cấp nguồn trực tiếp từ chân 5V/3.3V của ESP32. Hãy sử dụng nguồn ngoài (như mạch Buck hạ áp xuống 5V - 6V, dòng tối thiểu 3A - 5A) và **phải nối chung Mass (GND)** với ESP32!

---

# 3. L298N ↔ Động cơ (Motors)

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

# 4. Kết nối nguồn cấp & Mass chung

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

# 5. Tóm tắt mã nguồn cấu hình chân (Pin Map Code)

Trích đoạn khai báo chân từ `main.cpp`:

```cpp
// --- SƠ ĐỒ CHÂN ĐỘNG CƠ DC THEO ĐÚNG ĐẤU NỐI THỰC TẾ ---
int IN1 = 27;
int IN2 = 26;
int ENA = 14; // OUT1 & OUT2 -> Cụm bánh TRÁI (Đỏ OUT1, Đen OUT2)

int IN3 = 25;
int IN4 = 33;
int ENB = 32; // OUT3 & OUT4 -> Cụm bánh PHẢI (Đỏ OUT3, Đen OUT4)

// --- SƠ ĐỒ CHÂN CÁC ĐỘNG CƠ SERVO CÁNH TAY ROBOT ---
const int PIN_BASE_ROT   = 15; // Servo xoay đế 1 (Base Rotation)
const int PIN_BASE_LIFT  = 4;  // Servo nâng đế 2 (Base Lift)
const int PIN_MID        = 16; // Servo khớp giữa (Elbow)
const int PIN_OUTER      = 17; // Servo khớp ngoài cùng, sát kẹp gắp (Wrist)
const int PIN_GRIPPER    = 5;  // Servo khớp kẹp gắp (Gripper)
```

---

# 6. Cấu trúc và Thuật toán điều khiển trong `main.cpp`

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

## C. Điều khiển Trơn Tru Cánh Tay Robot bằng Cần gạt Phải (Right Joystick) & Các Nút
Hệ thống điều khiển cánh tay robot sử dụng các cơ chế nội suy góc và giới hạn hành trình thông minh để bảo vệ cơ cấu cơ khí:
1. **Điều khiển Analog cho 2 Servo Đế (Base Lift):**
   - Trục Y cần Analog Phải (`axisRY`): Điều khiển đồng thời cả **Servo Đế 1 (Pin 15)** và **Servo Đế 2 (Pin 4)** từ `0°` đến `180°` để nâng/hạ cánh tay. Đẩy lên để nâng khớp đế cánh tay lên, kéo xuống để hạ xuống. Hai Servo hoạt động đồng hành cùng nhau (đồng bộ hoặc ngược chiều tùy cách lắp ráp cơ khí) giúp chịu tải tốt hơn.
   - Trục X cần Analog Phải (`axisRX`): Không sử dụng (vì khớp đế chỉ có nâng/hạ lên xuống chứ không xoay ngang).
2. **Điều khiển Khớp Giữa (Elbow) và Khớp Ngoài Cùng (Wrist) bằng Nút:**
   - **Nút L1 / R1:** Co / duỗi khớp giữa **Servo Giữa (Pin 16)** từ `0°` đến `180°`. Giúp gập và mở rộng tầm với của cánh tay.
   - **Nút X / Y:** Nâng / hạ khớp cổ tay **Servo Ngoài Cùng (Pin 17)** từ `0°` đến `180°`. Giúp chỉnh hướng kẹp gắp vật thể.
3. **Điều khiển Kẹp Gắp (Gripper) bằng Nút A / B:**
   - **Nút A:** Mở rộng kẹp gắp (giới hạn an toàn tối thiểu `10°`).
   - **Nút B:** Đóng khít kẹp gắp để ôm vật thể (giới hạn an toàn tối đa `170°`).
4. **Cơ chế nội suy trơn mượt & bảo vệ (Interpolation & Protection):**
   - Xe liên tục cộng / trừ góc điều khiển dựa trên độ lệch của cần gạt và trạng thái giữ nút bấm ở mỗi chu kỳ (`loop()`). Cánh tay sẽ di chuyển mượt mà, không bị giật cục.
   - Khóa góc giới hạn tối đa và tối thiểu (`constrain()`) để các bánh răng servo không bị kẹt cứng hoặc làm hỏng phần cứng khi chạm mốc vật lý.

---

# 7. Quy ước hướng Robot và Motor

```
             [ Bánh TRÁI ]                 [ Bánh PHẢI ]
              (Cụm OUT1/2)                  (Cụm OUT3/4)
           (ENA, IN1, IN2)               (ENB, IN3, IN4)
       
                  ^                             ^
                  |                             |
                  +-------- HƯỚNG TIẾN ---------+
```
