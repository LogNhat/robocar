# Robocar ESP32 Line Follower (Gyro + QTR-8) - Wiring (`mapingth.md`)

Tài liệu này hướng dẫn chi tiết sơ đồ đấu nối phần cứng cho xe Robocar dò line (Line Follower) kết hợp cảm biến Gyro (MPU6050) và cảm biến line 8 kênh QTR-8 Analog dựa theo cấu hình thực tế trong code `th.text`.

## Cấu hình phần cứng
- **ESP32 DevKit** (30 chân)
- **QTR-8 Analog Sensor** (8 kênh dò line: D1-D8)
- **MPU6050 Sensor** (Cảm biến gia tốc & con quay hồi chuyển Gyro)
- **TB6612FNG** (Mạch điều khiển động cơ Dual H-Bridge)
- **2x N20 DC Motor** (Động cơ DC giảm tốc)
- **LiPo 2S Battery** (Nguồn cấp chính 7.4V - 8.4V)
- **Buck Converter** (Hạ áp xuống 5V cấp nguồn cho ESP32)
- **Tụ 330uF** (Lọc nhiễu nguồn động cơ)

---

# 1. ESP32 ↔ QTR-8 Analog

Cảm biến QTR-8 Analog sử dụng 8 chân ADC của ESP32. Thứ tự các chân cảm biến được cấu hình trong mảng `sensorPins` theo thứ tự từ phải sang trái hoặc ngược lại tùy theo cách lắp đặt (trong code, mảng cảm biến được định nghĩa là `{26, 25, 33, 32, 35, 34, 39, 36}` ứng với `D8 -> D1`).

| QTR-8 | ESP32 (GPIO) | Loại chân | Ghi chú |
| :---: | :----------: | :-------: | :------ |
| **VCC**| 3V3          | Nguồn     | Cấp nguồn logic 3.3V |
| **GND**| GND          | Nguồn     | Mass chung |
| **IR** | 3V3 / VCC    | Điều khiển| Không dùng chân điều khiển IR trong code (nối lên VCC/3V3 để luôn bật LED IR) |
| **D1** | **GPIO 36**  | ADC (VP)  | Cảm biến 1 |
| **D2** | **GPIO 39**  | ADC (VN)  | Cảm biến 2 |
| **D3** | **GPIO 34**  | ADC       | Cảm biến 3 |
| **D4** | **GPIO 35**  | ADC       | Cảm biến 4 |
| **D5** | **GPIO 32**  | ADC       | Cảm biến 5 |
| **D6** | **GPIO 33**  | ADC       | Cảm biến 6 |
| **D7** | **GPIO 25**  | ADC       | Cảm biến 7 |
| **D8** | **GPIO 26**  | ADC       | Cảm biến 8 |

---

# 2. ESP32 ↔ MPU6050 (Gyro Sensor) and vl53l0x

Cảm biến MPU6050 giao tiếp qua giao thức I2C, được khởi tạo tại địa chỉ `0x68` với các chân I2C tùy biến trong hàm `Wire.begin(21, 22)`.

| MPU6050 / VL53L0X | ESP32 (GPIO) | Ghi chú |
| :---------------: | :----------: | :------ |
| **VCC**           | 3V3 / 5V     | Nguồn logic (Khuyến nghị cấp 5V qua mạch ổn áp cảm biến) |
| **GND**           | GND          | Mass chung |
| **SDA**           | **GPIO 21**  | Chân dữ liệu I2C SDA chung |
| **SCL**           | **GPIO 22**  | Chân xung nhịp I2C SCL chung |
| **XSHUT CENTER**  | **GPIO 17**  | Chân điều khiển bật/tắt cảm biến Giữa |
| **XSHUT LEFT**    | **GPIO 15**  | Chân điều khiển bật/tắt cảm biến Trái |
| **XSHUT RIGHT**   | **GPIO 16**  | Chân điều khiển bật/tắt cảm biến Phải |

---

# 3. ESP32 ↔ TB6612FNG

Mạch cầu H TB6612FNG dùng để điều khiển hướng và tốc độ của hai động cơ. 

> [!NOTE]
> Trong code `th.text`, **Động cơ TRÁI** được gán cho **Kênh B (Channel B)** và **Động cơ PHẢI** được gán cho **Kênh A (Channel A)**.
> Chân kích hoạt driver **STBY** không được cấu hình trong code, do đó cần nối trực tiếp chân này lên nguồn 3.3V (VCC) để mạch luôn hoạt động.

| TB6612 | ESP32 (GPIO) | Chức năng | Ghi chú |
| :----: | :----------: | :-------- | :------ |
| **PWMA**| **GPIO 27**  | PWM động cơ PHẢI | Điều khiển tốc độ bánh bên phải |
| **AIN1**| **GPIO 18**  | Chiều động cơ PHẢI | Hướng quay bánh bên phải |
| **AIN2**| **GPIO 19**  | Chiều động cơ PHẢI | Hướng quay bánh bên phải |
| **PWMB**| **GPIO 13**  | PWM động cơ TRÁI  | Điều khiển tốc độ bánh bên trái |
| **BIN1**| **GPIO 5**   | Chiều động cơ TRÁI  | Hướng quay bánh bên trái |
| **BIN2**| **GPIO 23**  | Chiều động cơ TRÁI  | Hướng quay bánh bên trái |
| **STBY**| 3V3 / High   | Enable Driver      | Nối trực tiếp lên nguồn 3.3V của ESP32 |
| **VCC** | 3V3          | Nguồn logic        | Cấp nguồn logic điều khiển mạch |
| **GND** | GND          | Mass chung         | Nối với cực âm GND của hệ thống |

---

# 4. TB6612 ↔ Động cơ

## Động cơ TRÁI (Motor Left)
| TB6612 | Kết nối Động cơ |
| :----: | :-------------- |
| **B01**| Chân động cơ trái 1 |
| **B02**| Chân động cơ trái 2 |

## Động cơ PHẢI (Motor Right)
| TB6612 | Kết nối Động cơ |
| :----: | :-------------- |
| **A01**| Chân động cơ phải 1 |
| **A02**| Chân động cơ phải 2 |

> [!TIP]
> Nếu sau khi lắp ráp và chạy thử, động cơ nào quay ngược hướng so với mong muốn (ví dụ: lệnh Tiến nhưng bánh quay Lùi), bạn chỉ cần đảo ngược vị trí hai dây nối từ động cơ đó vào mạch cầu H.

---

# 5. Sơ đồ nguồn & Buck Converter

Nguồn năng lượng chính của hệ thống là pin LiPo 2S (7.4V - 8.4V). Cần chia làm 2 nhánh nguồn: Nguồn công suất cấp trực tiếp cho động cơ qua chân VM của TB6612FNG và nguồn điều khiển được hạ áp qua Buck Converter cấp cho ESP32.

```
                  +--------------+
                  |  LiPo 2S     |
                  |  (7.4V-8.4V) |
                  +---+------+---+
                      |      |
             (+) / VCC|      |(-) / GND
                      |      +------------------------+
                      +-------------------+           |
                      |                   |           |
             +--------+--------+     +----+----+      |
             |   Buck Converter|     | TB6612  |      |
             |  (Hạ áp xuống 5V)|    |   VM    |      |
             +--------+--------+     +----+----+      |
                      |                   |           |
               OUT(+) | 5V                |           |
                      v                   |           |
             +--------+--------+          |           |
             |   ESP32 VIN     |          |           |
             +--------+--------+          |           |
                      |                   |           |
                      v GND               v           v
             =========+===================+===========+========= (Mass chung GND)
```

## Kết nối Buck Converter
- **Đầu vào (Input):**
  - `IN+` ↔ Cực dương (+) Pin LiPo
  - `IN-` ↔ Cực âm (-) Pin LiPo
- **Đầu ra (Output):**
  - `OUT+` (Đã điều chỉnh về 5V) ↔ Chân **VIN** (hoặc chân 5V) của ESP32
  - `OUT-` ↔ Chân **GND** của ESP32

---

# 6. Tụ lọc nguồn động cơ (330uF)

Để giảm nhiễu sinh ra từ động cơ chổi than DC ảnh hưởng tới vi điều khiển ESP32 và cảm biến Gyro/Analog, cần mắc song song tụ điện hóa 330uF (hoặc lớn hơn) vào đầu nguồn cấp động cơ của TB6612FNG.

| Tụ hóa | Kết nối |
| :----: | :------ |
| **Cực dương (+)** | Điểm nối nguồn dương `VM` của mạch TB6612FNG |
| **Cực âm (-)**    | Điểm nối đất `GND` của mạch TB6612FNG |

> [!CAUTION]
> Tụ hóa có phân biệt cực tính (chân dài hơn là cực dương, bên hông có sọc màu xám/trắng kí hiệu dấu trừ `-` là cực âm). Tuyệt đối **KHÔNG** cắm ngược cực của tụ vì có thể gây cháy nổ tụ và hỏng mạch.

---

# 7. Tóm tắt mã nguồn cấu hình chân (Pin Map Code)

Dưới đây là trích đoạn mã nguồn trong `th.text` quy định cấu hình chân:

```cpp
// Cảm biến dò line QTR-8 Analog
const int numSensors = 8;
const int sensorPins[numSensors] = {26, 25, 33, 32, 35, 34, 39, 36}; // D8, D7, D6, D5, D4, D3, D2, D1

// Giao tiếp I2C cho MPU6050 Gyro
// Wire.begin(21, 22); // SDA = 21, SCL = 22

// Động cơ TB6612FNG
const int PWMA = 27; // Điều khiển tốc độ bánh Phải (Kênh A)
const int AIN1 = 18; // Chiều bánh Phải
const int AIN2 = 19; // Chiều bánh Phải

const int PWMB = 13; // Điều khiển tốc độ bánh Trái (Kênh B)
const int BIN1 = 5;  // Chiều bánh Trái
const int BIN2 = 23; // Chiều bánh Trái
```

---

# 8. Quy ước hướng Robot

Hướng quy ước khi nhìn robot từ phía sau (nhìn theo hướng di chuyển tiến):

```
       [ Bánh TRÁI ]               [ Bánh PHẢI ]
         (Kênh B)                    (Kênh A)
      (BIN1, BIN2, PWMB)          (AIN1, AIN2, PWMA)
       
              ^                      ^
              |                      |
              +------- HƯỚNG TIẾN ---+
```
