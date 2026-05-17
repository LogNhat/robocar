# Robocar ESP32 Line Follower - Wiring

## Cấu hình phần cứng
- ESP32 DevKit (30 chân)
- QTR-8 Analog (IR + VCC + GND + D1-D8)
- TB6612FNG
- 2x N20 DC Motor
- LiPo 2S
- Buck converter (hạ áp)
- Tụ 330uF

---

# 1. ESP32 ↔ QTR-8 Analog

| QTR-8 | ESP32 | Ghi chú |
|------|-------|---------|
| VCC | 3V3 | cấp nguồn cảm biến |
| GND | GND | mass chung |
| IR | GPIO27 | bật LED IR |
| D1 | GPIO36 | ADC |
| D2 | GPIO39 | ADC |
| D3 | GPIO34 | ADC |
| D4 | GPIO35 | ADC |
| D5 | GPIO32 | ADC |
| D6 | GPIO33 | ADC |
| D7 | GPIO25 | ADC |
| D8 | GPIO26 | ADC |

---

# 2. ESP32 ↔ TB6612FNG

## Logic pins

| TB6612 | ESP32 | Chức năng |
|--------|-------|-----------|
| STBY | GPIO23 | enable driver |
| PWMA | GPIO18 | PWM motor trái |
| AIN1 | GPIO19 | chiều motor trái |
| AIN2 | GPIO21 | chiều motor trái |
| PWMB | GPIO5 | PWM motor phải |
| BIN1 | GPIO17 | chiều motor phải |
| BIN2 | GPIO16 | chiều motor phải |
| VCC | 3V3 | logic driver |
| GND | GND | mass chung |

---

# 3. TB6612 ↔ Motors

## Motor trái
| TB6612 | Motor |
|--------|-------|
| A01 | dây motor trái |
| A02 | dây motor trái |

## Motor phải
| TB6612 | Motor |
|--------|-------|
| B01 | dây motor phải |
| B02 | dây motor phải |

> Nếu motor quay ngược chỉ cần đảo 2 dây motor đó.

---

# 4. Nguồn

## LiPo 2S

| Nguồn | Kết nối |
|------|---------|
| LiPo + | TB6612 VM |
| LiPo - | TB6612 GND |

---

# 5. Buck Converter

## Input
| Buck | Nối |
|------|-----|
| IN+ | LiPo + |
| IN- | LiPo - |

## Output
| Buck | Nối |
|------|-----|
| OUT+ | ESP32 VIN / 5V |
| OUT- | ESP32 GND |

---

# 6. Tụ 330uF

## Cách nối
Mắc song song với nguồn motor:

| Tụ | Nối |
|----|-----|
| (+) | cùng điểm với TB6612 VM |
| (-) | cùng điểm với TB6612 GND |

Sơ đồ:

LiPo+ ----+---- TB6612 VM
          |
         TỤ (+)

LiPo- ----+---- TB6612 GND
          |
         TỤ (-)

## Lưu ý
- chân dài = +
- bên có sọc = -

KHÔNG cắm ngược cực.

---

# 7. Pin map code

```cpp
QTR
IR   = 27
D1   = 36
D2   = 39
D3   = 34
D4   = 35
D5   = 32
D6   = 33
D7   = 25
D8   = 26

TB6612
STBY = 23
PWMA = 18
AIN1 = 19
AIN2 = 21
PWMB = 5
BIN1 = 17
BIN2 = 16
```

---

# 8. Chân còn trống (nâng cấp sau)

## I2C
| ESP32 |
|------|
| GPIO22 SDA |
| GPIO4 SCL |

Dùng cho:
- MPU6050
- BNO085
- VL53L0X

---

# 9. Quy ước robot

Nhìn theo hướng robot chạy:

[ Motor trái ]   [ Motor phải ]

        ↑
   hướng tiến

TB6612:
- Channel A = motor trái
- Channel B = motor phải