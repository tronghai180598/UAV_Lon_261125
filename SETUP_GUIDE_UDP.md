# Hướng Dẫn Kết Nối Qt với ESP32 qua ESP8266

## 🔌 Kiến Trúc Hệ Thống

```
Qt Application (PC)
    ↓ UDP Port 14550
ESP8266 (WiFi AP: 192.168.4.1)
    ↓ UART (Serial1)
ESP32 (Drone Controller)
```

## 📋 Cấu Hình Qt

### 1. Thay Đổi trong `Qt_Flix/dialog.h`
- ✅ Thêm `QUdpSocket *udpSocket`
- ✅ Thêm `QHostAddress espAddress` 
- ✅ Thêm hàm `setupUdp()` và `onUdpReadyRead()`

### 2. Thay Đổi trong `Qt_Flix/dialog.cpp`
- ✅ Loại bỏ Serial, thay thế bằng UDP
- ✅ Constructor khởi tạo UDP socket
- ✅ `sendCommand()` gửi qua UDP thay vì Serial
- ✅ `onUdpReadyRead()` nhận dữ liệu từ ESP8266

### 3. Cấu Hình WiFi
Thay đổi IP và port trong `dialog.cpp`:
```cpp
espAddress = QHostAddress("192.168.4.1");  // IP AP của ESP8266
espPort = 14550;                            // Port UDP
```

## 🔧 Cấu Hình ESP8266

### 1. File: `flixESPDrone/esp32_relay.ino` (MỚI)
- Khởi tạo UART1 với ESP32
- Gửi lệnh từ Qt tới ESP32 qua UART
- Nhận phản hồi từ ESP32

### 2. Chỉnh Sửa Pin UART1
Tùy theo board ESP8266 của bạn:
```cpp
#define ESP32_RX_PIN 9    // ESP8266 RX1 (đổi theo board)
#define ESP32_TX_PIN 10   // ESP8266 TX1 (đổi theo board)
```

### 3. Kết Nối Vật Lý: ESP8266 ↔ ESP32
```
ESP8266 TX1 (GPIO10) → ESP32 RX0
ESP8266 RX1 (GPIO9)  ← ESP32 TX0
GND → GND (chung)
```

### 4. File: `flixESPDrone/mavlink.ino` (CẬP NHẬT)
- ✅ `processMavlink()` gọi `sendCommandToESP32()`
- ✅ Gọi `receiveFromESP32()` để nhận phản hồi

## 🚀 Cách Sử Dụng

### Bước 1: Cấp Nguồn và Kết Nối
1. Cấp nguồn cho ESP8266 (connected với ESP32 qua UART)
2. Đợi ESP8266 bật WiFi AP

### Bước 2: Kết Nối Qt
1. Chạy ứng dụng Qt
2. Kết nối máy tính với WiFi ESP8266: SSID=`flix`, Password=`flixwifi`
3. Check Debug console - Qt sẽ hiển thị kết nối UDP

### Bước 3: Gửi Lệnh
Bấm nút trong Qt → Lệnh gửi qua UDP → ESP8266 relay qua UART → ESP32 thực thi

## 📊 Luồng Dữ Liệu

### Gửi Lệnh (Qt → ESP32):
```
Qt: sendCommand("arm")
  ↓ UDP Datagram
ESP8266: receiveWiFi() nhận "arm"
  ↓ processMavlink() gọi sendCommandToESP32()
  ↓ UART Serial1.write() tới ESP32
ESP32: nhận "arm" qua Serial0
```

### Nhận Phản Hồi (ESP32 → Qt):
```
ESP32: Serial0.write() phản hồi
  ↓ UART Serial1 trên ESP8266
ESP8266: receiveFromESP32() đọc dữ liệu
  ↓ sendWiFi() gửi qua UDP
Qt: onUdpReadyRead() nhận dữ liệu
  ↓ Cập nhật GUI
```

## ⚠️ Debug

### Kiểm Tra Kết Nối WiFi
```cpp
// Serial Monitor:
Setup Wi-Fi
Initializing flix
ESP32 Relay UART initialized
```

### Kiểm Tra Gửi Lệnh
```
Qt Debug:
>> UDP gửi tới 192.168.4.1:14550 - arm

ESP8266 Serial Monitor:
>> Relay tới ESP32: arm
```

## 📝 Các Lệnh Hỗ Trợ

| Lệnh | Mô Tả |
|------|-------|
| `arm` | Arm drone (chuẩn bị bay) |
| `disarm` | Disarm drone |
| `mtr 4 <speed>` | Điều khiển motor |
| `dscnl 1 <0/1>` | Bật/tắt roll |
| `dscnl 2 <0/1>` | Bật/tắt pitch |
| `p <name> <value>` | Đặt tham số |

## 🔍 Troubleshooting

### ❌ Qt không kết nối được
- Kiểm tra WiFi SSID/Password
- Ping 192.168.4.1 từ máy tính
- Check firewall UDP port 14550

### ❌ ESP8266 không relay được
- Kiểm tra kết nối UART với ESP32
- Xác nhận pin RX/TX đúng
- Check Serial baud rate

### ❌ ESP32 không nhận lệnh
- Verify UART Serial0 baud rate = 115200
- Check kết nối GND chung
- Trace UART signal bằng oscilloscope

## 📚 Tài Liệu Tham Khảo

- [Flix Repository](https://github.com/okalachev/flix)
- ESP8266 Datasheet
- ESP32 Datasheet
