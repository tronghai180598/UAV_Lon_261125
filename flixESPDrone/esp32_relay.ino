// ESP32 Relay - Chuyển tiếp lệnh từ Qt (qua ESP8266 WiFi) tới ESP32 qua UART

// UART0 (Serial0): Liên lạc với giao tiếp nạp code / Debug
// UART1 (Serial1): Kết nối với ESP32
// RX1 = GPIO9, TX1 = GPIO10 (tuỳ chỉnh theo board của bạn)

#define ESP32_UART_BAUDRATE 115200
#define ESP32_RX_PIN 9    // ESP8266 RX1
#define ESP32_TX_PIN 10   // ESP8266 TX1

// Khởi tạo giao tiếp UART với ESP32
void setupESP32Relay() {
    Serial1.begin(ESP32_UART_BAUDRATE, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);
    print("ESP32 Relay UART initialized\n");
}

// Gửi lệnh từ Qt qua UART tới ESP32
void sendCommandToESP32(const uint8_t *buf, int len) {
    if (Serial1) {
        Serial1.write(buf, len);
        print(">> Relay tới ESP32: ");
        for (int i = 0; i < len; i++) {
            if (buf[i] >= 32 && buf[i] <= 126) {
                Serial.write(buf[i]);
            }
        }
        print("\n");
    }
}

// Nhận dữ liệu từ ESP32 (nếu cần)
void receiveFromESP32() {
    while (Serial1.available()) {
        uint8_t data = Serial1.read();
        // Có thể chuyển tiếp lại cho Qt hoặc xử lý tại đây
        Serial.write(data);
    }
}
