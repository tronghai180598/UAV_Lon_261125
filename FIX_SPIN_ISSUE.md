# 🚨 CẢNH BÁO: Vấn Đề Quay Nhanh và Cách Sửa

## ❌ Vấn Đề Phát Hiện

**Triệu chứng:** 
1. Sau khi arm, khi điều khiển roll/pitch → UAV quay nhanh tới cực đại
2. ⭐ **MỚI:** Khi bật/tắt kênh điều khiển (dscnl) → UAV vọt lố

## 🔍 Nguyên Nhân

**Bộ tích phân (Integrator) không được reset!**

Có **3 trường hợp** cần reset:
1. ❌ Khi arm/disarm
2. ❌ Khi nhận lệnh arm/disarm qua MAVLink (Qt)
3. ⭐ **MỚI** ❌ Khi bật/tắt kênh điều khiển (dscnl)

### Giải Thích Kỹ Thuật:
```
Khi arm:
- Biến "armed = true" được set
- ❌ NHƯNG các bộ điều khiển PID/Integrator KHÔNG được reset
- Chúng vẫn giữ giá trị từ lần bay trước (có thể từ -1000 đến +1000)
- Khi bạn yêu cầu tilt, tích phân phải CLEAR trước khi bắt đầu điều khiển mới
- Nếu không clear, nó cộng thêm vào, gây overshoot/out of control

Sơ đồ bộ điều khiển (KrenCtrl):
┌─────────────────────────────────────┐
│ Set Point (yêu cầu tilt từ bạn)     │
└────────────┬────────────────────────┘
             │
             ├──→ [Kpf] (Proportional)
             │
             ├──→ [Kdv] (Derivative)  
             │
             ├──→ [Kpv] (Rate PID)
             │
             ├──→ [Te Integrator] ← ⚠️ CẦN RESET
             │
             └──→ [Ti Integrator] ← ⚠️ CẦN RESET
             
Output: Lệnh điều khiển tới motor
```

## ✅ Giải Pháp Đã Áp Dụng

### 1. **File: `cli.ino`** (Lệnh từ Serial Console)

**Đã sửa:**
```cpp
} else if (command == "arm") {
    armed = true;
    pdpiRoll.reset();    // ⚠️ RESET bộ điều khiển Roll
    pdpiPitch.reset();   // ⚠️ RESET bộ điều khiển Pitch
    print("Drone armed - Controllers reset\n");
} else if (command == "disarm") {
    armed = false;
    pdpiRoll.reset();    // ⚠️ RESET khi disarm
    pdpiPitch.reset();
    print("Drone disarmed - Controllers reset\n");
}
```

### 2. **File: `mavlink.ino`** (Lệnh từ MAVLink/Qt)

**Đã sửa:**
```cpp
if (m.command == MAV_CMD_COMPONENT_ARM_DISARM) {
    if (m.param1 && controlThrottle > 0.05) return;
    accepted = true;
    armed = m.param1 == 1;
    // ⚠️ RESET bộ điều khiển khi arm/disarm qua MAVLink
    if (armed) {
        pdpiRoll.reset();
        pdpiPitch.reset();
    } else {
        pdpiRoll.reset();
        pdpiPitch.reset();
    }
}
```

### 3. **File: `control.ino`** (Khi bật/tắt kênh điều khiển) ⭐ MỚI

**Đã sửa:**
```cpp
void DisableCnl(int cnl, int val){	
    if(cnl == NoRoll) {
        bnRll = val;
        if (!val) pdpiRoll.reset();  // ⚠️ RESET khi bật roll (val=0)
    }
    else if(cnl == NoPitch) {
        bnPtch = val;
        if (!val) pdpiPitch.reset();  // ⚠️ RESET khi bật pitch (val=0)
    }
    else if(cnl == NoYaw) bnYaw = val;
    else {
        bnRll = bnPtch = bnYaw = val;
        if (!val) {
            pdpiRoll.reset();   // ⚠️ RESET tất cả kênh
            pdpiPitch.reset();
        }
    }
    print("Channel %d set to %d (0=enable, 1=disable)\n", cnl, val);
}
```

**Giải thích:**
- `val=0` → bật kênh (enable) → RESET bộ điều khiển
- `val=1` → tắt kênh (disable) → không reset (kênh không hoạt động)

## 🧪 Test Sau Khi Sửa

### Bước 1: Upload code mới
```
1. Nạp code mới vào ESP8266/ESP32
2. Đảm bảo không có lỗi compile
```

### Bước 2: Thử arm/disarm từ Serial Console
```
> arm
Drone armed - Controllers reset

> disarm
Drone disarmed - Controllers reset
```

### Bước 3: Thử bật/tắt kênh điều khiển
```
> dscnl 1 0
Channel 1 set to 0 (0=enable, 1=disable)
>>> Controller RESET ✅

> dscnl 2 0
Channel 2 set to 0 (0=enable, 1=disable)
>>> Controller RESET ✅

✅ Khi bật thêm kênh, không nên vọt lố nữa
```

### Bước 4: Thử điều khiển roll/pitch
```
✅ Khi arm, vào STAB mode
✅ Chậm chậm điều khiển roll/pitch (+10% → +20% → ...)
✅ Quan sát phản ứng của UAV (nên mượt mà, không bất ngờ)
✅ Bật/tắt các kênh → không vọt lố
❌ Nếu vẫn quay nhanh → Vấn đề khác (xem Troubleshooting)
```

## ⚠️ Troubleshooting - Nếu Vẫn Quay Nhanh

### ❌ Vấn đề 1: Code không được nạp lên đúng
- Check Arduino Serial Monitor:
  ```
  > arm
  Drone armed - Controllers reset  ← Phải có dòng này
  ```

### ❌ Vấn đề 2: Lệnh arm đi từ Qt (UDP) chứ không phải Serial Console
- Kiểm tra trong `mavlink.ino` có được sửa chưa
- Qt gửi MAVLink lệnh → `m.command == MAV_CMD_COMPONENT_ARM_DISARM` → reset()

### ❌ Vấn đề 3: Hệ số Kp/Ki/Kd quá cao
- Nếu vẫn lắc rung sau khi reset, có thể **hệ số điều khiển quá cao**
- Giảm `Kpf` (gain cho góc):
  ```cpp
  // Trong KrenCtrl.cpp
  Kpf = mTf / (15 * mTmu); // Giảm từ 15 xuống 20-30
  ```

### ❌ Vấn đề 4: Sensor (IMU) bị lỗi
- Test IMU:
  ```
  > imu
  ```
- Kiểm tra gyro có drift không

## 📊 Kiểm Tra Reset() Hoạt Động

**Hàm reset() trong KrenCtrl.cpp:**
```cpp
void KrenCtrl::reset()
{
    Us = 0.0f;      // Clear proportional output
    Uv = 0.0f;      // Clear integrator (Te)
    Ui = 0.0f;      // Clear integrator (Ti) ← ⚠️ MỨC ĐỒ NGUY HIỂM
    Um = 0.0f;      // Clear total output
    Umm = 0.0f;     // Clear filtered output
    
    erVi1 = 0.0f;   // Clear previous error
    mdVi = 0.0f;    // Clear derivative
    muMd = 0.0f;    // Clear model output
    uMold = 0.0f;   // Clear old value
    oUi = 0.0f;     // Clear sliding mode
}
```

✅ Tất cả các tích phân được clear → An toàn bay tiếp!

## 🎯 Kết Luận

**Nguyên nhân:** Integrator không reset  
**Giải pháp:** Gọi `reset()` khi arm/disarm  
**Status:** ✅ ĐÃ SỬA  
**Nguy Hiểm:** ⚠️ **CẤP BÁCH** - Phải fix ngay trước khi bay lại!

---

**Sau khi sửa xong, hãy:**
1. ✅ Upload code mới
2. ✅ Test arm/disarm từ Serial
3. ✅ Test điều khiển roll/pitch từ từ
4. ✅ Quan sát phản ứng (nên bình thường)
5. ✅ Bay thử với propped removed nếu còn nghi ngờ
