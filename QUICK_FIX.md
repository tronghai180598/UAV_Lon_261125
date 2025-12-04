# ✅ QUICK FIX SUMMARY: Integrator Reset

## 🎯 Vấn Đề
- ❌ UAV quay nhanh khi arm
- ❌ UAV vọt lố khi bật/tắt kênh (dscnl)
- **Nguyên nhân:** Integrator không reset

## ✅ 3 Nơi Đã Sửa

### 1️⃣ **cli.ino** (Line ~123)
```cpp
} else if (command == "arm") {
    armed = true;
    pdpiRoll.reset();      // ← THÊM DÒNG NÀY
    pdpiPitch.reset();     // ← THÊM DÒNG NÀY
} else if (command == "disarm") {
    armed = false;
    pdpiRoll.reset();      // ← THÊM DÒNG NÀY
    pdpiPitch.reset();     // ← THÊM DÒNG NÀY
}
```

### 2️⃣ **mavlink.ino** (Line ~246)
```cpp
if (m.command == MAV_CMD_COMPONENT_ARM_DISARM) {
    if (m.param1 && controlThrottle > 0.05) return;
    accepted = true;
    armed = m.param1 == 1;
    if (armed) {
        pdpiRoll.reset();   // ← THÊM DÒNG NÀY
        pdpiPitch.reset();  // ← THÊM DÒNG NÀY
    } else {
        pdpiRoll.reset();   // ← THÊM DÒNG NÀY
        pdpiPitch.reset();  // ← THÊM DÒNG NÀY
    }
}
```

### 3️⃣ **control.ino** (Line ~118) ⭐ MỚI
```cpp
void DisableCnl(int cnl, int val){	
    if(cnl == NoRoll) {
        bnRll = val;
        if (!val) pdpiRoll.reset();    // ← RESET khi bật roll
    }
    else if(cnl == NoPitch) {
        bnPtch = val;
        if (!val) pdpiPitch.reset();   // ← RESET khi bật pitch
    }
    else if(cnl == NoYaw) bnYaw = val;
    else {
        bnRll = bnPtch = bnYaw = val;
        if (!val) {
            pdpiRoll.reset();          // ← RESET tất cả
            pdpiPitch.reset();
        }
    }
}
```

## 🧪 Test
```
> arm
> dscnl 1 0  ← Nên mượt, không vọt lố
> dscnl 2 0  ← Nên mượt, không vọt lố
```

## 📝 Giải Thích val
- `val = 0` → Enable (bật) kênh → **RESET**
- `val = 1` → Disable (tắt) kênh → không reset (kênh không dùng)
