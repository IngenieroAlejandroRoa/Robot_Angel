# ESP32/Arduino Upload System

## Quick Start

### Upload Code to ESP32/Arduino

1. **Connect board** via USB
2. **Open IDE** and write Arduino code
3. **Select board** from dropdown (auto-detected)
4. **Click Upload** button
5. **Monitor progress** in terminal

### Test Code
```cpp
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
```

## Features

- ✅ Auto-detect ESP32/Arduino boards
- ✅ Board selector dropdown (refreshes every 5s)
- ✅ One-click upload to board
- ✅ Compile verification
- ✅ Progress shown in terminal
- ✅ Supports ESP32, ESP32-S2, ESP32-S3, ESP32-C3, Arduino Uno/Mega/Nano

## Components Created

### Python Backend
- `robot_angel/utils/board_manager.py` - Board detection
- `robot_angel/utils/arduino_uploader.py` - Compilation and upload

### TypeScript Backend
- `src/node/board-manager-backend.ts` - RPC service

### Frontend
- `src/browser/board-manager-service.ts` - Service layer
- `src/components/TopToolbar.tsx` - UI with board selector
- `src/App.tsx` - Upload handler

## Troubleshooting

**Board not detected?**
- Check USB cable (must support data)
- Install board drivers
- Add user to dialout group: `sudo usermod -a -G dialout $USER`

**ESP32 core not installed?**
```bash
~/tools/bin/arduino-cli core install esp32:esp32
```

**Permission denied?**
```bash
sudo chmod a+rw /dev/ttyUSB0  # or your port
```
