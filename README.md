# ESP32-S3 WiFi CSI Vital Signs Radar
## 7" Waveshare Touch LCD Display Project

### 📡 Features
- **Non-contact vital signs monitoring** using WiFi CSI technology
- Real-time heart rate detection (40-180 BPM)
- Breathing rate monitoring (8-30 RPM)
- Interactive radar sweep visualization
- Touch-enabled control panel
- Adjustable sensitivity (10-100%)
- Multiple detection modes (Normal/Precision/Long Range)

### 🎯 Touch Controls
- **Sensitivity Slider**: Adjust detection sensitivity
- **Mode Selection**: Normal, High Precision, Long Range
- **Noise Filter**: Toggle noise filtering
- **Calibrate Button**: Auto-calibrate baseline
- **Reset Base Button**: Reset baseline measurements

### 📊 Display Layout
- **Left**: Animated radar display with sweep and detection points
- **Center**: Heart rate & breathing rate monitors with graphs
- **Right**: Touch control panel
- **Bottom**: Status bar with signal strength

### 🔧 Hardware Requirements
- ESP32-S3 with 16MB Flash, 8MB PSRAM
- Waveshare 7" Touch LCD (800x480)
- GT911 Touch Controller

### 💻 Software Requirements
- Arduino IDE 2.x
- ESP32 Board Package 3.x
- Required Libraries:
  - ESP32_Display_Panel (v1.0.3+)
  - LVGL (v8.4.0)
  - ESP32_IO_Expander

### 🔨 Build Instructions

1. **Arduino IDE Settings**:
```
Board: ESP32-S3 Dev Module
Partition Scheme: Huge APP (3MB)
PSRAM: OPI PSRAM
Flash Size: 16MB
```

2. **Compile & Upload**:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=huge_app,PSRAM=opi ESP32_CarDashboard.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3:PartitionScheme=huge_app,PSRAM=opi ESP32_CarDashboard.ino
```

### 🚀 Future Plans
- USB Serial Control for external devices
- Integration with Microchip VelocityDRIVE
- Enhanced motion detection algorithms
- Data logging and export features
- BLE connectivity for mobile app

### 📄 License
MIT License

### 👤 Author
hwkim3330
