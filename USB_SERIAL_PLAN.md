# USB Serial Control Integration Plan
## ESP32-S3 ↔ Microchip VelocityDRIVE CT

### Phase 1: Basic Serial Communication (Week 1)
- **USB Host Mode Setup**
  - Configure ESP32-S3 USB OTG as host
  - Implement CDC-ACM driver for serial devices
  - Test with `/dev/ttyACM0` emulation

- **Serial Protocol Bridge**
  ```cpp
  // ESP32 as USB-Serial bridge
  Serial1.begin(115200);  // To external device
  USBSerial.begin(115200); // From ESP32
  ```

### Phase 2: MUP1 Protocol Implementation (Week 2)
- **Microchip UART Protocol #1**
  - Frame format: `>TYPE[DATA]<[<]CHECKSUM`
  - Commands: GET/SET/NOTIFICATION
  - Response parsing and error handling

- **Integration Points**
  ```cpp
  class MUP1Protocol {
    void sendCommand(String cmd);
    String receiveResponse();
    bool validateChecksum(String frame);
  };
  ```

### Phase 3: VelocityDRIVE Integration (Week 3)
- **mvdct CLI Commands**
  - Port `/mvdct device` commands to ESP32
  - YANG model queries via serial
  - Real-time status monitoring

- **Display Integration**
  - Show TSN metrics on LCD
  - Touch controls for VelocityDRIVE settings
  - Queue status visualization

### Phase 4: Advanced Features (Week 4)
- **CoAP/CORECONF Protocol**
  - CBOR encoding/decoding
  - SID-based addressing
  - YANG browser on touch display

- **TSN Performance Monitoring**
  - CBS/TAS configuration via touch
  - Real-time latency graphs
  - Packet loss indicators

### Hardware Requirements
- USB OTG cable (Type-C to Type-A)
- FTDI/CP2102 USB-Serial adapter (testing)
- LAN9662 board connection

### Software Components
```
ESP32_CarDashboard/
├── usb_serial/
│   ├── USBHost.cpp         # USB host implementation
│   ├── MUP1Protocol.cpp    # Protocol handler
│   ├── VelocityDrive.cpp   # Device control
│   └── TSNMonitor.cpp      # Performance metrics
```

### Testing Approach
1. **Loopback Test**: ESP32 ↔ ESP32
2. **FTDI Test**: ESP32 → USB-Serial adapter
3. **VelocityDRIVE**: ESP32 → LAN9662 board
4. **Full Integration**: Touch UI + Serial control

### Example Implementation
```cpp
// USB Serial Control Handler
void handleUSBSerial() {
  if (USBSerial.available()) {
    String cmd = USBSerial.readStringUntil('\n');

    if (cmd.startsWith("GET /")) {
      String response = velocityDrive.get(cmd);
      updateDisplay(response);
    }
    else if (cmd.startsWith("SET /")) {
      velocityDrive.set(cmd);
      showNotification("Config updated");
    }
  }
}

// Touch UI Integration
static void device_control_btn_cb(lv_event_t *e) {
  String cmd = ">GET[/ietf-system:system-state]<";
  USBSerial.println(cmd);

  // Update display with response
  String resp = waitForResponse(1000);
  lv_label_set_text(status_label, resp.c_str());
}
```

### Development Timeline
- **Week 1**: USB host mode, basic serial I/O
- **Week 2**: MUP1 protocol, command parsing
- **Week 3**: VelocityDRIVE commands, display integration
- **Week 4**: Advanced protocols, performance monitoring

### Resources
- ESP32-S3 USB OTG documentation
- Microchip VelocityDRIVE CT-CLI source
- MUP1 protocol specification
- YANG models for LAN9662

### Next Steps
1. Install ESP32 USB Host library
2. Create test serial device
3. Implement MUP1 frame parser
4. Design touch UI for device control
5. Integrate with existing WiFi CSI display