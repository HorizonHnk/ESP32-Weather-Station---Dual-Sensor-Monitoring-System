# 🌡️ ESP32 Weather Station - Dual Sensor Monitoring System

[![Arduino](https://img.shields.io/badge/Arduino-Compatible-brightgreen.svg)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/ESP32-Supported-blue.svg)](https://espressif.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![YouTube](https://img.shields.io/badge/YouTube-Tutorial%20Series-red.svg)](https://www.youtube.com/playlist?list=PLrZbkNpNVSwwBlIkETVtyEscY6WLFQlGf)
[![Contributors](https://img.shields.io/github/contributors/yourusername/esp32-weather-station.svg)](https://github.com/yourusername/esp32-weather-station/graphs/contributors)

> **Advanced ESP32-based weather monitoring system with dual temperature sensors, continuous WiFi operation, real-time web dashboard, and mobile-responsive interface. Perfect for 24/7 environmental monitoring applications.**

## 📺 Video Tutorial Series

🎬 **Complete step-by-step tutorials available on YouTube:**
[ESP32 Weather Station Playlist](https://www.youtube.com/playlist?list=PLrZbkNpNVSwwBlIkETVtyEscY6WLFQlGf)

---

## 📋 Table of Contents

- [✨ Features](#-features)
- [🔧 Hardware Requirements](#-hardware-requirements)
- [📐 Circuit Diagram](#-circuit-diagram)
- [💻 Software Setup](#-software-setup)
- [🚀 Quick Start Guide](#-quick-start-guide)
- [⚙️ Configuration](#️-configuration)
- [🌐 Web Dashboard](#-web-dashboard)
- [🔧 Troubleshooting](#-troubleshooting)
- [📊 API Documentation](#-api-documentation)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)

---

## ✨ Features

### 🌡️ **Dual Temperature Monitoring**
- **DHT22 Digital Sensor**: High-precision temperature and humidity measurement
- **TMP36 Analog Sensor**: Wide-range temperature monitoring (-40°C to +125°C)
- **Auto-calibration**: Intelligent voltage reference calibration for TMP36
- **Error Handling**: Robust sensor failure detection and recovery

### 📡 **Continuous WiFi Operation**
- **24/7 Hotspot**: No automatic shutdowns or timeouts
- **Captive Portal**: Automatic redirection to dashboard
- **Power Optimized**: Disabled sleep modes for maximum stability
- **Watchdog Protection**: Automatic crash recovery system

### 🌐 **Real-time Web Dashboard**
- **Mobile Responsive**: Optimized for all screen sizes
- **Live Updates**: WebSocket-based real-time data streaming
- **Interactive Charts**: Dynamic temperature trend visualization
- **Touch Controls**: Intuitive slider-based fan control

### 🔧 **Smart Control System**
- **Manual Fan Control**: 0-100% speed control via web interface
- **Automatic Alerts**: Temperature threshold-based activation
- **Buzzer Notifications**: Audio alerts for temperature events
- **LCD Display**: Local status monitoring with rotating screens

### 📊 **Data Management**
- **Historical Logging**: 50-point rolling data history
- **CSV Export**: Downloadable temperature data
- **API Endpoints**: RESTful API for external integration
- **Real-time Broadcasting**: Multi-client WebSocket support

---

## 🔧 Hardware Requirements

### 📱 **Core Components**

| Component | Model/Type | Quantity | Purpose |
|-----------|------------|----------|---------|
| **Microcontroller** | ESP32 Development Board | 1 | Main processing unit |
| **Digital Sensor** | DHT22 (AM2302) | 1 | Temperature & humidity |
| **Analog Sensor** | TMP36 | 1 | Analog temperature reading |
| **Display** | 16x2 I2C LCD (Optional) | 1 | Local status display |
| **Fan** | 5V DC Motor Fan | 1 | Cooling system |
| **Buzzer** | Active Buzzer | 1 | Audio alerts |
| **Transistor** | TIP122 NPN Darlington | 1 | Fan speed control |
| **Resistor** | 10kΩ | 1 | Pull-up resistor |
| **Resistor** | 3.3kΩ | 1 | Base current limiting |
| **Diode** | 1N4007 | 1 | Flyback protection |

### 🔌 **Power Requirements**
- **ESP32**: 3.3V (USB powered)
- **Fan**: 5V DC (separate power supply recommended)
- **Total Current**: ~500mA (with fan at full speed)

### 📦 **Optional Components**
- **Enclosure**: Weather-resistant case for outdoor use
- **SD Card Module**: For extended data logging
- **External Antenna**: For improved WiFi range

---

## 📐 Circuit Diagram

### 🔗 **Component Connections**

#### **DHT22 Digital Sensor:**
```
DHT22    →    ESP32
VCC      →    3.3V
GND      →    GND
DATA     →    Digital GPIO Pin
```

#### **TMP36 Analog Sensor:**
```
TMP36    →    ESP32
VCC      →    3.3V
GND      →    GND
VOUT     →    ADC Pin
```

#### **Fan Control Circuit:**
```
ESP32 PWM Pin ──[3.3kΩ]── TIP122 Base
                          TIP122 Emitter ── GND
                          TIP122 Collector ── Fan(-) & Diode Cathode
5V Supply ──── Fan(+)
5V Supply ──── Diode Anode (1N4007)
```

#### **LCD Display (Optional):**
```
LCD I2C  →    ESP32
VCC      →    5V (or 3.3V)
GND      →    GND
SDA      →    I2C Data Pin
SCL      →    I2C Clock Pin
```

#### **Additional Components:**
```
Active Buzzer  →  Digital GPIO Pin + GND
Status LED     →  Built-in LED or External LED
```

---

## 💻 Software Setup

### 📋 **Prerequisites**

1. **Arduino IDE** (version 1.8.19 or newer)
2. **ESP32 Board Package** 
3. **Required Libraries** (auto-installed via Arduino Library Manager)

### 📚 **Required Libraries**

```cpp
// Core ESP32 libraries (included with board package)
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>

// External libraries (install via Library Manager)
#include <WebSocketsServer.h>      // by Markus Sattler
#include <DHT.h>                   // by Adafruit
#include <LiquidCrystal_I2C.h>     // by Frank de Brabander (optional)
#include <DNSServer.h>             // ESP32 core library
```

### ⚙️ **Arduino IDE Setup**

1. **Install ESP32 Board Package:**
   ```
   File → Preferences → Additional Board Manager URLs:
   https://dl.espressif.com/dl/package_esp32_index.json
   ```

2. **Install Required Libraries:**
   ```
   Tools → Manage Libraries → Search and install:
   - WebSockets by Markus Sattler
   - DHT sensor library by Adafruit
   - LiquidCrystal I2C by Frank de Brabander
   - ArduinoJson by Benoit Blanchon
   ```

3. **Board Configuration:**
   ```
   Tools → Board → ESP32 Arduino → ESP32 Dev Module
   Tools → Upload Speed → 921600
   Tools → CPU Frequency → 240MHz (WiFi/BT)
   Tools → Flash Size → 4MB
   Tools → Partition Scheme → Default 4MB with spiffs
   ```

---

## 🚀 Quick Start Guide

### 1️⃣ **Hardware Assembly**

> ⚠️ **Safety First**: Always disconnect power when making connections

```bash
# Assembly Checklist
□ Connect DHT22 to digital GPIO pin with 3.3V and GND
□ Connect TMP36 to ADC pin with 3.3V and GND  
□ Wire fan control circuit with TIP122 transistor
□ Connect buzzer to digital GPIO pin
□ Optional: Connect I2C LCD to SDA/SCL pins
□ Double-check all connections before powering on
```

### 2️⃣ **Software Upload**

```bash
# Step-by-step upload process
1. Clone this repository
   git clone https://github.com/yourusername/esp32-weather-station.git

2. Open esp32_weather_station.ino in Arduino IDE

3. Configure your settings (see Configuration section)

4. Select correct board and port
   Tools → Board → ESP32 Dev Module
   Tools → Port → [Your ESP32 COM port]

5. Upload the code
   Sketch → Upload (Ctrl+U)
```

### 3️⃣ **First Boot**

```bash
# Monitor startup sequence
1. Open Serial Monitor (115200 baud)
2. Watch for successful WiFi hotspot creation
3. Look for "SYSTEM READY FOR CONNECTIONS" message
4. Note the IP address (default: 192.168.4.1)
```

### 4️⃣ **Connect to Dashboard**

```bash
# Connection steps
1. Connect device to WiFi network: "ESP32_Weather_Station"
2. Password: "weather123"
3. Browser should auto-redirect to dashboard
4. Manual access: http://192.168.4.1
```

---

## ⚙️ Configuration

### 🔧 **Basic Settings**

```cpp
// WiFi Hotspot Configuration
const char* AP_SSID     = "ESP32_Weather_Station";
const char* AP_PASSWORD = "weather123";              // Min 8 characters
const int   AP_CHANNEL  = 6;                        // WiFi channel (1-13)

// Temperature Calibration
float actualVRef = 3.3;                             // ESP32 voltage reference
float tmp36CalibrationOffset = 0.0;                 // Temperature offset

// Hardware Features (set to false if not using)
#define USE_LCD true                                 // Enable LCD display
#define USE_BUZZER true                             // Enable buzzer alerts
#define USE_FAN_CONTROL true                        // Enable fan control
```

### 🎯 **Pin Configuration**

```cpp
// Sensor Pins
#define DHT_PIN      [DIGITAL_GPIO]     // DHT22 data pin
#define TMP36_PIN    [ADC_PIN]          // TMP36 analog input

// Control Pins  
#define BUZZER_PIN   [DIGITAL_GPIO]     // Active buzzer
#define FAN_PIN      [PWM_GPIO]         // PWM fan control
#define LED_PIN      [LED_GPIO]         // Built-in LED (status indicator)

// I2C Pins (for LCD)
#define SDA_PIN      [I2C_SDA]          // I2C Data
#define SCL_PIN      [I2C_SCL]          // I2C Clock
```

### 📊 **Advanced Configuration**

```cpp
// Timing Intervals (milliseconds)
const int SENSOR_READ_INTERVAL   = 5000;   // Sensor reading frequency
const int LCD_UPDATE_INTERVAL    = 2000;   // LCD refresh rate
const int DATA_BROADCAST_INTERVAL = 1000;  // WebSocket update rate

// DHT22 Error Handling
const int DHT_MAX_CONSECUTIVE_ERRORS = 10;  // Before marking inactive
const int DHT_MAX_RETRY_ATTEMPTS     = 3;   // Retry attempts per reading

// Fan Control
const int FAN_PWM_FREQ       = 5000;  // PWM frequency (Hz)
const int FAN_PWM_RESOLUTION = 8;     // 8-bit resolution (0-255)
```

---

## 🌐 Web Dashboard

### 🎨 **Dashboard Features**

#### **Real-time Monitoring**
- 📊 **Dual Temperature Gauges**: Color-coded temperature displays
- 📈 **Live Charts**: Real-time temperature trends comparison
- 📱 **Mobile Optimized**: Responsive design for all devices
- 🔄 **Auto-refresh**: WebSocket-based live updates

#### **Control Interface**
- 🌀 **Fan Speed Slider**: 0-100% manual control
- 🎯 **Threshold Setting**: Customizable temperature alerts
- 🔀 **Sensor Selection**: Choose primary sensor for alerts
- 📊 **Data Export**: Download historical data as CSV

#### **Status Monitoring**
- 📶 **Connection Status**: Real-time client count
- ⏱️ **System Uptime**: Continuous operation tracking
- ⚠️ **Alert Status**: Visual and audio alert indicators
- 🔧 **Sensor Health**: DHT22 error monitoring

### 🌍 **Network Information**

```bash
# Default Network Settings
SSID: ESP32_Weather_Station
Password: weather123
IP Address: 192.168.4.1
WebSocket Port: 81
DNS Server: Captive portal enabled

# Access URLs
Main Dashboard: http://192.168.4.1/dashboard
API Endpoint: http://192.168.4.1/api/data
Data Export: http://192.168.4.1/api/export
```

---

## 🔧 Troubleshooting

### ❌ **Common Issues & Solutions**

#### **1. WiFi Connection Problems**

**Symptoms:**
- Can't see WiFi network
- Connection drops frequently
- Slow response times

**Solutions:**
```cpp
// Check these settings in code:
1. Verify AP_SSID and AP_PASSWORD are correct
2. Try different WiFi channel (1, 6, or 11 recommended)
3. Check power supply stability (min 1A capacity)
4. Ensure WiFi power saving is disabled:
   esp_wifi_set_ps(WIFI_PS_NONE);
```

#### **2. Sensor Reading Errors**

**DHT22 Issues:**
```bash
Error: "DHT22 reading failed - NaN values"
Solutions:
□ Check wiring: VCC→3.3V, GND→GND, DATA→Digital GPIO
□ Verify 3.3V power supply stability
□ Try different GPIO pin if interference suspected
□ Add pull-up resistor (4.7kΩ) to data line
□ Increase delay between readings in code
```

**TMP36 Issues:**
```bash
Error: "TMP36 temperature out of range"
Solutions:
□ Verify wiring: VCC→3.3V, GND→GND, VOUT→ADC Pin
□ Check ADC voltage reference calibration
□ Adjust actualVRef value (try 3.0V, 3.15V, or 3.3V)
□ Fine-tune with tmp36CalibrationOffset parameter
□ Ensure good electrical connections
```

#### **3. Fan Control Problems**

**Symptoms:**
- Fan doesn't respond to speed changes
- Buzzer not working
- Erratic fan behavior

**Solutions:**
```cpp
// Check circuit connections:
1. Verify TIP122 transistor wiring
2. Ensure 1N4007 diode is correctly oriented
3. Check 3.3kΩ base resistor connection
4. Verify 5V power supply for fan
5. Test with multimeter: Base voltage should be 0-3.3V
```

#### **4. Memory Issues**

**Symptoms:**
- System crashes or resets
- Slow response times
- WebSocket disconnections

**Solutions:**
```cpp
// Monitor memory usage:
Serial.println("Free heap: " + String(ESP.getFreeHeap()));

// Optimization strategies:
1. Reduce DATA_BROADCAST_INTERVAL if needed
2. Limit maximum WebSocket clients
3. Clear unused variables in large functions
4. Use PROGMEM for large string constants
```

#### **5. LCD Display Issues**

**Symptoms:**
- Blank LCD screen
- Garbled text
- No backlight

**Solutions:**
```bash
Common fixes:
□ Check I2C address (try 0x27, 0x3F, or use I2C scanner)
□ Verify SDA/SCL connections to I2C pins
□ Ensure 5V power supply to LCD
□ Check contrast adjustment potentiometer
□ Try different I2C pull-up resistors (4.7kΩ)
□ Set USE_LCD to false if not using display
```

### 🔍 **Diagnostic Tools**

#### **Serial Monitor Debug Output**
```cpp
// Enable detailed debugging:
1. Set Serial Monitor to 115200 baud
2. Watch for startup sequence messages
3. Monitor sensor readings and error counts
4. Check WiFi status updates
5. Observe WebSocket connection logs
```

#### **I2C Scanner Code**
```cpp
// Use this code to find LCD I2C address:
#include <Wire.h>
void setup() {
  Wire.begin();  // Use default SDA, SCL pins
  Serial.begin(115200);
  Serial.println("I2C Scanner");
}
void loop() {
  for(byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0) {
      Serial.printf("Found I2C device at 0x%02X\n", i);
    }
  }
  delay(5000);
}
```

---

## 📊 API Documentation

### 🔌 **REST API Endpoints**

#### **GET /api/data**
Returns current sensor readings and system status.

**Response:**
```json
{
  "digitalTemp": 25.6,
  "analogTemp": 25.8,
  "threshold": 28.0,
  "fanSpeed": 75,
  "alertActive": false,
  "selectedSensor": "digital",
  "dhtActive": true,
  "timestamp": 1640995200000,
  "connectedClients": 2,
  "uptime": 86400,
  "continuousMode": true,
  "sensorType": "TMP36"
}
```

#### **GET /api/history**
Returns historical temperature data (last 50 readings).

**Response:**
```json
{
  "history": [
    {
      "timestamp": 1640995200000,
      "digitalTemp": 25.6,
      "analogTemp": 25.8,
      "dhtActive": true
    },
    // ... more data points
  ]
}
```

#### **GET /api/status**
Returns detailed system status information.

**Response:**
```json
{
  "wifiMode": "Access Point (Continuous)",
  "ssid": "ESP32_Weather_Station",
  "ipAddress": "192.168.4.1",
  "connectedClients": 2,
  "uptime": 86400,
  "freeHeap": 234567,
  "dhtErrors": 5,
  "dhtActive": true,
  "continuousMode": true,
  "powerSaving": false,
  "wifiChannel": 6,
  "sensorType": "TMP36"
}
```

#### **POST /api/fan**
Controls fan speed.

**Parameters:**
- `speed` (integer): Fan speed percentage (0-100)

**Example:**
```bash
curl -X POST -d "speed=75" http://192.168.4.1/api/fan
```

**Response:**
```json
{
  "status": "ok",
  "speed": 75
}
```

#### **POST /api/threshold**
Sets temperature threshold for automatic alerts.

**Parameters:**
- `threshold` (float): Temperature threshold in Celsius

**Example:**
```bash
curl -X POST -d "threshold=28.5" http://192.168.4.1/api/threshold
```

#### **POST /api/sensor**
Selects primary sensor for threshold monitoring.

**Parameters:**
- `sensor` (string): "digital" for DHT22 or "analog" for TMP36

**Example:**
```bash
curl -X POST -d "sensor=analog" http://192.168.4.1/api/sensor
```

#### **GET /api/export**
Downloads historical data as CSV file.

**Response:**
```csv
timestamp,digitalTemp(DHT22),analogTemp(TMP36),dhtActive
1640995200000,25.60,25.80,true
1640995205000,25.65,25.82,true
...
```

### 🔌 **WebSocket API**

**Connection:** `ws://192.168.4.1:81`

**Real-time Data Message:**
```json
{
  "type": "sensorData",
  "digitalTemp": 25.6,
  "analogTemp": 25.8,
  "threshold": 28.0,
  "fanSpeed": 75,
  "alertActive": false,
  "selectedSensor": "digital",
  "dhtActive": true,
  "timestamp": 1640995200000,
  "connectedClients": 2,
  "uptime": 86400,
  "continuousMode": true,
  "sensorType": "TMP36"
}
```

---

## 🤝 Contributing

We welcome contributions from the community! Here's how you can help:

### 🐛 **Bug Reports**

When reporting bugs, please include:
- ESP32 board model and revision
- Arduino IDE version
- Library versions
- Complete error messages
- Serial monitor output
- Steps to reproduce

### 💡 **Feature Requests**

Before submitting feature requests:
- Check existing issues for duplicates
- Clearly describe the desired functionality
- Explain the use case and benefits
- Consider implementation complexity

### 🔀 **Pull Requests**

**Development Workflow:**
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/new-sensor-support`
3. Make your changes with clear, commented code
4. Test thoroughly on hardware
5. Update documentation as needed
6. Submit pull request with detailed description

**Code Standards:**
```cpp
// Follow these conventions:
- Use descriptive variable names
- Comment complex logic sections
- Maintain consistent indentation (2 spaces)
- Include error handling for new features
- Update README for new configuration options
```

### 📋 **Development Setup**

```bash
# Clone the repository
git clone https://github.com/yourusername/esp32-weather-station.git
cd esp32-weather-station

# Create development branch
git checkout -b feature/your-feature-name

# Make changes and test
# Update documentation

# Commit and push
git add .
git commit -m "Add: New sensor support with calibration"
git push origin feature/your-feature-name
```

---

## 📸 Project Gallery

### 🖼️ **Hardware Photos**

> 📷 Add photos of your completed project to help others with assembly

- [ ] Breadboard assembly
- [ ] Final enclosure
- [ ] LCD display in action
- [ ] Web dashboard screenshots
- [ ] Mobile interface views

### 🎬 **Video Demonstrations**

📺 **Watch the complete tutorial series:**
[ESP32 Weather Station Playlist](https://www.youtube.com/playlist?list=PLrZbkNpNVSwwBlIkETVtyEscY6WLFQlGf)

- Setup and assembly guide
- Code walkthrough and explanation
- Troubleshooting common issues
- Advanced customization tips

---

## 🏆 Acknowledgments

### 👨‍💻 **Libraries Used**
- **WebSockets** by Markus Sattler
- **DHT Sensor Library** by Adafruit
- **ArduinoJson** by Benoit Blanchon
- **LiquidCrystal I2C** by Frank de Brabander

### 🙏 **Special Thanks**
- ESP32 community for continuous support
- Arduino IDE team for excellent development environment
- All contributors and testers who helped improve this project

### 📚 **Inspiration**
This project was inspired by the need for reliable, continuous weather monitoring with dual sensor redundancy and mobile-friendly interfaces.

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2024 ESP32 Weather Station Project

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject so the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 📞 Support

### 💬 **Get Help**
- **GitHub Issues**: [Report bugs or request features](https://github.com/horizonhnk/esp32-weather-station/issues)
- **YouTube Comments**: Ask questions on tutorial videos
- **Arduino Forum**: Community support for hardware issues
- **ESP32 Discord**: Real-time chat with other developers

### 📊 **Project Stats**
![GitHub stars](https://img.shields.io/github/stars/horizonhnk/esp32-weather-station.svg?style=social)
![GitHub forks](https://img.shields.io/github/forks/horizonhnk/esp32-weather-station.svg?style=social)
![GitHub watchers](https://img.shields.io/github/watchers/horizonhnk/esp32-weather-station.svg?style=social)

---

<div align="center">

### 🌟 **Star this repository if it helped you!** 🌟

**Made with ❤️ by the ESP32 Community**

[⬆ Back to Top](#-esp32-weather-station---dual-sensor-monitoring-system)

</div>
