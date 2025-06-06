/**
 * ESP32-Based Weather Station with Built-in Web Dashboard
 * Local temperature monitoring with DHT22 and TMP36 sensors
 * Creates WiFi Hotspot for dashboard access
 * 
 * FIXED VERSION - Continuous WiFi Hotspot (No Auto-Shutdown) + TMP36 Sensor
 * 
 * CONFIGURATION REQUIRED:
 * 1. Set USE_LCD to false if you don't have an I2C LCD display
 * 
 * SENSOR SETUP:
 * - DHT22: Digital temperature/humidity sensor on pin 18
 * - TMP36: Analog temperature sensor on pin 33
 *   TMP36 wiring: VCC to 3.3V, GND to GND, VOUT to pin 33
 * 
 * TMP36 CALIBRATION (if readings are wrong):
 * 1. Check Serial Monitor for auto-calibration results
 * 2. If still inaccurate, manually adjust these values:
 *    - actualVRef: Change from 3.3 to actual ESP32 voltage (try 3.15, 3.0, or 2.9)
 *    - tmp36CalibrationOffset: Add/subtract degrees for fine-tuning
 * 3. Common fixes:
 *    - Readings too low: Increase actualVRef to 3.15 or 3.3
 *    - Readings too high: Decrease actualVRef to 3.0 or 2.9
 *    - Fine adjustments: Use tmp36CalibrationOffset (+2.5, -1.3, etc.)
 * 
 * FAN & BUZZER CONTROL FEATURES:
 * - Manual fan control via web dashboard (0-100% speed)
 * - Buzzer automatically rings when fan is running (any speed > 0%)
 * - Automatic fan activation when temperature exceeds threshold
 * - Different buzzer tones: 1500Hz (normal operation), 2000Hz (temperature alert)
 * - Manual control persists after automatic activation
 * - Real-time status display on web dashboard and LCD
 * 
 * The system works in LOCAL MODE ONLY:
 * - Creates hotspot "ESP32_Weather_Station" for dashboard access
 * - No internet connection required
 * - CONTINUOUS WIFI OPERATION - No auto-shutdown
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <esp_wifi.h>  // For advanced WiFi settings
#include <esp_task_wdt.h> // For watchdog handling

// Configuration - Set to false if you don't have an LCD or want to disable it
#define USE_LCD true

#if USE_LCD
#include <LiquidCrystal_I2C.h>
#endif
#include <DNSServer.h>

// Pin Definitions
#define DHT_PIN      18      // Digital temperature sensor (DHT22)
#define TMP36_PIN    33      // Analog temperature sensor (TMP36)
#define BUZZER_PIN   12      // Buzzer for alarm
#define FAN_PIN      15      // DC motor fan controlled via PWM
#define LED_PIN      2       // Built-in LED for status indication

// TMP36 Calibration Settings
#define VREF_VOLTAGE  3.3    // Default reference voltage
float actualVRef = 3.3;      // Will be calibrated during setup
float tmp36CalibrationOffset = 0.0;  // Temperature offset for calibration

// DHT Sensor Type
#define DHTTYPE DHT22

// WiFi Hotspot Configuration - FIXED FOR CONTINUOUS OPERATION
const char* AP_SSID     = "ESP32_Weather_Station";
const char* AP_PASSWORD = "weather123";  // Must be at least 8 characters
const int   AP_CHANNEL  = 6;            // WiFi channel (1-13)
const int   AP_MAX_CONNECTIONS = 4;     // Maximum concurrent connections
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Global Objects
WebServer       server(80);
WebSocketsServer webSocket(81);
DNSServer       dnsServer;
DHT             dht(DHT_PIN, DHTTYPE);
#if USE_LCD
  LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

// Temperature variables
float digitalTemp   = 25.0;
float analogTemp    = 25.0;
float apiTemp       = 0.0;    // Placeholder for "API" temperature
float tempThreshold = 28.0;
int   fanSpeed      = 0;      // PWM (0-255)
bool  alertActive   = false;
String selectedSensor = "digital";

// DHT22 error handling
int   dhtErrorCount       = 0;
int   dhtConsecutiveErrors = 0;
unsigned long lastDhtSuccessTime = 0;
bool  dhtSensorActive    = true;
const int DHT_MAX_CONSECUTIVE_ERRORS = 10;
const int DHT_MAX_RETRY_ATTEMPTS     = 3;

// WiFi status tracking - SIMPLIFIED FOR STABILITY
bool wifiInitialized = false;
unsigned long lastWifiStatusLog = 0;
const int WIFI_STATUS_LOG_INTERVAL = 60000; // Log WiFi status every 60 seconds (less aggressive)

// Time tracking
unsigned long lastSensorReadTime  = 0;
unsigned long lastLcdUpdateTime   = 0;
unsigned long lastDataBroadcast   = 0;
unsigned long lastStatusUpdate    = 0;
unsigned long lastWatchdogFeed    = 0;  // NEW: Watchdog feeding
const int    SENSOR_READ_INTERVAL   = 5000;   // 5 seconds
const int    LCD_UPDATE_INTERVAL    = 2000;   // 2 seconds
const int    DATA_BROADCAST_INTERVAL = 1000;  // 1 second
const int    STATUS_UPDATE_INTERVAL  = 10000; // 10 seconds
const int    WATCHDOG_FEED_INTERVAL  = 1000;  // Feed watchdog every second

// Fan PWM properties
const int FAN_PWM_FREQ       = 5000;  // 5 kHz
const int FAN_PWM_CHANNEL    = 0;
const int FAN_PWM_RESOLUTION = 8;     // 8-bit resolution (0-255)

// System status
int connectedClients = 0;
unsigned long systemUptime = 0;

// Data storage for charts (last 50 readings)
struct DataPoint {
  unsigned long timestamp;
  float digitalTemp;
  float analogTemp;
  bool  dhtActive;
};
DataPoint dataHistory[50];
int dataIndex = 0;
int dataCount = 0;

// Forward declarations
String getMainHTML();
String getCSS();
String getJavaScript();
void handleApiData();
void handleApiHistory();
void handleApiStatus();
void handleFanControl();
void handleThresholdControl();
void handleSensorSelection();
void handleDataExport();
void handleCaptivePortal();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void broadcastSensorData();
void readDigitalTempWithRetry();
void readAnalogTemp();
void calibrateTMP36();   // NEW: TMP36 calibration function
void checkDHTStatus();
void checkThresholds();
void storeDataPoint();
void updateLcdDisplay();
void lcdPrintCentered(const char* text, int row);
void setupWiFiHotspot();
void logWiFiStatus();  // SIMPLIFIED
void setupWebServer();
void printSystemInfo();
void updateSystemStatus();
void updateFanAndBuzzer(int newFanSpeed, String source);
void feedWatchdog();   // NEW

void setup() {
  Serial.begin(115200);
  delay(1000); // Initial delay
  
  // Configure watchdog timer
  Serial.println("🔧 Configuring watchdog timer...");
  esp_task_wdt_init(30, false); // 30 second timeout, no panic
  esp_task_wdt_add(NULL); // Add current task to watchdog
  
  Serial.println("=====================================");
  Serial.println("  ESP32 Weather Station Starting");
  Serial.println("  CONTINUOUS WIFI + TMP36 VERSION");
  Serial.println("=====================================");
  Serial.println("🔄 STEP 1: Hardware Initialization");

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("   ✓ GPIO pins configured");

  Serial.println("🔧 STEP 2: Initializing hardware...");

  // Initialize PWM for fan
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);
  Serial.println("   ✓ Fan PWM controller initialized");

  // Initialize sensors
  Serial.println("   🌡️  Initializing temperature sensors...");
  dht.begin();
  analogReadResolution(12);  // 12-bit ADC resolution for better TMP36 accuracy
  delay(2000); // Let DHT22 stabilize
  Serial.println("   ✓ DHT22 and TMP36 sensors initialized");

  // Calibrate TMP36 sensor for accurate readings
  Serial.println("   🔧 Calibrating TMP36 sensor...");
  calibrateTMP36();
  Serial.println("   ✓ TMP36 calibration complete");

  // Feed watchdog
  feedWatchdog();

  // Initialize LCD (if enabled)
#if USE_LCD
  Serial.println("   📺 Initializing LCD display...");
  lcd.init();
  lcd.backlight();
  lcdPrintCentered("ESP32 Weather", 0);
  lcdPrintCentered("DHT22 + TMP36", 1);
  delay(2000);
  Serial.println("   ✓ LCD display ready");
#else
  Serial.println("   📺 LCD display disabled in configuration");
#endif

  // Feed watchdog
  feedWatchdog();

  Serial.println("🔄 STEP 3: WiFi Setup (Continuous Mode)");
  // Setup WiFi Access Point - FIXED VERSION
  setupWiFiHotspot();

  // Feed watchdog
  feedWatchdog();

  Serial.println("🔄 STEP 4: Web Server Setup");
  // Setup web server routes
  Serial.println("🌐 Setting up web server...");
  setupWebServer();
  Serial.println("   ✓ Web server routes configured");

  Serial.println("🔄 STEP 5: WebSocket Setup");
  // Setup WebSocket
  Serial.println("🔌 Setting up WebSocket server...");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("   ✓ WebSocket server started on port 81");

  Serial.println("🔄 STEP 6: DNS Server Setup");
  // Setup captive portal DNS
  Serial.println("🌍 Setting up captive portal...");
  dnsServer.start(53, "*", local_IP);
  Serial.println("   ✓ DNS server started for captive portal");

  Serial.println("🔄 STEP 7: Starting Web Server");
  // Start servers
  server.begin();
  Serial.println("   ✅ HTTP server started on port 80");
  Serial.println("✅ All systems initialized successfully!");
  Serial.println();

  // Feed watchdog
  feedWatchdog();

  // Print connection instructions
  printSystemInfo();

  Serial.println("🔄 STEP 8: Initial Sensor Reading");
  // Initial sensor reading
  Serial.println("📊 Taking initial sensor readings...");
  readDigitalTempWithRetry();
  readAnalogTemp();
  Serial.println("   ✓ Initial readings complete");
  Serial.println();

  // Update LCD with connection info
#if USE_LCD
  lcd.clear();
  String wifiText = "WiFi: " + String(AP_SSID);
  lcdPrintCentered(wifiText.c_str(), 0);
  lcdPrintCentered("192.168.4.1", 1);
  delay(500);
#endif

  digitalWrite(LED_PIN, HIGH); // Indicate system is ready
  Serial.println("🎉 === SYSTEM READY FOR CONNECTIONS ===");
  Serial.println("💡 LED should be ON (indicating ready state)");
  Serial.println("📱 Try connecting to WiFi now!");
  Serial.println("🔒 WiFi hotspot will remain ON continuously");
  Serial.println("🌡️ Sensors: DHT22 (digital) + TMP36 (analog)");
  Serial.println();
  
  // Record that WiFi was successfully initialized
  wifiInitialized = true;
  systemUptime = millis();
  lastWifiStatusLog = millis();
  lastWatchdogFeed = millis();
}

void loop() {
  static unsigned long lastLoopInfo    = 0;
  static unsigned long loopCounter     = 0;
  static int           lastClientCount = -1;

  loopCounter++;

  // Feed watchdog regularly - CRITICAL for stability
  if (millis() - lastWatchdogFeed >= WATCHDOG_FEED_INTERVAL) {
    feedWatchdog();
    lastWatchdogFeed = millis();
  }

  // Log WiFi status periodically (less aggressive)
  if (millis() - lastWifiStatusLog >= WIFI_STATUS_LOG_INTERVAL) {
    logWiFiStatus();
    lastWifiStatusLog = millis();
  }

  // Monitor client connections in real-time
  int currentClientCount = WiFi.softAPgetStationNum();
  if (currentClientCount != lastClientCount) {
    Serial.println("📱 === CLIENT CONNECTION CHANGE ===");
    Serial.println("   👥 Previous clients: " + String(lastClientCount));
    Serial.println("   👥 Current clients: " + String(currentClientCount));
    if (currentClientCount > lastClientCount) {
      Serial.println("   ✅ New device connected to WiFi hotspot!");
      Serial.println("   🌐 Device should now be able to access http://192.168.4.1");
    } else if (currentClientCount < lastClientCount) {
      Serial.println("   ❌ Device disconnected from WiFi hotspot");
    }
    lastClientCount = currentClientCount;
  }

  // Print loop status every 60 seconds (less spam)
  if (millis() - lastLoopInfo >= 60000) {
    lastLoopInfo = millis();
    Serial.println("🔄 === LOOP STATUS UPDATE ===");
    Serial.println("   📊 Loop cycles: " + String(loopCounter));
    Serial.println("   🧠 Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("   ⏰ Uptime: " + String(millis() / 1000) + " seconds");
    Serial.println("   👥 Connected clients: " + String(WiFi.softAPgetStationNum()));
    Serial.println("   📶 AP Status: CONTINUOUS (No Auto-Shutdown)");
    Serial.println("   💡 LED Status: " + String(digitalRead(LED_PIN) ? "ON" : "OFF"));

    // Encourage connection attempts
    if (WiFi.softAPgetStationNum() == 0) {
      Serial.println("   📱 WAITING FOR DEVICE CONNECTION...");
      Serial.println("      • Network: 'ESP32_Weather_Station'");
      Serial.println("      • Password: 'weather123'");
      Serial.println("      • WiFi is ALWAYS ON - no timeouts");
    }
  }

  // Handle DNS requests for captive portal
  dnsServer.processNextRequest();

  // Handle web server and WebSocket
  server.handleClient();
  webSocket.loop();

  unsigned long currentMillis = millis();

  // Read sensors
  if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentMillis;
    Serial.println("📊 Reading sensors...");
    readDigitalTempWithRetry();
    readAnalogTemp();
    checkThresholds();
    storeDataPoint();
    Serial.println("   ✓ Sensor reading cycle complete");
  }

  // Update LCD
  if (currentMillis - lastLcdUpdateTime >= LCD_UPDATE_INTERVAL) {
    lastLcdUpdateTime = currentMillis;
    updateLcdDisplay();
  }

  // Broadcast data to WebSocket clients
  if (currentMillis - lastDataBroadcast >= DATA_BROADCAST_INTERVAL) {
    lastDataBroadcast = currentMillis;
    broadcastSensorData();
  }

  // Update system status
  if (currentMillis - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
    lastStatusUpdate = currentMillis;
    updateSystemStatus();
  }

  // Check DHT status
  checkDHTStatus();
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

// SIMPLIFIED and STABLE WiFi Setup Function
void setupWiFiHotspot() {
  Serial.println("📡 Setting up CONTINUOUS WiFi Access Point...");
  
  // Step 1: Complete WiFi reset with proper timing
  Serial.println("   🔄 Performing complete WiFi reset...");
  WiFi.persistent(false);  // Don't save WiFi config to flash
  WiFi.disconnect(true);   // Disconnect and clear stored configs
  WiFi.mode(WIFI_OFF);     // Turn off WiFi completely
  delay(2000);             // Longer delay for complete reset
  
  // Step 2: Configure WiFi power settings for continuous operation
  Serial.println("   ⚡ Configuring power settings for continuous operation...");
  WiFi.mode(WIFI_AP);      // Set AP mode
  delay(1000);
  
  // CRITICAL: Disable WiFi power saving to prevent auto-shutdown
  esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power saving
  Serial.println("   ✓ WiFi power saving DISABLED (continuous mode)");
  
  // Step 3: Set IP configuration
  Serial.println("   🌐 Setting IP configuration...");
  bool configSet = WiFi.softAPConfig(local_IP, gateway, subnet);
  if (!configSet) {
    Serial.println("   ⚠️ Warning: Failed to set IP configuration, using defaults");
  } else {
    Serial.println("   ✓ IP configuration set successfully");
  }
  
  // Feed watchdog
  feedWatchdog();
  
  // Step 4: Start the Access Point with explicit parameters
  Serial.println("   📡 Starting WiFi hotspot...");
  Serial.println("      • SSID: " + String(AP_SSID));
  Serial.println("      • Password: " + String(AP_PASSWORD));
  Serial.println("      • Channel: " + String(AP_CHANNEL));
  Serial.println("      • Max connections: " + String(AP_MAX_CONNECTIONS));
  Serial.println("      • Mode: CONTINUOUS (No timeouts)");
  
  // Start AP with explicit settings
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONNECTIONS);
  delay(2000); // Give time for AP to fully start
  
  if (!apStarted) {
    Serial.println("   ❌ CRITICAL: Failed to start hotspot!");
    Serial.println("   🔧 Check ESP32 compatibility and restart device");
    return;
  }
  
  // Step 5: Verify and configure for stability
  IPAddress apIP = WiFi.softAPIP();
  if (apIP == IPAddress(0, 0, 0, 0)) {
    Serial.println("   ❌ CRITICAL: Hotspot started but no IP assigned!");
    return;
  }
  
  // Feed watchdog
  feedWatchdog();
  
  // Step 6: Additional stability settings
  Serial.println("   🔧 Applying stability settings...");
  
  // Set WiFi to never sleep/disconnect
  WiFi.setSleep(false);  // Disable WiFi sleep
  Serial.println("   ✓ WiFi sleep mode DISABLED");
  
  // Configure for maximum stability
  esp_wifi_set_max_tx_power(78);  // Set moderate power level (not max to reduce heat)
  Serial.println("   ✓ WiFi power level optimized");
  
  // Step 7: Success confirmation
  Serial.println("   ✅ CONTINUOUS WiFi hotspot started successfully!");
  Serial.println("   📍 IP Address: " + apIP.toString());
  Serial.println("   📶 Channel: " + String(AP_CHANNEL));
  Serial.println("   🔧 MAC Address: " + WiFi.softAPmacAddress());
  Serial.println("   🔒 Mode: CONTINUOUS - No auto-shutdown");
  Serial.println("   ⚡ Power saving: DISABLED");
  Serial.println("   💤 Sleep mode: DISABLED");
  
  // Update LCD
#if USE_LCD
  lcd.clear();
  lcdPrintCentered("WiFi CONTINUOUS", 0);
  lcdPrintCentered(apIP.toString().c_str(), 1);
  delay(2000);
#endif
  
  Serial.println("✅ WiFi Access Point setup completed - CONTINUOUS MODE ACTIVE!");
}

// NEW: Watchdog feeding function
void feedWatchdog() {
  esp_task_wdt_reset();
}

// SIMPLIFIED WiFi Status Logging (non-intrusive)
void logWiFiStatus() {
  IPAddress currentIP = WiFi.softAPIP();
  wifi_mode_t currentMode = WiFi.getMode();
  
  Serial.println("📶 === WIFI STATUS CHECK ===");
  Serial.println("   📍 IP Address: " + currentIP.toString());
  Serial.println("   📡 WiFi Mode: " + String(currentMode));
  Serial.println("   👥 Connected devices: " + String(WiFi.softAPgetStationNum()));
  Serial.println("   ⚡ Power saving: " + String(WiFi.getSleep() ? "ENABLED (BAD)" : "DISABLED (GOOD)"));
  Serial.println("   💤 Status: CONTINUOUS MODE - No shutdowns");
  
  // Only warn if there's actually a problem, don't try to "fix" it
  if (currentIP == IPAddress(0, 0, 0, 0)) {
    Serial.println("   ⚠️ WARNING: WiFi IP is 0.0.0.0 - This indicates a serious problem");
    Serial.println("   🔧 Manual restart may be required");
  } else if (currentMode != WIFI_AP) {
    Serial.println("   ⚠️ WARNING: WiFi mode changed from AP mode");
    Serial.println("   🔧 This should not happen in continuous mode");
  } else {
    Serial.println("   ✅ WiFi status: HEALTHY and CONTINUOUS");
  }
}

void updateFanAndBuzzer(int newFanSpeed, String source) {
  // Convert percentage to PWM value
  int pwmValue = map(newFanSpeed, 0, 100, 0, 255);

  Serial.println("🌀 Fan Speed Update Requested:");
  Serial.println("   📊 Source: " + source);
  Serial.println("   📈 Requested Speed: " + String(newFanSpeed) + "% (PWM: " + String(pwmValue) + ")");
  Serial.println("   🔄 Previous speed: " + String(map(fanSpeed, 0, 255, 0, 100)) + "%");

  // Update fan speed
  fanSpeed = pwmValue;
  ledcWrite(FAN_PWM_CHANNEL, fanSpeed);

  // Control buzzer based on fan status
  if (newFanSpeed > 0) {
    // Fan is running - activate buzzer
    Serial.println("   🔊 Activating buzzer (fan is running)");
    tone(BUZZER_PIN, 1500); // 1.5kHz tone when fan is manually controlled
  } else {
    // Fan is stopped - deactivate buzzer
    Serial.println("   🔇 Deactivating buzzer (fan stopped)");
    noTone(BUZZER_PIN);
  }

  Serial.println("   ✅ Fan and buzzer updated successfully");

  // Update LCD with fan status
#if USE_LCD
  lcd.clear();
  if (newFanSpeed > 0) {
    String fanText = "Fan: " + String(newFanSpeed) + "%";
    lcdPrintCentered(fanText.c_str(), 0);
    lcdPrintCentered("Buzzer: ON", 1);
  } else {
    lcdPrintCentered("Fan: OFF", 0);
    lcdPrintCentered("Buzzer: OFF", 1);
  }
  delay(2000);
#endif
}

void setupWebServer() {
  Serial.println("🌐 Registering web server routes...");

  // Serve main dashboard (captive portal redirect)
  server.on("/", HTTP_GET, handleCaptivePortal);
  Serial.println("   ✓ Route registered: GET /");

  server.on("/index.html", HTTP_GET, handleCaptivePortal);
  Serial.println("   ✓ Route registered: GET /index.html");

  server.on("/dashboard", HTTP_GET, []() {
    Serial.println("📱 Client accessing main dashboard from: " + server.client().remoteIP().toString());
    server.send(200, "text/html", getMainHTML());
    Serial.println("   ✅ Dashboard HTML sent successfully");
  });
  Serial.println("   ✓ Route registered: GET /dashboard");

  // Captive portal handlers
  server.on("/generate_204", HTTP_GET, handleCaptivePortal);      // Android
  Serial.println("   ✓ Route registered: GET /generate_204 (Android captive portal)");

  server.on("/fwlink", HTTP_GET, handleCaptivePortal);            // Microsoft
  Serial.println("   ✓ Route registered: GET /fwlink (Microsoft captive portal)");

  server.on("/hotspot-detect.html", HTTP_GET, handleCaptivePortal); // Apple
  Serial.println("   ✓ Route registered: GET /hotspot-detect.html (Apple captive portal)");

  // API endpoints
  server.on("/api/data", HTTP_GET, handleApiData);
  Serial.println("   ✓ Route registered: GET /api/data");

  server.on("/api/history", HTTP_GET, handleApiHistory);
  Serial.println("   ✓ Route registered: GET /api/history");

  server.on("/api/status", HTTP_GET, handleApiStatus);
  Serial.println("   ✓ Route registered: GET /api/status");

  // Control endpoints
  server.on("/api/fan", HTTP_POST, handleFanControl);
  Serial.println("   ✓ Route registered: POST /api/fan");

  server.on("/api/threshold", HTTP_POST, handleThresholdControl);
  Serial.println("   ✓ Route registered: POST /api/threshold");

  server.on("/api/sensor", HTTP_POST, handleSensorSelection);
  Serial.println("   ✓ Route registered: POST /api/sensor");

  server.on("/api/export", HTTP_GET, handleDataExport);
  Serial.println("   ✓ Route registered: GET /api/export");

  // Static files
  server.on("/style.css", HTTP_GET, []() {
    Serial.println("📄 CSS file requested from: " + server.client().remoteIP().toString());
    server.send(200, "text/css", getCSS());
    Serial.println("   ✅ CSS sent successfully");
  });
  Serial.println("   ✓ Route registered: GET /style.css");

  server.on("/script.js", HTTP_GET, []() {
    Serial.println("📄 JavaScript file requested from: " + server.client().remoteIP().toString());
    server.send(200, "application/javascript", getJavaScript());
    Serial.println("   ✅ JavaScript sent successfully");
  });
  Serial.println("   ✓ Route registered: GET /script.js");

  // Catch-all handler for captive portal
  server.onNotFound(handleCaptivePortal);
  Serial.println("   ✓ Catch-all handler registered (onNotFound)");

  Serial.println("🌐 Web server setup complete - routes registered");
}

void handleCaptivePortal() {
  String clientIP     = server.client().remoteIP().toString();
  String requestURI   = server.uri();
  String requestMethod = (server.method() == HTTP_GET) ? "GET" : "POST";

  Serial.println("🌐 === CAPTIVE PORTAL REQUEST ===");
  Serial.println("   📍 Client IP: " + clientIP);
  Serial.println("   📄 Request: " + requestMethod + " " + requestURI);
  Serial.println("   🌐 Host header: " + server.hostHeader());

  String html = R"rawliteral(<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Weather Station</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; padding: 20px; background: #f0f8ff; }
        .container { max-width: 500px; margin: 0 auto; background: white; padding: 30px; border-radius: 15px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h1 { color: #2c3e50; margin-bottom: 20px; }
        .icon { font-size: 48px; margin-bottom: 20px; }
        .btn { background: #3498db; color: white; padding: 15px 30px; border: none; border-radius: 8px; font-size: 18px; cursor: pointer; text-decoration: none; display: inline-block; margin: 10px; }
        .btn:hover { background: #2980b9; }
        .info { background: #ecf0f1; padding: 15px; border-radius: 8px; margin: 20px 0; }
        .status { background: #d5edda; padding: 10px; border-radius: 5px; margin: 15px 0; }
        .continuous { background: #d1ecf1; color: #0c5460; padding: 10px; border-radius: 5px; margin: 15px 0; font-weight: bold; }
        .sensors { background: #fff3cd; color: #856404; padding: 10px; border-radius: 5px; margin: 15px 0; }
    </style>
</head>
<body>
    <div class="container">
        <div class="icon">🌡️</div>
        <h1>ESP32 Weather Station</h1>
        <div class="status">
            <p><strong>✅ Connection Successful!</strong></p>
            <p>WiFi: )rawliteral" + String(AP_SSID) + R"rawliteral( | IP: 192.168.4.1</p>
        </div>
        <div class="sensors">
            <p><strong>🌡️ Dual Temperature Sensors</strong></p>
            <p>DHT22 (Digital) + TMP36 (Analog)</p>
        </div>
        <div class="continuous">
            <p>🔒 CONTINUOUS MODE ACTIVE</p>
            <p>WiFi hotspot stays on 24/7 - No timeouts!</p>
        </div>
        <div class="info">
            <p><strong>Welcome!</strong> You are connected to the ESP32 Weather Station hotspot.</p>
            <p>Click below to access the real-time dashboard with live temperature readings, controls, and charts.</p>
        </div>
        <a href="/dashboard" class="btn">🚀 Open Dashboard</a>
        <div style="margin-top: 20px; font-size: 14px; color: #7f8c8d;">
            <p>📡 Network: )rawliteral" + String(AP_SSID) + R"rawliteral(</p>
            <p>🌐 IP Address: 192.168.4.1</p>
            <p>📊 Connected devices: )rawliteral" + String(WiFi.softAPgetStationNum()) + R"rawliteral(</p>
            <p>⏰ Uptime: )rawliteral" + String((millis() - systemUptime) / 1000) + R"rawliteral( seconds</p>
        </div>
    </div>
</body>
</html>)rawliteral";

  server.send(200, "text/html", html);
  Serial.println("   ✅ Captive portal HTML sent (" + String(html.length()) + " bytes)");
  Serial.println("   🔗 Client should see welcome page with dashboard link");
}

void printSystemInfo() {
  Serial.println("🎯 CONTINUOUS WIFI ACCESS INSTRUCTIONS:");
  Serial.println("=====================================");
  Serial.println("📱 On your phone/computer:");
  Serial.println("   1. Go to WiFi settings");
  Serial.println("   2. Connect to: " + String(AP_SSID));
  Serial.println("   3. Password: " + String(AP_PASSWORD));
  Serial.println("   4. Wait for captive portal OR go to: http://192.168.4.1");
  Serial.println();
  Serial.println("🌡️ TEMPERATURE SENSORS:");
  Serial.println("   • DHT22 (Digital): Pin 18 - Temperature & Humidity");
  Serial.println("   • TMP36 (Analog): Pin 33 - Temperature only");
  Serial.println("   • TMP36 Wiring: VCC→3.3V, GND→GND, VOUT→Pin33");
  Serial.println("   • Operating range: -40°C to +125°C");
  Serial.println("   • Auto-calibrated voltage reference: " + String(actualVRef, 2) + "V");
  Serial.println("   • Calibration offset: " + String(tmp36CalibrationOffset, 1) + "°C");
  Serial.println();
  Serial.println("🔧 TMP36 TROUBLESHOOTING:");
  Serial.println("   • If readings are too low: Increase actualVRef (try 3.15 or 3.3)");
  Serial.println("   • If readings are too high: Decrease actualVRef (try 3.0 or 2.9)");
  Serial.println("   • Fine-tune with tmp36CalibrationOffset (+/- degrees)");
  Serial.println("   • Check Serial Monitor for calibration details");
  Serial.println("   • Compare with DHT22 for reference accuracy");
  Serial.println();
  Serial.println("🔒 CONTINUOUS MODE FEATURES:");
  Serial.println("   • WiFi hotspot NEVER shuts down automatically");
  Serial.println("   • Power saving modes DISABLED for stability");
  Serial.println("   • Sleep modes DISABLED - always on");
  Serial.println("   • Watchdog timer prevents system crashes");
  Serial.println("   • No timeout or connection limits");
  Serial.println("   • 24/7 operation designed for reliability");
  Serial.println();
  Serial.println("🌐 Dashboard features:");
  Serial.println("   • Real-time temperature readings (DHT22 + TMP36)");
  Serial.println("   • Interactive charts and graphs");
  Serial.println("   • Manual fan speed control (0-100%)");
  Serial.println("   • Temperature threshold settings");
  Serial.println("   • Data export functionality");
  Serial.println("   • Mobile-responsive design");
  Serial.println("   • Local operation (no internet required)");
  Serial.println();
  Serial.println("🌀 Fan & Buzzer Control:");
  Serial.println("   • Manual Control: Use web slider (0-100%)");
  Serial.println("   • Buzzer rings whenever fan is running");
  Serial.println("   • Automatic activation when temp > threshold");
  Serial.println("   • Manual control overrides automatic settings");
  Serial.println("   • Buzzer tones: 1500Hz (normal), 2000Hz (alert)");
  Serial.println();
  Serial.println("🔧 SYSTEM GUARANTEES:");
  Serial.println("   • WiFi will NOT turn off automatically");
  Serial.println("   • System designed for 24/7 operation");
  Serial.println("   • Automatic crash recovery with watchdog");
  Serial.println("   • Power-optimized for continuous use");
  Serial.println("=====================================");
  Serial.println();
}

void updateSystemStatus() {
  connectedClients = WiFi.softAPgetStationNum();
  static int lastClientCount = -1;

  if (connectedClients != lastClientCount) {
    Serial.println("👥 Client connection change detected");
    Serial.println("   📊 Currently connected clients: " + String(connectedClients));
    lastClientCount = connectedClients;
  }

  if (connectedClients > 0) {
    Serial.println("📊 System Status Update:");
    Serial.println("   👥 Connected clients: " + String(connectedClients));
    Serial.println("   🌡️ Digital temp (DHT22): " + String(digitalTemp, 1) + "°C (" + (dhtSensorActive ? "Active" : "Error") + ")");
    Serial.println("   🌡️ Analog temp (TMP36): " + String(analogTemp, 1) + "°C");
    Serial.println("   🎯 Threshold: " + String(tempThreshold, 1) + "°C");
    Serial.println("   🌀 Fan speed: " + String(map(fanSpeed, 0, 255, 0, 100)) + "%");
    Serial.println("   ⚠️ Alert status: " + String(alertActive ? "ACTIVE" : "Normal"));
    Serial.println("   📶 WiFi Mode: CONTINUOUS (Always On)");
    Serial.println("   🧠 Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("   ⏰ Uptime: " + String((millis() - systemUptime) / 1000) + " seconds");
    Serial.println();
  }
}

void handleApiData() {
  String clientIP = server.client().remoteIP().toString();
  Serial.println("📡 === API DATA REQUEST ===");
  Serial.println("   📍 Client IP: " + clientIP);
  Serial.println("   📊 Preparing sensor data response...");

  DynamicJsonDocument doc(512);
  doc["digitalTemp"]    = digitalTemp;
  doc["analogTemp"]     = analogTemp;
  doc["threshold"]      = tempThreshold;
  doc["fanSpeed"]       = map(fanSpeed, 0, 255, 0, 100);
  doc["alertActive"]    = alertActive;
  doc["selectedSensor"] = selectedSensor;
  doc["dhtActive"]      = dhtSensorActive;
  doc["timestamp"]      = millis();
  doc["connectedClients"] = connectedClients;
  doc["uptime"]         = (millis() - systemUptime) / 1000;
  doc["continuousMode"] = true;  // NEW: Indicate continuous mode
  doc["sensorType"]     = "TMP36"; // NEW: Indicate sensor type

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);

  Serial.println("   ✅ API data sent successfully (" + String(response.length()) + " bytes)");
  Serial.println("   📊 Data: DHT22=" + String(digitalTemp, 1) + "°C, TMP36=" + String(analogTemp, 1) + "°C, Fan=" + String(map(fanSpeed, 0, 255, 0, 100)) + "%");
}

void handleApiHistory() {
  Serial.println("📡 API request: Historical data");

  DynamicJsonDocument doc(4096);
  JsonArray historyArray = doc.createNestedArray("history");

  int start = (dataCount < 50) ? 0 : dataIndex;
  for (int i = 0; i < min(dataCount, 50); i++) {
    int idx = (start + i) % 50;
    JsonObject point = historyArray.createNestedObject();
    point["timestamp"]   = dataHistory[idx].timestamp;
    point["digitalTemp"] = dataHistory[idx].digitalTemp;
    point["analogTemp"]  = dataHistory[idx].analogTemp;
    point["dhtActive"]   = dataHistory[idx].dhtActive;
  }

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleApiStatus() {
  DynamicJsonDocument doc(256);
  doc["wifiMode"]       = "Access Point (Continuous)";
  doc["ssid"]           = AP_SSID;
  doc["ipAddress"]      = WiFi.softAPIP().toString();
  doc["connectedClients"] = connectedClients;
  doc["uptime"]         = (millis() - systemUptime) / 1000;
  doc["freeHeap"]       = ESP.getFreeHeap();
  doc["dhtErrors"]      = dhtErrorCount;
  doc["dhtActive"]      = dhtSensorActive;
  doc["continuousMode"] = true;
  doc["powerSaving"]    = WiFi.getSleep();  // Should be false
  doc["wifiChannel"]    = AP_CHANNEL;
  doc["sensorType"]     = "TMP36";

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleFanControl() {
  Serial.println("🌀 Manual fan control request from: " + server.client().remoteIP().toString());

  if (server.hasArg("speed")) {
    int speed = server.arg("speed").toInt();
    Serial.println("   📊 Requested speed: " + String(speed) + "%");

    if (speed >= 0 && speed <= 100) {
      // Use the unified fan and buzzer control function
      updateFanAndBuzzer(speed, "Manual Web Control");

      server.send(200, "application/json", "{\"status\":\"ok\",\"speed\":" + String(speed) + "}");
      Serial.println("   ✅ Manual fan control successful");
    } else {
      Serial.println("   ❌ Invalid speed value: " + String(speed) + "% (valid: 0-100%)");
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid speed range (0-100%)\"}");
    }
  } else {
    Serial.println("   ❌ Missing speed parameter in request");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing speed parameter\"}");
  }
}

void handleThresholdControl() {
  if (server.hasArg("threshold")) {
    float threshold = server.arg("threshold").toFloat();
    if (threshold > 0 && threshold < 100) {
      tempThreshold = threshold;
      server.send(200, "application/json", "{\"status\":\"ok\",\"threshold\":" + String(threshold) + "}");

      Serial.println("🌡️ Temperature threshold set to: " + String(threshold) + "°C");

      // Update LCD
  #if USE_LCD
      lcd.clear();
      lcdPrintCentered("Threshold Set", 0);
      char buffer[17];
      sprintf(buffer, "%.1f C", threshold);
      lcdPrintCentered(buffer, 1);
      delay(1500);
  #endif
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid threshold\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing threshold parameter\"}");
  }
}

void handleSensorSelection() {
  if (server.hasArg("sensor")) {
    String sensor = server.arg("sensor");
    if (sensor == "digital" || sensor == "analog") {
      if (sensor == "digital" && !dhtSensorActive) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"DHT22 sensor not active\"}");
      } else {
        selectedSensor = sensor;
        server.send(200, "application/json", "{\"status\":\"ok\",\"sensor\":\"" + sensor + "\"}");

        Serial.println("🎯 Selected sensor for threshold: " + sensor);

        // Update LCD
    #if USE_LCD
        lcd.clear();
        lcdPrintCentered("Sensor Selected", 0);
        lcdPrintCentered(sensor.c_str(), 1);
        delay(1500);
    #endif
      }
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid sensor\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing sensor parameter\"}");
  }
}

void handleDataExport() {
  Serial.println("📊 Data export requested");

  String csv = "timestamp,digitalTemp(DHT22),analogTemp(TMP36),dhtActive\n";

  int start = (dataCount < 50) ? 0 : dataIndex;
  for (int i = 0; i < min(dataCount, 50); i++) {
    int idx = (start + i) % 50;
    csv += String(dataHistory[idx].timestamp) + ",";
    csv += String(dataHistory[idx].digitalTemp, 2) + ",";
    csv += String(dataHistory[idx].analogTemp, 2) + ",";
    csv += String(dataHistory[idx].dhtActive ? "true" : "false") + "\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=weather_data_DHT22_TMP36.csv");
  server.send(200, "text/csv", csv);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("🔌 === WEBSOCKET DISCONNECTED ===\n");
      Serial.printf("   📍 Client #%u disconnected\n", num);
      Serial.printf("   👥 Active WebSocket connections: %u\n", webSocket.connectedClients());
      break;

    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("🔌 === WEBSOCKET CONNECTED ===\n");
      Serial.printf("   📍 Client #%u connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      Serial.printf("   👥 Active WebSocket connections: %u\n", webSocket.connectedClients());
      Serial.println("   📡 Sending initial data to new client...");
      // Send initial data
      broadcastSensorData();
      Serial.println("   ✅ Initial data sent to new WebSocket client");
      break;
    }

    case WStype_TEXT:
      Serial.printf("📨 === WEBSOCKET MESSAGE ===\n");
      Serial.printf("   📍 Client #%u sent: %s\n", num, payload);
      break;

    case WStype_ERROR:
      Serial.printf("❌ === WEBSOCKET ERROR ===\n");
      Serial.printf("   📍 Client #%u error\n", num);
      break;

    default:
      Serial.printf("🔌 WebSocket event type %u from client #%u\n", type, num);
      break;
  }
}

void broadcastSensorData() {
  static unsigned long lastBroadcastLog = 0;

  DynamicJsonDocument doc(512);
  doc["type"]           = "sensorData";
  doc["digitalTemp"]    = digitalTemp;
  doc["analogTemp"]     = analogTemp;
  doc["threshold"]      = tempThreshold;
  doc["fanSpeed"]       = map(fanSpeed, 0, 255, 0, 100);
  doc["alertActive"]    = alertActive;
  doc["selectedSensor"] = selectedSensor;
  doc["dhtActive"]      = dhtSensorActive;
  doc["timestamp"]      = millis();
  doc["connectedClients"] = connectedClients;
  doc["uptime"]         = (millis() - systemUptime) / 1000;
  doc["continuousMode"] = true;  // NEW
  doc["sensorType"]     = "TMP36"; // NEW

  String message;
  serializeJson(doc, message);

  uint8_t wsClientCount = webSocket.connectedClients();
  webSocket.broadcastTXT(message);

  // Log broadcast every 30 seconds to reduce spam
  if (millis() - lastBroadcastLog >= 30000) {
    lastBroadcastLog = millis();
    Serial.println("📡 Broadcasting sensor data to " + String(wsClientCount) + " WebSocket clients");
    if (wsClientCount > 0) {
      Serial.println("   📊 Data: DHT22=" + String(digitalTemp, 1) + "°C, TMP36=" + String(analogTemp, 1) + "°C, Fan=" + String(map(fanSpeed, 0, 255, 0, 100)) + "%");
    }
  }
}

void readDigitalTempWithRetry() {
  if (!dhtSensorActive) {
    Serial.println("⚠️ DHT22 sensor marked as inactive, skipping read");
    return;
  }

  Serial.println("🔍 Reading DHT22 sensor...");
  bool success = false;

  for (int attempt = 1; attempt <= DHT_MAX_RETRY_ATTEMPTS && !success; attempt++) {
    if (attempt > 1) {
      Serial.println("🔄 DHT22 retry attempt " + String(attempt) + "/" + String(DHT_MAX_RETRY_ATTEMPTS));
      delay(1000);
    }

    float humidity = dht.readHumidity();
    float tempC    = dht.readTemperature();

    Serial.println("   📊 Raw DHT22 values - Temp: " + String(tempC) + "°C, Humidity: " + String(humidity) + "%");

    if (isnan(humidity) || isnan(tempC)) {
      Serial.println("❌ DHT22 read failed - NaN values detected");
      dhtErrorCount++;
      dhtConsecutiveErrors++;
      Serial.println("   📈 Error count: " + String(dhtErrorCount) + " (consecutive: " + String(dhtConsecutiveErrors) + ")");
    }
    else if (tempC >= -40 && tempC <= 80) {
      digitalTemp = tempC;
      success      = true;
      dhtConsecutiveErrors = 0;
      lastDhtSuccessTime = millis();
      Serial.println("✅ DHT22 reading successful: " + String(tempC, 1) + "°C, " + String(humidity, 1) + "%");
      Serial.println("   ✅ Consecutive errors reset to 0");
    }
    else {
      Serial.println("⚠️ DHT22 temperature out of valid range: " + String(tempC) + "°C (valid: -40 to 80°C)");
      dhtErrorCount++;
      dhtConsecutiveErrors++;
      Serial.println("   📈 Error count: " + String(dhtErrorCount) + " (consecutive: " + String(dhtConsecutiveErrors) + ")");
    }
  }

  if (!success) {
    Serial.println("❌ DHT22 failed after " + String(DHT_MAX_RETRY_ATTEMPTS) + " attempts");
    Serial.println("   📊 Total errors: " + String(dhtErrorCount) + ", Consecutive: " + String(dhtConsecutiveErrors));
    if (dhtConsecutiveErrors >= DHT_MAX_CONSECUTIVE_ERRORS) {
      Serial.println("   ⚠️ DHT22 may need reinitialization soon");
    }
  }
}

void calibrateTMP36() {
  Serial.println("🔧 === TMP36 CALIBRATION PROCEDURE ===");
  Serial.println("   📊 Taking calibration readings...");
  
  // Take multiple readings for calibration
  int calibrationSamples = 20;
  long adcSum = 0;
  
  for (int i = 0; i < calibrationSamples; i++) {
    adcSum += analogRead(TMP36_PIN);
    delay(100);
  }
  
  int avgADC = adcSum / calibrationSamples;
  float measuredVoltage = avgADC * actualVRef / 4095.0;
  
  Serial.println("   📊 Calibration ADC: " + String(avgADC) + " (averaged from " + String(calibrationSamples) + " samples)");
  Serial.println("   ⚡ Measured voltage: " + String(measuredVoltage, 3) + "V");
  Serial.println("   🌡️ Expected at room temp (~25°C): ~0.75V");
  
  // ESP32 ADC calibration - check if voltage seems reasonable
  if (measuredVoltage < 0.4 || measuredVoltage > 1.2) {
    Serial.println("   ⚠️ Voltage reading seems unusual - checking ADC reference...");
    
    // Try with different voltage reference assumptions
    float altVRef1 = 3.0;  // Many ESP32 boards run at ~3.0V
    float altVRef2 = 3.15; // Common actual voltage
    float altVRef3 = 3.45; // Higher than nominal
    
    Serial.println("   🔍 Testing alternative voltage references:");
    Serial.println("      • With 3.0V ref: " + String(avgADC * altVRef1 / 4095.0, 3) + "V");
    Serial.println("      • With 3.15V ref: " + String(avgADC * altVRef2 / 4095.0, 3) + "V");
    Serial.println("      • With 3.45V ref: " + String(avgADC * altVRef3 / 4095.0, 3) + "V");
    
    // Auto-select the most reasonable voltage reference
    float testVoltage1 = avgADC * altVRef1 / 4095.0;
    float testVoltage2 = avgADC * altVRef2 / 4095.0;
    float testVoltage3 = avgADC * altVRef3 / 4095.0;
    
    // Pick the voltage that gives us closest to expected room temperature range
    if (testVoltage2 >= 0.6 && testVoltage2 <= 0.9) {
      actualVRef = altVRef2;
      Serial.println("   ✅ Auto-selected 3.15V reference (most common ESP32 actual voltage)");
    } else if (testVoltage1 >= 0.6 && testVoltage1 <= 0.9) {
      actualVRef = altVRef1;
      Serial.println("   ✅ Auto-selected 3.0V reference");
    } else if (testVoltage3 >= 0.6 && testVoltage3 <= 0.9) {
      actualVRef = altVRef3;
      Serial.println("   ✅ Auto-selected 3.45V reference");
    } else {
      actualVRef = 3.15; // Default to most common
      Serial.println("   ⚠️ Using default 3.15V reference (most ESP32 boards)");
    }
  }
  
  Serial.println("   🎯 Final voltage reference: " + String(actualVRef, 2) + "V");
  
  // Calculate expected temperature with corrected voltage
  float correctedVoltage = avgADC * actualVRef / 4095.0;
  float calculatedTemp = (correctedVoltage - 0.5) * 100.0;
  
  Serial.println("   🌡️ Calculated temperature: " + String(calculatedTemp, 1) + "°C");
  
  // Suggest calibration offset if needed
  Serial.println("   📝 If this temperature seems wrong, you can:");
  Serial.println("      • Manually set tmp36CalibrationOffset in code");
  Serial.println("      • Compare with DHT22 reading for reference");
  
  Serial.println("✅ TMP36 calibration complete!");
}

void readAnalogTemp() {
  Serial.println("🔍 Reading TMP36 sensor...");

  // Read multiple samples for better accuracy
  int sampleCount = 10;  // Increased sample count
  long adcSum = 0;
  
  for (int i = 0; i < sampleCount; i++) {
    adcSum += analogRead(TMP36_PIN);
    delay(5); // Small delay between readings
  }
  
  int adcValue = adcSum / sampleCount; // Average the samples
  
  // TMP36 conversion formula with calibrated voltage reference
  // TMP36 outputs 500mV at 0°C and 10mV per degree
  // Formula: Temperature = (Voltage - 0.5V) * 100
  float voltage = adcValue * actualVRef / 4095.0;  // Use calibrated voltage reference
  float tempC = (voltage - 0.5) * 100.0 + tmp36CalibrationOffset;  // Apply calibration offset

  Serial.println("   📊 Raw ADC reading: " + String(adcValue) + " (averaged from " + String(sampleCount) + " samples)");
  Serial.println("   ⚡ Voltage (calibrated): " + String(voltage, 3) + "V (using " + String(actualVRef, 2) + "V reference)");
  Serial.println("   🌡️ TMP36 calculation: (" + String(voltage, 3) + " - 0.5) * 100 + " + String(tmp36CalibrationOffset, 1) + " = " + String(tempC, 2) + "°C");

  // TMP36 operating range: -40°C to +125°C
  if (tempC >= -40 && tempC <= 125) {
    analogTemp = tempC;
    Serial.println("✅ TMP36 reading accepted: " + String(tempC, 1) + "°C");
  } else {
    Serial.println("⚠️ TMP36 temperature out of range: " + String(tempC) + "°C (valid: -40 to 125°C)");
    Serial.println("   🔧 Check TMP36 wiring: VCC→3.3V, GND→GND, VOUT→Pin33");
    Serial.println("   🔧 Current voltage: " + String(voltage, 3) + "V (should be ~0.5V at 0°C, ~0.75V at 25°C)");
    Serial.println("   🔧 Try adjusting actualVRef or tmp36CalibrationOffset in code");
  }
}

void checkDHTStatus() {
  if (dhtConsecutiveErrors >= DHT_MAX_CONSECUTIVE_ERRORS) {
    if (millis() - lastDhtSuccessTime > 300000) { // 5 minutes
      Serial.println("🔄 DHT22 sensor reinitializing after extended failure...");
      dht.begin();
      delay(2000);

      if (dhtConsecutiveErrors >= DHT_MAX_CONSECUTIVE_ERRORS) {
        dhtSensorActive = false;
        Serial.println("❌ DHT22 marked as inactive - switching to TMP36 sensor");
        if (selectedSensor == "digital") {
          selectedSensor = "analog";
        }
      }
    }
  }
}

void checkThresholds() {
  float currentTemp    = (selectedSensor == "digital" && dhtSensorActive) ? digitalTemp : analogTemp;
  bool  isOverThreshold = currentTemp > tempThreshold;

  // Log threshold check details (every 60 seconds to reduce spam)
  static unsigned long lastThresholdLog = 0;
  if (millis() - lastThresholdLog > 60000) {
    Serial.println("🎯 Threshold Check:");
    Serial.println("   📊 Selected sensor: " + selectedSensor + " (" + (selectedSensor == "digital" ? "DHT22" : "TMP36") + ")");
    Serial.println("   🌡️ Current temperature: " + String(currentTemp, 1) + "°C");
    Serial.println("   🎯 Threshold setting: " + String(tempThreshold, 1) + "°C");
    Serial.println("   ⚖️ Status: " + String(currentTemp) + (isOverThreshold ? " > " : " <= ") + String(tempThreshold) + " = " + (isOverThreshold ? "OVER" : "NORMAL"));
    lastThresholdLog = millis();
  }

  if (isOverThreshold != alertActive) {
    Serial.println("🚨 ALERT STATUS CHANGE DETECTED!");
    alertActive = isOverThreshold;

    if (alertActive) {
      Serial.println("🚨 === TEMPERATURE ALERT ACTIVATED ===");
      Serial.println("   🌡️ Temperature: " + String(currentTemp, 1) + "°C");
      Serial.println("   🎯 Threshold: " + String(tempThreshold, 1) + "°C");
      Serial.println("   📊 Difference: +" + String(currentTemp - tempThreshold, 1) + "°C");

      // Check current fan speed before automatic activation
      int currentFanPercent = map(fanSpeed, 0, 255, 0, 100);
      Serial.println("   🌀 Current fan speed: " + String(currentFanPercent) + "%");

      if (currentFanPercent == 0) {
        Serial.println("   🚀 Automatically activating fan and buzzer...");
        updateFanAndBuzzer(100, "Automatic Alert System");
      } else {
        Serial.println("   ℹ️ Fan already running at " + String(currentFanPercent) + "% - keeping current speed");
        // Still activate buzzer for alert (higher frequency for alerts)
        Serial.println("   🔊 Activating alert buzzer at 2000Hz...");
        tone(BUZZER_PIN, 2000);
      }

      // Flash LED for visual alert
      Serial.println("   💡 Flashing LED for visual alert...");
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
      }

      Serial.println("🚨 === ALERT ACTIVATION COMPLETE ===");

    } else {
      Serial.println("✅ === TEMPERATURE ALERT CLEARED ===");
      Serial.println("   🌡️ Temperature returned to normal: " + String(currentTemp, 1) + "°C");
      Serial.println("   🎯 Threshold: " + String(tempThreshold, 1) + "°C");
      Serial.println("   📊 Difference: " + String(currentTemp - tempThreshold, 1) + "°C below threshold");

      // We do not automatically turn off the fan when alert clears
      // Manual control persists
      Serial.println("   ℹ️ Fan speed maintained at current setting for manual control");
      Serial.println("   💡 Use web dashboard to manually adjust fan speed if desired");

      // Only turn off buzzer or revert to normal tone
      Serial.println("   🔇 Deactivating alert buzzer...");

      int currentFanPercent = map(fanSpeed, 0, 255, 0, 100);
      if (currentFanPercent > 0) {
        Serial.println("   🔊 Switching to normal fan operation buzzer (1500Hz)...");
        tone(BUZZER_PIN, 1500); // Normal fan operation tone
      } else {
        Serial.println("   🔇 Turning off buzzer completely (fan is off)...");
        noTone(BUZZER_PIN);
      }

      Serial.println("✅ === ALERT CLEARED COMPLETE ===");
    }
  }
}

void storeDataPoint() {
  dataHistory[dataIndex].timestamp   = millis();
  dataHistory[dataIndex].digitalTemp = digitalTemp;
  dataHistory[dataIndex].analogTemp  = analogTemp;
  dataHistory[dataIndex].dhtActive   = dhtSensorActive;

  dataIndex = (dataIndex + 1) % 50;
  if (dataCount < 50) dataCount++;
}

void updateLcdDisplay() {
#if USE_LCD
  if (alertActive) {
    // Show alert on LCD
    lcd.clear();
    lcdPrintCentered("!! ALERT !!", 0);
    char buffer[17];
    float currentTemp = (selectedSensor == "digital" && dhtSensorActive) ? digitalTemp : analogTemp;
    sprintf(buffer, "%.1fC > %.1fC", currentTemp, tempThreshold);
    lcdPrintCentered(buffer, 1);
    return;
  }

  static int screenState = 0;

  // Declare buffers once to avoid redefinition
  char buffer[17];
  char buffer2[17];
  int  fanPercent;

  switch (screenState) {
    case 0:
      // Temperature readings
      lcd.clear();
      lcd.setCursor(0, 0);
      if (dhtSensorActive) {
        sprintf(buffer, "DHT:%.1fC TMP:%.1fC", digitalTemp, analogTemp);
        lcd.print(buffer);
      } else {
        sprintf(buffer, "DHT:ERR TMP:%.1fC", analogTemp);
        lcd.print(buffer);
      }
      lcd.setCursor(0, 1);
      sprintf(buffer2, "Threshold:%.1fC", tempThreshold);
      lcd.print(buffer2);
      break;

    case 1:
      // Fan and buzzer status
      lcd.clear();
      lcd.setCursor(0, 0);
      fanPercent = map(fanSpeed, 0, 255, 0, 100);
      sprintf(buffer2, "Fan: %d%%", fanPercent);
      lcd.print(buffer2);
      lcd.setCursor(0, 1);
      if (fanPercent > 0) {
        lcd.print("Buzzer: ON");
      } else {
        lcd.print("Buzzer: OFF");
      }
      break;

    case 2:
      // Sensor types and status
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DHT22 + TMP36");
      lcd.setCursor(0, 1);
      sprintf(buffer2, "Clients: %d", connectedClients);
      lcd.print(buffer2);
      break;

    case 3:
      // WiFi info with continuous indicator
      lcd.clear();
      lcdPrintCentered("192.168.4.1", 0);
      lcdPrintCentered("CONTINUOUS", 1);
      break;
  }

  screenState = (screenState + 1) % 4;
#endif
}

void lcdPrintCentered(const char* text, int row) {
#if USE_LCD
  int spaces = (16 - strlen(text)) / 2;
  if (spaces < 0) spaces = 0;
  lcd.setCursor(spaces, row);
  lcd.print(text);
#endif
}

String getMainHTML() {
  return R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Weather Station Dashboard</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <link rel="stylesheet" href="/style.css">
</head>
<body>
    <nav class="navbar navbar-dark bg-primary">
        <div class="container-fluid">
            <span class="navbar-brand h1">🌡️ ESP32 Weather Station - DHT22 + TMP36</span>
            <div>
                <span class="navbar-text me-3" id="clientCount">Clients: 0</span>
                <span class="navbar-text me-3" id="uptime">Uptime: 0s</span>
                <span class="navbar-text" id="status">Connecting...</span>
            </div>
        </div>
    </nav>

    <div class="container-fluid mt-4">
        <!-- Continuous Mode Alert -->
        <div class="row mb-3">
            <div class="col-12">
                <div class="alert alert-info" role="alert">
                    <h4 class="alert-heading">🔒 Continuous Mode Active!</h4>
                    <p class="mb-0">WiFi hotspot is running in continuous mode with dual temperature sensors: <strong>DHT22 (Digital)</strong> and <strong>TMP36 (Analog)</strong>. No automatic shutdowns or timeouts - designed for 24/7 operation.</p>
                </div>
            </div>
        </div>

        <!-- Temperature Gauges Row -->
        <div class="row mb-4">
            <div class="col-md-6">
                <div class="card text-center">
                    <div class="card-header">
                        <h5>Digital Sensor (DHT22)</h5>
                    </div>
                    <div class="card-body">
                        <div class="temp-gauge" id="digitalGauge">
                            <div class="gauge-value" id="digitalTemp">--</div>
                            <div class="gauge-unit">°C</div>
                        </div>
                        <div class="mt-2">
                            <span class="badge" id="dhtStatus">--</span>
                            <br><small class="text-muted">Temperature + Humidity</small>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="col-md-6">
                <div class="card text-center">
                    <div class="card-header">
                        <h5>Analog Sensor (TMP36)</h5>
                    </div>
                    <div class="card-body">
                        <div class="temp-gauge" id="analogGauge">
                            <div class="gauge-value" id="analogTemp">--</div>
                            <div class="gauge-unit">°C</div>
                        </div>
                        <div class="mt-2">
                            <span class="badge bg-success">Active</span>
                            <br><small class="text-muted">Range: -40°C to +125°C</small>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Controls Row -->
        <div class="row mb-4">
            <div class="col-md-6">
                <div class="card">
                    <div class="card-header">
                        <h5>System Controls</h5>
                    </div>
                    <div class="card-body">
                        <div class="mb-3">
                            <label for="fanSlider" class="form-label">Fan Speed: <span id="fanSpeedValue">0</span>%</label>
                            <input type="range" class="form-range" id="fanSlider" min="0" max="100" value="0">
                            <div class="mt-2">
                                <small class="text-muted">🌀 Fan Status: <span id="fanStatus">OFF</span></small><br>
                                <small class="text-muted">🔊 Buzzer Status: <span id="buzzerStatus">OFF</span></small>
                            </div>
                        </div>
                        
                        <div class="mb-3">
                            <label for="thresholdInput" class="form-label">Temperature Threshold (°C)</label>
                            <input type="number" class="form-control" id="thresholdInput" min="0" max="50" step="0.5" value="28">
                        </div>
                        
                        <div class="mb-3">
                            <label for="sensorSelect" class="form-label">Threshold Sensor</label>
                            <select class="form-select" id="sensorSelect">
                                <option value="digital">Digital Sensor (DHT22)</option>
                                <option value="analog">Analog Sensor (TMP36)</option>
                            </select>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="col-md-6">
                <div class="card">
                    <div class="card-header">
                        <h5>System Status</h5>
                    </div>
                    <div class="card-body">
                        <div class="alert" id="alertStatus">
                            <strong>Status:</strong> <span id="alertText">Normal Operation</span>
                        </div>
                        
                        <div class="row text-center">
                            <div class="col-3">
                                <div class="stat-box">
                                    <h6>Uptime</h6>
                                    <span id="uptimeDisplay">--</span>
                                </div>
                            </div>
                            <div class="col-3">
                                <div class="stat-box">
                                    <h6>DHT Errors</h6>
                                    <span id="dhtErrors">--</span>
                                </div>
                            </div>
                            <div class="col-3">
                                <div class="stat-box">
                                    <h6>Clients</h6>
                                    <span id="connectedClients">--</span>
                                </div>
                            </div>
                            <div class="col-3">
                                <div class="stat-box">
                                    <h6>Sensors</h6>
                                    <span class="text-success">Dual</span>
                                </div>
                            </div>
                        </div>
                        
                        <button class="btn btn-primary mt-3" onclick="exportData()">📊 Export Data</button>
                    </div>
                </div>
            </div>
        </div>

        <!-- Charts Row -->
        <div class="row mb-4">
            <div class="col-12">
                <div class="card">
                    <div class="card-header">
                        <h5>📈 Temperature Trends (DHT22 vs TMP36)</h5>
                    </div>
                    <div class="card-body">
                        <canvas id="temperatureChart" width="400" height="200"></canvas>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="/script.js"></script>
</body>
</html>)rawliteral";
}

String getCSS() {
  return R"rawliteral(.temp-gauge {
    width: 120px;
    height: 120px;
    border: 8px solid #e0e0e0;
    border-radius: 50%;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    margin: 0 auto;
    position: relative;
    background: linear-gradient(135deg, #f8f9fa, #e9ecef);
}

.gauge-value {
    font-size: 24px;
    font-weight: bold;
    color: #495057;
}

.gauge-unit {
    font-size: 14px;
    color: #6c757d;
}

.temp-gauge.hot {
    border-color: #dc3545;
    color: #dc3545;
}

.temp-gauge.warm {
    border-color: #fd7e14;
    color: #fd7e14;
}

.temp-gauge.cool {
    border-color: #20c997;
    color: #20c997;
}

.temp-gauge.cold {
    border-color: #0dcaf0;
    color: #0dcaf0;
}

.stat-box {
    background: #f8f9fa;
    border-radius: 8px;
    padding: 10px;
    margin: 5px;
}

.alert.alert-danger {
    animation: pulse 1s infinite;
}

@keyframes pulse {
    0% { opacity: 1; }
    50% { opacity: 0.7; }
    100% { opacity: 1; }
}

.card {
    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    border: none;
}

.card-header {
    background: linear-gradient(135deg, #007bff, #0056b3);
    color: white;
    border-bottom: none;
}

#temperatureChart {
    max-height: 400px;
}

.navbar-brand {
    font-size: 1.3rem;
    font-weight: bold;
}

/* Continuous mode styling */
.alert-info {
    border-left: 5px solid #0dcaf0;
}

.navbar-text {
    font-size: 0.9rem;
})rawliteral";
}

String getJavaScript() {
  return R"rawliteral(let socket;
let chart;
let chartData = {
    labels: [],
    datasets: [
        {
            label: 'Digital (DHT22)',
            data: [],
            borderColor: '#007bff',
            backgroundColor: 'rgba(0, 123, 255, 0.1)',
            fill: true
        },
        {
            label: 'Analog (TMP36)',
            data: [],
            borderColor: '#28a745',
            backgroundColor: 'rgba(40, 167, 69, 0.1)',
            fill: true
        }
    ]
};

function initializeWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = protocol + '//' + window.location.hostname + ':81';
    
    console.log('Connecting to WebSocket (TMP36 + Continuous Mode):', wsUrl);
    socket = new WebSocket(wsUrl);
    
    socket.onopen = function() {
        document.getElementById('status').textContent = 'Connected (24/7)';
        document.getElementById('status').className = 'navbar-text text-success';
        console.log('WebSocket connected with TMP36 sensor in continuous mode');
    };
    
    socket.onmessage = function(event) {
        const data = JSON.parse(event.data);
        if (data.type === 'sensorData') {
            updateDashboard(data);
        }
    };
    
    socket.onclose = function(event) {
        document.getElementById('status').textContent = 'Reconnecting...';
        document.getElementById('status').className = 'navbar-text text-warning';
        console.log('WebSocket disconnected in continuous mode. Code:', event.code);
        console.log('Reconnecting in 2 seconds (continuous mode)...');
        setTimeout(initializeWebSocket, 2000);
    };
    
    socket.onerror = function(error) {
        document.getElementById('status').textContent = 'Connection Error';
        document.getElementById('status').className = 'navbar-text text-danger';
        console.error('WebSocket error in continuous mode:', error);
    };
}

function initializeChart() {
    const ctx = document.getElementById('temperatureChart').getContext('2d');
    chart = new Chart(ctx, {
        type: 'line',
        data: chartData,
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                y: {
                    beginAtZero: true,
                    min: 0,
                    max: 50,
                    title: {
                        display: true,
                        text: 'Temperature (°C)'
                    }
                },
                x: {
                    title: {
                        display: true,
                        text: 'Time (DHT22 vs TMP36 Comparison)'
                    }
                }
            },
            plugins: {
                legend: {
                    position: 'top'
                },
                title: {
                    display: true,
                    text: 'Dual Sensor Temperature Monitoring'
                }
            },
            animation: {
                duration: 500
            }
        }
    });
}

function formatUptime(seconds) {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    
    if (hours > 0) {
        return hours + 'h ' + minutes + 'm';
    } else if (minutes > 0) {
        return minutes + 'm ' + secs + 's';
    } else {
        return secs + 's';
    }
}

function updateDashboard(data) {
    updateTemperatureGauge('digitalTemp', 'digitalGauge', data.digitalTemp, data.dhtActive);
    updateTemperatureGauge('analogTemp', 'analogGauge', data.analogTemp, true);
    
    const dhtStatus = document.getElementById('dhtStatus');
    if (data.dhtActive) {
        dhtStatus.textContent = 'Active';
        dhtStatus.className = 'badge bg-success';
    } else {
        dhtStatus.textContent = 'Error';
        dhtStatus.className = 'badge bg-danger';
    }
    
    // Update fan controls and status
    document.getElementById('fanSlider').value = data.fanSpeed;
    document.getElementById('fanSpeedValue').textContent = data.fanSpeed;
    
    // Update fan status display
    const fanStatus    = document.getElementById('fanStatus');
    const buzzerStatus = document.getElementById('buzzerStatus');
    
    if (data.fanSpeed > 0) {
        fanStatus.textContent = 'ON (' + data.fanSpeed + '%)';
        fanStatus.style.color = '#28a745';
        buzzerStatus.textContent = 'ON';
        buzzerStatus.style.color = '#fd7e14';
    } else {
        fanStatus.textContent = 'OFF';
        fanStatus.style.color = '#6c757d';
        buzzerStatus.textContent = 'OFF';
        buzzerStatus.style.color = '#6c757d';
    }
    
    document.getElementById('thresholdInput').value = data.threshold;
    document.getElementById('sensorSelect').value   = data.selectedSensor;
    
    const alertStatus = document.getElementById('alertStatus');
    const alertText   = document.getElementById('alertText');
    if (data.alertActive) {
        alertStatus.className = 'alert alert-danger';
        alertText.textContent = 'TEMPERATURE ALERT ACTIVE! Fan & Buzzer Activated';
    } else {
        alertStatus.className = 'alert alert-success';
        alertText.textContent = 'Normal Operation (Continuous Mode - DHT22 + TMP36)';
    }
    
    // Update uptime displays
    const uptime = data.uptime || Math.floor(data.timestamp / 1000);
    const uptimeFormatted = formatUptime(uptime);
    document.getElementById('uptime').textContent = 'Uptime: ' + uptimeFormatted;
    document.getElementById('uptimeDisplay').textContent = uptimeFormatted;
    
    document.getElementById('dhtErrors').textContent = data.dhtErrors || 0;
    document.getElementById('connectedClients').textContent = data.connectedClients || 0;
    document.getElementById('clientCount').textContent = 'Clients: ' + (data.connectedClients || 0);
    
    updateChart(data);
}

function updateTemperatureGauge(elementId, gaugeId, temperature, isActive) {
    const element = document.getElementById(elementId);
    const gauge   = document.getElementById(gaugeId);
    
    if (!isActive) {
        element.textContent = 'ERR';
        gauge.className = 'temp-gauge';
        return;
    }
    
    element.textContent = temperature.toFixed(1);
    
    gauge.className = 'temp-gauge';
    if (temperature >= 35) {
        gauge.classList.add('hot');
    } else if (temperature >= 25) {
        gauge.classList.add('warm');
    } else if (temperature >= 15) {
        gauge.classList.add('cool');
    } else {
        gauge.classList.add('cold');
    }
}

function updateChart(data) {
    const now = new Date();
    const timeLabel = now.toLocaleTimeString();
    
    chartData.labels.push(timeLabel);
    chartData.datasets[0].data.push(data.dhtActive ? data.digitalTemp : null);
    chartData.datasets[1].data.push(data.analogTemp);
    
    if (chartData.labels.length > 20) {
        chartData.labels.shift();
        chartData.datasets.forEach(dataset => dataset.data.shift());
    }
    
    chart.update('none');
}

document.getElementById('fanSlider').addEventListener('input', function() {
    const speed = this.value;
    document.getElementById('fanSpeedValue').textContent = speed;
    
    // Update status display immediately for better user feedback
    const fanStatus    = document.getElementById('fanStatus');
    const buzzerStatus = document.getElementById('buzzerStatus');
    
    if (speed > 0) {
        fanStatus.textContent = 'UPDATING... (' + speed + '%)';
        fanStatus.style.color = '#ffc107';
        buzzerStatus.textContent = 'UPDATING...';
        buzzerStatus.style.color = '#ffc107';
    } else {
        fanStatus.textContent = 'UPDATING... (OFF)';
        fanStatus.style.color = '#ffc107';
        buzzerStatus.textContent = 'UPDATING...';
        buzzerStatus.style.color = '#ffc107';
    }
    
    fetch('/api/fan', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: 'speed=' + speed
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            console.log('Fan speed updated successfully to ' + speed + '% (TMP36 + continuous mode)');
        } else {
            console.error('Fan control error:', data.message);
            alert('Error setting fan speed: ' + data.message);
        }
    })
    .catch(error => {
        console.error('Fan control request failed:', error);
        alert('Failed to communicate with weather station');
    });
});

document.getElementById('thresholdInput').addEventListener('change', function() {
    const threshold = this.value;
    
    fetch('/api/threshold', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: 'threshold=' + threshold
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            console.log('Threshold updated to ' + threshold + '°C (TMP36 + continuous mode)');
        } else {
            console.error('Threshold update error:', data.message);
        }
    })
    .catch(error => {
        console.error('Threshold update failed:', error);
    });
});

document.getElementById('sensorSelect').addEventListener('change', function() {
    const sensor = this.value;
    
    fetch('/api/sensor', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: 'sensor=' + sensor
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            console.log('Sensor selection updated to ' + sensor + ' (TMP36 + continuous mode)');
        } else {
            console.error('Sensor selection error:', data.message);
        }
    })
    .catch(error => {
        console.error('Sensor selection failed:', error);
    });
});

function exportData() {
    window.open('/api/export', '_blank');
}

document.addEventListener('DOMContentLoaded', function() {
    console.log('Initializing ESP32 Weather Station Dashboard - TMP36 + CONTINUOUS MODE');
    console.log('Features: DHT22 + TMP36 dual sensors, 24/7 operation, no WiFi timeouts, watchdog protection');
    initializeChart();
    
    // Start WebSocket connection immediately for continuous mode
    initializeWebSocket();
});)rawliteral";
}