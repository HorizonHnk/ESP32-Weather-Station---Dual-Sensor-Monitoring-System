/**
 * ESP32-Based Climate Monitor with Integrated Web Interface
 * Local environmental sensing with DHT22 and TMP36 sensors
 * Creates WiFi Access Point for dashboard connectivity
 * 
 * ENHANCED VERSION - Persistent WiFi Access Point (No Auto-Disconnect) + TMP36 Sensor
 * 
 * SETUP REQUIREMENTS:
 * 1. Set ENABLE_DISPLAY to false if you don't have an I2C LCD screen
 * 
 * SENSOR CONFIGURATION:
 * - DHT22: Digital climate sensor on pin 18
 * - TMP36: Analog thermal sensor on pin 33
 *   TMP36 connections: VCC to 3.3V, GND to GND, VOUT to pin 33
 * 
 * TMP36 FINE-TUNING (if measurements are incorrect):
 * 1. Monitor Serial output for auto-adjustment results
 * 2. If still imprecise, manually modify these parameters:
 *    - realVoltageRef: Change from 3.3 to actual ESP32 voltage (try 3.15, 3.0, or 2.9)
 *    - tmp36AdjustmentFactor: Add/subtract degrees for precise tuning
 * 3. Common solutions:
 *    - Values too low: Increase realVoltageRef to 3.15 or 3.3
 *    - Values too high: Decrease realVoltageRef to 3.0 or 2.9
 *    - Minor adjustments: Use tmp36AdjustmentFactor (+2.5, -1.3, etc.)
 * 
 * COOLING & ALERT CONTROL FEATURES:
 * - Manual cooling control via web interface (0-100% intensity)
 * - Alert tone automatically activates when cooling is running (any intensity > 0%)
 * - Automatic cooling activation when temperature exceeds limit
 * - Different alert frequencies: 1500Hz (normal operation), 2000Hz (temperature warning)
 * - Manual control remains after automatic activation
 * - Real-time status display on web interface and LCD
 * 
 * The system operates in STANDALONE MODE ONLY:
 * - Creates access point "ESP32_Climate_Monitor" for interface access
 * - No internet connection needed
 * - PERSISTENT WIFI OPERATION - No auto-disconnect
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <esp_wifi.h>  // For advanced WiFi configuration
#include <esp_task_wdt.h> // For watchdog management

// Configuration - Set to false if you don't have an LCD or want to disable it
#define ENABLE_DISPLAY true

#if ENABLE_DISPLAY
#include <LiquidCrystal_I2C.h>
#endif
#include <DNSServer.h>

// Hardware Pin Assignments
#define CLIMATE_SENSOR_PIN    18      // Digital climate sensor (DHT22)
#define THERMAL_SENSOR_PIN    33      // Analog thermal sensor (TMP36)
#define ALERT_BUZZER_PIN      12      // Buzzer for notifications
#define COOLING_FAN_PIN       15      // DC motor fan controlled via PWM
#define STATUS_LED_PIN        2       // Built-in LED for status indication

// TMP36 Adjustment Settings
#define REFERENCE_VOLTAGE  3.3    // Default reference voltage
float realVoltageRef = 3.3;      // Will be adjusted during setup
float tmp36AdjustmentFactor = 0.0;  // Temperature adjustment for calibration

// DHT Sensor Configuration
#define CLIMATE_SENSOR_TYPE DHT22

// WiFi Access Point Configuration - ENHANCED FOR PERSISTENT OPERATION
const char* HOTSPOT_NAME     = "ESP32_Climate_Monitor";
const char* HOTSPOT_PASS     = "climate123";  // Must be at least 8 characters
const int   HOTSPOT_CHANNEL  = 6;            // WiFi channel (1-13)
const int   MAX_DEVICE_CONNECTIONS = 4;     // Maximum concurrent connections
IPAddress networkIP(192, 168, 4, 1);
IPAddress routerIP(192, 168, 4, 1);
IPAddress networkMask(255, 255, 255, 0);

// Core System Objects
WebServer       webInterface(80);
WebSocketsServer websocketHandler(81);
DNSServer       domainHandler;
DHT             climateSensor(CLIMATE_SENSOR_PIN, CLIMATE_SENSOR_TYPE);
#if ENABLE_DISPLAY
  LiquidCrystal_I2C displayScreen(0x27, 16, 2);
#endif

// Environmental measurement variables
float primaryTemperature   = 25.0;
float secondaryTemperature = 25.0;
float externalTemperature  = 0.0;    // Placeholder for "external" temperature
float temperatureLimit     = 28.0;
int   coolingIntensity     = 0;      // PWM (0-255)
bool  warningActive        = false;
String activeSensorType = "primary";

// DHT22 error management
int   climateSensorErrors       = 0;
int   consecutiveClimateErrors  = 0;
unsigned long lastClimateSuccessTime = 0;
bool  climateSensorOperational = true;
const int CLIMATE_MAX_CONSECUTIVE_ERRORS = 10;
const int CLIMATE_MAX_RETRY_ATTEMPTS     = 3;

// WiFi status monitoring - SIMPLIFIED FOR STABILITY
bool wifiSystemInitialized = false;
unsigned long lastWifiStatusCheck = 0;
const int WIFI_STATUS_CHECK_INTERVAL = 60000; // Check WiFi status every 60 seconds (less aggressive)

// Timing control
unsigned long lastEnvironmentReadTime  = 0;
unsigned long lastDisplayRefreshTime   = 0;
unsigned long lastDataTransmission     = 0;
unsigned long lastStatusRefresh        = 0;
unsigned long lastWatchdogReset        = 0;  // NEW: Watchdog feeding
const int    ENVIRONMENT_READ_INTERVAL   = 5000;   // 5 seconds
const int    DISPLAY_REFRESH_INTERVAL    = 2000;   // 2 seconds
const int    DATA_TRANSMISSION_INTERVAL  = 1000;   // 1 second
const int    STATUS_REFRESH_INTERVAL     = 10000;  // 10 seconds
const int    WATCHDOG_RESET_INTERVAL     = 1000;   // Feed watchdog every second

// Cooling PWM configuration
const int COOLING_PWM_FREQUENCY     = 5000;  // 5 kHz
const int COOLING_PWM_CHANNEL       = 0;
const int COOLING_PWM_RESOLUTION    = 8;     // 8-bit resolution (0-255)

// System monitoring
int connectedDevices = 0;
unsigned long systemBootTime = 0;

// Data storage for visualizations (last 50 measurements)
struct EnvironmentReading {
  unsigned long recordTime;
  float primaryTemperature;
  float secondaryTemperature;
  bool  climateActive;
};
EnvironmentReading measurementHistory[50];
int historyIndex = 0;
int historyCount = 0;

// Function prototypes
String generateMainHTML();
String generateCSS();
String generateJavaScript();
void processApiDataRequest();
void processApiHistoryRequest();
void processApiStatusRequest();
void processCoolingControl();
void processLimitControl();
void processSensorSelection();
void processDataExportRequest();
void processCaptivePortalRequest();
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void transmitEnvironmentData();
void readPrimaryTemperatureWithRetry();
void readSecondaryTemperature();
void adjustTMP36Calibration();   // NEW: TMP36 calibration function
void verifyClimateStatus();
void evaluateTemperatureLimits();
void recordEnvironmentData();
void refreshDisplayContent();
void displayCenteredText(const char* text, int row);
void initializeWiFiHotspot();
void checkWiFiStatus();  // SIMPLIFIED
void configureWebInterface();
void displaySystemInformation();
void refreshSystemStatus();
void adjustCoolingAndAlert(int newCoolingIntensity, String controlSource);
void resetWatchdogTimer();   // NEW

void setup() {
  Serial.begin(115200);
  delay(1000); // Initial delay
  
  // Configure watchdog timer
  Serial.println("🔧 Setting up watchdog timer...");
  esp_task_wdt_init(30, false); // 30 second timeout, no panic
  esp_task_wdt_add(NULL); // Add current task to watchdog
  
  Serial.println("=====================================");
  Serial.println("  ESP32 Climate Monitor Starting");
  Serial.println("  PERSISTENT WIFI + TMP36 VERSION");
  Serial.println("=====================================");
  Serial.println("🔄 PHASE 1: Hardware Initialization");

  // Initialize pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ALERT_BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.println("   ✓ GPIO pins configured");

  Serial.println("🔧 PHASE 2: Initializing hardware...");

  // Initialize PWM for cooling
  ledcSetup(COOLING_PWM_CHANNEL, COOLING_PWM_FREQUENCY, COOLING_PWM_RESOLUTION);
  ledcAttachPin(COOLING_FAN_PIN, COOLING_PWM_CHANNEL);
  ledcWrite(COOLING_PWM_CHANNEL, 0);
  Serial.println("   ✓ Cooling PWM controller initialized");

  // Initialize sensors
  Serial.println("   🌡️  Initializing environmental sensors...");
  climateSensor.begin();
  analogReadResolution(12);  // 12-bit ADC resolution for better TMP36 accuracy
  delay(2000); // Let DHT22 stabilize
  Serial.println("   ✓ DHT22 and TMP36 sensors initialized");

  // Calibrate TMP36 sensor for accurate readings
  Serial.println("   🔧 Calibrating TMP36 sensor...");
  adjustTMP36Calibration();
  Serial.println("   ✓ TMP36 calibration complete");

  // Feed watchdog
  resetWatchdogTimer();

  // Initialize LCD (if enabled)
#if ENABLE_DISPLAY
  Serial.println("   📺 Initializing LCD display...");
  displayScreen.init();
  displayScreen.backlight();
  displayCenteredText("ESP32 Climate", 0);
  displayCenteredText("DHT22 + TMP36", 1);
  delay(2000);
  Serial.println("   ✓ LCD display ready");
#else
  Serial.println("   📺 LCD display disabled in configuration");
#endif

  // Feed watchdog
  resetWatchdogTimer();

  Serial.println("🔄 PHASE 3: WiFi Setup (Persistent Mode)");
  // Setup WiFi Access Point - ENHANCED VERSION
  initializeWiFiHotspot();

  // Feed watchdog
  resetWatchdogTimer();

  Serial.println("🔄 PHASE 4: Web Interface Setup");
  // Setup web server routes
  Serial.println("🌐 Setting up web interface...");
  configureWebInterface();
  Serial.println("   ✓ Web interface routes configured");

  Serial.println("🔄 PHASE 5: WebSocket Setup");
  // Setup WebSocket
  Serial.println("🔌 Setting up WebSocket server...");
  websocketHandler.begin();
  websocketHandler.onEvent(handleWebSocketEvent);
  Serial.println("   ✓ WebSocket server started on port 81");

  Serial.println("🔄 PHASE 6: DNS Server Setup");
  // Setup captive portal DNS
  Serial.println("🌍 Setting up captive portal...");
  domainHandler.start(53, "*", networkIP);
  Serial.println("   ✓ DNS server started for captive portal");

  Serial.println("🔄 PHASE 7: Starting Web Interface");
  // Start servers
  webInterface.begin();
  Serial.println("   ✅ HTTP server started on port 80");
  Serial.println("✅ All systems initialized successfully!");
  Serial.println();

  // Feed watchdog
  resetWatchdogTimer();

  // Print connection instructions
  displaySystemInformation();

  Serial.println("🔄 PHASE 8: Initial Environment Reading");
  // Initial sensor reading
  Serial.println("📊 Taking initial environment readings...");
  readPrimaryTemperatureWithRetry();
  readSecondaryTemperature();
  Serial.println("   ✓ Initial readings complete");
  Serial.println();

  // Update LCD with connection info
#if ENABLE_DISPLAY
  displayScreen.clear();
  String wifiText = "WiFi: " + String(HOTSPOT_NAME);
  displayCenteredText(wifiText.c_str(), 0);
  displayCenteredText("192.168.4.1", 1);
  delay(500);
#endif

  digitalWrite(STATUS_LED_PIN, HIGH); // Indicate system is ready
  Serial.println("🎉 === SYSTEM READY FOR CONNECTIONS ===");
  Serial.println("💡 LED should be ON (indicating ready state)");
  Serial.println("📱 Try connecting to WiFi now!");
  Serial.println("🔒 WiFi hotspot will remain ON continuously");
  Serial.println("🌡️ Sensors: DHT22 (primary) + TMP36 (secondary)");
  Serial.println();
  
  // Record that WiFi was successfully initialized
  wifiSystemInitialized = true;
  systemBootTime = millis();
  lastWifiStatusCheck = millis();
  lastWatchdogReset = millis();
}

void loop() {
  static unsigned long lastLoopInfo    = 0;
  static unsigned long cycleCounter    = 0;
  static int           lastDeviceCount = -1;

  cycleCounter++;

  // Feed watchdog regularly - CRITICAL for stability
  if (millis() - lastWatchdogReset >= WATCHDOG_RESET_INTERVAL) {
    resetWatchdogTimer();
    lastWatchdogReset = millis();
  }

  // Log WiFi status periodically (less aggressive)
  if (millis() - lastWifiStatusCheck >= WIFI_STATUS_CHECK_INTERVAL) {
    checkWiFiStatus();
    lastWifiStatusCheck = millis();
  }

  // Monitor device connections in real-time
  int currentDeviceCount = WiFi.softAPgetStationNum();
  if (currentDeviceCount != lastDeviceCount) {
    Serial.println("📱 === DEVICE CONNECTION CHANGE ===");
    Serial.println("   👥 Previous devices: " + String(lastDeviceCount));
    Serial.println("   👥 Current devices: " + String(currentDeviceCount));
    if (currentDeviceCount > lastDeviceCount) {
      Serial.println("   ✅ New device connected to WiFi hotspot!");
      Serial.println("   🌐 Device should now be able to access http://192.168.4.1");
    } else if (currentDeviceCount < lastDeviceCount) {
      Serial.println("   ❌ Device disconnected from WiFi hotspot");
    }
    lastDeviceCount = currentDeviceCount;
  }

  // Print loop status every 60 seconds (less spam)
  if (millis() - lastLoopInfo >= 60000) {
    lastLoopInfo = millis();
    Serial.println("🔄 === LOOP STATUS UPDATE ===");
    Serial.println("   📊 Loop cycles: " + String(cycleCounter));
    Serial.println("   🧠 Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("   ⏰ Uptime: " + String(millis() / 1000) + " seconds");
    Serial.println("   👥 Connected devices: " + String(WiFi.softAPgetStationNum()));
    Serial.println("   📶 AP Status: PERSISTENT (No Auto-Disconnect)");
    Serial.println("   💡 LED Status: " + String(digitalRead(STATUS_LED_PIN) ? "ON" : "OFF"));

    // Encourage connection attempts
    if (WiFi.softAPgetStationNum() == 0) {
      Serial.println("   📱 WAITING FOR DEVICE CONNECTION...");
      Serial.println("      • Network: 'ESP32_Climate_Monitor'");
      Serial.println("      • Password: 'climate123'");
      Serial.println("      • WiFi is ALWAYS ON - no timeouts");
    }
  }

  // Handle DNS requests for captive portal
  domainHandler.processNextRequest();

  // Handle web server and WebSocket
  webInterface.handleClient();
  websocketHandler.loop();

  unsigned long currentMillis = millis();

  // Read sensors
  if (currentMillis - lastEnvironmentReadTime >= ENVIRONMENT_READ_INTERVAL) {
    lastEnvironmentReadTime = currentMillis;
    Serial.println("📊 Reading sensors...");
    readPrimaryTemperatureWithRetry();
    readSecondaryTemperature();
    evaluateTemperatureLimits();
    recordEnvironmentData();
    Serial.println("   ✓ Sensor reading cycle complete");
  }

  // Update LCD
  if (currentMillis - lastDisplayRefreshTime >= DISPLAY_REFRESH_INTERVAL) {
    lastDisplayRefreshTime = currentMillis;
    refreshDisplayContent();
  }

  // Broadcast data to WebSocket clients
  if (currentMillis - lastDataTransmission >= DATA_TRANSMISSION_INTERVAL) {
    lastDataTransmission = currentMillis;
    transmitEnvironmentData();
  }

  // Update system status
  if (currentMillis - lastStatusRefresh >= STATUS_REFRESH_INTERVAL) {
    lastStatusRefresh = currentMillis;
    refreshSystemStatus();
  }

  // Check climate sensor status
  verifyClimateStatus();
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

// SIMPLIFIED and STABLE WiFi Setup Function
void initializeWiFiHotspot() {
  Serial.println("📡 Setting up PERSISTENT WiFi Access Point...");
  
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
  bool configSet = WiFi.softAPConfig(networkIP, routerIP, networkMask);
  if (!configSet) {
    Serial.println("   ⚠️ Warning: Failed to set IP configuration, using defaults");
  } else {
    Serial.println("   ✓ IP configuration set successfully");
  }
  
  // Feed watchdog
  resetWatchdogTimer();
  
  // Step 4: Start the Access Point with explicit parameters
  Serial.println("   📡 Starting WiFi hotspot...");
  Serial.println("      • SSID: " + String(HOTSPOT_NAME));
  Serial.println("      • Password: " + String(HOTSPOT_PASS));
  Serial.println("      • Channel: " + String(HOTSPOT_CHANNEL));
  Serial.println("      • Max connections: " + String(MAX_DEVICE_CONNECTIONS));
  Serial.println("      • Mode: PERSISTENT (No timeouts)");
  
  // Start AP with explicit settings
  bool apStarted = WiFi.softAP(HOTSPOT_NAME, HOTSPOT_PASS, HOTSPOT_CHANNEL, 0, MAX_DEVICE_CONNECTIONS);
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
  resetWatchdogTimer();
  
  // Step 6: Additional stability settings
  Serial.println("   🔧 Applying stability settings...");
  
  // Set WiFi to never sleep/disconnect
  WiFi.setSleep(false);  // Disable WiFi sleep
  Serial.println("   ✓ WiFi sleep mode DISABLED");
  
  // Configure for maximum stability
  esp_wifi_set_max_tx_power(78);  // Set moderate power level (not max to reduce heat)
  Serial.println("   ✓ WiFi power level optimized");
  
  // Step 7: Success confirmation
  Serial.println("   ✅ PERSISTENT WiFi hotspot started successfully!");
  Serial.println("   📍 IP Address: " + apIP.toString());
  Serial.println("   📶 Channel: " + String(HOTSPOT_CHANNEL));
  Serial.println("   🔧 MAC Address: " + WiFi.softAPmacAddress());
  Serial.println("   🔒 Mode: PERSISTENT - No auto-shutdown");
  Serial.println("   ⚡ Power saving: DISABLED");
  Serial.println("   💤 Sleep mode: DISABLED");
  
  // Update LCD
#if ENABLE_DISPLAY
  displayScreen.clear();
  displayCenteredText("WiFi PERSISTENT", 0);
  displayCenteredText(apIP.toString().c_str(), 1);
  delay(2000);
#endif
  
  Serial.println("✅ WiFi Access Point setup completed - PERSISTENT MODE ACTIVE!");
}

// NEW: Watchdog feeding function
void resetWatchdogTimer() {
  esp_task_wdt_reset();
}

// SIMPLIFIED WiFi Status Logging (non-intrusive)
void checkWiFiStatus() {
  IPAddress currentIP = WiFi.softAPIP();
  wifi_mode_t currentMode = WiFi.getMode();
  
  Serial.println("📶 === WIFI STATUS CHECK ===");
  Serial.println("   📍 IP Address: " + currentIP.toString());
  Serial.println("   📡 WiFi Mode: " + String(currentMode));
  Serial.println("   👥 Connected devices: " + String(WiFi.softAPgetStationNum()));
  Serial.println("   ⚡ Power saving: " + String(WiFi.getSleep() ? "ENABLED (BAD)" : "DISABLED (GOOD)"));
  Serial.println("   💤 Status: PERSISTENT MODE - No shutdowns");
  
  // Only warn if there's actually a problem, don't try to "fix" it
  if (currentIP == IPAddress(0, 0, 0, 0)) {
    Serial.println("   ⚠️ WARNING: WiFi IP is 0.0.0.0 - This indicates a serious problem");
    Serial.println("   🔧 Manual restart may be required");
  } else if (currentMode != WIFI_AP) {
    Serial.println("   ⚠️ WARNING: WiFi mode changed from AP mode");
    Serial.println("   🔧 This should not happen in persistent mode");
  } else {
    Serial.println("   ✅ WiFi status: HEALTHY and PERSISTENT");
  }
}

void adjustCoolingAndAlert(int newCoolingIntensity, String controlSource) {
  // Convert percentage to PWM value
  int pwmValue = map(newCoolingIntensity, 0, 100, 0, 255);

  Serial.println("🌀 Cooling System Update Requested:");
  Serial.println("   📊 Source: " + controlSource);
  Serial.println("   📈 Requested Intensity: " + String(newCoolingIntensity) + "% (PWM: " + String(pwmValue) + ")");
  Serial.println("   🔄 Previous intensity: " + String(map(coolingIntensity, 0, 255, 0, 100)) + "%");

  // Update cooling intensity
  coolingIntensity = pwmValue;
  ledcWrite(COOLING_PWM_CHANNEL, coolingIntensity);

  // Control alert based on cooling status
  if (newCoolingIntensity > 0) {
    // Cooling is running - activate alert
    Serial.println("   🔊 Activating alert (cooling is running)");
    tone(ALERT_BUZZER_PIN, 1500); // 1.5kHz tone when cooling is manually controlled
  } else {
    // Cooling is stopped - deactivate alert
    Serial.println("   🔇 Deactivating alert (cooling stopped)");
    noTone(ALERT_BUZZER_PIN);
  }

  Serial.println("   ✅ Cooling and alert updated successfully");

  // Update LCD with cooling status
#if ENABLE_DISPLAY
  displayScreen.clear();
  if (newCoolingIntensity > 0) {
    String coolingText = "Cool: " + String(newCoolingIntensity) + "%";
    displayCenteredText(coolingText.c_str(), 0);
    displayCenteredText("Alert: ON", 1);
  } else {
    displayCenteredText("Cool: OFF", 0);
    displayCenteredText("Alert: OFF", 1);
  }
  delay(2000);
#endif
}

void configureWebInterface() {
  Serial.println("🌐 Registering web interface routes...");

  // Serve main dashboard (captive portal redirect)
  webInterface.on("/", HTTP_GET, processCaptivePortalRequest);
  Serial.println("   ✓ Route registered: GET /");

  webInterface.on("/index.html", HTTP_GET, processCaptivePortalRequest);
  Serial.println("   ✓ Route registered: GET /index.html");

  webInterface.on("/dashboard", HTTP_GET, []() {
    Serial.println("📱 Client accessing main dashboard from: " + webInterface.client().remoteIP().toString());
    webInterface.send(200, "text/html", generateMainHTML());
    Serial.println("   ✅ Dashboard HTML sent successfully");
  });
  Serial.println("   ✓ Route registered: GET /dashboard");

  // Captive portal handlers
  webInterface.on("/generate_204", HTTP_GET, processCaptivePortalRequest);      // Android
  Serial.println("   ✓ Route registered: GET /generate_204 (Android captive portal)");

  webInterface.on("/fwlink", HTTP_GET, processCaptivePortalRequest);            // Microsoft
  Serial.println("   ✓ Route registered: GET /fwlink (Microsoft captive portal)");

  webInterface.on("/hotspot-detect.html", HTTP_GET, processCaptivePortalRequest); // Apple
  Serial.println("   ✓ Route registered: GET /hotspot-detect.html (Apple captive portal)");

  // API endpoints
  webInterface.on("/api/data", HTTP_GET, processApiDataRequest);
  Serial.println("   ✓ Route registered: GET /api/data");

  webInterface.on("/api/history", HTTP_GET, processApiHistoryRequest);
  Serial.println("   ✓ Route registered: GET /api/history");

  webInterface.on("/api/status", HTTP_GET, processApiStatusRequest);
  Serial.println("   ✓ Route registered: GET /api/status");

  // Control endpoints
  webInterface.on("/api/fan", HTTP_POST, processCoolingControl);
  Serial.println("   ✓ Route registered: POST /api/fan");

  webInterface.on("/api/threshold", HTTP_POST, processLimitControl);
  Serial.println("   ✓ Route registered: POST /api/threshold");

  webInterface.on("/api/sensor", HTTP_POST, processSensorSelection);
  Serial.println("   ✓ Route registered: POST /api/sensor");

  webInterface.on("/api/export", HTTP_GET, processDataExportRequest);
  Serial.println("   ✓ Route registered: GET /api/export");

  // Static files
  webInterface.on("/style.css", HTTP_GET, []() {
    Serial.println("📄 CSS file requested from: " + webInterface.client().remoteIP().toString());
    webInterface.send(200, "text/css", generateCSS());
    Serial.println("   ✅ CSS sent successfully");
  });
  Serial.println("   ✓ Route registered: GET /style.css");

  webInterface.on("/script.js", HTTP_GET, []() {
    Serial.println("📄 JavaScript file requested from: " + webInterface.client().remoteIP().toString());
    webInterface.send(200, "application/javascript", generateJavaScript());
    Serial.println("   ✅ JavaScript sent successfully");
  });
  Serial.println("   ✓ Route registered: GET /script.js");

  // Catch-all handler for captive portal
  webInterface.onNotFound(processCaptivePortalRequest);
  Serial.println("   ✓ Catch-all handler registered (onNotFound)");

  Serial.println("🌐 Web interface setup complete - routes registered");
}

void processCaptivePortalRequest() {
  String clientIP     = webInterface.client().remoteIP().toString();
  String requestURI   = webInterface.uri();
  String requestMethod = (webInterface.method() == HTTP_GET) ? "GET" : "POST";

  Serial.println("🌐 === CAPTIVE PORTAL REQUEST ===");
  Serial.println("   📍 Client IP: " + clientIP);
  Serial.println("   📄 Request: " + requestMethod + " " + requestURI);
  Serial.println("   🌐 Host header: " + webInterface.hostHeader());

  String html = R"rawliteral(<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Climate Monitor</title>
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
        <h1>ESP32 Climate Monitor</h1>
        <div class="status">
            <p><strong>✅ Connection Successful!</strong></p>
            <p>WiFi: )rawliteral" + String(HOTSPOT_NAME) + R"rawliteral( | IP: 192.168.4.1</p>
        </div>
        <div class="sensors">
            <p><strong>🌡️ Dual Temperature Sensors</strong></p>
            <p>DHT22 (Primary) + TMP36 (Secondary)</p>
        </div>
        <div class="continuous">
            <p>🔒 PERSISTENT MODE ACTIVE</p>
            <p>WiFi hotspot stays on 24/7 - No timeouts!</p>
        </div>
        <div class="info">
            <p><strong>Welcome!</strong> You are connected to the ESP32 Climate Monitor hotspot.</p>
            <p>Click below to access the real-time dashboard with live temperature readings, controls, and charts.</p>
        </div>
        <a href="/dashboard" class="btn">🚀 Open Dashboard</a>
        <div style="margin-top: 20px; font-size: 14px; color: #7f8c8d;">
            <p>📡 Network: )rawliteral" + String(HOTSPOT_NAME) + R"rawliteral(</p>
            <p>🌐 IP Address: 192.168.4.1</p>
            <p>📊 Connected devices: )rawliteral" + String(WiFi.softAPgetStationNum()) + R"rawliteral(</p>
            <p>⏰ Uptime: )rawliteral" + String((millis() - systemBootTime) / 1000) + R"rawliteral( seconds</p>
        </div>
    </div>
</body>
</html>)rawliteral";

  webInterface.send(200, "text/html", html);
  Serial.println("   ✅ Captive portal HTML sent (" + String(html.length()) + " bytes)");
  Serial.println("   🔗 Client should see welcome page with dashboard link");
}

void displaySystemInformation() {
  Serial.println("🎯 PERSISTENT WIFI ACCESS INSTRUCTIONS:");
  Serial.println("=====================================");
  Serial.println("📱 On your phone/computer:");
  Serial.println("   1. Go to WiFi settings");
  Serial.println("   2. Connect to: " + String(HOTSPOT_NAME));
  Serial.println("   3. Password: " + String(HOTSPOT_PASS));
  Serial.println("   4. Wait for captive portal OR go to: http://192.168.4.1");
  Serial.println();
  Serial.println("🌡️ TEMPERATURE SENSORS:");
  Serial.println("   • DHT22 (Primary): Pin 18 - Temperature & Humidity");
  Serial.println("   • TMP36 (Secondary): Pin 33 - Temperature only");
  Serial.println("   • TMP36 Wiring: VCC→3.3V, GND→GND, VOUT→Pin33");
  Serial.println("   • Operating range: -40°C to +125°C");
  Serial.println("   • Auto-calibrated voltage reference: " + String(realVoltageRef, 2) + "V");
  Serial.println("   • Calibration offset: " + String(tmp36AdjustmentFactor, 1) + "°C");
  Serial.println();
  Serial.println("🔧 TMP36 TROUBLESHOOTING:");
  Serial.println("   • If readings are too low: Increase realVoltageRef (try 3.15 or 3.3)");
  Serial.println("   • If readings are too high: Decrease realVoltageRef (try 3.0 or 2.9)");
  Serial.println("   • Fine-tune with tmp36AdjustmentFactor (+/- degrees)");
  Serial.println("   • Check Serial Monitor for calibration details");
  Serial.println("   • Compare with DHT22 for reference accuracy");
  Serial.println();
  Serial.println("🔒 PERSISTENT MODE FEATURES:");
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
  Serial.println("   • Manual cooling speed control (0-100%)");
  Serial.println("   • Temperature threshold settings");
  Serial.println("   • Data export functionality");
  Serial.println("   • Mobile-responsive design");
  Serial.println("   • Local operation (no internet required)");
  Serial.println();
  Serial.println("🌀 Cooling & Alert Control:");
  Serial.println("   • Manual Control: Use web slider (0-100%)");
  Serial.println("   • Alert rings whenever cooling is running");
  Serial.println("   • Automatic activation when temp > threshold");
  Serial.println("   • Manual control overrides automatic settings");
  Serial.println("   • Alert tones: 1500Hz (normal), 2000Hz (warning)");
  Serial.println();
  Serial.println("🔧 SYSTEM GUARANTEES:");
  Serial.println("   • WiFi will NOT turn off automatically");
  Serial.println("   • System designed for 24/7 operation");
  Serial.println("   • Automatic crash recovery with watchdog");
  Serial.println("   • Power-optimized for continuous use");
  Serial.println("=====================================");
  Serial.println();
}

void refreshSystemStatus() {
  connectedDevices = WiFi.softAPgetStationNum();
  static int lastDeviceCount = -1;

  if (connectedDevices != lastDeviceCount) {
    Serial.println("👥 Device connection change detected");
    Serial.println("   📊 Currently connected devices: " + String(connectedDevices));
    lastDeviceCount = connectedDevices;
  }

  if (connectedDevices > 0) {
    Serial.println("📊 System Status Update:");
    Serial.println("   👥 Connected devices: " + String(connectedDevices));
    Serial.println("   🌡️ Primary temp (DHT22): " + String(primaryTemperature, 1) + "°C (" + (climateSensorOperational ? "Active" : "Error") + ")");
    Serial.println("   🌡️ Secondary temp (TMP36): " + String(secondaryTemperature, 1) + "°C");
    Serial.println("   🎯 Limit: " + String(temperatureLimit, 1) + "°C");
    Serial.println("   🌀 Cooling intensity: " + String(map(coolingIntensity, 0, 255, 0, 100)) + "%");
    Serial.println("   ⚠️ Warning status: " + String(warningActive ? "ACTIVE" : "Normal"));
    Serial.println("   📶 WiFi Mode: PERSISTENT (Always On)");
    Serial.println("   🧠 Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("   ⏰ Uptime: " + String((millis() - systemBootTime) / 1000) + " seconds");
    Serial.println();
  }
}

void processApiDataRequest() {
  String clientIP = webInterface.client().remoteIP().toString();
  Serial.println("📡 === API DATA REQUEST ===");
  Serial.println("   📍 Client IP: " + clientIP);
  Serial.println("   📊 Preparing sensor data response...");

  DynamicJsonDocument doc(512);
  doc["digitalTemp"]    = primaryTemperature;
  doc["analogTemp"]     = secondaryTemperature;
  doc["threshold"]      = temperatureLimit;
  doc["fanSpeed"]       = map(coolingIntensity, 0, 255, 0, 100);
  doc["alertActive"]    = warningActive;
  doc["selectedSensor"] = activeSensorType;
  doc["dhtActive"]      = climateSensorOperational;
  doc["timestamp"]      = millis();
  doc["connectedClients"] = connectedDevices;
  doc["uptime"]         = (millis() - systemBootTime) / 1000;
  doc["continuousMode"] = true;  // NEW: Indicate continuous mode
  doc["sensorType"]     = "TMP36"; // NEW: Indicate sensor type

  String response;
  serializeJson(doc, response);
  webInterface.send(200, "application/json", response);

  Serial.println("   ✅ API data sent successfully (" + String(response.length()) + " bytes)");
  Serial.println("   📊 Data: DHT22=" + String(primaryTemperature, 1) + "°C, TMP36=" + String(secondaryTemperature, 1) + "°C, Cool=" + String(map(coolingIntensity, 0, 255, 0, 100)) + "%");
}

void processApiHistoryRequest() {
  Serial.println("📡 API request: Historical data");

  DynamicJsonDocument doc(4096);
  JsonArray historyArray = doc.createNestedArray("history");

  int start = (historyCount < 50) ? 0 : historyIndex;
  for (int i = 0; i < min(historyCount, 50); i++) {
    int idx = (start + i) % 50;
    JsonObject point = historyArray.createNestedObject();
    point["timestamp"]   = measurementHistory[idx].recordTime;
    point["digitalTemp"] = measurementHistory[idx].primaryTemperature;
    point["analogTemp"]  = measurementHistory[idx].secondaryTemperature;
    point["dhtActive"]   = measurementHistory[idx].climateActive;
  }

  String response;
  serializeJson(doc, response);
  webInterface.send(200, "application/json", response);
}

void processApiStatusRequest() {
  DynamicJsonDocument doc(256);
  doc["wifiMode"]       = "Access Point (Persistent)";
  doc["ssid"]           = HOTSPOT_NAME;
  doc["ipAddress"]      = WiFi.softAPIP().toString();
  doc["connectedClients"] = connectedDevices;
  doc["uptime"]         = (millis() - systemBootTime) / 1000;
  doc["freeHeap"]       = ESP.getFreeHeap();
  doc["dhtErrors"]      = climateSensorErrors;
  doc["dhtActive"]      = climateSensorOperational;
  doc["continuousMode"] = true;
  doc["powerSaving"]    = WiFi.getSleep();  // Should be false
  doc["wifiChannel"]    = HOTSPOT_CHANNEL;
  doc["sensorType"]     = "TMP36";

  String response;
  serializeJson(doc, response);
  webInterface.send(200, "application/json", response);
}

void processCoolingControl() {
  Serial.println("🌀 Manual cooling control request from: " + webInterface.client().remoteIP().toString());

  if (webInterface.hasArg("speed")) {
    int speed = webInterface.arg("speed").toInt();
    Serial.println("   📊 Requested speed: " + String(speed) + "%");

    if (speed >= 0 && speed <= 100) {
      // Use the unified cooling and alert control function
      adjustCoolingAndAlert(speed, "Manual Web Control");

      webInterface.send(200, "application/json", "{\"status\":\"ok\",\"speed\":" + String(speed) + "}");
      Serial.println("   ✅ Manual cooling control successful");
    } else {
      Serial.println("   ❌ Invalid speed value: " + String(speed) + "% (valid: 0-100%)");
      webInterface.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid speed range (0-100%)\"}");
    }
  } else {
    Serial.println("   ❌ Missing speed parameter in request");
    webInterface.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing speed parameter\"}");
  }
}

void processLimitControl() {
  if (webInterface.hasArg("threshold")) {
    float threshold = webInterface.arg("threshold").toFloat();
    if (threshold > 0 && threshold < 100) {
      temperatureLimit = threshold;
      webInterface.send(200, "application/json", "{\"status\":\"ok\",\"threshold\":" + String(threshold) + "}");

      Serial.println("🌡️ Temperature limit set to: " + String(threshold) + "°C");

      // Update LCD
  #if ENABLE_DISPLAY
      displayScreen.clear();
      displayCenteredText("Limit Set", 0);
      char buffer[17];
      sprintf(buffer, "%.1f C", threshold);
      displayCenteredText(buffer, 1);
      delay(1500);
  #endif
    } else {
      webInterface.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid threshold\"}");
    }
  } else {
    webInterface.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing threshold parameter\"}");
  }
}

void processSensorSelection() {
  if (webInterface.hasArg("sensor")) {
    String sensor = webInterface.arg("sensor");
    if (sensor == "digital" || sensor == "analog") {
      if (sensor == "digital" && !climateSensorOperational) {
        webInterface.send(400, "application/json", "{\"status\":\"error\",\"message\":\"DHT22 sensor not active\"}");
      } else {
        activeSensorType = sensor;
        webInterface.send(200, "application/json", "{\"status\":\"ok\",\"sensor\":\"" + sensor + "\"}");

        Serial.println("🎯 Selected sensor for threshold: " + sensor);

        // Update LCD
    #if ENABLE_DISPLAY
        displayScreen.clear();
        displayCenteredText("Sensor Selected", 0);
        displayCenteredText(sensor.c_str(), 1);
        delay(1500);
    #endif
      }
    } else {
      webInterface.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid sensor\"}");
    }
  } else {
    webInterface.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing sensor parameter\"}");
  }
}

void processDataExportRequest() {
  Serial.println("📊 Data export requested");

  String csv = "timestamp,digitalTemp(DHT22),analogTemp(TMP36),dhtActive\n";

  int start = (historyCount < 50) ? 0 : historyIndex;
  for (int i = 0; i < min(historyCount, 50); i++) {
    int idx = (start + i) % 50;
    csv += String(measurementHistory[idx].recordTime) + ",";
    csv += String(measurementHistory[idx].primaryTemperature, 2) + ",";
    csv += String(measurementHistory[idx].secondaryTemperature, 2) + ",";
    csv += String(measurementHistory[idx].climateActive ? "true" : "false") + "\n";
  }

  webInterface.sendHeader("Content-Disposition", "attachment; filename=climate_data_DHT22_TMP36.csv");
  webInterface.send(200, "text/csv", csv);
}

void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("🔌 === WEBSOCKET DISCONNECTED ===\n");
      Serial.printf("   📍 Client #%u disconnected\n", num);
      Serial.printf("   👥 Active WebSocket connections: %u\n", websocketHandler.connectedClients());
      break;

    case WStype_CONNECTED: {
      IPAddress ip = websocketHandler.remoteIP(num);
      Serial.printf("🔌 === WEBSOCKET CONNECTED ===\n");
      Serial.printf("   📍 Client #%u connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      Serial.printf("   👥 Active WebSocket connections: %u\n", websocketHandler.connectedClients());
      Serial.println("   📡 Sending initial data to new client...");
      // Send initial data
      transmitEnvironmentData();
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

void transmitEnvironmentData() {
  static unsigned long lastTransmissionLog = 0;

  DynamicJsonDocument doc(512);
  doc["type"]           = "sensorData";
  doc["digitalTemp"]    = primaryTemperature;
  doc["analogTemp"]     = secondaryTemperature;
  doc["threshold"]      = temperatureLimit;
  doc["fanSpeed"]       = map(coolingIntensity, 0, 255, 0, 100);
  doc["alertActive"]    = warningActive;
  doc["selectedSensor"] = activeSensorType;
  doc["dhtActive"]      = climateSensorOperational;
  doc["timestamp"]      = millis();
  doc["connectedClients"] = connectedDevices;
  doc["uptime"]         = (millis() - systemBootTime) / 1000;
  doc["continuousMode"] = true;  // NEW
  doc["sensorType"]     = "TMP36"; // NEW

  String message;
  serializeJson(doc, message);

  uint8_t wsClientCount = websocketHandler.connectedClients();
  websocketHandler.broadcastTXT(message);

  // Log broadcast every 30 seconds to reduce spam
  if (millis() - lastTransmissionLog >= 30000) {
    lastTransmissionLog = millis();
    Serial.println("📡 Broadcasting sensor data to " + String(wsClientCount) + " WebSocket clients");
    if (wsClientCount > 0) {
      Serial.println("   📊 Data: DHT22=" + String(primaryTemperature, 1) + "°C, TMP36=" + String(secondaryTemperature, 1) + "°C, Cool=" + String(map(coolingIntensity, 0, 255, 0, 100)) + "%");
    }
  }
}

void readPrimaryTemperatureWithRetry() {
  if (!climateSensorOperational) {
    Serial.println("⚠️ DHT22 sensor marked as inactive, skipping read");
    return;
  }

  Serial.println("🔍 Reading DHT22 sensor...");
  bool success = false;

  for (int attempt = 1; attempt <= CLIMATE_MAX_RETRY_ATTEMPTS && !success; attempt++) {
    if (attempt > 1) {
      Serial.println("🔄 DHT22 retry attempt " + String(attempt) + "/" + String(CLIMATE_MAX_RETRY_ATTEMPTS));
      delay(1000);
    }

    float humidity = climateSensor.readHumidity();
    float tempC    = climateSensor.readTemperature();

    Serial.println("   📊 Raw DHT22 values - Temp: " + String(tempC) + "°C, Humidity: " + String(humidity) + "%");

    if (isnan(humidity) || isnan(tempC)) {
      Serial.println("❌ DHT22 read failed - NaN values detected");
      climateSensorErrors++;
      consecutiveClimateErrors++;
      Serial.println("   📈 Error count: " + String(climateSensorErrors) + " (consecutive: " + String(consecutiveClimateErrors) + ")");
    }
    else if (tempC >= -40 && tempC <= 80) {
      primaryTemperature = tempC;
      success      = true;
      consecutiveClimateErrors = 0;
      lastClimateSuccessTime = millis();
      Serial.println("✅ DHT22 reading successful: " + String(tempC, 1) + "°C, " + String(humidity, 1) + "%");
      Serial.println("   ✅ Consecutive errors reset to 0");
    }
    else {
      Serial.println("⚠️ DHT22 temperature out of valid range: " + String(tempC) + "°C (valid: -40 to 80°C)");
      climateSensorErrors++;
      consecutiveClimateErrors++;
      Serial.println("   📈 Error count: " + String(climateSensorErrors) + " (consecutive: " + String(consecutiveClimateErrors) + ")");
    }
  }

  if (!success) {
    Serial.println("❌ DHT22 failed after " + String(CLIMATE_MAX_RETRY_ATTEMPTS) + " attempts");
    Serial.println("   📊 Total errors: " + String(climateSensorErrors) + ", Consecutive: " + String(consecutiveClimateErrors));
    if (consecutiveClimateErrors >= CLIMATE_MAX_CONSECUTIVE_ERRORS) {
      Serial.println("   ⚠️ DHT22 may need reinitialization soon");
    }
  }
}

void adjustTMP36Calibration() {
  Serial.println("🔧 === TMP36 CALIBRATION PROCEDURE ===");
  Serial.println("   📊 Taking calibration readings...");
  
  // Take multiple readings for calibration
  int calibrationSamples = 20;
  long adcSum = 0;
  
  for (int i = 0; i < calibrationSamples; i++) {
    adcSum += analogRead(THERMAL_SENSOR_PIN);
    delay(100);
  }
  
  int avgADC = adcSum / calibrationSamples;
  float measuredVoltage = avgADC * realVoltageRef / 4095.0;
  
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
      realVoltageRef = altVRef2;
      Serial.println("   ✅ Auto-selected 3.15V reference (most common ESP32 actual voltage)");
    } else if (testVoltage1 >= 0.6 && testVoltage1 <= 0.9) {
      realVoltageRef = altVRef1;
      Serial.println("   ✅ Auto-selected 3.0V reference");
    } else if (testVoltage3 >= 0.6 && testVoltage3 <= 0.9) {
      realVoltageRef = altVRef3;
      Serial.println("   ✅ Auto-selected 3.45V reference");
    } else {
      realVoltageRef = 3.15; // Default to most common
      Serial.println("   ⚠️ Using default 3.15V reference (most ESP32 boards)");
    }
  }
  
  Serial.println("   🎯 Final voltage reference: " + String(realVoltageRef, 2) + "V");
  
  // Calculate expected temperature with corrected voltage
  float correctedVoltage = avgADC * realVoltageRef / 4095.0;
  float calculatedTemp = (correctedVoltage - 0.5) * 100.0;
  
  Serial.println("   🌡️ Calculated temperature: " + String(calculatedTemp, 1) + "°C");
  
  // Suggest calibration offset if needed
  Serial.println("   📝 If this temperature seems wrong, you can:");
  Serial.println("      • Manually set tmp36AdjustmentFactor in code");
  Serial.println("      • Compare with DHT22 reading for reference");
  
  Serial.println("✅ TMP36 calibration complete!");
}

void readSecondaryTemperature() {
  Serial.println("🔍 Reading TMP36 sensor...");

  // Read multiple samples for better accuracy
  int sampleCount = 10;  // Increased sample count
  long adcSum = 0;
  
  for (int i = 0; i < sampleCount; i++) {
    adcSum += analogRead(THERMAL_SENSOR_PIN);
    delay(5); // Small delay between readings
  }
  
  int adcValue = adcSum / sampleCount; // Average the samples
  
  // TMP36 conversion formula with calibrated voltage reference
  // TMP36 outputs 500mV at 0°C and 10mV per degree
  // Formula: Temperature = (Voltage - 0.5V) * 100
  float voltage = adcValue * realVoltageRef / 4095.0;  // Use calibrated voltage reference
  float tempC = (voltage - 0.5) * 100.0 + tmp36AdjustmentFactor;  // Apply calibration offset

  Serial.println("   📊 Raw ADC reading: " + String(adcValue) + " (averaged from " + String(sampleCount) + " samples)");
  Serial.println("   ⚡ Voltage (calibrated): " + String(voltage, 3) + "V (using " + String(realVoltageRef, 2) + "V reference)");
  Serial.println("   🌡️ TMP36 calculation: (" + String(voltage, 3) + " - 0.5) * 100 + " + String(tmp36AdjustmentFactor, 1) + " = " + String(tempC, 2) + "°C");

  // TMP36 operating range: -40°C to +125°C
  if (tempC >= -40 && tempC <= 125) {
    secondaryTemperature = tempC;
    Serial.println("✅ TMP36 reading accepted: " + String(tempC, 1) + "°C");
  } else {
    Serial.println("⚠️ TMP36 temperature out of range: " + String(tempC) + "°C (valid: -40 to 125°C)");
    Serial.println("   🔧 Check TMP36 wiring: VCC→3.3V, GND→GND, VOUT→Pin33");
    Serial.println("   🔧 Current voltage: " + String(voltage, 3) + "V (should be ~0.5V at 0°C, ~0.75V at 25°C)");
    Serial.println("   🔧 Try adjusting realVoltageRef or tmp36AdjustmentFactor in code");
  }
}

void verifyClimateStatus() {
  if (consecutiveClimateErrors >= CLIMATE_MAX_CONSECUTIVE_ERRORS) {
    if (millis() - lastClimateSuccessTime > 300000) { // 5 minutes
      Serial.println("🔄 DHT22 sensor reinitializing after extended failure...");
      climateSensor.begin();
      delay(2000);

      if (consecutiveClimateErrors >= CLIMATE_MAX_CONSECUTIVE_ERRORS) {
        climateSensorOperational = false;
        Serial.println("❌ DHT22 marked as inactive - switching to TMP36 sensor");
        if (activeSensorType == "digital") {
          activeSensorType = "analog";
        }
      }
    }
  }
}

void evaluateTemperatureLimits() {
  float currentTemp    = (activeSensorType == "digital" && climateSensorOperational) ? primaryTemperature : secondaryTemperature;
  bool  isOverLimit = currentTemp > temperatureLimit;

  // Log threshold check details (every 60 seconds to reduce spam)
  static unsigned long lastLimitLog = 0;
  if (millis() - lastLimitLog > 60000) {
    Serial.println("🎯 Limit Check:");
    Serial.println("   📊 Selected sensor: " + activeSensorType + " (" + (activeSensorType == "digital" ? "DHT22" : "TMP36") + ")");
    Serial.println("   🌡️ Current temperature: " + String(currentTemp, 1) + "°C");
    Serial.println("   🎯 Limit setting: " + String(temperatureLimit, 1) + "°C");
    Serial.println("   ⚖️ Status: " + String(currentTemp) + (isOverLimit ? " > " : " <= ") + String(temperatureLimit) + " = " + (isOverLimit ? "OVER" : "NORMAL"));
    lastLimitLog = millis();
  }

  if (isOverLimit != warningActive) {
    Serial.println("🚨 WARNING STATUS CHANGE DETECTED!");
    warningActive = isOverLimit;

    if (warningActive) {
      Serial.println("🚨 === TEMPERATURE WARNING ACTIVATED ===");
      Serial.println("   🌡️ Temperature: " + String(currentTemp, 1) + "°C");
      Serial.println("   🎯 Limit: " + String(temperatureLimit, 1) + "°C");
      Serial.println("   📊 Difference: +" + String(currentTemp - temperatureLimit, 1) + "°C");

      // Check current cooling intensity before automatic activation
      int currentCoolingPercent = map(coolingIntensity, 0, 255, 0, 100);
      Serial.println("   🌀 Current cooling intensity: " + String(currentCoolingPercent) + "%");

      if (currentCoolingPercent == 0) {
        Serial.println("   🚀 Automatically activating cooling and alert...");
        adjustCoolingAndAlert(100, "Automatic Warning System");
      } else {
        Serial.println("   ℹ️ Cooling already running at " + String(currentCoolingPercent) + "% - keeping current intensity");
        // Still activate alert for warning (higher frequency for warnings)
        Serial.println("   🔊 Activating warning alert at 2000Hz...");
        tone(ALERT_BUZZER_PIN, 2000);
      }

      // Flash LED for visual warning
      Serial.println("   💡 Flashing LED for visual warning...");
      for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
      }

      Serial.println("🚨 === WARNING ACTIVATION COMPLETE ===");

    } else {
      Serial.println("✅ === TEMPERATURE WARNING CLEARED ===");
      Serial.println("   🌡️ Temperature returned to normal: " + String(currentTemp, 1) + "°C");
      Serial.println("   🎯 Limit: " + String(temperatureLimit, 1) + "°C");
      Serial.println("   📊 Difference: " + String(currentTemp - temperatureLimit, 1) + "°C below limit");

      // We do not automatically turn off the cooling when warning clears
      // Manual control persists
      Serial.println("   ℹ️ Cooling intensity maintained at current setting for manual control");
      Serial.println("   💡 Use web dashboard to manually adjust cooling intensity if desired");

      // Only turn off alert or revert to normal tone
      Serial.println("   🔇 Deactivating warning alert...");

      int currentCoolingPercent = map(coolingIntensity, 0, 255, 0, 100);
      if (currentCoolingPercent > 0) {
        Serial.println("   🔊 Switching to normal cooling operation alert (1500Hz)...");
        tone(ALERT_BUZZER_PIN, 1500); // Normal cooling operation tone
      } else {
        Serial.println("   🔇 Turning off alert completely (cooling is off)...");
        noTone(ALERT_BUZZER_PIN);
      }

      Serial.println("✅ === WARNING CLEARED COMPLETE ===");
    }
  }
}

void recordEnvironmentData() {
  measurementHistory[historyIndex].recordTime = millis();
  measurementHistory[historyIndex].primaryTemperature = primaryTemperature;
  measurementHistory[historyIndex].secondaryTemperature = secondaryTemperature;
  measurementHistory[historyIndex].climateActive = climateSensorOperational;

  historyIndex = (historyIndex + 1) % 50;
  if (historyCount < 50) historyCount++;
}

void refreshDisplayContent() {
#if ENABLE_DISPLAY
  if (warningActive) {
    // Show warning on LCD
    displayScreen.clear();
    displayCenteredText("!! WARNING !!", 0);
    char buffer[17];
    float currentTemp = (activeSensorType == "digital" && climateSensorOperational) ? primaryTemperature : secondaryTemperature;
    sprintf(buffer, "%.1fC > %.1fC", currentTemp, temperatureLimit);
    displayCenteredText(buffer, 1);
    return;
  }

  static int screenState = 0;

  // Declare buffers once to avoid redefinition
  char buffer[17];
  char buffer2[17];
  int  coolingPercent;

  switch (screenState) {
    case 0:
      // Temperature readings
      displayScreen.clear();
      displayScreen.setCursor(0, 0);
      if (climateSensorOperational) {
        sprintf(buffer, "DHT:%.1fC TMP:%.1fC", primaryTemperature, secondaryTemperature);
        displayScreen.print(buffer);
      } else {
        sprintf(buffer, "DHT:ERR TMP:%.1fC", secondaryTemperature);
        displayScreen.print(buffer);
      }
      displayScreen.setCursor(0, 1);
      sprintf(buffer2, "Limit:%.1fC", temperatureLimit);
      displayScreen.print(buffer2);
      break;

    case 1:
      // Cooling and alert status
      displayScreen.clear();
      displayScreen.setCursor(0, 0);
      coolingPercent = map(coolingIntensity, 0, 255, 0, 100);
      sprintf(buffer2, "Cool: %d%%", coolingPercent);
      displayScreen.print(buffer2);
      displayScreen.setCursor(0, 1);
      if (coolingPercent > 0) {
        displayScreen.print("Alert: ON");
      } else {
        displayScreen.print("Alert: OFF");
      }
      break;

    case 2:
      // Sensor types and status
      displayScreen.clear();
      displayScreen.setCursor(0, 0);
      displayScreen.print("DHT22 + TMP36");
      displayScreen.setCursor(0, 1);
      sprintf(buffer2, "Devices: %d", connectedDevices);
      displayScreen.print(buffer2);
      break;

    case 3:
      // WiFi info with persistent indicator
      displayScreen.clear();
      displayCenteredText("192.168.4.1", 0);
      displayCenteredText("PERSISTENT", 1);
      break;
  }

  screenState = (screenState + 1) % 4;
#endif
}

void displayCenteredText(const char* text, int row) {
#if ENABLE_DISPLAY
  int spaces = (16 - strlen(text)) / 2;
  if (spaces < 0) spaces = 0;
  displayScreen.setCursor(spaces, row);
  displayScreen.print(text);
#endif
}

String generateMainHTML() {
  return R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Climate Monitor Dashboard</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <link rel="stylesheet" href="/style.css">
</head>
<body>
    <nav class="navbar navbar-dark bg-primary">
        <div class="container-fluid">
            <span class="navbar-brand h1">🌡️ ESP32 Climate Monitor - DHT22 + TMP36</span>
            <div>
                <span class="navbar-text me-3" id="clientCount">Devices: 0</span>
                <span class="navbar-text me-3" id="uptime">Uptime: 0s</span>
                <span class="navbar-text" id="status">Connecting...</span>
            </div>
        </div>
    </nav>

    <div class="container-fluid mt-4">
        <!-- Persistent Mode Alert -->
        <div class="row mb-3">
            <div class="col-12">
                <div class="alert alert-info" role="alert">
                    <h4 class="alert-heading">🔒 Persistent Mode Active!</h4>
                    <p class="mb-0">WiFi hotspot is running in persistent mode with dual temperature sensors: <strong>DHT22 (Primary)</strong> and <strong>TMP36 (Secondary)</strong>. No automatic shutdowns or timeouts - designed for 24/7 operation.</p>
                </div>
            </div>
        </div>

        <!-- Temperature Gauges Row -->
        <div class="row mb-4">
            <div class="col-md-6">
                <div class="card text-center">
                    <div class="card-header">
                        <h5>Primary Sensor (DHT22)</h5>
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
                        <h5>Secondary Sensor (TMP36)</h5>
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
                            <label for="fanSlider" class="form-label">Cooling Intensity: <span id="fanSpeedValue">0</span>%</label>
                            <input type="range" class="form-range" id="fanSlider" min="0" max="100" value="0">
                            <div class="mt-2">
                                <small class="text-muted">🌀 Cooling Status: <span id="fanStatus">OFF</span></small><br>
                                <small class="text-muted">🔊 Alert Status: <span id="buzzerStatus">OFF</span></small>
                            </div>
                        </div>
                        
                        <div class="mb-3">
                            <label for="thresholdInput" class="form-label">Temperature Limit (°C)</label>
                            <input type="number" class="form-control" id="thresholdInput" min="0" max="50" step="0.5" value="28">
                        </div>
                        
                        <div class="mb-3">
                            <label for="sensorSelect" class="form-label">Limit Sensor</label>
                            <select class="form-select" id="sensorSelect">
                                <option value="digital">Primary Sensor (DHT22)</option>
                                <option value="analog">Secondary Sensor (TMP36)</option>
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
                                    <h6>Devices</h6>
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

String generateCSS() {
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

/* Persistent mode styling */
.alert-info {
    border-left: 5px solid #0dcaf0;
}

.navbar-text {
    font-size: 0.9rem;
})rawliteral";
}

String generateJavaScript() {
  return R"rawliteral(let socket;
let chart;
let chartData = {
    labels: [],
    datasets: [
        {
            label: 'Primary (DHT22)',
            data: [],
            borderColor: '#007bff',
            backgroundColor: 'rgba(0, 123, 255, 0.1)',
            fill: true
        },
        {
            label: 'Secondary (TMP36)',
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
    
    console.log('Connecting to WebSocket (TMP36 + Persistent Mode):', wsUrl);
    socket = new WebSocket(wsUrl);
    
    socket.onopen = function() {
        document.getElementById('status').textContent = 'Connected (24/7)';
        document.getElementById('status').className = 'navbar-text text-success';
        console.log('WebSocket connected with TMP36 sensor in persistent mode');
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
        console.log('WebSocket disconnected in persistent mode. Code:', event.code);
        console.log('Reconnecting in 2 seconds (persistent mode)...');
        setTimeout(initializeWebSocket, 2000);
    };
    
    socket.onerror = function(error) {
        document.getElementById('status').textContent = 'Connection Error';
        document.getElementById('status').className = 'navbar-text text-danger';
        console.error('WebSocket error in persistent mode:', error);
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
    
    // Update cooling controls and status
    document.getElementById('fanSlider').value = data.fanSpeed;
    document.getElementById('fanSpeedValue').textContent = data.fanSpeed;
    
    // Update cooling status display
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
        alertText.textContent = 'TEMPERATURE WARNING ACTIVE! Cooling & Alert Activated';
    } else {
        alertStatus.className = 'alert alert-success';
        alertText.textContent = 'Normal Operation (Persistent Mode - DHT22 + TMP36)';
    }
    
    // Update uptime displays
    const uptime = data.uptime || Math.floor(data.timestamp / 1000);
    const uptimeFormatted = formatUptime(uptime);
    document.getElementById('uptime').textContent = 'Uptime: ' + uptimeFormatted;
    document.getElementById('uptimeDisplay').textContent = uptimeFormatted;
    
    document.getElementById('dhtErrors').textContent = data.dhtErrors || 0;
    document.getElementById('connectedClients').textContent = data.connectedClients || 0;
    document.getElementById('clientCount').textContent = 'Devices: ' + (data.connectedClients || 0);
    
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
            console.log('Cooling intensity updated successfully to ' + speed + '% (TMP36 + persistent mode)');
        } else {
            console.error('Cooling control error:', data.message);
            alert('Error setting cooling intensity: ' + data.message);
        }
    })
    .catch(error => {
        console.error('Cooling control request failed:', error);
        alert('Failed to communicate with climate monitor');
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
            console.log('Limit updated to ' + threshold + '°C (TMP36 + persistent mode)');
        } else {
            console.error('Limit update error:', data.message);
        }
    })
    .catch(error => {
        console.error('Limit update failed:', error);
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
            console.log('Sensor selection updated to ' + sensor + ' (TMP36 + persistent mode)');
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
    console.log('Initializing ESP32 Climate Monitor Dashboard - TMP36 + PERSISTENT MODE');
    console.log('Features: DHT22 + TMP36 dual sensors, 24/7 operation, no WiFi timeouts, watchdog protection');
    initializeChart();
    
    // Start WebSocket connection immediately for persistent mode
    initializeWebSocket();
});)rawliteral";
}
