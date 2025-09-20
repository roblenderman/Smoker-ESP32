/*
 * ESP32 Smoker Temperature Controller
 *
 * This system provides precise temperature control for a smoker using:
 * - MAX6675 thermocouple for smoker chamber temperature (primary control sensor)
 * - Two thermistor sensors for meat temperature monitoring
 * - PID controller for maintaining target smoker temperature
 * - Time-proportional relay control for heater power modulation
 * - Rotary encoder for setpoint adjustment and mode switching
 * - LCD display and web interface for monitoring and control
 * - Telnet interface for wireless debugging and monitoring
 *
 * Control Modes:
 * - Smoker Mode: PID controls smoker temperature, encoder adjusts smoker setpoint
 * - Meat Mode: PID still controls smoker, encoder adjusts meat target for monitoring
 *
 * Hardware: ESP32, MAX6675, thermistors, rotary encoder, LCD, relay module
 */
#include <WiFi.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AiEsp32RotaryEncoder.h>
#include <movingAvg.h>  // Moving average library for filtering
#include <ArduinoOTA.h> // Include ArduinoOTA library
#include <ArduPID.h>    // Arduino PID library

// Wi-Fi credentials
const char* ssid = "LHome";
const char* password = "stacey8561";

// Telnet server for wireless serial monitoring
WiFiServer telnetServer(23);
WiFiClient telnetClient;

// Thermocouple (SPI)
int thermoCLK = 18; // SCLK
int thermoCS = 5;   // CS
int thermoDO = 19;  // MISO
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Thermocouple calibration offset (subtract from reading to correct)
const float THERMOCOUPLE_OFFSET_F = 0.0; // °F offset to subtract (reads 7°F high)

// Thermistor calibration offsets (adjust these based on your calibration tests)
const float THERMISTOR1_OFFSET_F = 4.0; // °F offset for thermistor 1 (adjust as needed)
const float THERMISTOR2_OFFSET_F = 4.0; // °F offset for thermistor 2 (adjust as needed)

// ADC Configuration
const int ADC_MAX_VALUE = 4095; // 12-bit ADC maximum value
const int THERMISTOR_SAMPLES = 20; // Number of ADC samples to average
const int THERMISTOR_SAMPLE_DELAY_MS = 5; // Delay between samples

// Thermocouple Configuration
const int THERMOCOUPLE_SAMPLES = 2; // Number of thermocouple samples to average
const int THERMOCOUPLE_SAMPLE_DELAY_MS = 250; // Delay between thermocouple samples

// I2C LCD (20x4)
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20x4 LCD

// Relay Control Pin (high-level trigger)
const int relayPin = 16;

// Voltage Divider (Thermistor 1)
const int THERMISTOR1_PIN = 32; // ADC1
const float THERMISTOR_FIXED_RESISTOR_OHMS = 12400.0; // 12.4kΩ
const float ADC_REFERENCE_VOLTAGE = 3.25; // 3.3V
const float THERMISTOR_NOMINAL_RESISTANCE = 107000.0; // 107kΩ at 25°C
const float THERMISTOR_BETA_COEFFICIENT = 3950.0; // Adjust if known

// Voltage Divider (Thermistor 2)
const int THERMISTOR2_PIN = 33; // ADC1
const float THERMISTOR2_FIXED_RESISTOR_OHMS = 12400.0; // 12.4kΩ

// Moving average filters for temperature readings (smoothing)
movingAvg thermistor1Filter(7);     // 10-sample moving average for thermistor 1
movingAvg thermistor2Filter(7);     // 10-sample moving average for thermistor 2
movingAvg thermocoupleFilter(4);     // 6-sample moving average for thermocouple (faster response, reduced overshoot)

// Rotary Encoder Pins (updated - avoid GPIO 12 for boot issues)
const int ROTARY_ENCODER_A_PIN = 13;  // Back to 13 for wiring check
const int ROTARY_ENCODER_B_PIN = 14;  // Swapped with A pin to fix backwards response
const int ROTARY_ENCODER_BUTTON_PIN = 25;
const int ROTARY_ENCODER_VCC_PIN = -1; // Not used
const int ROTARY_ENCODER_STEPS = 4;

AiEsp32RotaryEncoder rotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS, false); // false = use pullup resistors

void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

// Smoker and meat setpoints
float smokerTemp = 250.0; // °F, initial setpoint
float meatDoneTemp = 170.0; // °F, initial meat done temp
const float SMOKER_TEMP_MIN = 150.0; // °F
const float SMOKER_TEMP_MAX = 350.0; // °F
const float TEMP_STEP = 5.0; // °F increment

// LCD update flag
bool updateLCD = false;

// --- Meat temp prediction variables ---
const int PREDICTION_WINDOW_MINUTES = 15;
const int PREDICTION_INTERVAL_SECONDS = 60; // 1 minute
float meatTempHistory[PREDICTION_WINDOW_MINUTES];
unsigned long meatTempTimestamps[PREDICTION_WINDOW_MINUTES];
int meatTempHistoryIndex = 0;
int meatTempHistoryCount = 0;
unsigned long lastPredictionSampleTime = 0;
float lastMeatTempPredictionMinutes = -1;
bool predictionValid = false;

// PID parameters and variables
double pidSetpoint, pidInput, pidOutput;
double Kp = 11, Ki = 0.15, Kd = 35; // Tighter control: Ki↓ (0.15→0.12), Kd↑ (25→35) for reduced oscillation
ArduPID myPID; // Will be initialized in setup() with .begin() method

// PID Control Configuration
const unsigned long PID_TELEPLOT_INTERVAL_MS = 5000; // Send PID data every 5 seconds
const float PID_OUTPUT_MIN = 25.0;   // Minimum PID output percentage
const float PID_OUTPUT_MAX = 100.0; // Maximum PID output percentage

const float WINDOW_SIZE = 10000; // 5s window in ms (reduced for finer control)
unsigned long windowStartTime = 0;
bool relayState = false;
float pidOutputPercent = 0.0; // Current PID output percentage (0-100)

// Button hold-to-adjust (legacy - no longer used)
const unsigned long REPEAT_DELAY = 200; // ms between adjustments
//const unsigned long MODE_TIMEOUT = 5000; // 5s inactivity to exit meat temp mode
const unsigned long MODE_ENTER_HOLD = 1000; // 1s hold to enter meat temp mode
unsigned long lastButtonActivity = 0;
bool meatTempMode = false;

// Web server
AsyncWebServer server(80);

// Function declarations
void debugPrint(String message);
void debugPrintln(String message);

// Temperature reading functions
float readThermocoupleTemperature();
float readThermistorTemperature(int pin, float fixedResistorOhms, float offsetF);
void updateTemperatureFilters(float thermocoupleTemp, float thermistor1Temp, float thermistor2Temp);

// PID Control Functions
void initializePID();
float updatePID(float currentTemp, float setpoint);
void controlHeater(float pidOutput);
void resetPID();
void sendPIDTelemetry();

float calculateTemp(float r) {
  float t25 = 298.15; // 25°C in Kelvin
  float tempK = 1.0 / ( (1.0 / t25) + (log(r / THERMISTOR_NOMINAL_RESISTANCE) / THERMISTOR_BETA_COEFFICIENT) );
  float tempC = tempK - 273.15; // °C
  return tempC * 9.0 / 5.0 + 32.0; // °F
}

String getUptime() {
  unsigned long ms = millis();
  unsigned long hours = ms / 3600000;
  unsigned long minutes = (ms % 3600000) / 60000;
  return String(hours) + "h " + String(minutes) + "m";
}

String getUptimeLCD() {
  unsigned long ms = millis();
  unsigned long hours = ms / 3600000;
  unsigned long minutes = (ms % 3600000) / 60000;
  
  // Format as HH:MM with leading zeros - force two digits for both hours and minutes
  String timeStr = "Time:";
  
  // Force two digits for hours (00-99)
  if (hours < 10) {
    timeStr += "0";
  }
  timeStr += String(hours);
  timeStr += ":";
  
  // Force two digits for minutes (00-59)
  if (minutes < 10) {
    timeStr += "0";
  }
  timeStr += String(minutes);
  
  return timeStr;
}

// Add after includes and before setup()

// Helper function for dual output to Serial and Telnet
void debugPrint(String message) {
  Serial.print(message);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(message);
    telnetClient.flush();
  }
}

void debugPrintln(String message) {
  Serial.println(message);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(message);
    telnetClient.flush();
  }
}

// Temperature Reading Functions
// ============================

/**
 * Reads thermocouple temperature with averaging and filtering
 * @return Temperature in Fahrenheit (°F)
 */
float readThermocoupleTemperature() {
  float thermocoupleReading = 0;
  int validReadings = 0;

  // Take multiple samples for thermocouple stability
  for (int i = 0; i < THERMOCOUPLE_SAMPLES; i++) {
    float reading = thermocouple.readCelsius();
    if (!isnan(reading)) {
      thermocoupleReading += reading;
      validReadings++;
      //debugPrintln("TC: " + String(reading) + "°C");
    }
    delay(THERMOCOUPLE_SAMPLE_DELAY_MS);
  }

  float tempThermocoupleC;
  if (validReadings > 0) {
    tempThermocoupleC = thermocoupleFilter.reading(thermocoupleReading / validReadings);
  } else {
    tempThermocoupleC = thermocoupleFilter.reading(thermocouple.readCelsius());
  }

  // Convert to Fahrenheit and apply offset
  return (tempThermocoupleC * 9.0 / 5.0 + 32.0) - THERMOCOUPLE_OFFSET_F;
}

/**
 * Reads thermistor temperature with averaging and filtering
 * @param pin ADC pin number
 * @param fixedResistorOhms Fixed resistor value in ohms
 * @param offsetF Temperature offset in Fahrenheit
 * @return Temperature in Fahrenheit (°F)
 */
float readThermistorTemperature(int pin, float fixedResistorOhms, float offsetF) {
  // Take multiple ADC samples for stability
  int rawSum = 0;
  for (int i = 0; i < THERMISTOR_SAMPLES; i++) {
    rawSum += analogRead(pin);
    //debugPrintln("PIN: " + String(pin) + " ADC: " + String(analogRead(pin)));
    delay(THERMISTOR_SAMPLE_DELAY_MS);
  }
  int rawAverage = rawSum / THERMISTOR_SAMPLES;

  // Apply moving average filter
  int filteredRaw = (pin == THERMISTOR1_PIN) ?
    thermistor1Filter.reading(rawAverage) :
    thermistor2Filter.reading(rawAverage);

  // Convert ADC reading to voltage
  float voltage = (filteredRaw / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;

  // Calculate thermistor resistance using voltage divider formula
  float thermistorResistance = (voltage * fixedResistorOhms) / (ADC_REFERENCE_VOLTAGE - voltage);

  // Prevent invalid resistance values
  if (thermistorResistance <= 0) thermistorResistance = 1.0;

  // Calculate temperature and apply offset
  return calculateTemp(thermistorResistance) - offsetF;
}

/**
 * Reads all temperatures and updates global variables
 * @param thermocoupleF Reference to store thermocouple temperature
 * @param thermistor1F Reference to store thermistor 1 temperature
 * @param thermistor2F Reference to store thermistor 2 temperature
 */
void readAllTemperatures(float& thermocoupleF, float& thermistor1F, float& thermistor2F) {
  thermocoupleF = readThermocoupleTemperature();
  thermistor1F = readThermistorTemperature(THERMISTOR1_PIN, THERMISTOR_FIXED_RESISTOR_OHMS, THERMISTOR1_OFFSET_F);
  thermistor2F = readThermistorTemperature(THERMISTOR2_PIN, THERMISTOR2_FIXED_RESISTOR_OHMS, THERMISTOR2_OFFSET_F);
}

// PID Control Functions
// =====================

/**
 * Initializes the PID controller with proper settings
 */
void initializePID() {
  myPID.begin(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd);
  myPID.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX); // PID output 0-100%
  myPID.setWindUpLimits(0, 60); // Prevent integral windup, limit to ±60% of output range
  myPID.setSampleTime(1000); // 1 second sample time
  myPID.start(); // Start the PID controller
}

/**
 * Updates PID controller with current temperature and setpoint
 * @param currentTemp Current thermocouple temperature in °F
 * @param setpoint Target temperature in °F
 * @return PID output percentage (0-100%)
 */
float updatePID(float currentTemp, float setpoint) {
  pidInput = currentTemp;
  pidSetpoint = setpoint;

  myPID.compute(); // Calculate PID output based on error (setpoint - input)

  debugPrint("PID Compute Result: ");
  debugPrintln(String(pidOutput));

  // Store PID output percentage for web display
  pidOutputPercent = pidOutput;

  return pidOutput;
}

/**
 * Controls the heater relay using time-proportional control
 * @param pidOutput PID output percentage (0-100%)
 */
void controlHeater(float pidOutput) {
  unsigned long currentTime = millis();

  // Time-Proportional Relay Control
  // Converts PID output percentage to on/off timing within a fixed time window
  float effectivePidOutput = pidOutput;

  // Update display percentage to reflect PID output
  pidOutputPercent = effectivePidOutput;

  float onTime = (effectivePidOutput / 100.0) * WINDOW_SIZE; // Calculate ON time in milliseconds
  unsigned long windowElapsed = currentTime - windowStartTime;
  bool shouldBeOn = (windowElapsed <= onTime);

  static bool lastControlState = false;

  if (shouldBeOn != lastControlState) {
    if (shouldBeOn) {
      digitalWrite(relayPin, HIGH); // Relay ON (HIGH for high-trigger relay)
      relayState = true;
      debugPrintln("HEATER: Turning ON (relay pin HIGH)");
    } else {
      digitalWrite(relayPin, LOW); // Relay OFF (LOW for high-trigger relay)
      relayState = false;
      debugPrintln("HEATER: Turning OFF (relay pin LOW)");
    }
    lastControlState = shouldBeOn;
  }

  // Update Teleplot with relay state and pin voltage verification
  static unsigned long lastRelayTeleplotTime = 0;
  if (currentTime - lastRelayTeleplotTime > 1000) { // Send every 1 second for relay
    debugPrint(">relay_state:");
    debugPrintln(String(relayState ? 100.0 : 0.0));
    debugPrint(">relay_on_time_ms:");
    debugPrintln(String(onTime));
    debugPrint(">pid_output_percent:");
    debugPrintln(String(effectivePidOutput));

    debugPrint("Relay Control - Raw PID: ");
    debugPrint(String(pidOutput, 1));
    debugPrint("%, Effective: ");
    debugPrint(String(effectivePidOutput, 1));
    debugPrintln("%");

    lastRelayTeleplotTime = currentTime;
  }

  // Reset window timer when window period completes
  if (windowElapsed >= WINDOW_SIZE) {
    windowStartTime = currentTime;
  }
}

/**
 * Resets the PID controller integral term (useful when setpoint changes)
 */
void resetPID() {
  myPID.reset(); // Reset PID integral
}

/**
 * Sends PID telemetry data for debugging and monitoring
 */
void sendPIDTelemetry() {
  unsigned long currentTime = millis();

  // Update Teleplot with PID control data
  static unsigned long lastPidTeleplotTime = 0;
  if (currentTime - lastPidTeleplotTime > PID_TELEPLOT_INTERVAL_MS) { // Send every 5 seconds
    double error = pidSetpoint - pidInput;
    debugPrint(">pid_output:");
    debugPrintln(String(pidOutput));
   // debugPrint(">pid_error:");
   // debugPrintln(String(error));
   // debugPrint(">pid_setpoint:");
   // debugPrintln(String(pidSetpoint));
   // debugPrint(">pid_input:");
   // debugPrintln(String(pidInput));

    // Get internal PID term values from ArduPID
    //debugPrint(">pid_p_term:");
    //debugPrintln(String(myPID.P()));
    //debugPrint(">pid_i_term:");
    //debugPrintln(String(myPID.I()));
    //debugPrint(">pid_d_term:");
    //debugPrintln(String(myPID.D()));

    lastPidTeleplotTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  debugPrintln("ESP32 Starting - Debug Version");

  // Initialize Rotary Encoder BEFORE WiFi to avoid interrupt conflicts
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR, readEncoderISR);
  rotaryEncoder.setBoundaries(SMOKER_TEMP_MIN, SMOKER_TEMP_MAX, false);
  rotaryEncoder.setAcceleration(0);
  rotaryEncoder.setEncoderValue(smokerTemp);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), readEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), readEncoderISR, CHANGE);
  
  // Allow time for encoder to stabilize
  delay(100);
  
  // Initialize moving average filters for thermistors and thermocouple
  thermistor1Filter.begin();
  thermistor2Filter.begin();
  thermocoupleFilter.begin();
  debugPrintln("Moving average filters initialized");
  
  debugPrintln("Rotary encoder initialized");
  
  // Test encoder GPIO pins during startup
  debugPrintln("=== ENCODER GPIO TEST ===");
  for (int i = 0; i < 10; i++) {
    bool pinA = digitalRead(ROTARY_ENCODER_A_PIN);
    bool pinB = digitalRead(ROTARY_ENCODER_B_PIN);
    bool pinBtn = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
    debugPrint("Test ");
    debugPrint(String(i));
    debugPrint(": A=");
    debugPrint(pinA ? "H" : "L");
    debugPrint(", B=");
    debugPrint(pinB ? "H" : "L");
    debugPrint(", Btn=");
    debugPrint(pinBtn ? "H" : "L");
    debugPrint(", Encoder=");
    debugPrintln(String(rotaryEncoder.readEncoder()));
    delay(100);
  }
  debugPrintln("=== END GPIO TEST ===");
  debugPrintln("Try turning encoder now and watch for GPIO changes...");

  // Initialize I2C
  Wire.begin(21, 22); // SDA GPIO21, SCL GPIO22

  // Initialize Wi-Fi with stability improvements
  WiFi.mode(WIFI_STA);  // Set to station mode only
  WiFi.setAutoReconnect(true);  // Enable auto-reconnect
  WiFi.persistent(true);  // Save WiFi config to flash
  
  // Disable power saving mode for better stability
  WiFi.setSleep(false);
  
  // Set static IP to reduce DHCP issues (optional)
  IPAddress local_IP(192, 168, 1, 225);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);
  
  WiFi.begin(ssid, password);
  debugPrint("Connecting to WiFi...");
  int wifi_retry = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retry < 30) {
    delay(1000);
    debugPrint(".");
    wifi_retry++;
    
    // If taking too long, try reconnecting
    if (wifi_retry % 10 == 0) {
      debugPrintln("");
      debugPrint("Retry attempt ");
      debugPrint(String(wifi_retry / 10));
      debugPrint(" - Reconnecting...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, password);
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    debugPrintln("\nConnected to WiFi");
    debugPrint("IP Address: ");
    debugPrintln(WiFi.localIP().toString());
    debugPrint("Signal Strength: ");
    debugPrint(String(WiFi.RSSI()));
    debugPrintln(" dBm");
    
    // Determine WiFi band based on channel
    int channel = WiFi.channel();
    debugPrint("Channel: ");
    debugPrint(String(channel));
    if (channel >= 1 && channel <= 14) {
      debugPrintln(" (2.4GHz band)");
    } else if (channel >= 36 && channel <= 165) {
      debugPrintln(" (5GHz band)");
    } else {
      debugPrintln(" (Unknown band)");
    }
    
    debugPrint("MAC Address: ");
    debugPrintln(WiFi.macAddress());
    
    // Initialize OTA with standard ESP32 ArduinoOTA
    ArduinoOTA.setHostname("esp32-smoker");
    ArduinoOTA.setPort(3232);
    ArduinoOTA.begin();
    debugPrint("OTA Ready - Use IP address: ");
    debugPrintln(WiFi.localIP().toString());
    debugPrint("OTA Hostname: esp32-smoker, ");
    debugPrintln("OTA Port: 3232");
    
    // Start telnet server for wireless serial monitoring
    telnetServer.begin();
    debugPrint("Telnet server started on port 23 - Connect to: ");
    debugPrintln(WiFi.localIP().toString());
  } else {
    debugPrintln("\nFailed to connect to WiFi!");
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 1);
  lcd.print("IP: ");
  lcd.print(WiFi.localIP());
  
  // Show MAC address on LCD for 3 seconds
  lcd.setCursor(0, 2);
  lcd.print("MAC:");
  lcd.setCursor(0, 3);
  String macAddr = WiFi.macAddress();
  macAddr.replace(":", "");  // Remove colons to fit better
  lcd.print(macAddr.substring(0, 12));  // Show first 12 chars
  
  delay(5000); // Show IP and MAC for 5s

  // Initialize Relay Control Pin (high-level trigger)
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Start with LOW (relay OFF)
  debugPrintln("Relay control pin initialized - HIGH=ON, LOW=OFF");

  // Set ADC attenuation
  analogSetAttenuation(ADC_11db);

  // Initialize PID controller
  pidSetpoint = smokerTemp;
  pidInput = 0; // Initialize input
  pidOutput = 0; // Initialize output
  initializePID();
  windowStartTime = millis();

  // Setup web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html>";
    html += "<head><meta http-equiv='refresh' content='5'>";
    html += "<title>Smoker Control</title>";
    html += "<style>";
    html += "body { font-family: Arial; text-align: center; margin: 50px; background: #f7f7f7; }";
    html += "h1 { color: #333; }";
    html += "table { margin: 0 auto; border-collapse: collapse; background: #fff; box-shadow: 0 2px 8px #ccc; }";
    html += "th, td { padding: 12px 18px; border-bottom: 1px solid #eee; font-size: 1.1em; }";
    html += "th { background: #f0f0f0; }";
    html += "tr:last-child td { border-bottom: none; }";
    html += "button { padding: 6px 14px; margin: 0 4px; font-size: 1em; border-radius: 5px; border: 1px solid #888; background: #e0e0e0; cursor: pointer; transition: background 0.2s; }";
    html += "button:hover { background: #d0d0d0; }";
    html += ".btn-plus { background: #28a745; color: white; border: 1px solid #1e7e34; }";
    html += ".btn-plus:hover { background: #218838; }";
    html += ".btn-minus { background: #dc3545; color: white; border: 1px solid #bd2130; }";
    html += ".btn-minus:hover { background: #c82333; }";
    html += "</style></head>";
    html += "<body>";
    html += "<h1>Smoker Control</h1>";
    html += "<table>";
    html += "<tr><th>Sensor</th><th>Value</th><th>Setpoint</th><th>Actions</th></tr>";
    html += "<tr><td>Smoker</td><td>%TC_TEMP% &deg;F</td><td>%SET_TEMP% &deg;F</td>";
    html += "<td>";
    html += "<a href='/decrease'><button class='btn-minus' title='Decrease Smoker'>-5&deg;F</button></a>";
    html += "<a href='/increase'><button class='btn-plus' title='Increase Smoker'>+5&deg;F</button></a>";
    html += "</td></tr>";
    html += "<tr><td>Meat 1</td><td>%T1_TEMP% &deg;F</td><td>%MEAT_TEMP% &deg;F</td>";
    html += "<td>";
    html += "<a href='/decrease_meat'><button class='btn-minus' title='Decrease Meat'>-5&deg;F</button></a>";
    html += "<a href='/increase_meat'><button class='btn-plus' title='Increase Meat'>+5&deg;F</button></a>";
    html += "</td></tr>";
  html += "<tr><td>Meat 2</td><td>%T2_TEMP% &deg;F</td><td></td><td></td></tr>";
  // ETA row for meat temp prediction
  html += "<tr><td colspan='4' style='text-align:center;font-weight:bold;'>ETA to Meat Target: %MEAT_ETA%</td></tr>";
    html += "<tr><td>Runtime</td><td colspan='3'>%UPTIME%</td></tr>";
    html += "<tr><td>Heater Power</td><td colspan='3'>%HEATER_POWER%</td></tr>";
    html += "</table>";
    html += "</body></html>";

    // Replace placeholders with filtered readings
    float tempThermocoupleF, tempThermistor1F, tempThermistor2F;
    readAllTemperatures(tempThermocoupleF, tempThermistor1F, tempThermistor2F);

    html.replace("%TC_TEMP%", String((int)round(tempThermocoupleF)));
    html.replace("%T1_TEMP%", String((int)round(tempThermistor1F)));
    html.replace("%T2_TEMP%", String((int)round(tempThermistor2F)));
    html.replace("%SET_TEMP%", String((int)round(smokerTemp)));
    html.replace("%MEAT_TEMP%", String((int)round(meatDoneTemp)));
    // ETA prediction for web
    String etaStr = "--";
    if (predictionValid) {
      int min = (int)round(lastMeatTempPredictionMinutes);
      if (min < 1000) {
        etaStr = String(min) + " min";
      } else {
        etaStr = ">999 min";
      }
    }
    html.replace("%MEAT_ETA%", etaStr);
    html.replace("%UPTIME%", getUptime());
    html.replace("%HEATER_POWER%", String((int)round(pidOutputPercent)) + "%");

    request->send(200, "text/html", html);
  });

  server.on("/increase", HTTP_GET, [](AsyncWebServerRequest *request){
    smokerTemp += TEMP_STEP;
    if (smokerTemp > SMOKER_TEMP_MAX) {
      smokerTemp = SMOKER_TEMP_MAX;
    }
    pidSetpoint = smokerTemp; // Update PID setpoint immediately
    resetPID(); // Reset PID integral
    request->redirect("/");
  });

  server.on("/decrease", HTTP_GET, [](AsyncWebServerRequest *request){
    smokerTemp -= TEMP_STEP;
    if (smokerTemp < SMOKER_TEMP_MIN) {
      smokerTemp = SMOKER_TEMP_MIN;
    }
    pidSetpoint = smokerTemp; // Update PID setpoint immediately
    resetPID(); // Reset PID integral
    request->redirect("/");
  });

  server.on("/increase_meat", HTTP_GET, [](AsyncWebServerRequest *request){
    meatDoneTemp += TEMP_STEP;
    if (meatDoneTemp > SMOKER_TEMP_MAX) {
      meatDoneTemp = SMOKER_TEMP_MAX;
    }
    request->redirect("/");
  });

  server.on("/decrease_meat", HTTP_GET, [](AsyncWebServerRequest *request){
    meatDoneTemp -= TEMP_STEP;
    if (meatDoneTemp < SMOKER_TEMP_MIN) {
      meatDoneTemp = SMOKER_TEMP_MIN;
    }
    request->redirect("/");
  });

  server.begin();
  
  // Teleplot uses Serial output - no special initialization needed
  debugPrintln("Setup complete - Teleplot ready via Serial Monitor");
  debugPrintln("Install Teleplot extension in VS Code and connect to Serial port");
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA updates
  
  // Handle telnet connections for wireless serial monitoring
  if (telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) {
      telnetClient.stop(); // Disconnect previous client
    }
    telnetClient = telnetServer.available();
    if (telnetClient) {
      telnetClient.println("Connected to ESP32 Smoker Controller");
      telnetClient.print("IP: ");
      telnetClient.println(WiFi.localIP());
      telnetClient.flush();
    }
  }
  
  unsigned long currentTime = millis();
  
  // Periodic telnet test message
  static unsigned long lastTelnetTest = 0;
  if (currentTime - lastTelnetTest > 60000) { // Every 60 seconds
    debugPrintln("Telnet test - " + String(millis()) + "ms uptime");
    debugPrintln("WiFi Signal: " + String(WiFi.RSSI()) + " dBm");
    debugPrintln("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
    lastTelnetTest = currentTime;
  }
  // static unsigned long debugCounter = 0;
  // debugCounter++; // DISABLED - was causing excessive debug output
 // if (debugCounter % 1000 == 0) {  // Print every 1000 loops
 //   debugPrint("DEBUG: Encoder check loop #");
 //   debugPrintln(String(debugCounter));
 // }
  
  // Rotary encoder handling - check for changes
  static long lastEncoderValue = 0;
  long currentEncoderValue = rotaryEncoder.readEncoder();
  // Check for encoder rotation
  if (currentEncoderValue != lastEncoderValue) {
    int delta = currentEncoderValue - lastEncoderValue;
    debugPrint("Encoder changed from ");
    debugPrint(String(lastEncoderValue));
    debugPrint(" to ");
    debugPrint(String(currentEncoderValue));
    debugPrint(" (delta: ");
    debugPrint(String(delta));
    debugPrintln(")");
    
    // Sync encoder value with smoker temp if they don't match
    if (!meatTempMode) {
      smokerTemp = currentEncoderValue;
      smokerTemp = constrain(smokerTemp, SMOKER_TEMP_MIN, SMOKER_TEMP_MAX);
      debugPrint("New smoker temperature setpoint: ");
      debugPrintln(String(smokerTemp));
    } else {
      // For meat temp mode, encoder value directly sets meat temperature
      meatDoneTemp = currentEncoderValue;
      meatDoneTemp = constrain(meatDoneTemp, 100.0, 250.0);
      debugPrint("New meat target temperature: ");
      debugPrintln(String(meatDoneTemp));
    }
    updateLCD = true;
    lastEncoderValue = currentEncoderValue;
  }
  
  // Check for encoder button press
  if (rotaryEncoder.isEncoderButtonClicked()) {
    debugPrintln("Encoder button clicked!");
    // You can add mode switching or other button functionality here
  }

  // Enhanced WiFi reconnection with stability monitoring
  static unsigned long lastWifiCheck = 0;
  static unsigned long lastWifiDisconnect = 0;
  static int disconnectCount = 0;
  
  if (currentTime - lastWifiCheck > 5000) { // Check every 5 seconds
    if (WiFi.status() != WL_CONNECTED) {
      debugPrintln("WiFi disconnected! Attempting reconnection...");
      disconnectCount++;
      lastWifiDisconnect = currentTime;
      
      // Log signal strength before reconnecting
      debugPrint("Last RSSI: ");
      debugPrint(String(WiFi.RSSI()));
      debugPrintln(" dBm");
      
      // Try different reconnection strategies
      if (disconnectCount % 3 == 1) {
        // First attempt: simple reconnect
        WiFi.reconnect();
      } else if (disconnectCount % 3 == 2) {
        // Second attempt: disconnect and reconnect
        WiFi.disconnect();
        delay(50);  // Reduced delay to minimize interrupt disruption
        WiFi.begin(ssid, password);
      } else {
        // Third attempt: full reset
        WiFi.mode(WIFI_OFF);
        delay(50);  // Reduced delay
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.begin(ssid, password);
      }
      
      // Wait for reconnection (up to 10 seconds) with shorter delays
      int retry_count = 0;
      while (WiFi.status() != WL_CONNECTED && retry_count < 40) {
        delay(250);  // Reduced from 500ms to 250ms
        retry_count++;
        if (retry_count % 4 == 0) debugPrint(".");  // Print less frequently
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        debugPrintln("\nReconnected to WiFi!");
        debugPrint("New IP: ");
        debugPrintln(WiFi.localIP().toString());
        debugPrint("Signal: ");
        debugPrint(String(WiFi.RSSI()));
        debugPrintln(" dBm");
        
        // Show which band we reconnected to
        int channel = WiFi.channel();
        debugPrint("Channel: ");
        debugPrint(String(channel));
        if (channel >= 1 && channel <= 14) {
          debugPrintln(" (2.4GHz)");
        } else if (channel >= 36 && channel <= 165) {
          debugPrintln(" (5GHz)");
        } else {
          debugPrintln(" (Unknown)");
        }
      } else {
        debugPrintln("\nFailed to reconnect!");
        return; // Skip this loop iteration
      }
    } else {
      // WiFi is connected, reset disconnect counter
      if (disconnectCount > 0) {
        debugPrint("WiFi stable. Total disconnects since boot: ");
        debugPrintln(String(disconnectCount));
        disconnectCount = 0;
      }
    }
    lastWifiCheck = currentTime;
  }
  
  // Skip processing if WiFi is not connected
  if (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    return;
  }

  // Read all temperatures using dedicated functions
  float tempThermocoupleF, tempThermistor1F, tempThermistor2F;
  readAllTemperatures(tempThermocoupleF, tempThermistor1F, tempThermistor2F);

  // Debug output for thermistor diagnostics (preserved from original)
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime > 30000) { // Every 30 seconds
    // Note: Debug output simplified since we don't have raw ADC values anymore
    debugPrint("T1 Temp: ");
    debugPrint(String(tempThermistor1F, 1));
    debugPrint("°F | T2 Temp: ");
    debugPrint(String(tempThermistor2F, 1));
    debugPrintln("°F");
    lastDebugTime = currentTime;
  }

  // Send data to Teleplot for real-time visualization
  static unsigned long lastTeleplotTime = 0;
  if (currentTime - lastTeleplotTime > 1000) { // Send every 1 second
    // Teleplot format: >variableName:value
    debugPrint(">thermocouple:");
    debugPrintln(String(tempThermocoupleF));
    debugPrint(">thermistor1:");
    debugPrintln(String(tempThermistor1F));
    debugPrint(">thermistor2:");
    debugPrintln(String(tempThermistor2F));
    debugPrint(">smoker_setpoint:");
    debugPrintln(String(smokerTemp));
    debugPrint(">meat_target:");
    debugPrintln(String(meatDoneTemp));
    
    // Encoder and button state monitoring
    //debugPrint(">button_state:");
    //debugPrintln(String(digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW ? 1.0 : 0.0));
    //debugPrint(">current_mode:");
    //debugPrintln(String(meatTempMode ? 1.0 : 0.0));  // 1 = meat mode, 0 = smoker mode
    
    lastTeleplotTime = currentTime;
  }

  // --- Meat temp prediction logic ---
  // Use thermistor 1 as the primary meat probe
  float currentMeatTemp = tempThermistor1F;
  if (currentTime - lastPredictionSampleTime > PREDICTION_INTERVAL_SECONDS * 1000UL) {
    meatTempHistory[meatTempHistoryIndex] = currentMeatTemp;
    meatTempTimestamps[meatTempHistoryIndex] = currentTime;
    meatTempHistoryIndex = (meatTempHistoryIndex + 1) % PREDICTION_WINDOW_MINUTES;
    if (meatTempHistoryCount < PREDICTION_WINDOW_MINUTES) meatTempHistoryCount++;
    lastPredictionSampleTime = currentTime;
  }

  // Only predict if we have at least 2 samples and meat temp is below target
  predictionValid = false;
  lastMeatTempPredictionMinutes = -1;
  if (meatTempHistoryCount >= 2 && currentMeatTemp < meatDoneTemp - 0.5) {
    // Find the oldest sample in the buffer
    int oldestIdx = (meatTempHistoryIndex + (PREDICTION_WINDOW_MINUTES - meatTempHistoryCount)) % PREDICTION_WINDOW_MINUTES;
    float oldestTemp = meatTempHistory[oldestIdx];
    unsigned long oldestTime = meatTempTimestamps[oldestIdx];
    float deltaTemp = currentMeatTemp - oldestTemp;
    float deltaTimeMin = (currentTime - oldestTime) / 60000.0;
    if (deltaTemp > 0.1 && deltaTimeMin > 0.5) { // Require at least 0.1°F rise and 0.5 min window
      float rate = deltaTemp / deltaTimeMin; // degF per min
      float remaining = meatDoneTemp - currentMeatTemp;
      if (rate > 0.01) { // Require at least 0.01°F/min
        lastMeatTempPredictionMinutes = remaining / rate;
        predictionValid = true;
      }
    }
  }

  // Check if both thermistors have reached the meat target temperature
  float diff1 = tempThermistor1F - meatDoneTemp;  // Positive if above target
  float diff2 = tempThermistor2F - meatDoneTemp;  // Positive if above target
  
  // Debug output for meat temp logic
  static unsigned long lastMeatDebugTime = 0;
  if (currentTime - lastMeatDebugTime > 5000) { // Every 5 seconds
    debugPrint("Meat Temp Logic - T1: "); debugPrint(String(tempThermistor1F, 1));
    debugPrint("°F (over target: "); debugPrint(String(diff1, 1)); debugPrint("°F), T2: ");
    debugPrint(String(tempThermistor2F, 1)); debugPrint("°F (over target: "); debugPrint(String(diff2, 1));
    debugPrint("°F), Target: "); debugPrint(String(meatDoneTemp, 1)); debugPrintln("°F");
    if (predictionValid) {
      debugPrint("Estimated time to target: ");
      debugPrint(String(lastMeatTempPredictionMinutes, 1));
      debugPrintln(" min");
    } else {
      debugPrintln("Estimated time to target: --");
    }
    lastMeatDebugTime = currentTime;
  }
  
  // If both thermistors are at or above target (within 5°F below OR any amount above)
  if ((diff1 >= -5.0) && (diff2 >= -5.0)) {
    static bool meatTempReached = false;
    if (!meatTempReached) {
      debugPrintln("*** MEAT TEMPERATURE REACHED! Setting smoker temp to meat temp ***");
      meatTempReached = true;
    }
    smokerTemp = meatDoneTemp;
    pidSetpoint = smokerTemp; // Update PID setpoint immediately
    resetPID(); // Reset PID integral to stabilize at meatDoneTemp
  }

  // Encoder Button Handling (mode switch) - using direct GPIO reading for reliability
  static unsigned long lastButtonPressTime = 0;
  static bool lastButtonState = HIGH; // HIGH = not pressed (pullup)
  static bool buttonProcessed = false;
  const unsigned long BUTTON_DEBOUNCE_MS = 50; // Reduced from 300ms to 50ms for faster response
  
  // Read button state directly from GPIO pin
  bool currentButtonState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  
  // Reset processed flag when button is released (goes HIGH)
  if (currentButtonState == HIGH) {
    buttonProcessed = false;
  }
  
  // Detect button press (HIGH to LOW transition) with debouncing
  if (currentButtonState == LOW && lastButtonState == HIGH && !buttonProcessed) {
    unsigned long currentTime = millis();
    if (currentTime - lastButtonPressTime > BUTTON_DEBOUNCE_MS) {
      // Button just pressed and debounce period has passed
      meatTempMode = !meatTempMode;
      lastButtonActivity = currentTime;
      lastButtonPressTime = currentTime;
      buttonProcessed = true;
      
      // Sync encoder value with the new mode's temperature
      if (meatTempMode) {
        rotaryEncoder.setEncoderValue((long)meatDoneTemp);
      } else {
        rotaryEncoder.setEncoderValue((long)smokerTemp);
      }
      
      // Debug output to serial
      debugPrint("DIRECT BUTTON PRESS! Mode changed to: ");
      debugPrintln(meatTempMode ? "Meat Temp" : "Smoker Temp");
      debugPrint("Encoder synced to: ");
      debugPrintln(String(meatTempMode ? (long)meatDoneTemp : (long)smokerTemp));
      
      // Send button press and mode change to Teleplot
      debugPrint(">button_pressed:");
      debugPrintln("1.0");  // Spike to indicate button press
      debugPrint(">mode_changed:");
      debugPrintln(meatTempMode ? "1.0" : "0.0");  // 1 = meat mode, 0 = smoker mode
    }
  }
  
  lastButtonState = currentButtonState;


  // Mode Switching Logic
  // Two modes: Smoker Temp Control vs Meat Temp Monitoring
  // - Smoker mode: PID controls smoker temperature, encoder adjusts smoker setpoint
  // - Meat mode: PID still controls smoker temp, but encoder adjusts meat target for monitoring
  // Switch modes by pressing encoder button

  // PID Temperature Control
  // Input: Current thermocouple temperature (°F)
  // Setpoint: Target smoker temperature (°F)
  // Output: Heater power percentage (0-100%)
  float pidOutput = updatePID(tempThermocoupleF, smokerTemp);

  // Control heater using time-proportional relay control
  controlHeater(pidOutput);

  // Send PID telemetry data for debugging and monitoring
  sendPIDTelemetry();

  // Display on 20x4 LCD with optimized updates
  static int lastTempTC = -999;
  static int lastTemp1 = -999;
  static int lastTemp2 = -999;
  static int lastSmokerTemp = -999;
  static int lastMeatTemp = -999;
  static int lastPidOutput = -999;
  static bool lastMode = !meatTempMode;  // Initialize different to force first update
  static String lastUptime = "";  // Track uptime changes

  // Update only changed values
  int currentTempTC = (int)round(tempThermocoupleF);
  if (currentTempTC != lastTempTC) {
    lcd.setCursor(8, 0);  // Position after "Smoker: "
    lcd.print("    ");     // Clear previous value (4 spaces to ensure complete clearing)
    lcd.setCursor(8, 0);
    lcd.print(currentTempTC);
    lastTempTC = currentTempTC;
  }

  // Update PID output on line 1, far right
  int currentPidOutput = (int)round(pidOutput);
  if (currentPidOutput != lastPidOutput) {
    lcd.setCursor(12, 0);  // Position after temperature value with some space
    lcd.print("        ");  // Clear previous value (8 spaces for "Pwr:XXX%")
    lcd.setCursor(12, 0);
    lcd.print("Pwr:");
    lcd.print(currentPidOutput);
    lcd.print("%");
    lastPidOutput = currentPidOutput;
  }

  int currentTemp1 = (int)round(tempThermistor1F);
  if (currentTemp1 != lastTemp1) {
    lcd.setCursor(4, 1);  // Position after "T1: " (back to original position)
    lcd.print("    ");    // Clear previous value
    lcd.setCursor(4, 1);
    lcd.print(currentTemp1);
    lastTemp1 = currentTemp1;
  }

  // Update uptime on line 1, right side (every minute to avoid too frequent updates)
  String currentUptime = getUptimeLCD();
  if (currentUptime != lastUptime) {
    lcd.setCursor(10, 1);  // Move one position left (position 10-18 for 9 chars "Time:HH:MM")
    lcd.print("          ");  // Clear the right side (10 spaces to be safe)
    lcd.setCursor(10, 1);
    lcd.print(currentUptime);
    // Ensure we don't overrun - the string should be exactly 9 chars: "Time:HH:MM"
    lastUptime = currentUptime;
  }

  int currentTemp2 = (int)round(tempThermistor2F);
  if (currentTemp2 != lastTemp2) {
    lcd.setCursor(4, 2);  // Position after "T2: "
    lcd.print("    ");    // Clear previous value
    lcd.setCursor(4, 2);
    lcd.print(currentTemp2);
    lastTemp2 = currentTemp2;
  }

  // --- LCD: Show meat temp prediction as TLeft:HH:MM right-justified on row 2 (3rd row, 0-based) ---
  static float lastDisplayedPrediction = -999;
  static bool lastDisplayedPredictionValid = false;
  static String lastDisplayedTimeStr = "";
  String timeStr = "";
  if (predictionValid) {
    int min = (int)round(lastMeatTempPredictionMinutes);
    if (min < 10000) {
      int hours = min / 60;
      int mins = min % 60;
      char buf[10];
      snprintf(buf, sizeof(buf), "%02d:%02d", hours, mins);
      timeStr = String(buf);
    } else {
      timeStr = ">99:59";
    }
  } else {
    timeStr = "--:--";
  }
  // Only update if changed
  if (predictionValid != lastDisplayedPredictionValid || (predictionValid && fabs(lastMeatTempPredictionMinutes - lastDisplayedPrediction) > 0.5) || timeStr != lastDisplayedTimeStr) {
    // Format: 'TLeft:HH:MM' right-justified (end at col 19)
    String label = "TLeft:";
    String fullStr = label + timeStr;
    int startCol = 20 - fullStr.length();
    if (startCol < 0) startCol = 0;
    lcd.setCursor(startCol, 2);
    for (int i = startCol; i < 20; ++i) lcd.print(" "); // Clear to end
    lcd.setCursor(startCol, 2);
    lcd.print(fullStr);
    lastDisplayedPrediction = lastMeatTempPredictionMinutes;
    lastDisplayedPredictionValid = predictionValid;
    lastDisplayedTimeStr = timeStr;
  }

  // Bottom row updates
  int currentSmokerTemp = (int)round(smokerTemp);
  if (currentSmokerTemp != lastSmokerTemp) {
    lcd.setCursor(2, 3);  // Position after "S:"
    lcd.print("   ");     // Clear previous value
    lcd.setCursor(2, 3);
    lcd.print(currentSmokerTemp);
    lastSmokerTemp = currentSmokerTemp;
  }

  int currentMeatTempInt = (int)round(meatDoneTemp);
  if (currentMeatTempInt != lastMeatTemp) {
    lcd.setCursor(8, 3);  // Position after "M:"
    lcd.print("   ");     // Clear previous value
    lcd.setCursor(8, 3);
    lcd.print(currentMeatTempInt);
    lastMeatTemp = currentMeatTempInt;
  }

  if (meatTempMode != lastMode) {
    lcd.setCursor(14, 3);  // Position after "M:"
    lcd.print("      "); // Clear previous value (up to 7 chars)
    lcd.setCursor(14, 3);
    lcd.print(meatTempMode ? "Meat" : "Smoker");
    lastMode = meatTempMode;
  }

  // Initialize display if it hasn't been done (first run)
  static bool firstRun = true;
  if (firstRun) {
    lcd.clear();
    // Write static labels
    lcd.setCursor(0, 0); lcd.print("Smoker: ");
    lcd.setCursor(12, 0); lcd.print("Pwr: ");
    // Line 1 will show "T1: XXX      Time:HH:MM"
    lcd.setCursor(0, 1); lcd.print("T1: ");
    lcd.setCursor(0, 2); lcd.print("T2: ");
    lcd.setCursor(0, 3); lcd.print("S:");
  lcd.setCursor(6, 3); lcd.print("M:");
  lcd.setCursor(12, 3); lcd.print("M:");
    
    // Force initial values to display
    lcd.setCursor(8, 0); lcd.print((int)round(tempThermocoupleF));
    lcd.setCursor(12, 0); lcd.print("Pwr:"); lcd.print((int)round(pidOutput)); lcd.print("%");
    lcd.setCursor(4, 1); lcd.print((int)round(tempThermistor1F));
    lcd.setCursor(10, 1); lcd.print(getUptimeLCD());  // Moved to position 10
    lcd.setCursor(4, 2); lcd.print((int)round(tempThermistor2F));
    lcd.setCursor(2, 3); lcd.print((int)round(smokerTemp));
    lcd.setCursor(8, 3); lcd.print((int)round(meatDoneTemp));
  lcd.setCursor(14, 3); lcd.print(meatTempMode ? "Meat" : "Smoker");
    
    // Update last values to match what we just displayed
    lastTempTC = (int)round(tempThermocoupleF);
    lastTemp1 = (int)round(tempThermistor1F);
    lastTemp2 = (int)round(tempThermistor2F);
    lastSmokerTemp = (int)round(smokerTemp);
    lastMeatTemp = (int)round(meatDoneTemp);
    lastPidOutput = (int)round(pidOutput);
    lastMode = meatTempMode;
    lastUptime = getUptimeLCD();
    
    firstRun = false;
  }
  //lcd.print(" R:");
 // lcd.print(relayState ? "ON" : "OFF");

  // Serial output
  /*
  debugPrintln("-------------------");
  debugPrint("Thermocouple: "); debugPrint(String((int)round(tempThermocoupleF))); debugPrintln(" F");
  debugPrint("Thermistor 1: "); debugPrint(String((int)round(tempThermistor1F))); debugPrintln(" F");
  debugPrint("Thermistor 2: "); debugPrint(String((int)round(tempThermistor2F))); debugPrintln(" F");
  debugPrint("Smoker Setpoint: "); debugPrint(String((int)round(smokerTemp))); debugPrintln(" F");
  debugPrint("Meat Done Temp: "); debugPrint(String(meatDoneTemp)); debugPrintln(" F");
  debugPrint("Mode: "); debugPrintln(meatTempMode ? "Meat Temp" : "Smoker Temp");
  debugPrint("Button State: "); 
  ButtonState libraryButtonState = rotaryEncoder.readButtonState();
  switch(libraryButtonState) {
    case BUT_DOWN: debugPrint("DOWN"); break;
    case BUT_PUSHED: debugPrint("PUSHED"); break;
    case BUT_UP: debugPrint("UP"); break;
    case BUT_RELEASED: debugPrint("RELEASED"); break;
    case BUT_DISABLED: debugPrint("DISABLED"); break;
    default: debugPrint("UNKNOWN"); break;
  }
  debugPrint(" | Direct GPIO: ");
  debugPrintln(digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW ? "PRESSED" : "NOT_PRESSED");
  debugPrint("Uptime: "); debugPrintln(getUptime());
  debugPrint("PID Output: "); debugPrint(String(pidOutput)); debugPrintln(" %");
  debugPrint("Rotary Encoder: "); debugPrintln(String(rotaryEncoder.readEncoder()));
  
  // DEBUG: Show raw GPIO states every time we print status (COMMENTED OUT - was causing freeze)
  /*
  bool pinA = digitalRead(ROTARY_ENCODER_A_PIN);
  bool pinB = digitalRead(ROTARY_ENCODER_B_PIN);  
  bool pinBtn = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  debugPrint("GPIO A:"); debugPrint(pinA ? "H" : "L");
  debugPrint(" B:"); debugPrint(pinB ? "H" : "L"); 
  debugPrint(" Btn:"); debugPrintln(pinBtn ? "H" : "L");
  debugPrint("Encoder Button Raw: "); debugPrintln(rotaryEncoder.isEncoderButtonDown() ? "Pressed" : "Not Pressed");
  */
  debugPrint("Relay: "); debugPrintln(relayState ? "ON" : "OFF");

  delay(50);
}
