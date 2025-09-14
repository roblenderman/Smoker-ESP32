#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AiEsp32RotaryEncoder.h>

// Wi-Fi credentials
const char* ssid = "LHome";
const char* password = "stacey8561";

// Thermocouple (SPI)
int thermoCLK = 18; // SCLK
int thermoCS = 5;   // CS
int thermoDO = 19;  // MISO
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// I2C LCD (20x4)
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20x4 LCD

// Relay (SLA-05VDC-SL-C, low-level trigger)
const int relayPin = 16;

// Voltage Divider (Thermistor 1)
const int thermistorPin1 = 32; // ADC1
const float R_FIXED1 = 10000.0; // 10kΩ
const float V_IN = 3.3; // 3.3V
const float R_25 = 110000.0; // 110kΩ at 25°C
const float BETA = 3950.0; // Adjust if known

// Voltage Divider (Thermistor 2)
const int thermistorPin2 = 33; // ADC1
const float R_FIXED2 = 10000.0; // 10kΩ



// Rotary Encoder Pins (updated - avoid GPIO 12 for boot issues)
const int ROTARY_ENCODER_A_PIN = 13;  // Back to 13 for wiring check
const int ROTARY_ENCODER_B_PIN = 14;  // Swapped with A pin to fix backwards response
const int ROTARY_ENCODER_BUTTON_PIN = 25;
const int ROTARY_ENCODER_VCC_PIN = -1; // Not used
const int ROTARY_ENCODER_STEPS = 4;

AiEsp32RotaryEncoder rotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS, false); // false = use pullup resistors

// Smoker and meat setpoints
float smokerTemp = 200.0; // °F, initial setpoint
float meatDoneTemp = 165.0; // °F, initial meat done temp
const float SMOKER_TEMP_MIN = 150.0; // °F
const float SMOKER_TEMP_MAX = 350.0; // °F
const float TEMP_STEP = 5.0; // °F increment

// LCD update flag
bool updateLCD = false;

// PID parameters
const float Kp = 7.0;
const float Ki = 0.1;
const float Kd = 100.0;
float integral = 0.0;
float prevError = 0.0;
const float WINDOW_SIZE = 10000; // 10s window in ms
unsigned long windowStartTime = 0;
unsigned long lastPidTime = 0;  // Track last PID update time
bool relayState = false;

// Button hold-to-adjust (legacy - no longer used)
const unsigned long REPEAT_DELAY = 200; // ms between adjustments
const unsigned long MODE_TIMEOUT = 5000; // 5s inactivity to exit meat temp mode
const unsigned long MODE_ENTER_HOLD = 1000; // 1s hold to enter meat temp mode
unsigned long lastButtonActivity = 0;
bool meatTempMode = false;

// Web server
AsyncWebServer server(80);

float calculateTemp(float r) {
  float t25 = 298.15; // 25°C in Kelvin
  float tempK = 1.0 / ( (1.0 / t25) + (log(r / R_25) / BETA) );
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
void testEncoderGPIO() {
  static unsigned long lastTest = 0;
  if (millis() - lastTest > 500) {  // Test every 500ms
    lastTest = millis();
    
    bool pinA = digitalRead(ROTARY_ENCODER_A_PIN);
    bool pinB = digitalRead(ROTARY_ENCODER_B_PIN);
    bool pinBtn = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
    long encoderValue = rotaryEncoder.readEncoder();
    
    // Only print if something changed
    static bool lastA = true, lastB = true, lastBtn = true;
    static long lastEnc = 0;
    
    if (pinA != lastA || pinB != lastB || pinBtn != lastBtn || encoderValue != lastEnc) {
      Serial.print("GPIO Change - A:");
      Serial.print(pinA ? "H" : "L");
      Serial.print(" B:");
      Serial.print(pinB ? "H" : "L");
      Serial.print(" Btn:");
      Serial.print(pinBtn ? "H" : "L");
      Serial.print(" Enc:");
      Serial.println(encoderValue);
      
      lastA = pinA;
      lastB = pinB;
      lastBtn = pinBtn;
      lastEnc = encoderValue;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize Rotary Encoder BEFORE WiFi to avoid interrupt conflicts
  rotaryEncoder.begin();
  rotaryEncoder.isButtonPulldown = false; // Use pullup for button
  rotaryEncoder.setup(
    []{ rotaryEncoder.readEncoder_ISR(); },  // Encoder ISR
    []{ rotaryEncoder.readButton_ISR(); }    // Button ISR
  );
  rotaryEncoder.setBoundaries(SMOKER_TEMP_MIN, SMOKER_TEMP_MAX, false);
  rotaryEncoder.setAcceleration(250);
  
  // Set initial encoder value to match current smoker temp
  rotaryEncoder.setEncoderValue(smokerTemp);

  // Attach interrupts for rotary encoder pins with proper error handling
  if (digitalPinToInterrupt(ROTARY_ENCODER_A_PIN) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), [](){ rotaryEncoder.readEncoder_ISR(); }, CHANGE);
  }
  if (digitalPinToInterrupt(ROTARY_ENCODER_B_PIN) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), [](){ rotaryEncoder.readEncoder_ISR(); }, CHANGE);
  }
  
  // Attach interrupt for encoder button - use FALLING edge to reduce bounce
  if (digitalPinToInterrupt(ROTARY_ENCODER_BUTTON_PIN) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_BUTTON_PIN), [](){ rotaryEncoder.readButton_ISR(); }, FALLING);
  }
  
  // Allow time for encoder to stabilize
  delay(100);
  
  Serial.println("Rotary encoder initialized");
  
  // Test encoder GPIO pins during startup
  Serial.println("=== ENCODER GPIO TEST ===");
  for (int i = 0; i < 10; i++) {
    bool pinA = digitalRead(ROTARY_ENCODER_A_PIN);
    bool pinB = digitalRead(ROTARY_ENCODER_B_PIN);
    bool pinBtn = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
    Serial.print("Test ");
    Serial.print(i);
    Serial.print(": A=");
    Serial.print(pinA ? "H" : "L");
    Serial.print(", B=");
    Serial.print(pinB ? "H" : "L");
    Serial.print(", Btn=");
    Serial.print(pinBtn ? "H" : "L");
    Serial.print(", Encoder=");
    Serial.println(rotaryEncoder.readEncoder());
    delay(100);
  }
  Serial.println("=== END GPIO TEST ===");
  Serial.println("Try turning encoder now and watch for GPIO changes...");

  // Initialize I2C
  Wire.begin(21, 22); // SDA GPIO21, SCL GPIO22

  // Initialize Wi-Fi with stability improvements
  WiFi.mode(WIFI_STA);  // Set to station mode only
  WiFi.setAutoReconnect(true);  // Enable auto-reconnect
  WiFi.persistent(true);  // Save WiFi config to flash
  
  // Disable power saving mode for better stability
  WiFi.setSleep(false);
  
  // Set static IP to reduce DHCP issues (optional)
  // IPAddress local_IP(192, 168, 1, 100);
  // IPAddress gateway(192, 168, 1, 1);
  // IPAddress subnet(255, 255, 255, 0);
  // WiFi.config(local_IP, gateway, subnet);
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  int wifi_retry = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retry < 30) {
    delay(1000);
    Serial.print(".");
    wifi_retry++;
    
    // If taking too long, try reconnecting
    if (wifi_retry % 10 == 0) {
      Serial.println();
      Serial.print("Retry attempt ");
      Serial.print(wifi_retry / 10);
      Serial.print(" - Reconnecting...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, password);
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    // Determine WiFi band based on channel
    int channel = WiFi.channel();
    Serial.print("Channel: ");
    Serial.print(channel);
    if (channel >= 1 && channel <= 14) {
      Serial.println(" (2.4GHz band)");
    } else if (channel >= 36 && channel <= 165) {
      Serial.println(" (5GHz band)");
    } else {
      Serial.println(" (Unknown band)");
    }
    
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("\nFailed to connect to WiFi!");
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 1);
  lcd.print("IP: ");
  lcd.print(WiFi.localIP());
  delay(3000); // Show IP for 3s

  // Initialize Relay
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Relay off

  // Set ADC attenuation
  analogSetAttenuation(ADC_11db);

  // Initialize PID timing
  windowStartTime = millis();
  lastPidTime = millis();

  // Setup web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html>";
    html += "<head><meta http-equiv='refresh' content='5'>";
    html += "<title>Smoker Control</title>";
    html += "<style>";
    html += "body { font-family: Arial; text-align: center; margin: 50px; }";
    html += "h1 { color: #333; }";
    html += "p { font-size: 1.2em; }";
    html += "button { padding: 10px 20px; margin: 10px; font-size: 1em; }";
    html += "</style></head>";
    html += "<body>";
    html += "<h1>Smoker Control</h1>";
    html += "<p>Thermocouple: %TC_TEMP% &deg;F</p>";
    html += "<p>Thermistor 1: %T1_TEMP% &deg;F</p>";
    html += "<p>Thermistor 2: %T2_TEMP% &deg;F</p>";
    html += "<p>Smoker Setpoint: %SET_TEMP% &deg;F</p>";
    html += "<p>Meat Done Temp: %MEAT_TEMP% &deg;F</p>";
    html += "<p>Uptime: %UPTIME%</p>";
    html += "<p>Relay: %RELAY_STATE%</p>";
    html += "<a href='/increase'><button>Increase Smoker (+5&deg;F)</button></a>";
    html += "<a href='/decrease'><button>Decrease Smoker (-5&deg;F)</button></a>";
    html += "<a href='/increase_meat'><button>Increase Meat (+5&deg;F)</button></a>";
    html += "<a href='/decrease_meat'><button>Decrease Meat (-5&deg;F)</button></a>";
    html += "</body></html>";

    // Replace placeholders
    float tempThermocoupleF = thermocouple.readCelsius() * 9.0 / 5.0 + 32.0;
    int raw1 = 0;
    for (int i = 0; i < 10; i++) {
      raw1 += analogRead(thermistorPin1);
      delay(10);
    }
    raw1 /= 10;
    float vOut1 = (raw1 / 4095.0) * V_IN;
    float rThermistor1 = (vOut1 * R_FIXED1) / (V_IN - vOut1);
    if (rThermistor1 <= 0) rThermistor1 = 1.0; // Prevent invalid resistance
    float tempThermistor1F = calculateTemp(rThermistor1);

    int raw2 = 0;
    for (int i = 0; i < 10; i++) {
      raw2 += analogRead(thermistorPin2);
      delay(10);
    }
    raw2 /= 10;
    float vOut2 = (raw2 / 4095.0) * V_IN;
    float rThermistor2 = (vOut2 * R_FIXED2) / (V_IN - vOut2);
    if (rThermistor2 <= 0) rThermistor2 = 1.0; // Prevent invalid resistance
    float tempThermistor2F = calculateTemp(rThermistor2);

    html.replace("%TC_TEMP%", String((int)round(tempThermocoupleF)));
    html.replace("%T1_TEMP%", String((int)round(tempThermistor1F)));
    html.replace("%T2_TEMP%", String((int)round(tempThermistor2F)));
    html.replace("%SET_TEMP%", String((int)round(smokerTemp)));
    html.replace("%MEAT_TEMP%", String(meatDoneTemp, 1));
    html.replace("%UPTIME%", getUptime());
    html.replace("%RELAY_STATE%", relayState ? "ON" : "OFF");

    request->send(200, "text/html", html);
  });

  server.on("/increase", HTTP_GET, [](AsyncWebServerRequest *request){
    smokerTemp += TEMP_STEP;
    if (smokerTemp > SMOKER_TEMP_MAX) {
      smokerTemp = SMOKER_TEMP_MAX;
    }
    integral = 0.0; // Reset integral
    request->redirect("/");
  });

  server.on("/decrease", HTTP_GET, [](AsyncWebServerRequest *request){
    smokerTemp -= TEMP_STEP;
    if (smokerTemp < SMOKER_TEMP_MIN) {
      smokerTemp = SMOKER_TEMP_MIN;
    }
    integral = 0.0; // Reset integral
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
  Serial.println("Setup complete - Teleplot ready via Serial Monitor");
  Serial.println("Install Teleplot extension in VS Code and connect to Serial port");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Test encoder GPIO pins (remove this line once encoder is working)
  testEncoderGPIO();
  
  // DEBUG: Simple proof this code runs
  static unsigned long debugCounter = 0;
  debugCounter++;
  if (debugCounter % 1000 == 0) {  // Print every 1000 loops
    Serial.print("DEBUG: Encoder check loop #");
    Serial.println(debugCounter);
  }
  
  // Rotary encoder handling - check for changes
  static long lastEncoderValue = 0;
  long currentEncoderValue = rotaryEncoder.readEncoder();
  
  // Check for encoder rotation
  if (currentEncoderValue != lastEncoderValue) {
    int delta = currentEncoderValue - lastEncoderValue;
    Serial.print("Encoder changed from ");
    Serial.print(lastEncoderValue);
    Serial.print(" to ");
    Serial.print(currentEncoderValue);
    Serial.print(" (delta: ");
    Serial.print(delta);
    Serial.println(")");
    
    // Adjust temperature based on rotation and current mode
    if (meatTempMode) {
      // In meat mode - adjust meat target temperature
      if (delta > 0) {
        meatDoneTemp += 5.0;
        Serial.println("Meat target temperature increased by 5°F");
      } else {
        meatDoneTemp -= 5.0;
        Serial.println("Meat target temperature decreased by 5°F");
      }
      
      // Constrain meat temperature to reasonable limits
      meatDoneTemp = constrain(meatDoneTemp, 100.0, 200.0);
      Serial.print("New meat target temperature: ");
      Serial.println(meatDoneTemp);
    } else {
      // In smoker mode - adjust smoker temperature
      if (delta > 0) {
        smokerTemp += 5.0;
        Serial.println("Smoker temperature increased by 5°F");
      } else {
        smokerTemp -= 5.0;
        Serial.println("Smoker temperature decreased by 5°F");
      }
      
      // Constrain smoker temperature to reasonable limits
      smokerTemp = constrain(smokerTemp, 100.0, 400.0);
      Serial.print("New smoker temperature setpoint: ");
      Serial.println(smokerTemp);
    }
    
    // Update LCD immediately
    updateLCD = true;
    
    lastEncoderValue = currentEncoderValue;
  }
  
  // Check for encoder button press
  if (rotaryEncoder.isEncoderButtonClicked()) {
    Serial.println("Encoder button clicked!");
    // You can add mode switching or other button functionality here
  }

  // Enhanced WiFi reconnection with stability monitoring
  static unsigned long lastWifiCheck = 0;
  static unsigned long lastWifiDisconnect = 0;
  static int disconnectCount = 0;
  
  if (currentTime - lastWifiCheck > 5000) { // Check every 5 seconds
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Attempting reconnection...");
      disconnectCount++;
      lastWifiDisconnect = currentTime;
      
      // Log signal strength before reconnecting
      Serial.print("Last RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      
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
        if (retry_count % 4 == 0) Serial.print(".");  // Print less frequently
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nReconnected to WiFi!");
        Serial.print("New IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        
        // Show which band we reconnected to
        int channel = WiFi.channel();
        Serial.print("Channel: ");
        Serial.print(channel);
        if (channel >= 1 && channel <= 14) {
          Serial.println(" (2.4GHz)");
        } else if (channel >= 36 && channel <= 165) {
          Serial.println(" (5GHz)");
        } else {
          Serial.println(" (Unknown)");
        }
      } else {
        Serial.println("\nFailed to reconnect!");
        return; // Skip this loop iteration
      }
    } else {
      // WiFi is connected, reset disconnect counter
      if (disconnectCount > 0) {
        Serial.print("WiFi stable. Total disconnects since boot: ");
        Serial.println(disconnectCount);
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

  // Read thermocouple
  float tempThermocouple = thermocouple.readCelsius();
  float tempThermocoupleF = tempThermocouple * 9.0 / 5.0 + 32.0;

  // Read thermistor 1 (average for stability)
  int raw1 = 0;
  for (int i = 0; i < 10; i++) {
    raw1 += analogRead(thermistorPin1);
    delay(10);
  }
  raw1 /= 10;
  float vOut1 = (raw1 / 4095.0) * V_IN;
  float rThermistor1 = (vOut1 * R_FIXED1) / (V_IN - vOut1);
  if (rThermistor1 <= 0) rThermistor1 = 1.0; // Prevent invalid resistance
  float tempThermistor1F = calculateTemp(rThermistor1);

  // Read thermistor 2 (average for stability)
  int raw2 = 0;
  for (int i = 0; i < 10; i++) {
    raw2 += analogRead(thermistorPin2);
    delay(10);
  }
  raw2 /= 10;
  float vOut2 = (raw2 / 4095.0) * V_IN;
  float rThermistor2 = (vOut2 * R_FIXED2) / (V_IN - vOut2);
  if (rThermistor2 <= 0) rThermistor2 = 1.0; // Prevent invalid resistance
  float tempThermistor2F = calculateTemp(rThermistor2);

  // Send data to Teleplot for real-time visualization
  static unsigned long lastTeleplotTime = 0;
  if (currentTime - lastTeleplotTime > 1000) { // Send every 1 second
    // Teleplot format: >variableName:value
    Serial.print(">thermocouple:");
    Serial.println(tempThermocoupleF);
    Serial.print(">thermistor1:");
    Serial.println(tempThermistor1F);
    Serial.print(">thermistor2:");
    Serial.println(tempThermistor2F);
    Serial.print(">smoker_setpoint:");
    Serial.println(smokerTemp);
    Serial.print(">meat_target:");
    Serial.println(meatDoneTemp);
    
    // Encoder and button state monitoring
    Serial.print(">button_state:");
    Serial.println(digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW ? 1.0 : 0.0);
    Serial.print(">current_mode:");
    Serial.println(meatTempMode ? 1.0 : 0.0);  // 1 = meat mode, 0 = smoker mode
    
    lastTeleplotTime = currentTime;
  }

  // Check if both thermistors have reached the meat target temperature
  float diff1 = tempThermistor1F - meatDoneTemp;  // Positive if above target
  float diff2 = tempThermistor2F - meatDoneTemp;  // Positive if above target
  
  // Debug output for meat temp logic
  static unsigned long lastMeatDebugTime = 0;
  if (currentTime - lastMeatDebugTime > 5000) { // Every 5 seconds
    Serial.print("Meat Temp Logic - T1: "); Serial.print(tempThermistor1F, 1);
    Serial.print("°F (over target: "); Serial.print(diff1, 1); Serial.print("°F), T2: ");
    Serial.print(tempThermistor2F, 1); Serial.print("°F (over target: "); Serial.print(diff2, 1);
    Serial.print("°F), Target: "); Serial.print(meatDoneTemp, 1); Serial.println("°F");
    lastMeatDebugTime = currentTime;
  }
  
  // If both thermistors are at or above target (within 5°F below OR any amount above)
  if ((diff1 >= -5.0) && (diff2 >= -5.0)) {
    static bool meatTempReached = false;
    if (!meatTempReached) {
      Serial.println("*** MEAT TEMPERATURE REACHED! Setting smoker temp to meat temp ***");
      meatTempReached = true;
    }
    smokerTemp = meatDoneTemp;
    integral = 0.0; // Reset integral to stabilize at meatDoneTemp
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
      
      // Debug output to serial
      Serial.print("DIRECT BUTTON PRESS! Mode changed to: ");
      Serial.println(meatTempMode ? "Meat Temp" : "Smoker Temp");
      
      // Send button press and mode change to Teleplot
      Serial.print(">button_pressed:");
      Serial.println(1.0);  // Spike to indicate button press
      Serial.print(">mode_changed:");
      Serial.println(meatTempMode ? 1.0 : 0.0);  // 1 = meat mode, 0 = smoker mode
    }
  }
  
  lastButtonState = currentButtonState;


  // No automatic exit from meatTempMode; mode is switched only by encoder button

  // PID control
  float error = smokerTemp - tempThermocoupleF;
  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0; // Time since last PID update
  if (dt == 0) dt = 0.001; // Avoid division by zero, use small value
  lastPidTime = now;

  // Proportional
  float P = Kp * error;

  // Integral with anti-windup
  integral += Ki * error * dt;
  float output = P + integral;
  if (output > 100.0) {
    integral -= Ki * error * dt; // Undo integral update
    output = 100.0;
  } else if (output < 0.0) {
    integral -= Ki * error * dt;
    output = 0.0;
  }

  // Derivative
  float D = Kd * (error - prevError) / dt;
  prevError = error;

  // Total PID output
  output = P + integral + D;
  if (output > 100.0) output = 100.0;
  if (output < 0.0) output = 0.0;

  // Update Teleplot with PID control data
  static unsigned long lastPidTeleplotTime = 0;
  if (currentTime - lastPidTeleplotTime > 1000) { // Send every 1 second
    Serial.print(">pid_output:");
    Serial.println(output);
    Serial.print(">pid_error:");
    Serial.println(error);
    Serial.print(">pid_P:");
    Serial.println(P);
    Serial.print(">pid_I:");
    Serial.println(integral);
    Serial.print(">pid_D:");
    Serial.println(D);
    lastPidTeleplotTime = currentTime;
  }

  // Time-proportional relay control
  float onTime = (output / 100.0) * WINDOW_SIZE; // ms
  if (currentTime - windowStartTime <= onTime) {
    digitalWrite(relayPin, LOW); // Relay on
    relayState = true;
  } else {
    digitalWrite(relayPin, HIGH); // Relay off
    relayState = false;
  }
  
  // Update Teleplot with relay state
  static unsigned long lastRelayTeleplotTime = 0;
  if (currentTime - lastRelayTeleplotTime > 500) { // Send every 0.5 seconds for relay
    Serial.print(">relay_state:");
    Serial.println(relayState ? 100.0 : 0.0);
    Serial.print(">relay_on_time_ms:");
    Serial.println(onTime);
    lastRelayTeleplotTime = currentTime;
  }

  // Reset window if time exceeds WINDOW_SIZE
  if (currentTime - windowStartTime >= WINDOW_SIZE) {
    windowStartTime = currentTime;
    prevError = error; // Update prevError at window reset
  }

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
    lcd.print("   ");     // Clear previous value (reduced to 3 spaces to make room for Power)
    lcd.setCursor(8, 0);
    lcd.print(currentTempTC);
    lastTempTC = currentTempTC;
  }

  // Update PID output on line 1, far right
  int currentPidOutput = (int)round(output);
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

  // Bottom row updates
  int currentSmokerTemp = (int)round(smokerTemp);
  if (currentSmokerTemp != lastSmokerTemp) {
    lcd.setCursor(2, 3);  // Position after "S:"
    lcd.print("   ");     // Clear previous value
    lcd.setCursor(2, 3);
    lcd.print(currentSmokerTemp);
    lastSmokerTemp = currentSmokerTemp;
  }

  int currentMeatTemp = (int)round(meatDoneTemp);
  if (currentMeatTemp != lastMeatTemp) {
    lcd.setCursor(8, 3);  // Position after "M:"
    lcd.print("   ");     // Clear previous value
    lcd.setCursor(8, 3);
    lcd.print(currentMeatTemp);
    lastMeatTemp = currentMeatTemp;
  }

  if (meatTempMode != lastMode) {
    lcd.setCursor(14, 3);  // Position after "B:"
    lcd.print(meatTempMode ? "M" : "S");
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
    lcd.setCursor(12, 3); lcd.print("B:");
    
    // Force initial values to display
    lcd.setCursor(8, 0); lcd.print((int)round(tempThermocoupleF));
    lcd.setCursor(12, 0); lcd.print("Pwr:"); lcd.print((int)round(output)); lcd.print("%");
    lcd.setCursor(4, 1); lcd.print((int)round(tempThermistor1F));
    lcd.setCursor(10, 1); lcd.print(getUptimeLCD());  // Moved to position 10
    lcd.setCursor(4, 2); lcd.print((int)round(tempThermistor2F));
    lcd.setCursor(2, 3); lcd.print((int)round(smokerTemp));
    lcd.setCursor(8, 3); lcd.print((int)round(meatDoneTemp));
    lcd.setCursor(14, 3); lcd.print(meatTempMode ? "M" : "S");
    
    // Update last values to match what we just displayed
    lastTempTC = (int)round(tempThermocoupleF);
    lastTemp1 = (int)round(tempThermistor1F);
    lastTemp2 = (int)round(tempThermistor2F);
    lastSmokerTemp = (int)round(smokerTemp);
    lastMeatTemp = (int)round(meatDoneTemp);
    lastPidOutput = (int)round(output);
    lastMode = meatTempMode;
    lastUptime = getUptimeLCD();
    
    firstRun = false;
  }
  //lcd.print(" R:");
 // lcd.print(relayState ? "ON" : "OFF");

  // Serial output
  Serial.println("-------------------");
  Serial.print("Thermocouple: "); Serial.print((int)round(tempThermocoupleF)); Serial.println(" F");
  Serial.print("Thermistor 1: "); Serial.print((int)round(tempThermistor1F)); Serial.println(" F");
  Serial.print("Thermistor 2: "); Serial.print((int)round(tempThermistor2F)); Serial.println(" F");
  Serial.print("Smoker Setpoint: "); Serial.print((int)round(smokerTemp)); Serial.println(" F");
  Serial.print("Meat Done Temp: "); Serial.print(meatDoneTemp); Serial.println(" F");
  Serial.print("Mode: "); Serial.println(meatTempMode ? "Meat Temp" : "Smoker Temp");
  Serial.print("Button State: "); 
  ButtonState libraryButtonState = rotaryEncoder.readButtonState();
  switch(libraryButtonState) {
    case BUT_DOWN: Serial.print("DOWN"); break;
    case BUT_PUSHED: Serial.print("PUSHED"); break;
    case BUT_UP: Serial.print("UP"); break;
    case BUT_RELEASED: Serial.print("RELEASED"); break;
    case BUT_DISABLED: Serial.print("DISABLED"); break;
    default: Serial.print("UNKNOWN"); break;
  }
  Serial.print(" | Direct GPIO: ");
  Serial.println(digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW ? "PRESSED" : "NOT_PRESSED");
  Serial.print("Uptime: "); Serial.println(getUptime());
  Serial.print("PID Output: "); Serial.print(output); Serial.println(" %");
  Serial.print("Rotary Encoder: "); Serial.println(rotaryEncoder.readEncoder());
  
  // DEBUG: Show raw GPIO states every time we print status
  bool pinA = digitalRead(ROTARY_ENCODER_A_PIN);
  bool pinB = digitalRead(ROTARY_ENCODER_B_PIN);  
  bool pinBtn = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  Serial.print("GPIO A:"); Serial.print(pinA ? "H" : "L");
  Serial.print(" B:"); Serial.print(pinB ? "H" : "L"); 
  Serial.print(" Btn:"); Serial.println(pinBtn ? "H" : "L");
  Serial.print("Encoder Button Raw: "); Serial.println(rotaryEncoder.isEncoderButtonDown() ? "Pressed" : "Not Pressed");
  Serial.print("Relay: "); Serial.println(relayState ? "ON" : "OFF");

  delay(300);
}