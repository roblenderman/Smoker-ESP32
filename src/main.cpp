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


// Rotary Encoder Pins
const int ROTARY_ENCODER_A_PIN = 26;
const int ROTARY_ENCODER_B_PIN = 27;
const int ROTARY_ENCODER_BUTTON_PIN = 25;
const int ROTARY_ENCODER_VCC_PIN = -1; // Not used
const int ROTARY_ENCODER_STEPS = 4;

AiEsp32RotaryEncoder rotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

// Smoker and meat setpoints
float smokerTemp = 200.0; // °F, initial setpoint
float meatDoneTemp = 165.0; // °F, initial meat done temp
const float SMOKER_TEMP_MIN = 150.0; // °F
const float SMOKER_TEMP_MAX = 350.0; // °F
const float TEMP_STEP = 5.0; // °F increment

// PID parameters
const float Kp = 7.0;
const float Ki = 0.1;
const float Kd = 100.0;
float integral = 0.0;
float prevError = 0.0;
const float WINDOW_SIZE = 10000; // 10s window in ms
unsigned long windowStartTime = 0;
bool relayState = false;

// Button hold-to-adjust
const unsigned long REPEAT_DELAY = 200; // ms between adjustments
const unsigned long MODE_TIMEOUT = 5000; // 5s inactivity to exit meat temp mode
const unsigned long MODE_ENTER_HOLD = 1000; // 1s hold to enter meat temp mode
unsigned long lastButton1Time = 0;
unsigned long lastButton2Time = 0;
unsigned long lastButtonActivity = 0;
bool button1LastState = LOW;  // Initial state is LOW (not pressed)
bool button2LastState = LOW;  // Initial state is LOW (not pressed)
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

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(21, 22); // SDA GPIO21, SCL GPIO22

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

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

  // Initialize Rotary Encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup([]{ rotaryEncoder.readEncoder_ISR(); });
  rotaryEncoder.setBoundaries(SMOKER_TEMP_MIN, SMOKER_TEMP_MAX, false);
  rotaryEncoder.setAcceleration(250);
  
  // Set ADC attenuation
  analogSetAttenuation(ADC_11db);

  // Initialize PID timing
  windowStartTime = millis();

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
    float tempThermistor1F = calculateTemp(rThermistor1);

    int raw2 = 0;
    for (int i = 0; i < 10; i++) {
      raw2 += analogRead(thermistorPin2);
      delay(10);
    }
    raw2 /= 10;
    float vOut2 = (raw2 / 4095.0) * V_IN;
    float rThermistor2 = (vOut2 * R_FIXED2) / (V_IN - vOut2);
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
}

void loop() {
  unsigned long currentTime = millis();

  // Reconnect WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
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
  float tempThermistor2F = calculateTemp(rThermistor2);

  // Check if both thermistors are within ±5°F of meatDoneTemp
  if (abs(tempThermistor1F - meatDoneTemp) <= 5.0 && abs(tempThermistor2F - meatDoneTemp) <= 5.0) {
    smokerTemp = meatDoneTemp;
    integral = 0.0; // Reset integral to stabilize at meatDoneTemp
  }


  // Rotary Encoder Handling
  static int32_t lastEncoderValue = 0;
  rotaryEncoder.loop();
  int32_t encoderValue = rotaryEncoder.readEncoder();
  if (encoderValue != lastEncoderValue) {
    int32_t diff = encoderValue - lastEncoderValue;
    if (meatTempMode) {
      meatDoneTemp += diff * TEMP_STEP;
      if (meatDoneTemp > SMOKER_TEMP_MAX) meatDoneTemp = SMOKER_TEMP_MAX;
      if (meatDoneTemp < SMOKER_TEMP_MIN) meatDoneTemp = SMOKER_TEMP_MIN;
    } else {
      smokerTemp += diff * TEMP_STEP;
      if (smokerTemp > SMOKER_TEMP_MAX) smokerTemp = SMOKER_TEMP_MAX;
      if (smokerTemp < SMOKER_TEMP_MIN) smokerTemp = SMOKER_TEMP_MIN;
      integral = 0.0; // Reset integral
    }
    lastEncoderValue = encoderValue;
    lastButtonActivity = currentTime;
  }

  // Encoder Button Handling (mode switch)
  static bool lastButtonState = HIGH;
  bool buttonState = rotaryEncoder.isEncoderButtonDown();
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Button pressed (falling edge)
    meatTempMode = !meatTempMode;
    lastButtonActivity = currentTime;
    delay(200); // Debounce
  }
  lastButtonState = buttonState;


  // No automatic exit from meatTempMode; mode is switched only by encoder button

  // PID control
  float error = smokerTemp - tempThermocoupleF;
  float dt = (currentTime - windowStartTime) / 1000.0; // Approximate dt in seconds
  if (dt == 0) dt = 1.0; // Avoid division by zero

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

  // Time-proportional relay control
  float onTime = (output / 100.0) * WINDOW_SIZE; // ms
  if (currentTime - windowStartTime <= onTime) {
    digitalWrite(relayPin, LOW); // Relay on
    relayState = true;
  } else {
    digitalWrite(relayPin, HIGH); // Relay off
    relayState = false;
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
  static bool lastMode = !meatTempMode;  // Initialize different to force first update

  // Update only changed values
  int currentTempTC = (int)round(tempThermocoupleF);
  if (currentTempTC != lastTempTC) {
    lcd.setCursor(8, 0);  // Position after "Smoker: "
    lcd.print("    ");    // Clear previous value
    lcd.setCursor(8, 0);
    lcd.print(currentTempTC);
    lastTempTC = currentTempTC;
  }

  int currentTemp1 = (int)round(tempThermistor1F);
  if (currentTemp1 != lastTemp1) {
    lcd.setCursor(4, 1);  // Position after "T1: "
    lcd.print("    ");    // Clear previous value
    lcd.setCursor(4, 1);
    lcd.print(currentTemp1);
    lastTemp1 = currentTemp1;
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
    lcd.setCursor(0, 1); lcd.print("T1: ");
    lcd.setCursor(0, 2); lcd.print("T2: ");
    lcd.setCursor(0, 3); lcd.print("S:");
    lcd.setCursor(6, 3); lcd.print("M:");
    lcd.setCursor(12, 3); lcd.print("B:");
    
    // Force initial values to display
    lcd.setCursor(8, 0); lcd.print((int)round(tempThermocoupleF));
    lcd.setCursor(4, 1); lcd.print((int)round(tempThermistor1F));
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
    lastMode = meatTempMode;
    
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
  Serial.print("Uptime: "); Serial.println(getUptime());
  Serial.print("PID Output: "); Serial.print(output); Serial.println(" %");
  Serial.print("Rotary Encoder: "); Serial.println(rotaryEncoder.readEncoder());
  Serial.print("Encoder Button: "); Serial.println(rotaryEncoder.isEncoderButtonDown() ? "Pressed" : "Not Pressed");
  Serial.print("Relay: "); Serial.println(relayState ? "ON" : "OFF");

  delay(300);
}