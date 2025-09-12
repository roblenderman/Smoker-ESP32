#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

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

// Buttons
const int button1Pin = 26;  // Increase smokerTemp or meatDoneTemp
const int button2Pin = 27;  // Decrease smokerTemp or meatDoneTemp

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

  // Initialize Buttons with internal pull-down
  // Buttons will read HIGH when pressed (connected to 3.3V) and LOW when not pressed (pulled down)
  pinMode(button1Pin, INPUT_PULLDOWN);  // GPIO4 with internal pull-down
  pinMode(button2Pin, INPUT_PULLDOWN);  // GPIO5 with internal pull-down
  delay(50);  // Allow pins to stabilize
  button1LastState = digitalRead(button1Pin);  // Initialize with actual reading
  delay(10);
  button2LastState = digitalRead(button2Pin);  // Initialize with actual reading
  
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

  // Read buttons with hold-to-adjust and debounce
  bool button1Pressed = false;
  bool button2Pressed = false;
  
  // Read buttons directly (using internal pull-down)
  int button1State = digitalRead(button1Pin);  // HIGH when pressed (3.3V)
  delay(5);
  int button2State = digitalRead(button2Pin);  // HIGH when pressed (3.3V)
  delay(5);
  
  // Debug button states and pins with raw readings
  Serial.println("\n--- Button States ---");
  Serial.print("Button1 (Pin 26) State: "); Serial.print(button1State);
  Serial.print(" Digital: "); Serial.print(digitalRead(button1Pin));
  Serial.print(" Raw ADC: "); Serial.print(analogRead(button1Pin));
  Serial.print(" Voltage: "); Serial.print((analogRead(button1Pin) * 3.3) / 4095.0, 2);
  Serial.println("V");
  
  Serial.print("Button2 (Pin 27) State: "); Serial.print(button2State);
  Serial.print(" Digital: "); Serial.print(digitalRead(button2Pin));
  Serial.print(" Raw ADC: "); Serial.print(analogRead(button2Pin));
  Serial.print(" Voltage: "); Serial.print((analogRead(button2Pin) * 3.3) / 4095.0, 2);
  Serial.println("V");
  
  // Double check if both buttons are reported as pressed
  if (button1State == HIGH && button2State == HIGH) {
    // Verify again with a delay
    delay(10);
    if (digitalRead(button1Pin) != digitalRead(button2Pin)) {
      // If they're different now, update the states
      button1State = digitalRead(button1Pin);
      button2State = digitalRead(button2Pin);
    }
  }

  // Debounce both buttons
  if (button1State != button1LastState || button2State != button2LastState) {
    delay(50); // Debounce delay
    button1State = digitalRead(button1Pin);
    button2State = digitalRead(button2Pin);
  }

  // Check for meat temp mode entry/exit (both buttons pressed for 1s)
  static unsigned long bothButtonsStart = 0;
  if (button1State == HIGH && button2State == HIGH) {
    if (button1LastState == LOW && button2LastState == LOW) {
      bothButtonsStart = currentTime;
    } else if (currentTime - bothButtonsStart >= MODE_ENTER_HOLD) {
      meatTempMode = !meatTempMode; // Toggle mode
      bothButtonsStart = currentTime; // Prevent immediate re-toggle
      lastButtonActivity = currentTime;
      // Wait for button release
      while (digitalRead(button1Pin) == HIGH || digitalRead(button2Pin) == HIGH) {
        delay(10);
      }
    }
  }

  // Exit meat temp mode after 5s inactivity
  if (meatTempMode && currentTime - lastButtonActivity >= MODE_TIMEOUT) {
    meatTempMode = false;
  }

  // Button 1 (Increase smokerTemp or meatDoneTemp)
  if (button1State == HIGH && button1LastState == LOW) {
    delay(50); // Debounce
    if (digitalRead(button1Pin) == HIGH) {
      button1Pressed = true;
      if (meatTempMode) {
        meatDoneTemp += TEMP_STEP;
        if (meatDoneTemp > SMOKER_TEMP_MAX) {
          meatDoneTemp = SMOKER_TEMP_MAX;
        }
      } else {
        smokerTemp += TEMP_STEP;
        if (smokerTemp > SMOKER_TEMP_MAX) {
          smokerTemp = SMOKER_TEMP_MAX;
        }
        integral = 0.0; // Reset integral
      }
      lastButton1Time = currentTime;
      lastButtonActivity = currentTime;
    }
  } else if (button1State == HIGH && currentTime - lastButton1Time >= REPEAT_DELAY) {
    button1Pressed = true;
    if (meatTempMode) {
      meatDoneTemp += TEMP_STEP;
      if (meatDoneTemp > SMOKER_TEMP_MAX) {
        meatDoneTemp = SMOKER_TEMP_MAX;
      }
    } else {
      smokerTemp += TEMP_STEP;
      if (smokerTemp > SMOKER_TEMP_MAX) {
        smokerTemp = SMOKER_TEMP_MAX;
      }
      integral = 0.0; // Reset integral
    }
    lastButton1Time = currentTime;
    lastButtonActivity = currentTime;
  }

  // Button 2 (Decrease smokerTemp or meatDoneTemp)
  if (button2State == HIGH && button2LastState == LOW) {
    delay(50); // Debounce
    if (digitalRead(button2Pin) == HIGH) {
      button2Pressed = true;
      if (meatTempMode) {
        meatDoneTemp -= TEMP_STEP;
        if (meatDoneTemp < SMOKER_TEMP_MIN) {
          meatDoneTemp = SMOKER_TEMP_MIN;
        }
      } else {
        smokerTemp -= TEMP_STEP;
        if (smokerTemp < SMOKER_TEMP_MIN) {
          smokerTemp = SMOKER_TEMP_MIN;
        }
        integral = 0.0; // Reset integral
      }
      lastButton2Time = currentTime;
      lastButtonActivity = currentTime;
    }
  } else if (button2State == HIGH && currentTime - lastButton2Time >= REPEAT_DELAY) {
    button2Pressed = true;
    if (meatTempMode) {
      meatDoneTemp -= TEMP_STEP;
      if (meatDoneTemp < SMOKER_TEMP_MIN) {
        meatDoneTemp = SMOKER_TEMP_MIN;
      }
    } else {
      smokerTemp -= TEMP_STEP;
      if (smokerTemp < SMOKER_TEMP_MIN) {
        smokerTemp = SMOKER_TEMP_MIN;
      }
      integral = 0.0; // Reset integral
    }
    lastButton2Time = currentTime;
    lastButtonActivity = currentTime;
  }

  button1LastState = button1State;
  button2LastState = button2State;

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

  // Display on 20x4 LCD
  lcd.clear(); // Clear to avoid overlap
  // Row 1: Thermocouple temperature
  lcd.setCursor(0, 0);
  lcd.print("Smoker: ");
  lcd.print((int)round(tempThermocoupleF));
  lcd.print("F");

  // Row 2: Thermistor 1 temperature
  lcd.setCursor(0, 1);
  lcd.print("T1: ");
  lcd.print((int)round(tempThermistor1F));
  lcd.print("F");

  // Row 3: Thermistor 2 temperature
  lcd.setCursor(0, 2);
  lcd.print("T2: ");
  lcd.print((int)round(tempThermistor2F));
  lcd.print("F");

  // Row 4: Smoker setpoint, meat done temp, button mode, relay state
  lcd.setCursor(0, 3);
  lcd.print("S:");
  lcd.print((int)smokerTemp);
  lcd.print(" M:");
  lcd.print((int)meatDoneTemp);
  lcd.print(" B:");
  lcd.print(meatTempMode ? "M" : "S");
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
  Serial.print("Button 1: "); Serial.println(button1Pressed ? "Pressed" : "Not Pressed");
  Serial.print("Button 2: "); Serial.println(button2Pressed ? "Pressed" : "Not Pressed");
  Serial.print("Relay: "); Serial.println(relayState ? "ON" : "OFF");

  delay(300);
}