/*
 * ESP32 Smoker Temperature Controller
 * Full version with non-blocking WiFi and complete web interface
 */
#include <WiFi.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AiEsp32RotaryEncoder.h>
#include <movingAvg.h>
#include <ArduinoOTA.h>
#include <ArduPID.h>
#include <ESPmDNS.h>

// Wi-Fi credentials
const char* ssid = "LHome";
const char* password = "stacey8561";

// Telnet server
WiFiServer telnetServer(23);
WiFiClient telnetClient;

// Thermocouple (SPI)
int thermoCLK = 18;
int thermoCS = 5;
int thermoDO = 19;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Calibration
const float THERMOCOUPLE_OFFSET_F = 0.0;
const float THERMISTOR1_OFFSET_F = 0.0;
const float THERMISTOR2_OFFSET_F = 0.0;

// ADC
const int ADC_MAX_VALUE = 4095;
const int THERMISTOR_SAMPLES = 20;
const int THERMISTOR_SAMPLE_DELAY_MS = 5;

// Thermocouple
const int THERMOCOUPLE_SAMPLES = 2;
const int THERMOCOUPLE_SAMPLE_DELAY_MS = 300;

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Relay
const int relayPin = 16;

// Thermistor 1
const int THERMISTOR1_PIN = 32;
const float THERMISTOR_FIXED_RESISTOR_OHMS = 12340.0;
const float ADC_REFERENCE_VOLTAGE = 3.3;
const float THERMISTOR_NOMINAL_RESISTANCE = 124500.0;
const float THERMISTOR_BETA_COEFFICIENT = 3960.0;

// Thermistor 2
const int THERMISTOR2_PIN = 33;
const float THERMISTOR2_FIXED_RESISTOR_OHMS = 12340.0;

// Filters
movingAvg thermistor1Filter(7);
movingAvg thermistor2Filter(7);
movingAvg thermocoupleFilter(4);

// Encoder
const int ROTARY_ENCODER_A_PIN = 13;
const int ROTARY_ENCODER_B_PIN = 14;
const int ROTARY_ENCODER_BUTTON_PIN = 25;
const int ROTARY_ENCODER_VCC_PIN = -1;
const int ROTARY_ENCODER_STEPS = 4;

AiEsp32RotaryEncoder rotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS, false);

void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

// Setpoints
float smokerTemp = 250.0;
float meatDoneTemp = 170.0;
const float SMOKER_TEMP_MIN = 100.0;
const float SMOKER_TEMP_MAX = 350.0;
bool smokerEnabled = true;
const float TEMP_STEP = 5.0;

// Prediction
const int PREDICTION_WINDOW_MINUTES = 5;
const int PREDICTION_INTERVAL_SECONDS = 30;
float meatTempHistory[5];
unsigned long meatTempTimestamps[5];
int meatTempHistoryIndex = 0;
int meatTempHistoryCount = 0;
unsigned long lastPredictionSampleTime = 0;
float lastMeatTempPredictionMinutes = -1;
bool predictionValid = false;

// PID
double pidSetpoint, pidInput, pidOutput;
double Kp = 13, Ki = 0.18, Kd = 35;
ArduPID myPID;

const unsigned long PID_TELEPLOT_INTERVAL_MS = 5000;
const float PID_OUTPUT_MIN = 30.0;
const float PID_OUTPUT_MAX = 100.0;

const float WINDOW_SIZE = 10000;
unsigned long windowStartTime = 0;
bool relayState = false;
float pidOutputPercent = 0.0;

unsigned long lastButtonActivity = 0;
bool meatTempHoldMode = false;
bool meatTempMode = false;

// Data logging
struct DataPoint {
  unsigned long timestamp;
  float smokerActualTemp;
  float smokerSetpointTemp;
  float meatTemp;
  float powerPercent;
};

#define MAX_DATA_POINTS 1024
DataPoint dataBuffer[MAX_DATA_POINTS];
int dataIndex = 0;
int dataCount = 0;
unsigned long lastDataLogTime = 0;

// Web server
AsyncWebServer server(80);

// --- NON-BLOCKING WIFI ---
enum WiFiState { WIFI_IDLE, WIFI_START, WIFI_WAIT_FOR_CONNECT, WIFI_RETRY };
WiFiState wifiState = WIFI_IDLE;
unsigned long wifiLastCheck = 0;
int wifiRetryCount = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000;
const int MAX_RETRIES = 3;

void handleWiFiNonBlocking();

// --- DEBUG ---
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

// --- TEMP CALC ---
float calculateTemp(float r) {
  float t25 = 292.04;
  float tempK = 1.0 / ((1.0 / t25) + (log(r / THERMISTOR_NOMINAL_RESISTANCE) / THERMISTOR_BETA_COEFFICIENT));
  float tempC = tempK - 273.15;
  return (tempC - 0.07 * (tempC - 19.0)) * 9.0 / 5.0 + 32.0;
}

// --- UPTIME ---
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
  String timeStr = "Time:";
  if (hours < 10) timeStr += "0";
  timeStr += String(hours) + ":";
  if (minutes < 10) timeStr += "0";
  timeStr += String(minutes);
  return timeStr;
}

// --- TEMPERATURE ---
float readThermocoupleTemperature() {
  float sum = 0;
  int valid = 0;
  for (int i = 0; i < THERMOCOUPLE_SAMPLES; i++) {
    float r = thermocouple.readCelsius();
    if (!isnan(r)) { sum += r; valid++; }
    delay(THERMOCOUPLE_SAMPLE_DELAY_MS);
  }
  float tempC = valid > 0 ? thermocoupleFilter.reading(sum / valid) : thermocoupleFilter.reading(thermocouple.readCelsius());
  return (tempC * 9.0 / 5.0 + 32.0) - THERMOCOUPLE_OFFSET_F;
}

float readThermistorTemperature(int pin, float fixedResistorOhms, float offsetF) {
  int rawSum = 0;
  for (int i = 0; i < THERMISTOR_SAMPLES; i++) {
    rawSum += analogRead(pin);
    delay(THERMISTOR_SAMPLE_DELAY_MS);
  }
  int rawAvg = rawSum / THERMISTOR_SAMPLES;
  int filtered = (pin == THERMISTOR1_PIN) ? thermistor1Filter.reading(rawAvg) : thermistor2Filter.reading(rawAvg);
  float voltage = (filtered / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;
  float resistance = (voltage * fixedResistorOhms) / (ADC_REFERENCE_VOLTAGE - voltage);
  if (resistance <= 0) resistance = 1.0;
  return calculateTemp(resistance) - offsetF;
}

void readAllTemperatures(float& tc, float& t1, float& t2) {
  tc = readThermocoupleTemperature();
  t1 = readThermistorTemperature(THERMISTOR1_PIN, THERMISTOR_FIXED_RESISTOR_OHMS, THERMISTOR1_OFFSET_F);
  t2 = readThermistorTemperature(THERMISTOR2_PIN, THERMISTOR2_FIXED_RESISTOR_OHMS, THERMISTOR2_OFFSET_F);
  if (t1 < 32.0) t1 = 32.0;
  if (t2 < 32.0) t2 = 32.0;
}

// --- PID ---
void initializePID() {
  myPID.begin(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd);
  myPID.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  myPID.setWindUpLimits(0, 60);
  myPID.setSampleTime(0);
  myPID.start();
}

float updatePID(float current, float setpoint) {
  pidInput = current;
  pidSetpoint = setpoint;
  float minOut = meatTempHoldMode ? 0.0 : PID_OUTPUT_MIN;
  myPID.setOutputLimits(minOut, PID_OUTPUT_MAX);
  myPID.compute();
  pidOutputPercent = pidOutput;
  return pidOutput;
}

void controlHeater(float output) {
  unsigned long now = millis();
  unsigned long elapsed = now - windowStartTime;
  if (elapsed >= WINDOW_SIZE) windowStartTime = now;
  float onTime = (output / 100.0) * WINDOW_SIZE;
  bool shouldOn = (elapsed < onTime);
  static bool last = false;
  if (shouldOn != last) {
    digitalWrite(relayPin, shouldOn ? HIGH : LOW);
    relayState = shouldOn;
    last = shouldOn;
  }
}

void resetPID() { myPID.reset(); }

// --- WIFI NON-BLOCKING ---
void handleWiFiNonBlocking() {
  unsigned long now = millis();
  if (now - wifiLastCheck < WIFI_CHECK_INTERVAL) return;
  wifiLastCheck = now;

  switch (wifiState) {
    case WIFI_IDLE:
      if (WiFi.status() != WL_CONNECTED) {
        debugPrintln("WiFi lost – starting reconnect");
        wifiState = WIFI_START;
        wifiRetryCount = 0;
      }
      break;
    case WIFI_START:
      WiFi.disconnect();
      delay(10);
      WiFi.begin(ssid, password);
      debugPrintln("WiFi.begin() called");
      wifiState = WIFI_WAIT_FOR_CONNECT;
      break;
    case WIFI_WAIT_FOR_CONNECT:
      if (WiFi.status() == WL_CONNECTED) {
        debugPrintln("\nWiFi re-connected!");
        debugPrint("IP: "); debugPrintln(WiFi.localIP().toString());
        debugPrint("RSSI: "); debugPrint(String(WiFi.RSSI())); debugPrintln(" dBm");  // FIXED
        ArduinoOTA.begin();
        wifiState = WIFI_IDLE;
        wifiRetryCount = 0;
      }
      break;
    case WIFI_RETRY:
      wifiRetryCount++;
      if (wifiRetryCount >= MAX_RETRIES) {
        debugPrintln("Too many failures – full WiFi reset");
        WiFi.mode(WIFI_OFF);
        delay(50);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        wifiRetryCount = 0;
      }
      wifiState = WIFI_START;
      break;
  }

  static unsigned long lastFail = 0;
  if (wifiState != WIFI_IDLE && WiFi.status() != WL_CONNECTED) {
    if (now - lastFail > 15000) {
      lastFail = now;
      debugPrintln("WiFi stalled – retry");
      wifiState = WIFI_RETRY;
    }
  } else {
    lastFail = now;
  }
}
// --- SETUP ---
void setup() {
  Serial.begin(115200);
  debugPrintln("ESP32 Starting - Debug Version");
  Serial.setDebugOutput(true);

  // Encoder
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR, readEncoderISR);
  rotaryEncoder.setBoundaries(SMOKER_TEMP_MIN, SMOKER_TEMP_MAX, false);
  rotaryEncoder.setAcceleration(0);
  rotaryEncoder.setEncoderValue(smokerTemp);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), readEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), readEncoderISR, CHANGE);
  delay(100);

  analogReadResolution(12);
  thermistor1Filter.begin();
  thermistor2Filter.begin();
  thermocoupleFilter.begin();

  Wire.begin(21, 22);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.setSleep(false);

  IPAddress local_IP(192, 168, 1, 225);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);
  IPAddress secondaryDNS(8, 8, 4, 4);
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  int wifi_retry = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retry < 30) {
    delay(1000); debugPrint("."); wifi_retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    debugPrintln("\nWiFi Connected");
    debugPrint("IP: "); debugPrintln(WiFi.localIP().toString());
    ArduinoOTA.setHostname("smoker");
    ArduinoOTA.setPort(3232);
    ArduinoOTA.begin();
    telnetServer.begin();
  } else {
    debugPrintln("\nWiFi Failed");
  }

  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("WiFi Connected");
  lcd.setCursor(0, 1); lcd.print("IP: "); lcd.print(WiFi.localIP());
  delay(5000);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  analogSetAttenuation(ADC_11db);

  pidSetpoint = smokerTemp;
  initializePID();
  windowStartTime = millis();

  // --- WEB SERVER ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    float tc, t1, t2;
    readAllTemperatures(tc, t1, t2);
    String html = R"rawliteral(
<!DOCTYPE html><html>
<head><meta http-equiv='refresh' content='30'><meta charset="UTF-8">
<title>Smoker Control</title>
<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>
<style>
  body { font-family: Arial; text-align: center; margin: 20px; background: #f7f7f7; }
  h1 { color: #333; margin-bottom: 20px; }
  table { margin: 0 auto 20px auto; border-collapse: collapse; background: #fff; box-shadow: 0 2px 8px #ccc; }
  th, td { padding: 12px 18px; border-bottom: 1px solid #eee; font-size: 1.1em; }
  th { background: #f0f0f0; }
  tr:last-child td { border-bottom: none; }
  button { padding: 6px 14px; margin: 0 4px; font-size: 1em; border-radius: 5px; border: 1px solid #888; background: #e0e0e0; cursor: pointer; transition: background 0.2s; }
  button:hover { background: #d0d0d0; }
  .btn-plus { background: #28a745; color: white; border: 1px solid #1e7e34; }
  .btn-plus:hover { background: #218838; }
  .btn-minus { background: #dc3545; color: white; border: 1px solid #bd2130; }
  .btn-minus:hover { background: #c82333; }
  .chart-container { width: 90%; max-width: 800px; height: 700px; margin: 20px auto; background: #fff; padding: 20px; border-radius: 10px; box-shadow: 0 2px 8px #ccc; display: flex; flex-direction: column; }
  .chart-controls button { margin: 0 5px; padding: 8px 16px; }
  #tempChart { width: 100%; flex-grow: 1; min-height: 600px; }
</style></head>
<body>
<h1>Smoker Control</h1>
<table>
<tr><th>Sensor</th><th>Value</th><th>Setpoint</th><th>Actions</th></tr>
<tr><td>Smoker</td><td>%TC_TEMP% °F</td><td>%SET_TEMP% °F</td>
<td><a href='/decrease'><button class='btn-minus'>-5°F</button></a>
<a href='/increase'><button class='btn-plus'>+5°F</button></a></td></tr>
<tr><td>Meat 1</td><td>%T1_TEMP% °F</td><td>%MEAT_TEMP% °F</td>
<td><a href='/decrease_meat'><button class='btn-minus'>-5°F</button></a>
<a href='/increase_meat'><button class='btn-plus'>+5°F</button></a></td></tr>
<tr><td>Meat 2</td><td>%T2_TEMP% °F</td><td></td><td></td></tr>
<tr><td colspan='4' style='text-align:center;font-weight:bold;'>ETA to Meat Target: %MEAT_ETA%</td></tr>
<tr><td>Runtime</td><td colspan='3'>%UPTIME%</td></tr>
<tr><td>Heater Power</td><td>%HEATER_POWER%</td>
<td colspan='2'>)rawliteral";

    if (smokerEnabled) {
      html += "<a href='/smoker/off'><button style='background:#4CAF50;color:white;'>On</button></a> ";
      html += "<a href='/smoker/off'><button style='background:#f44336;color:white;opacity:0.5;'>Off</button></a></td></tr>";
    } else {
      html += "<a href='/smoker/on'><button style='background:#4CAF50;color:white;opacity:0.5;'>On</button></a> ";
      html += "<a href='/smoker/on'><button style='background:#f44336;color:white;'>Off</button></a></td></tr>";
    }
    html += R"rawliteral(</table>
<div class='chart-container'>
<h2>Temperature History</h2>
<div class='chart-controls'>
<button onclick='updateChart("5m")'>5 Min</button>
<button onclick='updateChart("1h")'>1 Hour</button>
<button onclick='updateChart("4h")'>4 Hours</button>
<button onclick='updateChart("10h")'>10 Hours</button>
<button onclick='updateChart("all")'>All Data</button>
</div>
<canvas id='tempChart'></canvas>
</div>
<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.0'></script>
<script>
window.chart = null;
window.formatElapsedTime = function(ms) {
  const hours = Math.floor(ms / 3600000);
  const minutes = Math.floor((ms % 3600000) / 60000);
  const seconds = Math.floor((ms % 60000) / 1000);
  return hours + ':' + minutes.toString().padStart(2,'0') + ':' + seconds.toString().padStart(2,'0');
};
window.updateChart = async function(range) {
  try {
    const response = await fetch('/api/data?range=' + range);
    if (!response.ok) return false;
    const data = await response.json();
    if (data && data.data && data.data.length > 0) {
      const labels = data.data.map(d => formatElapsedTime(d.timestamp));
      chart.data.labels = labels;
      chart.data.datasets[0].data = data.data.map(d => d.smokerActualTemp);
      chart.data.datasets[1].data = data.data.map(d => d.meatTemp);
      chart.data.datasets[2].data = data.data.map(d => d.smokerSetpointTemp);
      chart.data.datasets[3].data = data.data.map(d => d.powerPercent);
      await chart.update('none');
      return true;
    }
  } catch (err) { console.error(err); }
  return false;
};
function initChart() {
  const ctx = document.getElementById('tempChart').getContext('2d');
  chart = new Chart(ctx, {
    type: 'line', data: { labels: [], datasets: [
      { label: 'Smoker Temp', borderColor: 'rgb(255, 99, 132)', yAxisID: 'y', tension: 0.1 },
      { label: 'Meat Temp', borderColor: 'rgb(54, 162, 235)', yAxisID: 'y1', tension: 0.1 },
      { label: 'Setpoint', borderColor: 'rgb(255, 205, 86)', tension: 0.1 },
      { label: 'Power %', borderColor: 'rgb(75, 192, 192)', yAxisID: 'y2', tension: 0.1 }
    ]}, options: {
      responsive: true, maintainAspectRatio: false,
      scales: {
        y: { position: 'left', title: { display: true, text: 'Temperature (°F)' }, min: 100, max: 375 },
        y1: { position: 'right', title: { display: true, text: 'Meat Temp (°F)' }, min: 30, max: 220, grid: { drawOnChartArea: false } },
        y2: { position: 'right', title: { display: true, text: 'Power (%)' }, min: 0, max: 100, grid: { drawOnChartArea: false } }
      }
    }
  });
}
document.addEventListener('DOMContentLoaded', async () => {
  initChart();
  const success = await updateChart('current');
  if (!success) await updateChart('all');
  setInterval(async () => { if (!await updateChart('current')) await updateChart('all'); }, 30000);
});
</script>
</body></html>
)rawliteral";

    html.replace("%TC_TEMP%", String((int)round(tc)));
    html.replace("%T1_TEMP%", String((int)round(t1)));
    html.replace("%T2_TEMP%", String((int)round(t2)));
    html.replace("%SET_TEMP%", String((int)round(smokerTemp)));
    html.replace("%MEAT_TEMP%", String((int)round(meatDoneTemp)));
    String etaStr = predictionValid ? String((int)round(lastMeatTempPredictionMinutes)) + " min" : "--";
    html.replace("%MEAT_ETA%", etaStr);
    html.replace("%UPTIME%", getUptime());
    html.replace("%HEATER_POWER%", String((int)round(pidOutputPercent)) + "%");
    request->send(200, "text/html", html);
  });

  server.on("/increase", HTTP_GET, [](AsyncWebServerRequest *r){ smokerTemp = constrain(smokerTemp + TEMP_STEP, SMOKER_TEMP_MIN, SMOKER_TEMP_MAX); pidSetpoint = smokerTemp; resetPID(); r->redirect("/"); });
  server.on("/decrease", HTTP_GET, [](AsyncWebServerRequest *r){ smokerTemp = constrain(smokerTemp - TEMP_STEP, SMOKER_TEMP_MIN, SMOKER_TEMP_MAX); pidSetpoint = smokerTemp; resetPID(); r->redirect("/"); });
  server.on("/increase_meat", HTTP_GET, [](AsyncWebServerRequest *r){ meatDoneTemp = constrain(meatDoneTemp + TEMP_STEP, SMOKER_TEMP_MIN, SMOKER_TEMP_MAX); r->redirect("/"); });
  server.on("/decrease_meat", HTTP_GET, [](AsyncWebServerRequest *r){ meatDoneTemp = constrain(meatDoneTemp - TEMP_STEP, SMOKER_TEMP_MIN, SMOKER_TEMP_MAX); r->redirect("/"); });
  server.on("/smoker/on", HTTP_GET, [](AsyncWebServerRequest *r){ smokerEnabled = true; r->redirect("/"); });
  server.on("/smoker/off", HTTP_GET, [](AsyncWebServerRequest *r){ smokerEnabled = false; r->redirect("/"); });

  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String range = request->hasParam("range") ? request->getParam("range")->value() : "all";
    unsigned long cutoff = 0;
    if (range == "5m") cutoff = millis() - 300000;
    else if (range == "1h") cutoff = millis() - 3600000;
    else if (range == "4h") cutoff = millis() - 14400000;
    else if (range == "10h") cutoff = millis() - 36000000;
    else if (range == "current") cutoff = millis() - 3600000;

    String json = "{\"data\":[";
    bool first = true;
    for (int i = 0; i < dataCount; i++) {
      int idx = dataCount < MAX_DATA_POINTS ? i : (dataIndex + i) % MAX_DATA_POINTS;
      DataPoint& p = dataBuffer[idx];
      if (range != "all" && p.timestamp < cutoff) continue;
      if (!first) json += ",";
      json += "{\"timestamp\":" + String(p.timestamp) + 
              ",\"smokerActualTemp\":" + String(p.smokerActualTemp,1) +
              ",\"smokerSetpointTemp\":" + String(p.smokerSetpointTemp,1) +
              ",\"meatTemp\":" + String(p.meatTemp,1) +
              ",\"powerPercent\":" + String(p.powerPercent,1) + "}";
      first = false;
    }
    json += "],\"count\":" + String(dataCount) + ",\"range\":\"" + range + "\"}";
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", json);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });

  server.begin();
}

// --- LOOP ---
void loop() {
  ArduinoOTA.handle();

  if (telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) telnetClient.stop();
    telnetClient = telnetServer.available();
    if (telnetClient) {
      telnetClient.println("Connected to Smoker Controller");
      telnetClient.print("IP: "); telnetClient.println(WiFi.localIP());
    }
  }

  unsigned long currentTime = millis();

  // Non-blocking WiFi
  if (currentTime - wifiLastCheck > 5000) {
    wifiLastCheck = currentTime;
    handleWiFiNonBlocking();
  }

  // Encoder
  static long lastEncoderValue = 0;
  long enc = rotaryEncoder.readEncoder();
  if (enc != lastEncoderValue) {
    if (!meatTempMode) {
      smokerTemp = enc;
      smokerTemp = constrain(smokerTemp, SMOKER_TEMP_MIN, SMOKER_TEMP_MAX);
      if (smokerTemp > meatDoneTemp + 5.0 && meatTempHoldMode) {
        myPID.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
        meatTempHoldMode = false;
      }
    } else {
      meatDoneTemp = enc;
      meatDoneTemp = constrain(meatDoneTemp, 100.0, 250.0);
    }
    lastEncoderValue = enc;
  }

  // Button
  static bool lastBtn = HIGH;
  bool btn = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  if (btn == LOW && lastBtn == HIGH) {
    delay(50);
    if (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW) {
      meatTempMode = !meatTempMode;
      rotaryEncoder.setEncoderValue(meatTempMode ? (long)meatDoneTemp : (long)smokerTemp);
    }
  }
  lastBtn = btn;

  if (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    return;
  }

  float tc, t1, t2;
  readAllTemperatures(tc, t1, t2);

  // Prediction
  float avgMeat = (t1 + t2) / 2.0;
  if (currentTime - lastPredictionSampleTime > PREDICTION_INTERVAL_SECONDS * 1000) {
    meatTempHistory[meatTempHistoryIndex] = avgMeat;
    meatTempTimestamps[meatTempHistoryIndex] = currentTime;
    meatTempHistoryIndex = (meatTempHistoryIndex + 1) % PREDICTION_WINDOW_MINUTES;
    if (meatTempHistoryCount < PREDICTION_WINDOW_MINUTES) meatTempHistoryCount++;
    lastPredictionSampleTime = currentTime;
  }
  predictionValid = false;
  if (meatTempHistoryCount >= 2 && avgMeat < meatDoneTemp - 0.5) {
    int oldest = (meatTempHistoryIndex + PREDICTION_WINDOW_MINUTES - meatTempHistoryCount) % PREDICTION_WINDOW_MINUTES;
    float deltaT = avgMeat - meatTempHistory[oldest];
    float deltaM = (currentTime - meatTempTimestamps[oldest]) / 60000.0;
    if (deltaT > 0.05 && deltaM > 0.25) {
      float rate = deltaT / deltaM;
      if (rate > 0.005) {
        lastMeatTempPredictionMinutes = (meatDoneTemp - avgMeat) / rate;
        predictionValid = true;
      }
    }
  }

  // Meat done logic
  if ((t1 >= meatDoneTemp - 10.0) && (t2 >= meatDoneTemp - 10.0)) {
    if (!meatTempHoldMode) {
      meatTempHoldMode = true;
      myPID.setOutputLimits(0.0, PID_OUTPUT_MAX);
    }
    smokerTemp = meatDoneTemp;
    pidSetpoint = smokerTemp;
    resetPID();
  }

  float pidOut = updatePID(tc, smokerTemp);
  if (!smokerEnabled) pidOut = 0;
  controlHeater(pidOut);

  // Data log
  if (currentTime - lastDataLogTime > 5000) {
    DataPoint p = { currentTime, tc, smokerTemp, avgMeat, pidOutputPercent };
    dataBuffer[dataIndex] = p;
    dataIndex = (dataIndex + 1) % MAX_DATA_POINTS;
    if (dataCount < MAX_DATA_POINTS) dataCount++;
    lastDataLogTime = currentTime;
  }

  // LCD (minimal updates)
  static int lastTC = -1, lastT1 = -1, lastT2 = -1, lastSet = -1, lastMeat = -1, lastPwr = -1;
  int iTC = round(tc), iT1 = round(t1), iT2 = round(t2), iSet = round(smokerTemp), iMeat = round(meatDoneTemp), iPwr = round(pidOutputPercent);
  if (iTC != lastTC) { lcd.setCursor(8,0); lcd.print("    "); lcd.setCursor(8,0); lcd.print(iTC); lastTC = iTC; }
  if (iPwr != lastPwr) { lcd.setCursor(12,0); lcd.print("        "); lcd.setCursor(12,0); lcd.print("Pwr:"); lcd.print(iPwr); lcd.print("%"); lastPwr = iPwr; }
  if (iT1 != lastT1) { lcd.setCursor(4,1); lcd.print("    "); lcd.setCursor(4,1); lcd.print(iT1); lastT1 = iT1; }
  if (iT2 != lastT2) { lcd.setCursor(4,2); lcd.print("    "); lcd.setCursor(4,2); lcd.print(iT2); lastT2 = iT2; }
  if (iSet != lastSet) { lcd.setCursor(2,3); lcd.print("   "); lcd.setCursor(2,3); lcd.print(iSet); lastSet = iSet; }
  if (iMeat != lastMeat) { lcd.setCursor(8,3); lcd.print("   "); lcd.setCursor(8,3); lcd.print(iMeat); lastMeat = iMeat; }

  static bool first = true;
  if (first) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Smoker: ");
    lcd.setCursor(0,1); lcd.print("T1: ");
    lcd.setCursor(0,2); lcd.print("T2: ");
    lcd.setCursor(0,3); lcd.print("S:"); lcd.setCursor(6,3); lcd.print("M:"); lcd.setCursor(12,3); lcd.print("M:");
    first = false;
  }

  delay(50);
}