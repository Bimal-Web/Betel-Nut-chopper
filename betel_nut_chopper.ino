/*
  IoT Betel Nut Chopper - ESP32
  -----------------------------------------------------------
  Features: IR nut detection, L298N motor control, safety
  interlocks (E-stop, blade cover), overload/jam detection,
  production counter, OLED display, MQTT remote monitoring.

  Libraries (Library Manager):
    - PubSubClient            (Nick O'Leary)
    - Adafruit SSD1306
    - Adafruit GFX Library
    - ArduinoJson             (Benoit Blanchon)
  Board: "ESP32 Dev Module", ESP32 Arduino core 3.x
  (for core 2.x replace ledcAttach() with ledcSetup()+ledcAttachPin()
   and ledcWrite(CHANNEL, duty))

  WIRING
    IR sensor OUT   -> GPIO34 (LOW = nut present)
    L298N ENA       -> GPIO25 (PWM)
    L298N IN1       -> GPIO26
    L298N IN2       -> GPIO27
    E-stop button   -> GPIO32 to GND (normally-closed, opens on press)
    Cover reed sw   -> GPIO33 to GND (closes when cover is shut)
    ACS712 OUT      -> GPIO35 via voltage divider (5V -> 3.3V max!)
    OLED SDA/SCL    -> GPIO21 / GPIO22
    Status LED      -> GPIO2 (onboard)
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- USER SETTINGS ----------------
const char* WIFI_SSID   = "YOUR_WIFI_NAME";
const char* WIFI_PASS   = "YOUR_WIFI_PASSWORD";
const char* MQTT_SERVER = "broker.hivemq.com";   // public test broker
const int   MQTT_PORT   = 1883;
// Change this to something unique so others don't see your data
const char* TOPIC_BASE  = "bca_betel_chopper_1234";

const int   MOTOR_SPEED      = 200;     // PWM 0-255
const unsigned long CUT_TIME_MS      = 1200;   // blade run time per nut
const unsigned long COOLDOWN_MS      = 800;    // wait between nuts
const unsigned long DEBOUNCE_MS      = 60;

// Current sensing (ACS712-5A: 185 mV/A, centre = 2.5V; after 2:3 divider)
const float ADC_REF_V        = 3.3;
const float DIVIDER_FACTOR   = 1.5;     // (R1+R2)/R2 for 5V->3.3V divider
const float SENSITIVITY_V_A  = 0.185;
const float OVERLOAD_AMPS    = 4.0;     // tune for your motor
const unsigned long OVERLOAD_TIME_MS = 500;
// ------------------------------------------------

// Pins
#define PIN_IR      34
#define PIN_ENA     25
#define PIN_IN1     26
#define PIN_IN2     27
#define PIN_ESTOP   32
#define PIN_COVER   33
#define PIN_CURRENT 35
#define PIN_LED      2

#define PWM_FREQ 1000
#define PWM_RES  8

// OLED
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oledOK = false;

// MQTT
WiFiClient espClient;
PubSubClient mqtt(espClient);
String tCmd, tStatus, tAlert;

// State machine
enum State { IDLE, CUTTING, COOLDOWN, FAULT };
State state = IDLE;

enum Fault { F_NONE, F_ESTOP, F_COVER, F_OVERLOAD };
Fault fault = F_NONE;

bool autoEnabled = false;       // must be enabled (remote or local) to run
unsigned long cutCount = 0;
unsigned long stateStart = 0;
unsigned long overloadStart = 0;
unsigned long lastIrChange = 0;
unsigned long lastPublish = 0;
unsigned long lastDisplay = 0;
unsigned long runtimeMs = 0;    // total blade-on time
unsigned long lastTick = 0;
bool irStable = false;          // debounced "nut present"
bool irLastRaw = false;
float currentA = 0;
float zeroOffsetV = 2.5;        // calibrated at boot (motor off)

// ---------------- Helpers ----------------
const char* stateName() {
  switch (state) {
    case IDLE:     return autoEnabled ? "READY" : "STOPPED";
    case CUTTING:  return "CUTTING";
    case COOLDOWN: return "COOLDOWN";
    case FAULT:    return "FAULT";
  }
  return "?";
}

const char* faultName() {
  switch (fault) {
    case F_ESTOP:    return "EMERGENCY_STOP";
    case F_COVER:    return "COVER_OPEN";
    case F_OVERLOAD: return "OVERLOAD_JAM";
    default:         return "NONE";
  }
}

void motorStop() {
  ledcWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

void motorRun() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, MOTOR_SPEED);
}

bool estopPressed() { return digitalRead(PIN_ESTOP) == HIGH; }  // NC opens -> pull-up reads HIGH
bool coverOpen()    { return digitalRead(PIN_COVER) == HIGH; }  // reed closed -> LOW

float readCurrent() {
  // average 20 samples
  long sum = 0;
  for (int i = 0; i < 20; i++) sum += analogReadMilliVolts(PIN_CURRENT);
  float v = (sum / 20.0) / 1000.0 * DIVIDER_FACTOR;
  return fabs(v - zeroOffsetV) / SENSITIVITY_V_A;
}

void publishAlert(const char* msg) {
  if (mqtt.connected()) mqtt.publish(tAlert.c_str(), msg);
  Serial.printf("[ALERT] %s\n", msg);
}

void enterFault(Fault f) {
  motorStop();
  fault = f;
  state = FAULT;
  publishAlert(faultName());
}

void publishStatus() {
  if (!mqtt.connected()) return;
  StaticJsonDocument<256> doc;
  doc["state"]   = stateName();
  doc["fault"]   = faultName();
  doc["count"]   = cutCount;
  doc["current"] = round(currentA * 100) / 100.0;
  doc["hours"]   = runtimeMs / 3600000.0;
  doc["auto"]    = autoEnabled;
  char buf[256];
  serializeJson(doc, buf);
  mqtt.publish(tStatus.c_str(), buf);
}

// ---------------- MQTT ----------------
void onMessage(char* topic, byte* payload, unsigned int len) {
  String cmd;
  for (unsigned int i = 0; i < len; i++) cmd += (char)payload[i];
  cmd.trim();
  cmd.toLowerCase();
  Serial.printf("[CMD] %s\n", cmd.c_str());

  if (cmd == "start") {
    // Remote start only arms the machine. Physical safety still wins.
    if (!estopPressed() && !coverOpen()) autoEnabled = true;
    else publishAlert("CANNOT_START_SAFETY_ACTIVE");
  } else if (cmd == "stop") {
    autoEnabled = false;
    if (state == CUTTING) { motorStop(); state = IDLE; }
  } else if (cmd == "reset") {
    // Clear a fault only if the cause is gone
    if (state == FAULT && !estopPressed() && !coverOpen()) {
      fault = F_NONE;
      state = IDLE;
      autoEnabled = false;      // require explicit "start" again
    }
  } else if (cmd == "resetcount") {
    cutCount = 0;
  }
  publishStatus();
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) delay(200);
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  String id = "esp32-chopper-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(id.c_str())) {
    mqtt.subscribe(tCmd.c_str());
    publishStatus();
  }
}

// ---------------- Display ----------------
void updateDisplay() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("BETEL NUT CHOPPER");
  display.setCursor(0, 14);
  display.printf("State: %s", stateName());
  display.setCursor(0, 26);
  display.printf("Count: %lu", cutCount);
  display.setCursor(0, 38);
  display.printf("Current: %.2f A", currentA);
  display.setCursor(0, 50);
  if (state == FAULT) display.print(faultName());
  else display.print(WiFi.status() == WL_CONNECTED ? (mqtt.connected() ? "WiFi+MQTT OK" : "WiFi OK") : "Offline");
  display.display();
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IR, INPUT);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_COVER, INPUT_PULLUP);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);   // core 3.x API
  motorStop();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_CURRENT, ADC_11db);

  Wire.begin(21, 22);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Calibrate current-sensor zero with motor OFF
  delay(500);
  long sum = 0;
  for (int i = 0; i < 100; i++) { sum += analogReadMilliVolts(PIN_CURRENT); delay(2); }
  zeroOffsetV = (sum / 100.0) / 1000.0 * DIVIDER_FACTOR;

  tCmd    = String(TOPIC_BASE) + "/cmd";
  tStatus = String(TOPIC_BASE) + "/status";
  tAlert  = String(TOPIC_BASE) + "/alert";

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(onMessage);
  connectWiFi();

  // Safety check at boot
  if (estopPressed()) enterFault(F_ESTOP);
  else if (coverOpen()) enterFault(F_COVER);

  lastTick = millis();
}

// ---------------- Loop ----------------
void loop() {
  unsigned long now = millis();

  // --- network (non-blocking-ish) ---
  static unsigned long lastNet = 0;
  if (now - lastNet > 5000) {
    lastNet = now;
    connectWiFi();
    connectMQTT();
  }
  if (mqtt.connected()) mqtt.loop();

  // --- safety first, every loop ---
  if (state != FAULT) {
    if (estopPressed())      enterFault(F_ESTOP);
    else if (coverOpen())    enterFault(F_COVER);
  }

  // --- local reset: clear E-stop/cover fault once physically fixed ---
  if (state == FAULT && fault != F_OVERLOAD && !estopPressed() && !coverOpen()) {
    // Wait for explicit remote "reset" (or hold power-cycle) - stay in FAULT
  }

  // --- IR debounce (sensor LOW = nut present) ---
  bool irRaw = (digitalRead(PIN_IR) == LOW);
  if (irRaw != irLastRaw) { lastIrChange = now; irLastRaw = irRaw; }
  if (now - lastIrChange > DEBOUNCE_MS) irStable = irRaw;

  // --- current measurement ---
  if (state == CUTTING) {
    currentA = readCurrent();
    runtimeMs += (now - lastTick);
  } else {
    currentA = 0;
  }
  lastTick = now;

  // --- state machine ---
  switch (state) {
    case IDLE:
      if (autoEnabled && irStable) {
        motorRun();
        stateStart = now;
        overloadStart = 0;
        state = CUTTING;
      }
      break;

    case CUTTING:
      // Overload / jam detection
      if (currentA > OVERLOAD_AMPS) {
        if (overloadStart == 0) overloadStart = now;
        else if (now - overloadStart > OVERLOAD_TIME_MS) {
          enterFault(F_OVERLOAD);
          break;
        }
      } else {
        overloadStart = 0;
      }
      // Finished cut
      if (now - stateStart >= CUT_TIME_MS) {
        motorStop();
        cutCount++;
        stateStart = now;
        state = COOLDOWN;
        publishStatus();
      }
      break;

    case COOLDOWN:
      if (now - stateStart >= COOLDOWN_MS) state = IDLE;
      break;

    case FAULT:
      motorStop();
      break;
  }

  // --- status LED: solid = ready, blink = fault ---
  if (state == FAULT) digitalWrite(PIN_LED, (now / 250) % 2);
  else digitalWrite(PIN_LED, autoEnabled ? HIGH : LOW);

  // --- periodic publish and display ---
  if (now - lastPublish > 5000) { lastPublish = now; publishStatus(); }
  if (now - lastDisplay > 300)  { lastDisplay = now; updateDisplay(); }
}
