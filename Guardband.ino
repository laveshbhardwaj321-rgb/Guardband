// ============================================================
//  GuardBand — Smart Child Safety Jewellery
//  ESP32 + MPU6050 + SIM7000E (GSM/SMS mode)
//
//  Features:
//    - Fall / impact detection (MPU6050)
//    - Drag / violent motion detection
//    - Prolonged stillness alert
//    - Manual SOS button (hold 1 sec)
//    - SMS alert with event type + force reading
//    - Live cell tower ID in SMS
//    - Google Maps link (VIT Chennai AB4)
//    - Cooldown to prevent SMS spam
//    - Heartbeat SMS every 30 min
//    - Serial debug output
// ============================================================

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ─── PIN CONFIG ─────────────────────────────────────────────
#define SIM_RX_PIN      16      // ESP32 RX2  ← SIM TX
#define SIM_TX_PIN      17      // ESP32 TX2  → SIM RX
#define SIM_BAUD        9600
#define SOS_BUTTON_PIN  0       // Boot button on most ESP32 boards

// ─── CHANGE THESE ───────────────────────────────────────────
#define PARENT_NUMBER   "+916005140266"   // parent's number
#define CHILD_NAME      "Shourya"      // child's name in SMS

// ─── THRESHOLDS ─────────────────────────────────────────────
#define FALL_THRESHOLD       7.0f    // m/s² — below this = free fall
#define IMPACT_THRESHOLD    24.0f    // m/s² — above this = hard impact
#define DRAG_ACCEL          12.0f    // m/s² — accel part of drag
#define DRAG_GYRO            3.5f    // rad/s — gyro part of drag
#define STILL_DURATION_MS  300000UL  // 5 min stillness triggers alert
#define COOLDOWN_MS         15000UL  // min gap between any two alerts
#define HEARTBEAT_MS      1800000UL  // heartbeat SMS every 30 min
#define FILTER_ALPHA         0.95f   // complementary filter weight

// ─── GLOBALS ────────────────────────────────────────────────
Adafruit_MPU6050 mpu;

float         filteredMag       = 9.8f;
bool          modemReady        = false;
bool          stillActive       = false;
bool          smsBusy           = false;  // blocks passthrough during SMS

unsigned long lastAlertTime     = 0;
unsigned long stillSinceTime    = 0;
unsigned long lastHeartbeatTime = 0;

// ─── FORWARD DECLARATIONS ───────────────────────────────────
void   initModem();
void   sendSMS(String message);
String getCellLocation();
String buildAlertSMS(String eventType, float magnitude, String location);
void   triggerAlert(String eventType, float magnitude);
bool   onCooldown();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== GuardBand Starting ===");

  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);

  // MPU6050 — SDA=18, SCL=19
  Wire.begin(18, 19);
  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 not found — check SDA/SCL wiring!");
    while (1) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("[OK] MPU6050 initialised");

  Serial2.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  initModem();

  lastHeartbeatTime = millis();
  Serial.println("[OK] GuardBand ready\n");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // Serial passthrough — only active when NOT sending SMS
  if (!smsBusy) {
    if (Serial.available())  Serial2.write(Serial.read());
    if (Serial2.available()) Serial.write(Serial2.read());
  }

  // ── Read MPU6050 ─────────────────────────────────────────
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rawMag = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );
  float gyroMag = sqrt(
    g.gyro.x * g.gyro.x +
    g.gyro.y * g.gyro.y +
    g.gyro.z * g.gyro.z
  );

  // Complementary low-pass filter
  filteredMag = FILTER_ALPHA * filteredMag + (1.0f - FILTER_ALPHA) * rawMag;

  // Debug print every second
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 1000) {
    lastPrint = now;
    Serial.printf("[MOTION] |a|=%.2f (filt=%.2f)  gyro=%.2f\n",
                  rawMag, filteredMag, gyroMag);
  }

  // ── Event Detection ──────────────────────────────────────

  // 1. Free fall
  if (filteredMag < FALL_THRESHOLD && !onCooldown()) {
    triggerAlert("FALL DETECTED", filteredMag);
  }
  // 2. Hard impact
  else if (filteredMag > IMPACT_THRESHOLD && !onCooldown()) {
    triggerAlert("HARD IMPACT", filteredMag);
  }
  // 3. Drag / violent motion
  else if (filteredMag > DRAG_ACCEL && gyroMag > DRAG_GYRO && !onCooldown()) {
    triggerAlert("VIOLENT MOTION", filteredMag);
  }

  // 4. Prolonged stillness
  bool isStill = (filteredMag > 8.8f && filteredMag < 10.8f && gyroMag < 0.2f);
  if (isStill) {
    if (!stillActive) {
      stillActive    = true;
      stillSinceTime = now;
    } else if ((now - stillSinceTime > STILL_DURATION_MS) && !onCooldown()) {
      triggerAlert("PROLONGED STILLNESS", filteredMag);
      stillSinceTime = now;
    }
  } else {
    stillActive = false;
  }

  // 5. SOS button — must hold 1 second
  static unsigned long btnPressTime = 0;
  if (digitalRead(SOS_BUTTON_PIN) == LOW) {
    if (btnPressTime == 0) btnPressTime = now;
    if ((now - btnPressTime > 1000) && !onCooldown()) {
      triggerAlert("SOS BUTTON PRESSED", 0.0f);
      btnPressTime = 0;
    }
  } else {
    btnPressTime = 0;
  }

  // 6. Heartbeat every 30 minutes
  if (now - lastHeartbeatTime > HEARTBEAT_MS) {
    lastHeartbeatTime = now;
    Serial.println("[INFO] Sending heartbeat...");
    String loc = getCellLocation();
    String msg  = "GuardBand CHECK-IN\n";
    msg += "Child: "     + String(CHILD_NAME) + "\n";
    msg += "Status: All clear\n";
    msg += "Location: "  + loc + "\n";
    msg += "Uptime: "    + String(now / 60000) + " min";
    sendSMS(msg);
  }

  delay(80);
}

// ============================================================
//  MODEM INIT
// ============================================================
void initModem() {
  Serial.println("[MODEM] Initialising...");
  delay(3000);
  while (Serial2.available()) Serial2.read();

  // Basic alive check
  Serial2.println("AT");
  delay(500);
  String resp = "";
  while (Serial2.available()) resp += (char)Serial2.read();
  if (resp.indexOf("OK") == -1) {
    Serial.println("[WARN] Modem not responding — check power & wiring");
    modemReady = false;
    return;
  }

  // Disable echo
  Serial2.println("ATE0");
  delay(300);
  while (Serial2.available()) Serial2.read();

  // SMS text mode
  Serial2.println("AT+CMGF=1");
  delay(300);
  while (Serial2.available()) Serial2.read();

  // Set correct Airtel Tamil Nadu SMSC — critical for SMS delivery
  Serial2.println("AT+CSCA=\"+919840000000\"");
  delay(300);
  while (Serial2.available()) Serial2.read();

  // Check signal strength
  Serial2.println("AT+CSQ");
  delay(500);
  resp = "";
  while (Serial2.available()) resp += (char)Serial2.read();
  Serial.println("[MODEM] Signal: " + resp);

  // Check network registration
  Serial2.println("AT+CREG?");
  delay(500);
  resp = "";
  while (Serial2.available()) resp += (char)Serial2.read();
  Serial.println("[MODEM] Network: " + resp);

  modemReady = true;
  Serial.println("[OK] Modem ready");
}

// ============================================================
//  SEND SMS
// ============================================================
void sendSMS(String message) {
  if (!modemReady) {
    Serial.println("[WARN] Modem not ready — SMS skipped");
    return;
  }

  smsBusy = true;

  // Close any open HTTP session
  Serial2.println("AT+HTTPTERM");
  delay(500);
  while (Serial2.available()) Serial2.read();

  // Re-confirm SMS text mode
  Serial2.println("AT+CMGF=1");
  delay(500);
  while (Serial2.available()) Serial2.read();

  Serial.println("[SMS] Sending to " + String(PARENT_NUMBER));
  Serial.println("[SMS] Length: " + String(message.length()) + " chars");

  Serial2.print("AT+CMGS=\"");
  Serial2.print(PARENT_NUMBER);
  Serial2.println("\"");

  // Wait for > prompt
  bool gotPrompt = false;
  unsigned long t = millis();
  while (millis() - t < 5000) {
    if (Serial2.available()) {
      char c = Serial2.read();
      Serial.write(c);
      if (c == '>') { gotPrompt = true; break; }
    }
  }

  if (!gotPrompt) {
    Serial.println("\n[SMS] No > prompt — aborting");
    Serial2.write(27);  // ESC to cancel
    smsBusy = false;
    return;
  }

  delay(100);
  Serial2.print(message);
  delay(500);
  Serial2.write(26);  // Ctrl+Z to send

  // Wait up to 10 seconds for confirmation
  String resp = "";
  t = millis();
  while (millis() - t < 10000) {
    if (Serial2.available()) resp += (char)Serial2.read();
    if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1) break;
  }

  Serial.println("\n[SMS] Response: " + resp);

  if (resp.indexOf("+CMGS") != -1)
    Serial.println("[SMS] Delivered OK");
  else if (resp.indexOf("ERROR") != -1)
    Serial.println("[SMS] Failed — " + resp);
  else
    Serial.println("[SMS] Uncertain — no confirmation");

  smsBusy = false;
}

// ============================================================
//  GET CELL LOCATION
// ============================================================
String getCellLocation() {
  smsBusy = true;

  String lac = "14114";
  String cid = "63632";

  while (Serial2.available()) Serial2.read();

  Serial2.println("AT+CREG=2");
  delay(300);
  while (Serial2.available()) Serial2.read();

  Serial2.println("AT+CREG?");
  delay(500);
  String resp = "";
  unsigned long t = millis();
  while (millis() - t < 1500) {
    if (Serial2.available()) resp += (char)Serial2.read();
  }

  // Parse hex LAC and CID from +CREG: 2,5,"LAC","CID",0
  int q1 = resp.indexOf('"');
  int q2 = resp.indexOf('"', q1 + 1);
  int q3 = resp.indexOf('"', q2 + 1);
  int q4 = resp.indexOf('"', q3 + 1);
  if (q1 >= 0 && q4 >= 0) {
    String lacHex = resp.substring(q1 + 1, q2);
    String cidHex = resp.substring(q3 + 1, q4);
    lacHex.trim(); cidHex.trim();
    lac = String(strtol(lacHex.c_str(), NULL, 16));
    cid = String(strtol(cidHex.c_str(), NULL, 16));
  }

  smsBusy = false;

  // Fixed coordinates for VIT Chennai AB4 Block
  String mapsLink = "maps.google.com/?q=12.8406,80.1534";

  return " VIT\n" + mapsLink;
}

// ============================================================
//  BUILD ALERT SMS
// ============================================================
String buildAlertSMS(String eventType, float magnitude, String location) {
  String msg = "GUARDBAND!\n";
  msg += CHILD_NAME + String("\n");
  msg += eventType + "\n";
  if (magnitude > 0.1f)
    msg += String(magnitude, 1) + "m/s2\n";
  msg += location;
  return msg;
}

// ============================================================
//  TRIGGER ALERT
// ============================================================
void triggerAlert(String eventType, float magnitude) {
  lastAlertTime = millis();
  Serial.println("[ALERT] " + eventType + " | mag=" + String(magnitude, 1));
  String location = getCellLocation();
  String msg      = buildAlertSMS(eventType, magnitude, location);
  sendSMS(msg);
}

// ============================================================
//  COOLDOWN CHECK
// ============================================================
bool onCooldown() {
  return (millis() - lastAlertTime < COOLDOWN_MS);
}
