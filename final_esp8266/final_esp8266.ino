#define BLYNK_TEMPLATE_ID "TMPL6B7oqmQY5"
#define BLYNK_TEMPLATE_NAME "Smart home"
#define BLYNK_AUTH_TOKEN "4eG6UHtMG4p8PSwzH8TOfosFMwkNZsvW" // <- Thay bằng token của bạn

// Thư viện ---------------------------------------------------------------
#include <Wire.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>
#include <PCF8575.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <EEPROM.h>

// Thông tin kết nối -----------------------------------------------------
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Home";           // <- SSID WiFi của bạn
char pass[] = "123456789";     // <- Mật khẩu WiFi

// Chân phần cứng --------------------------------------------------------
#define DHTPIN D5
#define DHTTYPE DHT22
#define SERVO_PIN D4
#define MQ2_PIN A0
#define RELAY_MASTER_GPIO D6

// Khởi tạo đối tượng cho cảm biến / module
DHT dht(DHTPIN, DHTTYPE);
Servo myServo;
PCF8575 pcf(0x20);                 // I2C address của PCF8575
LiquidCrystal_I2C lcd(0x27, 16, 2);
BlynkTimer timer;                  // timer không đồng bộ

// Virtual pins dành cho app Blynk --------------------------------------
#define VP_STATUS_SUMMARY V10
#define VP_TEMP V3
#define VP_HUM  V4
#define VP_RELAY1 V1
#define VP_RELAY2 V2
#define VP_AUTO   V5
#define VP_FIRE   V6
#define VP_THEFT  V7
#define VP_RESET  V8
#define VP_TEMP_THR V9 // slider để đặt ngưỡng nhiệt

// Prototype (khai báo hàm) ---------------------------------------------
void updateSensors();
bool checkFireAlert();
bool checkTheftAlert();
void updateBuzzer(bool alertActive);
void updateLCD();
void handleButtons();
void handleKeypad();
String getStatusText();
void pushStatusToBlynk_ifChanged();
void saveSettingsToEEPROM();
void loadSettingsFromEEPROM();
void writeImmediateEvent(const String &msg); // wrapper backward-compatible

// PCF8575 mapping (P0..P15) ---------------------------------------------
#define RELAY1_PIN 4
#define RELAY2_PIN 5
#define BUTTON1_PIN 6
#define BUTTON2_PIN 7
#define FLAME_GPIO 16   // chân ESP (digitalRead)
#define PIR_PIN 9
#define BUZZER_PIN 10
#define LDR_PIN 11
#define MODE_AUTO_BUTTON_PIN 12
#define MODE_ALERT_BUTTON_PIN 13
#define MODE_THEFT_BUTTON_PIN 14
#define MASTER_BUTTON_PIN 15
#define KEY1_PIN 0
#define KEY2_PIN 1
#define KEY3_PIN 2
#define KEY4_PIN 3

// Trạng thái runtime ----------------------------------------------------
bool relay1State = false;
bool relay2State = false;
bool autoMode = false;
bool fireGasAlertEnabled = false;
bool antiTheftEnabled = false;
bool systemEnabled = true;     // bật/tắt toàn bộ hệ thống
bool lastFireAlert = false;    // trạng thái trước đó để phát hiện rising/falling
bool lastTheftAlert = false;
float tempThreshold = 30.0;    // ngưỡng nhiệt độ (độ C)

// Thông báo/ sự kiện ----------------------------------------------------
String lastEventMessage = ""; // mô tả sự kiện mới nhất (transient)
String lastPasswordMsg = "";  // hiển thị trạng thái mật khẩu
String lastOwnerCheck = "none"; // "owner"/"intruder"/"none"

// Debounce cho các nút --------------------------------------------------
unsigned long lastButtonRelay1 = 0;
unsigned long lastButtonRelay2 = 0;
unsigned long lastButtonAuto = 0;
unsigned long lastButtonAlert = 0;
unsigned long lastButtonTheft = 0;
unsigned long lastButtonMaster = 0;
const unsigned long debounceDelay = 250;

// Buzzer (nháy) ---------------------------------------------------------
unsigned long buzzerPreviousMillis = 0;
const unsigned long buzzerInterval = 500; // tần số chớp cho còi
bool buzzerState = false;

// LCD (ghi chênh lệch để tránh in lại cùng dòng) ----------------------
char lastLine1[20] = "";
char lastLine2[20] = "";

// PIR cooldown để tránh gửi thông báo liên tục khi chuyển động liên tục ----
unsigned long lastPirTrigger = 0;
const unsigned long pirCooldown = 5000; // 5s

// Notification rate-limit mặc định cho từng loại (ms)
unsigned long lastNotifyMillis = 0;
const unsigned long notifyCooldown = 60000; // 60s giữa 2 notify giống nhau
String lastNotifyMsg = "";

// Server PC (dùng cho check owner/intruder) -----------------------------
const char* pcServer = "http://10.87.168.149:5000/analyze";

// MQ2 baseline ----------------------------------------------------------
int mq2Baseline = 0;

// EEPROM
#define EEPROM_SIZE 32
#define ADDR_TEMP_THR 0

// Giảm lưu lượng mạng (chỉ gửi status khi khác trước) ------------------
String lastStatus = "";

// ----------------------------------------------------------------------
// Helpers kết nối
// ----------------------------------------------------------------------
void connectWiFi() {
  // Nếu chưa kết nối WiFi -> khởi kết nối và chờ tối đa 5s
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
      delay(100);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) Serial.println(F("\nWiFi reconnected"));
  }
}
void connectBlynk() {
  // Kết nối tới Blynk nếu chưa
  if (!Blynk.connected()) Blynk.connect(2000);
}
void checkConnection() { connectWiFi(); connectBlynk(); }
bool isWiFiConnected() { return WiFi.status() == WL_CONNECTED; }

// ----------------------------------------------------------------------
// EEPROM helpers - lưu/ đọc ngưỡng nhiệt
// ----------------------------------------------------------------------
void saveSettingsToEEPROM() {
  int val = int(tempThreshold * 10.0f);
  uint8_t high = (val >> 8) & 0xFF;
  uint8_t low  = val & 0xFF;
  EEPROM.write(ADDR_TEMP_THR, high);
  EEPROM.write(ADDR_TEMP_THR + 1, low);
  EEPROM.commit();
  Serial.print("Saved tempThreshold: "); Serial.println(tempThreshold);
}

void loadSettingsFromEEPROM() {
  uint8_t high = EEPROM.read(ADDR_TEMP_THR);
  uint8_t low  = EEPROM.read(ADDR_TEMP_THR + 1);
  int val = (high << 8) | low;
  if (val == 0x0000 || val == 0xFFFF) {
    // EEPROM chưa được ghi -> dùng mặc định
    Serial.println("EEPROM tempThreshold not set -> using default");
  } else {
    tempThreshold = float(val) / 10.0f;
    Serial.print("Loaded tempThreshold: "); Serial.println(tempThreshold);
  }
}

// ----------------------------------------------------------------------
// Xây dựng chuỗi trạng thái hiển thị (status summary)
// Tùy biến để gửi lên Blynk widget; bao gồm nhiều thông tin để debug
// ----------------------------------------------------------------------
String getStatusText() {
  String s = "";

  // Mode / flags
  s += String(systemEnabled ? "SYSTEM: ON" : "SYSTEM: OFF");
  s += " | " + String(autoMode ? "MODE: AUTO" : "MODE: MANUAL");
  s += " | " + String(fireGasAlertEnabled ? "FIRE ALERT: ENABLED" : "FIRE ALERT: OFF");
  s += " | " + String(antiTheftEnabled ? "THEFT: ENABLED" : "THEFT: OFF");

  // Nhiệt / độ ẩm
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  s += " | T:";
  if (!isnan(t)) s += String(t,1) + "C"; else s += "N/A";
  s += " H:";
  if (!isnan(h)) s += String(h,0) + "%"; else s += "N/A";
  s += " Th:" + String(tempThreshold,1) + "C";

  // Relay states
  s += " | Relay1:"; s += relay1State ? "ON" : "OFF";
  s += " Relay2:"; s += relay2State ? "ON" : "OFF";

  // Gas / flame
  int mq = analogRead(MQ2_PIN);
  int flameRaw = digitalRead(FLAME_GPIO);
  bool flameDetected = (flameRaw == 0);
  bool gasAlert = (mq > mq2Baseline + 150);
  if (gasAlert && flameDetected) {
    s += " | ALERT: FLAME+GAS";
  } else if (flameDetected) {
    s += " | ALERT: FLAME";
  } else if (gasAlert) {
    s += " | ALERT: GAS";
  } else {
    s += " | AREA: SAFE";
  }
  s += " | GASval:"; s += mq;
  s += " Gth:"; s += (mq2Baseline + 150);

  // PIR / door / buzzer
  bool pir = (pcf.readButton(PIR_PIN) == 1);
  s += " | PIR:"; s += pir ? "MOTION" : "idle";
  s += " | Door:"; s += (myServo.read() > 10) ? "OPEN" : "CLOSED";
  s += " | Buzz:"; s += buzzerState ? "ON" : "OFF";

  // Owner / password / last event
  s += " | LastAccess:";
  if (lastPasswordMsg.length()) {
    s += lastPasswordMsg; // hiển thị dạng "Mật khẩu đúng" hoặc "Mật khẩu sai"
  } else if (lastOwnerCheck == "owner") {
    s += "OwnerVerified";
  } else if (lastOwnerCheck == "intruder") {
    s += "Intruder";
  } else {
    s += "None";
  }

  // RSSI / uptime
  s += " | RSSI:"; s += String(WiFi.RSSI());
  long sec = millis() / 1000;
  s += " | Up:"; s += String(sec/3600) + "h" + String((sec%3600)/60) + "m";

  return s;
}

// ----------------------------------------------------------------------
// Gửi trạng thái lên Blynk khi có thay đổi (giảm lưu lượng mạng)
// ----------------------------------------------------------------------
void pushStatusToBlynk_ifChanged() {
  String s = getStatusText();
  if (s != lastStatus) {
    lastStatus = s;
    if (isWiFiConnected() && Blynk.connected()) {
      Blynk.virtualWrite(VP_STATUS_SUMMARY, s);
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if (!isnan(t)) Blynk.virtualWrite(VP_TEMP, t);
      if (!isnan(h)) Blynk.virtualWrite(VP_HUM, h);
    }
  }
}

// ----------------------------------------------------------------------
// Event logging: tách event theo loại (fire/theft/password/server/generic)
// - Sử dụng Blynk.logEvent(eventId, msg) để tương thích với nhiều phiên bản
// - Mỗi loại có rate-limit riêng để tránh spam
// ----------------------------------------------------------------------
#define EVT_FIRE 0
#define EVT_THEFT 1
#define EVT_PASSWORD 2
#define EVT_SERVER 3
#define EVT_GENERIC 4

unsigned long lastNotifyMillisArr[5] = {0,0,0,0,0};
String lastNotifyMsgArr[5] = {"","","","",""};

const char* eventIdMap[5] = { "alert_fire", "alert_theft", "alert_password", "alert_server", "alert" };

// Hàm trung tâm ghi event theo type
void writeEventType(int evtType, const String &msg) {
  if (evtType < 0 || evtType > 4) evtType = EVT_GENERIC;
  lastEventMessage = msg;
  if (isWiFiConnected() && Blynk.connected()) {
    // Cập nhật summary nhanh lên app
    Blynk.virtualWrite(VP_STATUS_SUMMARY, msg);
    // Ghi event lên server Blynk (Eventor) với rate-limit per-type
    if (msg != lastNotifyMsgArr[evtType] || millis() - lastNotifyMillisArr[evtType] > notifyCooldown) {
      Blynk.logEvent(eventIdMap[evtType], msg);
      lastNotifyMsgArr[evtType] = msg;
      lastNotifyMillisArr[evtType] = millis();
    }
  } else {
    Serial.println("Offline - event: " + msg);
  }
}

// Wrapper tương thích cũ
void writeImmediateEvent(const String &msg) {
  writeEventType(EVT_GENERIC, msg);
}

// ----------------------------------------------------------------------
// KEYPAD xử lý mật khẩu
// - Khi nhập đủ ký tự, so sánh và mở cửa/khóa còi
// - Ghi event loại EVT_PASSWORD
// ----------------------------------------------------------------------
String inputPassword = "";
String correctPassword = "1234";
void handleKeypad() {
  static unsigned long lastKeyMillis = 0;
  const unsigned long keyDebounce = 150;
  unsigned long now = millis();
  if (now - lastKeyMillis < keyDebounce) return;

  bool pressed = false;
  if (pcf.readButton(KEY1_PIN) == 0) { inputPassword += "1"; pressed = true; }
  else if (pcf.readButton(KEY2_PIN) == 0) { inputPassword += "2"; pressed = true; }
  else if (pcf.readButton(KEY3_PIN) == 0) { inputPassword += "3"; pressed = true; }
  else if (pcf.readButton(KEY4_PIN) == 0) { inputPassword += "4"; pressed = true; }

  if (pressed) {
    lastKeyMillis = now;
    if (inputPassword.length() == correctPassword.length()) {
      if (inputPassword == correctPassword) {
        myServo.write(120);
        pcf.write(BUZZER_PIN, 0);
        lastPasswordMsg = "Mật khẩu đúng"; // show human friendly
        lastOwnerCheck = "owner";
        writeEventType(EVT_PASSWORD, "✅ Mật khẩu đúng - mở cửa");
      } else {
        pcf.write(BUZZER_PIN, 1);
        timer.setTimeout(600, [](){ pcf.write(BUZZER_PIN, 0); });
        lastPasswordMsg = "Mật khẩu sai";
        writeEventType(EVT_PASSWORD, "🚨 Sai mật khẩu");
      }
      inputPassword = "";
      pushStatusToBlynk_ifChanged();
      // Xóa thông báo mật khẩu sau 8s
      timer.setTimeout(8000L, [](){ lastPasswordMsg = ""; });
    }
  }
}

// ----------------------------------------------------------------------
// Xử lý nút bấm trên PCF8575
// ----------------------------------------------------------------------
void handleButtons() {
  unsigned long now = millis();

  if (pcf.readButton(BUTTON1_PIN) == 0 && now - lastButtonRelay1 > debounceDelay) {
    lastButtonRelay1 = now;
    relay1State = !relay1State;
    if (systemEnabled) pcf.write(RELAY1_PIN, relay1State ? 0 : 1);
    if (isWiFiConnected() && Blynk.connected()) Blynk.virtualWrite(VP_RELAY1, relay1State);
    pushStatusToBlynk_ifChanged();
  }

  if (pcf.readButton(BUTTON2_PIN) == 0 && !autoMode && now - lastButtonRelay2 > debounceDelay) {
    lastButtonRelay2 = now;
    relay2State = !relay2State;
    if (systemEnabled) pcf.write(RELAY2_PIN, relay2State ? 0 : 1);
    if (isWiFiConnected() && Blynk.connected()) Blynk.virtualWrite(VP_RELAY2, relay2State);
    pushStatusToBlynk_ifChanged();
  }

  if (pcf.readButton(MODE_AUTO_BUTTON_PIN) == 0 && now - lastButtonAuto > debounceDelay) {
    lastButtonAuto = now;
    autoMode = !autoMode;
    if (isWiFiConnected() && Blynk.connected()) Blynk.virtualWrite(VP_AUTO, autoMode);
    pushStatusToBlynk_ifChanged();
  }

  if (pcf.readButton(MODE_ALERT_BUTTON_PIN) == 0 && now - lastButtonAlert > debounceDelay) {
    lastButtonAlert = now;
    fireGasAlertEnabled = !fireGasAlertEnabled;
    lastFireAlert = false;
    myServo.write(0);
    if (isWiFiConnected() && Blynk.connected()) Blynk.virtualWrite(VP_FIRE, fireGasAlertEnabled);
    pushStatusToBlynk_ifChanged();
  }

  if (pcf.readButton(MODE_THEFT_BUTTON_PIN) == 0 && now - lastButtonTheft > debounceDelay) {
    lastButtonTheft = now;
    antiTheftEnabled = !antiTheftEnabled;
    if (isWiFiConnected() && Blynk.connected()) Blynk.virtualWrite(VP_THEFT, antiTheftEnabled);
    pushStatusToBlynk_ifChanged();
  }

  if (pcf.readButton(MASTER_BUTTON_PIN) == 0 && now - lastButtonMaster > debounceDelay) {
    lastButtonMaster = now;
    systemEnabled = !systemEnabled;
    digitalWrite(RELAY_MASTER_GPIO, systemEnabled ? LOW : HIGH);
    if (isWiFiConnected() && Blynk.connected()) Blynk.virtualWrite(VP_RESET, systemEnabled);
    pushStatusToBlynk_ifChanged();
  }
}

// ----------------------------------------------------------------------
// Cập nhật cảm biến (được gọi định kỳ)
// - Tự động điều khiển quạt/đèn khi autoMode
// ----------------------------------------------------------------------
void updateSensors() {
  if (!systemEnabled) return;

  float temp = dht.readTemperature();
  float humi = dht.readHumidity();

  if (!isnan(temp) && !isnan(humi)) {
    // chỉ số temp/humi được gửi qua pushStatusToBlynk_ifChanged
  }

  if (autoMode) {
    bool fanOn = (!isnan(temp) && temp >= tempThreshold);
    pcf.write(RELAY1_PIN, fanOn ? 0 : 1);
    relay1State = fanOn; // đồng bộ trạng thái

    bool isDark = (pcf.readButton(LDR_PIN) == 1);
    pcf.write(RELAY2_PIN, isDark ? 0 : 1);
    relay2State = isDark;
  }

  pushStatusToBlynk_ifChanged();
}

// ----------------------------------------------------------------------
// Kiểm tra cảnh báo lửa/khí (fire/gas)
// - Khi phát hiện: mở cửa, bật còi, log event kiểu EVT_FIRE
// - Khi hết cảnh báo: tắt còi, đóng cửa, log event
// ----------------------------------------------------------------------
bool checkFireAlert() {
  if (!systemEnabled || !fireGasAlertEnabled) return false;

  int mq2Value = analogRead(MQ2_PIN);
  int flameRaw = digitalRead(FLAME_GPIO);
  bool flameDetected = (flameRaw == 0);
  bool gasAlert = (mq2Value > mq2Baseline + 150);
  bool alert = flameDetected || gasAlert;

  if (alert && !lastFireAlert) {
    // rising edge: mới phát hiện cảnh báo
    lastFireAlert = true;
    myServo.write(120); // mở cửa
    pcf.write(BUZZER_PIN, 1);
    writeEventType(EVT_FIRE, "🔥 Phát hiện GAS/LỬA! Mở cửa khẩn");
    pushStatusToBlynk_ifChanged();
  } else if (!alert && lastFireAlert) {
    // falling edge: cảnh báo đã hết
    lastFireAlert = false;
    myServo.write(0);
    pcf.write(BUZZER_PIN, 0);
    writeEventType(EVT_FIRE, "✅ Không còn GAS/LỬA");
    pushStatusToBlynk_ifChanged();
  }
  return alert;
}

// ----------------------------------------------------------------------
// Kiểm tra cảnh báo trộm (PIR) kết hợp server để phân biệt chủ nhà/khách
// - Nếu có chuyển động, gọi server PC -> trả "owner" hoặc khác
// - Ghi event EVT_THEFT với kết quả
// ----------------------------------------------------------------------
bool checkTheftAlert() {
  if (!systemEnabled || !antiTheftEnabled) return false;

  bool motion = (pcf.readButton(PIR_PIN) == 1);
  if (motion && millis() - lastPirTrigger > pirCooldown) {
    lastPirTrigger = millis();
    if (isWiFiConnected()) {
      // Gọi server phân loại
      WiFiClient client;
      HTTPClient http;
      if (http.begin(client, pcServer)) {
        int httpCode = http.GET();
        if (httpCode == 200) {
          String payload = http.getString();
          payload.trim();
          Serial.print("PC result: "); Serial.println(payload);
          if (payload == "owner") {
            lastOwnerCheck = "owner";
            myServo.write(120);
            pcf.write(BUZZER_PIN, 0);
            writeEventType(EVT_THEFT, "✅ Chủ nhà - mở cửa");
          } else {
            lastOwnerCheck = "intruder";
            pcf.write(BUZZER_PIN, 1);
            writeEventType(EVT_THEFT, "🚨 Người lạ - báo động");
          }
        } else {
          pcf.write(BUZZER_PIN, 1);
          writeEventType(EVT_SERVER, "⚠️ Kiểm tra server thất bại");
        }
        http.end();
      } else {
        pcf.write(BUZZER_PIN, 1);
        writeEventType(EVT_SERVER, "⚠️ Không thể kết nối HTTP tới server");
      }
      pushStatusToBlynk_ifChanged();
    } else {
      // offline -> chỉ bật còi và log offline
      pcf.write(BUZZER_PIN, 1);
      writeEventType(EVT_THEFT, "🚨 Phát hiện chuyển động (offline) - bật còi");
      pushStatusToBlynk_ifChanged();
    }
    return true;
  }

  // Clear state nếu không còn motion trong thời gian cooldown
  if (!motion && lastOwnerCheck != "none" && millis() - lastPirTrigger > pirCooldown) {
    lastOwnerCheck = "none";
    pushStatusToBlynk_ifChanged();
  }

  return false;
}

// ----------------------------------------------------------------------
// Điều khiển còi (buzzer) theo trạng thái cảnh báo
// ----------------------------------------------------------------------
void updateBuzzer(bool alertActive) {
  if (!systemEnabled) {
    pcf.write(BUZZER_PIN, 0);
    buzzerState = false;
    return;
  }

  unsigned long now = millis();
  if (alertActive) {
    if (now - buzzerPreviousMillis >= buzzerInterval) {
      buzzerPreviousMillis = now;
      buzzerState = !buzzerState;
      pcf.write(BUZZER_PIN, buzzerState ? 1 : 0);
    }
  } else {
    pcf.write(BUZZER_PIN, 0);
    buzzerState = false;
  }
}

// ----------------------------------------------------------------------
// Cập nhật LCD (ghi chữ chỉ khi khác) để giảm flicker
// ----------------------------------------------------------------------
void updateLCD() {
  char line1[20], line2[20];
  sprintf(line1, "Time %02d:%02d", hour(), minute());
  sprintf(line2, "%s %c %s %s",
          systemEnabled ? "ON" : "OFF",
          fireGasAlertEnabled ? 'A' : '-',
          autoMode ? "Auto" : "Man",
          antiTheftEnabled ? "Safe" : "Open");
  if (strcmp(line1, lastLine1) != 0) {
    lcd.setCursor(0,0); lcd.print("                ");
    lcd.setCursor(0,0); lcd.print(line1);
    strcpy(lastLine1, line1);
  }
  if (strcmp(line2, lastLine2) != 0) {
    lcd.setCursor(0,1); lcd.print("                ");
    lcd.setCursor(0,1); lcd.print(line2);
    strcpy(lastLine2, line2);
  }
}

// ----------------------------------------------------------------------
// setup() - khởi tạo phần cứng và trạng thái ban đầu
// ----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  loadSettingsFromEEPROM();

  // Kết nối WiFi (blocking ở đây) - có thể đổi sang non-blocking nếu muốn
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println(F("\nWiFi connected"));

  // Cấu hình Blynk
  Blynk.config(auth);
  Blynk.connect();

  // Khởi tạo cảm biến / module
  dht.begin();
  myServo.attach(SERVO_PIN);
  myServo.write(0);
  pcf.begin();
  pcf.write(RELAY1_PIN, 1); // relay off (active low trên PCF8575 mapping hiện tại)
  pcf.write(RELAY2_PIN, 1);
  pcf.write(BUZZER_PIN, 0);
  pcf.write(PIR_PIN, 1);
  pcf.write(LDR_PIN, 1);

  pinMode(RELAY_MASTER_GPIO, OUTPUT);
  digitalWrite(RELAY_MASTER_GPIO, HIGH); // start = off
  pinMode(FLAME_GPIO, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print(F("Smart Home Ready"));

  // Đồng bộ trạng thái ban đầu lên app (chỉ control pins)
  if (isWiFiConnected() && Blynk.connected()) {
    Blynk.virtualWrite(VP_AUTO, autoMode);
    Blynk.virtualWrite(VP_FIRE, fireGasAlertEnabled);
    Blynk.virtualWrite(VP_THEFT, antiTheftEnabled);
    Blynk.virtualWrite(VP_RESET, systemEnabled);
    Blynk.virtualWrite(VP_TEMP_THR, tempThreshold);
  }

  // Tính baseline cho MQ2 (trung bình 50 mẫu)
  long sum = 0;
  for (int i=0;i<50;i++){ sum += analogRead(MQ2_PIN); delay(20); }
  mq2Baseline = sum/50;
  Serial.print("MQ2 baseline: "); Serial.println(mq2Baseline);

  // Timer định kỳ
  timer.setInterval(10000L, checkConnection); // kiểm tra kết nối
  timer.setInterval(2000L, updateSensors);    // đọc cảm biến chính
  timer.setInterval(500L, [](){               // check alert + cập nhật buzzer + lcd
    bool f = checkFireAlert();
    bool t = checkTheftAlert();
    updateBuzzer(f || t);
    updateLCD();
  });

  pushStatusToBlynk_ifChanged();
}

// ----------------------------------------------------------------------
// loop() - gọi Blynk.run và các routine
// ----------------------------------------------------------------------
void loop() {
  if (Blynk.connected()) Blynk.run();
  timer.run();
  handleButtons();
  handleKeypad();
}

// ----------------------------------------------------------------------
// Blynk callbacks: nhận lệnh từ app (remote control)
// ----------------------------------------------------------------------
BLYNK_WRITE(VP_RELAY1) {
  relay1State = param.asInt();
  if (systemEnabled) pcf.write(RELAY1_PIN, relay1State ? 0 : 1);
  pushStatusToBlynk_ifChanged();
}
BLYNK_WRITE(VP_RELAY2) {
  relay2State = param.asInt();
  if (systemEnabled && !autoMode) pcf.write(RELAY2_PIN, relay2State ? 0 : 1);
  pushStatusToBlynk_ifChanged();
}
BLYNK_WRITE(VP_AUTO) {
  autoMode = param.asInt();
  Serial.println(autoMode ? "Auto ON" : "Auto OFF");
  pushStatusToBlynk_ifChanged();
}
BLYNK_WRITE(VP_FIRE) {
  fireGasAlertEnabled = param.asInt();
  lastFireAlert = false;
  myServo.write(0);
  Serial.println(fireGasAlertEnabled ? "Fire alert enabled" : "Fire alert disabled");
  pushStatusToBlynk_ifChanged();
}
BLYNK_WRITE(VP_THEFT) {
  antiTheftEnabled = param.asInt();
  Serial.println(antiTheftEnabled ? "Theft enabled" : "Theft disabled");
  pushStatusToBlynk_ifChanged();
}
BLYNK_WRITE(VP_RESET) {
  systemEnabled = param.asInt();
  digitalWrite(RELAY_MASTER_GPIO, systemEnabled ? LOW : HIGH);
  Serial.println(systemEnabled ? "System ON" : "System OFF");
  pushStatusToBlynk_ifChanged();
}
BLYNK_WRITE(VP_TEMP_THR) {
  float newThr = param.asFloat();
  if (newThr < -10) newThr = -10;
  if (newThr > 80) newThr = 80;
  tempThreshold = newThr;
  Serial.print("Temp threshold set: "); Serial.println(tempThreshold);
  saveSettingsToEEPROM();
  pushStatusToBlynk_ifChanged();
}
