#define BLYNK_TEMPLATE_ID "TMPL6B7oqmQY5"
#define BLYNK_TEMPLATE_NAME "Smart home"
#define BLYNK_AUTH_TOKEN "4eG6UHtMG4p8PSwzH8TOfosFMwkNZsvW"

#include <Wire.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>
#include <PCF8575.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Home";
char pass[] = "123456789";

#define DHTPIN D5
#define DHTTYPE DHT22
#define SERVO_PIN D4
#define MQ2_PIN A0
#define RELAY_MASTER_GPIO D6
#define BUZZER_PIN D7

DHT dht(DHTPIN, DHTTYPE);
Servo myServo;
PCF8575 pcf(0x20);
LiquidCrystal_I2C lcd(0x27, 16, 2);
BlynkTimer timer;
// Trạng thái
bool relay1State = false;
bool relay2State = false;
bool autoMode = false;
bool fireGasAlertEnabled = false;
bool antiTheftEnabled = false;
bool systemEnabled = true;
bool lastFireAlert = false;
bool lastTheftAlert = false;
float tempThreshold = 30.0;

// Mật khẩu keypad
String inputPassword = "";
String correctPassword = "1234";

// Debounce
unsigned long lastButtonRelay1 = 0;
unsigned long lastButtonRelay2 = 0;
unsigned long lastButtonAuto = 0;
unsigned long lastButtonAlert = 0;
unsigned long lastButtonTheft = 0;
unsigned long lastButtonMaster = 0;
unsigned long lastButtonClose = 0;
const unsigned long debounceDelay = 250;

// Buzzer
unsigned long buzzerPreviousMillis = 0;
const unsigned long buzzerInterval = 500;
bool buzzerState = false;

// LCD
char lastLine1[20] = "";
char lastLine2[20] = "";

// PIR cooldown
unsigned long lastPirTrigger = 0;
const unsigned long pirCooldown = 5000;

// Thời gian cửa mở sau khi nhận diện chủ nhà
const unsigned long OWNER_LOCKOUT_TIME = 20000L; 

// PC server
const char* pcServer = "http://10.87.168.149:5000/analyze";
#define RELAY1_PIN 4
#define RELAY2_PIN 5
#define BUTTON1_PIN 6
#define BUTTON2_PIN 7
#define CLOSE_DOOR_BUTTON_PIN 8 
#define FLAME_GPIO 16
#define PIR_PIN 9
#define LDR_PIN 11
#define MODE_AUTO_BUTTON_PIN 12
#define MODE_ALERT_BUTTON_PIN 13
#define MODE_THEFT_BUTTON_PIN 14
#define MASTER_BUTTON_PIN 15
#define KEY1_PIN 0
#define KEY2_PIN 1
#define KEY3_PIN 2
#define KEY4_PIN 3

int mq2Baseline = 0;
// ================== KẾT NỐI ==================
void connectWiFi() {
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
  if (!Blynk.connected()) Blynk.connect(2000); // timeout ngắn để tránh treo
}

void checkConnection() {
  connectWiFi();
  connectBlynk();
}

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);

  // Kết nối WiFi ban đầu (timeout 15s)
  WiFi.begin(ssid, pass);
  unsigned long startAttempt = millis();
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(300); 
    Serial.print("."); 
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\nWiFi connected"));
  } else {
    Serial.println(F("\nWiFi connection failed after 15s. Offline mode."));
  }

  // Chỉ config Blynk, không connect chặn
  Blynk.config(auth);

  dht.begin();
  myServo.attach(SERVO_PIN);
  myServo.write(0);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pcf.begin();
  pcf.write(RELAY1_PIN, 1);
  pcf.write(RELAY2_PIN, 1);
  pcf.write(PIR_PIN, 1);
  pcf.write(LDR_PIN, 1);

  pinMode(RELAY_MASTER_GPIO, OUTPUT);
  digitalWrite(RELAY_MASTER_GPIO, HIGH);
  pinMode(FLAME_GPIO, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Smart Home Ready"));

  // MQ2 baseline
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(MQ2_PIN);
    delay(50);
  }
  mq2Baseline = sum / 10;
  Serial.print("MQ2 baseline: "); Serial.println(mq2Baseline);

  // Timer định kỳ
  timer.setInterval(5000L, checkConnection);
  timer.setInterval(2000L, updateSensors);
  timer.setInterval(500L, []() {
    bool fireAlert = checkFireAlert();
    bool theftAlert = checkTheftAlert();
    updateBuzzer(fireAlert || theftAlert);
    updateLCD();
  });
}
// ================== BLYNK CONNECTED ==================
BLYNK_CONNECTED() {
  // Lấy lại trạng thái từ app xuống thiết bị
  Blynk.syncVirtual(V1, V2, V5, V6, V8, V13, V14, V15);

  // Đồng bộ trạng thái hiện tại từ thiết bị lên app
  Blynk.virtualWrite(V1, relay1State);
  Blynk.virtualWrite(V2, relay2State);
  Blynk.virtualWrite(V5, autoMode);
  Blynk.virtualWrite(V6, tempThreshold);
  Blynk.virtualWrite(V8, fireGasAlertEnabled);
  Blynk.virtualWrite(V13, antiTheftEnabled);
  Blynk.virtualWrite(V14, systemEnabled);

  // Khởi tạo thông báo
  Blynk.virtualWrite(V7, "✅ Hệ thống cảnh báo Khí/Lửa sẵn sàng.");
  Blynk.virtualWrite(V12, "✅ Hệ thống chống trộm sẵn sàng.");
}

// ================== HÀM ĐÓNG/MỞ CỬA ==================
void closeDoorNow() {
  myServo.write(0); // Đóng cửa
  if (Blynk.connected()) {
    Blynk.virtualWrite(V12, "🚪 Đã đóng cửa ngay.");
  }
}

void openDoorTemporarily() {
  myServo.write(120); // Mở cửa
  digitalWrite(BUZZER_PIN, LOW); // Tắt buzzer
  if (Blynk.connected()) {
    Blynk.virtualWrite(V12, "✅ Cửa mở thủ công (20s)");
  }

  // Đóng cửa sau 20 giây
  timer.setTimeout(20000L, []() {
    myServo.write(0);
    if (Blynk.connected()) {
      Blynk.virtualWrite(V12, "🚪 Cửa tự động đóng (20s)");
    }
  });
}
// ================== KEYPAD ==================
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
        digitalWrite(BUZZER_PIN, LOW);
        if (Blynk.connected()) Blynk.virtualWrite(V12, "✅ Mật khẩu đúng, mở cửa");
        timer.setTimeout(5000L, []() {
          myServo.write(0);
          if (Blynk.connected()) Blynk.virtualWrite(V12, "🚪 Cửa tự động đóng (Keypad)");
        });
      } else {
        digitalWrite(BUZZER_PIN, HIGH);
        timer.setTimeout(600, [](){ digitalWrite(BUZZER_PIN, LOW); });
        if (Blynk.connected()) Blynk.virtualWrite(V12, "🚨 Sai mật khẩu");
      }
      inputPassword = ""; // reset sau khi so sánh
    }
  }
}

// ================== NÚT BẤM ==================
void handleButtons() {
  unsigned long now = millis();

  // Relay 1
  if (pcf.readButton(BUTTON1_PIN) == 0 && now - lastButtonRelay1 > debounceDelay) {
    lastButtonRelay1 = now;
    relay1State = !relay1State;
    if (systemEnabled) pcf.write(RELAY1_PIN, relay1State ? 0 : 1);
    if (Blynk.connected()) Blynk.virtualWrite(V1, relay1State); 
  }

  // Relay 2
  if (pcf.readButton(BUTTON2_PIN) == 0 && !autoMode && now - lastButtonRelay2 > debounceDelay) {
    lastButtonRelay2 = now;
    relay2State = !relay2State;
    if (systemEnabled) pcf.write(RELAY2_PIN, relay2State ? 0 : 1);
    if (Blynk.connected()) Blynk.virtualWrite(V2, relay2State); 
  }

  // Auto Mode
  if (pcf.readButton(MODE_AUTO_BUTTON_PIN) == 0 && now - lastButtonAuto > debounceDelay) {
    lastButtonAuto = now;
    autoMode = !autoMode;
    if (Blynk.connected()) Blynk.virtualWrite(V5, autoMode); 
  }

  // Alert Mode
  if (pcf.readButton(MODE_ALERT_BUTTON_PIN) == 0 && now - lastButtonAlert > debounceDelay) {
    lastButtonAlert = now;
    fireGasAlertEnabled = !fireGasAlertEnabled;
    if (Blynk.connected()) Blynk.virtualWrite(V8, fireGasAlertEnabled); 
  }

  // Theft Mode
  if (pcf.readButton(MODE_THEFT_BUTTON_PIN) == 0 && now - lastButtonTheft > debounceDelay) {
    lastButtonTheft = now;
    antiTheftEnabled = !antiTheftEnabled;
    if (Blynk.connected()) Blynk.virtualWrite(V13, antiTheftEnabled); 
  }

  // Master Mode
  if (pcf.readButton(MASTER_BUTTON_PIN) == 0 && now - lastButtonMaster > debounceDelay) {
    lastButtonMaster = now;
    systemEnabled = !systemEnabled;
    digitalWrite(RELAY_MASTER_GPIO, systemEnabled ? LOW : HIGH);
    if (Blynk.connected()) Blynk.virtualWrite(V14, systemEnabled); 
  }
  
  // Nút mở/đóng cửa tạm thời
  if (pcf.readButton(CLOSE_DOOR_BUTTON_PIN) == 0 && now - lastButtonClose > debounceDelay) {
    lastButtonClose = now;
    if (myServo.read() == 0) {
      openDoorTemporarily();
    } else {
      closeDoorNow();
    }
  }
}
// ================== CẬP NHẬT CẢM BIẾN ==================
void updateSensors() {
  if (!systemEnabled) return;

  float temp = dht.readTemperature();
  float humi = dht.readHumidity();

  // Gửi nhiệt độ và độ ẩm (V3, V4)
  if (!isnan(temp) && !isnan(humi) && Blynk.connected()) { 
    Blynk.virtualWrite(V3, temp);
    Blynk.virtualWrite(V4, humi);
  }

  if (autoMode) {
    // Điều khiển Quạt (Relay1)
    relay1State = temp >= tempThreshold;
    pcf.write(RELAY1_PIN, relay1State ? 0 : 1);
    if (Blynk.connected()) Blynk.virtualWrite(V1, relay1State); 

    // Điều khiển Đèn (Relay2)
    relay2State = (pcf.readButton(LDR_PIN) == 1);
    pcf.write(RELAY2_PIN, relay2State ? 0 : 1);
    if (Blynk.connected()) Blynk.virtualWrite(V2, relay2State); 
  }
}

// ================== HÀM BÁO CHÁY ==================
bool checkFireAlert() {
  if (!systemEnabled || !fireGasAlertEnabled) return false;

  int mq2Value = analogRead(MQ2_PIN);
  int flameRaw = digitalRead(FLAME_GPIO);

  bool flameDetected = (flameRaw == 0); 
  bool alert = (mq2Value > mq2Baseline + 150 || flameDetected);

  if (alert && !lastFireAlert) {
    if (Blynk.connected()) { 
      Blynk.virtualWrite(V7, "🔥 Phát hiện khí hoặc lửa!");
      Blynk.logEvent("fire_alert", "🔥 Phát hiện khí hoặc lửa!");
    }
    myServo.write(120);   // mở cửa khẩn cấp
    lastFireAlert = true;
  } else if (!alert && lastFireAlert) {
    if (Blynk.connected()) { 
      Blynk.virtualWrite(V7, "✅ Không còn khí hoặc lửa.");
      Blynk.logEvent("fire_safe", "✅ Không còn khí hoặc lửa.");
    }
    myServo.write(0);     // đóng lại
    lastFireAlert = false;
  }

  return alert;
}

// ================== CẢNH BÁO CHỐNG TRỘM ==================
bool checkTheftAlert() {
  if (!systemEnabled || !antiTheftEnabled) return false;

  bool motion = (pcf.readButton(PIR_PIN) == 1);

  if (motion && !lastTheftAlert && millis() - lastPirTrigger > pirCooldown) {
    lastPirTrigger = millis();
    lastTheftAlert = true;
    Serial.println("--- Motion detected. Starting PC analysis...");

    if (Blynk.connected()) { 
      Blynk.virtualWrite(V12, "🚨 Phát hiện chuyển động lạ, đang quét...");
      Blynk.logEvent("theft_alert", "🚨 Phát hiện chuyển động lạ!");
    }

    WiFiClient client;
    HTTPClient http;
    http.begin(client, pcServer);
    http.setTimeout(5000); // timeout 5s
    int httpCode = http.GET();
    
    Serial.print("HTTP response code: "); Serial.println(httpCode);

    if (httpCode == 200) {
      String payload = http.getString();
      payload.trim(); 
      payload.toLowerCase(); 
      
      if (payload == "owner") { 
        myServo.write(120); 
        digitalWrite(BUZZER_PIN, LOW); 
        if (Blynk.connected()) Blynk.virtualWrite(V12, "✅ Chủ nhà được xác nhận, cửa mở 20s"); 

        timer.setTimeout(OWNER_LOCKOUT_TIME, []() {
          myServo.write(0); 
          if (Blynk.connected()) Blynk.virtualWrite(V12, "🚪 Cửa tự động đóng (Nhận diện)"); 
        });

        lastPirTrigger = millis() + OWNER_LOCKOUT_TIME; 
        lastTheftAlert = false; 
        Serial.println("--- Analysis complete: Owner recognized.");
      
      } else {
        digitalWrite(BUZZER_PIN, HIGH); 
        if (Blynk.connected()) Blynk.virtualWrite(V12, "🚨 Người lạ hoặc lỗi!");
        Serial.println("--- Analysis complete: Intruder or error.");
      }
      
    } else {
       if (Blynk.connected()) Blynk.virtualWrite(V12, "Lỗi kết nối PC server"); 
       Serial.println("--- HTTP Error or Timeout occurred.");
    }
    http.end();

  } else if (!motion && lastTheftAlert && millis() - lastPirTrigger > pirCooldown) {
    lastTheftAlert = false;
    if (Blynk.connected()) { 
      Blynk.virtualWrite(V12, "✅ Khu vực an toàn");
      Blynk.logEvent("theft_safe", "✅ Không còn chuyển động lạ");
    }
    digitalWrite(BUZZER_PIN, LOW); 
  }
  return motion;
}
// ================== BUZZER ==================
void updateBuzzer(bool alertActive) {
  if (!systemEnabled) {
    digitalWrite(BUZZER_PIN, LOW); 
    buzzerState = false;
    return;
  }

  unsigned long now = millis();
  bool theftActive = lastTheftAlert && (myServo.read() == 0); 

  if (alertActive || theftActive) {
    if (now - buzzerPreviousMillis >= buzzerInterval) {
      buzzerPreviousMillis = now;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW); 
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW); 
    buzzerState = false;
  }
}

// ================== LCD ==================
void updateLCD() {
  char line1[20];
  char line2[20];

  sprintf(line1, "Time %02d:%02d", hour(), minute());
  sprintf(line2, "%s %c %s %s",
          systemEnabled ? "ON" : "OFF",
          fireGasAlertEnabled ? 'A' : '-',
          autoMode ? "Auto" : "Man",
          antiTheftEnabled ? "Safe" : "Open");

  if (strcmp(line1, lastLine1) != 0) {
    lcd.setCursor(0, 0);
    lcd.print("                   ");
    lcd.setCursor(0, 0);
    lcd.print(line1);
    strcpy(lastLine1, line1);
  }

  if (strcmp(line2, lastLine2) != 0) {
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print(line2);
    strcpy(lastLine2, line2);
  }
}

// ================== LOOP ==================
void loop() {
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();
  handleButtons();
  handleKeypad();
}

// ================== BLYNK CALLBACKS ==================
BLYNK_WRITE(V1) {
  relay1State = param.asInt();
  if (systemEnabled) {
    pcf.write(RELAY1_PIN, relay1State ? 0 : 1); 
  }
  Blynk.virtualWrite(V1, relay1State);
}

BLYNK_WRITE(V2) {
  relay2State = param.asInt();
  if (systemEnabled && !autoMode) {
    pcf.write(RELAY2_PIN, relay2State ? 0 : 1);
  }
  Blynk.virtualWrite(V2, relay2State);
}

BLYNK_WRITE(V6) {
  tempThreshold = param.asFloat(); 
  Blynk.virtualWrite(V6, tempThreshold);
}

BLYNK_WRITE(V5) {
  autoMode = param.asInt();   
  Blynk.virtualWrite(V5, autoMode);
}

BLYNK_WRITE(V8) {
  fireGasAlertEnabled = param.asInt();
  lastFireAlert = false;    
  myServo.write(0);         
  Blynk.virtualWrite(V8, fireGasAlertEnabled);
}

BLYNK_WRITE(V13) {
  antiTheftEnabled = param.asInt();
  Blynk.virtualWrite(V13, antiTheftEnabled);
}

BLYNK_WRITE(V14) {
  systemEnabled = param.asInt();
  digitalWrite(RELAY_MASTER_GPIO, systemEnabled ? LOW : HIGH);
  Blynk.virtualWrite(V14, systemEnabled);
}

BLYNK_WRITE(V15) {
  if (param.asInt() == 1) { 
    if (myServo.read() == 0) {
      openDoorTemporarily();
    } else {
      closeDoorNow();
    }
  }
}
