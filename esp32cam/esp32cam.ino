#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ====== WiFi ======
const char* ssid = "Home";
const char* password = "123456789";

WebServer server(80);

// ====== Pinout ESP32-CAM AI Thinker ======
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ====== Flash LED ======
#define FLASH_LED_PIN 4
bool flashOn = false;

// ====== Cấu hình camera ======
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_SVGA;       // 800x600
  config.pixel_format = PIXFORMAT_JPEG;    // JPEG
  config.jpeg_quality = 12;                // chất lượng vừa phải
  config.fb_count = 2;                     // 2 buffer để stream mượt hơn

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true) delay(1000);
  }
}

// ====== Handler chụp ảnh ======
void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Content-Type", "image/jpeg");
  server.send_P(200, "image/jpeg", (char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ====== Handler stream video ======
void handleStream() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "multipart/x-mixed-replace; boundary=frame");

  while (true) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) continue;

    server.sendContent("--frame\r\n");
    server.sendContent("Content-Type: image/jpeg\r\n\r\n");
    server.sendContent((const char*)fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb);

    if (!server.client().connected()) break;
  }
}

// ====== Handler flash LED ======
void handleFlash() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      digitalWrite(FLASH_LED_PIN, HIGH);
      flashOn = true;
      server.send(200, "text/plain", "Flash ON");
    } else if (state == "off") {
      digitalWrite(FLASH_LED_PIN, LOW);
      flashOn = false;
      server.send(200, "text/plain", "Flash OFF");
    } else {
      server.send(400, "text/plain", "Invalid state");
    }
  } else {
    server.send(400, "text/plain", "Missing state arg");
  }
}

// ====== Handler kiểm tra trạng thái flash ======
void handleFlashStatus() {
  if (flashOn) {
    server.send(200, "text/plain", "Flash is ON");
  } else {
    server.send(200, "text/plain", "Flash is OFF");
  }
}

// ====== WiFi reconnect ======
void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
      delay(100);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi reconnected");
      Serial.print("ESP32-CAM IP: ");
      Serial.println(WiFi.localIP());
    }
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32-CAM IP: ");
  Serial.println(WiFi.localIP());

  setupCamera();

  // Luôn tắt đèn khi khởi động
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  flashOn = false;

  // Khai báo các endpoint
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/flash", HTTP_GET, handleFlash);
  server.on("/flash/status", HTTP_GET, handleFlashStatus);
  server.begin();
}

// ====== Loop ======
void loop() {
  checkWiFi();
  server.handleClient();
}
