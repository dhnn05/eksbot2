/*
  ESP32-CAM: Dual Mode Firmware
  ------------------------------
  MODE STREAM : serve JPEG di http://<ip>/cam.jpg  (untuk kalibrasi visual di PC)
  MODE DETECT : capture frame RGB565 kecil, cari blob warna target di board,
                kirim koordinat centroid via UDP ke PC (tanpa gambar).

  Endpoint kontrol dari PC:
    GET /setcolor?hmin=..&hmax=..&smin=..&smax=..&vmin=..&vmax=..
        -> set threshold warna target (H:0-179, S:0-255, V:0-255, skala ala OpenCV)
    GET /setdest?ip=<pc_ip>&port=<pc_port>
        -> set tujuan pengiriman UDP koordinat
    GET /mode/stream
        -> pindah ke mode streaming JPEG (kalibrasi)
    GET /mode/detect
        -> pindah ke mode deteksi + kirim UDP (butuh /setdest sudah dipanggil dulu)

  Board: AI-Thinker ESP32-CAM (OV2640)
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include "esp_camera.h"

// ================= KONFIGURASI WIFI =================
const char* WIFI_SSID = "iPhone";
const char* WIFI_PASS = "aaaaaaaa";

// ================= PIN KAMERA (AI-Thinker) =================
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

WebServer server(80);
WiFiUDP udp;

// ================= STATE =================
enum Mode { MODE_STREAM, MODE_DETECT };
volatile Mode currentMode = MODE_STREAM;

// Threshold warna target (default: contoh biru, skala ala OpenCV H:0-179)
int targetHmin = 92, targetHmax = 142;
int targetSmin = 57,  targetSmax = 255;
int targetVmin = 50,  targetVmax = 255;

// Tujuan UDP
IPAddress destIP(192, 168, 1, 100);
uint16_t destPort = 5005;
bool destSet = false;

// Resolusi mode deteksi (QQVGA = 160x120). Kecil = cepat.
#define DETECT_W 160
#define DETECT_H 120

// Ambil tiap N piksel (1 = semua piksel, 2 = lompat 1, dst) untuk mempercepat scan.
// Naikkan angka ini kalau frame rate deteksi masih kurang cepat.
#define PIXEL_STRIDE 2

// Minimum jumlah piksel cocok supaya dianggap "kubus terdeteksi" (bukan noise)
#define MIN_MATCH_PIXELS 25

// =====================================================
// INIT KAMERA
// =====================================================
bool initCameraJPEG() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  // 320x240 -- cukup untuk kalibrasi, jauh lebih ringan dari VGA
  config.jpeg_quality = 12;
  config.fb_count = 2;

  return esp_camera_init(&config) == ESP_OK;
}

bool initCameraRGB() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA;  // 160x120
  config.fb_count = 1;

  return esp_camera_init(&config) == ESP_OK;
}

void switchToStream() {
  esp_camera_deinit();
  delay(50);
  initCameraJPEG();
  currentMode = MODE_STREAM;
  Serial.println("-> MODE STREAM");
}

void switchToDetect() {
  esp_camera_deinit();
  delay(50);
  initCameraRGB();
  currentMode = MODE_DETECT;
  Serial.println("-> MODE DETECT");
}

// =====================================================
// KONVERSI RGB565 -> HSV (skala H:0-179 ala OpenCV)
// =====================================================
void rgb565ToHSV(uint16_t pixel, uint8_t &h, uint8_t &s, uint8_t &v) {
  uint8_t r5 = (pixel >> 11) & 0x1F;
  uint8_t g6 = (pixel >> 5) & 0x3F;
  uint8_t b5 = pixel & 0x1F;

  uint8_t r = (r5 * 527 + 23) >> 6;
  uint8_t g = (g6 * 259 + 33) >> 6;
  uint8_t b = (b5 * 527 + 23) >> 6;

  uint8_t maxc = max(r, max(g, b));
  uint8_t minc = min(r, min(g, b));
  int16_t delta = maxc - minc;

  v = maxc;
  s = (maxc == 0) ? 0 : (uint16_t)delta * 255 / maxc;

  int16_t hue;
  if (delta == 0) {
    hue = 0;
  } else if (maxc == r) {
    hue = (int16_t)(30.0f * (g - b) / delta);
    if (hue < 0) hue += 180;
  } else if (maxc == g) {
    hue = (int16_t)(30.0f * (b - r) / delta) + 60;
  } else {
    hue = (int16_t)(30.0f * (r - g) / delta) + 120;
  }
  h = (uint8_t)hue;
}

// =====================================================
// DETEKSI BLOB WARNA
// =====================================================
// Catatan: sengaja pakai output parameter (bukan return struct custom) karena
// Arduino IDE auto-prototype generator sering error kalau function return
// type-nya struct custom yang didefinisikan di file yang sama ("does not
// name a type"). Dengan bool return + parameter by-reference, masalah ini
// tidak muncul.
bool detectColor(camera_fb_t *fb, int &outCx, int &outCy, int &outArea) {
  uint16_t *pixels = (uint16_t *)fb->buf;
  long sumX = 0, sumY = 0;
  int count = 0;

  for (int y = 0; y < fb->height; y += PIXEL_STRIDE) {
    for (int x = 0; x < fb->width; x += PIXEL_STRIDE) {
      uint16_t px = pixels[y * fb->width + x];
      px = (px >> 8) | (px << 8); // byte swap (sensor kirim big-endian)

      uint8_t h, s, v;
      rgb565ToHSV(px, h, s, v);

      if (h >= targetHmin && h <= targetHmax &&
          s >= targetSmin && s <= targetSmax &&
          v >= targetVmin && v <= targetVmax) {
        sumX += x;
        sumY += y;
        count++;
      }
    }
  }

  if (count > MIN_MATCH_PIXELS) {
    outCx = sumX / count;
    outCy = sumY / count;
    outArea = count * PIXEL_STRIDE * PIXEL_STRIDE; // estimasi area asli
    return true;
  }
  return false;
}

// =====================================================
// HTTP HANDLERS
// =====================================================
void serveJpg() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(503, "", "");
    return;
  }
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// MJPEG multipart stream: satu koneksi tetap terbuka, ESP terus push frame.
// Jauh lebih cepat dibanding request JPEG satu-satu (menghindari overhead
// TCP handshake + HTTP header per frame).
// Catatan: karena WebServer library ini single-threaded, selama endpoint ini
// aktif, request HTTP lain (setcolor/setdest/mode) TIDAK akan diproses.
// Di sisi PC, putuskan koneksi stream dulu (cap.release()) sebelum memanggil
// endpoint lain seperti /mode/detect.
void handleStream() {
  WiFiClient client = server.client();

  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (client.connected() && currentMode == MODE_STREAM) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }

    server.sendContent("--frame\r\n");
    server.sendContent("Content-Type: image/jpeg\r\n");
    server.sendContent("Content-Length: " + String(fb->len) + "\r\n\r\n");
    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb);

    if (!client.connected()) break;
  }
}

void handleSetColor() {
  if (server.hasArg("hmin")) targetHmin = server.arg("hmin").toInt();
  if (server.hasArg("hmax")) targetHmax = server.arg("hmax").toInt();
  if (server.hasArg("smin")) targetSmin = server.arg("smin").toInt();
  if (server.hasArg("smax")) targetSmax = server.arg("smax").toInt();
  if (server.hasArg("vmin")) targetVmin = server.arg("vmin").toInt();
  if (server.hasArg("vmax")) targetVmax = server.arg("vmax").toInt();
  server.send(200, "text/plain", "OK");
}

void handleSetDest() {
  if (server.hasArg("ip")) {
    destIP.fromString(server.arg("ip"));
    destSet = true;
  }
  if (server.hasArg("port")) destPort = server.arg("port").toInt();
  server.send(200, "text/plain", "OK dest=" + destIP.toString() + ":" + String(destPort));
}

void handleModeStream() {
  switchToStream();
  server.send(200, "text/plain", "Mode: STREAM");
}

void handleModeDetect() {
  if (!destSet) {
    server.send(400, "text/plain", "Panggil /setdest?ip=..&port=.. dulu sebelum mode detect");
    return;
  }
  switchToDetect();
  server.send(200, "text/plain", "Mode: DETECT");
}

void handleStatus() {
  String s = "mode=" + String(currentMode == MODE_STREAM ? "stream" : "detect");
  s += " H[" + String(targetHmin) + "-" + String(targetHmax) + "]";
  s += " S[" + String(targetSmin) + "-" + String(targetSmax) + "]";
  s += " V[" + String(targetVmin) + "-" + String(targetVmax) + "]";
  s += " dest=" + destIP.toString() + ":" + String(destPort);
  server.send(200, "text/plain", s);
}

// =====================================================
// SETUP & LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  WiFi.setSleep(false); // matikan modem-sleep -> hilangkan lag/delay random dari WiFi power-save
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  initCameraJPEG(); // mulai di mode stream untuk kalibrasi

  server.on("/cam.jpg", serveJpg);
  server.on("/stream", handleStream);
  server.on("/setcolor", handleSetColor);
  server.on("/setdest", handleSetDest);
  server.on("/mode/stream", handleModeStream);
  server.on("/mode/detect", handleModeDetect);
  server.on("/status", handleStatus);
  server.begin();

  udp.begin(0);

  Serial.println("Endpoint siap:");
  Serial.println("  /cam.jpg");
  Serial.println("  /setcolor?hmin=&hmax=&smin=&smax=&vmin=&vmax=");
  Serial.println("  /setdest?ip=&port=");
  Serial.println("  /mode/stream  |  /mode/detect");
  Serial.println("  /status");
}

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 50; // ~20Hz maksimum

void loop() {
  server.handleClient();

  if (currentMode == MODE_DETECT) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      int cx = 0, cy = 0, area = 0;
      bool found = detectColor(fb, cx, cy, area);
      esp_camera_fb_return(fb);

      if (millis() - lastSend > SEND_INTERVAL_MS) {
        char buf[80];
        if (found) {
          snprintf(buf, sizeof(buf),
                    "{\"found\":1,\"x\":%d,\"y\":%d,\"area\":%d,\"w\":%d,\"h\":%d}",
                    cx, cy, area, DETECT_W, DETECT_H);
        } else {
          snprintf(buf, sizeof(buf), "{\"found\":0}");
        }
        udp.beginPacket(destIP, destPort);
        udp.write((const uint8_t *)buf, strlen(buf));
        udp.endPacket();
        lastSend = millis();
      }
    }
  }
}
