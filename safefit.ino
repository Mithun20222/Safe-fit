#include <ESP8266WiFi.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <MAX3010x.h>
#include "filters.h"
#include "Secrets.h"
#include <ESPSupabase.h>

Supabase supabase;

// SIM800L & Button
SoftwareSerial sim800l(D7, D8); // SIM800L TX, RX
const int buttonPin = D3;
unsigned long lastPress = 0;
int pressCount = 0;
const unsigned long doublePressGap = 500;
String phoneNumber = "ur mobile number"; // your target SMS number

// === MPU6050 ===
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax, ay, az, gx, gy, gz;
float vectorPrevious = 0;
float vectorCurrent = 0;
float totalVectorChange = 0;
int steps = 0;

// === GPS ===
TinyGPSPlus gps;
SoftwareSerial gpsSerial(D6, D5); // RX, TX
float lastLat = 0.0, lastLng = 0.0;

// === MAX30102 ===
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

LowPassFilter low_pass_filter_red(5.0, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(5.0, kSamplingFrequency);
HighPassFilter high_pass_filter(0.5, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<5> averager_bpm;
MovingAverageFilter<5> averager_spo2;
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

long last_heartbeat = 0;
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  sim800l.begin(9600);
  Wire.begin();
  pinMode(buttonPin, INPUT_PULLUP);

  // Connect Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // Initialize Supabase
  supabase.begin(SUPABASE_URL, SUPABASE_ANON_KEY);

  // Initialize MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Initialize MAX30102
  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("MAX30102 Initialized");
  } else {
    Serial.println("MAX30102 Not Found");
    while (1);
  }

  Serial.println("System Initialized");
}

void loop() {
  handleButton();

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    yield();
  }

  mpu_read();
  totalVectorChange = abs(sqrt(AcX * AcX + AcY * AcY + AcZ * AcZ) - vectorPrevious);
  if (totalVectorChange > 8000) {
    steps++;
    vectorPrevious = sqrt(AcX * AcX + AcY * AcY + AcZ * AcZ);
    delay(250);
  }

  auto sample = sensor.readSample(10);
  if (sample.red < 10000 || sample.ir < 10000) return;

  float red = low_pass_filter_red.process(sample.red);
  float ir = low_pass_filter_ir.process(sample.ir);
  stat_red.process(red);
  stat_ir.process(ir);
  float diff = differentiator.process(high_pass_filter.process(red));

  if (!isnan(diff) && !isnan(last_diff)) {
    if (last_diff > 0 && diff < 0) {
      crossed = true;
      crossed_time = millis();
    }
    if (diff > 0) crossed = false;

    if (crossed && diff < -2000) {
      if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
        int bpm = 60000 / (crossed_time - last_heartbeat);
        float r = ((stat_red.maximum() - stat_red.minimum()) / stat_red.average()) /
                  ((stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average());
        float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

        if (bpm > 50 && bpm < 250) {
          int average_bpm = averager_bpm.process(bpm);
          int average_spo2 = averager_spo2.process(spo2);
          if (averager_bpm.count() >= 3) {
            Serial.printf("BPM: %d\nSpO2: %.2f\nSteps: %d\n", average_bpm, average_spo2, steps);
            displayGPS();
            static unsigned long lastUpload = 0;
            if (millis() - lastUpload > 10000) {
              uploadToSupabase(average_bpm, average_spo2, steps, lastLat, lastLng);
              lastUpload = millis();
            }
          }
        }
        stat_red.reset(); stat_ir.reset();
      }
      crossed = false;
      last_heartbeat = crossed_time;
    }
  }
  last_diff = diff;
  delay(100);
}

// --- Handle Button for Double Press ---
void handleButton() {
  static unsigned long lastPressTime = 0;
  static int pressCount = 0;
  const unsigned long debounceDelay = 50;   // for stable press detection
  const unsigned long doublePressWindow = 1000; // 1 second window for double press

  int buttonState = digitalRead(buttonPin);
  static int lastButtonState = HIGH;

  if (buttonState != lastButtonState && buttonState == LOW) {
    unsigned long now = millis();

    if (now - lastPressTime > debounceDelay) {
      pressCount++;
      lastPressTime = now;
    }
  }

  // Check for double press within 1 second
  if (pressCount == 2 && millis() - lastPressTime < doublePressWindow) {
    Serial.println("Double press detected! Sending SMS...");
    sendSMSWithGPS();
    pressCount = 0;
  }

  // Reset press count if no second press within window
  if (pressCount == 1 && millis() - lastPressTime > doublePressWindow) {
    pressCount = 0;
  }

  lastButtonState = buttonState;
}


// --- SMS with GPS ---
void sendSMSWithGPS() {
  String msg;
  if (gps.location.isValid()) {
    msg = "Emergency! Location: https://maps.google.com/?q=" +
          String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  } else {
    msg = "Emergency! Last known: https://maps.google.com/?q=" +
          String(lastLat, 6) + "," + String(lastLng, 6);
  }

  sim800l.println("AT");
  delay(1000);
  sim800l.println("AT+CMGF=1");
  delay(500);
  sim800l.print("AT+CMGS=\"");
  sim800l.print(phoneNumber);
  sim800l.println("\"");
  delay(500);
  sim800l.println(msg);
  delay(300);
  sim800l.write(26);
  delay(5000);
  Serial.println("SMS Sent Successfully!");
}

// --- Supabase Upload ---
void uploadToSupabase(int bpm, float spo2, int steps, float lat, float lng) {
  DynamicJsonDocument doc(512);
  doc["bpm"] = bpm;
  doc["spo2"] = spo2;
  doc["steps"] = steps;
  doc["latitude"] = lat;
  doc["longitude"] = lng;

  String payload;
  serializeJson(doc, payload);
  int status = supabase.insert("sensor_data", payload, false);

  if (status == 201) Serial.println("Data uploaded successfully!");
  else {
    Serial.print("Upload failed: ");
    Serial.println(status);
  }
}

// --- MPU6050 Reading ---
void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_addr, (size_t)14, (bool)true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// --- GPS Display ---
void displayGPS() {
  Serial.print("Location: ");
  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    Serial.printf("%.6f, %.6f\n", lastLat, lastLng);
  } else {
    Serial.printf("GPS connecting... last known: %.6f, %.6f\n", lastLat, lastLng);
  }
}
