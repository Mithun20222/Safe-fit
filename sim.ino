#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <MAX3010x.h>
#include "filters.h" // IMPORTANT: This file must be present in your project
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// WiFi Credentials
const char* ssid = "wifi ssid";
const char* password = "wifi pass";

// Supabase Configuration
const char* supabase_url = "url";
const char* supabase_key = "anon key";

// MPU6050 Accelerometer/Gyroscope
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
float vectorPrevious = 0;
float vectorCurrent = 0;
float totalVectorChange = 0;
int steps = 0;
boolean fall = false;
boolean trigger1 = false, trigger2 = false, trigger3 = false;
byte trigger1count = 0, trigger2count = 0, trigger3count = 0;
int angleChange = 0;

// === GPS ===
TinyGPSPlus gps;
SoftwareSerial gpsSerial(D6, D5); // RX, TX
float lastLat = 26.250992, lastLng = 78.172691;

// === MAX30102 Heart Rate and SpO2 Sensor ===
MAX30105 sensor;
bool max30102_active = false;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Filters (definitions expected in filters.h)
LowPassFilter low_pass_filter_red(5.0, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(5.0, kSamplingFrequency);
HighPassFilter high_pass_filter(0.5, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<5> averager_bpm;
MovingAverageFilter<5> averager_r;
MovingAverageFilter<5> averager_spo2;
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// SpO2 Calculation Coefficients
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

long last_heartbeat = 0;
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

// === SIM800L GSM Module ===
SoftwareSerial sim800(D7, D8); // RX, TX
String fallbackNumber = "+919492841569"; // Emergency contact number

// === Button for emergency SMS ===
#define BUTTON_PIN D3
unsigned long lastPressTime = 0;
int pressCount = 0;
unsigned long doubleClickWindow = 500; // Time window for double-click detection
bool buttonPreviouslyPressed = false;

// Function to initialize MAX30102 sensor
bool initializeMAX30102() {
  Serial.print("Initializing MAX30102...");
  Wire.begin(); // Initialize I2C communication
  delay(50);
  if (sensor.begin()) {
    delay(50);
    if (sensor.setSamplingRate(kSamplingRate)) {
      Serial.println(" Success!");
      return true;
    } else {
      Serial.println(" Failed to set sampling rate.");
    }
  }
  Serial.println(" Not Found or Failed to begin.");
  return false;
}

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  gpsSerial.begin(9600); // Initialize SoftwareSerial for GPS
  sim800.begin(9600);   // Initialize SoftwareSerial for SIM800L
  delay(1000);
  sim800.println("AT+CMGF=1"); // Set SIM800L to text mode
  delay(1000);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize MPU6050
  Wire.begin(); // Initialize I2C for MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Initialize MAX30102
  max30102_active = initializeMAX30102();

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin with internal pull-up
  Serial.println("System Initialized");
}

void loop() {
  checkDoubleClick(); // Check for double-click on the button

  // Read GPS data if available
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Read MPU6050 data
  mpu_read();
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  float Raw_Amp = sqrt(ax * ax + ay * ay + az * az);
  int Amp = Raw_Amp * 10;

  // === Fall Detection Logic ===
  // Trigger 1: Low amplitude (possible free fall)
  if (Amp <= 2 && !trigger2) { trigger1 = true; }
  if (trigger1) {
    trigger1count++;
    if (Amp >= 12) { // Followed by high amplitude (impact)
      trigger2 = true; trigger1 = false; trigger1count = 0;
    }
  }
  // Trigger 2: High amplitude detected
  if (trigger2) {
    trigger2count++;
    angleChange = sqrt(gx * gx + gy * gy + gz * gz);
    if (angleChange >= 30 && angleChange <= 400) { // Significant angular change
      trigger3 = true; trigger2 = false; trigger2count = 0;
    }
  }
  // Trigger 3: Angular change detected
  if (trigger3) {
    trigger3count++;
    if (trigger3count >= 10) { // After a short period
      angleChange = sqrt(gx * gx + gy * gy + gz * gz);
      if (angleChange >= 0 && angleChange <= 10) { // Followed by low angular change (lying still)
        fall = true;
      }
      trigger3 = false; trigger3count = 0;
    }
  }

  // Reset triggers if conditions not met within time
  if (trigger1count >= 6) { trigger1 = false; trigger1count = 0; }
  if (trigger2count >= 6) { trigger2 = false; trigger2count = 0; }

  // === Step Counting Logic ===
  vectorCurrent = sqrt(AcX * AcX + AcY * AcY + AcZ * AcZ);
  totalVectorChange = abs(vectorCurrent - vectorPrevious);
  if (totalVectorChange > 8000) { // Threshold for step detection
    steps++;
    vectorPrevious = vectorCurrent;
    delay(250); // Debounce delay for step counting
  }

  // === MAX30102 Auto-Check and Reading ===
  if (!max30102_active) {
    Serial.println("MAX30102 inactive. Retrying init...");
    max30102_active = initializeMAX30102();
    if (!max30102_active) {
      delay(1000); // Wait before retrying if init failed
      return;
    }
  }

  // Read a sample from the MAX30102 sensor
  auto sample = sensor.readSample(10); // Read 10 samples for better accuracy
  if (sample.red < 10000 || sample.ir < 10000) {
    Serial.println("No finger detected on MAX30102.");
    max30102_active = false; // Mark as inactive if no finger
    return;
  }

  // Process sensor data through filters
  float red = low_pass_filter_red.process(sample.red);
  float ir = low_pass_filter_ir.process(sample.ir);
  stat_red.process(red);
  stat_ir.process(ir);
  float filtered = high_pass_filter.process(red);
  float diff = differentiator.process(filtered);

  if (!isnan(diff) && !isnan(last_diff)) {
    // Detect heartbeat peak (zero crossing of the derivative)
    if (last_diff > 0 && diff < 0) {
      crossed = true;
      crossed_time = millis();
    }
    if (diff > 0) crossed = false;

    // When a heartbeat is detected (derivative crosses zero and is significantly negative)
    if (crossed && diff < -2000) {
      if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) { // Prevent false positives
        int bpm = 60000 / (crossed_time - last_heartbeat); // Calculate BPM
        // Calculate R ratio for SpO2
        float r = ((stat_red.maximum() - stat_red.minimum()) / stat_red.average()) /
                  ((stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average());
        // Calculate SpO2 using empirical formula
        float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

        // Clamp SpO2 to realistic range
        if (spo2 > 100.0) spo2 = 100.0;
        if (spo2 < 70.0) spo2 = 70.0;

        // Filter BPM and SpO2 with moving average
        if (bpm > 50 && bpm < 250) { // Valid BPM range
          int average_bpm = averager_bpm.process(bpm);
          int average_spo2 = averager_spo2.process(spo2);

          // Only send data once enough samples for averaging are collected
          if (averager_bpm.count() >= 3) {
            Serial.print("BPM: "); Serial.println(average_bpm);
            Serial.print("SpO2: "); Serial.println(average_spo2);
            Serial.print("Steps: "); Serial.println(steps);
            displayGPS(); // Display current GPS status

            // Send data to Supabase if connected to WiFi
            if (WiFi.status() == WL_CONNECTED) {
              WiFiClient client;
              HTTPClient http;

              // Corrected: Single declaration of payload and added missing http.begin() and headers
              String payload = "{";
              payload += "\"bpm\": " + String(average_bpm) + ",";
              payload += "\"spo2\": " + String(average_spo2) + ",";
              payload += "\"steps\": " + String(steps) + ",";
              payload += "\"fall\": " + String(fall ? "true" : "false") + ",";
              payload += "\"latitude\": " + String(lastLat, 6) + ",";
              payload += "\"longitude\": " + String(lastLng, 6);
              payload += "}";

              Serial.println("Sending payload:");
              Serial.println(payload);

              // Begin HTTP request
              http.begin(client, supabase_url); // Corrected: Added http.begin() with client and URL
              http.addHeader("Content-Type", "application/json"); // Required for JSON payload
              http.addHeader("apikey", supabase_key);             // Supabase API key
              http.addHeader("Authorization", "Bearer " + String(supabase_key)); // Supabase Authorization header

              int httpCode = http.POST(payload); // Perform POST request
              Serial.print("HTTP Response Code: ");
              Serial.println(httpCode);

              if (httpCode > 0) {
                // HTTP header has been send and server response header has been handled
                Serial.println("Data sent to Supabase!");
                // String response = http.getString(); // Uncomment to read response from server
                // Serial.println(response);
              } else {
                Serial.printf("Failed to send data to Supabase, error: %s\n", http.errorToString(httpCode).c_str());
              }

              http.end(); // Free resources
            }

            // Check for fall and send emergency SMS
            if (fall) {
              Serial.println("Fallen! Sending emergency SMS...");
              sendEmergencySMS();
              fall = false; // Reset fall status after sending SMS
            }
          }
        }
        stat_red.reset(); // Reset statistics for next heartbeat
        stat_ir.reset();
      }
      crossed = false;
      last_heartbeat = crossed_time; // Update last heartbeat time
    }
  }
  last_diff = diff; // Store current derivative for next iteration
  delay(100); // Short delay to prevent busy-waiting
}

// Function to read MPU6050 sensor data
void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // Request 14 bytes (AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ)
  AcX = Wire.read() << 8 | Wire.read(); // Read accelerometer X
  AcY = Wire.read() << 8 | Wire.read(); // Read accelerometer Y
  AcZ = Wire.read() << 8 | Wire.read(); // Read accelerometer Z
  Tmp = Wire.read() << 8 | Wire.read(); // Read temperature
  GyX = Wire.read() << 8 | Wire.read(); // Read gyroscope X
  GyY = Wire.read() << 8 | Wire.read(); // Read gyroscope Y
  GyZ = Wire.read() << 8 | Wire.read(); // Read gyroscope Z
}

// Function to display GPS coordinates
void displayGPS() {
  Serial.print("Location: ");
  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    Serial.print(lastLat, 6); Serial.print(","); Serial.println(lastLng, 6);
  } else {
    Serial.print("Connecting... Last location: ");
    Serial.print(lastLat, 6); Serial.print(","); Serial.println(lastLng, 6);
  }
}

// Function to send emergency SMS via SIM800L
void sendEmergencySMS() {
  String msg = "Emergency! I am in DANGER.";
  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    msg += "\nMy current location: Lat: " + String(lastLat, 6) + ", Lng: " + String(lastLng, 6);
    msg += "\nMaps: https://maps.google.com/?q=" + String(lastLat, 6) + "," + String(lastLng, 6);
  } else if (lastLat != 0.0 && lastLng != 0.0) {
    msg += "\nLast known location: Lat: " + String(lastLat, 6) + ", Lng: " + String(lastLng, 6);
    msg += "\nMaps: https://maps.google.com/?q=" + String(lastLat, 6) + "," + String(lastLng, 6);
  } else {
    msg += "\nLocation not available at this time.";
  }

  sim800.print("AT+CMGS=\"");
  sim800.print(fallbackNumber);
  sim800.println("\"");
  delay(1000);
  sim800.print(msg);
  delay(100);
  sim800.write(0x1A); // Ctrl+Z to send SMS
  delay(3000); // Give SIM800L time to send
  Serial.println("SMS Sent to " + fallbackNumber + ":\n" + msg);
}

// Function to check for double-click on the button
void checkDoubleClick() {
  // Check if button is pressed (LOW due to INPUT_PULLUP) and was not previously pressed
  if (digitalRead(BUTTON_PIN) == LOW && !buttonPreviouslyPressed) {
    buttonPreviouslyPressed = true; // Mark button as currently pressed
    unsigned long currentTime = millis();
    // Check if the current press is within the double-click window
    if (currentTime - lastPressTime < doubleClickWindow) {
      pressCount++;
      if (pressCount == 2) {
        // Double-click detected!
        pressCount = 0; // Reset count
        Serial.println("Double click detected! Sending emergency SMS.");
        sendEmergencySMS(); // Trigger emergency SMS
      }
    } else {
      // First click or outside double-click window
      pressCount = 1;
    }
    lastPressTime = currentTime; // Update last press time
    delay(50); // Small debounce delay for button
  }
  // Reset buttonPreviouslyPressed when button is released
  if (digitalRead(BUTTON_PIN) == HIGH) {
    buttonPreviouslyPressed = false;
  }
}
