#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>

const char *ap_ssid = "RC_Tracker_Net";
const char *ap_password = "password123";
const unsigned int local_udp_port = 4242;

#define IMU_SDA_PIN 22
#define IMU_SCL_PIN 21

#define SD_CS_PIN 5
#define SD_SCK_PIN 18
#define SD_MOSI_PIN 19
#define SD_MISO_PIN 23

#define LED_SYNC_PIN 2

Adafruit_LSM6DS3 lsm6ds3;
WiFiUDP udp;
WebServer server(80);

File log_file;
String active_filename = "";
int64_t time_offset_us = 0;
bool is_logging = false;
uint32_t sample_count = 0;

float accelBiasX = 0.0f;
float accelBiasY = 0.0f;
float accelBiasZ = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

int64_t get_synced_unix_time_us() {
  return time_offset_us + esp_timer_get_time();
}

String get_next_filename() {
  char filename_buffer[32];
  for (int i = 1; i <= 999; i++) {
    snprintf(filename_buffer, sizeof(filename_buffer), "/imu_run_%03d.csv", i);
    if (!SD.exists(filename_buffer)) {
      return String(filename_buffer);
    }
  }
  return "/imu_run_999.csv";
}

bool readIMURaw(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  if (lsm6ds3.getEvent(&accel, &gyro, &temp)) {
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;
    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;
    return true;
  }
  return false;
}

void calibrateIMU() {
  const int numSamples = 300;
  float aSumX = 0.0f;
  float aSumY = 0.0f;
  float aSumZ = 0.0f;
  float gSumX = 0.0f;
  float gSumY = 0.0f;
  float gSumZ = 0.0f;
  int validSamples = 0;

  Serial.println("calibrating imu biases, keep vehicle still");

  for (int i = 0; i < numSamples; i++) {
    float gx, gy, gz, ax, ay, az;

    if (readIMURaw(gx, gy, gz, ax, ay, az)) {
      aSumX += ax;
      aSumY += ay;
      aSumZ += az;
      gSumX += gx;
      gSumY += gy;
      gSumZ += gz;
      validSamples++;
    }

    if (i % 25 == 0) {
      server.handleClient();
    }
    delay(5);
  }

  if (validSamples > 0) {
    accelBiasX = aSumX / (float)validSamples;
    accelBiasY = aSumY / (float)validSamples;
    accelBiasZ = (aSumZ / (float)validSamples) + 9.81f;

    gyroBiasX = gSumX / (float)validSamples;
    gyroBiasY = gSumY / (float)validSamples;
    gyroBiasZ = gSumZ / (float)validSamples;

    Serial.println("imu calibration completed");
  }
}

void start_new_logging_session(int64_t host_time_us) {
  if (is_logging) {
    log_file.close();
  }

  time_offset_us = host_time_us - esp_timer_get_time();
  active_filename = get_next_filename();

  log_file = SD.open(active_filename, FILE_WRITE);
  if (log_file) {
    log_file.println("timestamp_unix_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
    log_file.flush();
    is_logging = true;
    sample_count = 0;

    digitalWrite(LED_SYNC_PIN, HIGH);
    delay(50);
    digitalWrite(LED_SYNC_PIN, LOW);

    Serial.print("started logging session: ");
    Serial.println(active_filename);
  } else {
    Serial.println("failed to create log file on sd card");
  }
}

void stop_logging_session() {
  if (is_logging) {
    log_file.flush();
    log_file.close();
    is_logging = false;
    Serial.print("stopped logging session: ");
    Serial.println(active_filename);
  }
}

void handle_root() {
  String html = "<html><body>";
  html += "<h1>ESP32 Telemetry Status</h1>";
  html += "<p>Active File: " + active_filename + "</p>";
  html += "<p>Status: " + String(is_logging ? "Logging" : "Standby") + "</p>";
  html += "<p>Logged Samples: " + String(sample_count) + "</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_SYNC_PIN, OUTPUT);
  digitalWrite(LED_SYNC_PIN, LOW);

  Serial.println("starting access point");
  WiFi.softAP(ap_ssid, ap_password);

  IPAddress my_ip = WiFi.softAPIP();
  Serial.print("access point IP address: ");
  Serial.println(my_ip);

  udp.begin(local_udp_port);
  server.on("/", handle_root);
  server.begin();

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  bool imu_found = lsm6ds3.begin_I2C(0x6A, &Wire) || lsm6ds3.begin_I2C(0x6B, &Wire);

  if (!imu_found) {
    Serial.println("imu initialization failed");
    while (1) {
      server.handleClient();
      delay(10);
    }
  }

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("sd card initialization failed");
    while (1) {
      server.handleClient();
      delay(10);
    }
  }

  Serial.println("esp32 ready in standby, waiting for commands");
}

void loop() {
  server.handleClient();

  char packet_buffer[255];
  int packet_size = udp.parsePacket();
  if (packet_size) {
    int len = udp.read(packet_buffer, 255);
    if (len > 0) {
      packet_buffer[len] = 0;
    }

    String msg = String(packet_buffer);

    if (msg.equals("CALIBRATE")) {
      calibrateIMU();

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("ACK:CALIB_DONE");
      udp.endPacket();

    } else if (msg.startsWith("START:")) {
      int64_t host_time_us = atoll(msg.substring(6).c_str());
      start_new_logging_session(host_time_us);

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("ACK:OK");
      udp.endPacket();

    } else if (msg.equals("STOP")) {
      stop_logging_session();

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("ACK:STOP");
      udp.endPacket();
    }
  }

  if (!is_logging || time_offset_us == 0) {
    delay(5);
    return;
  }

  float sGx, sGy, sGz, sAx, sAy, sAz;

  if (!readIMURaw(sGx, sGy, sGz, sAx, sAy, sAz)) {
    return;
  }

  // Subtract static biases measured during calibration
  float rawAx = sAx - accelBiasX;
  float rawAy = sAy - accelBiasY;
  float rawAz = sAz - accelBiasZ;

  float rawGx = sGx - gyroBiasX;
  float rawGy = sGy - gyroBiasY;
  float rawGz = sGz - gyroBiasZ;

  // Remap raw acceleration and gyro directly to vehicle frame
  float curAx = -rawAy;
  float curAy = rawAx;
  float curAz = rawAz;

  float curGx = -rawGy;
  float curGy = rawGx;
  float curGz = rawGz;

  int64_t ts_us = get_synced_unix_time_us();

  log_file.printf("%lld,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                  ts_us,
                  curAx, curAy, curAz,
                  curGx, curGy, curGz);

  sample_count++;

  static uint32_t last_flush = 0;
  if (millis() - last_flush > 1000) {
    log_file.flush();
    last_flush = millis();
  }

  delay(5);
}