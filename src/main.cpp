#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// ========== НАСТРОЙКИ ========== //
const char* ssid = "MGTS_GPON_2357"; // Имя вашей Wi-Fi сети
const char* password = "HRBPPWRD"; // Пароль от вашей Wi-fi сети
const char* mqtt_server = "192.168.1.75"; // IP адрес вашего MQTT брокера
const char* mqtt_topic = "sensors/esp32_data"; // Топик для отправки данных

const char* target_ble_name = "ESP32_Beacon"; // Имя вашего BLE маяка (ошейника)
const int mpu_address = 0x68; // Адрес MPU-6050
const float accel_sensitivity = 16384.0; // ±2g

const float ACCEL_THRESHOLD = 2.0; // Порог для ускорения
const long FAST_SEND_INTERVAL = 5000; // 5 секунд
const long SLOW_SEND_INTERVAL = 15000; // 15 секунд
// =============================== //

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// BLE Scanner не нужен для ошейника, так как он только маяк
// BLEScan* bleScanner;
// int ble_rssi = -100;
// bool ble_beacon_found = false;

// Для отслеживания времени отправки
long last_send_time = 0;
float previous_acceleration = 0.0;

// Класс для BLE-маяка (AdvertisedDeviceCallbacks не нужен, так как не сканируем)
// Вместо этого используем BLEAdvertising
BLEAdvertising *pAdvertising;

void setupWifi() {
  delay(10);
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupMqtt() {
  mqttClient.setServer(mqtt_server, 1883);
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32_Sensor_Node")) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5s");
      delay(5000);
    }
  }
}

void setupMpu6050() {
  Wire.begin();
  Wire.beginTransmission(mpu_address);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
}

void setupBleBeacon() {
  BLEDevice::init(target_ble_name); // Инициализируем BLE с именем маяка
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE Beacon started with name: " + String(target_ble_name));
}

int16_t readMpuRegister(uint8_t reg) {
  Wire.beginTransmission(mpu_address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

float readAccelerometer() {
  int16_t ax = readMpuRegister(0x3B);
  int16_t ay = readMpuRegister(0x3D);
  int16_t az = readMpuRegister(0x3F);
  
  // Convert to g and calculate magnitude
  float acceleration = sqrt(
    pow(ax / accel_sensitivity, 2) + 
    pow(ay / accel_sensitivity, 2) + 
    pow(az / accel_sensitivity, 2)
  );
  
  return acceleration;
}

// Изменена функция отправки данных для акселерометра
void sendAccelerometerData(float acceleration) {
  String payload = "{";
  payload += "\"accel\":" + String(acceleration, 3);
  payload += "}";
  
  mqttClient.publish(mqtt_topic, payload.c_str());
  Serial.println("Sent: " + payload);
}

void setup() {
  setupWifi();
  setupMqtt();
  setupMpu6050();
  setupBleBeacon(); // Инициализация BLE как маяка
  last_send_time = millis(); // Инициализируем время последней отправки
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();
  
  float current_acceleration = readAccelerometer();
  
  long current_time = millis();
  long send_interval;

  if (previous_acceleration > ACCEL_THRESHOLD) {
    send_interval = FAST_SEND_INTERVAL;
  } else {
    send_interval = SLOW_SEND_INTERVAL;
  }

  if (current_time - last_send_time >= send_interval) {
    sendAccelerometerData(current_acceleration);
    last_send_time = current_time;
  }
  
  previous_acceleration = current_acceleration; // Обновляем предыдущее значение
  
  delay(100); // Небольшая задержка, чтобы не загружать процессор слишком сильно
}