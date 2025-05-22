#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// ========== НАСТРОЙКИ ========== //
const char* ssid = "ssid";
const char* password = "passwrd";
const char* mqtt_server = "192.168.1.75";
const char* mqtt_topic = "sensors/esp32_data";

const char* target_ble_name = "ESP32_Beacon"; // Имя вашего BLE маяка (ошейника)
const int mpu_address = 0x68; // Адрес MPU-6050
const float accel_sensitivity = 16384.0; // ±2g

const float ACCEL_THRESHOLD = 2.0; // Порог для ускорения
const long FAST_SEND_INTERVAL = 5000; // 5 секунд
const long SLOW_SEND_INTERVAL = 15000; // 15 секунд

// Настройки NTP для получения точного времени
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3 * 3600; // Часовой пояс (например, для Центральной Европы +1 час = 3600 секунд)
const int daylightOffset_sec = 0; // Летнее время (если есть, +1 час = 3600 секунд)

// ========== НАКОПЛЕНИЕ ДАННЫХ ========== //
// Структура для хранения одного измерения
struct AccelData {
  float acceleration;
  time_t timestamp; // Время в секундах с эпохи Unix
};

// Буфер для накопления данных в ОЗУ
const int MAX_BUFFER_SIZE = 1000; // Максимальное количество записей в буфере (настраивается)
AccelData dataBuffer[MAX_BUFFER_SIZE];
int bufferIndex = 0; // Текущий индекс для записи

// =============================== //

WiFiClient espClient;
PubSubClient mqttClient(espClient);

BLEAdvertising *pAdvertising;

// Для отслеживания времени отправки
long last_send_time = 0;
float previous_acceleration = 0.0;

// Функции для Wi-Fi
void setupWifi() {
  delay(10);
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  // Неблокирующий подход для подключения к Wi-Fi
  unsigned long wifi_connect_start_time = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_connect_start_time < 30000) { // Попытка подключения 30 секунд
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection FAILED!");
  }
}

// Функции для MQTT
void setupMqtt() {
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setKeepAlive(60); // Keep-Alive интервал
}

void reconnectMqtt() {
  while (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) { // Только если Wi-Fi подключен
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
  BLEAdvertisementData advData;
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
  float simulated_accel = acceleration; 
  return simulated_accel;
}

// Функция для отправки одного измерения акселерометра с временной меткой
void sendSingleAccelerometerData(float acceleration, time_t timestamp) {
  String payload = "{";
  payload += "\"accel\":" + String(acceleration, 3) + ",";
  payload += "\"timestamp\":" + String(timestamp); // Отправляем Unix timestamp
  payload += "}";
  
  mqttClient.publish(mqtt_topic, payload.c_str());
  Serial.println("Sent: " + payload);
}

// Функция для отправки накопленных данных
void sendBufferedData() {
  if (bufferIndex == 0) {
    Serial.println("No buffered data to send.");
    return;
  }
  
  Serial.print("Sending ");
  Serial.print(bufferIndex);
  Serial.println(" buffered data points.");

  for (int i = 0; i < bufferIndex; i++) {
    if (mqttClient.connected()) {
      sendSingleAccelerometerData(dataBuffer[i].acceleration, dataBuffer[i].timestamp);
      delay(50); // Небольшая задержка между отправками, чтобы не перегружать MQTT брокер
    } else {
      Serial.println("MQTT disconnected while sending buffered data. Stopping.");
      break; // Прекращаем отправку, если MQTT отвалился
    }
  }
  bufferIndex = 0; // Очищаем буфер после успешной отправки
  Serial.println("Buffered data sent and buffer cleared.");
}

void setup() {
  setupWifi();
  setupMqtt();
  //setupMpu6050();
  setupBleBeacon(); // Инициализация BLE как маяка
  last_send_time = millis(); // Инициализируем время последней отправки

  // Настройка NTP для синхронизации времени
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Waiting for NTP time synchronization...");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2 && WiFi.status() == WL_CONNECTED) { // Ждем, пока время не синхронизируется (не менее 2 часов от эпохи)
    delay(500);
    now = time(nullptr);
  }
  if (WiFi.status() == WL_CONNECTED) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    Serial.print("Current time synchronized: ");
    Serial.println(asctime(&timeinfo));
  } else {
    Serial.println("NTP time synchronization failed (no WiFi). Time will be inaccurate.");
  }
}

void loop() {
  float current_acceleration = readAccelerometer();
  long current_time_ms = millis();
  long send_interval;

  if (previous_acceleration > ACCEL_THRESHOLD) {
    send_interval = FAST_SEND_INTERVAL;
  } else {
    send_interval = SLOW_SEND_INTERVAL;
  }

  // Получаем текущее время Unix timestamp
  time_t current_unix_timestamp = time(nullptr);

  if (current_time_ms - last_send_time >= send_interval) {
    // Проверяем статус Wi-Fi
    if (WiFi.status() == WL_CONNECTED) {
      // Если Wi-Fi есть, пытаемся подключиться к MQTT и отправить данные
      if (!mqttClient.connected()) {
        reconnectMqtt();
      }
      mqttClient.loop(); // Обрабатываем MQTT-события

      if (mqttClient.connected()) {
        // Сначала отправляем накопленные данные, если они есть
        sendBufferedData();
        // Затем отправляем текущее измерение
        sendSingleAccelerometerData(current_acceleration, current_unix_timestamp);
      } else {
        // Если MQTT не подключился, сохраняем текущие данные в буфер
        if (bufferIndex < MAX_BUFFER_SIZE) {
          dataBuffer[bufferIndex] = {current_acceleration, current_unix_timestamp};
          bufferIndex++;
          Serial.println("MQTT not connected, buffering data. Buffer size: " + String(bufferIndex));
        } else {
          Serial.println("Buffer full, dropping oldest data.");
          // Опционально: сдвинуть буфер и добавить новое значение
          // Или просто отбросить, если буфер полон, чтобы не перезаписать старые данные
        }
      }
    } else {
      // Если Wi-Fi нет, сохраняем данные в буфер
      if (bufferIndex < MAX_BUFFER_SIZE) {
        dataBuffer[bufferIndex] = {current_acceleration, current_unix_timestamp};
        bufferIndex++;
        Serial.println("WiFi not connected, buffering data. Buffer size: " + String(bufferIndex));
      } else {
        Serial.println("Buffer full, dropping oldest data.");
        // Опционально: сдвинуть буфер и добавить новое значение
        // Или просто отбросить, если буфер полон
      }
    }
    last_send_time = current_time_ms;
  }
  
  previous_acceleration = current_acceleration; // Обновляем предыдущее значение
  
  delay(100); // Небольшая задержка, чтобы не загружать процессор слишком сильно
}