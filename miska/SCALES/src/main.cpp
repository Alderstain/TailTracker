#include <Arduino.h>
#include "HX711.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// ========== НАСТРОЙКИ ========== //
const char* ssid = "MGTS_GPON_2357"; // Имя вашей Wi-Fi сети
const char* password = "HRBPPWRD"; // Пароль от вашей Wi-Fi сети
const char* mqtt_server = "192.168.1.75"; // IP адрес вашего MQTT брокера
const char* mqtt_topic_water_intake = "sensors/water_intake"; // Топик для выпитой воды
const char* mqtt_topic_current_weight = "sensors/current_weight"; // Новый топик для текущего веса

const char* target_ble_name = "ESP32_Beacon"; // Имя вашего BLE маяка (ошейника)
const int RSSI_THRESHOLD = -70; // Пороговое значение RSSI. Подбирается экспериментально.

// Пины для HX711
const int LOADCELL_DOUT_PIN = 13; // DT pin
const int LOADCELL_SCK_PIN = 12;   // SCK pin

// Коэффициент калибровки (настраивается индивидуально)
float calibration_factor = 7.46400; // Примерное значение, требует калибровки

// =============================== //

WiFiClient espClient;
PubSubClient mqttClient(espClient);
HX711 scale;

BLEScan* bleScanner;
int ble_rssi = -100;
bool ble_beacon_found = false;

float weight_before_approach = 0.0;
bool collar_was_near = false;

// Для отслеживания времени для регулярной отправки веса
long last_current_weight_send_time = 0;
const long CURRENT_WEIGHT_SEND_INTERVAL = 5 * 60 * 1000; // 5 минут в миллисекундах

// Класс для обработки результатов сканирования BLE
class BleAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if(advertisedDevice.getName() == target_ble_name) {
        ble_rssi = advertisedDevice.getRSSI();
        ble_beacon_found = true;
      }
    }
};

// Функции для Wi-Fi
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

// Функции для MQTT
void setupMqtt() {
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setKeepAlive(30);
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32_Water_Scales_Node")) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5s");
      delay(5000);
    }
  }
}

// Функции для BLE
void setupBle() {
  BLEDevice::init("");
  bleScanner = BLEDevice::getScan();
  bleScanner->setAdvertisedDeviceCallbacks(new BleAdvertisedDeviceCallbacks());
  bleScanner->setActiveScan(true);
  bleScanner->setInterval(100);
  bleScanner->setWindow(99);
}

void scanBle() {
  ble_beacon_found = false;
  BLEScanResults scanResults = bleScanner->start(2, false); // Scan for 2 seconds
  
  if (!ble_beacon_found) {
    ble_rssi = -100; // Default value if beacon not found
  }
  
  bleScanner->clearResults();
}

// Функция для отправки данных о выпитой воде по MQTT (RSSI убрано)
void sendWaterIntakeData(float water_amount) { // Измененные аргументы
  String payload = "{";
  payload += "\"water_amount\":" + String(water_amount, 1);
  payload += "}";
  
  mqttClient.publish(mqtt_topic_water_intake, payload.c_str());
  Serial.println("Sent Water Intake: " + payload);
}

// Новая функция для отправки текущего веса по MQTT
void sendCurrentWeightData(float current_weight) {
  String payload = "{";
  payload += "\"current_weight\":" + String(current_weight, 1);
  payload += "}";

  mqttClient.publish(mqtt_topic_current_weight, payload.c_str());
  Serial.println("Sent Current Weight: " + payload);
}


void setup() {
  Serial.begin(115200);
  setupWifi();
  setupMqtt();
  setupBle(); // Инициализация BLE сканера
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(); // Сброс показаний на ноль при старте
  
  Serial.println("Scales setup complete.");
  last_current_weight_send_time = millis(); // Инициализируем время для регулярной отправки
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();
  
  scanBle(); // Сканируем BLE маяк
  float current_weight = 0.0;
  if (scale.is_ready()) {
    current_weight = scale.get_units(5); // Получаем усредненное значение (5 измерений)
    Serial.print("Текущий вес: ");
    Serial.print(current_weight, 1);
    Serial.print(" г, RSSI ошейника: ");
    Serial.println(ble_rssi);
  } else {
    Serial.println("HX711 не готов. Проверьте подключение.");
    delay(500); // Даем время для инициализации
    return;
  }

  // Логика определения и измерения выпитой воды
  if (ble_rssi > RSSI_THRESHOLD && !collar_was_near) { // Ошейник приблизился (RSSI выше порога)
    weight_before_approach = current_weight;
    collar_was_near = true;
    Serial.println("Ошейник рядом. Запомнен вес: " + String(weight_before_approach, 1) + " г");
  } else if (ble_rssi <= RSSI_THRESHOLD && collar_was_near) { // Ошейник отошел (RSSI ниже порога)
    float water_drunk = weight_before_approach - current_weight;
    if (water_drunk > 0.0) { // Проверяем, что вода действительно была выпита
      Serial.println("Ошейник отошел. Выпито воды: " + String(water_drunk, 1) + " г");
      sendWaterIntakeData(water_drunk); // RSSI больше не передается
    } else {
      Serial.println("Ошейник отошел, но изменений веса не обнаружено или вес увеличился.");
    }
    collar_was_near = false;
  }
  
  // Логика для регулярной отправки текущего веса
  long current_time = millis();
  if (current_time - last_current_weight_send_time >= CURRENT_WEIGHT_SEND_INTERVAL) {
    sendCurrentWeightData(current_weight);
    last_current_weight_send_time = current_time;
  }

  delay(2000); // Задержка между измерениями и проверками
}