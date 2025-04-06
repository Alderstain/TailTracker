#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// ========== НАСТРОЙКИ ========== //
const char* ssid = "jopa";
const char* password = "jopa";
const char* mqtt_server = "192.168.1.75";
const char* mqtt_topic = "sensors/esp32_data";

const char* target_ble_name = "ESP32_Beacon"; // Имя вашего BLE маяка
const int mpu_address = 0x68; // Адрес MPU-6050
const float accel_sensitivity = 16384.0; // ±2g
// =============================== //

WiFiClient espClient;
PubSubClient mqttClient(espClient);
BLEScan* bleScanner;
int ble_rssi = -100;
bool ble_beacon_found = false;

class BleAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if(advertisedDevice.getName() == target_ble_name) {
        ble_rssi = advertisedDevice.getRSSI();
        ble_beacon_found = true;
      }
    }
};

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

void setupBle() {
  BLEDevice::init("");
  bleScanner = BLEDevice::getScan();
  bleScanner->setAdvertisedDeviceCallbacks(new BleAdvertisedDeviceCallbacks());
  bleScanner->setActiveScan(true);
  bleScanner->setInterval(100);
  bleScanner->setWindow(99);
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

void scanBle() {
  ble_beacon_found = false;
  BLEScanResults scanResults = bleScanner->start(2, false); // Scan for 2 seconds
  
  if (!ble_beacon_found) {
    ble_rssi = -100; // Default value if beacon not found
  }
  
  bleScanner->clearResults();
}

/*void sendData(float acceleration, int rssi) {
  String payload = "{\"accel\":" + String(acceleration, 3) + 
  ",\"rssi\":" + String(rssi) + 
  ",\"timestamp\":" + String(millis()/1000) + "}";
  
  mqttClient.publish(mqtt_topic, payload.c_str());
  Serial.println("Sent: " + payload);
}*/

void sendData(float acceleration, int rssi) {
  String accelPayload = "{\"accel\":" + String(acceleration, 3) + "}";
  mqttClient.publish("sensors/accel", accelPayload.c_str(), true); // QoS=1
  Serial.print("Отправлено: ");
  Serial.println(accelPayload);
  

  String rssiPayload = "{\"rssi\":" + String(rssi) + "}";
  mqttClient.publish("sensors/rssi", rssiPayload.c_str(), true);

}

void setup() {
  setupWifi();
  setupMqtt();
  reconnectMqtt();
  setupMpu6050();
  setupBle();
}

void loop() {
  /*if (!mqttClient.connected()) {
    reconnectMqtt();
  }*/
  mqttClient.loop();
  
  // Read sensors
  scanBle();
  float acceleration = readAccelerometer();
  
  // Send data
  sendData(acceleration, ble_rssi);
  
  
  delay(1000); // Send data every 1 second
}