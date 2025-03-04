#include <Arduino.h>
#include <Wire.h>

// Адрес MPU-6050 по умолчанию
const int MPU_ADDR = 0x68;

// Чувствительность акселерометра и гироскопа (по умолчанию)
const float ACCEL_SENSITIVITY = 16384.0;  // Для диапазона ±2g
const float GYRO_SENSITIVITY = 131.0;     // Для диапазона ±250 °/s

void setup() {
  Serial.begin(115200);

  // Инициализация I2C
  Wire.begin();

  // Выход из спящего режима MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Регистр PWR_MGMT_1
  Wire.write(0);     // Установить в 0 для выхода из спящего режима
  Wire.endTransmission(true);
}

// Функция для чтения данных из регистров
int16_t readSensor(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);  // Указываем регистр для чтения
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);  // Запрашиваем 2 байта данных
  
    // Чтение данных
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
  
    // Объединение байтов в 16-битное значение
    return (highByte << 8) | lowByte;
  }

void loop() {
  // Чтение данных акселерометра
  int16_t ax_raw = readSensor(0x3B);
  int16_t ay_raw = readSensor(0x3D);
  int16_t az_raw = readSensor(0x3F);

  // Чтение данных гироскопа
  int16_t gx_raw = readSensor(0x43);
  int16_t gy_raw = readSensor(0x45);
  int16_t gz_raw = readSensor(0x47);

  // Масштабирование данных акселерометра (в g)
  float ax = ax_raw / ACCEL_SENSITIVITY;
  float ay = ay_raw / ACCEL_SENSITIVITY;
  float az = az_raw / ACCEL_SENSITIVITY;

  // Масштабирование данных гироскопа (в °/s)
  float gx = gx_raw / GYRO_SENSITIVITY;
  float gy = gy_raw / GYRO_SENSITIVITY;
  float gz = gz_raw / GYRO_SENSITIVITY;

  // Вывод данных в Serial Monitor
  Serial.print("Accelerometer (g): ");
  Serial.print("X = "); Serial.print(ax, 3);
  Serial.print(" Y = "); Serial.print(ay, 3);
  Serial.print(" Z = "); Serial.println(az, 3);

  Serial.print("Gyroscope (°/s): ");
  Serial.print("X = "); Serial.print(gx, 3);
  Serial.print(" Y = "); Serial.print(gy, 3);
  Serial.print(" Z = "); Serial.println(gz, 3);

  // Задержка для удобства чтения данных
  delay(1000);
}
