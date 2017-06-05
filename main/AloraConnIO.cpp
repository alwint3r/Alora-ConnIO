
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <Arduino.h>
#include <Wire.h>
#include <BME280_I2C.h>
#include <AllAboutEE_MAX11609.h>
#include <Adafruit_SHT31.h>

using namespace AllAboutEE;

static const char* TAG = "APP";
#define GAS_SENSOR_HEAT_PIN 17
#define GAS_SENSOR_ADC_CHANNEL 0
#define SOUND_SENSOR_ADDRESS 0x2f
#define SOUND_SENSOR_R 51000
#define SOUND_SENSOR_CHANNEL 1

BME280_I2C bme;
MAX11609 adc;
Adafruit_SHT31 sht31;

void writeToMicSensor(uint8_t reg, uint8_t data);
void readSensorTask(void* parameter);
void bmeReadTask(void* parameter);
void gasSensorReadTask(void* parameter);
void soundSensorReadTask(void* parameter);

extern "C" void app_main() {
    initArduino();
    ESP_LOGI(TAG, "Initializing...");

    if (!bme.begin()) {
        ESP_LOGE(TAG, "Failed to initialize BME280");
    }

    if (!sht31.begin(0x44)) {
        ESP_LOGE(TAG, "Failed to initialize SHT31");
    }

    ESP_LOGI(TAG, "Initializing gas sensor.");
    adc.begin(MAX11609::REF_VDD);
    pinMode(GAS_SENSOR_HEAT_PIN, OUTPUT);
    digitalWrite(GAS_SENSOR_HEAT_PIN, HIGH);

    xTaskCreate(readSensorTask, "readSensorTask", 2048, NULL, 1, NULL);
}

void readSensorTask(void* parameter) {

    while (true) {
        // BME280
        bme.readSensor();
        ESP_LOGI("BME280", "T: %f\tP: %f\tH: %f",
            bme.getTemperature_C(),
            bme.getPressure_HP()/1000,
            bme.getHumidity());
        
        // SHT31
        ESP_LOGI("SHT31", "T: %f\tH: %f", sht31.readTemperature(), sht31.readHumidity());
        
        // Gas Sensor
        ESP_LOGI("GAS_SENSOR", "%d", adc.read(GAS_SENSOR_ADC_CHANNEL));

        // Sound Sensor
        writeToMicSensor(0x00, (SOUND_SENSOR_R * 1.0f/(100*1000/256)));
        ESP_LOGI("SOUND_SENSOR", "%d", adc.read(SOUND_SENSOR_CHANNEL));

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void writeToMicSensor(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(SOUND_SENSOR_ADDRESS);
    Wire.write(reg);
    Wire.write(data);//(SOUND_SENSOR_R * 1.0f/(100*1000/256)));
    Wire.endTransmission();
}
