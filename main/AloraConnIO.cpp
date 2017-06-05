
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <Arduino.h>
#include <Wire.h>
#include <BME280_I2C.h>
#include <AllAboutEE_MAX11609.h>

using namespace AllAboutEE;

static const char* TAG = "APP";
#define GAS_SENSOR_HEAT_PIN 17
#define GAS_SENSOR_ADC_CHANNEL 0
#define SOUND_SENSOR_ADDRESS 0x2f
#define SOUND_SENSOR_R 51000
#define SOUND_SENSOR_CHANNEL 1

BME280_I2C bme;
MAX11609 adc;

void bmeReadTask(void* parameter);
void gasSensorReadTask(void* parameter);
void soundSensorReadTask(void* parameter);

extern "C" void app_main() {
    initArduino();
    ESP_LOGI(TAG, "Initializing...");

    if (!bme.begin()) {
        ESP_LOGE(TAG, "Failed to initialize BME280");
    } else {
        ESP_LOGI(TAG, "Starting BME280 reading task");
        xTaskCreate(bmeReadTask, "bmeReadTask", 2048, NULL, 1, NULL);
    }

    ESP_LOGI(TAG, "Initializing gas sensor.");
    adc.begin(MAX11609::REF_VDD);
    pinMode(GAS_SENSOR_HEAT_PIN, OUTPUT);
    digitalWrite(GAS_SENSOR_HEAT_PIN, HIGH);
    xTaskCreate(gasSensorReadTask, "gasSensorReadTask", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Initializing sound sensor.");
    xTaskCreate(soundSensorReadTask, "soundSensorReadTask", 2048, NULL, 1, NULL);
}

void bmeReadTask(void* parameter) {
    bme.setTempCal(0);
    const char* LOGTAG = "BME280";

    while (true) {
        bme.readSensor();
        ESP_LOGI(LOGTAG, "T: %f\tP: %f\tH: %f",
            bme.getTemperature_C(),
            bme.getPressure_HP()/1000,
            bme.getHumidity());
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void gasSensorReadTask(void* parameter) {
    const char* LOGTAG = "GAS_SENSOR";

    while (true) {
        ESP_LOGI(LOGTAG, "Read: %d", adc.read(GAS_SENSOR_ADC_CHANNEL));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void soundSensorReadTask(void* parameter) {
    const char* LOGTAG = "SOUND_SENSOR";
    while (true) {
        Wire.beginTransmission(SOUND_SENSOR_ADDRESS);
        Wire.write((uint8_t) 0x00);
        Wire.write((uint8_t) (SOUND_SENSOR_R * 1.0f/(100*1000/256)));
        Wire.endTransmission();

        ESP_LOGI(LOGTAG, "Read: %d", adc.read(SOUND_SENSOR_CHANNEL));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
