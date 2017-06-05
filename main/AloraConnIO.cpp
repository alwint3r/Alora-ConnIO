
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <Arduino.h>
#include <BME280_I2C.h>

static const char* TAG = "APP";

BME280_I2C bme;

void bmeReadTask(void* parameter);

extern "C" void app_main() {
    initArduino();
    ESP_LOGI(TAG, "Initializing...");

    if (!bme.begin()) {
        ESP_LOGE(TAG, "Failed to initialize BME280");
        while (1);
    } else {
        xTaskCreate(bmeReadTask, "bmeReadTask", 2048, NULL, 1, NULL);
    }
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