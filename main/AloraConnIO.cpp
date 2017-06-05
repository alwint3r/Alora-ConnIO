
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <Arduino.h>
#include <Wire.h>
#include <BME280_I2C.h>
#include <AllAboutEE_MAX11609.h>
#include <Adafruit_SHT31.h>
#include <SFE_LSM9DS0.h>
#include <SparkFunTSL2561.h>

using namespace AllAboutEE;

static const char* TAG = "APP";
unsigned int lightSensorReadDelay = 0;

#define GAS_SENSOR_HEAT_PIN 17
#define GAS_SENSOR_ADC_CHANNEL 0
#define SOUND_SENSOR_ADDRESS 0x2f
#define SOUND_SENSOR_R 51000
#define SOUND_SENSOR_CHANNEL 1
#define LSM9DS0_XM_ADDRESS 0x1d
#define LSM9DS0_G 0x6b

BME280_I2C bme;
MAX11609 adc;
Adafruit_SHT31 sht31;
LSM9DS0 imuSensor(MODE_I2C, LSM9DS0_G, LSM9DS0_XM_ADDRESS);
SFE_TSL2561 lightSensor;

void readSensorTask(void* parameter);
void printGyro();
void printAccel();
void printMag();
void writeToMicSensor(uint8_t reg, uint8_t data);
void printLightSensor();

extern "C" void app_main() {
    initArduino();
    ESP_LOGI(TAG, "Initializing...");

    if (!bme.begin()) {
        ESP_LOGE(TAG, "Failed to initialize BME280");
    }

    if (!sht31.begin(0x44)) {
        ESP_LOGE(TAG, "Failed to initialize SHT31");
    }

    imuSensor.begin();
    lightSensor.begin(TSL2561_ADDR_0);

    unsigned char TSLID;
    if (lightSensor.getID(TSLID)) {
        ESP_LOGI(TAG, "Got TSL2561 factory ID: 0x%x", TSLID);

        unsigned char time = 2;
        lightSensor.setTiming(0, time, lightSensorReadDelay);

        ESP_LOGI(TAG, "Powering TSL2561");
        lightSensor.setPowerUp();
    } else {
        ESP_LOGE(TAG, "Failed to initialize TSL2561");
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

        // LSM9DS0
        printGyro();
        printAccel();
        printMag();

        // TSL2561
        printLightSensor();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void writeToMicSensor(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(SOUND_SENSOR_ADDRESS);
    Wire.write(reg);
    Wire.write(data);//(SOUND_SENSOR_R * 1.0f/(100*1000/256)));
    Wire.endTransmission();
}

void printGyro() {
    imuSensor.readGyro();
    ESP_LOGI("LSM9DS0_GYRO", "GX: %f\tGY: %f\tGZ: %f",
        imuSensor.calcGyro(imuSensor.gx),
        imuSensor.calcGyro(imuSensor.gy),
        imuSensor.calcGyro(imuSensor.gz));
}

void printAccel() {
    imuSensor.readAccel();
    ESP_LOGI("LSM9DS0_ACCEL", "AX: %f\tAY: %f\tAZ: %f",
        imuSensor.calcAccel(imuSensor.ax),
        imuSensor.calcAccel(imuSensor.ay),
        imuSensor.calcAccel(imuSensor.az));
}

void printMag() {
    imuSensor.readMag();

    float heading;

    if (imuSensor.my > 0) {
        heading = 90 - (atan(imuSensor.mx / imuSensor.my) * (180 / PI));
    } else if (imuSensor.my < 0) {
        heading = -(atan(imuSensor.mx / imuSensor.my) * (180 / PI));
    } else {
        if (imuSensor.mx < 0) {
            heading = 180;
        } else {
            heading = 0;
        }
    }

    ESP_LOGI("LSM9DS0_MAG", "MX: %f\tMY: %f\tMZ: %f\tH: %f",
        imuSensor.calcMag(imuSensor.mx),
        imuSensor.calcMag(imuSensor.my),
        imuSensor.calcMag(imuSensor.mz),
        heading);
}

void printLightSensor() {
    lightSensor.manualStart();
    delay(lightSensorReadDelay);
    lightSensor.manualStop();

    unsigned int data0, data1;
    double lux;

    if (lightSensor.getData(data0, data1)) {
        bool good;
        bool gain = false;

        good = lightSensor.getLux(gain, lightSensorReadDelay, data0, data1, lux);
    } else {
        lux = 0;
    }

    ESP_LOGI("LIGHT", "Lux: %f", lux);
}
