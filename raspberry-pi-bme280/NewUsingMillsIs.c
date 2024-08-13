#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "bme280.h"

#define PIR_Pin 0
#define DEBOUNCE_TIME 20000 
#define PIR_CALIBRATION_TIME 40 
#define BME_CALIBRATION_TIME 30

unsigned long lastDetectionTime = 0;
bool pirState = LOW; 

void calibrate_pir() {
    printf("Kalibrasi sensor PIR ");
    for (int i = 0; i < PIR_CALIBRATION_TIME; i++) {
        printf(".");
        fflush(stdout);
        delay(1000);
    }
    printf(" selesai\n");
}

void calibrate_bme(){
    printf("Kalibrasi sensor BME280");
    for (int a = 0; a < BME_CALIBRATION_TIME; a++) {
        printf(".");
        fflush(stdout);
        delay(1000);
    }
    printf("Selesai, perangkat siap digunakan\n");
}

int main() {
    if (wiringPiSetup() == -1) {
        printf("Wiring Pi Setup Error\n");
        return -1;
    }

    pinMode(PIR_Pin, INPUT);

    calibrate_pir();
    calibrate_bme();

    int fd = wiringPiI2CSetup(BME280_ADDRESS);
    if (fd < 0) {
        printf("Device not found\n");
        return -1;
    }

    bme280_calib_data cal;
    readCalibrationData(fd, &cal);

    wiringPiI2CWriteReg8(fd, 0xf2, 0x01);   
    wiringPiI2CWriteReg8(fd, 0xf4, 0x25);  

    while (1) {
        bme280_raw_data raw;
        getRawData(fd, &raw);

        int32_t t_fine = getTemperatureCalibration(&cal, raw.temperature);
        float t = compensateTemperature(t_fine);
        float p = compensatePressure(raw.pressure, &cal, t_fine) / 100;
        float h = compensateHumidity(raw.humidity, &cal, t_fine);
        float a = getAltitude(p);

        unsigned long currentMillis = millis();
        int reading = digitalRead(PIR_Pin);

        if (reading == HIGH && pirState == LOW && (currentMillis - lastDetectionTime >= DEBOUNCE_TIME)) {
            pirState = HIGH;
            lastDetectionTime = currentMillis;
            printf("Gerakan Terdeteksi\n");
            printf("Temperature: %.2f C\nPressure: %.2f hPa\nHumidity: %.2f%%\nAltitude: %.2f m\n", t, p, h, a, (int)time(NULL));
        } else if (reading == LOW && pirState == HIGH && (currentMillis - lastDetectionTime >= DEBOUNCE_TIME)) {
            pirState = LOW;
            lastDetectionTime = currentMillis;
            printf("Gerakan Berhenti\n");
        }

        if (currentMillis - lastDetectionTime >= DEBOUNCE_TIME && pirState == LOW) {
            printf("Tidak ada gerakan, pencetakan data sensor...\n");
            printf("Temperature: %.2f C\nPressure: %.2f hPa\nHumidity: %.2f%%\nAltitude: %.2f m\n", t, p, h, a, (int)time(NULL));
            lastDetectionTime = currentMillis;
        }

        delay(100); 
    }

    return 0;
}


// Sisanya sama seperti kode bme280.c
