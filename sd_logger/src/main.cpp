#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"

/**
 * @file main.cpp
 * @brief ESP32-CAM serial-to-SD logger for ECVT telemetry.
 *
 * Wiring: ECVT GPIO32 (TX) -> ESP32-CAM GPIO13 (RX), and a common GND.
 * The SD card runs in 1-bit mode so GPIO13 is free and the flash LED
 * (GPIO4) does not flicker during writes.
 */

#define LOG_RX_PIN 13        // UART RX from the ECVT controller
#define LOG_BAUD 115200      // must match SD_LOG_BAUD on the ECVT
#define LOG_RX_BUFFER_SIZE 4096
#define LOG_CSV_HEADER "time_ms,engine_rpm,motor_setpoint,motor_position,brake_state,control_mode"
#define FLUSH_INTERVAL_MS 1000 // at most this much data is lost on power cut

#define STATUS_LED_PIN 33    // onboard red LED, active low
#define FLASH_LED_PIN 4

File logFile;
uint32_t lastFlushTime = 0;

/**
 * @brief Mount the SD card and open a new, uniquely numbered log file.
 * @return true if a log file is open and ready for writing.
 */
bool openLogFile() {
    SD_MMC.end();
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("ERROR: SD card mount failed");
        return false;
    }

    char path[24];
    for (int i = 0; i < 10000; i++) {
        snprintf(path, sizeof(path), "/log_%04d.csv", i);
        if (!SD_MMC.exists(path)) {
            break;
        }
    }

    logFile = SD_MMC.open(path, FILE_WRITE);
    if (!logFile) {
        Serial.printf("ERROR: could not open %s\n", path);
        return false;
    }

    logFile.println(LOG_CSV_HEADER);
    Serial.printf("Logging to %s\n", path);
    return true;
}

/**
 * @brief Arduino setup entry point.
 */
void setup() {
    Serial.begin(115200);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);

    Serial2.setRxBufferSize(LOG_RX_BUFFER_SIZE);
    Serial2.begin(LOG_BAUD, SERIAL_8N1, LOG_RX_PIN, -1);

    while (!openLogFile()) {
        // Fast blink while there is no usable SD card.
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        delay(200);
    }

    // Keep the flash LED off (GPIO4 is unused by the SD card in 1-bit mode).
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW);
}

/**
 * @brief Copy incoming serial bytes to the SD card and flush periodically.
 */
void loop() {
    static uint8_t buffer[512];

    size_t available = Serial2.available();
    if (available > 0) {
        size_t len = Serial2.readBytes(buffer, min(available, sizeof(buffer)));
        if (logFile.write(buffer, len) != len) {
            Serial.println("ERROR: SD write failed, remounting");
            logFile.close();
            while (!openLogFile()) {
                delay(200);
            }
        }
        // Blink the status LED while data is arriving.
        digitalWrite(STATUS_LED_PIN, (millis() / 100) % 2);
    }

    if (millis() - lastFlushTime >= FLUSH_INTERVAL_MS) {
        logFile.flush();
        lastFlushTime = millis();
    }
}
