#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "telemetry.h"

/**
 * @file main.cpp
 * @brief ESP-NOW receiver for ECVT telemetry.
 *
 * Receives TelemetryPacket frames from the ECVT and prints each one over USB
 * serial as a CSV row. The ECVT must have this board's MAC as ESPNOW_PEER_MAC.
 */

#define ESPNOW_CHANNEL 1 // must match ESPNOW_CHANNEL in include/config.h
#define PACKET_QUEUE_LENGTH 32
#define LINK_TIMEOUT_MS 1000 // warn when no packet arrives for this long
#define STATUS_LED_PIN 2     // onboard blue LED on the DOIT devkit

#define LOG_CSV_HEADER "time_ms,engine_rpm,motor_setpoint,motor_position,brake_state,control_mode"

// Same order as the ControlMode enum in include/controller.h.
static const char *const CONTROL_MODE_NAMES[] = {
    "TORQUE", "POWER", "MANUAL", "BRAKE", "DEBUG", "HOMING", "BRAKE_CHECK", "ACCELERATION"};

QueueHandle_t packetQueue;
volatile uint32_t droppedPackets = 0;

/**
 * @brief ESP-NOW receive callback (runs in the WiFi task, so only queue the packet).
 */
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(TelemetryPacket) || data[0] != TELEMETRY_PACKET_VERSION) {
        droppedPackets++;
        return;
    }
    TelemetryPacket packet;
    memcpy(&packet, data, sizeof(packet));
    if (xQueueSend(packetQueue, &packet, 0) != pdTRUE) {
        droppedPackets++;
    }
}

/**
 * @brief Arduino setup entry point.
 */
void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED_PIN, OUTPUT);

    packetQueue = xQueueCreate(PACKET_QUEUE_LENGTH, sizeof(TelemetryPacket));

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW init failed");
        while (true) {
            delay(1000);
        }
    }
    esp_now_register_recv_cb(onReceive);

    Serial.printf("ESP-NOW receiver ready, MAC %s, channel %d\n", WiFi.macAddress().c_str(), ESPNOW_CHANNEL);
    Serial.println(LOG_CSV_HEADER);
}

/**
 * @brief Print queued packets and report link loss.
 */
void loop() {
    static uint32_t lastPacketTime = 0;
    static bool linkLost = true;
    static uint32_t reportedDrops = 0;

    TelemetryPacket packet;
    if (xQueueReceive(packetQueue, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
        const char *mode = packet.control_mode < sizeof(CONTROL_MODE_NAMES) / sizeof(CONTROL_MODE_NAMES[0])
                               ? CONTROL_MODE_NAMES[packet.control_mode]
                               : "UNKNOWN";
        Serial.printf("%lu,%.1f,%ld,%ld,%u,%s\n",
                      (unsigned long)packet.time_ms,
                      packet.engine_rpm,
                      (long)packet.motor_setpoint,
                      (long)packet.motor_position,
                      packet.brake_state,
                      mode);
        lastPacketTime = millis();
        linkLost = false;
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    } else if (!linkLost && millis() - lastPacketTime > LINK_TIMEOUT_MS) {
        Serial.println("# link lost: no packets from ECVT");
        linkLost = true;
        digitalWrite(STATUS_LED_PIN, LOW);
    }

    if (droppedPackets != reportedDrops) {
        reportedDrops = droppedPackets;
        Serial.printf("# dropped packets: %lu\n", (unsigned long)reportedDrops);
    }
}
