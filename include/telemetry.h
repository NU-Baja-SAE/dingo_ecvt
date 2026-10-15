/**
 * @file telemetry.h
 * @brief ESP-NOW telemetry packet shared with the receiver.
 *
 * A receiver can include this header and memcpy the received bytes into a
 * TelemetryPacket after checking the length and version.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#define TELEMETRY_PACKET_VERSION 1

/**
 * @brief One telemetry sample. Packed so the layout is identical on both ends.
 */
struct __attribute__((packed)) TelemetryPacket {
    uint8_t version;         // TELEMETRY_PACKET_VERSION
    uint8_t control_mode;    // ControlMode enum value
    uint8_t brake_state;     // 1 if the brake is pressed
    uint32_t time_ms;        // millis() on the ECVT
    float engine_rpm;        // filtered engine RPM
    int32_t motor_setpoint;  // steps
    int32_t motor_position;  // steps
};

#endif // TELEMETRY_H
