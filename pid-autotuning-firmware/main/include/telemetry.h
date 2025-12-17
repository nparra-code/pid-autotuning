/**
 * @file telemetry.h
 * @brief Telemetry module interface for data collection and reporting.
 * @version 1.0
 * @date 4-12-2025
 */

// telemetry.h
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#include "telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>

// Configuration
#define TELEMETRY_BUFFER_SIZE 512
#define TELEMETRY_SAMPLE_WINDOW 100  // Number of samples per batch
#define TELEMETRY_SERVER_PORT 8888
#define IDENT_SERVER_PORT 8889
#define TELEMETRY_TIMEOUT_MS 5000

// Protocol message types
typedef enum {
    MSG_TYPE_DATA_BATCH = 0x01,
    MSG_TYPE_PID_RESPONSE = 0x02,
    MSG_TYPE_ACK = 0x03,
    MSG_TYPE_ERROR = 0x04,
    MSG_TYPE_IDENT_DATA = 0x05
} telemetry_msg_type_t;

// Robot sensor data structure (customize based on your sensors)
typedef struct {
    uint32_t timestamp_ms;      // Timestamp in milliseconds
    float motor_state[3];       // Motor states (3 wheels)
    float motor_setpoint[3];    // Motor setpoints
    float errors[3][3];        // Error history for each motor
} __attribute__((packed)) robot_sample_t;

typedef struct {
    uint32_t timestamp_ms;      ///< Timestamp in milliseconds
    float pwm_input;         ///< PWM command (-100 to 100) for each motor
    float velocity_response; ///< Measured velocity in rad/s for each motor
} __attribute__((packed)) motor_ident_sample_t;

// Data batch structure
typedef struct {
    uint16_t sample_count;
    uint16_t sequence_number;
    robot_sample_t samples[TELEMETRY_SAMPLE_WINDOW];
} __attribute__((packed)) data_batch_t;

// Identification batch
typedef struct {
    uint16_t sample_count;
    uint16_t sequence_number;
    motor_ident_sample_t samples[TELEMETRY_SAMPLE_WINDOW];
} __attribute__((packed)) ident_data_batch_t;

// PID constants response structure
typedef struct {
    float kp[3];  // Proportional gains for 3 motors
    float ki[3];  // Integral gains
    float kd[3];  // Derivative gains
    uint8_t iteration;  // Autotuning iteration number
    uint8_t converged;  // 1 if tuning is complete
} __attribute__((packed)) pid_response_t;

// Message header
typedef struct {
    uint8_t type;          // Message type
    uint8_t version;       // Protocol version
    uint16_t length;       // Payload length
    uint32_t checksum;     // CRC32 checksum
} __attribute__((packed)) msg_header_t;

// Telemetry status
typedef enum {
    TELEMETRY_IDLE = 0,
    TELEMETRY_CONNECTED,
    TELEMETRY_SENDING,
    TELEMETRY_WAITING_RESPONSE,
    TELEMETRY_ERROR
} telemetry_status_t;

// Public API
esp_err_t telemetry_init(const char *server_ip);
esp_err_t telemetry_connect(void);
esp_err_t telemetry_ident_connect(void);
esp_err_t telemetry_disconnect(void);
esp_err_t telemetry_add_sample(const robot_sample_t *sample);
esp_err_t telemetry_ident_add_sample(const motor_ident_sample_t *sample);
esp_err_t telemetry_send_batch(void);
esp_err_t telemetry_ident_send_batch(void);
esp_err_t telemetry_receive_pid(pid_response_t *pid_data, uint32_t timeout_ms);
telemetry_status_t telemetry_get_status(void);
void telemetry_get_stats(uint32_t *samples_sent, uint32_t *responses_recv, uint32_t *errors);

#endif // TELEMETRY_H