/**
 * @file firmware_main.c
 * 
 * @brief Main file for PID autotuning firmware
 * 
 * This file contains the main application logic for the PID autotuning firmware.
 * 
 * @authors Nelson Fernando Parra Guardia
 * 
 * @date 3-12-2025
 * 
 * @version 1.0
 * 
 * @copyright Copyright (c) RoboCup SISTEMIC 2025
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "control_main.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "telemetry.h"
#include "esp_netif.h"

static const char *TAG = "MAIN";

// WiFi event group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#define ESP_MAXIMUM_RETRY  20

// Your WiFi credentials
#define WIFI_SSID "TP-Link_CCAF"
#define WIFI_PASS "26944777"
#define SERVER_IP "192.168.0.233"  // Your station IP

// extern encoder_data_t right_encoder_data;
// extern encoder_data_t left_encoder_data;
// extern encoder_data_t back_encoder_data;

// extern pid_parameter_t pid_paramR;
// extern pid_parameter_t pid_paramL;
// extern pid_parameter_t pid_paramB;

static AS5600_t gAs5600R, gAs5600L, gAs5600B;  ///< AS5600 object for angle sensor right, left and back

extern encoder_data_t right_encoder_data, left_encoder_data, back_encoder_data; ///< Encoder data for right, left and back wheels

static bldc_pwm_motor_t pwmR, pwmL, pwmB;   ///< BLDC motor object right, left and back
static pid_block_handle_t pidR, pidL, pidB; ///< PID control block handle

extern pid_parameter_t pid_paramR, pid_paramL, pid_paramB; ///< PID parameters for right, left and back wheels

// Current PID constants (will be updated by autotuner)
static pid_response_t current_pid = {
    .kp = {1.0f, 1.0f, 1.0f},
    .ki = {0.1f, 0.1f, 0.1f},
    .kd = {0.01f, 0.01f, 0.01f},
    .iteration = 0,
    .converged = 0
};

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi station started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (attempt %d/%d)", s_retry_num, ESP_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to WiFi after %d attempts", ESP_MAXIMUM_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// WiFi initialization with event handlers
static void wifi_init(void) {
    // Create event group for WiFi events
    s_wifi_event_group = xEventGroupCreate();
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Create default WiFi station
    esp_netif_create_default_wifi_sta();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Register event handlers for WiFi and IP events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL));
    
    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = esp_wifi_start();
    vTaskDelay(pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Wi-Fi: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Wi-Fi started successfully, waiting for connection...");
    
    // Wait for WiFi connection with 30 second timeout
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(30000));
    
    // Check which event occurred
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✓ Connected to WiFi SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "✗ Failed to connect to WiFi after %d attempts", ESP_MAXIMUM_RETRY);
    } else {
        ESP_LOGE(TAG, "✗ WiFi connection timeout (30 seconds)");
    }
}

// Read actual sensor data from robot encoders
static void read_robot_sensors(robot_sample_t *sample) {
    // Read timestamp
    sample->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Motor current (placeholder - add actual current sensing if available)
    sample->motor_state[0] = right_encoder_data.state;
    sample->motor_state[1] = left_encoder_data.state;
    sample->motor_state[2] = back_encoder_data.state;

    // Motor setpoints from PID parameters
    sample->motor_setpoint[0] = pidR->set_point;
    sample->motor_setpoint[1] = pidL->set_point;
    sample->motor_setpoint[2] = pidB->set_point;

    // Error history (difference between setpoint and actual state)
    // Shift previous errors (k-2)
    sample->errors[0][2] = sample->errors[0][1];
    sample->errors[1][2] = sample->errors[1][1]; 
    sample->errors[2][2] = sample->errors[2][1];
    
    // Shift last errors (k-1)
    sample->errors[0][1] = sample->errors[0][0];
    sample->errors[1][1] = sample->errors[1][0];
    sample->errors[2][1] = sample->errors[2][0];

    sample->errors[0][0] = pidR->set_point - right_encoder_data.state; // Current motor 1 error (k)
    sample->errors[1][0] = pidL->set_point - left_encoder_data.state;  // Current motor 2 error (k)
    sample->errors[2][0] = pidB->set_point - back_encoder_data.state;  // Current motor 3 error (k)
}

// Apply PID constants to motor controllers
static void apply_pid_constants(const pid_response_t *pid_response) {
    ESP_LOGI(TAG, "Applying PID constants:");
    
    // Update right wheel PID
    pid_paramR.kp = pid_response->kp[0];
    pid_paramR.ki = pid_response->ki[0];
    pid_paramR.kd = pid_response->kd[0];
    ESP_LOGI(TAG, "  Right Motor: Kp=%.3f, Ki=%.3f, Kd=%.3f",
             pid_paramR.kp, pid_paramR.ki, pid_paramR.kd);
    
    // Update left wheel PID
    pid_paramL.kp = pid_response->kp[1];
    pid_paramL.ki = pid_response->ki[1];
    pid_paramL.kd = pid_response->kd[1];
    ESP_LOGI(TAG, "  Left Motor: Kp=%.3f, Ki=%.3f, Kd=%.3f",
             pid_paramL.kp, pid_paramL.ki, pid_paramL.kd);
    
    // Update back wheel PID
    pid_paramB.kp = pid_response->kp[2];
    pid_paramB.ki = pid_response->ki[2];
    pid_paramB.kd = pid_response->kd[2];
    ESP_LOGI(TAG, "  Back Motor: Kp=%.3f, Ki=%.3f, Kd=%.3f",
             pid_paramB.kp, pid_paramB.ki, pid_paramB.kd);
    
    if (pid_response->converged) {
        ESP_LOGI(TAG, "PID autotuning CONVERGED at iteration %d!", pid_response->iteration);
    } else {
        ESP_LOGI(TAG, "PID autotuning iteration %d", pid_response->iteration);
    }
}

// Main autotuning task
static void autotuning_task(void *pvParameters) {
    robot_sample_t sample;
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting autotuning task");
    
    // Wait for WiFi connection event
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    
    ESP_LOGI(TAG, "WiFi connected, initializing telemetry...");
    
    // Initialize telemetry
    ret = telemetry_init(SERVER_IP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize telemetry");
        vTaskDelete(NULL);
        return;
    }
    
    // Give some time for network stack to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Connect to server
    ret = telemetry_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to server");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting autotuning loop");
    
    while (!current_pid.converged) {
        // Phase 1: Collect data
        ESP_LOGI(TAG, "Collecting %d samples...", TELEMETRY_SAMPLE_WINDOW);
        
        for (int i = 0; i < TELEMETRY_SAMPLE_WINDOW; i++) {
            read_robot_sensors(&sample);
            
            ret = telemetry_add_sample(&sample);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add sample");
                break;
            }
            
            // Sample at 100Hz (10ms interval)
            vTaskDelay(pdMS_TO_TICKS(7));
        }
        
        // Phase 2: Send batch to server
        ESP_LOGI(TAG, "Sending data batch to server...");
        ret = telemetry_send_batch();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send batch");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Phase 3: Wait for PID response
        ESP_LOGI(TAG, "Waiting for PID response...");
        ret = telemetry_receive_pid(&current_pid, 10000);  // 10 second timeout
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive PID response");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Phase 4: Apply new PID constants
        apply_pid_constants(&current_pid);
        
        // Print statistics
        uint32_t samples_sent, responses_recv, errors;
        telemetry_get_stats(&samples_sent, &responses_recv, &errors);
        ESP_LOGI(TAG, "Stats: samples=%lu, responses=%lu, errors=%lu",
                 samples_sent, responses_recv, errors);
        
        // Small delay before next iteration
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    ESP_LOGI(TAG, "Autotuning complete!");
    
    // Disconnect
    telemetry_disconnect();
    
    vTaskDelete(NULL);
}

void app_main(void)
{

    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init();
    
    ///<------- Initialize the BLDC motors PWMs ----------
    bldc_init(&pwmR, PWM_GPIO_R, PWM_REV_GPIO_R, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmR); ///< Enable the BLDC motor
    bldc_set_duty(&pwmR, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmL, PWM_GPIO_L, PWM_REV_GPIO_L, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmL); ///< Enable the BLDC motor
    bldc_set_duty(&pwmL, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmB, PWM_GPIO_B, PWM_REV_GPIO_B, PWM_FREQ, 1, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmB); ///< Enable the BLDC motor
    bldc_set_duty(&pwmB, 0); ///< Set the duty cycle to 0%
    ///<--------------------------------------------------

    ///<---------- Initialize the AS5600 sensors ---------
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis 2LSB
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 8x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Fast filter threshold 6LSB
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog off
    };

    adc_oneshot_unit_handle_t handle;
    if (!adc_create_unit(&handle, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE("AS5600_ADC_UNIT", "AS5600 ADC initialization failed.\n");
        return;
    }

    gAs5600R.conf = conf; ///< Set the configuration for the right AS5600 sensor
    gAs5600R.out = AS5600_OUT_GPIO_RIGHT; ///< Set the OUT GPIO pin for the right AS5600 sensor
    gAs5600R.adc_handle.adc_handle = handle; ///< Set the ADC handle for the right AS5600 sensor
    if (!adc_config_channel(&gAs5600R.adc_handle, AS5600_OUT_GPIO_RIGHT, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 right sensor ADC initialization failed\n");
    }
    
    gAs5600L.conf = conf; ///< Set the configuration for the left AS5600 sensor
    gAs5600L.out = AS5600_OUT_GPIO_LEFT; ///< Set the OUT
    gAs5600L.adc_handle.adc_handle = handle; ///< Set the ADC handle for the left AS5600 sensor
    if (!adc_config_channel(&gAs5600L.adc_handle, AS5600_OUT_GPIO_LEFT, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 left sensor ADC initialization failed\n");
    }

    gAs5600B.conf = conf; ///< Set the configuration for the back AS5600 sensor
    gAs5600B.out = AS5600_OUT_GPIO_BACK; ///< Set the OUT GPIO pin for the back AS5600 sensor
    gAs5600B.adc_handle.adc_handle = handle; ///< Set the ADC handle for the back AS5600 sensor
    if (!adc_config_channel(&gAs5600B.adc_handle, AS5600_OUT_GPIO_BACK, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 back sensor ADC initialization failed\n");
    }
    
    // CRITICAL: Wait for ADC calibration to complete before any tasks use it
    ESP_LOGI(TAG, "Waiting for ADC calibration to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(500));
    ///<--------------------------------------------------

     ///<------------- Initialize the PID controllers ------
    pid_config_t pid_config = {
        .init_param = pid_paramR
    };
    pid_new_control_block(&pid_config, &pidR);

    pid_config.init_param = pid_paramL;
    pid_new_control_block(&pid_config, &pidL);

    pid_config.init_param = pid_paramB;
    pid_new_control_block(&pid_config, &pidB);
    ///<---------------------------------------------------

    ///<---------- Initialize control parameters ---------
    static control_params_t right_control_params = {
        .gStruct = &gAs5600R,
        .sensor_data = &right_encoder_data,
        .pid_block = &pidR,
        .pwm_motor = &pwmR,

        .predef_move = 0, ///< Predefined movements for the robot, can be set later
        .vel_selection = 2 ///< Velocity selection for the robot, can be set later
    };

    static control_params_t left_control_params = {
        .gStruct = &gAs5600L,
        .sensor_data = &left_encoder_data,
        .pid_block = &pidL,
        .pwm_motor = &pwmL,

        .predef_move = 1, ///< Predefined movements for the robot, can be set later
        .vel_selection = 0 ///< Velocity selection for the robot, can be set later
    };

    static control_params_t back_control_params = {
        .gStruct = &gAs5600B,
        .sensor_data = &back_encoder_data,
        .pid_block = &pidB,
        .pwm_motor = &pwmB,

        .predef_move = 2, ///< Predefined movements for the robot, can be set later
        .vel_selection = 1 ///< Velocity selection for the robot, can be set later
    };

    static encoder_params_t encoder_params = {
        .right_gStruct = &gAs5600R,
        .left_gStruct = &gAs5600L,
        .back_gStruct = &gAs5600B,
        .right_sensor_data = &right_encoder_data,
        .left_sensor_data = &left_encoder_data,
        .back_sensor_data = &back_encoder_data
    };

    static distance_params_t distance_params = {
        .target_distance = 5.0f, ///< Set the target distance to 100 cm
        .encoder_data_right = &right_encoder_data,
        .encoder_data_left = &left_encoder_data,
        .encoder_data_back = &back_encoder_data
    };

    ///<--------------------------------------------------

    TaskHandle_t xRightEncoderTaskHandle, xLeftEncoderTaskHandle, xBackEncoderTaskHandle, xEncodersTaskHandle; ///< Task handles for encoders
    
    TaskHandle_t xRightControlTaskHandle, xLeftControlTaskHandle, xBackControlTaskHandle, xDistanceTaskHandle; ///< Task handles for control tasks

    ESP_LOGI(TAG, "=== PID Autotuner Robot ===");

    ///<-------------- Create the encoders tasks ---------------

    // xTaskCreatePinnedToCore(vTaskEncoder, "right_encoder_task", 4096, &right_control_params, 8, &xRightEncoderTaskHandle, 0); ///< Create the task to read from right encoder
    // xTaskCreatePinnedToCore(vTaskEncoder, "left_encoder_task", 4096, &left_control_params, 8, &xLeftEncoderTaskHandle, 0);    ///< Create the task to read from left encoder
    // xTaskCreatePinnedToCore(vTaskEncoder, "back_encoder_task", 4096, &back_control_params, 8, &xBackEncoderTaskHandle, 0);    ///< Create the task to read from back encoder

    // configASSERT(xRightEncoderTaskHandle); ///< Check if the task was created successfully
    // if (xRightEncoderTaskHandle == NULL) {
    //     ESP_LOGE("ENCODER_TASK", "Failed to create task...");
    //     return;
    // }
    // configASSERT(xLeftEncoderTaskHandle); ///< Check if the task was created successfully
    // if (xLeftEncoderTaskHandle == NULL) {
    //     ESP_LOGE("ENCODER_TASK", "Failed to create task...");
    //     return;
    // }
    // configASSERT(xBackEncoderTaskHandle); ///< Check if the task was created successfully
    // if (xBackEncoderTaskHandle == NULL) {
    //     ESP_LOGE("ENCODER_TASK", "Failed to create task...");
    //     return;
    // }

    xTaskCreatePinnedToCore(vTaskEncoders, "encoders_task", 12288, &encoder_params, 8, &xEncodersTaskHandle, 0); ///< Create the task to read from all encoders
    configASSERT(xEncodersTaskHandle); ///< Check if the task was created successfully
    if (xEncodersTaskHandle == NULL) {
        ESP_LOGE("ENCODERS_TASK", "Failed to create task...");
        return;
    }
    ///<--------------------------------------------------------

    ///<-------------- WiFi and Autotuning ----------

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Creating autotuning task...");
    configASSERT(xTaskCreatePinnedToCore(autotuning_task, "autotuning", 8192, NULL, 7, NULL, 0));
    ///<--------------------------------------------------------

    // vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 1 second to ensure all peripherals are initialized
    
    // // Create autotuning task
    // configASSERT(xTaskCreatePinnedToCore(autotuning_task, "autotuning", 8192, NULL, 9, NULL, 0));

    // vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 1 second to ensure all peripherals are initialized

    ///<--------------------------------------------------------

    ///<-------------- Create the control tasks ----------------
    
    xTaskCreatePinnedToCore(vTaskControl, "rwh_control_task", 4096, &right_control_params, 9, &xRightControlTaskHandle, 1); ///< Create the task to control the right wheel
    xTaskCreatePinnedToCore(vTaskControl, "lwh_control_task", 4096, &left_control_params, 9, &xLeftControlTaskHandle, 1);   ///< Create the task to control the left wheel
    xTaskCreatePinnedToCore(vTaskControl, "bwh_control_task", 4096, &back_control_params, 9, &xBackControlTaskHandle, 1);   ///< Create the task to control the back wheel

    configASSERT(xRightControlTaskHandle); ///< Check if the task was created successfully
    if (xRightControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xLeftControlTaskHandle); ///< Check if the task was created successfully
    if (xLeftControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xBackControlTaskHandle); ///< Check if the task was created successfully
    if (xBackControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }

    xTaskCreatePinnedToCore(vTaskDistance, "distance_task", 2048, &distance_params, 8, &xDistanceTaskHandle, 1); ///< Create the task to keep track of distance
    configASSERT(xDistanceTaskHandle); ///< Check if the task was created successfully
    if (xDistanceTaskHandle == NULL) {
        ESP_LOGE("DISTANCE_TASK", "Failed to create task...");
        return;
    }

    ///<--------------------------------------------------------

    

    // app_main can safely return - FreeRTOS tasks will continue running
    ESP_LOGI(TAG, "Initialization complete, tasks running...");
}
