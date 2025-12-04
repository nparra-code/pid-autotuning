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

#include "control_main.h"

void app_main(void)
{

    static AS5600_t gAs5600R, gAs5600L, gAs5600B;  ///< AS5600 object for angle sensor right, left and back

    extern encoder_data_t right_encoder_data, left_encoder_data, back_encoder_data; ///< Encoder data for right, left and back wheels

    static bldc_pwm_motor_t pwmR, pwmL, pwmB;   ///< BLDC motor object right, left and back
    static pid_block_handle_t pidR, pidL, pidB; ///< PID control block handle

    extern pid_parameter_t pid_paramR, pid_paramL, pid_paramB; ///< PID parameters for right, left and back wheels 

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

    static distance_params_t distance_params = {
        .target_distance = 5.0f, ///< Set the target distance to 100 cm
        .encoder_data_right = &right_encoder_data,
        .encoder_data_left = &left_encoder_data,
        .encoder_data_back = &back_encoder_data
    };

    ///<--------------------------------------------------

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 1 second to ensure all peripherals are initialized

    ///<-------------- Create the task ---------------

    TaskHandle_t xRightEncoderTaskHandle, xLeftEncoderTaskHandle, xBackEncoderTaskHandle; ///< Task handles for encoders
    
    TaskHandle_t xRightControlTaskHandle, xLeftControlTaskHandle, xBackControlTaskHandle, xDistanceTaskHandle; ///< Task handles for control tasks
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

    ESP_LOGI("TASKS", "Right encoder handle: 0x%04X", gAs5600R.out); ///< Log the task handles
    xTaskCreatePinnedToCore(vTaskEncoder, "right_encoder_task", 4096, &right_control_params, 8, &xRightEncoderTaskHandle, 0); ///< Create the task to read from right encoder
    xTaskCreatePinnedToCore(vTaskEncoder, "left_encoder_task", 4096, &left_control_params, 8, &xLeftEncoderTaskHandle, 0);    ///< Create the task to read from left encoder
    xTaskCreatePinnedToCore(vTaskEncoder, "back_encoder_task", 4096, &back_control_params, 8, &xBackEncoderTaskHandle, 0);    ///< Create the task to read from back encoder

    configASSERT(xRightEncoderTaskHandle); ///< Check if the task was created successfully
    if (xRightEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xLeftEncoderTaskHandle); ///< Check if the task was created successfully
    if (xLeftEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xBackEncoderTaskHandle); ///< Check if the task was created successfully
    if (xBackEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }
    ///<--------------------------------------------------


    





}
