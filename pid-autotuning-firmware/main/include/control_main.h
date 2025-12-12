#ifndef CONTROL_H
#define CONTROL_H

// Include standar libraries 
#include <stdio.h>

// Include personalized sensors libraries
#include "as5600_lib.h"

// Include personalized driver libraries
#include "bldc_pwm.h"
#include "pid_ext.h"
#include "sensor_fusion.h"
#include "mov_calculation.h"

// Include ESP IDF libraries
#include <assert.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"

#define WHEEL_RADIO 3.0f ///< Radio of the wheel in cm

///<-------------- AS5600 configuration --------------
#define AS5600_OUT_GPIO_RIGHT 5         ///< gpio number for right OUT signal
#define AS5600_OUT_GPIO_LEFT 6          ///< gpio number for left OUT signal
#define AS5600_OUT_GPIO_BACK 7          ///< gpio number for back OUT signal
#define AS5600_ADC_UNIT_ID ADC_UNIT_1   ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

///<-------------- BLDC configuration -----------------
#define PWM_GPIO_R 20               ///< GPIO number for right PWM signal
#define PWM_REV_GPIO_R 21           ///< GPIO number for right PWM reverse signal

#define PWM_GPIO_L 47               ///< GPIO number for left PWM signal
#define PWM_REV_GPIO_L 48           ///< GPIO number for left PWM reverse signal

#define PWM_GPIO_B 45               ///< GPIO number for back PWM signal
#define PWM_REV_GPIO_B 0           ///< GPIO number for back PWM reverse signal

#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

///<-------------- PID configuration -----------------
#define PID_KP .04//.01f
#define PID_KI .02//.01f
#define PID_KD 0.0//.001f
#define EULER 2.71828
#define PI 3.14159
///<--------------------------------------------------

typedef struct {
    AS5600_t * gStruct;             ///< Velocity estimation from encoder in cm/s
    encoder_data_t * sensor_data;   ///< Velocity estimation from IMU in cm/s
    pid_block_handle_t * pid_block; ///< Velocity estimation from Lidar in cm/s
    bldc_pwm_motor_t * pwm_motor;   ///< BLDC motor object

    uint8_t predef_move;            ///< Predefined movements for the robot
    uint8_t vel_selection;          ///< Velocity selection for the robot
} control_params_t;

typedef struct {
    AS5600_t * right_gStruct;             ///< Velocity estimation from right encoder in cm/s
    AS5600_t * left_gStruct;              ///< Velocity estimation from left encoder in cm/s
    AS5600_t * back_gStruct;              ///< Velocity estimation from back encoder in cm/s
    encoder_data_t * right_sensor_data;   ///< Encoder data structure for right wheel
    encoder_data_t * left_sensor_data;    ///< Encoder data structure for left wheel
    encoder_data_t * back_sensor_data;    ///< Encoder data structure for back wheel
} encoder_params_t;

typedef struct {
    float target_distance; ///< Distance measurement
    encoder_data_t * encoder_data_right; ///< Encoder data structure for right wheel
    encoder_data_t * encoder_data_left;  ///< Encoder data structure for left wheel
    encoder_data_t * encoder_data_back;  ///< Encoder data structure for back wheel

} distance_params_t;

/**
 * @brief Task to read from encoder
 */
void vTaskEncoder(void * pvParameters);

void vTaskEncoders(void * pvParameters);

/**
 * @brief Task to control the wheel
 * 
 * @param pvParameters 
 */
void vTaskControl( void * pvParameters );

/**
 * @brief Task to keep track of distace
 * 
 * @param pvParameters
 */
void vTaskDistance(void * pvParameters);

#endif // CONTROL_H