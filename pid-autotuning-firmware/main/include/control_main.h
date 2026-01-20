/**
 * @file control_main.h
 * @brief Header file for omniwheel robot control system
 * @details Defines structures, constants, and function prototypes for
 *          encoder reading, PID control, and motor actuation tasks
 * 
 * @author Nelson Fernando Parra Guardia
 * @date January 2026
 * @version 1.0
 */

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
#define AS5600_OUT_GPIO_RIGHT 5 ///< gpio number for right OUT signal
#define AS5600_OUT_GPIO_LEFT  6 ///< gpio number for left OUT signal
#define AS5600_OUT_GPIO_BACK  7 ///< gpio number for back OUT signal

// #define AS5600_OUT_GPIO_RIGHT 1 ///< gpio number for right OUT signal
// #define AS5600_OUT_GPIO_LEFT  6 ///< gpio number for left OUT signal
// #define AS5600_OUT_GPIO_BACK  7 ///< gpio number for back OUT signal

#define AS5600_ADC_UNIT_ID ADC_UNIT_1   ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

///<-------------- BLDC configuration -----------------
#define PWM_GPIO_R 20               ///< GPIO number for right PWM signal
#define PWM_REV_GPIO_R 21           ///< GPIO number for right PWM reverse signal

#define PWM_GPIO_L 47               ///< GPIO number for left PWM signal
#define PWM_REV_GPIO_L 48           ///< GPIO number for left PWM reverse signal

#define PWM_GPIO_B 35               ///< GPIO number for back PWM signal
#define PWM_REV_GPIO_B 36           ///< GPIO number for back PWM reverse signal

// #define PWM_GPIO_R 46               ///< GPIO number for right PWM signal
// #define PWM_REV_GPIO_R 9           ///< GPIO number for right PWM reverse signal

// #define PWM_GPIO_L 17               ///< GPIO number for left PWM signal
// #define PWM_REV_GPIO_L 18           ///< GPIO number for left PWM reverse signal

// #define PWM_GPIO_B 8               ///< GPIO number for back PWM signal
// #define PWM_REV_GPIO_B 3           ///< GPIO number for back PWM reverse signal

#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

///<-------------- PID configuration -----------------
#define PID_KP_R 2.4658 // Proportional gain for right motor
#define PID_KI_R 0.0019 // Integral gain for right motor
#define PID_KD_R 0.0019 // Derivative gain for right motor

#define PID_KP_L 2.6909 // Proportional gain for left motor
#define PID_KI_L 0.0033 // Integral gain for left motor
#define PID_KD_L 0.0021 // Derivative gain for left motor

#define PID_KP_B 2.8562 // Proportional gain for back motor
#define PID_KI_B 0.0023 // Integral gain for back motor
#define PID_KD_B 0.0034 // Derivative gain for back motor

#define EULER 2.71828
#define PI 3.14159
///<--------------------------------------------------

/**
 * @brief Control parameters structure for individual motor control task
 * @details Contains all necessary pointers and configuration for a single
 *          motor's control loop including sensor data, PID controller,
 *          and PWM output
 */
typedef struct {
    AS5600_t * gStruct;             ///< AS5600 magnetic encoder interface
    encoder_data_t * sensor_data;   ///< Encoder velocity and position data
    pid_block_handle_t * pid_block; ///< PID controller instance
    bldc_pwm_motor_t * pwm_motor;   ///< BLDC PWM motor driver interface

    uint8_t predef_move;            ///< Predefined movement selection index
    uint8_t vel_selection;          ///< Wheel identification (0=left, 1=back, 2=right)
} control_params_t;

/**
 * @brief Encoder parameters structure for all three wheels
 * @details Groups AS5600 sensor interfaces and encoder data for the
 *          vTaskEncoders function to read all wheels simultaneously
 */
typedef struct {
    AS5600_t * right_gStruct;             ///< Right wheel AS5600 encoder interface
    AS5600_t * left_gStruct;              ///< Left wheel AS5600 encoder interface
    AS5600_t * back_gStruct;              ///< Back wheel AS5600 encoder interface
    encoder_data_t * right_sensor_data;   ///< Right wheel velocity/position data
    encoder_data_t * left_sensor_data;    ///< Left wheel velocity/position data
    encoder_data_t * back_sensor_data;    ///< Back wheel velocity/position data
} encoder_params_t;

/**
 * @brief Distance tracking parameters structure
 * @details Used by vTaskDistance to monitor accumulated travel distance
 *          from all three wheels
 */
typedef struct {
    float target_distance; ///< Target distance to travel (cm)
    encoder_data_t * encoder_data_right; ///< Right wheel encoder data
    encoder_data_t * encoder_data_left;  ///< Left wheel encoder data
    encoder_data_t * encoder_data_back;  ///< Back wheel encoder data

} distance_params_t;

/**
 * @brief Task to read from encoders
 */
void vTaskEncoders(void * pvParameters);

/**
 * @brief Task to control the wheel
 * 
 * @param pvParameters 
 */
void vTaskControl( void * pvParameters );
/**
 * @brief Task to keep track of distance
 *
 * @param pvParameters
 */
void vTaskDistance(void * pvParameters);

/**
 * @brief Task to input PWMs to the motor controller
 * 
 * @param pvParameters 
 */
void vTaskIdent( void * pvParameters );

// Global variables for movement completion tracking
extern volatile bool g_all_movements_complete;
extern volatile int g_movements_complete_count;
extern bool g_reset_movements_flag;

#endif // CONTROL_H