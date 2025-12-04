#include "control_main.h"

bool forward_mov[] = {true, true, true, false, false, false, false, true}; ///< Forward movements for the robot
float linear_velocity[] = {15.0f, 0.0f, 15.0f, 0.0f, 15.0f, 0.0f, 15.0f, 0.0f}; ///< Linear velocities for the robot in cm/s
float angle[] = {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 90.0f, 0.0f}; ///< Angles for the robot in degrees

float predef_move2[3][8] = { // {right, left, back} velocity in cm/s
    {-15.0f, 0.0f, 15.0f, 0.0f, -15.0f, 0.0f, 15.0f, 0.0f}, ///< Predefined movements for the robots right wheel
    {15.0f, 0.0f, 15.0f, 0.0f, -15.0f, 0.0f, -15.0f, 0.0f}, ///< Predefined movements for the robots left wheel
    {0.0f, 0.0f, -15.0f, 0.0f, 15.0f, 0.0f, 0.0f, 0.0f} ///< Predefined movements for the robots back wheel
};

encoder_data_t right_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

encoder_data_t left_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

encoder_data_t back_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

pid_parameter_t pid_paramR = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramL = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramB = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

enum movements_num movement = LINEAR; ///< Movement type
float x_vel = .0f, y_vel = .0f; ///< Generalized velocities for the robot
float goal_time = 2.0f; ///< Goal time for linear movement in seconds

// Task to read from encoder
void vTaskEncoder(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    ///<-------------- Get angle through ADC -------------
    while (1) {

        encoder_data->angle = AS5600_ADC_GetAngle(params->gStruct); ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data

        // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 100 / SAMPLE_TIME) {
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }

        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
    ///<--------------------------------------------------
}

// Task to control the wheel
void vTaskControl( void * pvParameters ){

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    pid_block_handle_t pid_block = *(params->pid_block); ///< PID control block handle
    
    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    float beta = exp(-2 * PI * 1 / 100);  // 10Hz cutoff frequency
    float output = 0.0f;
    float setpoint = 0.0f;

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    Movement movements[] = {
        {LINEAR, true, 15, 90, .0, 10.0f},
        {CIRCULAR, true, 5, 360, 20.0f, (360.0 / 360.0) * 2 * PI * 20 / 5},
    };

    while (1)
    {
        ///<-------------- PID Control ---------------
        // Low-pass filter
        est_velocity = encoder_data->velocity;
        est_velocity = beta * last_est_velocity + (1 - beta) * est_velocity;

        last_est_velocity = est_velocity; ///< Update the last estimated velocity

        multiple_movements(movements, 2, &x_vel, &y_vel); ///< Get the generalized velocities for the robot
        cal_lin_to_ang_velocity(x_vel, y_vel, params->vel_selection, &setpoint); ///< Calculate the angular velocity for the wheel

        if (pid_update_set_point(pid_block, setpoint) != PID_OK) {
            // ESP_LOGE(task_name, "Failed to update PID parameters for %s", task_name);
        }

        // Update PID Controller
        pid_compute(pid_block, est_velocity, &output);
        bldc_set_duty(params->pwm_motor, output); ///< Set the duty cycle to the output of the PID controller

        // Log every 100ms because of the ESP_LOGI overhead
        static int ctr = 0;
        if (++ctr >= 100 / SAMPLE_TIME) { 
            // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
            ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f\tSetpoint: %.2f", est_velocity, output, setpoint); ///< Log the PID parameters
            // ESP_LOGI("CTRL_TASK", "X_vel: %.2f\tY_vel: %.2f", x_vel, y_vel); ///< Log the generalized velocities
            ctr = 0;
        }
        
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}

// Task to keep track of distance
void vTaskDistance(void *pvParameters){
    
    distance_params_t *params = (distance_params_t *)pvParameters; ///< Distance parameters structure
    encoder_data_t *encoder_data_right = params->encoder_data_right; ///< Encoder data structure for right wheel
    encoder_data_t *encoder_data_left = params->encoder_data_left; ///< Encoder data structure for left wheel
    encoder_data_t *encoder_data_back = params->encoder_data_back; ///< Encoder data structure for back wheel

    float dx, dy, distance = 0, beta = 0.9; ///< Variables to store the distance

    while(1){
        static uint16_t time_count = 0; ///< Counter to keep track of the number of iterations

        if(time_count >= goal_time * 1000 && movement == LINEAR) { ///< Check if the goal time has been reached
            movement = DO_NOT_MOVE; ///< Set the movement to do not move
            time_count = 0; ///< Reset the time count
        } else {
            time_count += 5 * SAMPLE_TIME; ///< Increment the time count
        }

        vTaskDelay(5 * SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }

}