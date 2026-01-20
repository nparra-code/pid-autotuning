/**
 * @file mov_calculation.c
 * @brief Movement kinematics calculations for omniwheel robot
 * @details Implements inverse kinematics transformations to convert desired
 *          robot velocities to individual wheel velocities for a 3-wheel
 *          omniwheel configuration with 30° orientation angle
 * 
 * @author Nelson Fernando Parra Guardia
 * @date January 2026
 * @version 1.0
 */

#include "mov_calculation.h"

// Global flag for resetting movement sequence
bool g_reset_movements_flag = false;

/**
 * @brief Calculate velocity components for linear motion
 * @details Decomposes desired linear motion into x and y velocity components
 *          based on direction and angle
 * 
 * @param forward True for forward motion, false for reverse
 * @param linear_velocity Desired linear speed in cm/s
 * @param angle Direction angle in degrees (0° = forward, 90° = right)
 * @param x_velocity Output pointer for x-component velocity (cm/s)
 * @param y_velocity Output pointer for y-component velocity (cm/s)
 */
void linear_movement(bool forward, float linear_velocity, float angle, float *x_velocity, float *y_velocity) {
    if (forward) {
        *x_velocity = -linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = -linear_velocity * cosf(angle * PI / 180.0f);
    } else {
        *x_velocity = linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = linear_velocity * cosf(angle * PI / 180.0f);
    }
}

/**
 * @brief Calculate velocity components for circular motion
 * @details Generates tangential velocities for following a circular arc.
 *          Uses parametric equations with time-varying angular position.
 * 
 * @param cw True for clockwise rotation, false for counter-clockwise
 * @param linear_velocity Tangential speed along circle (cm/s)
 * @param angle Total angle to traverse in degrees
 * @param radius Radius of circular path (cm)
 * @param x_velocity Output pointer for x-component velocity (cm/s)
 * @param y_velocity Output pointer for y-component velocity (cm/s)
 * 
 * @note Uses static time variable that increments at SAMPLE_TIME rate
 * @note Automatically resets when motion completes
 */
void circular_movement(bool cw, float linear_velocity, float angle, float radius, float *x_velocity, float *y_velocity) {
    static float t = 0;

    if (t < (angle / 360.0) * 2 * PI * radius / linear_velocity) { ///< Time to reach the goal angle in seconds 360° is a full circle

        if (cw) {
            *x_velocity = -radius * sinf((linear_velocity / radius) * t);
            *y_velocity =  radius * cosf((linear_velocity / radius) * t);
        } else {
            *x_velocity = -radius * sinf((linear_velocity / radius) * t);
            *y_velocity = -radius * cosf((linear_velocity / radius) * t);
        }

        t += 0.002; ///< Increment time by the sample time in seconds

    } else {

        *x_velocity = 0.0f; ///< Stop the movement
        *y_velocity = 0.0f; ///< Stop the movement
        t = 0.0f; ///< Reset time for the next circular movement
        
    }
}

/**
 * @brief Convert linear velocities to wheel angular velocity
 * @details Applies inverse kinematics transformation for 3-wheel omniwheel
 *          configuration. Accounts for wheel orientation angle (DELTA) and
 *          gear ratio (N/R).
 * 
 * @param x_velocity Body-frame x velocity component (cm/s)
 * @param y_velocity Body-frame y velocity component (cm/s)
 * @param vel_selection Wheel selector: 0=left, 1=back, 2=right
 * @param wheel_velocity Output pointer for wheel angular velocity (cm/s)
 * 
 * @note Includes scaling factor of 1/5 for motor characteristics
 * @note Uses DELTA=π/6 (30°) for wheel orientation
 */
void cal_lin_to_ang_velocity(float x_velocity, float y_velocity, uint8_t vel_selection, float *wheel_velocity) {

    float scale = N / R;

    float cos_d = cosf(DELTA);
    float sin_d = sinf(DELTA);

    // wb = 0, so ignore third column
    switch (vel_selection)  // vel_selection is a constant expression
    {
    case 0: // left wheel
        *wheel_velocity = scale * ( - sin_d * x_velocity - cos_d * y_velocity ) / 5;
        break;
    case 1: // back wheel
        *wheel_velocity = scale * ( x_velocity ) / 5; ///< Adjusted for consistency;
        break;
    case 2: // right wheel
        *wheel_velocity = scale * ( -sin_d * x_velocity + cos_d * y_velocity ) / 5; ///< Adjusted for consistency;
        break;
    
    default:
        break;
    }
}

/**
 * @brief Reset movement sequence to initial state
 * @details Sets global flag to restart movement sequence from beginning.
 *          Clears all static variables in multiple_movements().
 */
void reset_movements(void) {
    // External reset flag that will be checked by multiple_movements
    extern bool g_reset_movements_flag;
    g_reset_movements_flag = true;
}

/**
 * @brief Execute a sequence of movements
 * @details State machine that steps through an array of movement commands,
 *          generating appropriate velocity outputs for each phase. Handles
 *          timing, transitions, and completion detection.
 * 
 * @param movements Pointer to array of Movement structures
 * @param movement_count Number of movements in array
 * @param x_velocity Output pointer for current x velocity (cm/s)
 * @param y_velocity Output pointer for current y velocity (cm/s)
 * 
 * @return true when all movements completed, false during execution
 * 
 * @note Uses static variables for state persistence across calls
 * @note Call rate should match SAMPLE_TIME for accurate timing
 * @note Responds to g_reset_movements_flag for external reset
 */
bool multiple_movements(Movement *movements, uint8_t movement_count, float *x_velocity, float *y_velocity)
{
    static float g_time = 0.0f;
    static uint8_t i = 0;
    static Movement mov;  // Make mov static
    static bool initialized = false;  // Flag to initialize mov once
    static bool indefinite = false, stop = false;
    static bool movements_complete = false;
    
    // Check for external reset request
    extern bool g_reset_movements_flag;
    if (g_reset_movements_flag) {
        g_time = 0.0f;
        i = 0;
        initialized = false;
        indefinite = false;
        stop = false;
        movements_complete = false;
        g_reset_movements_flag = false;
        ESP_LOGI("MUL_MOVS", "Movement sequence reset");
    }
    
    // Initialize mov on first call
    if (!initialized) {
        mov = movements[0];
        initialized = true;
    }
        
    if (g_time >= mov.duration && !indefinite) {
        g_time = 0.0f; // Reset global time for the next movement
        if (++i >= movement_count) {
            i = 0;
            stop = true;
            indefinite = true; // All movements completed, enter indefinite mode
            movements_complete = true;
            // ESP_LOGW("MUL_MOVS", "All movements completed. Stop!");
        }
        // ESP_LOGI("MUL_MOVS", "Completed movement %d", i);
        mov = movements[i];
    } else if (!indefinite) {
        g_time += SAMPLE_TIME / 1000.0f; // Increment global time by the sample time in seconds
    }

    // static int ctr = 0;
    // if (++ctr >= 200 / SAMPLE_TIME) { 
    //     // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
    //     // ESP_LOGI("MUL_MOVS", "Movement %d: Type: %d, Dir: %d, Lin Vel: %.2f, Angle: %.2f, Radius: %.2f, Duration: %.2f, Elapsed Time: %.2f", 
    //     //             i, mov.movement_type, mov.direction, mov.linear_velocity, mov.angle, mov.radius, mov.duration, g_time); ///< Log the current movement
    //     ctr = 0;
    // }

    float temp_x = 0.0f;
    float temp_y = 0.0f;

    switch (stop ? DO_NOT_MOVE : mov.movement_type) {
        case LINEAR:
            linear_movement(mov.direction, mov.linear_velocity, mov.angle, &temp_x, &temp_y);
            break;
        case CIRCULAR:
            circular_movement(mov.direction, mov.linear_velocity, mov.angle, mov.radius, &temp_x, &temp_y);
            break;
        case ROTATION:
            // Rotation logic can be added here if needed
            break;
        case DO_NOT_MOVE:
            temp_x = 0.0f;
            temp_y = 0.0f;
            break;
        default:
            break;
    }

    *x_velocity = temp_x;
    *y_velocity = temp_y;
    return movements_complete;
}
