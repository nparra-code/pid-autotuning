#include "mov_calculation.h"
#include "esp_log.h"

void linear_movement(bool forward, float linear_velocity, float angle, float *x_velocity, float *y_velocity) {
    if (forward) {
        *x_velocity = -linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = -linear_velocity * cosf(angle * PI / 180.0f);
    } else {
        *x_velocity = linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = linear_velocity * cosf(angle * PI / 180.0f);
    }
}

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
        
    }
}

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

void multiple_movements(Movement *movements, uint8_t movement_count, float *x_velocity, float *y_velocity)
{
    static float g_time = 0.0f;
    static uint8_t i = 0;
    static Movement mov;  // Make mov static
    static bool initialized = false;  // Flag to initialize mov once
    static bool indefinite = false, stop = false;
    
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
            // ESP_LOGW("MUL_MOVS", "All movements completed. Stop!");
        }
        // ESP_LOGI("MUL_MOVS", "Completed movement %d", i);
        mov = movements[i];
    } else if (!indefinite) {
        g_time += SAMPLE_TIME / 1000.0f; // Increment global time by the sample time in seconds
    }

    static int ctr = 0;
    if (++ctr >= 200 / SAMPLE_TIME) { 
        // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
        // ESP_LOGI("MUL_MOVS", "Movement %d: Type: %d, Dir: %d, Lin Vel: %.2f, Angle: %.2f, Radius: %.2f, Duration: %.2f, Elapsed Time: %.2f", 
        //             i, mov.movement_type, mov.direction, mov.linear_velocity, mov.angle, mov.radius, mov.duration, g_time); ///< Log the current movement
        ctr = 0;
    }

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
    return;
}
