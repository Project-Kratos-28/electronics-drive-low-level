
extern "C" {
    #include <rcl/rcl.h>
    #include <rclc/executor.h>
}
#include "motor_control.h"
#include "SpeedMotor.h"
#include "main.h"
#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"


// =====================================================
// ROS 2 INTEGRATION VARIABLES & HOOKS
// =====================================================
extern "C" {
    extern double   motor_cmd_shared[6];
    extern uint32_t motor_cmd_stamp_ms;
    extern bool     motor_cmd_valid;

    // Mandatory hooks required by the micro-ROS skeleton
    bool motor_ros_init(rcl_node_t *node, rclc_executor_t *executor) { (void)node; (void)executor; return true; }
    void motor_ros_fini(rcl_node_t *node) { (void)node; }
    void motor_ros_publish(void) { }
}

// =====================================================
// HARDWARE PERIPHERAL DECLARATIONS
// =====================================================
// --- ENCODER TIMERS ---
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

// --- PWM TIMERS ---
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

// =====================================================
// MOTOR & HARDWARE ARRAYS
// =====================================================
static SpeedMotor drive_motors[DRIVE_MOTOR_COUNT] = {
    SpeedMotor(&htim13, TIM_CHANNEL_1, 3199, GPIOG, GPIO_PIN_8, GPIO_PIN_8, 2.0f, 1.0f, 0.0f), // Motor 0
    SpeedMotor(&htim14, TIM_CHANNEL_1, 3199, GPIOG, GPIO_PIN_9, GPIO_PIN_9, 2.0f, 1.0f, 0.0f), // Motor 1
    SpeedMotor(&htim15, TIM_CHANNEL_2, 3199, GPIOG, GPIO_PIN_5, GPIO_PIN_5, 2.0f, 1.0f, 0.0f), // Motor 2
    SpeedMotor(&htim5, TIM_CHANNEL_3, 3199, GPIOG, GPIO_PIN_4, GPIO_PIN_4, 2.0f, 1.0f, 0.0f), // Motor 3
    SpeedMotor(&htim5, TIM_CHANNEL_2, 3199, GPIOG, GPIO_PIN_3, GPIO_PIN_3, 2.0f, 1.0f, 0.0f), // Motor 4
    SpeedMotor(&htim17, TIM_CHANNEL_1, 3199, GPIOG, GPIO_PIN_7, GPIO_PIN_7, 2.0f, 1.0f, 0.0f)  // Motor 5
};

static TIM_HandleTypeDef* drive_encoders[ENCODER_COUNT] = {&htim1, &htim2, &htim3, &htim4};

static uint16_t drive_prev_counts[DRIVE_MOTOR_COUNT] = {0,0,0,0,0,0};
static float current_rpms[DRIVE_MOTOR_COUNT] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

// Live target speeds updated by ROS 2 (Initialized to 0.0f for safety)
static float target_rpms[DRIVE_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// =====================================================
// INTERNAL C++ FUNCTIONS
// =====================================================
static float ReadDriveSpeedRPM(uint8_t motor_id, float dt_seconds)
{
    if (dt_seconds <= 0.0f) return 0.0f;

    TIM_HandleTypeDef* htim = drive_encoders[motor_id];
    uint16_t current_count = (uint16_t)__HAL_TIM_GET_COUNTER(htim);

    int16_t delta_ticks = (int16_t)(current_count - drive_prev_counts[motor_id]);
    drive_prev_counts[motor_id] = current_count;

    const float ENCODER_PPR = 13.0f;
    const float GEAR_RATIO = 181.0f;
    const float ENCODER_MODE_MULT = 4.0f;
    const float TICKS_PER_REV = ENCODER_PPR * GEAR_RATIO * ENCODER_MODE_MULT;

    float revolutions = (float)delta_ticks / TICKS_PER_REV;
    return (revolutions / dt_seconds) * 60.0f;
}

// =====================================================
// C-ACCESSIBLE WRAPPER FUNCTIONS (Used by FreeRTOS)
// =====================================================
extern "C" {

void MotorControl_Init(void)
{
    float gains[DRIVE_MOTOR_COUNT][3]=
    {{1.05,0,0},
     {1.1,0,0},
	 {2,0,0},
	 {2,0,0},
	 {2,0,0},
	 {2,0,0}
    };

    for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
        drive_motors[i].begin();
        drive_motors[i].setGains(gains[i][0],gains[i][1],gains[i][2]);
    }

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);


    for (int i = 0; i < ENCODER_COUNT; i++) {
        drive_prev_counts[i] = (uint16_t)__HAL_TIM_GET_COUNTER(drive_encoders[i]);
    }

}

void MotorControl_Update(float dt_seconds)
{
    if (dt_seconds <= 0.0f) return;

    // --- ROS 2 Integration & Safety Timeout ---
    double   cmd[6];
    uint32_t stamp;
    bool     valid;

    taskENTER_CRITICAL();
    memcpy(cmd, motor_cmd_shared, sizeof(cmd));
    stamp = motor_cmd_stamp_ms;
    valid = motor_cmd_valid;
    taskEXIT_CRITICAL();

    if (valid && (osKernelGetTickCount() - stamp) < 500){
        for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
            target_rpms[i] = (float)cmd[i];
        }
    } else {
        // Connection lost or stale data -> Stop all motors
        for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
            target_rpms[i] = 0.0f;
        }
    }
    // ------------------------------------------

    // 1. Read all 6 hardware timers (2 and 3 will read junk/zero)
        for (int i = 0; i < ENCODER_COUNT; i++) {
            current_rpms[i] = ReadDriveSpeedRPM(i, dt_seconds);
        }

        // 2. Fabricate the middle wheel values by averaging the sides
        current_rpms[4] = (current_rpms[0] + current_rpms[2]) / 2.0f; // Middle Right
        current_rpms[5] = (current_rpms[1] + current_rpms[3]) / 2.0f; // Middle Left


        // 3. Update the PID controllers
        for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
            drive_motors[i].setTargetSpeed(target_rpms[i]);
            drive_motors[i].update(current_rpms[i], dt_seconds);
        }
}

// FreeRTOS Task Entry Point
void motor_control_Run(void *argument)
{
    (void)argument;
    
    static bool initialised = false;
    if (!initialised)
    {
        MotorControl_Init();
        initialised = true;
    }

    // freertos.c handles the osDelay(50) loop, so we pass 0.05s directly
    MotorControl_Update(0.05f);
}

} // end extern "C"
