#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================
// GLOBAL VARIABLES
// =====================================================
extern volatile float g_target_rpm;
extern volatile float g_current_rpm;
extern volatile float g_kp;
extern volatile float g_ki;
extern volatile float g_kd;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================
/* Hardware initialization for timers and encoders */
void MotorControl_Init(void);

/* Main PID and ROS update math loop */
void MotorControl_Update(float dt_seconds);
/* FreeRTOS task entry point (Called by freertos.c) */
void motor_control_Run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
