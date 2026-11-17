#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#define DRIVE_MOTOR_COUNT 6
#define ENCODER_COUNT 4
#ifdef __cplusplus
extern "C" {
#endif

// =====================================================
// GLOBAL VARIABLES
// =====================================================

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
