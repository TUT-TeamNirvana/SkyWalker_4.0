#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#include "main.h"
#include "bsp_can.h"
#include "pid.h"

#define M3508_MAX_NUM 4

typedef struct
{
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temp;
} M3508_Feedback_t;

typedef struct
{
    CANInstance *can;
    M3508_Feedback_t feedback;
    PID_t pid;
    float target_speed;
    uint8_t id;
} M3508_t;

void M3508_InitAll(M3508_t *motors, CAN_HandleTypeDef *hcan);
void M3508_SetTarget(M3508_t *motor, float target_rpm);
void M3508_UpdateAll(M3508_t *motors, uint8_t motor_count);
void M3508_Callback(CANInstance *instance);

#endif
