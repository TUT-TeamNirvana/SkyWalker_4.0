#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->output = 0;
    pid->output_max = max_output;
}

float PID_Calc(PID_t *pid, float ref, float feedback)
{
    float error = ref - feedback;
    pid->integral += error;
    float derivative = error - pid->last_error;
    pid->last_error = error;

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}
