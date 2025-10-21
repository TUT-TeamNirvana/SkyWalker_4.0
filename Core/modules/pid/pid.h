#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float output;
    float output_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output);
float PID_Calc(PID_t *pid, float ref, float feedback);

#endif
