#include "m3508_motor.h"
#include <stdio.h>

static M3508_t *motor_list[M3508_MAX_NUM] = {0};

/* -------------------- 初始化所有电机 -------------------- */
void M3508_InitAll(M3508_t *motors, CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < M3508_MAX_NUM; i++)
    {
        uint32_t rx_id = 0x201 + i;

        CAN_Init_Config_s config = {
            .can_handle = hcan,
            .tx_id = 0x200,
            .rx_id = rx_id,
            .can_module_callback = M3508_Callback,
            .id = &motors[i]
        };

        motors[i].can = CANRegister(&config);
        motors[i].id = i + 1; // 电机编号（1~4）
        motors[i].target_speed = 0;
        PID_Init(&motors[i].pid, 1.2f, 0.0f, 0.05f, 10000.0f);
        CANSetDLC(motors[i].can, 8);
        motor_list[i] = &motors[i];
    }
}

/* -------------------- 设置目标转速 -------------------- */
void M3508_SetTarget(M3508_t *motor, float target_rpm)
{
    motor->target_speed = target_rpm;
}

/* -------------------- PID计算并统一发送 -------------------- */
void M3508_UpdateAll(M3508_t *motors, uint8_t motor_count)
{
    int16_t currents[4] = {0};

    for (int i = 0; i < motor_count; i++)
    {
        float out = PID_Calc(&motors[i].pid,
                             motors[i].target_speed,
                             motors[i].feedback.speed_rpm);
        if (out > 10000) out = 10000;
        if (out < -10000) out = -10000;
        currents[i] = (int16_t)out;
    }

    // 打包发送 0x200 帧
    motors[0].can->tx_buff[0] = (currents[0] >> 8) & 0xFF;
    motors[0].can->tx_buff[1] = (currents[0]) & 0xFF;
    motors[0].can->tx_buff[2] = (currents[1] >> 8) & 0xFF;
    motors[0].can->tx_buff[3] = (currents[1]) & 0xFF;
    motors[0].can->tx_buff[4] = (currents[2] >> 8) & 0xFF;
    motors[0].can->tx_buff[5] = (currents[2]) & 0xFF;
    motors[0].can->tx_buff[6] = (currents[3] >> 8) & 0xFF;
    motors[0].can->tx_buff[7] = (currents[3]) & 0xFF;

    CANTransmit(motors[0].can, 2);
}

/* -------------------- CAN反馈回调 -------------------- */
void M3508_Callback(CANInstance *instance)
{
    M3508_t *motor = (M3508_t *)instance->id;
    uint8_t *d = instance->rx_buff;

    motor->feedback.speed_rpm     = (int16_t)((d[2] << 8) | d[3]);
    motor->feedback.given_current = (int16_t)((d[4] << 8) | d[5]);
    motor->feedback.temp          = d[6];
}
