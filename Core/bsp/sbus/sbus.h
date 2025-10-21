#ifndef __SBUS_H
#define __SBUS_H

#include "stm32f4xx_hal.h"   // 根据芯片修改

#define SBUS_FRAME_LEN 25

#define CH1 (rc.channels[0])
#define CH2 (rc.channels[1])
#define CH3 (rc.channels[2])
#define CH4 (rc.channels[3])
#define CH5 (rc.channels[4])
#define CH6 (rc.channels[5])
#define CH7 (rc.channels[6])
#define CH8 (rc.channels[7])
#define CH9 (rc.channels[8])
#define CH10 (rc.channels[9])
#define CH11 (rc.channels[10])
#define CH12 (rc.channels[11])
#define CH13 (rc.channels[12])
#define CH14 (rc.channels[13])
#define CH15 (rc.channels[14])
#define CH16 (rc.channels[15])

typedef struct {
    uint16_t channels[16];
    uint8_t failsafe;
    uint8_t lostFrame;
} SBUS_Data_t;

extern SBUS_Data_t rc;

void SBUS_Init(void);

#endif
