//
// Created by Administrator on 25-9-17.
//

#include "usb.h"

int _write(const int file, char *ptr, const int len)
{
    (void)file;
    const uint32_t start = HAL_GetTick();
    while (1)
    {
        const uint32_t timeout = 500;
        const uint8_t res = CDC_Transmit_FS((uint8_t *) ptr, len);
        if (res == 0) return len;
        if ((HAL_GetTick() - start) > timeout) break;
        HAL_Delay(1);
    }
    return -1;
}
