#include "sbus.h"
#include "usart.h"   // CubeMX 生成的 USART1 句柄在这里

// ================== 内部缓冲区 ==================
uint8_t sbus_dma_buf[SBUS_FRAME_LEN];   // DMA 接收缓存
SBUS_Data_t rc;                         // 全局 RC 数据

// ================== CRC / Frame 解析 ==================
static void SBUS_ProcessFrame(uint8_t *frame)
{
    if (frame[0] != 0x0F) return;   // 帧头校验
    if (frame[24] != 0x00 && frame[24] != 0x04 &&
        frame[24] != 0x14 && frame[24] != 0x24) return; // 帧尾常见值

    rc.channels[0]  = ((frame[1]    | frame[2] << 8) & 0x07FF);
    rc.channels[1]  = ((frame[2]>>3 | frame[3] << 5) & 0x07FF);
    rc.channels[2]  = ((frame[3]>>6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
    rc.channels[3]  = ((frame[5]>>1 | frame[6] << 7) & 0x07FF);
    rc.channels[4]  = ((frame[6]>>4 | frame[7] << 4) & 0x07FF);
    rc.channels[5]  = ((frame[7]>>7 | frame[8] << 1 | frame[9] << 9) & 0x07FF);
    rc.channels[6]  = ((frame[9]>>2 | frame[10] << 6) & 0x07FF);
    rc.channels[7]  = ((frame[10]>>5 | frame[11] << 3) & 0x07FF);

    rc.channels[8]  = ((frame[12]   | frame[13] << 8) & 0x07FF);
    rc.channels[9]  = ((frame[13]>>3| frame[14] << 5) & 0x07FF);
    rc.channels[10] = ((frame[14]>>6| frame[15] << 2 | frame[16] << 10) & 0x07FF);
    rc.channels[11] = ((frame[16]>>1| frame[17] << 7) & 0x07FF);
    rc.channels[12] = ((frame[17]>>4| frame[18] << 4) & 0x07FF);
    rc.channels[13] = ((frame[18]>>7| frame[19] << 1 | frame[20] << 9) & 0x07FF);
    rc.channels[14] = ((frame[20]>>2| frame[21] << 6) & 0x07FF);
    rc.channels[15] = ((frame[21]>>5| frame[22] << 3) & 0x07FF);

    rc.lostFrame = (frame[23] >> 2) & 0x01;
    rc.failsafe  = (frame[23] >> 3) & 0x01;
}

// ================== 初始化 ==================
void SBUS_Init(void)
{
    // 开启 DMA+IDLE 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, sbus_dma_buf, SBUS_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); // 关闭半传输中断，避免干扰
}

// ================== UART 回调 ==================
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (Size == SBUS_FRAME_LEN && sbus_dma_buf[0] == 0x0F) {
            SBUS_ProcessFrame(sbus_dma_buf);
        }

        // 重新打开接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, sbus_dma_buf, SBUS_FRAME_LEN);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
