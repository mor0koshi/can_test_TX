#ifndef __LIDAR_SENSOR_H
#define __LIDAR_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define DMA_BUF_SIZE 128

/* ペリフェラルハンドル (main.c で定義) */
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;

/* lidarセンサーの受信バッファ・距離値 (main.c で定義) */
extern uint8_t rx_dma_buf4[DMA_BUF_SIZE];
extern uint16_t distance4;

extern uint8_t rx_dma_buf7[DMA_BUF_SIZE];
extern uint16_t distance7;

/* 関数プロトタイプ */
void lidar(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIDAR_SENSOR_H */
