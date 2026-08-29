/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : lidar_sensor.c
 * @brief          : lidarセンサー(UART4/UART7)の距離データ解析処理
 ******************************************************************************
 */
/* USER CODE END Header */
#include "lidar_sensor.h"

void lidar(void){
static uint16_t last_index4 = 0;
static uint16_t last_index7 = 0;

  // -------------------------------------------------------------
  // 1. UART4 (センサー1) の解析処理
  // -------------------------------------------------------------
  uint16_t current_index4 = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
  while (last_index4 != current_index4) {
      if (rx_dma_buf4[last_index4] == 0x5C) {
          uint16_t idx_lsb = (last_index4 + 1) % DMA_BUF_SIZE;
          uint16_t idx_msb = (last_index4 + 2) % DMA_BUF_SIZE;

          uint16_t temp_dist = (rx_dma_buf4[idx_msb] << 8) | rx_dma_buf4[idx_lsb];
          if (temp_dist <= 20000) {
              distance4 = temp_dist; // 1台目の距離
          }
      }
      last_index4 = (last_index4 + 1) % DMA_BUF_SIZE;
  }

  // -------------------------------------------------------------
  // 2. UART7 (センサー2) の解析処理 ★追加
  // -------------------------------------------------------------
  uint16_t current_index7 = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);
  while (last_index7 != current_index7) {
      if (rx_dma_buf7[last_index7] == 0x5C) {
          uint16_t idx_lsb = (last_index7 + 1) % DMA_BUF_SIZE;
          uint16_t idx_msb = (last_index7 + 2) % DMA_BUF_SIZE;

          uint16_t temp_dist = (rx_dma_buf7[idx_msb] << 8) | rx_dma_buf7[idx_lsb];
          if (temp_dist <= 20000) {
              distance7 = temp_dist; // 2台目の距離
          }
      }
      last_index7 = (last_index7 + 1) % DMA_BUF_SIZE;
  }
}
