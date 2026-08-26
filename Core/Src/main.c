/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sbus.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_BUF_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

CAN_HandleTypeDef hcan1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart7_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

int rx;   // rxスティック
int ly;   // lyスティック
int ry;   // ryスティック
int lx;   // lxスティック

int Lmayu; 
int Rmayu; 
int Ltuno;
int Rtuno;

volatile int m1; 
volatile int m2; 
volatile int m3; 
volatile int m4;

int d1;
int d2;
int d3;
int d4;

volatile int16_t PV1 = 0; // 現在値
volatile int16_t PV2 = 0; // 現在値
volatile int16_t PV3 = 0; // 現在値
volatile int16_t PV4 = 0; // 現在値
volatile int16_t PV5 = 0; // 現在値
volatile int16_t PV6 = 0; // 現在値

int dir1 = 0;
int dir2 = 0;
int dir3 = 0;
int dir4 = 0;

int pwm1 = 0;
int pwm2 = 0;
int pwm3 = 0;
int pwm4 = 0;
int pwm5 = 0;
int pwm6 = 0;
int pwm7 = 0;
int pwm8 = 0;

int maxpwm = (255 * 7) / 10; // 最大PWM値を0.7倍にして減速させる
//int test;

int stop_flag = 0;
int timer_flag = 0;
int reset_flag = 0;
int set_flag = 0;

int roller_dir = 0; // ローラーの回転方向を保持する変数

int dummy = 0;

//lidar
// UART4 用（センサー1）
uint8_t rx_dma_buf4[DMA_BUF_SIZE]; 
uint16_t distance4 = 0;

// UART7 用（センサー2）
uint8_t rx_dma_buf7[DMA_BUF_SIZE]; 
uint16_t distance7 = 0;

// 自動モード時の仮想スティック出力
int auto_ly = 0; // 左右移動
int auto_rx = 0; // 旋回

//timercount
uint32_t time1 = 0;
uint32_t time2 = 0;
uint32_t now = 0;

uint32_t last_can_rx = 0;

// map関数
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (x < in_min)
        x = in_min;
    if (x > in_max)
        x = in_max;

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// SBUS

uint8_t sbus_rxbuf[SBUS_FRAME_LEN];
uint8_t sbus_frame[SBUS_FRAME_LEN];
volatile uint16_t SBUS_CH[16];
uint8_t SBUS_Failsafe = 0;
uint8_t SBUS_LostFrame = 0;

void SBUS_Process(void);

/* ===================== */
/* 初期化                */
/* ===================== */
void SBUS_Init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rxbuf, SBUS_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

/* ===================== */
/* 受信イベント          */
/* ===================== */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART5) {
        for (int i = 0; i < Size - 24; i++) {
            if (sbus_rxbuf[i] == 0x0F) {
                memcpy(sbus_frame, &sbus_rxbuf[i], 25);
                SBUS_Process();
                break;
            }
        }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rxbuf, SBUS_FRAME_LEN);
}
/* ===================== */
/* SBUSデコード          */
/* ===================== */
void SBUS_Process(void) {
    //@brief  The application entry point.
    // if(sbus_frame[0] != 0x0F)
    //    return;

    SBUS_CH[0] = (sbus_frame[1] | sbus_frame[2] << 8) & 0x07FF;
    SBUS_CH[1] = (sbus_frame[2] >> 3 | sbus_frame[3] << 5) & 0x07FF;
    SBUS_CH[2] = (sbus_frame[3] >> 6 | sbus_frame[4] << 2 | sbus_frame[5] << 10) & 0x07FF;
    SBUS_CH[3] = (sbus_frame[5] >> 1 | sbus_frame[6] << 7) & 0x07FF;
    SBUS_CH[4] = (sbus_frame[6] >> 4 | sbus_frame[7] << 4) & 0x07FF;
    SBUS_CH[5] = (sbus_frame[7] >> 7 | sbus_frame[8] << 1 | sbus_frame[9] << 9) & 0x07FF;
    SBUS_CH[6] = (sbus_frame[9] >> 2 | sbus_frame[10] << 6) & 0x07FF;
    SBUS_CH[7] = (sbus_frame[10] >> 5 | sbus_frame[11] << 3) & 0x07FF;
    SBUS_CH[8] = (sbus_frame[12] | sbus_frame[13] << 8) & 0x07FF;
    SBUS_CH[9] = (sbus_frame[13] >> 3 | sbus_frame[14] << 5) & 0x07FF;
    SBUS_CH[10] = (sbus_frame[14] >> 6 | sbus_frame[15] << 2 | sbus_frame[16] << 10) & 0x07FF;
    SBUS_CH[11] = (sbus_frame[16] >> 1 | sbus_frame[17] << 7) & 0x07FF;
    SBUS_CH[12] = (sbus_frame[17] >> 4 | sbus_frame[18] << 4) & 0x07FF;
    SBUS_CH[13] = (sbus_frame[18] >> 7 | sbus_frame[19] << 1 | sbus_frame[20] << 9) & 0x07FF;
    SBUS_CH[14] = (sbus_frame[20] >> 2 | sbus_frame[21] << 6) & 0x07FF;
    SBUS_CH[15] = (sbus_frame[21] >> 5 | sbus_frame[22] << 3) & 0x07FF;

    SBUS_LostFrame = (sbus_frame[23] >> 2) & 0x01;
    SBUS_Failsafe = (sbus_frame[23] >> 3) & 0x01;
}

// CAN
 void CAN_TX(uint32_t recipient) {
     //送信用インスタンス等
 	CAN_TxHeaderTypeDef TxHeader;
 	uint32_t TxMailbox;
 	uint8_t TxData[8];
     //送信メールボックスに空きがあったら送信開始
 	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) {
         //送信用インスタンスの設定
 		TxHeader.StdId = recipient;// 受取手のCANのID
 		TxHeader.RTR = CAN_RTR_DATA;
 		TxHeader.IDE = CAN_ID_STD;
 		TxHeader.DLC = 8;//データ長を8byteに設定
 		TxHeader.TransmitGlobalTime = DISABLE;
         //各データ
 		TxData[0] = 1;
 		TxData[1] = 0;
 		TxData[2] = 0;
 		TxData[3] = 0;
 		TxData[4] = 0;
 		TxData[5] = 0;
 		TxData[6] = 0;
 		TxData[7] = 0;
         //CANメッセージを送信
 		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
 != HAL_OK) {Error_Handler();
 		}
 	}
 }
// RX割り込みコールバック関数
volatile uint8_t use_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    CAN_RxHeaderTypeDef RxHeader; // 受信メッセージの情報が格納されるインスタンス
    uint8_t RxData[8];            // 受信したデータを一時保存する配列
    uint32_t id;                  // CANメッセージIDを格納する変数
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        id = RxHeader.StdId; // RxHeaderの中に入っているidを取り出す
        if (id == 0x001 && RxHeader.DLC >= 8) { // idが0x001でデータ長が8以上の場合
            last_can_rx = HAL_GetTick();          // 受信時刻を更新
            for (int i = 0; i <= 7; i++) {
                use_data[i] = RxData[i];
            }
        }
    }
}




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_ETH_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */

// if(sbus_frame[0] != 0x0F)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor_control(int SV,int PV,int maxMV ,int down_pwm,int max_pwm,int *pwmm,int *dirr) {

    int error = 0;
    int MV = 0;
    int lastMV = 0;
    int pwm = *pwmm; //前回の値を保持
    int target_dir = 0;
    int dir = *dirr; //前回の値を保持

    // リミッター処理
    if (SV > max_pwm) {
        SV = max_pwm;
    }else if (SV < -max_pwm) {
        SV = -max_pwm;
    }

    // dir設定と値を絶対値にしている

    if (SV < 0) {
        target_dir = 0;
        SV = -SV;
    } else if (SV > 0) {
        target_dir = 1;
    }
    error = SV - PV;

    MV =  error / 10;//ki=0.1

    if (MV > maxMV) {
        lastMV = maxMV;
    } else if (MV < -maxMV) {
        lastMV = -maxMV;
    } else {
        lastMV = MV;
    }

    // モーターの回転方向が目標と異なる場合一旦pwmが0になってから回転方向を変える
    if (SV != 0) {
        if (dir != target_dir) {
            if (pwm > down_pwm) {
                pwm -= down_pwm;
            } else {
                pwm = 0;
                dir = target_dir;
            }
        } else {
            pwm += lastMV;
            if (pwm < 0) {
                pwm = 0;
            }
        }
    }
    // PWMの値が0~maxpwmの範囲を超えないようにする
    if (pwm > max_pwm) {
        pwm = max_pwm;
    } else if (pwm < 0) {
        pwm = 0;
    }
    // コントローラーの値が0の時に緩やかにモーターの停止
    if (SV == 0) {
        if (pwm > down_pwm) {
          pwm -= down_pwm;
        } else {
          pwm = 0;
        }
    }

    // pwmの値を正の値に変換
    pwm = abs(pwm);
    *pwmm = pwm;
    *dirr = dir;
}

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

// SBUS用のヘルパー関数
int get_switch_state(int ch_value) {
    if (ch_value > 1100) {
      return -1;
    }
    if (ch_value < 300){
      return 1;
    } 
    return 0;
}

int process_stick(int ch_value) {
    if (ch_value > 1000 && ch_value < 1050){
      ch_value = 1024;
    } 
    int mapped = map(ch_value, 368, 1680, -255, 255);
    if (mapped <= 2 && mapped >= -2){
      return 0; // デッドバンド
    } 
    return mapped;
}

void sbus(void){
    Lmayu = get_switch_state(SBUS_CH[4]);
    Rmayu = get_switch_state(SBUS_CH[5]);
    Ltuno = get_switch_state(SBUS_CH[6]);
    Rtuno = get_switch_state(SBUS_CH[7]);

    rx = process_stick(SBUS_CH[0]);
    ly = process_stick(SBUS_CH[1]);
    ry = process_stick(SBUS_CH[2]);
    lx = process_stick(SBUS_CH[3]);

    m1 =  ly - lx - rx; // 左前 (Front Left)
    m2 = -ly - lx - rx; // 右前 (Front Right)
    m3 = -ly + lx - rx; // 左後 (Rear Left)
    m4 =  ly + lx - rx; // 右後 (Rear Right)

    // モーターの値を0.7倍して減速させる
    m1 = (m1 * 7) / 10;
    m2 = (m2 * 7) / 10;
    m3 = (m3 * 7) / 10;
    m4 = (m4 * 7) / 10;
    }

    // マジックナンバーを意味のある定数に置き換えます
    const int ROLLER_SPEED = 240;
    const int ROLLER_STOP = 0;
    const int ROLLER_SPIN_NORMAL_PWM = 80;
    const int ROLLER_SPIN_REVERSE_PWM = 150;

    void roller(void){
        switch (Lmayu) {
            case 1: 
                if ((Ltuno == 1 && stop_flag == 0) || Ltuno == -1) {
                    roller_dir = 0; // 正転
                    pwm7 = ROLLER_SPIN_NORMAL_PWM;
                    motor_control(ROLLER_SPEED, PV5, 5, 5, 255, &pwm5, &dummy);
                    motor_control(ROLLER_SPEED, PV6, 5, 5, 255, &pwm6, &dummy);
                } else { // Ltuno == 0 または (Ltuno == 1 && stop_flag == 1)
                    pwm7 = 0;
                    motor_control(ROLLER_STOP, PV5, 5, 5, 230, &pwm5, &dummy);
                    motor_control(ROLLER_STOP, PV6, 5, 5, 230, &pwm6, &dummy);
                }
                break;

            case 0: 
                stop_flag = 0;
                timer_flag = 0;
                pwm7 = 0;
                motor_control(ROLLER_SPEED, PV5, 5, 5, 255, &pwm5, &dummy);
                motor_control(ROLLER_SPEED, PV6, 5, 5, 255, &pwm6, &dummy);
                break;

          case -1:
                stop_flag = 0;
                timer_flag = 0;
                motor_control(ROLLER_STOP, PV5, 5, 5, 230, &pwm5, &dummy);
                motor_control(ROLLER_STOP, PV6, 5, 5, 230, &pwm6, &dummy);

                if (reset_flag == 1 && set_flag == 0) {
                    pwm7 = ROLLER_SPIN_REVERSE_PWM;
                    roller_dir = 1; // 逆転
                } else {
                    pwm7 = 0;
                    if (reset_flag == 1 && set_flag == 1) {
                        reset_flag = 0;
                        set_flag = 0;
                    }
                }
                break;
        }
    }


 void auto_mode(int distance1, int distance2, int reset_flag ,int target_dist) {
  typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
} PID;
// 距離用(横移動)と角度用(旋回)のPID実体を作成（ゲインは実機で要調整）
static PID distance  = {1.3, 0.008, 0.05, 0, 0}; 
static PID angle = {1.3, 0.01, 0.2, 0, 0};
    float target_distance = target_dist; // 目標距離 (mm)
    float dt = 0.02; // 20ms周期

    // --- 距離（平均）と角度（差分）の計算 ---
    float current_dist = (distance1 + distance2) / 2.0;//現在の距離
    float error_dist = current_dist - target_distance; // 距離のズレ
    float error_angle = distance1 - distance2;        // 角度のズレ

    // --- モード切替時のリセット処理 ---
    if (reset_flag == 1) {
        distance.Ki = 0;   
        distance.prev_error = error_dist;
        angle.Ki = 0;  
        angle.prev_error = error_angle;
        auto_ly = 0; auto_rx = 0;
        return;
    }

    // --- 1. 距離を保つためのPID（縦移動力 ly を計算） ---
    distance.integral += error_dist * dt;
    if (distance.integral > 2000) distance.integral = 2000;   // 暴走防止
    if (distance.integral < -2000) distance.integral = -2000;
    
    float derivative_dist = (error_dist - distance.prev_error) / dt;
    
    // ※ 符号は実機の「mae移動がプラスかマイナスか」に合わせて反転させてください
    auto_ly = (int)((distance.Kp * error_dist) + (distance.Ki * distance.integral) + (distance.Kd * derivative_dist));
    distance.prev_error = error_dist;

    // --- 2. 平行にするためのPID（旋回力 rx を計算） ---
    angle.integral += error_angle * dt;
    if (angle.integral > 2000) angle.integral = 2000; // 暴走防止
    if (angle.integral < -2000) angle.integral = -2000;
    
    float derivative_angle = (error_angle - angle.prev_error) / dt;
    
    // ※ 符号は実機の「右旋回がプラスかマイナスか」に合わせて反転させてください
    auto_rx = (int)((angle.Kp * error_angle) + (angle.Ki * angle.integral) + (angle.Kd * derivative_angle));
    angle.prev_error = error_angle;
}

void safety(void) {
  int sbus_error = 0;
  int can_error = 0;
  uint8_t blink_state = (now / 300) % 2;

  if(SBUS_CH[0] == 0 || SBUS_LostFrame){
    sbus_error = 1;
  }else{
    sbus_error = 0;
  }
  if(HAL_GetTick() - last_can_rx > 100){
    can_error = 1;
  }else{
    can_error = 0;
  }
    // SBUSの値とCANが来ていない場合、モーターを停止
    if (sbus_error == 1 || can_error == 1) {
        pwm1 = 0;
        pwm2 = 0;
        pwm3 = 0;
        pwm4 = 0;
        pwm5 = 0;
        pwm6 = 0;
        pwm7 = 0;
        pwm8 = 0;
    } else if(sbus_error == 1 && can_error == 1){
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);//green
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);//blue
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);//red
    }
    //ローラーと足回りが同時に動かないようにする
    if(pwm5 > 0 || pwm6 > 0){
      pwm1 = 0;
      pwm2 = 0;
      pwm3 = 0;
      pwm4 = 0;
    }

    if(sbus_error == 1 ){//SBUSが来ていない場合青点滅
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,0);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,blink_state);
    }
    if(can_error == 1){//CANが来ていない場合赤点滅
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, blink_state);
}
}

  

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

    setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_ETH_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
    HAL_CAN_Start(&hcan1); // CANstart
    HAL_CAN_ActivateNotification(&hcan1,
    CAN_IT_RX_FIFO0_MSG_PENDING); // 割り込み有効　

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


    SBUS_Init(); // ★【修正】SBUSの受信を開始

  HAL_Delay(100); // センサーの起動待ち

  // 1. センサーに「Start Reading（測定開始）」のコマンドだけを送る
  uint8_t startCmd[] = {0x5A, 0x0A, 0x02, 0x02, 0x00, 0xF1};
  // センサー1 (UART4) に送信 & DMAスタート
  HAL_UART_Transmit(&huart4, startCmd, sizeof(startCmd), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart7, startCmd, sizeof(startCmd), HAL_MAX_DELAY);
  HAL_Delay(20);
  HAL_UART_Receive_DMA(&huart4, rx_dma_buf4, DMA_BUF_SIZE);
  HAL_UART_Receive_DMA(&huart7, rx_dma_buf7, DMA_BUF_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
      now = HAL_GetTick();
        PV1 = use_data[0];
        PV2 = use_data[1];
        PV3 = use_data[2];
        PV4 = use_data[3];
        PV5 = use_data[4];
        PV6 = use_data[5];

        sbus();//SBUSの値の加工

        lidar();//lidarの値の読み取り
      
        if(HAL_GPIO_ReadPin(lock6_GPIO_Port, lock6_Pin) == 0){
          stop_flag = 1;
        }
        if(HAL_GPIO_ReadPin(lock7_GPIO_Port, lock7_Pin) == 0){
          reset_flag = 1;
        }
        if(HAL_GPIO_ReadPin(lock8_GPIO_Port, lock8_Pin) == 0){
          set_flag = 1;
        }

    
    // 足回り
   if (now - time1 >= 20) {
      if(Rtuno == 1) {
        // 手動モード
        auto_mode(distance4 ,distance7, 1,1000); // ★追加：裏でPIDの記憶をリセットしておく
        motor_control(m1, PV1, 40, 40, maxpwm, &pwm1, &dir1);
        motor_control(m2, PV2, 40, 40, maxpwm, &pwm2, &dir2);
        motor_control(m3, PV3, 40, 40, maxpwm, &pwm3, &dir3);
        motor_control(m4, PV4, 40, 40, maxpwm, &pwm4, &dir4);
      } else if(Rtuno == 0){
        auto_mode(distance4 - 11, distance7 - 38, 0,(distance4  + distance7 -49) / 2); 
        int gauto_m1 =  ly - lx + auto_rx;
        int gauto_m2 = -ly - lx + auto_rx;
        int gauto_m3 = -ly + lx + auto_rx;
        int gauto_m4 =  ly + lx + auto_rx;

        motor_control(gauto_m1, PV1, 40, 40, maxpwm, &pwm1, &dir1);
        motor_control(gauto_m2, PV2, 40, 40, maxpwm, &pwm2, &dir2);
        motor_control(gauto_m3, PV3, 40, 40, maxpwm, &pwm3, &dir3);
        motor_control(gauto_m4, PV4, 40, 40, maxpwm, &pwm4, &dir4);

      }else if(Rtuno == -1) {
        // 自動モード
        auto_mode(distance4 + -11, distance7 - 38, 0,500); // ★修正：通常計算

        // 自動計算された ly, rx を使って m1〜m4 を計算（あなたの式を再利用！）
        int auto_m1 =  -auto_ly - lx + auto_rx;
        int auto_m2 =   auto_ly - lx + auto_rx;
        int auto_m3 =   auto_ly + lx + auto_rx;
        int auto_m4 =  -auto_ly + lx + auto_rx;

        // 計算結果をモーターに出力
        motor_control(auto_m1, PV1, 40, 40, maxpwm, &pwm1, &dir1);
        motor_control(auto_m2, PV2, 40, 40, maxpwm, &pwm2, &dir2);
        motor_control(auto_m3, PV3, 40, 40, maxpwm, &pwm3, &dir3);
        motor_control(auto_m4, PV4, 40, 40, maxpwm, &pwm4, &dir4);
      }

      roller();

      time1 = now;
    }

    // 電磁弁
    if (Rmayu == 0) {
        HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 0);
        HAL_GPIO_WritePin(lock4_GPIO_Port, lock4_Pin, 0);
    } else if (Rmayu == 1) {
        HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 1);
        HAL_GPIO_WritePin(lock4_GPIO_Port, lock4_Pin, 0);
    } else if (Rmayu == -1) {
        HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 0);
        HAL_GPIO_WritePin(lock4_GPIO_Port, lock4_Pin, 1);
    }

    safety(); // 安全機能の呼び出し



    // デバッグ用のprintf、100msごとに出力
    uint32_t time = 0;
    if (HAL_GetTick() - time >= 100) {
      //printf("data0:%d data1:%d data2:%d data3:%d data4:%d data5:%d data6:%d data7:%d\n", use_data[0], use_data[1], use_data[2], use_data[3], use_data[4], use_data[5], use_data[6], use_data[7]); 
       //printf("0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d\n", sbus0, sbus1, sbus2, sbus3 ,SBUS_CH[4], SBUS_CH[5], SBUS_CH[6], SBUS_CH[7]);
      // printf("PV1:%d PV2:%d  PV3:%d PV4:%d m1:%d m2:%d m3:%d m4:%d lastMV1:%.1f lastMV2:%.1f lastMV3:%.1f lastMV4:%.1f\n", PV1, PV2, PV3, PV4, m1, m2, m3, m4, (float)lastMV1, (float)lastMV2, (float)lastMV3, (float)lastMV4);
      // printf("m1:%d d1:%d m2:%d d2:%d m3:%d d3:%d m4:%d d4:%d ltsw:%d rtsw:%d pwm1 %d\n", m1, d1, m2, d2, m3, d3, m4, d4,ltsw,rtsw,pwm1);
      //printf("stop_flag: %d\n", stop_flag);
      printf("distance4: %d distance7: %d\n", distance4, distance7);
      time = HAL_GetTick(); // 時間更新
    }

    //motor
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm1); // m1 LF
    HAL_GPIO_WritePin(d3_GPIO_Port, d3_Pin, dir1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm2); // m2 RF
    HAL_GPIO_WritePin(d4_GPIO_Port, d4_Pin, dir2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm3); // m3 LB
    HAL_GPIO_WritePin(d1_GPIO_Port, d1_Pin, dir3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm4); // m4  RB
    HAL_GPIO_WritePin(d2_GPIO_Port, d2_Pin, dir4);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm5); // m5 ue
    HAL_GPIO_WritePin(d7_GPIO_Port, d7_Pin, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm6); // m6 sita
    HAL_GPIO_WritePin(d8_GPIO_Port, d8_Pin, 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm7); // m7 souten
    HAL_GPIO_WritePin(d5_GPIO_Port, d5_Pin, roller_dir);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm8); // m8
    HAL_GPIO_WritePin(d6_GPIO_Port, d6_Pin, 1);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef filter;
  filter.FilterIdHigh = 0x001 << 5;               // フィルターID1
  filter.FilterIdLow = 0x002 << 5;                // フィルターID2
  filter.FilterMaskIdHigh = 0x003 << 5;           // フィルターID3
  filter.FilterMaskIdLow = 0x004 << 5;            // フィルターID4
  filter.FilterScale = CAN_FILTERSCALE_16BIT;     // 16モード
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0へ格納
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST; // IDリストモード
  filter.SlaveStartFilterBank = 14;
  filter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &filter);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 254;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 254;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 460800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 100000;
  huart5.Init.WordLength = UART_WORDLENGTH_9B;
  huart5.Init.StopBits = UART_STOPBITS_2;
  huart5.Init.Parity = UART_PARITY_EVEN;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 460800;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, d5_Pin|d7_Pin|d8_Pin|d6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|d1_Pin|d2_Pin|LD3_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(d3_GPIO_Port, d3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(d4_GPIO_Port, d4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, lock1_Pin|lock4_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : d5_Pin d7_Pin d8_Pin d6_Pin */
  GPIO_InitStruct.Pin = d5_Pin|d7_Pin|d8_Pin|d6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin d1_Pin d2_Pin LD3_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|d1_Pin|d2_Pin|LD3_Pin
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : d3_Pin */
  GPIO_InitStruct.Pin = d3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(d3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : d4_Pin */
  GPIO_InitStruct.Pin = d4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(d4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : lock1_Pin lock4_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = lock1_Pin|lock4_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : lock6_Pin lock7_Pin */
  GPIO_InitStruct.Pin = lock6_Pin|lock7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : lock8_Pin */
  GPIO_InitStruct.Pin = lock8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(lock8_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 10);
  return len;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
