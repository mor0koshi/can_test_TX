/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : function.c
 * @brief          : main.c から分離したユーザー定義関数
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"
#include <stdint.h>
#include <stdlib.h>

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 10);
  return len;
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

// マジックナンバーを意味のある定数に置き換えます
static const int ROLLER_SPEED = 700;
static const int BAKETU_ROLLER_SPEED = 450;
static const int ROLLER_STOP = 0;
static const int ROLLER_SPIN_NORMAL_PWM = 300;
static const int ROLLER_SPIN_REVERSE_PWM = 500;

uint32_t time3 = 0;
void roller(void){
    switch (Lmayu) {
        case 1:
            if ((Ltuno == 1 && stop_flag == 0) ) {
                roller_dir = 0; // 正転
                pwm7 = ROLLER_SPIN_NORMAL_PWM;
                motor_control(ROLLER_SPEED, PV5, 20, 20, 900, &pwm5, &dummy);
                motor_control(ROLLER_SPEED, PV6, 20, 20, 900, &pwm6, &dummy);
            } else if((Ltuno == -1 && stop_flag == 0) ) {
                roller_dir = 0; // 正転
                pwm7 = ROLLER_SPIN_NORMAL_PWM;
                motor_control(BAKETU_ROLLER_SPEED, PV5, 20, 20, 900, &pwm5, &dummy);
                motor_control(BAKETU_ROLLER_SPEED, PV6, 20, 20, 900, &pwm6, &dummy);

            } else if((Ltuno == 0 && stop_flag == 0) ) {
                pwm7 = 0;
                motor_control(ROLLER_STOP, PV5, 20, 20, 900, &pwm5, &dummy);
                motor_control(ROLLER_STOP, PV6, 20, 20, 900, &pwm6, &dummy);
            } else if(stop_flag == 1){
                if(timer_flag == 0){
                    timer_flag  = 1;
                    time3 = now;
                }if(now-time3 >= 800){
                pwm7 = 0;
                motor_control(ROLLER_STOP, PV5, 20, 20, 900, &pwm5, &dummy);
                motor_control(ROLLER_STOP, PV6, 20, 20, 900, &pwm6, &dummy);
                }   
            }
            break;

        case 0:
            stop_flag = 0;
            timer_flag = 0;
            pwm7 = 0;
        if(Ltuno == 1){
             motor_control(ROLLER_SPEED, PV5, 20, 20, 900, &pwm5, &dummy);
            motor_control(ROLLER_SPEED, PV6, 20, 20, 900, &pwm6, &dummy);
        }

            else if(Ltuno == -1  ) {
                roller_dir = 0; // 正転
                motor_control(BAKETU_ROLLER_SPEED, PV5, 20, 20, 900, &pwm5, &dummy);
                motor_control(BAKETU_ROLLER_SPEED, PV6, 20, 20, 900, &pwm6, &dummy);

            }
            break;

      case -1:
            stop_flag = 0;
            timer_flag = 0;
            motor_control(ROLLER_STOP, PV5, 20, 20, 900, &pwm5, &dummy);
            motor_control(ROLLER_STOP, PV6, 20, 20, 900, &pwm6, &dummy);
            break;     
    }
    if (reset_flag == 1 && set_flag == 0) {
        pwm7 = ROLLER_SPIN_REVERSE_PWM;
        roller_dir = 1; // 逆転
       } else if(set_flag == 1 && reset_flag ==  1){
        pwm7 = 0;
        reset_flag = 0;
        set_flag = 0;
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
static PID angle = {0.6, 0.01, 0.2, 0, 0};
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
