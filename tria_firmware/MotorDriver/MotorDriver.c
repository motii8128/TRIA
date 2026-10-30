#include <stdio.h>
#include "pico/stdlib.h"

// ユーザー設定がdefineされてる
#include "user_config.h"

// CAN IDをLEDにより可視化するモジュール
#include "id_visualizer.h"

// スイッチの状況も見ながらPWM出力を行うモジュール
#include "pwm_manager.h"

// ロータリーエンコーダ(AMT102-V)を管理するモジュール
#include "amt102_v_manager.h"

// CAN通信を行うhkdt_picoのライブラリ
#include "can.h"

int main()
{
    sleep_ms(1000);
    stdio_init_all();

    canbus_setup(CAN_TX_PIN, CAN_RX_PIN, CAN_BIT_RATE);

    ID_Visualizer visualizer = init_id_visualizer(CAN_ID, LED_ENABLE_PIN, LED_DATA_PIN);

    PwmManager pwm_manager = init_pwm_manager(PWM1_PIN, PWM2_PIN, SWITCH1_PIN, SWITCH2_PIN);

    struct can2040_msg recv_msg;
    struct can2040_msg send_msg = {
        0x300+CAN_ID,
        8,
        {0,0,0,0,0,0,0,0}
    };

    absolute_time_t last_time = get_absolute_time();

    int16_t tmp_received = 0;

    while (true) {
        absolute_time_t now = to_ms_since_boot(get_absolute_time());
        update_id_visualizer(&visualizer, now);

        absolute_time_t current_time = get_absolute_time();
        // 前回実行時からの経過時間（マイクロ秒）を計算
        int64_t diff_us = absolute_time_diff_us(last_time, current_time);

        if(diff_us >= 20000)
        {
            bool ret = can_transmit(send_msg);
            if(ret)
            {
                // printf("Push Message\n");
            }

            last_time = delayed_by_us(last_time, 20000);
        }

        if(can_receive(&recv_msg))
        {
            if(recv_msg.id == 0x300)
            {       
                if(CAN_ID == 1)
                {
                    tmp_received = (int16_t)(((uint16_t)recv_msg.data[0] << 8) | (recv_msg.data[1] & 0xFF));
                    printf("%d\n", tmp_received);
                    update_pwm_manager(&pwm_manager, tmp_received);
                }
                if(CAN_ID == 2)
                {
                    tmp_received = (int16_t)(((uint16_t)recv_msg.data[2] << 8) | (recv_msg.data[3] & 0xFF));
                    update_pwm_manager(&pwm_manager, tmp_received);
                }
                if(CAN_ID == 3)
                {
                    tmp_received = (int16_t)(((uint16_t)recv_msg.data[4] << 8) | (recv_msg.data[5] & 0xFF));
                    update_pwm_manager(&pwm_manager, tmp_received);
                }
            }
        }
    }
}
