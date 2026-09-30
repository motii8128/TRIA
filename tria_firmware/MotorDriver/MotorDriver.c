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
    canbus_setup(CAN_TX_PIN, CAN_RX_PIN, CAN_BIT_RATE);

    ID_Visualizer visualizer = init_id_visualizer(CAN_ID, LED_ENABLE_PIN, LED_DATA_PIN);

    PwmManager pwm_manager = init_pwm_manager(PWM1_PIN, PWM2_PIN, SWITCH1_PIN, SWITCH2_PIN);

    struct can2040_msg recv_msg;
    struct can2040_msg send_msg = {
        0x200 + CAN_ID,
        8,
        {0,0,0,0,0,0,0,0}
    };

    while (true) {
        absolute_time_t now = to_ms_since_boot(get_absolute_time());
        update_id_visualizer(&visualizer, now);

        bool can_recv_result = can_receive(&recv_msg);
        if(can_recv_result)
        {

        }
        else
        {
            update_pwm_manager(&pwm_manager, 0);
        }
    }
}
