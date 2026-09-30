#ifndef TRIA_MOTOR_DRIVER_ID_VISUALIZER_H_
#define TRIA_MOTOR_DRIVER_ID_VISUALIZER_H_

#include "pico/stdlib.h"
#include "ws2812.h"

/// @brief CAN通信におけるIDをLEDで可視化する
typedef struct {
    WS2812 led;
    uint id;
    bool state;
    uint32_t interval_ms;
    uint32_t last_toggle_ms;
}ID_Visualizer;

ID_Visualizer init_id_visualizer(uint can_id, uint led_enable_pin, uint led_data_pin)
{
    gpio_init(led_enable_pin);
    gpio_set_dir(led_enable_pin, true);
    gpio_put(led_enable_pin, true);

    WS2812 internal_led;
    // WS2812にはPIO1を使用する(CAN通信でPIO0を用いるから)
    init_ws2812_pio(&internal_led, 1, led_data_pin);


    int interval_ms = 500 / can_id;

    ID_Visualizer id_v = {internal_led, can_id, false, interval_ms, 0};

    return id_v;
}

void update_id_visualizer(ID_Visualizer* visualizer, uint32_t current_time_ms)
{
    // 1秒(1000ms)の中での経過時間を計算 (0 ~ 999ms)
    uint32_t phase = current_time_ms % 1500;

    // 後半500ms（500ms ~ 999ms）は強制消灯
    if (phase >= 1000) {
        if (visualizer->state) {
            visualizer->state = false;
            control_ws2812(&visualizer->led, 0, 0, 0);
        }
        return;
    }

    uint32_t step = (phase * (2 * visualizer->id)) / 1000;
    bool new_state = (step % 2 == 0);

    if(visualizer->state != new_state)
    {
        visualizer->state = new_state;

        if(visualizer->state)
        {
            control_ws2812(&visualizer->led, 0, 64, 0);
        }
        else  
        {
            control_ws2812(&visualizer->led, 0, 0, 0);
        }
    }
}

#endif