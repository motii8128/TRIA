#ifndef TRIA_MOTOR_DRIVER_PWM_MANAGER_H_
#define TRIA_MOTOR_DRIVER_PWM_MANAGER_H_

#include "pwm.h"

/// @brief モタドラへ入力するPWM信号を管理する
typedef struct
{
    uint sw1_pin;
    uint sw2_pin;
    int pwm1_duty;
    int pwm2_duty;
    PWM pwm1;
    PWM pwm2;
}PwmManager;


PwmManager init_pwm_manager(uint pwm1_pin, uint pwm2_pin, uint sw1_pin, uint sw2_pin)
{
    PWM pwm1 = init_pwm(pwm1_pin, 1000);
    PWM pwm2 = init_pwm(pwm2_pin, 1000);

    // スイッチ１を読むピンの初期化
    gpio_init(sw1_pin);
    gpio_set_dir(sw1_pin, false);

    // スイッチ１を読むピンの初期化
    gpio_init(sw2_pin);
    gpio_set_dir(sw2_pin, false);

    PwmManager manager = {sw1_pin, sw2_pin, 0, 0, pwm1, pwm2};

    return manager;
}

void update_pwm_manager(PwmManager* manager, int duty)
{
    bool sw1_state = gpio_get(manager->sw1_pin);
    bool sw2_state = gpio_get(manager->sw2_pin);

    if(sw1_state && !sw2_state) // SW1のみ押されている
    {
        control_pwm(&manager->pwm1, 999);
    }
    else if(sw2_state && !sw1_state) // SW2のみ押されている
    {
        control_pwm(&manager->pwm2, 999);
    }
    else // それ以外は与えられたduty比で制御する
    {
        if(duty > 0)
        {
            control_pwm(&manager->pwm1, duty);
        }
        else if(duty < 0)
        {
            control_pwm(&manager->pwm2, duty);
        }
        else
        {
            control_pwm(&manager->pwm1, 0);
            control_pwm(&manager->pwm2, 0);
        }
    }
}

#endif