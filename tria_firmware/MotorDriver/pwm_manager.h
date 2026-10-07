#ifndef TRIA_MOTOR_DRIVER_PWM_MANAGER_H_
#define TRIA_MOTOR_DRIVER_PWM_MANAGER_H_

#include "pwm.h"
#include "math.h"

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
    // ２つのPWM出力ピンの初期化
    // gpio_set_drive_strength(pwm1_pin, GPIO_DRIVE_STRENGTH_12MA);
    PWM pwm1 = init_pwm(pwm1_pin, 1000);
    // gpio_set_drive_strength(pwm2_pin, GPIO_DRIVE_STRENGTH_12MA);
    PWM pwm2 = init_pwm(pwm2_pin, 1000);

    // PWM pwm1, pwm2;
    // gpio_init(pwm1_pin);
    // gpio_set_dir(pwm1_pin, true);
    // gpio_set_drive_strength(pwm1_pin, GPIO_DRIVE_STRENGTH_12MA);
    // gpio_init(pwm2_pin);
    // gpio_set_dir(pwm2_pin, true);
    // gpio_set_drive_strength(pwm2_pin, GPIO_DRIVE_STRENGTH_12MA);  
    // pwm1.channel = pwm1_pin;
    // pwm2.channel = pwm2_pin;
    
    // スイッチ１を読むピンの初期化
    gpio_init(sw1_pin);
    gpio_set_dir(sw1_pin, false);

    // スイッチ２を読むピンの初期化
    gpio_init(sw2_pin);
    gpio_set_dir(sw2_pin, false);

    PwmManager manager = {sw1_pin, sw2_pin, 0, 0, pwm1, pwm2};

    return manager;
}

void update_pwm_manager(PwmManager* manager, int duty)
{
    static float x = 0.0;

    bool sw1_state = gpio_get(manager->sw1_pin);
    bool sw2_state = gpio_get(manager->sw2_pin);

    uint sw_duty = 300;

    if(sw1_state && !sw2_state) // SW1のみ押されている
    {
        control_pwm(&manager->pwm1, sw_duty);
        // gpio_put(manager->pwm1.channel, false);
    }
    else if(sw2_state && !sw1_state) // SW2のみ押されている
    {
        control_pwm(&manager->pwm2, sw_duty);
        // gpio_put(manager->pwm2.channel, false);
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
            // gpio_put(manager->pwm1.channel, true);
            // gpio_put(manager->pwm2.channel, true);
        }
    }
}

#endif