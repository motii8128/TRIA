#ifndef TRIA_MOTOR_DRIVER_USER_CONFIG_H_
#define TRIA_MOTOR_DRIVER_USER_CONFIG_H_

#include "pico/stdlib.h"
#include "ws2812.h"

// CAN通信設定
#define CAN_ID 2
#define CAN_TX_PIN 6
#define CAN_RX_PIN 7
#define CAN_BIT_RATE 1000000

#define PWM1_PIN 4
#define PWM2_PIN 3

#define SWITCH1_PIN 27
#define SWITCH2_PIN 26

#define LED_ENABLE_PIN 11
#define LED_DATA_PIN 12


#endif