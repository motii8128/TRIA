#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "user_config.h"

// CAN通信ライブラリ
#include "can.h"
// ロボマスライブラリ
#include "robomaster.h"

// メッセージが定義されている
#include "tria_message/tria_message.h"

// UDP通信タスクが定義されている
#include "UDP_Task.h"


int main()
{
    stdio_init_all();
    mutex_init(&g_mutex);

    canbus_setup(CAN_TX, CAN_RX, CAN_BIT_RATE);

    struct can2040_msg can_send_msg = {
        .id = 0x200,
        .dlc = 8,
        .data = {0,0,0,0,0,0,0,0}
    };

    tria_CommandPacket command = tria_CommandPacket_init_default;
    tria_SensorPacket sensor = tria_SensorPacket_init_default;
    bool udp_initialized = false;

    multicore_launch_core1(udp_task);

    while (true) 
    {    
        mutex_enter_blocking(&g_mutex);
        command = g_data.command_packet;
        g_data.sensor_packet = sensor;
        udp_initialized = g_data.udp_status;
        mutex_exit(&g_mutex);

        int16_t tmp = command.hand_motor;
        can_send_msg.data[0] = (tmp >> 8) & 0xFF;
        can_send_msg.data[1] = tmp & 0xFF;

        if(can_transmit(can_send_msg))
        {

        }
        struct can2040_msg can_recv_msg;
        if(can_receive(&can_recv_msg))
        {
            RoboMasterSensor rm;
            hp_parse_CANMessage(can_recv_msg.data, &rm);

            // CAN受信キューにデータがある場合
            sensor.can_id = can_recv_msg.id - 0x200;
            sensor.angle = rm.angle;
            sensor.velocity = rm.velocity;
            sensor.torque = rm.torque;
        }
    }
}
