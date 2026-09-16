#ifndef UDP_TASK_H_
#define UDP_TASK_H_

#include "user_config.h"

#include <pico/sync.h> /* ２つのスレッド同士が変数を共有するのに必要 */
#include "w6300_ethernet.h" /* W6300を用いたEthernet通信に必要 */
#include "tria_message/tria_message.h" /* 通信に用いるパケットが記述されている */

// コア間で共有するデータ構造体
typedef struct{
    tria_CommandPacket command_packet;
    tria_SensorPacket sensor_packet;
    bool udp_status;
}SharedData;


mutex_t g_mutex;
SharedData g_data = {.command_packet = tria_CommandPacket_init_default, .sensor_packet = tria_SensorPacket_init_default, .udp_status = false};



void udp_task(void)
{
    int socket_num = 0;

    int udp_miss_count = 0;

    uint8_t my_ip[4] = MY_IP_ADDR;
    uint16_t my_port = MY_PORT;
    uint8_t dest_ip[4] = DEST_IP_ADDR;
    uint16_t dest_port = DEST_PORT;

    uint8_t gateway[4] = GATE_WAY;

    bool initalized = false;

    tria_CommandPacket command = tria_CommandPacket_init_default;
    tria_SensorPacket sensor = tria_SensorPacket_init_default;

    for(;;)
    {
        // データ共有は一括で行う
        mutex_enter_blocking(&g_mutex);
        sensor = g_data.sensor_packet;
        g_data.command_packet = command;
        g_data.udp_status = initalized;
        mutex_exit(&g_mutex);

        if(!initalized)
        {
            if(initialize_w6300_ethernet(socket_num, my_ip, gateway, my_port) == socket_num)
            {
                // 初期化に成功した場合
                initalized = true;
            }
            else
            {
                // 初期化に失敗した場合
                initalized = false;
                sleep_ms(500);
            }
        }
        else
        {
            // 受信処理から行う
            uint8_t recv_buffer[RECV_BUFFER_SIZE];
            
            int recv_size = recv_w6300_udp(socket_num, recv_buffer, RECV_BUFFER_SIZE);
            if(recv_size > 0)
            {
                // データ受信に成功した場合
                command = decode_command_packet(recv_buffer, recv_size);
            }


            // 送信処理を行う
            uint8_t send_buffer[128];

            int send_size = encode_sensor_packet(send_buffer, sensor);
            if(send_size > 0)
            {
                // データ変換に成功した場合
                int send_result = send_w6300_udp(socket_num, send_buffer, send_size, dest_ip, dest_port);

                if(send_result < 0)
                {
                    //　送信失敗
                    udp_miss_count++;
                }
            }
        }
        
        // 100回送信の失敗を確認したらW6300との通信を終了し初期化モードに戻る
        if(udp_miss_count > 100)
        {
            close_w6300_ethernet(socket_num);
            initalized = false;
            sleep_ms(1000);
        }
    }
}

#endif