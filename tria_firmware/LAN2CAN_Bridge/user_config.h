#ifndef USER_CONFIG_H_
#define USER_CONFIG_H_

//////////// UDP設定 ////////////
#define MY_IP_ADDR {192, 168, 11, 2}
#define MY_PORT 64201
#define DEST_IP_ADDR {192, 168, 11, 4}
#define DEST_PORT 64201

#define GATE_WAY {192, 168, 11, 1}

#define RECV_BUFFER_SIZE 128


//////////// CAN通信設定 ////////////
#define CAN_TX 7
#define CAN_RX 6
#define CAN_BIT_RATE 1000000


#endif