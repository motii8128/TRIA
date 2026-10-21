#ifndef UDP_UTILS_HPP_
#define UDP_UTILS_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string>
#include <stdint.h>
#include <vector>


namespace device_communicator
{
    /// @brief UDPHandlerの状態を示すのに使う
    enum UDP_STATE
    {
        ERR_CREATE_SOCKET = 1,
        ERR_BIND_SOCKET = 2,
        ERR_SET_TIMEOUT = 3,
        ERR_UDP = 4,
        OK_UDP = 5,
    };

    /// @brief UDP通信を行うクラス
    class UdpHandler
    {
        public:
        /// @brief コンストラクタ
        /// @param port 通信相手のポート
        /// @param addr 通信相手のアドレス
        UdpHandler(uint16_t port, std::string addr)
        {
            port_ = port;
            addr_str_ = addr;
        }

        /// @brief UDP通信を起動する
        /// @param timeout タイムアウトの秒数を指定する（milli sec）
        /// @return 成功か失敗か返す
        UDP_STATE open(uint timeout)
        {
            sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

            if(sock_fd_ < 0)
            {
                close(sock_fd_);
                return UDP_STATE::ERR_CREATE_SOCKET;
            }

            addr_ = sockaddr_in{};
            addr_.sin_family = AF_INET;
            addr_.sin_port = htons(port_);
            addr_.sin_addr.s_addr = inet_addr(addr_str_.c_str());

            if(bind(sock_fd_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
            {
                close(sock_fd_);
                return UDP_STATE::ERR_BIND_SOCKET;
            }

            struct timeval tv;
            tv.tv_sec = timeout / 1000;
            tv.tv_usec = (timeout % 1000) * 1000;

            if(setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0)
            {
                close(sock_fd_);
                return UDP_STATE::ERR_SET_TIMEOUT;
            }

            return UDP_STATE::OK_UDP;
        }

        /// @brief 受信を行う。
        /// @return 成功か失敗か返す
        UDP_STATE receive(uint8_t* buffer, size_t size)
        {
            ssize_t len = recv(sock_fd_, buffer, size, 0);

            if(len > 0)
            {
                buffer[len] = '\0';

                return UDP_STATE::OK_UDP;
            }
            else
            {
                // close(sock_fd_);
                return UDP_STATE::ERR_UDP;
            }
        }

        /// @brief UDP通信により文字列を送信する
        /// @param tx_packet 送信内容
        /// @param dest_addr 送信先のアドレス
        /// @param dest_port 送信先のポート
        /// @return 成功か失敗か返す
        UDP_STATE send(const char* tx_packet, size_t length, std::string dest_addr, uint16_t dest_port)
        {
            sockaddr_in dest_addr_{};
            dest_addr_.sin_family = AF_INET;
            dest_addr_.sin_port = htons(dest_port);
            inet_pton(AF_INET, dest_addr.c_str(), &dest_addr_.sin_addr);

            ssize_t sent = sendto(sock_fd_, tx_packet, length, 0, (sockaddr*)&dest_addr_, sizeof(dest_addr_));

            if(sent > 0)
            {
                return UDP_STATE::OK_UDP;
            }
            else
            {
                // close(sock_fd_);
                return UDP_STATE::ERR_UDP;
            }
        }

        private:
        int sock_fd_;
        sockaddr_in addr_;
        uint16_t port_;
        std::string addr_str_;
    };
}

#endif