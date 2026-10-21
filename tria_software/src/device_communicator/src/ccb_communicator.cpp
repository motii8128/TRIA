#include "device_communicator/ccb_communicator.hpp"

namespace device_communicator
{
    CcbCommunicator::CcbCommunicator() : rclcpp::Node("ccb_communicator")
    {
        udp_success_count_ = 0;

        // ロボマスクラスを初期化
        for(int i = 0; i < 3; i++)robomaster_[i] = RoboMaster();

        my_addr_ = this->declare_parameter("udp_my_addr", "127.0.0.1");
        dest_addr_ = this->declare_parameter("udp_dest_addr", "127.0.0.1");

        my_port_ = this->declare_parameter("udp_my_port", 64201);
        dest_port_ = this->declare_parameter("udp_dest_port", 64201);

        const auto timer_interval = this->declare_parameter("udp_interval_ms", 1);
        const auto udp_timeout = this->declare_parameter("udp_timeout_ms", 10);

        // UDP通信を初期化する
        udp_ = std::make_shared<UdpHandler>(my_port_, my_addr_);

        const auto open_result = udp_->open(udp_timeout);
        if(open_result != UDP_STATE::OK_UDP)
        {
            RCLCPP_ERROR(this->get_logger(), "UDP通信の初期化に失敗しました。");
            switch (open_result)
            {
            case UDP_STATE::ERR_BIND_SOCKET:
                RCLCPP_ERROR(this->get_logger(), "アドレスを確認してください");
                break;
            case UDP_STATE::ERR_CREATE_SOCKET:
                RCLCPP_ERROR(this->get_logger(), "ソケット作成に失敗しています");
                break;
            case UDP_STATE::ERR_SET_TIMEOUT:
                RCLCPP_ERROR(this->get_logger(), "タイムアウト設定に失敗しました");
                break;
            
            default:
                break;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "UDP通信の初期化に成功しました：%s:%d", my_addr_.c_str(), my_port_);
        }

        // ROS2の初期化
        machine_current_ = tria_msgs::msg::MachineCurrent();
        wheel_duty_ = tria_msgs::msg::WheelDuty();
        sensor_feedback_ = tria_msgs::msg::SensorFeedback();

        // UDP通信をするタイマーを初期化
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval), 
            std::bind(&CcbCommunicator::timer_callback, this)
        );

        using std::placeholders::_1; /* 変数がひとつであることを示す */
        // サブスクライバの初期化
        machine_current_subscriber_ = this->create_subscription<tria_msgs::msg::MachineCurrent>(
            "/ccb/machine_current", 
            rclcpp::SystemDefaultsQoS(),
            std::bind(&CcbCommunicator::machine_current_callback, this, _1)
        );

        wheel_duty_subscriber_ = this->create_subscription<tria_msgs::msg::WheelDuty>(
            "ccb/wheel_duty",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&CcbCommunicator::wheel_duty_callback, this, _1)
        );

        //　パブリッシャーの初期化
        feedback_publisher_ = this->create_publisher<tria_msgs::msg::SensorFeedback>("/ccb/sensor_feedback", rclcpp::SystemDefaultsQoS());
    }

    void CcbCommunicator::machine_current_callback(const tria_msgs::msg::MachineCurrent::SharedPtr msg)
    {
        machine_current_ = *msg;
    }

    void CcbCommunicator::wheel_duty_callback(const tria_msgs::msg::WheelDuty::SharedPtr msg)
    {
        wheel_duty_ = *msg;
    }

    void CcbCommunicator::timer_callback()
    {
        // 受信したメッセージから送信するパケットを作成
        const auto command_packet = create_command_packet();

        std::string send_buffer;
        command_packet.SerializeToString(&send_buffer);

        const auto send_result = udp_->send(send_buffer.data(), send_buffer.size(), dest_addr_, dest_port_);

        if(send_result == UDP_STATE::OK_UDP)
        {
            // 送信成功したら受信処理
            uint8_t recv_buffer[50];

            const auto recv_result = udp_->receive(recv_buffer, 50);

            if(recv_result == UDP_STATE::OK_UDP)
            {
                tria::SensorPacket sensor;
                if(sensor.ParseFromArray(recv_buffer, 50))
                {
                    switch (sensor.can_id())
                    {
                    case 1:
                        // CAN IDが１なら水平移動のモーターであるので
                        robomaster_[0].update(sensor.velocity(), sensor.angle(), sensor.torque());
                        sensor_feedback_.horizontal = robomaster_[0].to_ROS2();
                        break;
                    case 2:
                        // CAN IDが２なら上下移動のモーターであるので
                        robomaster_[1].update(sensor.velocity(), sensor.angle(), sensor.torque());
                        sensor_feedback_.vertical = robomaster_[1].to_ROS2();
                        break;
                    case 3:
                        // CAN IDが３ならハンドのモーターであるので
                        robomaster_[2].update(sensor.velocity(), sensor.angle(), sensor.torque());
                        sensor_feedback_.hand = robomaster_[2].to_ROS2();
                        break;

                    default:
                        break;
                    }
                }

                udp_success_count_++;

                if(udp_success_count_ == 500)
                {
                    RCLCPP_INFO(this->get_logger(), "CCBとのコネクションが正常です。");
                    udp_success_count_ = 0;
                }

                feedback_publisher_->publish(sensor_feedback_);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "UDP受信に失敗");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "UDP送信に失敗");
        }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<device_communicator::CcbCommunicator>());
  rclcpp::shutdown();
  return 0;
}