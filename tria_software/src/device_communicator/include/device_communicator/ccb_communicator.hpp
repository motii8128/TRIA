#include <rclcpp/rclcpp.hpp>

// ROS2 カスタムメッセージ
#include <tria_msgs/msg/sensor_feedback.hpp>
#include <tria_msgs/msg/motor_feedback.hpp>
#include <tria_msgs/msg/machine_current.hpp>
#include <tria_msgs/msg/wheel_duty.hpp>

// protocol buffers
#include "tria.pb.h"

// UDP通信ライブラリ
#include "udp_utils.hpp"

// ロボマスライブラリ
#include "robomaster_utils.hpp"

#include <chrono>

using namespace tria;

namespace device_communicator
{
    class CcbCommunicator : public rclcpp::Node
    {
        public:
        CcbCommunicator();

        void timer_callback();
        void machine_current_callback(const tria_msgs::msg::MachineCurrent::SharedPtr msg);
        void wheel_duty_callback(const tria_msgs::msg::WheelDuty::SharedPtr msg);

        private:
        CommandPacket create_command_packet()
        {
            CommandPacket cp;

            cp.set_hand_motor(machine_current_.hand_motor);
            cp.set_horizontal_motor(machine_current_.horizontal_motor);
            cp.set_vertical_motor(machine_current_.vertical_motor);

            cp.set_wheel1(wheel_duty_.motor1);
            cp.set_wheel2(wheel_duty_.motor2);
            cp.set_wheel3(wheel_duty_.motor3);

            return cp;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<tria_msgs::msg::SensorFeedback>::SharedPtr feedback_publisher_;

        rclcpp::Subscription<tria_msgs::msg::MachineCurrent>::SharedPtr machine_current_subscriber_;
        rclcpp::Subscription<tria_msgs::msg::WheelDuty>::SharedPtr wheel_duty_subscriber_;

        tria_msgs::msg::MachineCurrent machine_current_;
        tria_msgs::msg::WheelDuty wheel_duty_;

        tria_msgs::msg::SensorFeedback sensor_feedback_;
        // ロボマスのエンコーダ処理
        RoboMaster robomaster_[3];

        // UDP通信
        std::shared_ptr<UdpHandler> udp_;
        int udp_success_count_;
        std::string my_addr_, dest_addr_;
        uint16_t my_port_, dest_port_;
    };
}