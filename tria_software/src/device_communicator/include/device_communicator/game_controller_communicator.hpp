#include <rclcpp/rclcpp.hpp>

// ROS2 カスタムメッセージ
#include <sensor_msgs/msg/joy.hpp>
#include <tria_msgs/msg/machine_control.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace device_communicator
{
    class GameControllerCommunicator : public rclcpp::Node
    {
        public:
        GameControllerCommunicator();

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<tria_msgs::msg::MachineControl>::SharedPtr machine_control_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

        tria_msgs::msg::MachineControl machine_control_msg_;

        double vertical_max_position_;
        double vertical_min_position_;
        double horizontal_max_position_;
        double horizontal_min_position_;
        double hand_max_position_;
        double hand_min_position_;
    };
}