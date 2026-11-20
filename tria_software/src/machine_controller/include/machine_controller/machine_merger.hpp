#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int16.hpp>
#include <tria_msgs/msg/machine_current.hpp>

#include <chrono>

namespace machine_controller
{
    class MachineMerger : public rclcpp::Node
    {
        public:
        MachineMerger();

        void vertical_callback(const std_msgs::msg::Int16::SharedPtr msg);
        void horizontal_callback(const std_msgs::msg::Int16::SharedPtr msg);
        void hand_callback(const std_msgs::msg::Int16::SharedPtr msg);

        void timer_callback();

        private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr vertical_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr horizontal_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr hand_subscriber_;

        rclcpp::Publisher<tria_msgs::msg::MachineCurrent>::SharedPtr machine_current_publisher_;

        std_msgs::msg::Int16::SharedPtr vertical_current_;
        std_msgs::msg::Int16::SharedPtr horizontal_current_;
        std_msgs::msg::Int16::SharedPtr hand_current_;
    };
}