#include <rclcpp/rclcpp.hpp>

#include <tria_msgs/msg/wheel_duty.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "omni_utils.hpp"

namespace wheel_controller
{
    class SimpleController : public rclcpp::Node
    {
        public:
        SimpleController();

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
        rclcpp::Publisher<tria_msgs::msg::WheelDuty>::SharedPtr wheel_duty_publisher_;

        std::shared_ptr<Omni> omni_;
    };
}