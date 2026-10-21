#include "wheel_controller/simple_controller.hpp"

namespace wheel_controller
{
    SimpleController::SimpleController() : rclcpp::Node("simple_controller")
    {
        const auto to_rad = M_PI / 180.0;
        const auto body_radius = this->declare_parameter("body_radius", 0.3);
        const auto wheel1_angle = this->declare_parameter("wheel1_angle", 60.0) * to_rad;
        const auto wheel2_angle = this->declare_parameter("wheel2_angle", 60.0) * to_rad;
        const auto wheel3_angle = this->declare_parameter("wheel3_angle", 60.0) * to_rad;

        const auto max_speed = this->declare_parameter("max_speed", 1.0);

        omni_ = std::make_shared<Omni>(body_radius, max_speed, wheel1_angle, wheel2_angle, wheel3_angle);

        using std::placeholders::_1;

        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SimpleController::twist_callback, this, _1)
        );

        wheel_duty_publisher_ = this->create_publisher<tria_msgs::msg::WheelDuty>("/ccb/wheel_duty", rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(), "SimpleControllerを起動しました");
    }

    void SimpleController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const auto target_speed = omni_->calc(
            msg->linear.x,
            msg->linear.y,
            msg->angular.z
        );

        // SimpleControllerは単純に比例制御を行う
        auto wheel_duty = tria_msgs::msg::WheelDuty();

        wheel_duty.motor1 = target_speed[0] * 1000;
        wheel_duty.motor2 = target_speed[1] * 1000;
        wheel_duty.motor3 = target_speed[2] * 1000;

        wheel_duty_publisher_->publish(wheel_duty);
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_controller::SimpleController>());
  rclcpp::shutdown();
  return 0;
}