#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <tria_msgs/msg/motor_feedback.hpp>

#include "pid_utils.hpp"

namespace machine_controller
{
    class MotorController : public rclcpp::Node
    {
        public:
        MotorController();

        void target_position_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void motor_feedback_callback(const tria_msgs::msg::MotorFeedback::SharedPtr msg);


        private:
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_position_subscriber_;
        rclcpp::Subscription<tria_msgs::msg::MotorFeedback>::SharedPtr motor_feedback_subscriber_;

        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr target_current_publisher_;

        rclcpp::Time last_time_;

        // 水平移動機構
        std::shared_ptr<PID> pid_;
        std::shared_ptr<CascadeController> cascade_;

        tria_msgs::msg::MotorFeedback::SharedPtr motor_feedback_;
    };
}