#include <rclcpp/rclcpp.hpp>

#include <tria_msgs/msg/machine_control.hpp>
#include <tria_msgs/msg/machine_current.hpp>
#include <tria_msgs/msg/sensor_feedback.hpp>

#include "pid_utils.hpp"

namespace machine_controller
{
    class PidController : public rclcpp::Node
    {
        public:
        PidController();

        void machine_control_callback(const tria_msgs::msg::MachineControl::SharedPtr msg);
        void sensor_feedback_callback(const tria_msgs::msg::SensorFeedback::SharedPtr msg);

        void calibration_mode(const tria_msgs::msg::SensorFeedback::SharedPtr msg, float dt);
        void zero_mode(const tria_msgs::msg::SensorFeedback::SharedPtr msg, float dt);
        void normal_mode(const tria_msgs::msg::SensorFeedback::SharedPtr msg, float dt);

        private:
        rclcpp::Subscription<tria_msgs::msg::MachineControl>::SharedPtr machine_control_subscriber_;
        rclcpp::Subscription<tria_msgs::msg::SensorFeedback>::SharedPtr sensor_feedback_subscriber_;

        rclcpp::Publisher<tria_msgs::msg::MachineCurrent>::SharedPtr machine_current_publisher_;

        rclcpp::Time last_time_;

        // 水平移動機構
        std::shared_ptr<PID> horizontal_pid_;
        std::shared_ptr<CascadeController> horizontal_cascade_;

        // 上下移動機構
        std::shared_ptr<PID> vertical_pid_;
        std::shared_ptr<CascadeController> vertical_cascade_;

        // ハンド機構
        std::shared_ptr<PID> hand_pid_;
        std::shared_ptr<CascadeController> hand_cascade_;

        tria_msgs::msg::MachineControl::SharedPtr machine_control_;
    };
}