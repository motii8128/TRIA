#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <tria_auto_msgs/action/wheel.hpp>

#include "pid_utils.hpp"

#include <cmath>

namespace tria_auto_core
{
    using WheelAction = tria_auto_msgs::action::Wheel;
    using GoalHandleWheel = rclcpp_action::ServerGoalHandle<WheelAction>;

    class PidPlanner : public rclcpp::Node
    {
        public:
        PidPlanner();

        void timer_callback();

        // Actionで制御する目標値を受信する
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const WheelAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWheel>);
        void handle_accepted(const std::shared_ptr<GoalHandleWheel> goal_handle);

        private:
        std::shared_ptr<GoalHandleWheel> active_goal_handle_;
        rclcpp_action::Server<WheelAction>::SharedPtr action_server_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::shared_ptr<PID> x_pid_;
        std::shared_ptr<PID> y_pid_;
        std::shared_ptr<PID> rotation_pid_;

        geometry_msgs::msg::PoseStamped target_pose_;

        rclcpp::Time last_time_;

        double linear_threshold_;
        double angular_threshold_;
    };

    double to_radian(geometry_msgs::msg::Quaternion q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
}