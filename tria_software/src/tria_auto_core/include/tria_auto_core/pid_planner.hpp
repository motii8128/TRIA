#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "pid_utils.hpp"

#include <cmath>

namespace tria_auto_core
{
    class PidPlanner : public rclcpp::Node
    {
        public:
        PidPlanner();

        void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::shared_ptr<PID> x_pid_;
        std::shared_ptr<PID> y_pid_;
        std::shared_ptr<PID> rotation_pid_;

        geometry_msgs::msg::PoseStamped::SharedPtr target_pose_;

        rclcpp::Time last_time_;
    };

    double to_radian(geometry_msgs::msg::Quaternion q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
}