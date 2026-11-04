#ifndef TRIA_AUTO_CORE_HPP_
#define TRIA_AUTO_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tria_auto_msgs/msg/task.hpp>
#include <tria_msgs/msg/sensor_feedback.hpp>
#include <tria_msgs/msg/machine_control.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>

namespace tria_auto_core
{
    enum State
    {
        WAIT,
        POSITIONING,
        COLLECT_OR_RELEASE,
    };

    class TaskManager : public rclcpp::Node
    {
        public:
        TaskManager();

        void task_callback(const tria_auto_msgs::msg::Task::SharedPtr msg);

        void sensor_feedback_callback(const tria_msgs::msg::SensorFeedback::SharedPtr msg);

        void timer_callback();

        void new_task(bool enable_log, const geometry_msgs::msg::TransformStamped current_position);
        void progress_task(bool enable_log, const geometry_msgs::msg::TransformStamped current_position);

        private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<tria_auto_msgs::msg::Task>::SharedPtr task_subscriber_;
        rclcpp::Subscription<tria_msgs::msg::SensorFeedback>::SharedPtr sensor_feedback_subscriber_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
        rclcpp::Publisher<tria_msgs::msg::MachineControl>::SharedPtr machine_control_publisher_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::vector<tria_auto_msgs::msg::Task::SharedPtr> task_;
        geometry_msgs::msg::PoseStamped target_pose_;
        tria_msgs::msg::MachineControl target_machine_position_;
        State state_;
        tria_msgs::msg::SensorFeedback::SharedPtr sensor_feedback_;

        // 目標に達したと判定する閾値
        double position_goal_threshold_;
        double arm_goal_threshold_;
        double hand_goal_threshold_;

        // 機構制御を許可する閾値
        double enable_arm_distance_;

        // 物を掴んだと判定するTorque値
        int16_t torque_threshold_;

        // 回収時の機構の目標位置(パラメータ)
        double collect_vertical_pos_;
        double collect_horizontal_pos_;
        double collect_hand_pos_;

        double arm_length_;

        int log_count_;
    };

    std::string to_string(const tria_auto_msgs::msg::Task::SharedPtr msg)
    {
        auto mode = std::string("モード：");
        switch (msg->purpose)
        {
        case tria_auto_msgs::msg::Task::COLLECT:
            mode += std::string("回収");
            break;
        case tria_auto_msgs::msg::Task::RELEASE:
            mode += std::string("設置");
            break;
        case tria_auto_msgs::msg::Task::CALIBRATION:
            mode += std::string("キャリブレーション");
            break;
        
        default:
            break;
        }

        return mode;
    }
}

#endif