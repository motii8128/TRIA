#ifndef TRIA_AUTO_CORE_HPP_
#define TRIA_AUTO_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tria_auto_msgs/msg/task.hpp>
#include <tria_auto_msgs/action/machine.hpp>
#include <tria_auto_msgs/action/wheel.hpp>

#include <vector>

namespace tria_auto_core
{
    using namespace std::placeholders;

    /// @brief boolを束ねただけ。各制御が開始したのか、終了したのか、を保持する
    class ComponentManager
    {
        public:
        bool wheel_started, vertical_started, horizontal_started, hand_started;
        bool wheel_reached, vertical_reached, horizontal_reached, hand_reached;

        ComponentManager():wheel_started(false),wheel_reached(false),vertical_started(false),vertical_reached(false),
        horizontal_started(false),horizontal_reached(false),hand_started(false),hand_reached(false){}

        void reset()
        {
            wheel_started = false;
            wheel_reached = false;
            vertical_started = false;
            vertical_reached = false;
            horizontal_started = false;
            horizontal_reached = false;
            hand_started = false;
            hand_reached = false;
        }

        bool complete()
        {
            if(wheel_reached && vertical_reached && horizontal_reached && hand_reached)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    };

    enum AutoState
    {
        WAIT,
        COLLECT,
        RELEASE,
        CALIBRATION
    };

    struct MachineTarget
    {
        float vertical, horizontal, hand;
    };

    using MachineAction = tria_auto_msgs::action::Machine;
    using WheelAction = tria_auto_msgs::action::Wheel;
    using GoalHandle_Machine = rclcpp_action::ClientGoalHandle<MachineAction>;
    using GoalHandle_Wheel = rclcpp_action::ClientGoalHandle<WheelAction>;

    class TaskManager : public rclcpp::Node
    {
        public:
        TaskManager();

        void task_callback(const tria_auto_msgs::msg::Task::SharedPtr msg);

        void timer_callback();

        void new_task(bool enable_log);
        void progress_task(bool enable_log);

        private:
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<tria_auto_msgs::msg::Task>::SharedPtr task_subscriber_;

        // 足回りAction
        rclcpp_action::Client<WheelAction>::SharedPtr wheel_action_client_;
        void wheel_send_goal()
        {
            auto wheel_goal_msg = WheelAction::Goal();
            wheel_goal_msg.pose = target_pose_;
            auto send_goal_options = rclcpp_action::Client<WheelAction>::SendGoalOptions();
            send_goal_options.feedback_callback =
            std::bind(&TaskManager::wheel_feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
            std::bind(&TaskManager::wheel_result_callback, this, _1);

            wheel_action_client_->async_send_goal(wheel_goal_msg, send_goal_options);
        }
        void wheel_feedback_callback(GoalHandle_Wheel::SharedPtr, const std::shared_ptr<const WheelAction::Feedback> feedback)
        {
            current_distance_ = feedback->distance;
            current_delta_theta_ = feedback->delta_theta;
        }
        void wheel_result_callback(const GoalHandle_Wheel::WrappedResult & result)
        {
            RCLCPP_INFO(this->get_logger(), "足回りが目標位置に到達しました 残り距離:%lf", result.result->distance);

            comp_manager_.wheel_reached = true;
        }
        // 昇降機構Action
        rclcpp_action::Client<MachineAction>::SharedPtr vertical_action_client_;
        void vertical_send_goal(double target)
        {
            auto goal_msg = MachineAction::Goal();
            goal_msg.target_position = target;
            auto send_goal_options = rclcpp_action::Client<MachineAction>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&TaskManager::vertical_result_callback, this, _1);

            vertical_action_client_->async_send_goal(goal_msg, send_goal_options);
        }
        void vertical_result_callback(const GoalHandle_Machine::WrappedResult & result)
        {
            RCLCPP_INFO(this->get_logger(), "昇降機構が目標位置に到達しました 残り距離:%lf", result.result->error);

            comp_manager_.vertical_reached = true;
        }
        // 水平移動機構Action
        rclcpp_action::Client<MachineAction>::SharedPtr horizontal_action_client_;
        void horizontal_send_goal(double target)
        {
            auto goal_msg = MachineAction::Goal();
            goal_msg.target_position = target;
            auto send_goal_options = rclcpp_action::Client<MachineAction>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&TaskManager::horizontal_result_callback, this, _1);

            horizontal_action_client_->async_send_goal(goal_msg, send_goal_options);
        }
        void horizontal_result_callback(const GoalHandle_Machine::WrappedResult & result)
        {
            RCLCPP_INFO(this->get_logger(), "水平移動機構が目標位置に到達しました 残り距離:%lf", result.result->error);

            comp_manager_.horizontal_reached = true;
        }
        // ハンド機構Action
        rclcpp_action::Client<MachineAction>::SharedPtr hand_action_client_;
        void hand_send_goal(double target)
        {
            auto goal_msg = MachineAction::Goal();
            goal_msg.target_position = target;
            auto send_goal_options = rclcpp_action::Client<MachineAction>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&TaskManager::hand_result_callback, this, _1);

            hand_action_client_->async_send_goal(goal_msg, send_goal_options);
        }
        void hand_result_callback(const GoalHandle_Machine::WrappedResult & result)
        {
            RCLCPP_INFO(this->get_logger(), "ハンド機構が目標位置に到達しました 残り距離:%lf", result.result->error);

            comp_manager_.hand_reached = true;
        }

        // 機構制御を許可する閾値(パラメータ)
        double enable_arm_distance_;
        // 回収時の機構の目標位置(パラメータ)
        double collect_vertical_pos_;
        double collect_horizontal_pos_;
        double collect_hand_pos_;
        // アームの長さ、アームの長さを考慮した足回りの目標位置を算出するため。(パラメータ)
        double arm_length_;


        // 状態を管理する
        AutoState state_;
        // タスクを保持するvector(先頭から処理する)
        std::vector<tria_auto_msgs::msg::Task::SharedPtr> task_;
        // 目標位置を保持する
        geometry_msgs::msg::PoseStamped target_pose_;
        // 機構の目標位置を保持する
        MachineTarget machine_target_;
        // 足回りにおける目標との誤差
        double current_distance_;
        // 姿勢角度の誤差
        double current_delta_theta_;
        // カウント用変数
        int log_count_;

        // trueならResult受取済み
        ComponentManager comp_manager_;
    };

    double to_radian(geometry_msgs::msg::Quaternion q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

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