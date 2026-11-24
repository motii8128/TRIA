#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int16.hpp>
#include <tria_msgs/msg/motor_feedback.hpp>
#include <tria_auto_msgs/action/machine.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>

#include "pid_utils.hpp"

namespace machine_controller
{
    using MachineAction = tria_auto_msgs::action::Machine;
    using GoalHandleMachine = rclcpp_action::ServerGoalHandle<MachineAction>;

    class CascadePositionController : public rclcpp::Node
    {
        public:
        CascadePositionController();

        // モーターのセンサーを受信する
        void motor_feedback_callback(const tria_msgs::msg::MotorFeedback::SharedPtr msg);

        // 制御を1msで実行する
        void timer_callback();

        // Actionで制御する目標値を受信する
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MachineAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMachine>);
        void handle_accepted(const std::shared_ptr<GoalHandleMachine> goal_handle);

        private:
        std::shared_ptr<GoalHandleMachine> active_goal_handle_;
        rclcpp_action::Server<MachineAction>::SharedPtr action_server_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<tria_msgs::msg::MotorFeedback>::SharedPtr motor_feedback_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr target_current_publisher_;

        std::shared_ptr<CascadeController> cascade_;

        double goal_threshold_;
        double target_position_;
        tria_msgs::msg::MotorFeedback::SharedPtr motor_feedback_;
        rclcpp::Time last_time_;
    };
}