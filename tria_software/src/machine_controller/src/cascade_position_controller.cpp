#include "machine_controller/cascade_position_controller.hpp"

namespace machine_controller
{
    CascadePositionController::CascadePositionController() : rclcpp::Node("motor_controller")
    {
        PidConfig p_config = PidConfig();
        PidConfig v_config = PidConfig();

        // 上下機構のPIDパラメータ
        p_config.p_gain = this->declare_parameter("pos.p_gain", 1.0);
        p_config.i_gain = this->declare_parameter("pos.i_gain", 0.0);
        p_config.d_gain = this->declare_parameter("pos.d_gain", 0.0);
        p_config.enable_dynamic_integral_reset = false;
        p_config.max_output = this->declare_parameter("pos.max_output", 3000.0);
        p_config.max_integral = this->declare_parameter("pos.max_integral", 1000.0);

        v_config.p_gain = this->declare_parameter("vel.p_gain", 1.0);
        v_config.i_gain = this->declare_parameter("vel.i_gain", 0.0);
        v_config.d_gain = this->declare_parameter("vel.d_gain", 0.0);
        v_config.enable_dynamic_integral_reset = true;
        v_config.max_output = this->declare_parameter("vel.max_output", 3000.0);
        v_config.max_integral = this->declare_parameter("vel.max_integral", 1000.0);

        goal_threshold_ = this->declare_parameter("goal_threshold", 0.01);

        cascade_ = std::make_shared<CascadeController>(p_config, v_config);

        motor_feedback_ = nullptr;
        target_position_ = 0.0;

        target_current_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/target_current", rclcpp::SystemDefaultsQoS());

        using std::placeholders::_1;

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(1ms, std::bind(&CascadePositionController::timer_callback, this));


        motor_feedback_subscriber_ = this->create_subscription<tria_msgs::msg::MotorFeedback>(
            "/feedback",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&CascadePositionController::motor_feedback_callback, this, _1)
        );

        action_server_ = rclcpp_action::create_server<MachineAction>(
            this,
            "machine_action",
            std::bind(&CascadePositionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CascadePositionController::handle_cancel, this, std::placeholders::_1),
            std::bind(&CascadePositionController::handle_accepted, this, std::placeholders::_1)
        );

        active_goal_handle_ = nullptr;
        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "%sを起動しました", this->get_name());
    }

    void CascadePositionController::timer_callback()
    {
        // センサーの値が来てないなら制御を行わない
        if(motor_feedback_ == nullptr) return;

        const auto now_time = this->now();
        const auto dt = (now_time - last_time_).seconds();

        // アクティブなGoalがない場合は電流０で待機
        if(active_goal_handle_ == nullptr || !active_goal_handle_->is_active())
        {
            auto current_msg = std_msgs::msg::Int16();
            current_msg.data = 0;
            target_current_publisher_->publish(current_msg);
            return;
        }

        const auto now_position = motor_feedback_->position;
        const auto abs_error = std::abs(target_position_ - now_position);

        const auto current_cmd = cascade_->compute(
            target_position_, 
            motor_feedback_->position,
            motor_feedback_->rpm,
            dt
        );

        auto current_msg = std_msgs::msg::Int16();
        current_msg.data = static_cast<int16_t>(std::llround(current_cmd));
        target_current_publisher_->publish(current_msg);

        if(abs_error < goal_threshold_)
        {
            auto current_msg = std_msgs::msg::Int16();
            current_msg.data = 0;
            target_current_publisher_->publish(current_msg);

            auto result = std::make_shared<MachineAction::Result>();
            result->error = abs_error;
            active_goal_handle_->succeed(result);
            active_goal_handle_ = nullptr;

            RCLCPP_INFO(this->get_logger(), "制御完了 誤差:%.4f", abs_error);
        }
    }

    

    void CascadePositionController::motor_feedback_callback(const tria_msgs::msg::MotorFeedback::SharedPtr msg)
    {
        motor_feedback_ = msg;
    }


    // Goalを受信した際に呼び出される
    rclcpp_action::GoalResponse CascadePositionController::handle_goal(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const MachineAction::Goal> goal
    )
    {
        if (!std::isfinite(goal->target_position)) {
            RCLCPP_WARN(this->get_logger(), "意味不明な目標位置なのでGoalを棄却しました");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // GoalをAcceptした際に呼び出される
    void CascadePositionController::handle_accepted(const std::shared_ptr<GoalHandleMachine> goal_handle)
    {
        active_goal_handle_ = goal_handle;
        target_position_ = active_goal_handle_->get_goal()->target_position;

        RCLCPP_INFO(this->get_logger(), "新たな目標位置を取得しました: %.4f", target_position_);
    }

    // キャンセルを受信したら呼び出される
    rclcpp_action::CancelResponse CascadePositionController::handle_cancel(const std::shared_ptr<GoalHandleMachine>)
    {
        RCLCPP_INFO(this->get_logger(), "キャンセルのリクエストを受け付けました");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<machine_controller::CascadePositionController>());
  rclcpp::shutdown();
  return 0;
}