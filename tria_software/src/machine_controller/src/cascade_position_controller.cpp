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
        status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/status", rclcpp::SystemDefaultsQoS());

        using std::placeholders::_1;

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(1ms, std::bind(&CascadePositionController::timer_callback, this));

        target_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/target",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&CascadePositionController::target_position_callback, this, _1)
        );

        motor_feedback_subscriber_ = this->create_subscription<tria_msgs::msg::MotorFeedback>(
            "/feedback",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&CascadePositionController::motor_feedback_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "%sを起動しました", this->get_name());
    }

    void CascadePositionController::timer_callback()
    {
        // センサーの値が来てないなら制御を行わない
        if(motor_feedback_ == nullptr) return;

        const auto delta_position = target_position_ - motor_feedback_->position;

        auto current_msg = std_msgs::msg::Int16();
        auto status_msg = std_msgs::msg::Bool();

        if(delta_position < goal_threshold_)
        {
            current_msg.data = 0;
            status_msg.data = true;
            target_current_publisher_->publish(current_msg);
            status_publisher_->publish(status_msg);
        }
        else
        {
            current_msg.data = (int16_t)cascade_->compute(
                target_position_,
                motor_feedback_->position,
                motor_feedback_->rpm,
                0.001
            );
            status_msg.data = false;

            target_current_publisher_->publish(current_msg);
            status_publisher_->publish(status_msg);
        }
    }

    void CascadePositionController::target_position_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {     
        target_position_ = msg->data;
    }

    void CascadePositionController::motor_feedback_callback(const tria_msgs::msg::MotorFeedback::SharedPtr msg)
    {
        motor_feedback_ = msg;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<machine_controller::CascadePositionController>());
  rclcpp::shutdown();
  return 0;
}