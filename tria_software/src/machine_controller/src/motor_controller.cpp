#include "machine_controller/motor_controller.hpp"

namespace machine_controller
{
    MotorController::MotorController() : rclcpp::Node("motor_controller")
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

        cascade_ = std::make_shared<CascadeController>(p_config, v_config);

        motor_feedback_ = nullptr;

        target_current_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/target_current", rclcpp::SystemDefaultsQoS());

        using std::placeholders::_1;

        target_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/target",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MotorController::target_position_callback, this, _1)
        );

        motor_feedback_subscriber_ = this->create_subscription<tria_msgs::msg::MotorFeedback>(
            "/feedback",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MotorController::motor_feedback_callback, this, _1)
        );

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "PidControllerを起動しました");
    }

    void MotorController::target_position_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // センサーの値が来てないなら制御を行わない
        if(motor_feedback_ == nullptr) return;

        const auto now = this->get_clock()->now();
        const auto dt = (now - last_time_).seconds();   
        
        auto current_msg = std_msgs::msg::Int16();

        current_msg.data = cascade_->compute(
            msg->data,
            motor_feedback_->position,
            motor_feedback_->rpm,
            dt
        );

        target_current_publisher_->publish(current_msg);

        last_time_ = now;
    }

    void MotorController::motor_feedback_callback(const tria_msgs::msg::MotorFeedback::SharedPtr msg)
    {
        motor_feedback_ = msg;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<machine_controller::MotorController>());
  rclcpp::shutdown();
  return 0;
}