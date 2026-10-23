#include "device_communicator/game_controller_communicator.hpp"

namespace device_communicator
{
    GameControllerCommunicator::GameControllerCommunicator() : rclcpp::Node("game_controller_communicator")
    {
        vertical_max_position_ = this->declare_parameter("vertical.position.max", 10.0);
        vertical_min_position_ = this->declare_parameter("vertical.position.min", 0.0);
        horizontal_max_position_ = this->declare_parameter("horizontal.position.max", 10.0);
        horizontal_min_position_ = this->declare_parameter("horizontal.position.min", 0.0);
        hand_max_position_ = this->declare_parameter("hand.position.max", 10.0);
        hand_min_position_ = this->declare_parameter("hand.position.min", 0.0);

        machine_control_msg_ = tria_msgs::msg::MachineControl();

        // ROS2の初期化
        using std::placeholders::_1; /* 変数がひとつであることを示す */
        // サブスクライバの初期化
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&GameControllerCommunicator::joy_callback, this, _1)
        );

        machine_control_publisher_ = this->create_publisher<tria_msgs::msg::MachineControl>("/machine_control", rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(), "%sを初期化しました", this->get_name());
        RCLCPP_INFO(this->get_logger(), "上下移動機構：\tmax=%lf\tmin=%lf", vertical_max_position_, vertical_min_position_);
        RCLCPP_INFO(this->get_logger(), "前後移動機構：\tmax=%lf\tmin=%lf", horizontal_max_position_, horizontal_min_position_);
        RCLCPP_INFO(this->get_logger(), "ハンド機構：\tmax=%lf\tmin=%lf", hand_max_position_, hand_min_position_);
    }

    void GameControllerCommunicator::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const auto x_axis = 1.0 * msg->axes[7];
        const auto y_axis = -1.0 * msg->axes[6];

        const auto cc = msg->buttons[1] - msg->buttons[0];
        
        machine_control_msg_.horizontal_position += x_axis;
        machine_control_msg_.vertical_position += y_axis;

        if(cc > 0)
        {
            machine_control_msg_.hand_position = hand_max_position_;
        }
        else if(cc < 0)
        {
            machine_control_msg_.hand_position = hand_min_position_;
        }

        if(machine_control_msg_.horizontal_position > horizontal_max_position_)
        {
            machine_control_msg_.horizontal_position = horizontal_max_position_;
        }
        if(machine_control_msg_.horizontal_position < horizontal_min_position_)
        {
            machine_control_msg_.horizontal_position = horizontal_min_position_;
        }

        if(machine_control_msg_.vertical_position > vertical_max_position_)
        {
            machine_control_msg_.vertical_position = vertical_max_position_;
        }
        if(machine_control_msg_.vertical_position < vertical_min_position_)
        {
            machine_control_msg_.vertical_position = vertical_min_position_;
        }

        machine_control_publisher_->publish(machine_control_msg_);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<device_communicator::GameControllerCommunicator>());
  rclcpp::shutdown();
  return 0;
}