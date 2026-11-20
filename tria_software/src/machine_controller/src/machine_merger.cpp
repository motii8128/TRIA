#include "machine_controller/machine_merger.hpp"

namespace machine_controller
{
    MachineMerger::MachineMerger() : Node("machine_merger")
    {
        machine_current_publisher_ = this->create_publisher<tria_msgs::msg::MachineCurrent>("/ccb/machine_current", rclcpp::SystemDefaultsQoS());

        vertical_current_ = nullptr;
        horizontal_current_ = nullptr;
        hand_current_ = nullptr;

        using std::placeholders::_1;
        vertical_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "/target_current/vertical",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MachineMerger::vertical_callback, this, _1)
        );

        horizontal_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "/target_current/horizontal",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MachineMerger::horizontal_callback, this, _1)
        );

        hand_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "/target_current/hand",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MachineMerger::hand_callback, this, _1)
        );

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(1ms, std::bind(&MachineMerger::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Start %s", this->get_name());
    }

    void MachineMerger::timer_callback()
    {
        auto machine_current = tria_msgs::msg::MachineCurrent();

        if(vertical_current_ != nullptr) machine_current.vertical_motor = vertical_current_->data;

        if(horizontal_current_ != nullptr) machine_current.horizontal_motor = horizontal_current_->data;

        if(hand_current_ != nullptr) machine_current.hand_motor = hand_current_->data;

        machine_current_publisher_->publish(machine_current);
    }

    void MachineMerger::vertical_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        vertical_current_ = msg;
    }

    void MachineMerger::horizontal_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        horizontal_current_ = msg;
    }

    void MachineMerger::hand_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        hand_current_ = msg;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<machine_controller::MachineMerger>());
  rclcpp::shutdown();
  return 0;
}