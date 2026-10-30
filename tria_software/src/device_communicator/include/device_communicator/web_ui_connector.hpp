#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <tria_auto_msgs/msg/task.hpp>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace device_communicator
{
    class WebUIConnector : public rclcpp::Node
    {
        public:
        WebUIConnector();

        void string_callback(const std_msgs::msg::String::SharedPtr msg);

        private:
        rclcpp::Publisher<tria_auto_msgs::msg::Task>::SharedPtr task_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscriber_;
    };
}