#include "device_communicator/web_ui_connector.hpp"

namespace device_communicator
{
    WebUIConnector::WebUIConnector() : Node("web_ui_connector")
    {
        task_publisher_ = this->create_publisher<tria_auto_msgs::msg::Task>("/task", rclcpp::SystemDefaultsQoS());

        using std::placeholders::_1;
        string_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/web/command",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&WebUIConnector::string_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "WebUIConnectorを起動しました");
    }

    void WebUIConnector::string_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            json j = json::parse(msg->data.c_str());

            auto task_msg = tria_auto_msgs::msg::Task();
            auto target_pose = geometry_msgs::msg::PoseStamped();

            target_pose.header.frame_id = j.at("frame_id").get<std::string>();
            target_pose.pose.position.x = j.at("x").get<double>();
            target_pose.pose.position.y = j.at("y").get<double>();

            task_msg.pose = target_pose;

            const auto mode_str = j.at("mode").get<std::string>();
            if(mode_str == "collect")
            {
                task_msg.purpose = tria_auto_msgs::msg::Task::COLLECT;
            }
            if(mode_str == "place")
            {
                task_msg.purpose = tria_auto_msgs::msg::Task::RELEASE;
            }

            task_publisher_->publish(task_msg);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "JSON parse error: %s", e.what());
        }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<device_communicator::WebUIConnector>());
  rclcpp::shutdown();
  return 0;
}