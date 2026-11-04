#include "tria_localization/lidar_only_odometer.hpp"

namespace tria_localization
{
    LidarOnlyOdometer::LidarOnlyOdometer() : Node("lidar_only_odometer")
    {
        max_iteration_ = this->declare_parameter("max_iteration", 20);
        tolerance_ = this->declare_parameter("tolerance", 0.001);
        radius_search_ = this->declare_parameter("search_radius", 0.3);
        neighbor_threshold_ = this->declare_parameter("neighbor_threshold", 0.3);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        using std::placeholders::_1;

        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/merged_points",
            rclcpp::SensorDataQoS().reliable(),
            std::bind(&LidarOnlyOdometer::pointcloud_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "LidarOnlyOdometerが初期化されました");

        prev_points_ = Points(0);
    }

    void LidarOnlyOdometer::pointcloud_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        Points new_points;
        for(const auto& p : msg->points)
        {
            if(std::isfinite(p.x) && std::isfinite(p.y))
            {
                new_points.emplace_back(
                    Point{p.x, p.y}
                );
            }
        }

        if(prev_points_.empty())
        {
            prev_points_ = new_points;
            return;
        }

        IterativeClosestPoint icp(max_iteration_, radius_search_, tolerance_, neighbor_threshold_);

        const auto icp_result = icp.compute(new_points, prev_points_);
        prev_points_ = new_points;

        if(!icp_result)
        {
            RCLCPP_ERROR(this->get_logger(), "ICPが収束しませんでした。");
            return;
        }

        const auto t = icp.get_translation();
        const auto yaw = icp.get_yaw();

        auto transform_stamped = geometry_msgs::msg::TransformStamped();

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "odom";

        transform_stamped.transform.translation.x = t[0];
        transform_stamped.transform.translation.y = t[1];

        transform_stamped.transform.rotation.w = 0.0;
        transform_stamped.transform.rotation.x = 0.0;
        if(std::isfinite(yaw) && yaw != 0.0)
        {
            transform_stamped.transform.rotation.w = std::cos(yaw / 2.0);
            transform_stamped.transform.rotation.z = std::sin(yaw / 2.0);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "yawが非有限値です: %f", yaw);
            transform_stamped.transform.rotation.w = 1.0;
            transform_stamped.transform.rotation.z = 0.0;
        }

        tf_broadcaster_->sendTransform(transform_stamped);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tria_localization::LidarOnlyOdometer>());
    rclcpp::shutdown();
    return 0;
}