#include "tria_localization/lidar_only_odometer.hpp"

namespace tria_localization
{
    LidarOnlyOdometer::LidarOnlyOdometer() : Node("lidar_only_odometer")
    {
        const auto max_iter = this->declare_parameter("max_iteration", 20);
        const auto tol = this->declare_parameter("tolerance", 0.001);
        const auto radi = this->declare_parameter("search_radius", 0.3);

        icp_ = std::make_shared<IterativeClosestPoint>(max_iter, radi, tol);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());

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
            new_points.emplace_back(
                Point{p.x, p.y}
            );
        }

        if(prev_points_.empty())
        {
            prev_points_ = new_points;
            return;
        }

        icp_->compute(new_points, prev_points_);

        const auto t = icp_->get_translation();
        const auto yaw = icp_->get_yaw();

        tf2::Quaternion q;
        q.setEuler(yaw, 0.0, 0.0);

        auto transform_stamped = geometry_msgs::msg::TransformStamped();
        auto odom_msg = nav_msgs::msg::Odometry();

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";

        odom_msg.header = transform_stamped.header;
        odom_msg.child_frame_id = transform_stamped.child_frame_id;

        transform_stamped.transform.translation.x = t[0];
        transform_stamped.transform.translation.y = t[1];

        transform_stamped.transform.rotation.w = q.w();
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();

        odom_msg.pose.pose.position.x = t[0];
        odom_msg.pose.pose.position.y = t[1];
        odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;

        tf_broadcaster_->sendTransform(transform_stamped);
        odom_publisher_->publish(odom_msg);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tria_localization::LidarOnlyOdometer>());
    rclcpp::shutdown();
    return 0;
}