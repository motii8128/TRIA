#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "p2l_icp.hpp"

namespace tria_localization
{
    class LidarOnlyOdometer : public rclcpp::Node
    {
        public:
        LidarOnlyOdometer();

        void pointcloud_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

        private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_subscriber_;

        int max_iteration_;
        double tolerance_;
        double radius_search_;
        double neighbor_threshold_;

        double x_, y_, yaw_;

        Points prev_points_;
    };
}