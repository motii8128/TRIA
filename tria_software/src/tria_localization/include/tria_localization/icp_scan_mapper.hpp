#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <message_filters/subscriber.h>

#include "p2p_icp.hpp"
#include "occupancy_grid_builder.hpp"
#include "mapcloud.hpp"

namespace tria_localization
{
    class IcpScanMapper : public rclcpp::Node
    {
        public:
        IcpScanMapper();

        void pointcloud_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

        private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_subscriber_;


        std::shared_ptr<OccupancyGridBuilder> builder_;
        std::shared_ptr<PointCloudMap> pc_map_;

        bool initalized_;

        int max_iteration_;
        double tolerance_;
        double neighbor_threshold_;
    };

    double to_radian(geometry_msgs::msg::Quaternion q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
}