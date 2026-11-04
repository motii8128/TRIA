#include "tria_localization/icp_scan_mapper.hpp"

namespace tria_localization
{
    IcpScanMapper::IcpScanMapper() : Node("icp_scan_mapper")
    {
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;

        max_iteration_ = this->declare_parameter("max_iteration", 20);
        tolerance_ = this->declare_parameter("tolerance", 0.001);
        neighbor_threshold_ = this->declare_parameter("neighbor_threshold", 0.2);

        GridParams params;
        params.resolution = this->declare_parameter("resolution", 0.05);
        params.min_range = this->declare_parameter("min_range", 0.1);
        params.max_range = this->declare_parameter("max_range", 12.0);
        params.l_occ = this->declare_parameter("l_occ", 0.85);
        params.l_free = this->declare_parameter("l_free", -0.4);
        params.l_min = this->declare_parameter("l_min", -2.0);
        params.l_max = this->declare_parameter("l_max", 3.5);

        params.occupied_threshold = this->declare_parameter("occupied_threshold", 0.65);
        params.free_threshold = this->declare_parameter("free_threshold", 0.35);

        MapParams map_params;
        map_params.max_range = params.max_range;
        map_params.min_range = params.min_range;
        map_params.max_count = 20;
        map_params.voxel_size = params.resolution;
        pc_map_ = std::make_shared<PointCloudMap>(map_params);

        builder_ = std::make_shared<OccupancyGridBuilder>(params);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        using std::placeholders::_1;
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/merged_points",
            rclcpp::SystemDefaultsQoS().reliable(),
            std::bind(&IcpScanMapper::pointcloud_callback, this, _1)
        );

        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local().reliable());

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());

        initalized_ = false;

        RCLCPP_INFO(this->get_logger(), "IcpScanMapperが初期化されました");
    }

    void IcpScanMapper::pointcloud_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        if(!initalized_)
        {
            Points new_points(0);
            for(const auto &ros_point : msg->points)
            {
                new_points.push_back(Point{ros_point.x, ros_point.y});
            }
            pc_map_->add_scan(new_points, 0.0, 0.0, 0.0);
            builder_->add_scan(new_points, 0.0, 0.0, 0.0);

            initalized_ = true;

            return;
        }

        try
        {
            auto transform_stamped = geometry_msgs::msg::TransformStamped();
            transform_stamped = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

            const auto x = transform_stamped.transform.translation.x;
            const auto y = transform_stamped.transform.translation.y;
            const auto yaw = to_radian(transform_stamped.transform.rotation);
            

            Point_to_Point icp(max_iteration_, tolerance_, neighbor_threshold_);
            icp.set_pose(x_ + x, y_ + y, yaw_ + yaw);

            Points new_points(0);
            for(const auto &ros_point : msg->points)
            {
                new_points.push_back(Point{ros_point.x, ros_point.y});
            }

            Points target = pc_map_->points();
            const auto icp_result = icp.compute(new_points, target);

            if(!icp_result)
            {
                RCLCPP_ERROR(this->get_logger(), "ICPが収束しませんでした");
                return;
            }
            const auto icp_trans = icp.get_translation();
            const auto icp_yaw = icp.get_yaw();
            x_ = icp_trans[0];
            y_ = icp_trans[1];
            yaw_ = icp_yaw;

            builder_->add_scan(new_points, icp_trans[0], icp_trans[1], icp_yaw);

            auto map_msg = nav_msgs::msg::OccupancyGrid();

            to_occupancy_grid_msg(builder_->to_grid(), map_msg);
            map_msg.header = msg->header;
            map_publisher_->publish(map_msg);

            auto icp_tf = geometry_msgs::msg::TransformStamped();

            icp_tf.header.stamp = this->get_clock()->now();
            icp_tf.header.frame_id = "map";
            icp_tf.child_frame_id = "base_link";

            icp_tf.transform.translation.x = icp_trans[0];
            icp_tf.transform.translation.y = icp_trans[1];

            icp_tf.transform.rotation.w = 0.0;
            icp_tf.transform.rotation.x = 0.0;
            if(std::isfinite(icp_yaw) && icp_yaw != 0.0)
            {
                icp_tf.transform.rotation.w = std::cos(icp_yaw / 2.0);
                icp_tf.transform.rotation.z = std::sin(icp_yaw / 2.0);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "yawが非有限値です: %f", icp_yaw);
                icp_tf.transform.rotation.w = 1.0;
                icp_tf.transform.rotation.z = 0.0;
            }

            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header = icp_tf.header;
            odom_msg.child_frame_id = icp_tf.child_frame_id;
            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            odom_msg.pose.pose.orientation = icp_tf.transform.rotation;
            odom_publisher_->publish(odom_msg);
            tf_broadcaster_->sendTransform(icp_tf);
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "自己位置を取得できません");
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tria_localization::IcpScanMapper>());
    rclcpp::shutdown();
    return 0;
}