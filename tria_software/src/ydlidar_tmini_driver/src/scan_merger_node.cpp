// 複数台の LaserScan を、センサごとの取り付け位置・向き・傾き (パラメータ) で共通フレームへ変換し、
// 1 つの sensor_msgs/PointCloud2 に統合して publish する ROS 2 ノード。
//
// - 固定周期 (publish_rate_hz) で、各センサの「最新の scan」を統合して出力する。
// - max_age_s より古い scan は使わない (センサが止まっても、生きている側だけで出力を続ける)。
// - 前回の出力以降に新しい scan が 1 本も来ていなければ出力しない (同じデータの再送を避ける)。
// - TF は使わず、姿勢はすべてパラメータで与える (座標規約は scan_merger.hpp を参照)。
// - 回転中の自己運動の補正 (デスキュー) はしていない。
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "ydlidar_tmini_driver/scan_merger.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
}

class ScanMergerNode : public rclcpp::Node {
public:
    ScanMergerNode() : rclcpp::Node("tmini_merge") {
        target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
        double rate = declare_parameter<double>("publish_rate_hz", 10.0);
        max_age_s_ = declare_parameter<double>("max_age_s", 0.25);
        const std::vector<std::string> names = declare_parameter<std::vector<std::string>>(
            "sensors", std::vector<std::string>{"plus", "pro"});
        if (rate <= 0.0) rate = 10.0;

        for (const std::string& name : names) {
            auto s = std::make_unique<Sensor>();
            s->name = name;
            s->topic = declare_parameter<std::string>(name + ".topic", "tmini_" + name + "/scan");

            ydlidar_tmini::MountPose pose;
            pose.x = declare_parameter<double>(name + ".x", 0.0);
            pose.y = declare_parameter<double>(name + ".y", 0.0);
            pose.z = declare_parameter<double>(name + ".z", 0.0);
            pose.roll_rad = declare_parameter<double>(name + ".roll_deg", 0.0) * kPi / 180.0;
            pose.pitch_rad = declare_parameter<double>(name + ".pitch_deg", 0.0) * kPi / 180.0;
            pose.yaw_rad = declare_parameter<double>(name + ".yaw_deg", 0.0) * kPi / 180.0;
            s->transform = ydlidar_tmini::RigidTransform(pose);

            Sensor* raw = s.get();
            s->sub = create_subscription<sensor_msgs::msg::LaserScan>(
                s->topic, rclcpp::SensorDataQoS(),
                [this, raw](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                    onScan(*raw, std::move(msg));
                });
            RCLCPP_INFO(get_logger(),
                        "sensor '%s': topic=%s pos=(%.3f, %.3f, %.3f) rpy=(%.1f, %.1f, %.1f) deg",
                        name.c_str(), s->topic.c_str(), pose.x, pose.y, pose.z,
                        pose.roll_rad * 180.0 / kPi, pose.pitch_rad * 180.0 / kPi,
                        pose.yaw_rad * 180.0 / kPi);
            sensors_.push_back(std::move(s));
        }
        if (sensors_.empty()) {
            RCLCPP_ERROR(get_logger(), "parameter `sensors` is empty; nothing to merge");
        }

        pub_ = create_publisher<sensor_msgs::msg::PointCloud>("merged_points",
                                                               rclcpp::SensorDataQoS().reliable());
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                                   [this] { publishMerged(); });
        RCLCPP_INFO(get_logger(), "merging %zu sensors into '%s' at %.1f Hz -> merged_points",
                    sensors_.size(), target_frame_.c_str(), rate);
    }

private:
    struct Sensor {
        std::string name;
        std::string topic;
        ydlidar_tmini::RigidTransform transform;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
        // 以下は mutex_ で保護
        sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
        rclcpp::Time received{0, 0, RCL_ROS_TIME};
        bool fresh = false;  // 前回の出力以降に新しい scan が来た
    };

    void onScan(Sensor& s, sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        s.scan = std::move(msg);
        s.received = now();
        s.fresh = true;
    }

    void publishMerged() {
        std::vector<ydlidar_tmini::CloudPoint> points;
        int64_t newest_ns = 0;
        bool any_fresh = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const rclcpp::Time t_now = now();
            for (auto& sp : sensors_) {
                Sensor& s = *sp;
                if (!s.scan) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "sensor '%s': no scan received yet on %s", s.name.c_str(),
                                         s.topic.c_str());
                    continue;
                }
                const double age = (t_now - s.received).seconds();
                if (age > max_age_s_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "sensor '%s': last scan is %.2f s old, excluded from the merge",
                                         s.name.c_str(), age);
                    continue;
                }
                ydlidar_tmini::appendScanPoints(s.transform, s.scan->angle_min,
                                                s.scan->angle_increment, s.scan->range_min,
                                                s.scan->range_max, s.scan->ranges,
                                                s.scan->intensities, points);
                any_fresh = any_fresh || s.fresh;
                s.fresh = false;
                const int64_t ns = static_cast<int64_t>(s.scan->header.stamp.sec) * 1000000000LL +
                                   static_cast<int64_t>(s.scan->header.stamp.nanosec);
                if (ns > newest_ns) newest_ns = ns;
            }
        }
        if (!any_fresh) return;  // 新しいデータが無ければ再送しない

        auto msg = std::make_unique<sensor_msgs::msg::PointCloud>();
        if (newest_ns <= 0) newest_ns = now().nanoseconds();  // scan に時刻が入っていない場合
        msg->header.stamp.sec = static_cast<int32_t>(newest_ns / 1000000000LL);
        msg->header.stamp.nanosec = static_cast<uint32_t>(newest_ns % 1000000000LL);
        msg->header.frame_id = target_frame_;

        for(const auto &point : points)
        {
            auto ros_point = geometry_msgs::msg::Point32();
            ros_point.x = point.x;
            ros_point.y = point.y;
            ros_point.z = point.z;

            msg->points.push_back(ros_point);
        }
        pub_->publish(std::move(msg));
    }

    std::string target_frame_;
    double max_age_s_ = 0.25;
    std::vector<std::unique_ptr<Sensor>> sensors_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMergerNode>());
    rclcpp::shutdown();
    return 0;
}