#include "tria_auto_core/pid_planner.hpp"

namespace tria_auto_core
{
    PidPlanner::PidPlanner() : Node("pid_planner")
    {
        PidConfig conf = PidConfig();
        conf.p_gain = this->declare_parameter("p_gain", 1.0);
        conf.i_gain = this->declare_parameter("i_gain", 0.0);
        conf.d_gain = this->declare_parameter("d_gain", 0.0);

        conf.enable_dynamic_integral_reset = true;
        conf.max_integral = this->declare_parameter("max_integral", 1.0);
        conf.max_output = this->declare_parameter("max_output", 1.0);

        x_pid_ = std::make_shared<PID>(conf);
        y_pid_ = std::make_shared<PID>(conf);
        rotation_pid_ = std::make_shared<PID>(conf);


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

        using std::placeholders::_1;
        target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&PidPlanner::target_pose_callback, this, _1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&PidPlanner::timer_callback, this)
        );

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());

        target_pose_ = nullptr;
        last_time_ = this->get_clock()->now();
    }

    void PidPlanner::timer_callback()
    {
        if(target_pose_ == nullptr) return;

        const auto now = this->get_clock()->now();
        const auto dt = (now - last_time_).seconds();  

        try
        {
            auto transform_stamped = geometry_msgs::msg::TransformStamped();
            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            auto cmd_msg = geometry_msgs::msg::Twist();

            const auto dx = target_pose_->pose.position.x - transform_stamped.transform.translation.x;
            const auto dy = target_pose_->pose.position.y - transform_stamped.transform.translation.y;
            const auto dtheta = to_radian(target_pose_->pose.orientation) - to_radian(transform_stamped.transform.rotation);

            auto vx = 0.0;
            auto vy = 0.0;
            if(std::abs(dx) > 0.01)
            {
                vx = x_pid_->compute(
                    target_pose_->pose.position.x,
                    transform_stamped.transform.translation.x,
                    dt
                );
            }
            else
            {
                vx = 0.0;
            }

            if(std::abs(dy) > 0.01)
            {
                vy = y_pid_->compute(
                    target_pose_->pose.position.y,
                    transform_stamped.transform.translation.y,
                    dt
                );
            }
            else
            {
                vy = 0.0;
            }

            if(std::abs(dtheta) > 0.01)
            {
                cmd_msg.angular.z = rotation_pid_->compute(
                    to_radian(target_pose_->pose.orientation),
                    to_radian(transform_stamped.transform.rotation),
                    dt
                );
            }
            else
            {
                cmd_msg.angular.z = 0.0;
            }

            const auto cos_yaw = std::cos(to_radian(transform_stamped.transform.rotation));
            const auto sin_yaw = std::sin(to_radian(transform_stamped.transform.rotation));
            cmd_msg.linear.x = vx * cos_yaw - vy * sin_yaw;
            cmd_msg.linear.y = vx * sin_yaw + vy * cos_yaw;

            twist_publisher_->publish(cmd_msg);
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "自己位置を取得できません");
        }
        
        last_time_ = now;
    }

    void PidPlanner::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = msg;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tria_auto_core::PidPlanner>());
  rclcpp::shutdown();
  return 0;
}