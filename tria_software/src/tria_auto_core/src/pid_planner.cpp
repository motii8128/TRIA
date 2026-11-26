#include "tria_auto_core/pid_planner.hpp"

namespace tria_auto_core
{
    PidPlanner::PidPlanner() : Node("pid_planner")
    {
        PidConfig conf = PidConfig();
        conf.p_gain = this->declare_parameter("linear.p_gain", 1.0);
        conf.i_gain = this->declare_parameter("linear.i_gain", 0.0);
        conf.d_gain = this->declare_parameter("linear.d_gain", 0.0);

        conf.enable_dynamic_integral_reset = true;
        conf.max_integral = this->declare_parameter("linear.max_integral", 1.0);
        conf.max_output = this->declare_parameter("linear.max_output", 1.0);

        x_pid_ = std::make_shared<PID>(conf);
        y_pid_ = std::make_shared<PID>(conf);


        conf.p_gain = this->declare_parameter("angular.p_gain", 1.0);
        conf.i_gain = this->declare_parameter("angular.i_gain", 0.0);
        conf.d_gain = this->declare_parameter("angular.d_gain", 0.0);

        conf.enable_dynamic_integral_reset = true;
        conf.max_integral = this->declare_parameter("angular.max_integral", 1.0);
        conf.max_output = this->declare_parameter("angular.max_output", 1.0);
        rotation_pid_ = std::make_shared<PID>(conf);

        linear_threshold_ = this->declare_parameter("linear_threshold", 0.01);
        angular_threshold_ = this->declare_parameter("angular_threshold", 0.01);


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

        action_server_ = rclcpp_action::create_server<WheelAction>(
            this,
            "wheel_action",
            std::bind(&PidPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PidPlanner::handle_cancel, this, std::placeholders::_1),
            std::bind(&PidPlanner::handle_accepted, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&PidPlanner::timer_callback, this)
        );

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());

        target_pose_ = geometry_msgs::msg::PoseStamped();
        active_goal_handle_ = nullptr;
        last_time_ = this->get_clock()->now();
    }

    void PidPlanner::timer_callback()
    {
        const auto now = this->get_clock()->now();
        const auto dt = (now - last_time_).seconds();
        
        if(active_goal_handle_ == nullptr || !active_goal_handle_->is_active())
        {
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = 0.0;
            cmd_msg.linear.y = 0.0;
            cmd_msg.angular.z = 0.0;

            twist_publisher_->publish(cmd_msg);
            return;
        }

        try
        {
            auto transform_stamped = geometry_msgs::msg::TransformStamped();
            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            const auto cos_yaw = std::cos(to_radian(transform_stamped.transform.rotation));
            const auto sin_yaw = std::sin(to_radian(transform_stamped.transform.rotation));

            auto cmd_msg = geometry_msgs::msg::Twist();

            const auto dx = target_pose_.pose.position.x - transform_stamped.transform.translation.x;
            const auto dy = target_pose_.pose.position.y - transform_stamped.transform.translation.y;
            const auto dtheta = std::abs(to_radian(target_pose_.pose.orientation) - to_radian(transform_stamped.transform.rotation));

            const auto distance = std::sqrt(dx*dx + dy*dy);

            auto vx = x_pid_->compute(
                target_pose_.pose.position.x,
                transform_stamped.transform.translation.x,
                dt
            );
            auto vy = y_pid_->compute(
                target_pose_.pose.position.y,
                transform_stamped.transform.translation.y,
                dt
            );

            cmd_msg.angular.z = rotation_pid_->compute(
                to_radian(target_pose_.pose.orientation),
                to_radian(transform_stamped.transform.rotation),
                dt
            );
            
            cmd_msg.linear.x = vx * cos_yaw + vy * sin_yaw;
            cmd_msg.linear.y = -vx * sin_yaw + vy * cos_yaw;

            twist_publisher_->publish(cmd_msg);

            auto feedback = std::make_shared<WheelAction::Feedback>();
            feedback->distance = distance;
            feedback->delta_theta = dtheta;
            active_goal_handle_->publish_feedback(feedback);

            if(distance < linear_threshold_ && dtheta < angular_threshold_)
            {
                auto cmd_msg = geometry_msgs::msg::Twist();
                cmd_msg.linear.x = 0.0;
                cmd_msg.linear.y = 0.0;
                cmd_msg.angular.z = 0.0;

                twist_publisher_->publish(cmd_msg);

                auto result = std::make_shared<WheelAction::Result>();
                result->distance = distance;
                result->delta_theta = dtheta;
                active_goal_handle_->succeed(result);
                active_goal_handle_ = nullptr;

                RCLCPP_INFO(this->get_logger(), "制御完了 誤差:%.4f", distance);
            }
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "自己位置を取得できません");
        }
        
        last_time_ = now;
    }

    rclcpp_action::GoalResponse PidPlanner::handle_goal(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const WheelAction::Goal> goal
    )
    {        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // GoalをAcceptした際に呼び出される
    void PidPlanner::handle_accepted(const std::shared_ptr<GoalHandleWheel> goal_handle)
    {
        active_goal_handle_ = goal_handle;
        target_pose_ = active_goal_handle_->get_goal()->pose;

        RCLCPP_INFO(this->get_logger(), "新たな目標位置を取得しました");
    }

    // キャンセルを受信したら呼び出される
    rclcpp_action::CancelResponse PidPlanner::handle_cancel(const std::shared_ptr<GoalHandleWheel>)
    {
        RCLCPP_INFO(this->get_logger(), "キャンセルのリクエストを受け付けました");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tria_auto_core::PidPlanner>());
  rclcpp::shutdown();
  return 0;
}