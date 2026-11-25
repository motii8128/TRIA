#include "tria_auto_core/task_manager.hpp"

namespace tria_auto_core
{
    TaskManager::TaskManager() : Node("tria_auto_core")
    {
        // パラメータの取得  
        enable_arm_distance_ = this->declare_parameter("enable_arm_distance", 0.3);
        collect_horizontal_pos_ = this->declare_parameter("collect_horizontal_pos", -1.8);
        collect_vertical_pos_ = this->declare_parameter("collect_vertical_pos", -1.54);
        collect_hand_pos_ = this->declare_parameter("collect_hand_pos", -0.23);
        arm_length_ = this->declare_parameter("arm_length", 0.3);

        // 変数の初期化
        task_ = std::vector<tria_auto_msgs::msg::Task::SharedPtr>();
        task_.clear();
        target_pose_ = geometry_msgs::msg::PoseStamped();
        machine_target_ = MachineTarget{0.0, 0.0, 0.0};
        log_count_ = 0;
        current_delta_theta_ = 0.0;
        current_distance_ = 0.0;

        state_ = AutoState::WAIT;

        comp_manager_ = ComponentManager();

        // サブスクライバの初期化
        using std::placeholders::_1;
        task_subscriber_ = this->create_subscription<tria_auto_msgs::msg::Task>(
            "/task",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&TaskManager::task_callback, this, _1)
        );

        // タイマーの初期化
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&TaskManager::timer_callback, this)
        );

        // ActionClientの初期化
        wheel_action_client_ = rclcpp_action::create_client<WheelAction>(this, "/action/wheel");
        vertical_action_client_ = rclcpp_action::create_client<MachineAction>(this, "action/machine/vertical");
        horizontal_action_client_ = rclcpp_action::create_client<MachineAction>(this, "action/machine/horizontal");
        hand_action_client_ = rclcpp_action::create_client<MachineAction>(this, "action/machine/hand");

        // TFの初期化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
    }

    void TaskManager::timer_callback()
    {
        const auto enable_log = log_count_ > 1000;
        if(enable_log)log_count_ = 0;

        if(state_ == AutoState::WAIT)
        {
            new_task(enable_log);
        }
        else
        {
            progress_task(enable_log);
        }

        log_count_++;
    }

    void TaskManager::new_task(bool enable_log)
    {
        if(task_.empty())
        {
            if(enable_log)RCLCPP_INFO(this->get_logger(), "タスクがありません。");

            return;
        }

        auto current_position = geometry_msgs::msg::TransformStamped();
        try
        {
            current_position = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch(const tf2::TransformException &)
        {
            if(enable_log)RCLCPP_WARN(this->get_logger(), "自己位置を取得できません");
            return;
        }
        

        // 進行中のタスクに先頭のタスクを追加し、状態を「進行中」に変更する
        const auto progress_task = task_.front();
        task_.erase(task_.begin());

        const auto arm_position = progress_task->pose;
        const auto dx = arm_position.pose.position.x - current_position.transform.translation.x;
        const auto dy = arm_position.pose.position.y - current_position.transform.translation.y;
        const auto distance = std::sqrt(dx*dx + dy*dy);

        const auto target_angle = std::atan2(dy, dx);
        target_pose_.header.frame_id = "map";
        target_pose_.header.stamp = this->get_clock()->now();
        target_pose_.pose.position.x = current_position.transform.translation.x + std::cos(target_angle) * (distance - arm_length_);
        target_pose_.pose.position.y = current_position.transform.translation.y + std::sin(target_angle) * (distance - arm_length_);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, target_angle - M_PI / 2.0);
        target_pose_.pose.orientation.x = q.x();
        target_pose_.pose.orientation.y = q.y();
        target_pose_.pose.orientation.z = q.z();
        target_pose_.pose.orientation.w = q.w();

        if(progress_task->purpose == tria_auto_msgs::msg::Task::COLLECT)
        {
            state_ = AutoState::COLLECT;
        }
        else if(progress_task->purpose == tria_auto_msgs::msg::Task::RELEASE)
        {
            state_ = AutoState::RELEASE;
        }
        else if(progress_task->purpose == tria_auto_msgs::msg::Task::CALIBRATION)
        {
            target_pose_.pose.position.x = 0.0;
            target_pose_.pose.position.y = 0.0;
            q.setRPY(0.0, 0.0, 0.0);
            target_pose_.pose.orientation.x = q.x();
            target_pose_.pose.orientation.y = q.y();
            target_pose_.pose.orientation.z = q.z();
            target_pose_.pose.orientation.w = q.w();

            state_ = AutoState::CALIBRATION;
        }

        comp_manager_.reset();
    }

    void TaskManager::progress_task(bool enable_log)
    {
        auto vertical_goal_msg = MachineAction::Goal();
        auto horizontal_goal_msg = MachineAction::Goal();
        auto hand_goal_msg = MachineAction::Goal();

        if(state_ == AutoState::COLLECT)
        {
            vertical_goal_msg.target_position = collect_vertical_pos_;
            horizontal_goal_msg.target_position = collect_horizontal_pos_;
            hand_goal_msg.target_position = collect_hand_pos_;
        }
        else if(state_ == AutoState::RELEASE)
        {
            vertical_goal_msg.target_position = 0.0;
            horizontal_goal_msg.target_position = collect_horizontal_pos_;
            hand_goal_msg.target_position = 0.0;
        }
        else if(state_ == AutoState::CALIBRATION)
        {
            vertical_goal_msg.target_position = 0.0;
            horizontal_goal_msg.target_position = 0.0;
            hand_goal_msg.target_position = 0.0;
        }
        
        // 足回りの制御を開始していないなら開始
        if(!comp_manager_.wheel_started)
        {
            RCLCPP_INFO(this->get_logger(), "回収を開始します");
            auto wheel_goal_msg = WheelAction::Goal();
            wheel_goal_msg.pose = target_pose_;
            wheel_action_client_->async_send_goal(wheel_goal_msg);
            RCLCPP_INFO(this->get_logger(), "足回りの目標位置を送信しました");
            comp_manager_.wheel_started = true;
        }

        const auto distance_is_ok = current_distance_ < enable_arm_distance_;
        const auto delta_theta_is_ok = current_delta_theta_ < enable_arm_distance_;

        // 足回りの目標との誤差が閾値内かつ水平移動機構の制御を開始していないなら
        if(!comp_manager_.horizontal_started && distance_is_ok && delta_theta_is_ok)
        {
            horizontal_action_client_->async_send_goal(horizontal_goal_msg);
            RCLCPP_INFO(this->get_logger(), "水平移動機構の目標位置を送信しました");
            comp_manager_.horizontal_started = true;
        }

        // 水平移動機構まで制御が終了したら、昇降機構の動作を許可する
        if(!comp_manager_.vertical_started && comp_manager_.horizontal_reached)
        {
            vertical_action_client_->async_send_goal(vertical_goal_msg);
            RCLCPP_INFO(this->get_logger(), "昇降機構、ハンド機構の目標位置を送信しました");
            comp_manager_.vertical_started = true;
        }

        // 昇降機構まで制御が終了したら、ハンド機構の動作を許可する
        if(!comp_manager_.hand_started && comp_manager_.vertical_reached)
        {
            hand_action_client_->async_send_goal(hand_goal_msg);
            RCLCPP_INFO(this->get_logger(), "ハンド機構の目標位置を送信しました");
            comp_manager_.hand_started = true;
        }

        const auto collect_or_release = state_==AutoState::COLLECT || state_==AutoState::RELEASE;

        // 終了判定
        if(comp_manager_.complete())
        {
            comp_manager_.reset();
            // 回収もしくは設置モードなら、ハンド以外は原点に戻して終了する
            if(collect_or_release)
            {
                auto goal_msg = MachineAction::Goal();
                goal_msg.target_position = 0.0;

                vertical_action_client_->async_send_goal(goal_msg);
                horizontal_action_client_->async_send_goal(goal_msg);
            }

            state_ = AutoState::WAIT;
        }
    }

    void TaskManager::task_callback(const tria_auto_msgs::msg::Task::SharedPtr msg)
    {
        task_.push_back(msg);
        RCLCPP_INFO(this->get_logger(), "タスクを受信しました。 %s", to_string(msg).c_str());
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tria_auto_core::TaskManager>());
  rclcpp::shutdown();
  return 0;
}