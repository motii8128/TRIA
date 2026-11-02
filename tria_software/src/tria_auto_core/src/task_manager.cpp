#include "tria_auto_core/task_manager.hpp"

namespace tria_auto_core
{
    TaskManager::TaskManager() : Node("tria_auto_core")
    {
        // パラメータの取得
        position_goal_threshold_ = this->declare_parameter("position_goal_threshold", 0.01);
        arm_goal_threshold_ = this->declare_parameter("arm_goal_threshold", 0.01);
        
        collect_horizontal_pos_ = this->declare_parameter("collect_horizontal_pos", -1.8);
        collect_vertical_pos_ = this->declare_parameter("collect_vertical_pos", -1.54);
        enable_arm_distance_ = this->declare_parameter("enable_arm_distance", 0.3);

        // ハンド系
        hand_goal_threshold_ = this->declare_parameter("hand_goal_threshold", 0.01);
        torque_threshold_ = this->declare_parameter("torque_threshold", 1000);
        collect_hand_pos_ = this->declare_parameter("collect_hand_pos", -0.23);

        // 変数の初期化
        task_ = std::vector<tria_auto_msgs::msg::Task::SharedPtr>();
        task_.clear();
        
        target_pose_ = geometry_msgs::msg::PoseStamped();
        target_machine_position_ = tria_msgs::msg::MachineControl();

        sensor_feedback_ = nullptr;
        state_ = State::WAIT;
        log_count_ = 0;

        // TFまわりの初期化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

        // サブスクライバの初期化
        using std::placeholders::_1;
        task_subscriber_ = this->create_subscription<tria_auto_msgs::msg::Task>(
            "/task",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&TaskManager::task_callback, this, _1)
        );

        sensor_feedback_subscriber_ = this->create_subscription<tria_msgs::msg::SensorFeedback>(
            "/ccb/sensor_feedback",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&TaskManager::sensor_feedback_callback, this, _1)
        );

        // タイマーの初期化
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&TaskManager::timer_callback, this)
        );

        // パブリッシャーの初期化
        machine_control_publisher_ = this->create_publisher<tria_msgs::msg::MachineControl>("/machine_control", rclcpp::SystemDefaultsQoS());
        pose_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::SystemDefaultsQoS());
    }

    void TaskManager::timer_callback()
    {
        const auto enable_log = log_count_ > 1000;
        if(enable_log)log_count_ = 0;

        // 状態に応じて別の関数を実行する
        if(state_ == State::WAIT)
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

            target_machine_position_.horizontal_position = 0.0;
            target_machine_position_.vertical_position = 0.0;

            machine_control_publisher_->publish(target_machine_position_);

            return;
        }

        if(sensor_feedback_ == nullptr)
        {
            if(enable_log)RCLCPP_INFO(this->get_logger(), "機構位置を取得できません。");
        }

        // 進行中のタスクに先頭のタスクを追加し、状態を「進行中」に変更する
        const auto progress_task = task_.front();
        task_.erase(task_.begin());

        target_pose_ = progress_task->pose;
        
        switch (progress_task->purpose)
        {
        case tria_auto_msgs::msg::Task::COLLECT:
            target_machine_position_.hand_position = 0.00000;
            target_machine_position_.horizontal_position = collect_horizontal_pos_;
            target_machine_position_.vertical_position = collect_vertical_pos_;
            break;
        
        case tria_auto_msgs::msg::Task::RELEASE:
            target_machine_position_.hand_position = collect_hand_pos_;
            target_machine_position_.horizontal_position = collect_horizontal_pos_;
            target_machine_position_.vertical_position = 0.0;

        case tria_auto_msgs::msg::Task::CALIBRATION:
            target_machine_position_.hand_position = 0.00000;
            target_machine_position_.horizontal_position = 0.0;
            target_machine_position_.vertical_position = 0.0;

        default:
            break;
        }

        state_ = State::POSITIONING;
    }

    void TaskManager::progress_task(bool enable_log)
    {
        try {
            // 自己位置の取得
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            // 現在位置と目標位置の距離
            const auto distance = p2p_distance(target_pose_.pose.position, transform_stamped.transform.translation);

            // 現在と目標の機構位置の差
            const auto dist_hand = target_machine_position_.hand_position - sensor_feedback_->hand.position;
            const auto dist_horizontal = target_machine_position_.horizontal_position - sensor_feedback_->horizontal.position;
            const auto dist_vertical = target_machine_position_.vertical_position - sensor_feedback_->vertical.position;
            const auto arm_distance = std::sqrt(dist_horizontal*dist_horizontal + dist_vertical * dist_vertical);

            switch (state_)
            {
                case State::POSITIONING:
                    if(enable_log)RCLCPP_INFO(this->get_logger(), "\t位置決めを行っています。距離: %f, アーム距離: %f", distance, arm_distance);
                    if(distance < position_goal_threshold_ && arm_distance < arm_goal_threshold_)
                    {
                        state_ = State::COLLECT_OR_RELEASE;
                        break;
                    }

                    if(distance < enable_arm_distance_)
                    {
                        machine_control_publisher_->publish(target_machine_position_);
                    }
                    pose_stamped_publisher_->publish(target_pose_);
                    break;

                case State::COLLECT_OR_RELEASE:
                    // 上昇機構の目標値が０でないなら回収であると判定する
                    if(target_machine_position_.vertical_position != 0.0)
                    {
                        if(enable_log)RCLCPP_INFO(this->get_logger(), "\t回収を行っています。");

                        if(sensor_feedback_->hand.torque < 0.0 && std::abs(sensor_feedback_->hand.torque) > torque_threshold_)
                        {
                            RCLCPP_INFO(this->get_logger(), "\t回収と判定しました。");
                            state_ = WAIT;
                            break;
                        }

                        target_machine_position_.hand_position = collect_hand_pos_;
                        
                        machine_control_publisher_->publish(target_machine_position_);
                    }
                    else
                    {
                        if(enable_log)RCLCPP_INFO(this->get_logger(), "\t設置を行っています。距離: %f, アーム距離: %f", distance, arm_distance);

                        if(dist_hand < hand_goal_threshold_)
                        {
                            RCLCPP_INFO(this->get_logger(), "\t設置と判定しました。");
                            state_ = WAIT;
                            break;
                        }
                        target_machine_position_.hand_position = 0.0;
                        machine_control_publisher_->publish(target_machine_position_);
                    }
                    
                
                default:
                    break;
            }
        }
        catch(const tf2::TransformException &ex)
        {
            if(enable_log)RCLCPP_WARN(this->get_logger(), "自己位置を取得できません");
        }
    }

    void TaskManager::task_callback(const tria_auto_msgs::msg::Task::SharedPtr msg)
    {
        task_.push_back(msg);
        RCLCPP_INFO(this->get_logger(), "タスクを受信しました。 %s", to_string(msg).c_str());
    }

    void TaskManager::sensor_feedback_callback(const tria_msgs::msg::SensorFeedback::SharedPtr msg)
    {
        sensor_feedback_ = msg;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tria_auto_core::TaskManager>());
  rclcpp::shutdown();
  return 0;
}