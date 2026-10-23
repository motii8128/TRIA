#ifndef ROBOMASTER_UTILS_HPP_
#define ROBOMASTER_UTILS_HPP_

#include <cstdint>

#include <tria_msgs/msg/motor_feedback.hpp>

namespace device_communicator
{
    class RoboMaster
    {
    public:
        float velocity = 0.0f;
        float position = 0.0f;
        std::int16_t torque = 0;

        RoboMaster() = default;

        void update(std::int16_t velocity_in, std::int16_t angle_count, std::int16_t torque_in)
        {
            velocity = static_cast<float>(velocity_in);
            torque = torque_in;

            if (last_count_ == -1)
            {
                last_count_ = angle_count;
                position = 0.0f;
            }
            else
            {
                // int で計算するのでオーバーフローしない
                int delta_count = static_cast<int>(angle_count) - static_cast<int>(last_count_);

                last_count_ = angle_count;

                if (delta_count > 4096)
                {
                    if(velocity_in > 0)
                    {

                    }
                    else
                    {
                        delta_count -= 8192;
                    }
                }
                else if (delta_count < -4096)
                {
                    if(velocity_in < 0)
                    {

                    }
                    else
                    {
                        delta_count += 8192;
                    }
                }

                position += static_cast<float>(delta_count) / 8192.0f;
            }
        }

        tria_msgs::msg::MotorFeedback to_ROS2()
        {
            auto motor_feedback = tria_msgs::msg::MotorFeedback();

            motor_feedback.position = position;
            motor_feedback.rpm = (int16_t)velocity;
            motor_feedback.torque = torque;

            return motor_feedback;
        }

    private:
        std::int16_t last_count_ = -1;
    };
}

#endif