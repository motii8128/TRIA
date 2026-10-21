#ifndef OMNI_UTILS_HPP_
#define OMNI_UTILS_HPP

#include <algorithm>
#include <array>
#include <cmath>

namespace wheel_controller
{
    using TargetSpeed = std::array<double, 3>;

    class Omni
    {
        public:
        Omni(
            const double body_radius, 
            const double max_speed,
            const double wheel1_angle,
            const double wheel2_angle,
            const double wheel3_angle
        )
        {
            body_radius_ = body_radius;
            max_speed_ = max_speed;

            angles[0] = wheel1_angle;
            angles[1] = wheel2_angle;
            angles[2] = wheel3_angle;
        }

        TargetSpeed calc(const double x_vel, const double y_vel, const double rotation_vel)
        {
            TargetSpeed target_speed{};

            double max_speed = 0.0;

            for(size_t i = 0; i < 3; i++)
            {
                const auto angle = angles[i];
                
                target_speed[i] = x_vel * cos(angle) + y_vel * sin(angle) + rotation_vel * body_radius_;
                
                if(target_speed[i] > max_speed)
                {
                    max_speed = target_speed[i];
                }
            }

            // 計算された最大速度が設定された最大速度を上回ったら、その車輪のみ最大速度となるように正規化する
            if(max_speed != 0.0 && max_speed > max_speed_)
            {
                for(size_t i = 0; i < 3; i++) target_speed[i] *=  max_speed_ / max_speed;
            }

            return target_speed;
        }

        private:
        double body_radius_;
        double max_speed_;
        double angles[3];
    };
}

#endif