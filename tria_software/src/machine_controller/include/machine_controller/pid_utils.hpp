#ifndef PID_UTILS_HPP_
#define PID_UTILS_HPP_

#include <cmath>
#include <type_traits>

namespace machine_controller
{
    struct PidConfig
    {
        float p_gain, i_gain, d_gain;
        float max_output;
        float max_integral;

        bool enable_dynamic_integral_reset;

        PidConfig():p_gain(0.0),i_gain(0.0),d_gain(0.0),
        max_output(std::numeric_limits<float>::max()),
        max_integral(std::numeric_limits<float>::max()),
        enable_dynamic_integral_reset(true)
        {

        }
    };

    class PID
    {
        public:
        PID() = default;

        PID(PidConfig config)
        {
            config_ = config;
            prev_prop_ = 0.0;
            prev_mesurement_ = 0.0;
            integral_ = 0.0;
        }

        float compute(const float &target, const float &now, const float &delta_time)
        {
            const auto prop = target - now;

            if(config_.enable_dynamic_integral_reset) dynamic_integral_reset(prop);

            integral_ += prop * delta_time;

            if(fabs(integral_) > config_.max_integral)
            {
                if(integral_ > 0.0)integral_ = config_.max_integral;
                if(integral_ < 0.0)integral_ = -1.0 * config_.max_integral;
            }

            const auto derivative = (prop - prev_prop_) / delta_time;
            prev_prop_ = prop;

            const auto out = config_.p_gain * prop + config_.i_gain * integral_ + config_.d_gain * derivative;

            return clamp(out);
        }

        float compute_derivative_ahead(const float &target, const float &now, const float &delta_time)
        {
            const auto prop = target - now;

            if(config_.enable_dynamic_integral_reset) dynamic_integral_reset(prop);

            integral_ += prop * delta_time;

            if(fabs(integral_) > config_.max_integral)
            {
                if(integral_ > 0.0)integral_ = config_.max_integral;
                if(integral_ < 0.0)integral_ = -1.0 * config_.max_integral;
            }

            const auto derivative = (now - prev_mesurement_) / delta_time;
            prev_mesurement_ = now;

            const auto out = config_.p_gain * prop + config_.i_gain * integral_ + config_.d_gain * derivative;

            return clamp(out);
        }

        private:
        void dynamic_integral_reset(const float& prop)
        {
            if(prop * prev_prop_ < 0.0)
            {
                integral_ = 0.0;
            }
        }

        float clamp(const float& value)
        {
            if(fabs(value) > config_.max_output)
            {
                if(value > 0.0) return config_.max_output;
                if(value < 0.0) return -1.0 * config_.max_output;
            }
            else
            {
                return value;
            }
        }
        float prev_prop_;
        float prev_mesurement_;
        float integral_;
        PidConfig config_;
    };

    class CascadeController
    {
        public:
        CascadeController() = default;

        CascadeController(PidConfig p_config, PidConfig s_config)
        {
            p_pid_ = PID(p_config);
            s_pid_ = PID(s_config);
        }

        float compute(const float& target_position, const float& now_position, const float& now_rpm, const float& delta_time)
        {
            const auto target_rpm = p_pid_.compute(target_position, now_position, delta_time);

            return s_pid_.compute(target_rpm, now_rpm, delta_time);
        }

        private:
        PID p_pid_;
        PID s_pid_;
    };
}

#endif