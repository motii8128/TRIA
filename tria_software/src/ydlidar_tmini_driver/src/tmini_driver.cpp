// 複数台の YDLIDAR T-mini (Plus / Pro) を同時に扱い、機種ごとに別トピックへ
// sensor_msgs/LaserScan を publish する ROS 2 ノード。
//
// 各シリアルポートを開いたあと、デバイス情報 (A5 90) の model 番号で機種を判定し、
// 対応する Plus / Pro のトピックへ割り当てる。ttyUSB の番号が入れ替わっても取り違えない。
//
// シリアル読み取りはポートごとの専用スレッドで行う (Lidar::readScan() はブロックするため)。
// 切断・タイムアウト・未割り当ての model などで失敗したら、一定間隔で再接続を試みる。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ydlidar_tmini_driver/laserscan_converter.hpp"
#include "ydlidar_tmini_driver/lidar.hpp"

namespace {
// readScan() の読み取りタイムアウトは 500 ms。これが連続したら (約 5 秒) 再接続する。
constexpr int kMaxConsecutiveTimeouts = 10;
}  // namespace

class TminiNode : public rclcpp::Node {
public:
    TminiNode() : rclcpp::Node("tmini_lidar") {
        ports_ = declare_parameter<std::vector<std::string>>(
            "ports", std::vector<std::string>{"/dev/ttyUSB0", "/dev/ttyUSB1"});
        scan_frequency_hz_ = declare_parameter<double>("scan_frequency_hz", 0.0);
        retry_interval_s_ = declare_parameter<double>("retry_interval_s", 2.0);

        ydlidar_tmini::LaserScanConfig cfg;  // 両機種共通の設定
        cfg.angle_increment_deg =
            static_cast<float>(declare_parameter<double>("angle_increment_deg", 1.0));
        cfg.range_min_m = static_cast<float>(declare_parameter<double>("range_min", 0.05));
        cfg.range_max_m = static_cast<float>(declare_parameter<double>("range_max", 12.0));
        cfg.with_intensity = declare_parameter<bool>("publish_intensity", true);
        const bool plus_drop_interference = declare_parameter<bool>("plus_drop_interference", false);

        // Plus の model は実機で 151 を確認済み。Pro の値は未確認なので未設定 (-1) にしてある:
        // 起動すると「model N は未割り当て」とログに出るので、その値を pro_model に設定する。
        channels_.push_back(makeChannel("plus", 151, "tmini_plus/scan", "tmini_plus_laser_frame", cfg,
                                        plus_drop_interference));
        channels_.push_back(
            makeChannel("pro", -1, "tmini_pro/scan", "tmini_pro_laser_frame", cfg, false));

        if (ports_.empty()) {
            RCLCPP_ERROR(get_logger(), "parameter `ports` is empty; nothing to do");
        }
        for (const std::string& port : ports_) {
            workers_.emplace_back([this, port] { runPort(port); });
        }
    }

    ~TminiNode() override {
        running_ = false;
        for (auto& t : workers_) {
            if (t.joinable()) t.join();
        }
    }

private:
    /// 機種 (Plus / Pro) ごとの出力先。model 番号で割り当てる。
    struct Channel {
        std::string label;  // "plus" / "pro"
        int64_t model = -1;  // -1 = 未設定 (どの機種ともマッチしない)
        std::string topic;
        std::string frame_id;
        float range_min = 0.05f;
        float range_max = 12.0f;
        std::unique_ptr<ydlidar_tmini::LaserScanConverter> converter;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
        std::atomic<bool> in_use{false};  // 1 つのデバイスだけが使える
    };

    /// 1 つの接続の間だけ Channel を専有する (同じ機種が 2 台つながった場合の検出用)。
    struct ChannelClaim {
        explicit ChannelClaim(Channel& c) : ch(c) {
            bool expected = false;
            if (!ch.in_use.compare_exchange_strong(expected, true)) {
                throw std::runtime_error("another device is already publishing as '" + ch.label +
                                         "' (do two devices report the same model number?)");
            }
        }
        ~ChannelClaim() { ch.in_use = false; }
        ChannelClaim(const ChannelClaim&) = delete;
        ChannelClaim& operator=(const ChannelClaim&) = delete;
        Channel& ch;
    };

    /// 1 周の周期の推定用 (接続ごとに持つ)。
    struct Timing {
        bool have_last = false;
        std::chrono::steady_clock::time_point last;
        double period_s = 0.0;
    };

    std::unique_ptr<Channel> makeChannel(const std::string& label, int64_t default_model,
                                         const std::string& default_topic,
                                         const std::string& default_frame,
                                         ydlidar_tmini::LaserScanConfig cfg, bool drop_interference) {
        auto ch = std::make_unique<Channel>();
        ch->label = label;
        ch->model = declare_parameter<int64_t>(label + "_model", default_model);
        ch->topic = declare_parameter<std::string>(label + "_topic", default_topic);
        ch->frame_id = declare_parameter<std::string>(label + "_frame_id", default_frame);
        cfg.angle_offset_deg =
            static_cast<float>(declare_parameter<double>(label + "_angle_offset_deg", 0.0));
        cfg.drop_interference = drop_interference;

        ch->range_min = cfg.range_min_m;
        ch->range_max = cfg.range_max_m;
        ch->converter = std::make_unique<ydlidar_tmini::LaserScanConverter>(cfg);
        // reliable にしておけば reliable / best-effort どちらの購読側ともつながる。
        ch->pub = create_publisher<sensor_msgs::msg::LaserScan>(ch->topic,
                                                                rclcpp::SensorDataQoS().reliable());
        RCLCPP_INFO(get_logger(), "%s: model=%lld topic=%s frame=%s bins=%zu", label.c_str(),
                    static_cast<long long>(ch->model), ch->topic.c_str(), ch->frame_id.c_str(),
                    ch->converter->numBins());
        return ch;
    }

    Channel* findChannel(int64_t model) {
        for (auto& ch : channels_) {
            if (ch->model >= 0 && ch->model == model) return ch.get();
        }
        return nullptr;
    }

    void runPort(const std::string& port) {
        while (running_ && rclcpp::ok()) {
            try {
                connectAndScan(port);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "[%s] %s (retry in %.1f s)", port.c_str(), e.what(),
                             retry_interval_s_);
                sleepInterruptible(retry_interval_s_);
            }
        }
    }

    void connectAndScan(const std::string& port) {
        ydlidar_tmini::Lidar lidar(port);
        const ydlidar_tmini::DeviceInfo info = lidar.deviceInfo();
        RCLCPP_INFO(get_logger(), "[%s] connected: %s", port.c_str(), info.toString().c_str());

        Channel* ch = findChannel(info.model);
        if (ch == nullptr) {
            throw std::runtime_error("model " + std::to_string(info.model) +
                                     " is not mapped to any topic; set plus_model or pro_model to " +
                                     std::to_string(info.model));
        }
        ChannelClaim claim(*ch);
        RCLCPP_INFO(get_logger(), "[%s] identified as %s -> %s", port.c_str(), ch->label.c_str(),
                    ch->topic.c_str());

        if (!lidar.health().isOk()) {
            RCLCPP_WARN(get_logger(), "[%s] lidar reports a health problem", port.c_str());
        }
        applyScanFrequency(lidar, port);

        lidar.startScan();
        RCLCPP_INFO(get_logger(), "[%s] scanning started", port.c_str());

        Timing timing;
        int consecutive_timeouts = 0;
        while (running_ && rclcpp::ok()) {
            ydlidar_tmini::Scan scan;
            try {
                scan = lidar.readScan();
                consecutive_timeouts = 0;
            } catch (const ydlidar_tmini::TimeoutError&) {
                if (++consecutive_timeouts >= kMaxConsecutiveTimeouts) {
                    throw std::runtime_error("no data from the lidar for several seconds");
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s] waiting for lidar data...",
                                     port.c_str());
                continue;
            }
            publishScan(*ch, scan, timing);
        }
        lidar.stopScan();
    }

    /// scan_frequency_hz > 0 のとき、スキャン開始前に回転周波数を目標へ寄せる。
    /// コマンドが効かなくても致命的ではないので、失敗は警告にとどめる。
    void applyScanFrequency(ydlidar_tmini::Lidar& lidar, const std::string& port) {
        if (scan_frequency_hz_ <= 0.0) return;
        const float target = static_cast<float>(scan_frequency_hz_);
        try {
            float f = lidar.scanFrequency();
            for (int i = 0; i < 30; ++i) {
                const float diff = target - f;
                if (std::fabs(diff) < 0.05f) break;

                ydlidar_tmini::FreqStep step;
                if (diff >= 0.95f) step = ydlidar_tmini::FreqStep::CoarseUp;
                else if (diff <= -0.95f) step = ydlidar_tmini::FreqStep::CoarseDown;
                else if (diff > 0.0f) step = ydlidar_tmini::FreqStep::FineUp;
                else step = ydlidar_tmini::FreqStep::FineDown;

                const float next = lidar.adjustScanFrequency(step);
                if (std::fabs(next - f) < 1e-3f) {
                    RCLCPP_WARN(get_logger(), "[%s] scan frequency did not change (stuck at %.2f Hz)",
                                port.c_str(), f);
                    break;
                }
                f = next;
            }
            RCLCPP_INFO(get_logger(), "[%s] scan frequency: %.2f Hz (target %.2f Hz)", port.c_str(), f,
                        target);
        } catch (const ydlidar_tmini::Error& e) {
            RCLCPP_WARN(get_logger(), "[%s] could not set scan frequency: %s", port.c_str(), e.what());
        }
    }

    void publishScan(Channel& ch, const ydlidar_tmini::Scan& scan, Timing& t) {
        // 1 周の周期: 完了間隔の移動平均。最初の周だけ CT の値 (無ければ 0.1 s) で代用。
        const auto now_steady = std::chrono::steady_clock::now();
        if (t.have_last) {
            const double dt = std::chrono::duration<double>(now_steady - t.last).count();
            if (dt > 0.02 && dt < 1.0) {
                t.period_s = t.period_s > 0.0 ? 0.8 * t.period_s + 0.2 * dt : dt;
            }
        } else if (t.period_s <= 0.0) {
            t.period_s = (scan.scan_freq_hz && *scan.scan_freq_hz > 0.0f)
                             ? 1.0 / static_cast<double>(*scan.scan_freq_hz)
                             : 0.1;
        }
        t.last = now_steady;
        t.have_last = true;

        ydlidar_tmini::LaserScanData data = ch.converter->convert(scan);

        auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        // 1 周が完了した時刻 - 周期 ≒ 計測開始時刻。シリアルの遅延は補正していない。
        msg->header.stamp = now() - rclcpp::Duration::from_seconds(t.period_s);
        msg->header.frame_id = ch.frame_id;
        msg->angle_min = data.angle_min;
        msg->angle_max = data.angle_max;
        msg->angle_increment = data.angle_increment;
        msg->scan_time = static_cast<float>(t.period_s);
        msg->time_increment = static_cast<float>(t.period_s / static_cast<double>(data.ranges.size()));
        msg->range_min = ch.range_min;
        msg->range_max = ch.range_max;
        msg->ranges = std::move(data.ranges);
        msg->intensities = std::move(data.intensities);
        ch.pub->publish(std::move(msg));
    }

    void sleepInterruptible(double seconds) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < end) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::vector<std::string> ports_;
    double scan_frequency_hz_ = 0.0;
    double retry_interval_s_ = 2.0;
    std::vector<std::unique_ptr<Channel>> channels_;

    std::vector<std::thread> workers_;
    std::atomic<bool> running_{true};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<TminiNode>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}