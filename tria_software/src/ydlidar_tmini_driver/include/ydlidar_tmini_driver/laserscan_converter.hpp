// ydlidar_tmini::Scan (センサ座標・不等間隔の点群) を、
// 固定ビンの LaserScan 形式 (ranges / intensities) に変換する。ROS 非依存。
//
// - 角度規約: センサ出力は時計回りに増加するので、LaserScan 規約 (反時計回り正,
//   x 前方) に反転する。angle_min = -pi, ビン数 N = 360 / angle_increment_deg。
// - 同じビンに複数の点が入ったら最も近い点を採用する (障害物回避に保守的)。
// - 点が無い / 無効 / 測距範囲外のビンは +inf。
#ifndef YDLIDAR_LASERSCAN_CONVERTER_HPP_
#define YDLIDAR_LASERSCAN_CONVERTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "protocol.hpp"

namespace ydlidar_tmini {

struct LaserScanConfig {
    /// ビンの角度刻み [deg]。1 周の点数 (4000 / 回転周波数) より粗くすること。
    /// 例: 10 Hz → 約 400 点/周 (0.9°) なので 1.0° でよい。12 Hz → 1.2° を推奨。
    float angle_increment_deg = 1.0f;
    /// センサ 0° 方向をフレームの x 軸からずらす量 (反時計回り正) [deg]。取り付け向きの補正用。
    float angle_offset_deg = 0.0f;
    float range_min_m = 0.05f;
    float range_max_m = 12.0f;
    /// true なら flag 2/3 の点を除外する (T-mini Plus 専用。Pro では使わない)。
    bool drop_interference = false;
    bool with_intensity = true;
};

struct LaserScanData {
    float angle_min = 0.0f;
    float angle_max = 0.0f;
    float angle_increment = 0.0f;  // [rad]
    std::vector<float> ranges;     // [m]
    std::vector<float> intensities;
};

class LaserScanConverter {
public:
    explicit LaserScanConverter(const LaserScanConfig& cfg) : cfg_(cfg) {
        const double inc = cfg.angle_increment_deg > 0.0f ? cfg.angle_increment_deg : 1.0;
        n_ = std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(360.0 / inc)));
        increment_rad_ = 2.0 * kPi / static_cast<double>(n_);
    }

    std::size_t numBins() const { return n_; }

    LaserScanData convert(const Scan& scan) const {
        LaserScanData out;
        out.angle_min = static_cast<float>(-kPi);
        out.angle_increment = static_cast<float>(increment_rad_);
        out.angle_max = static_cast<float>(-kPi + increment_rad_ * static_cast<double>(n_ - 1));
        out.ranges.assign(n_, std::numeric_limits<float>::infinity());
        if (cfg_.with_intensity) out.intensities.assign(n_, 0.0f);

        for (const Point& p : scan.points) {
            if (!p.isValid()) continue;
            if (cfg_.drop_interference && p.isInterference()) continue;
            const float r = static_cast<float>(p.distance_mm) / 1000.0f;
            if (r < cfg_.range_min_m || r > cfg_.range_max_m) continue;

            const std::size_t idx = binIndex(p.angle_deg);
            if (r < out.ranges[idx]) {
                out.ranges[idx] = r;
                if (cfg_.with_intensity) out.intensities[idx] = static_cast<float>(p.intensity);
            }
        }
        return out;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    std::size_t binIndex(float sensor_angle_deg) const {
        // センサ角度 (時計回り正) → フレーム角度 (反時計回り正), angle_min = -180° からの差 [0, 360)
        double a = -static_cast<double>(sensor_angle_deg) + static_cast<double>(cfg_.angle_offset_deg);
        a = std::fmod(a + 180.0, 360.0);
        if (a < 0.0) a += 360.0;
        long idx = std::lround((a * kPi / 180.0) / increment_rad_);
        if (idx >= static_cast<long>(n_)) idx -= static_cast<long>(n_);  // 360° 直前の丸めで N になる場合
        return static_cast<std::size_t>(idx);
    }

    LaserScanConfig cfg_;
    std::size_t n_ = 1;
    double increment_rad_ = 0.0;
};

}  // namespace ydlidar_tmini

#endif