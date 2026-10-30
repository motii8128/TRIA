// 複数の 2D LiDAR (LaserScan) を、取り付け位置・向き・傾きの設定で共通フレームの 3D 点群へ
// 変換して統合するための計算部。ROS 非依存・ヘッダのみ。
//
// 座標規約 (URDF / REP-103 の rpy と同じ):
//   p_target = R * p_sensor + t,   R = Rz(yaw) * Ry(pitch) * Rx(roll)
//   - センサ座標: x = スキャン 0° 方向, y = 反時計回り 90° 方向, z = スキャン面の法線 (上向き)
//   - yaw   : z 軸まわり。正で反時計回り (上から見て)
//   - pitch : y 軸まわり。正でセンサの x 軸 (0° 方向) が下を向く
//   - roll  : x 軸まわり。正でセンサの y 軸 (左) が上を向く
// 傾けて取り付けたセンサの点は水平面に載らないので、出力は 3D (x, y, z) になる。
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace ydlidar_tmini {

/// 共通フレーム (例: base_link) から見たセンサの取り付け姿勢。角度は rad。
struct MountPose {
    double x = 0.0, y = 0.0, z = 0.0;  // [m]
    double roll_rad = 0.0, pitch_rad = 0.0, yaw_rad = 0.0;
};

class RigidTransform {
public:
    RigidTransform() = default;  // 恒等変換

    explicit RigidTransform(const MountPose& p) {
        const double cr = std::cos(p.roll_rad), sr = std::sin(p.roll_rad);
        const double cp = std::cos(p.pitch_rad), sp = std::sin(p.pitch_rad);
        const double cy = std::cos(p.yaw_rad), sy = std::sin(p.yaw_rad);
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        r_[0][0] = cy * cp;  r_[0][1] = cy * sp * sr - sy * cr;  r_[0][2] = cy * sp * cr + sy * sr;
        r_[1][0] = sy * cp;  r_[1][1] = sy * sp * sr + cy * cr;  r_[1][2] = sy * sp * cr - cy * sr;
        r_[2][0] = -sp;      r_[2][1] = cp * sr;                 r_[2][2] = cp * cr;
        t_ = {p.x, p.y, p.z};
    }

    std::array<double, 3> apply(double x, double y, double z) const {
        return {r_[0][0] * x + r_[0][1] * y + r_[0][2] * z + t_[0],
                r_[1][0] * x + r_[1][1] * y + r_[1][2] * z + t_[1],
                r_[2][0] * x + r_[2][1] * y + r_[2][2] * z + t_[2]};
    }

private:
    double r_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::array<double, 3> t_ = {0, 0, 0};
};

struct CloudPoint {
    float x, y, z, intensity;
};
static_assert(sizeof(CloudPoint) == 16, "CloudPoint must be 4 packed floats");

/// 1 本の LaserScan を変換し、有効な点を `out` に追加する。
///
/// 無効な測距 (inf / NaN) と、[range_min, range_max] の外は捨てる。
/// `intensities` が空 or ranges より短い場合、足りない分の intensity は 0。
inline void appendScanPoints(const RigidTransform& tf, float angle_min, float angle_increment,
                             float range_min, float range_max, const std::vector<float>& ranges,
                             const std::vector<float>& intensities, std::vector<CloudPoint>& out) {
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        const float r = ranges[i];
        if (!std::isfinite(r) || r < range_min || r > range_max) continue;
        const double th = static_cast<double>(angle_min) + static_cast<double>(i) * angle_increment;
        const auto p = tf.apply(r * std::cos(th), r * std::sin(th), 0.0);
        out.push_back(CloudPoint{static_cast<float>(p[0]), static_cast<float>(p[1]),
                                 static_cast<float>(p[2]),
                                 i < intensities.size() ? intensities[i] : 0.0f});
    }
}

/// PointCloud2 (x, y, z, intensity: float32 x4, point_step = 16) のバイト列にする。
inline std::vector<std::uint8_t> serializeXYZI(const std::vector<CloudPoint>& pts) {
    std::vector<std::uint8_t> data(pts.size() * sizeof(CloudPoint));
    if (!pts.empty()) std::memcpy(data.data(), pts.data(), data.size());
    return data;
}

}  // namespace ydlidar_tmini