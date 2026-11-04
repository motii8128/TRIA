#ifndef TRIA_LOCALIZATION_MAPCLOUD_HPP_
#define TRIA_LOCALIZATION_MAPCLOUD_HPP_

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include "common_define.hpp"

namespace tria_localization
{
    struct MapParams
    {
        double voxel_size = 0.05;   // ボクセルの一辺[m]
        double min_range  = 0.1;    // これより近い点は無視(自機・ノイズ)
        double max_range  = 12.0;   // これより遠い点は無視(疎で誤差が大きい)
        int    max_count  = 20;     // ボクセル平均の重みの上限(古い点を固定化させない)
    };

    /// ROS非依存の点群マップ。
    /// スキャンを「推定姿勢でworld座標へ変換」してボクセルごとに平均しながら足していく。
    class PointCloudMap
    {
        public:
        explicit PointCloudMap(const MapParams& params = MapParams()) : params_(params) {}

        /// @param scan センサ座標系のスキャン
        /// @param pose そのスキャンを撮った時のセンサの姿勢(world座標系)
        void add_scan(const Points& scan, const double& x, const double& y, const double& yaw)
        {
            const auto cos_yaw = std::cos(yaw);
            const auto sin_yaw = std::sin(yaw);
            for(const auto& p : scan)
            {
                const double r = std::hypot(p[0], p[1]);
                if(!std::isfinite(r) || r < params_.min_range || r > params_.max_range) continue;

                // ← 求まった変換を点群に適用してworldへ
                const auto wx = cos_yaw * p[0] - sin_yaw * p[1] + x;
                const auto wy = sin_yaw * p[0] + cos_yaw * p[1] + y;

                const int32_t ix = static_cast<int32_t>(std::floor(wx / params_.voxel_size));
                const int32_t iy = static_cast<int32_t>(std::floor(wy / params_.voxel_size));
                auto& c = cells_[key(ix, iy)];

                if(c.n < params_.max_count) c.n++;
                // 逐次平均(重み 1/n)。nが上限に達したら一定の重みで指数移動平均になる
                c.x += (wx - c.x) / c.n;
                c.y += (wy - c.y) / c.n;
            }
        }

        /// マップ全体の点群
        Points points() const
        {
            Points out;
            out.reserve(cells_.size());
            for(const auto& kv : cells_) out.push_back(Point{static_cast<float>(kv.second.x), static_cast<float>(kv.second.y)});
            return out;
        }

        /// center から radius 以内の点だけ取り出す(ICPのターゲットを小さくするため)
        Points local_points(const Point& center, double radius) const
        {
            Points out;
            const double r2 = radius * radius;
            for(const auto& kv : cells_)
            {
                const double dx = kv.second.x - center[0], dy = kv.second.y - center[1];
                if(dx*dx + dy*dy <= r2) out.push_back(Point{static_cast<float>(kv.second.x), static_cast<float>(kv.second.y)});
            }
            return out;
        }

        size_t size() const { return cells_.size(); }
        void clear() { cells_.clear(); }

        private:
        struct Cell { double x = 0.0, y = 0.0; int n = 0; };

        static uint64_t key(int32_t ix, int32_t iy)
        {
            return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) | static_cast<uint32_t>(iy);
        }

        MapParams params_;
        std::unordered_map<uint64_t, Cell> cells_;
    };
}

#endif