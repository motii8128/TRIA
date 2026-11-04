#ifndef TRIA_LOCALIZATION_OCCUPANCY_GRID_BUILDER_HPP_
#define TRIA_LOCALIZATION_OCCUPANCY_GRID_BUILDER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common_define.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace tria_localization
{
    /// nav_msgs/OccupancyGrid と同じ並びの密なグリッド(ROS非依存)
    /// data[y * width + x]、(0,0)セルの左下隅がworld座標の (origin_x, origin_y)
    /// 値: -1=未知, 0=空き, 100=占有
    struct GridData
    {
        double resolution = 0.05;
        double origin_x = 0.0, origin_y = 0.0;
        int width = 0, height = 0;
        std::vector<int8_t> data;
    };

    struct GridParams
    {
        double resolution = 0.05;   // [m/cell]
        double min_range  = 0.1;
        double max_range  = 12.0;   // これより遠い点は使わない
        // ログオッズ(占有確率 p を log(p/(1-p)) にしたもの)
        float l_occ  =  0.85f;      // 命中1回で足す量 (p=0.7相当)
        float l_free = -0.40f;      // 通過1回で足す量 (p=0.4相当)
        float l_min  = -2.0f;       // 下限(動く物体が消えやすいように)
        float l_max  =  3.5f;       // 上限
        // 出力の閾値
        float occupied_threshold = 0.65f;   // p がこれ以上 → 100
        float free_threshold     = 0.35f;   // p がこれ以下 → 0 (その間は-1=未知)
    };

    /// スキャンをレイキャストしてログオッズを更新する占有格子マップ。
    /// 点群マップ(PointCloudMap)と違い、「空き」も扱えるのでOccupancyGridにできる。
    class OccupancyGridBuilder
    {
        public:
        explicit OccupancyGridBuilder(const GridParams& params = GridParams()) : params_(params) {}

        /// @param scan センサ座標系のスキャン
        /// @param pose そのスキャンを撮った時のセンサ姿勢(world座標系)
        void add_scan(const Points& scan, const double& x, const double& y, const double& yaw)
        {
            const auto cos_yaw = std::cos(yaw);
            const auto sin_yaw = std::sin(yaw);
            const int64_t sx = cell_of(x), sy = cell_of(y);

            // 1スキャン内で同じセルを何度も更新しない(壁の近くのレイが壁を消さないため)
            std::unordered_set<uint64_t> occupied, freed;

            for(const auto& p : scan)
            {
                const double r = std::hypot(p[0], p[1]);
                if(!std::isfinite(r) || r < params_.min_range || r > params_.max_range) continue;

                // 求まった変換を点群に適用してworldへ
                const auto wx = cos_yaw * p[0] - sin_yaw * p[1] + x;
                const auto wy = sin_yaw * p[0] + cos_yaw * p[1] + y;
                const int64_t hx = cell_of(wx), hy = cell_of(wy);

                // センサ位置から命中セルの手前まで = 空き
                bresenham(sx, sy, hx, hy, [&](int64_t x, int64_t y){ freed.insert(key(x, y)); });
                occupied.insert(key(hx, hy));
            }

            for(const auto k : freed)
            {
                if(occupied.count(k)) continue;   // 占有が優先
                update(k, params_.l_free);
            }
            for(const auto k : occupied) update(k, params_.l_occ);
        }

        /// 密なグリッドに変換する(観測があった範囲を囲む最小の矩形)
        GridData to_grid() const
        {
            GridData g;
            g.resolution = params_.resolution;
            if(cells_.empty()) return g;

            int64_t min_x = std::numeric_limits<int64_t>::max(), min_y = min_x;
            int64_t max_x = std::numeric_limits<int64_t>::min(), max_y = max_x;
            for(const auto& kv : cells_)
            {
                int64_t x, y; unkey(kv.first, x, y);
                min_x = std::min(min_x, x); max_x = std::max(max_x, x);
                min_y = std::min(min_y, y); max_y = std::max(max_y, y);
            }

            g.width  = static_cast<int>(max_x - min_x + 1);
            g.height = static_cast<int>(max_y - min_y + 1);
            g.origin_x = min_x * params_.resolution;
            g.origin_y = min_y * params_.resolution;
            g.data.assign(static_cast<size_t>(g.width) * g.height, -1);

            for(const auto& kv : cells_)
            {
                int64_t x, y; unkey(kv.first, x, y);
                const float p = 1.0f - 1.0f / (1.0f + std::exp(kv.second));
                int8_t v = -1;
                if(p >= params_.occupied_threshold) v = 100;
                else if(p <= params_.free_threshold) v = 0;
                g.data[static_cast<size_t>(y - min_y) * g.width + (x - min_x)] = v;
            }
            return g;
        }

        /// ポーズグラフ最適化後などに、全キーフレームから作り直すときに使う
        void clear() { cells_.clear(); }
        size_t size() const { return cells_.size(); }

        private:
        GridParams params_;
        std::unordered_map<uint64_t, float> cells_;   // キー -> ログオッズ

        int64_t cell_of(double v) const { return static_cast<int64_t>(std::floor(v / params_.resolution)); }

        static uint64_t key(int64_t x, int64_t y)
        {
            return (static_cast<uint64_t>(static_cast<uint32_t>(static_cast<int32_t>(x))) << 32)
                 | static_cast<uint32_t>(static_cast<int32_t>(y));
        }
        static void unkey(uint64_t k, int64_t& x, int64_t& y)
        {
            x = static_cast<int32_t>(static_cast<uint32_t>(k >> 32));
            y = static_cast<int32_t>(static_cast<uint32_t>(k & 0xffffffffu));
        }

        void update(uint64_t k, float delta)
        {
            float& l = cells_[k];   // 新規セルは0(p=0.5)から始まる
            l = std::min(params_.l_max, std::max(params_.l_min, l + delta));
        }

        // 終点(x1,y1)は含めない
        template<class F>
        static void bresenham(int64_t x0, int64_t y0, int64_t x1, int64_t y1, F&& visit)
        {
            const int64_t dx = std::llabs(x1 - x0), dy = -std::llabs(y1 - y0);
            const int64_t sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
            int64_t err = dx + dy;
            while(!(x0 == x1 && y0 == y1))
            {
                visit(x0, y0);
                const int64_t e2 = 2 * err;
                if(e2 >= dy){ err += dy; x0 += sx; }
                if(e2 <= dx){ err += dx; y0 += sy; }
            }
        }
    };

    /// GridData を nav_msgs::msg::OccupancyGrid に詰める(ROSのヘッダをここでは読み込まないためテンプレート)
    /// 使い方: nav_msgs::msg::OccupancyGrid msg; to_occupancy_grid_msg(grid, msg);
    ///         その後 msg.header.stamp / msg.header.frame_id = "map" を設定して publish

    void to_occupancy_grid_msg(const GridData& g, nav_msgs::msg::OccupancyGrid& msg)
    {
        msg.info.resolution = static_cast<float>(g.resolution);
        msg.info.width  = static_cast<uint32_t>(g.width);
        msg.info.height = static_cast<uint32_t>(g.height);
        msg.info.origin.position.x = g.origin_x;
        msg.info.origin.position.y = g.origin_y;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;
        msg.data.assign(g.data.begin(), g.data.end());
    }
}

#endif