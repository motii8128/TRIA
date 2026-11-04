#ifndef TRIA_LOCALIZATION_P2P_ICP_HPP_
#define TRIA_LOCALIZATION_P2P_ICP_HPP_

#include "common_define.hpp"
#include "kd_tree.hpp"

#include <cmath>
#include <vector>

namespace tria_localization
{
    /// @brief 点と点を対応させるICP(2D)。各反復で対応点を探し、重み付きKabsch法で剛体変換を閉形式に解く
    class Point_to_Point
    {
        public:
        Point_to_Point(int max_iteration, double tolerance, double neighbor_threshold)
        :max_iter_(max_iteration),tolerance_(tolerance),neighbor_threshold_(neighbor_threshold)
        {
            translation_ = {0.0, 0.0};
            yaw_ = 0.0;
        }

        void set_pose(double x, double y, double yaw)
        {
            translation_ = {static_cast<float>(x), static_cast<float>(y)};
            yaw_ = yaw;
        }

        bool compute(const Points& source_points, Points& target_points)
        {
            inlier_ratio_ = 0.0;
            if(source_points.size() < 3 || target_points.size() < 3) return false;
            KdTree kdtree(target_points);

            for(int iter = 0; iter < max_iter_; iter++)
            {
                const double x0 = translation_[0], y0 = translation_[1], t0 = yaw_;
                bool result = step(source_points, kdtree);

                if(!result)
                {
                    return false;
                };

                const double d = std::abs(translation_[0]-x0) + std::abs(translation_[1]-y0) + std::abs(yaw_-t0);
                if(d < tolerance_)
                {
                    return true;
                };
            }

            return false;
        }

        double inlier_ratio() const { return inlier_ratio_; }

        Point get_translation()
        {
            return translation_;
        }

        double get_yaw()
        {
            return yaw_;
        }

        private:
        int max_iter_;
        Point translation_;
        double yaw_;
        double tolerance_, neighbor_threshold_;
        double inlier_ratio_ = 0.0;

        // ICPの1ステップ。現在の姿勢で対応点を探し、「元のsrc点→対応するtgt点」を直接解く
        bool step(const Points& source_points, KdTree& kdtree)
        {
            const double cos_yaw = std::cos(yaw_);
            const double sin_yaw = std::sin(yaw_);
            const double tx = translation_[0];
            const double ty = translation_[1];
            const double thr2 = neighbor_threshold_ * neighbor_threshold_;

            // 重み付き重心用の合計
            double sw = 0.0;
            double ssx = 0.0, ssy = 0.0;   // 元のsrc点(センサ座標)の重み付き和
            double stx = 0.0, sty = 0.0;   // tgt点の重み付き和
            Points src_corr, tgt_corr;     // 元のsrc点 / 対応するtgt点
            std::vector<double> w_corr;
            src_corr.reserve(source_points.size());
            tgt_corr.reserve(source_points.size());
            w_corr.reserve(source_points.size());

            for(const auto& src_point : source_points)
            {
                const Point translated_src = {
                    static_cast<float>(cos_yaw * src_point[0] - sin_yaw * src_point[1] + tx),
                    static_cast<float>(sin_yaw * src_point[0] + cos_yaw * src_point[1] + ty)
                };

                Point nearest{0.0f, 0.0f};
                double distance_squared = 0.0;
                kdtree.nearest_point(translated_src, &nearest, &distance_squared);
                if(distance_squared > thr2) continue;

                // 実際の対応距離で重み付け(0〜1)。遠いほど信頼しない
                const double w = 1.0 - distance_squared / thr2;

                src_corr.push_back(src_point);
                tgt_corr.push_back(nearest);
                w_corr.push_back(w);
                sw += w;
                ssx += w * src_point[0]; ssy += w * src_point[1];
                stx += w * nearest[0];   sty += w * nearest[1];
            }

            const size_t pair_num = src_corr.size();
            inlier_ratio_ = static_cast<double>(pair_num) / static_cast<double>(source_points.size());
            if(pair_num < 3 || !(sw > 1e-9)) return false;

            const double scx = ssx / sw, scy = ssy / sw;
            const double tcx = stx / sw, tcy = sty / sw;

            // 2DのKabsch: 回転角 = atan2(Σ w s×t, Σ w s·t)
            double num = 0.0, den = 0.0;
            for(size_t i = 0; i < pair_num; i++)
            {
                const double sx = src_corr[i][0] - scx, sy = src_corr[i][1] - scy;
                const double qx = tgt_corr[i][0] - tcx, qy = tgt_corr[i][1] - tcy;
                num += w_corr[i] * (sx * qy - sy * qx);
                den += w_corr[i] * (sx * qx + sy * qy);
            }

            const double yaw = std::atan2(num, den);
            const double c = std::cos(yaw), s = std::sin(yaw);

            // 並進: t = tgt重心 - R * src重心  (srcは元の座標なので現在のtは混ぜない)
            translation_[0] = static_cast<float>(tcx - (c * scx - s * scy));
            translation_[1] = static_cast<float>(tcy - (s * scx + c * scy));
            yaw_ = yaw;

            return true;
        }
    };
}

#endif