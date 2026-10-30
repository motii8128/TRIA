#ifndef TRIA_LOCALIZATION_P2L_ICP_HPP_
#define TRIA_LOCALIZATION_P2L_ICP_HPP_

#include "common_define.hpp"
#include "kd_tree.hpp"

#include <cmath>

namespace tria_localization
{
    /// @brief 点と線を対応させるICP。ガウス・ニュートン法を用いて最小二乗法を解く
    class IterativeClosestPoint
    {
        public:
        IterativeClosestPoint(int max_iteration, double search_radius, double tolerance, double neighbor_threshold)
        :max_iter_(max_iteration),radius_for_search_(search_radius),tolerance_(tolerance),neighbor_threshold_(neighbor_threshold)
        {
            translation_ = {0.0, 0.0};
            yaw_ = 0.0;
        }

        void set_pose(double x, double y, double yaw)
        {
            translation_ = {x, y};
            yaw_ = yaw;
        }

        void compute(Points& source_points, Points& target_points)
        {
            KdTree kdtree(target_points);

            for(int iter = 0; iter < max_iter_; iter++)
            {
                const double x0 = translation_[0], y0 = translation_[1], t0 = yaw_;
                bool result = step(source_points, kdtree);

                if(!result) break;

                const double d = std::abs(translation_[0]-x0) + std::abs(translation_[1]-y0) + std::abs(yaw_-t0);
                if(d < tolerance_) break;
            }
        }

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
        double radius_for_search_, tolerance_, neighbor_threshold_;
        // ICPの１ステップ
        bool step(Points& source_points, KdTree& kdtree)
        {
            const auto cos_yaw = cos(yaw_);
            const auto sin_yaw = sin(yaw_);
            const auto tx = translation_[0];
            const auto ty = translation_[1];

            // ヘッセ行列
            // 実対称行列
            auto H00 = 0.0f, H01 = 0.0f, H02 = 0.0f;
            auto             H11 = 0.0f, H12 = 0.0f;
            auto                         H22 = 0.0f;

            // 右辺のベクトル
            auto g0 = 0.0f, g1 = 0.0f, g2 = 0.0f;

            auto valid_num = 0;

            for(const auto& src_point : source_points)
            {
                // 現在の変換でsrc点を移動
                const Point translated_src = {
                    cos_yaw * src_point[0] - sin_yaw * src_point[1] + tx ,
                    sin_yaw * src_point[0] + cos_yaw * src_point[1] + ty
                };

                Point nearest;
                double distance_squared;
                kdtree.nearest_point(translated_src, &nearest, &distance_squared);

                if(distance_squared > neighbor_threshold_ * neighbor_threshold_) continue;

                const auto neighbors = kdtree.radius_search(nearest, radius_for_search_);    
                
                // 近傍点が少なすぎる場合は線形近似できないのでスキップ
                const auto neighbor_num = neighbors.size();
                if(neighbor_num < 3) continue;

                // 共分散行列を求めてその最小固有値の固有ベクトルを計算する
                // 正規化された固有ベクトルは法線ベクトルにあたる
                Point normal_vec = calc_minimum_eigenvector(neighbors);

                const auto dx = translated_src[0] - nearest[0];
                const auto dy = translated_src[1] - nearest[1];

                // 法線ベクトルと最近傍ペアベクトルの内積をとる
                // errはターゲットの線と変換された点との距離であり、これを０にしたいと考えている
                const auto err = normal_vec[0] * dx + normal_vec[1] * dy;

                //　パラメータを変化させた際の誤差の変化率をヤコビアンJとしてもとめる
                // J = [de/dθ, de/dx, de/dy]
                // de/dx = n_xとde/dy = n_yはerrを求めた式を偏微分したら求まる
                const auto J_x = normal_vec[0];
                const auto J_y = normal_vec[1];

                // 回転行列のθ微分を用いてsrc点を変換する
                const auto j_trans_x = -sin_yaw * src_point[0] - cos_yaw * src_point[1];
                const auto j_trans_y =  cos_yaw * src_point[0] - sin_yaw * src_point[1];

                // これと法線の内積をとることで、回転を微小に動かしたとき、誤差eがどれだけ変化するか（de/dθ）が求まる
                const auto J_theta = normal_vec[0] * j_trans_x + normal_vec[1] * j_trans_y;

                // 我々が最小化したい目的関数Eは
                // E(⊿x) = Σ(J*⊿x + e)^2であるため、これから
                // (Σ J_transpose * J)⊿x = Σ J_transpose(- e)という連立方程式を得る
                //　これをH*⊿x = gと書くことにする
                // Hは３次正方行列であり、⊿x(求めたい値)とgは３次元列ベクトルである
                H00 += J_theta * J_theta;
                H01 += J_theta * J_x;
                H02 += J_theta * J_y;
                H11 += J_x     * J_x;
                H12 += J_x     * J_y;
                H22 += J_y     * J_y;

                g0 += -err * J_theta;
                g1 += -err * J_x;
                g2 += -err * J_y;

                valid_num++;
            }

            if(valid_num < 3) return false;

            // ヘッセ行列Hの逆行列を求める
            // Hの行列式(余因子展開して計算)
            const auto H_det = H00 * (H11 * H22 - H12 * H12) + H01 * (- H01 * H22 + H12 * H02) + H02 * (H01 * H12 - H11 * H02);
            if (std::abs(H_det) < 1e-9) return false;

            const auto inv_H_det = 1.0f / H_det;

            const auto inv_H00 = (H11 * H22 - H12 * H12) * inv_H_det;
            const auto inv_H01 = (- H01 * H22 + H12 * H02) * inv_H_det;
            const auto inv_H02 = (H01 * H12 - H11 * H02) * inv_H_det;

            const auto inv_H11 = (H00 * H22 - H02 * H02) * inv_H_det;
            const auto inv_H12 = (- H00 * H12 + H01 * H02) * inv_H_det;
            const auto inv_H22 = (H00 * H11 - H01 * H01) * inv_H_det;

            // ⊿θを計算
            yaw_ +=     inv_H00 * g0 + inv_H01 * g1 + inv_H02 * g2;
            // ⊿x
            translation_[0] += inv_H01 * g0 + inv_H11 * g1 + inv_H12 * g2;
            // ⊿y
            translation_[1] += inv_H02 * g0 + inv_H12 * g1 + inv_H22 * g2;

            return true;
        }

        // 共分散行列を求めて、その行列の最小固有値の固有ベクトルを求める
        Point calc_minimum_eigenvector(const Points& points)
        {
            const auto size = points.size();

            //　点群の重心を計算
            Point centroid{0.0, 0.0};
            for(const auto& point : points)
            {
                centroid[0] += point[0];
                centroid[1] += point[1];
            }
            centroid[0] /= (double)size;
            centroid[1] /= (double)size;

            // 点群の共分散行列を求める
            // |m00 m10|
            // |m10 m11|
            // この行列は必ず実対称行列になるので０行１列目の成分は計算する必要がない
            double m00=0.0, m10=0.0, m11 = 0.0;
            Point diff;
            for(const auto& point : points)
            {
                diff[0] = point[0] - centroid[0];
                diff[1] = point[1] - centroid[1];

                // diff * diff_Transpose
                m00 += diff[0] * diff[0];
                m10 += diff[1] * diff[0]; m11 += diff[1] * diff[1];
                //
            }

            // 特性方程式について解くことで最小固有値を求める
            // 実対称行列の特性方程式
            // λ^2 - (m00 + m11)λ + (m00 * m11 - m10 * m10) = 0

            // 対角成分の和
            const auto trace = m00 + m11;
            // 行列式
            const auto det = m00 * m11 - m10 * m10;

            // ２次方程式の判別式
            auto discriminant = trace * trace - 4.0f * det;

            if(discriminant < 0.0f) discriminant = 0.0f;

            double sqrt_disc = std::sqrt(discriminant);

            // ２つの固有値のうち、小さい方を求める
            // ２次方程式の解の公式
            double lambda_min = 0.5f * (trace - sqrt_disc);

            // 対応する固有ベクトルを求める
            auto nx = -m10;
            auto ny = m00 - lambda_min;

            // 例外処理
            if(std::abs(nx) < 1e-6 && std::abs(ny) < 1e-6)
            {
                nx = 1.0f;
                ny = 0.0f;
            }

            // 正規化する
            double norm = std::sqrt(nx * nx + ny * ny);
            if (norm > 1e-6f) 
            {
                nx /= norm;
                ny /= norm;
            }

            return Point{nx, ny};
        }
    };
}

#endif