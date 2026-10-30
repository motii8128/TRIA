#ifndef TRIA_LOCALIZATION_KD_TREE_HPP_
#define TRIA_LOCALIZATION_KD_TREE_HPP_

#include <array>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "common_define.hpp"

#define NONE SIZE_MAX

namespace tria_localization
{
    struct KdNode
    {
        Point point;
        size_t left;
        size_t right;
        size_t axis; // 0 for x-axis, 1 for y-axis

        KdNode(const Point& point_, const size_t left_, const size_t right_, const size_t axis_)
        :point(point_),left(left_),right(right_),axis(axis_){}

        KdNode() = default;
    };

    class KdTree
    {
        public:
        KdTree() = default;

        /// ２次元点群を渡して初期化する
        KdTree(Points& points)
        {
            nodes_ = std::vector<KdNode>();
            nodes_.clear();
            nodes_.reserve(points.size());

            root_ = build(points, nodes_, 0);
        }

        void nearest_point(const Point& target_point, Point* best_point, double* best_distance)
        {
            *best_distance = std::numeric_limits<double>::max();
            
            nearest_point_rec(root_, target_point, best_point, best_distance);
        }

        std::vector<Point> radius_search(const Point& target, double radius)
        {
            std::vector<Point> result;

            if(root_ == NONE || radius < 0.0f)return result;

            const auto radius_squared = radius * radius;

            radius_search_rec(root_, target, radius_squared, result);

            return result;
        }


        private:
        std::vector<KdNode> nodes_;
        size_t root_;

        
        size_t build(Points& points, std::vector<KdNode>& nodes, size_t depth)
        {
            if(points.empty()) return NONE;

            const auto points_size = points.size();

            const auto axis = depth % 2;
            const auto mid_index = points_size / 2;

            std::nth_element(
                points.begin(), points.begin()+mid_index, points.end(), 
                [axis](const Point& a, const Point& b){
                    if(axis == 0)
                    {
                        return a[0] < b[0];
                    }
                    else
                    {
                        return a[1] < b[1];
                    }
                }
            );

            // 点群を中央から２つに分ける
            auto left_points = std::vector(points.begin(), points.begin() + mid_index);
            auto right_points = std::vector(points.begin() + mid_index + 1, points.end());

            // 中央の点を取り出す
            const auto median = points[mid_index];

            // 新しいノードを作成してnodesに追加する
            const auto node_index = nodes.size();
            nodes.emplace_back(median, NONE, NONE, axis);

            const auto left_child = build(left_points, nodes, depth + 1);
            const auto right_child = build(right_points, nodes, depth + 1);

            nodes[node_index].left = left_child;
            nodes[node_index].right = right_child;

            return node_index;
        }

        void nearest_point_rec(size_t node_index, const Point& target, Point* best_point, double* best_distance)
        {
            if(node_index == NONE) return;

            const auto& node = nodes_[node_index];

            // ターゲットと現在のノードの距離を計算
            const auto dx = node.point[0] - target[0];
            const auto dy = node.point[1] - target[1];
            const auto distance = dx * dx + dy * dy;

            if(distance < *best_distance)
            {
                *best_point = node.point;
                *best_distance = distance;
            }

            const auto axis = node.axis;

            // 分割軸方向の距離
            const double diff = node.point[node.axis] - target[node.axis];

            // 近い側を先に探索
            const auto next = diff > 0.0 ? node.left : node.right;

            // 反対側
            const auto other = diff > 0.0 ? node.right : node.left;

            nearest_point_rec(next, target, best_point, best_distance);

            if(diff * diff < *best_distance)
            {
                nearest_point_rec(other, target, best_point, best_distance);
            }
        }

        void radius_search_rec(std::size_t node_index, const Point& target, double radius_squared, std::vector<Point>& result)
        {
            if(node_index == NONE) return;

            const auto& node = nodes_[node_index];

            // ターゲットと現在のノードの距離を計算
            const auto dx = node.point[0] - target[0];
            const auto dy = node.point[1] - target[1];
            const auto distance = dx * dx + dy * dy;

            if(distance <= radius_squared)
            {
                result.push_back(node.point);
            }

            const auto axis = node.axis;

            // 分割軸方向の距離
            const double diff = node.point[node.axis] - target[node.axis];

            // 近い側を先に探索
            const auto next = diff > 0.0 ? node.left : node.right;

            // 反対側
            const auto other = diff > 0.0 ? node.right : node.left;

            radius_search_rec(next, target, radius_squared, result);

            if(diff * diff <= radius_squared)
            {
                radius_search_rec(other, target, radius_squared, result);
            }
        }
    };
}

#endif