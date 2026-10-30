#ifndef TRIA_LOCALIZATION_COMMON_DEFINE_HPP_
#define TRIA_LOCALIZATION_COMMON_DEFINE_HPP_

#include <array>
#include <vector>

namespace tria_localization
{
    // x=[0] y=[1]
    using Point = std::array<float, 2>;

    using Points = std::vector<Point>;

}

#endif