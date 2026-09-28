#ifndef NAV2_COSTMAP_2D__CUBE_FOOTPRINT_COLLISION_CHECKER_HPP_
#define NAV2_COSTMAP_2D__CUBE_FOOTPRINT_COLLISION_CHECKER_HPP_

#include <memory>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/line_iterator.hpp"

namespace nav2_costmap_2d {

/**
 * @class CubeFootprintCollisionChecker
 * @brief Checker for collision with a footprint on a costmap
 */
template <typename CostmapT>
class CubeFootprintCollisionChecker
    : public FootprintCollisionChecker<CostmapT> {
 public:
  CubeFootprintCollisionChecker() {}

  /**
   * @brief A constructor.
   */
  explicit CubeFootprintCollisionChecker(CostmapT costmap) {}

  /**
   * @brief Find the footprint cost in oriented footprint
   */
  double footprintCost(
      const std::vector<geometry_msgs::msg::Point> &footprint) {
    // 现在我们必须在 costmap_ 网格中放置轮廓
    unsigned int x0, x1, y0, y1;
    double footprint_cost = 0.0;

    // 获取第一个点的单元格坐标
    if (!FootprintCollisionChecker<CostmapT>::worldToMap(
            footprint[0].x, footprint[0].y, x0, y0)) {
      return -1;
    }

    // 缓存开始以消除一个 worldToMap 调用
    unsigned int xstart = x0;
    unsigned int ystart = y0;

    // 我们需要对轮廓中的每一条线进行光栅化
    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
      // 获取第二个点的单元格坐标
      if (!FootprintCollisionChecker<CostmapT>::worldToMap(
              footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
        return -1;
      }

      double line_cost =
          CubeFootprintCollisionChecker<CostmapT>::lineCost(x0, x1, y0, y1);
      // std::cout << "line_cost" << line_cost << std::endl;
      if (line_cost == -1) {
        return -1;
      }

      footprint_cost += line_cost;

      // 第二个点是下一次迭代的第一个点
      x0 = x1;
      y0 = y1;
    }
    // 我们还需要连接轮廓中的第一个点和最后一个点
    // 最后一次迭代的 x1, y1 是最后一个轮廓点的坐标
    double line_cost =
        CubeFootprintCollisionChecker<CostmapT>::lineCost(x0, x1, y0, y1);
    if (line_cost == -1) {
      return -1;
    }

    footprint_cost += line_cost;

    return footprint_cost;
  }

  /**
   * @brief Get the cost for a line segment
   */
  double lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = 0;

    for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid();
         line.advance()) {
      point_cost = FootprintCollisionChecker<CostmapT>::pointCost(
          line.getX(), line.getY());  // 评分当前点

      // 如果发生碰撞，就不需要继续了
      if (point_cost == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
        return -1;
      }
      line_cost += point_cost;
    }

    return line_cost;
  }
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__CUBE_FOOTPRINT_COLLISION_CHECKER_HPP_
