/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "aid_costmap_plugin/keepout_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace aid_costmap_plugin
{

KeepoutLayer::KeepoutLayer()
: filter_info_sub_(nullptr), mask_sub_(nullptr), mask_costmap_(nullptr),
  mask_frame_(""), global_frame_(""),
  last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
  access_ = new nav2_costmap_2d::Costmap2D::mutex_t();
}

void KeepoutLayer::getParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("filter_info_topic", rclcpp::ParameterValue("/costmap_filter_info"));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "filter_info_topic", filter_info_topic_);
}


// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
KeepoutLayer::onInitialize()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  logger_ = node->get_logger();

  getParameters(node);
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    logger_,
    "KeepoutLayer: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutLayer::filterInfoCallback, this, std::placeholders::_1));

  global_frame_ = layered_costmap_->getGlobalFrameID();
}
void KeepoutLayer::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutLayer: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutLayer: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  // Checking that base and multiplier are set to their default values
  if (msg->base != 0.0 || msg->multiplier != 1.0) {
    RCLCPP_ERROR(
      logger_,
      "KeepoutLayer: For proper use of keepout filter base and multiplier"
      " in CostmapFilterInfo message should be set to their default values (%f and %f)",
      0.0, 1.0);
  }

  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  RCLCPP_INFO(
    logger_,
    "KeepoutLayer: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutLayer::maskCallback, this, std::placeholders::_1));
}
void KeepoutLayer::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_costmap_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutLayer: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutLayer: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    mask_costmap_.reset();
  }

  // Making a new mask_costmap_
  mask_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(*msg);
  mask_frame_ = msg->header.frame_id;
}
// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
KeepoutLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
KeepoutLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "KeepoutLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap keepout is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
KeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  if (!mask_costmap_) {
    // Show warning message every 2 seconds to not litter an output
    static auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(
      logger_, clock, 10000,
      "KeepoutFilter: Filter mask was not received");
    return;
  }

  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();  // initialize by identical transform
  int mg_min_x, mg_min_y;  // masger_grid indexes of bottom-left window corner
  int mg_max_x, mg_max_y;  // masger_grid indexes of top-right window corner

  if (mask_frame_ != global_frame_) {
    // Filter mask and current layer are in different frames:
    // prepare frame transformation if mask_frame_ != global_frame_
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        mask_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "KeepoutFilter: Failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame_.c_str(), mask_frame_.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf2_transform);

    mg_min_x = min_i;
    mg_min_y = min_j;
    mg_max_x = max_i;
    mg_max_y = max_j;
  } else {
    // Filter mask and current layer are in the same frame:
    // apply the following optimization - iterate only in overlapped
    // (min_i, min_j)..(max_i, max_j) & mask_costmap_ area.
    //
    //           mask_costmap_
    //       *----------------------------*
    //       |                            |
    //       |                            |
    //       |      (2)                   |
    // *-----+-------*                    |
    // |     |///////|<- overlapped area  |
    // |     |///////|   to iterate in    |
    // |     *-------+--------------------*
    // |    (1)      |
    // |             |
    // *-------------*
    //  master_grid (min_i, min_j)..(max_i, max_j) window
    //
    // ToDo: after costmap rotation will be added, this should be re-worked.

    double wx, wy;  // world coordinates

    // Calculating bounds corresponding to bottom-left overlapping (1) corner
    // mask_costmap_ -> master_grid intexes conversion
    const double half_cell_size = 0.5 * mask_costmap_->getResolution();
    wx = mask_costmap_->getOriginX() + half_cell_size;
    wy = mask_costmap_->getOriginY() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
    // Calculation of (1) corner bounds
    if (mg_min_x >= max_i || mg_min_y >= max_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);

    // Calculating bounds corresponding to top-right window (2) corner
    // mask_costmap_ -> master_grid intexes conversion
    wx = mask_costmap_->getOriginX() +
      mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution() + half_cell_size;
    wy = mask_costmap_->getOriginY() +
      mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
    // Calculation of (2) corner bounds
    if (mg_max_x <= min_i || mg_max_y <= min_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
  }

  // unsigned<-signed conversions.
  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;  // master_grid iterators
  unsigned int index;  // corresponding index of master_grid
  double gl_wx, gl_wy;  // world coordinates in a global_frame_
  double msk_wx, msk_wy;  // world coordinates in a mask_frame_
  unsigned int mx, my;  // mask_costmap_ coordinates
  unsigned char data, old_data;  // master_grid element data

  // Main master_grid updating loop
  // Iterate in costmap window by master_grid indexes
  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];
      // Calculating corresponding to (i, j) point at mask_costmap_:
      // Get world coordinates in global_frame_
      master_grid.mapToWorld(i, j, gl_wx, gl_wy);
      if (mask_frame_ != global_frame_) {
        // Transform (i, j) point from global_frame_ to mask_frame_
        tf2::Vector3 point(gl_wx, gl_wy, 0);
        point = tf2_transform * point;
        msk_wx = point.x();
        msk_wy = point.y();
      } else {
        // In this case master_grid and filter-mask are in the same frame
        msk_wx = gl_wx;
        msk_wy = gl_wy;
      }
      // Get mask coordinates corresponding to (i, j) point at mask_costmap_
      if (mask_costmap_->worldToMap(msk_wx, msk_wy, mx, my)) {
        data = mask_costmap_->getCost(mx, my);
        // Update if mask_ data is valid and greater than existing master_grid's one
        if (data == NO_INFORMATION) {
          continue;
        }
        if (data > old_data || old_data == NO_INFORMATION) {
          master_array[index] = data;
        }
      }
    }
  }
  current_ = true;
}

}  // namespace aid_costmap_plugin

// This is the macro allowing a aid_costmap_plugin::KeepoutLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(aid_costmap_plugin::KeepoutLayer, nav2_costmap_2d::Layer)
