#include "aid_costmap_plugin/safty_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "visualization_msgs/msg/marker.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

PLUGINLIB_EXPORT_CLASS(aid_costmap_plugin::SaftyLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using nav2_costmap_2d::Observation;
using nav2_costmap_2d::ObservationBuffer;
using rcl_interfaces::msg::ParameterType;

namespace aid_costmap_plugin {

SaftyLayer::~SaftyLayer() {
  dyn_params_handler_.reset();

  for (auto& notifier : observation_notifiers_) {
    notifier.reset();
  }
}

void SaftyLayer::onInitialize() {
  double transform_tolerance;

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;
  RCLCPP_INFO(logger_, "Initializing SaftyLayer plugin");

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("min_ground_height", rclcpp::ParameterValue(-0.05));
  declareParameter("max_ground_height", rclcpp::ParameterValue(0.05));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources",
                   rclcpp::ParameterValue(std::string("")));
  declareParameter("forward_offset", rclcpp::ParameterValue(1.0));
  declareParameter("region_size", rclcpp::ParameterValue(1.0));
  declareParameter("robot_base_frame",
                   rclcpp::ParameterValue(std::string("base_link")));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled",
                      footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "min_ground_height", min_ground_height_);
  node->get_parameter(name_ + "." + "max_ground_height", max_ground_height_);
  node->get_parameter("transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + "." + "observation_sources", topics_string);
  node->get_parameter(name_ + "." + "forward_offset", forward_offset_);
  node->get_parameter(name_ + "." + "region_size", region_size_);
  node->get_parameter(name_ + "." + "robot_base_frame", robot_base_frame_);

  dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(
      &SaftyLayer::dynamicParametersCallback, this, std::placeholders::_1));

  polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "vision_costmap_polygon_stamped", 1);

  RCLCPP_INFO(logger_, "Subscribed to Topics: %s", topics_string.c_str());

  rolling_window_ = layered_costmap_->isRolling();

  default_value_ = LETHAL_OBSTACLE;

  SaftyLayer::matchSize();
  current_ = true;
  was_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // now we need to split the topics based on whitespace which we can use a
  // stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {
    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_ground_height,
        max_ground_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing;

    declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
    declareParameter(source + "." + "sensor_frame",
                     rclcpp::ParameterValue(std::string("")));
    declareParameter(source + "." + "observation_persistence",
                     rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "expected_update_rate",
                     rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "data_type",
                     rclcpp::ParameterValue(std::string("LaserScan")));
    declareParameter(source + "." + "min_ground_height",
                     rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "max_ground_height",
                     rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "inf_is_valid",
                     rclcpp::ParameterValue(false));
    declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "obstacle_max_range",
                     rclcpp::ParameterValue(2.5));
    declareParameter(source + "." + "obstacle_min_range",
                     rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "raytrace_max_range",
                     rclcpp::ParameterValue(3.0));
    declareParameter(source + "." + "raytrace_min_range",
                     rclcpp::ParameterValue(0.0));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(name_ + "." + source + "." + "sensor_frame",
                        sensor_frame);
    node->get_parameter(name_ + "." + source + "." + "observation_persistence",
                        observation_keep_time);
    node->get_parameter(name_ + "." + source + "." + "expected_update_rate",
                        expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "min_ground_height",
                        min_ground_height);
    node->get_parameter(name_ + "." + source + "." + "max_ground_height",
                        max_ground_height);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid",
                        inf_is_valid);
    node->get_parameter(name_ + "." + source + "." + "clearing", clearing);
    if (!(data_type == "PointCloud2")) {
      RCLCPP_FATAL(
          logger_,
          "Only topics that use point cloud2s are currently supported");
      throw std::runtime_error(
          "Only topics that use point cloud2s are currently supported");
    }

    // get the obstacle range for the sensor
    double obstacle_max_range, obstacle_min_range;
    node->get_parameter(name_ + "." + source + "." + "obstacle_max_range",
                        obstacle_max_range);
    node->get_parameter(name_ + "." + source + "." + "obstacle_min_range",
                        obstacle_min_range);

    RCLCPP_DEBUG(
        logger_,
        "Creating an observation buffer for source %s, topic %s, frame %s",
        source.c_str(), topic.c_str(), sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
        std::shared_ptr<ObservationBuffer>(new ObservationBuffer(
            node, topic, observation_keep_time, expected_update_rate,
            min_ground_height, max_ground_height, obstacle_max_range,
            obstacle_min_range, 0, 0, *tf_, global_frame_, sensor_frame,
            tf2::durationFromSec(transform_tolerance))));

    marking_buffers_.push_back(observation_buffers_.back());

    RCLCPP_DEBUG(logger_,
                 "Created an observation buffer for source %s, topic %s, "
                 "global frame: %s, "
                 "expected update rate: %.2f, observation persistence: %.2f",
                 source.c_str(), topic.c_str(), global_frame_.c_str(),
                 expected_update_rate, observation_keep_time);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    auto sub = std::make_shared<message_filters::Subscriber<
        sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>(
        node, topic, custom_qos_profile, sub_opt);
    sub->unsubscribe();

    if (inf_is_valid) {
      RCLCPP_WARN(logger_,
                  "obstacle_layer: inf_is_valid option is not applicable to "
                  "PointCloud observations.");
    }

    auto filter =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            *sub, *tf_, global_frame_, 50, node->get_node_logging_interface(),
            node->get_node_clock_interface(),
            tf2::durationFromSec(transform_tolerance));

    filter->registerCallback(std::bind(&SaftyLayer::pointCloud2Callback, this,
                                       std::placeholders::_1,
                                       observation_buffers_.back()));

    observation_subscribers_.push_back(sub);
    observation_notifiers_.push_back(filter);

    if (sensor_frame != "") {
      std::vector<std::string> target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }
}
void SaftyLayer::publishPolygonMarker() {
  auto nh = node_.lock();
  if (!nh) return;

  // Also publish as PolygonStamped so it can be displayed like a footprint
  if (polygon_pub_) {
    geometry_msgs::msg::PolygonStamped poly_msg;
    poly_msg.header.frame_id = global_frame_;
    poly_msg.header.stamp = nh->now();
    poly_msg.polygon.points.reserve(polygon_points_.size());
    for (auto &pt : polygon_points_) {
      geometry_msgs::msg::Point32 p32;
      p32.x = static_cast<float>(pt.first);
      p32.y = static_cast<float>(pt.second);
      p32.z = 0.0f;
      poly_msg.polygon.points.push_back(p32);
    }
    polygon_pub_->publish(poly_msg);
  }
}

rcl_interfaces::msg::SetParametersResult SaftyLayer::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto& param_type = parameter.get_type();
    const auto& param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "min_ground_height") {
        min_ground_height_ = parameter.as_double();
      } else if (param_name == name_ + "." + "max_ground_height") {
        max_ground_height_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" &&
          enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        if (enabled_) {
          current_ = false;
        }
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
    }
  }

  result.successful = true;
  return result;
}

void SaftyLayer::pointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<ObservationBuffer>& buffer) {
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void SaftyLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x,
                              double* max_y) {
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2,
                 robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // update the global current status
  current_ = current;

  // place the new obstacles into a priority queue... each with a priority of
  // zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin();
       it != observations.end(); ++it) {
    const Observation& obs = *it;

    const sensor_msgs::msg::PointCloud2& cloud = *(obs.cloud_);

    double sq_obstacle_max_range =
        obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range =
        obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too low, we won't add it
      if (pz < min_ground_height_) {
        RCLCPP_DEBUG(logger_, "The point is too low");
        continue;
      }

      // if the obstacle is too high or too far away from the robot we won't add
      // it
      if (pz > max_ground_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's
      // origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                       (py - obs.origin_.y) * (py - obs.origin_.y) +
                       (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_max_range) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      // if the point is too close, do not conisder it
      if (sq_dist < sq_obstacle_min_range) {
        RCLCPP_DEBUG(logger_, "The point is too close");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = FREE_SPACE;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  double half = region_size_ / 2.0;
  double c = cos(robot_yaw);
  double s = sin(robot_yaw);
  double cx = robot_x + forward_offset_ * c;
  double cy = robot_y + forward_offset_ * s;

  // 计算旋转正方形四个顶点
  polygon_points_.clear();
    polygon_points_.emplace_back(
      cx - half * c + half * s,
      cy - half * s - half * c);
    polygon_points_.emplace_back(
      cx + half * c + half * s,
      cy + half * s - half * c);
    polygon_points_.emplace_back(
      cx + half * c - half * s,
      cy + half * s + half * c);
    polygon_points_.emplace_back(
      cx - half * c - half * s,
      cy - half * s + half * c);

  publishPolygonMarker();
}

void SaftyLayer::updateFootprint(double robot_x, double robot_y,
                                 double robot_yaw, double* min_x, double* min_y,
                                 double* max_x, double* max_y) {
  if (!footprint_clearing_enabled_) {
    return;
  }
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(),
                                      transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x,
          min_y, max_x, max_y);
  }
}

void SaftyLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i,
                             int min_j, int max_i, int max_j) {
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void SaftyLayer::updateWithMax(nav2_costmap_2d::Costmap2D& master_grid,
                               int min_i, int min_j, int max_i,
                               int max_j) {
  // Update only cells whose world coordinates fall inside polygon_points_.
  unsigned int mx, my;
  double wx, wy;

  auto pointInPolygon = [&](double x, double y) -> bool {
    bool inside = false;
    size_t n = polygon_points_.size();
    if (n < 3) return false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
      double xi = polygon_points_[i].first, yi = polygon_points_[i].second;
      double xj = polygon_points_[j].first, yj = polygon_points_[j].second;
      bool intersect = ((yi > y) != (yj > y)) &&
                       (x < (xj - xi) * (y - yi) / (yj - yi + 1e-20) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  };

  unsigned char* master_array = master_grid.getCharMap();

  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      master_grid.mapToWorld(i, j, wx, wy);
      if (!pointInPolygon(wx, wy)) continue;
      if (!worldToMap(wx, wy, mx, my)) continue;

      unsigned int layer_index = getIndex(mx, my);
      unsigned char layer_cost = costmap_[layer_index];
      if (layer_cost == NO_INFORMATION) continue;

      unsigned int master_index = master_grid.getIndex(i, j);
      unsigned char old_cost = master_array[master_index];

      if (old_cost == NO_INFORMATION || layer_cost > old_cost) {
        master_array[master_index] = layer_cost;
      }
    }
  }
  current_ = true;
}

void SaftyLayer::addStaticObservation(nav2_costmap_2d::Observation& obs, bool marking, bool clearing) {
  if (marking) {
    static_marking_observations_.push_back(obs);
  }
  if (clearing) {
    static_clearing_observations_.push_back(obs);
  }
}

void SaftyLayer::clearStaticObservations(bool marking, bool clearing) {
  if (marking) {
    static_marking_observations_.clear();
  }
  if (clearing) {
    static_clearing_observations_.clear();
  }
}

bool SaftyLayer::getMarkingObservations(
    std::vector<Observation>& marking_observations) const {
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(),
                              static_marking_observations_.end());
  return current;
}

void SaftyLayer::activate() {
  for (auto& notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }

  resetBuffersLastUpdated();
  RCLCPP_INFO(logger_, "SaftyLayer plugin activated");
}

void SaftyLayer::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating SaftyLayer plugin");
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

void SaftyLayer::reset() {
  RCLCPP_INFO(logger_, "Resetting SaftyLayer plugin");
  resetMaps();
  resetBuffersLastUpdated();
  current_ = false;
  was_reset_ = true;
}

void SaftyLayer::resetBuffersLastUpdated() {
  for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i]) {
      observation_buffers_[i]->resetLastUpdated();
    }
  }
}

}  // namespace aid_costmap_plugin
