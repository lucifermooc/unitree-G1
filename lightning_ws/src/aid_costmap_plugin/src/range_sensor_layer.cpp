/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 David V. Lu!!
 *  Copyright (c) 2020, Bytes Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "aid_costmap_plugin/range_sensor_layer.hpp"

PLUGINLIB_EXPORT_CLASS(aid_costmap_plugin::RangeSensorLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using namespace std::literals::chrono_literals;

namespace aid_costmap_plugin
{

RangeSensorLayer::RangeSensorLayer() {}

void RangeSensorLayer::onInitialize()
{
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(20)) 
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  // 初始化传感器层的基本状态
  current_ = true;           // 标记当前层状态为有效
  was_reset_ = false;        // 重置标志
  buffered_readings_ = 0;    // 缓存的传感器读数计数器
  last_reading_time_ = clock_->now();  // 记录最后一次读数时间
  default_value_ = to_cost(0.5);  // 默认代价值（中性概率）

  matchSize();  // 匹配代价地图大小
  resetRange();  // 重置范围边界

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 配置传感器层参数，这些参数控制传感器数据处理的各个方面
  
  // 是否启用传感器层
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  
  // phi参数：影响传感器模型的距离衰减特性
  // 控制传感器对不同距离的敏感程度
  declareParameter("phi", rclcpp::ParameterValue(1.2));
  node->get_parameter(name_ + "." + "phi", phi_v_);
  
  // 传感器锥形区域膨胀参数，控制感知区域大小
  // 1.0表示完全覆盖，小于1.0则只部分覆盖
  declareParameter("inflate_cone", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "inflate_cone", inflate_cone_);
  
  // 无传感器读数超时时间，超过此时间没有读数会触发警告
  declareParameter("no_readings_timeout", rclcpp::ParameterValue(0.0));
  node->get_parameter(name_ + "." + "no_readings_timeout", no_readings_timeout_);
  
  // 清除阈值：低于此值的代价被视为自由空间
  // 默认0.2意味着低概率区域被认为是可通行的
  declareParameter("clear_threshold", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + "." + "clear_threshold", clear_threshold_);
  
  // 标记阈值：高于此值的代价被视为致命障碍
  // 默认0.8意味着高概率区域被认为是不可通行的
  declareParameter("mark_threshold", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
  
  // 是否在最大读数时清除传感器锥形区域
  // 对于一些特定类型的传感器可能有用
  declareParameter("clear_on_max_reading", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "clear_on_max_reading", clear_on_max_reading_);

  // 坐标变换容差
  double temp_tf_tol = 0.0;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  // 订阅的传感器主题列表
  std::vector<std::string> topic_names{};
  declareParameter("topics", rclcpp::ParameterValue(topic_names));
  node->get_parameter(name_ + "." + "topics", topic_names);

  // 传感器输入类型：固定、可变或全部
  InputSensorType input_sensor_type = InputSensorType::ALL;
  std::string sensor_type_name;
  declareParameter("input_sensor_type", rclcpp::ParameterValue("ALL"));
  node->get_parameter(name_ + "." + "input_sensor_type", sensor_type_name);

  std::transform(
    sensor_type_name.begin(), sensor_type_name.end(),
    sensor_type_name.begin(), ::toupper);

  // 根据配置选择传感器类型
  if (sensor_type_name == "VARIABLE") {
    input_sensor_type = InputSensorType::VARIABLE;
  } else if (sensor_type_name == "FIXED") {
    input_sensor_type = InputSensorType::FIXED;
  } else if (sensor_type_name == "ALL") {
    input_sensor_type = InputSensorType::ALL;
  } else {
    RCLCPP_ERROR(
      logger_, "%s: Invalid input sensor type: %s. Defaulting to ALL.",
      name_.c_str(), sensor_type_name.c_str());
  }

  // 验证传感器主题列表
  if (topic_names.empty()) {
    RCLCPP_FATAL(
      logger_, "Invalid topic names list: it must"
      "be a non-empty list of strings");
    return;
  }

  // 订阅所有传感器主题
  for (auto & topic_name : topic_names) {
    // 根据传感器类型选择消息处理函数
    if (input_sensor_type == InputSensorType::VARIABLE) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processVariableRangeMsg, this,
        std::placeholders::_1);
    } else if (input_sensor_type == InputSensorType::FIXED) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processFixedRangeMsg, this,
        std::placeholders::_1);
    } else if (input_sensor_type == InputSensorType::ALL) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processRangeMsg, this,
        std::placeholders::_1);
    } else {
      RCLCPP_ERROR(
        logger_,
        "%s: Invalid input sensor type: %s. Did you make a new type"
        "and forgot to choose the subscriber for it?",
        name_.c_str(), sensor_type_name.c_str());
    }
    
    // 创建传感器消息订阅
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  callback_group_executor_->add_callback_group(callback_group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  range_subs_.push_back(
    node->create_subscription<sensor_msgs::msg::Range>(
      topic_name, custom_qos, std::bind(
        &RangeSensorLayer::bufferIncomingRangeMsg, this,
        std::placeholders::_1)));
    RCLCPP_INFO(
      logger_, "RangeSensorLayer: subscribed to "
      "topic %s", range_subs_.back()->get_topic_name());
  }
  global_frame_ = layered_costmap_->getGlobalFrameID();
}

// Gamma函数：计算角度衰减权重
// 目的：根据角度偏差计算传感器视野内的权重衰减
// 参数：
// - theta: 当前点相对传感器朝向的角度偏差
// - max_angle_: 传感器的最大视野角度
// 返回值：[0, 1]之间的权重，越接近传感器中心权重越高
double RangeSensorLayer::gamma(double theta)
{
  // 如果角度超出最大视野角度，权重为0
  if (fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    // 使用二次衰减函数，越接近中心权重越高
    // 权重 = 1 - (角度/最大角度)^2
    return 1 - pow(theta / max_angle_, 2);
  }
}

// Delta函数：计算距离衰减权重
// 目的：根据距离计算传感器探测的衰减特性
// 参数：
// - phi: 当前点到传感器的距离
// - phi_v_: 预设的参考距离，影响衰减曲线
// 返回值：[0, 1]之间的权重，反映距离对传感器探测的影响
double RangeSensorLayer::delta(double phi)
{
  // 使用双曲正切函数计算距离衰减
  // 1 - (1 + tanh(2 * (phi - phi_v_))) / 2
  // 这个公式可以根据phi_v_创建非对称的衰减曲线
  return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

// 传感器模型：计算单元格的占用概率
// 目的：根据传感器测量数据和空间特征，估算单元格的占用概率
// 参数：
// - r: 传感器测量距离（传感器探测到的最近距离）
// - phi: 当前点到传感器的距离
// - theta: 当前点相对传感器朝向的角度偏差
// 返回值：单元格的占用概率，范围[0, 1]
double RangeSensorLayer::sensor_model(double r, double phi, double theta)
{
  // 结合角度和距离衰减
  // lbda是综合的衰减权重，考虑了角度和距离两个维度
  double lbda = delta(phi) * gamma(theta);

  // 分辨率相关的距离误差
  // 考虑传感器测量的不确定性
  double delta = resolution_;

  // 根据距离和角度，分段计算占用概率
  // 这个模型模拟了传感器在不同区域的探测特性
  if (phi >= 0.0 && phi < r - 2 * delta * r) {
    // 远离传感器测量距离的区域，基本概率为0.5
    // 表示这个区域信息不确定
    return (1 - lbda) * (0.5);
  } else if (phi < r - delta * r) {
    // 接近传感器测量距离的前半区域
    // 使用二次函数计算概率，考虑距离和角度衰减
    // 概率随距离非线性变化
    return lbda * 0.5 * pow((phi - (r - 2 * delta * r)) / (delta * r), 2) +
           (1 - lbda) * .5;
  } else if (phi < r + delta * r) {
    // 接近传感器测量距离的后半区域
    // 使用复杂的概率计算公式
    // 反映传感器在测量距离附近的探测特性
    double J = (r - phi) / (delta * r);
    return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else {
    // 远离传感器测量距离的区域，基本概率为0.5
    // 表示这个区域信息不确定
    return 0.5;
  }
}

// 缓存传入的传感器消息
void RangeSensorLayer::bufferIncomingRangeMsg(
  const sensor_msgs::msg::Range::SharedPtr range_message)
{
  // RCLCPP_INFO(logger_, "bufferIncomingRangeMsg: r=%.3f frame=%s", 
  //     range_message->range, range_message->header.frame_id.c_str());
  // 使用互斥锁确保线程安全
  range_message_mutex_.lock();
  range_msgs_buffer_.push_back(*range_message);
  range_message_mutex_.unlock();
}

// 更新代价地图
void RangeSensorLayer::updateCostmap()
{
  // 创建消息缓存副本，避免在处理过程中持续锁定
  std::list<sensor_msgs::msg::Range> range_msgs_buffer_copy;

  range_message_mutex_.lock();
  range_msgs_buffer_copy = std::list<sensor_msgs::msg::Range>(range_msgs_buffer_);
  range_msgs_buffer_.clear();
  range_message_mutex_.unlock();

  // 处理所有缓存的传感器消息
  for (auto & range_msgs_it : range_msgs_buffer_copy) {
    processRangeMessageFunc_(range_msgs_it);
  }
}

// 处理传感器消息的通用方法
void RangeSensorLayer::processRangeMsg(sensor_msgs::msg::Range & range_message)
{
  // 根据传感器的最小和最大距离判断传感器类型
  if (range_message.min_range == range_message.max_range) {
    processFixedRangeMsg(range_message);
  } else {
    processVariableRangeMsg(range_message);
  }
}

// 处理固定距离传感器的消息
void RangeSensorLayer::processFixedRangeMsg(sensor_msgs::msg::Range & range_message)
{
  // 固定距离传感器只接受无穷大的值
  if (!std::isinf(range_message.range)) {
    RCLCPP_ERROR(
      logger_,
      "Fixed distance ranger (min_range == max_range) in frame %s sent invalid value. "
      "Only -Inf (== object detected) and Inf (== no object detected) are valid.",
      range_message.header.frame_id.c_str());
    return;
  }

  bool clear_sensor_cone = false;

  // 处理最大读数情况
  if (range_message.range > 0) {  // +inf
    if (!clear_on_max_reading_) {
      return;  // 不清除传感器锥形区域
    }
    clear_sensor_cone = true;
  }

  // 设置传感器距离为最小距离
  range_message.range = range_message.min_range;

  // 更新代价地图
  updateCostmap(range_message, clear_sensor_cone);
}

// 处理可变距离传感器的消息
void RangeSensorLayer::processVariableRangeMsg(sensor_msgs::msg::Range & range_message)
{
  // 检查距离是否在有效范围内
  if (range_message.range < range_message.min_range || range_message.range >
    range_message.max_range)
  {
    return;
  }

  bool clear_sensor_cone = false;

  // 处理最大读数情况
  if (range_message.range >= range_message.max_range && clear_on_max_reading_) {
    clear_sensor_cone = true;
  }

  // 更新代价地图
  updateCostmap(range_message, clear_sensor_cone);
}

// 更新代价地图的核心方法
void RangeSensorLayer::updateCostmap(
  sensor_msgs::msg::Range & range_message,
  bool clear_sensor_cone)
{
  // RCLCPP_INFO(logger_, "updateCostmap: r=%.3f", range_message.range);
  // 计算传感器的最大角度（视野的一半）
  max_angle_ = range_message.field_of_view / 2;

  // 准备坐标变换
  geometry_msgs::msg::PointStamped in, out;
  in.header.stamp = range_message.header.stamp;
  in.header.frame_id = range_message.header.frame_id;

  // 检查是否可以进行坐标变换
  if (!tf_->canTransform(
      in.header.frame_id, global_frame_,
      tf2_ros::fromMsg(in.header.stamp),
      tf2_ros::fromRclcpp(transform_tolerance_)))
  {
    // RCLCPP_INFO(
    //   logger_, "Range sensor layer can't transform from %s to %s",
    //   global_frame_.c_str(), in.header.frame_id.c_str());
    return;
  }

  // 变换传感器原点坐标
  tf_->transform(in, out, global_frame_, transform_tolerance_);

  double ox = out.point.x, oy = out.point.y;

  // 变换传感器测量点坐标
  in.point.x = range_message.range;

  tf_->transform(in, out, global_frame_, transform_tolerance_);

  double tx = out.point.x, ty = out.point.y;

  // 计算目标属性
  double dx = tx - ox, dy = ty - oy, theta = atan2(dy, dx), d = sqrt(dx * dx + dy * dy);

  // 整数边界更新
  int bx0, by0, bx1, by1;

  // 将被更新的三角形区域（由原点和传感器锥形两侧形成）
  int Ox, Oy, Ax, Ay, Bx, By;

  // 包含原点的边界
  worldToMapNoBounds(ox, oy, Ox, Oy);
  bx1 = bx0 = Ox;
  by1 = by0 = Oy;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // 更新目标点的地图
  unsigned int aa, ab;
  if (worldToMap(tx, ty, aa, ab)) {
    setCost(aa, ab, 233);
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }

  double mx, my;

  // 更新传感器锥形左侧
  mx = ox + cos(theta - max_angle_) * d * 1.2;
  my = oy + sin(theta - max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Ax, Ay);
  bx0 = std::min(bx0, Ax);
  bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay);
  by1 = std::max(by1, Ay);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // 更新传感器锥形右侧
  mx = ox + cos(theta + max_angle_) * d * 1.2;
  my = oy + sin(theta + max_angle_) * d * 1.2;

  worldToMapNoBounds(mx, my, Bx, By);
  bx0 = std::min(bx0, Bx);
  bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By);
  by1 = std::max(by1, By);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // 限制边界到网格范围
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min(static_cast<int>(size_x_), bx1);
  by1 = std::min(static_cast<int>(size_y_), by1);

  // 遍历更新区域内的每个单元格
  for (unsigned int x = bx0; x <= (unsigned int)bx1; x++) {
    for (unsigned int y = by0; y <= (unsigned int)by1; y++) {
      bool update_xy_cell = true;

      // 根据膨胀参数控制更新区域
      // 除非inflate_cone_设置为100%，否则只更新传感器锥形内的单元格
      if (inflate_cone_ < 1.0) {
        // 确定重心坐标
        int w0 = orient2d(Ax, Ay, Bx, By, x, y);
        int w1 = orient2d(Bx, By, Ox, Oy, x, y);
        int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

        // 重心坐标区域阈值
        // 这个计算不是严格数学意义上正确，但实际上有效
        float bcciath = -static_cast<float>(inflate_cone_) * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
      }

      // 更新单元格
      if (update_xy_cell) {
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        update_cell(ox, oy, theta, range_message.range, wx, wy, clear_sensor_cone);
      }
    }
  }

  // 遍历代价地图，检查并清理之前被占据的区域
  for (unsigned int x = 0; x < size_x_; x++) {
    for (unsigned int y = 0; y < size_y_; y++) {
      unsigned char prob = getCost(x, y);
      
      // 如果区域概率高于mark_threshold_，表示被认为是障碍
      if (prob > to_cost(mark_threshold_)) {
        double wx, wy;
        mapToWorld(x, y, wx, wy);

        // 判断该点是否在传感器移动路径之外
        if (std::abs(wx - ox) > 0.1 && std::abs(wy - oy) > 0.1) {
          // 将高概率区域降低到clear_threshold_
          setCost(x, y, to_cost(clear_threshold_));
        }
      }
    }
  }

  // 更新读数计数器和最后读数时间
  buffered_readings_++;
  last_reading_time_ = clock_->now();
}

// 更新单个代价地图单元格
void RangeSensorLayer::update_cell(
  double ox, double oy, double ot, double r,
  double nx, double ny, bool clear)
{
  unsigned int x, y;
  // 将世界坐标转换为地图坐标
  if (worldToMap(nx, ny, x, y)) {
    // 计算相对坐标和角度
    double dx = nx - ox, dy = ny - oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx * dx + dy * dy);
    
    // 计算传感器模型的概率
    double sensor = 0.0;
    if (!clear) {
      sensor = sensor_model(r, phi, theta);
    }

    // 贝叶斯概率更新
    // 1. 获取当前单元格的先验概率
    double prior = to_prob(getCost(x, y));
    
    // 2. 计算占用和非占用的联合概率
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    
    // 3. 计算后验概率（新的占用概率）
    double new_prob = prob_occ / (prob_occ + prob_not);

    // 调试信息：输出概率计算过程
    // RCLCPP_DEBUG(
    //   logger_,
    //   "%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    // RCLCPP_DEBUG(
    //   logger_,
    //   "%f 先验概率 | %f占用 %f 非占用| %f后验概率", prior, prob_occ, prob_not, new_prob);
    // RCLCPP_INFO(
    //   logger_,
    //   "%f dx %f dy | %f 角度 %f = %f 传感器模型的概率", dx, dy, theta, phi, sensor);
    // RCLCPP_INFO(
    //   logger_,
    //   "%f 先验概率 | %f占用 %f 非占用| %f后验概率", prior, prob_occ, prob_not, new_prob);
    if (new_prob >= mark_threshold_) {
      RCLCPP_INFO(
      logger_,
      "%f 先验概率 | %f占用 %f 非占用| %f后验概率 | %f传感器距离", prior, prob_occ, prob_not, new_prob, r);
    }
    // 将概率转换为代价并设置单元格
    unsigned char c = to_cost(new_prob);
    setCost(x, y, c);
  }
}

// 重置范围边界
void RangeSensorLayer::resetRange()
{
  // RCLCPP_INFO(logger_, "resetRange");
  // 将边界设置为最大/最小可能值
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

// 更新代价地图边界
void RangeSensorLayer::updateBounds(
  double robot_x, double robot_y,
  double robot_yaw, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  // RCLCPP_INFO(logger_, "updateBounds: buffered=%zu", range_msgs_buffer_.size());
  robot_yaw = 0 + robot_yaw;  // 避免未使用变量警告
  
  // 如果是滚动地图，更新地图原点
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // 更新代价地图
  updateCostmap();

  // 扩展边界
  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  // 重置范围
  resetRange();

  // 如果未启用，直接返回
  if (!enabled_) {
    current_ = true;
    return;
  }

  // 检查是否长时间没有传感器读数
  if (buffered_readings_ == 0) {
    if (no_readings_timeout_ > 0.0 &&
      (clock_->now() - last_reading_time_).seconds() >
      no_readings_timeout_)
    {
      RCLCPP_WARN(
        logger_,
        "No range readings received for %.2f seconds, while expected at least every %.2f seconds.",
        (clock_->now() - last_reading_time_).seconds(),
        no_readings_timeout_);
      current_ = false;
    }
  }
}

// 更新主代价地图的代价
void RangeSensorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // RCLCPP_INFO(
  //   logger_,
  //   "updateCosts: min_i=%d min_j=%d max_i=%d max_j=%d",
  //   min_i, min_j, max_i, max_j);
  // 如果未启用，直接返回
  if (!enabled_) {
    return;
  }

  // 获取主代价地图数组
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  
  // 获取清除和标记阈值
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  // 遍历指定区域的代价地图
  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char current;
      
      // 跳过无信息区域
      if (prob == nav2_costmap_2d::NO_INFORMATION) {
        it++;
        continue;
      } else if (prob > mark) {
        // 高于标记阈值：致命障碍
        current = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else if (prob < clear) {
        // 低于清除阈值：自由空间
        current = nav2_costmap_2d::FREE_SPACE;
      } else {
        // 介于阈值之间：保持不变
        it++;
        continue;
      }

      // 获取旧的代价值
      unsigned char old_cost = master_array[it];

      // 更新代价值（只有在新代价更高时才更新）
      if (old_cost == NO_INFORMATION || old_cost < current) {
        master_array[it] = current;
      }
      it++;
    }
  }

  // 重置缓存读数
  buffered_readings_ = 0;

  // 如果由于重置而不是当前状态，现在设置为当前
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
}

// 重置传感器层
void RangeSensorLayer::reset()
{
  // RCLCPP_INFO(logger_, "reset");
  RCLCPP_DEBUG(logger_, "Reseting range sensor layer...");
  deactivate();
  resetMaps();
  was_reset_ = true;
  activate();
}

void RangeSensorLayer::deactivate()
{
  range_msgs_buffer_.clear();
}

void RangeSensorLayer::activate()
{
  range_msgs_buffer_.clear();
}

}  // namespace aid_costmap_plugin
