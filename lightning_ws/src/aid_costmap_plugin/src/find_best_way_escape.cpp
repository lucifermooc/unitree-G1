#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "aid_costmap_plugin/cube_footprint_collision_checker.hpp"
#include "aid_robot_msgs/srv/find_best_velocity.hpp"
#include "nav2_core/behavior.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

typedef std::vector<geometry_msgs::msg::Point> Footprint;
class FindBestWayEscape : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief 寻找最佳逃生路径
   *
   * 初始化 FindBestWayEscape 类，设置相关参数和成员变量。
   *
   * 继承自 nav2_util::LifecycleNode，用于管理节点的生命周期。
   *
   * 成员变量包括：
   * - footprint_collision_checker_：足迹碰撞检测器指针（默认为 nullptr）
   * - simulate_ahead_time_：模拟前瞻时间（默认为 0.0）
   * - global_frame_：全局坐标系名称（默认为 "map"）
   * - robot_base_frame_：机器人基座坐标系名称（默认为 "base_link"）
   */
  FindBestWayEscape()
      : nav2_util::LifecycleNode("find_best_way_escape"),
        footprint_collision_checker_(nullptr),
        simulate_ahead_time_(0.0),
        global_frame_("map"),
        robot_base_frame_("base_link")
  {
    logger_ = this->get_logger();
    declare_parameter(
        "costmap_topic",
        rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
    declare_parameter("footprint_topic",
                      rclcpp::ParameterValue(
                          std::string("local_costmap/published_footprint")));
    declare_parameter("robot_base_frame_",
                      rclcpp::ParameterValue(std::string("base_link")));
    declare_parameter("global_frame",
                      rclcpp::ParameterValue(std::string("map")));

    declare_parameter("simulate_ahead_time", rclcpp::ParameterValue(2.0));
    declare_parameter("transform_tolerance", rclcpp::ParameterValue(1.0));
    declare_parameter("wheelbase", rclcpp::ParameterValue(0.6));
    declare_parameter("max_steering_angle", rclcpp::ParameterValue(0.3));
    declare_parameter("diff_num", rclcpp::ParameterValue(5));
    
  }

  /**
   * @brief 配置函数
   *
   * 在生命周期节点状态转变时进行配置。
   *
   * @param previous_state 之前的生命周期状态（未使用）
   *
   * @return 配置成功则返回 nav2_util::CallbackReturn::SUCCESS
   */
  nav2_util::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    std::string costmap_topic, footprint_topic;

    this->get_parameter("transform_tolerance", transform_tolerance_);
    this->get_parameter("costmap_topic", costmap_topic);
    this->get_parameter("footprint_topic", footprint_topic);
    this->get_parameter("robot_base_frame_", robot_base_frame_);
    this->get_parameter("global_frame", global_frame_);
    this->get_parameter("simulate_ahead_time", simulate_ahead_time_);
    this->get_parameter("wheelbase", wheelbase_);
    this->get_parameter("max_steering_angle", max_steering_angle_);
    this->get_parameter("diff_num", diff_num_);
    costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
        shared_from_this(), costmap_topic);
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
        shared_from_this(), footprint_topic, *tf_, robot_base_frame_,
        transform_tolerance_);

    collision_checker_ =
        std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
            *costmap_sub_, *footprint_sub_, this->get_name());
    obstacle_srv_ = this->create_service<aid_robot_msgs::srv::FindBestVelocity>(
        "/find_best_way_escape",
        std::bind(&FindBestWayEscape::obstacle_service_callback, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3));

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    transform_listener_.reset();
    tf_.reset();
    footprint_sub_.reset();
    costmap_sub_.reset();
    collision_checker_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  double CheckSteeringAngle(double v, double omega) const
  {
    if (omega == 0 || v == 0)
      return 0;

    double radius = v / omega;

    if (abs(std::atan(wheelbase_ / radius)) > max_steering_angle_)
    {
      omega = v * std::tan(max_steering_angle_ * ((omega<0)?(-1):(1))) / wheelbase_;
    }

    return omega;
  }

  /**
   * @brief 障碍物服务回调函数
   *
   * 当接收到障碍物服务请求时，此回调函数会被调用。它基于给定的请求参数，在成本地图中查找最佳速度，
   * 并将结果填充到响应中。
   *
   * @param request_header 请求头（当前未使用）
   * @param request 障碍物服务请求，包含最大速度和最大角速度等参数
   * @param response 障碍物服务响应，用于返回计算得到的最佳速度和是否成功的状态
   */
  void obstacle_service_callback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<aid_robot_msgs::srv::FindBestVelocity::Request>
          request,
      const std::shared_ptr<aid_robot_msgs::srv::FindBestVelocity::Response>
          response)
  {
    auto costmap = costmap_sub_->getCostmap();
    nav2_costmap_2d::FootprintCollisionChecker collision_checker(costmap);
    nav2_costmap_2d::Footprint footprint;
    geometry_msgs::msg::Twist get_cmd_vel;
    double max_angular_velocity = CheckSteeringAngle(request->max_speed,request->max_angular_velocity);
    if (FindBestSpeed(request->len, request->max_speed,
                      max_angular_velocity, get_cmd_vel))
    {
      response->cmd_vel = get_cmd_vel;
      response->success = true;
      response->msg = "success";
      return;
    }
    response->cmd_vel = get_cmd_vel;
    response->success = false;
    response->msg = "false";
  }

  /**
   * @brief 评估姿态的得分
   *
   * 根据给定的姿态和是否获取代价地图及足迹，评估姿态的得分。
   *
   * @param pose 姿态信息，类型为geometry_msgs::msg::Pose2D
   * @param fetch_costmap_and_footprint 是否获取代价地图和足迹的布尔值
   *
   * @return 返回姿态的得分，类型为double
   *
   * @throw nav2_costmap_2d::CollisionCheckerException 如果获取代价地图时发生运行时错误，则抛出此异常
   *
   * @note 如果姿态超出网格范围，则记录日志信息（未抛出异常）
   */
  double ScorePose(const geometry_msgs::msg::Pose2D &pose,
                   bool fetch_costmap_and_footprint)
  {
    if (fetch_costmap_and_footprint)
    {
      try
      {
        footprint_collision_checker_.setCostmap(costmap_sub_->getCostmap());
      }
      catch (const std::runtime_error &e)
      {
        throw nav2_costmap_2d::CollisionCheckerException(e.what());
      }
    }

    unsigned int cell_x, cell_y;
    if (!footprint_collision_checker_.worldToMap(pose.x, pose.y, cell_x,
                                                 cell_y))
    {
      RCLCPP_DEBUG(this->logger_, "Map Cell: [%d, %d]", cell_x, cell_y);
      // throw IllegalPoseException(name_, "Pose Goes Off Grid.");
    }

    return footprint_collision_checker_.footprintCost(
        getFootprint(pose, fetch_costmap_and_footprint));
  }
  /**
   * @brief 获取足迹
   *
   * 根据给定的位姿和是否获取最新足迹的标志，获取并返回足迹信息。
   *
   * @param pose 位姿，包含 x、y 和 theta 信息
   * @param fetch_latest_footprint 是否获取最新足迹的标志，true 表示获取最新足迹，false 表示使用当前足迹
   *
   * @return 返回计算后的足迹信息
   *
   * @throw nav2_costmap_2d::CollisionCheckerException 如果当前足迹不可用，则抛出异常
   */
  Footprint getFootprint(const geometry_msgs::msg::Pose2D &pose,
                         bool fetch_latest_footprint)
  {
    if (fetch_latest_footprint)
    {
      std_msgs::msg::Header header;
      if (!footprint_sub_->getFootprintInRobotFrame(footprint_, header))
      {
        throw nav2_costmap_2d::CollisionCheckerException(
            "Current footprint not available.");
      }
    }
    Footprint footprint;
    nav2_costmap_2d::transformFootprint(pose.x, pose.y, pose.theta, footprint_,
                                        footprint);

    return footprint;
  }

  /**
   * @brief 模拟路径检查并获取姿态评分
   *
   * 根据给定的距离、速度命令和初始姿态，模拟前进并计算姿态评分。
   *
   * @param distance 距离值
   * @param cmd_vel 速度命令，包含线速度和角速度
   * @param pose2d 初始姿态
   *
   * @return 姿态评分，如果评分失败则返回-1
   */
  double SimPathCheckAndGetPoseScore(const double &distance,
                                     geometry_msgs::msg::Twist &cmd_vel,
                                     geometry_msgs::msg::Pose2D &pose2d)
  {
    // Simulate ahead by simulate_ahead_time_ in cycle_frequency increments
    int cycle_count = 0;
    int cycle_frequency = 10;
    double sim_position_change = 0;
    const int max_cycle_count =
        static_cast<int>(cycle_frequency * simulate_ahead_time_);
    geometry_msgs::msg::Pose2D sim_pose = pose2d;
    geometry_msgs::msg::Pose2D last_sim_pose = pose2d;
    const double diff_dist = distance;
    bool fetch_data = true;
    double sim_diff_angle = cmd_vel.angular.z / cycle_frequency;
    double sim_diff_len = cmd_vel.linear.x / cycle_frequency;
    double pose_score = 0;
    while (cycle_count < max_cycle_count)
    {
      sim_pose.x = sim_pose.x + sim_diff_len * cos(sim_pose.theta);
      sim_pose.y = sim_pose.y + sim_diff_len * sin(sim_pose.theta);
      sim_pose.theta = sim_pose.theta + sim_diff_angle;
      cycle_count++;
      double diff_x = sim_pose.x - last_sim_pose.x;
      double diff_y = sim_pose.y - last_sim_pose.y;
      double diff_len = hypot(diff_x, diff_y);
      sim_position_change += diff_len;
      last_sim_pose = sim_pose;

      if (diff_dist - abs(sim_position_change) <= 0.)
      {
        break;
      }

      pose_score = ScorePose(sim_pose, fetch_data);

      if (pose_score == -1)
      {
        return -1;
      }
      fetch_data = false;
    }
    return pose_score;
  }

  /**
   * @brief 查找最佳速度
   *
   * 根据给定的长度、最大速度和最大角速度，在给定的机器人当前姿态下，计算并返回最佳的速度指令。
   *
   * @param len 长度
   * @param max_vel 最大速度
   * @param max_angular_speed 最大角速度
   * @param get_cmd_vel 存储最佳速度指令的变量
   *
   * @return 如果找到最佳速度指令，则返回true；否则返回false
   */
  bool FindBestSpeed(double len, double max_vel, double max_angular_speed,
                     geometry_msgs::msg::Twist &get_cmd_vel)
  {
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::Pose2D pose2d;
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(current_pose, *tf_, this->global_frame_,
                                   this->robot_base_frame_,
                                   this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return false;
    }

    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);
    double min_pose_score = DBL_MAX;
    double current_pose_score;
    geometry_msgs::msg::Twist min_score_cmd_vel;
    double diff_speed = max_vel/diff_num_;
    double diff_omega = max_angular_speed/diff_num_;

    for (double speed = max_vel; speed > -max_vel; speed -= diff_speed)
      for (double angular_speed = max_angular_speed;
           angular_speed > -max_angular_speed; angular_speed -= diff_omega)
      {
        cmd_vel.angular.z = angular_speed;
        cmd_vel.linear.x = speed;
        current_pose_score = SimPathCheckAndGetPoseScore(len, cmd_vel, pose2d);
        if (current_pose_score == -1)
        {
          continue;
        }
        std::cout << "angular_speed " << angular_speed <<" speed "<<speed << " current_pose_score " << current_pose_score << std::endl;

        if (current_pose_score < min_pose_score)
        {
          min_pose_score = current_pose_score;
          min_score_cmd_vel = cmd_vel;
        }
      }
    if (min_pose_score == DBL_MAX)
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not find best way.");
      return false;
    }
    std::cout << "best_speed" << min_score_cmd_vel.linear.x << std::endl;
    std::cout << "best_angular_speed" << min_score_cmd_vel.angular.z << std::endl;
    std::cout << "min_pose_score" << min_pose_score << std::endl;
    get_cmd_vel = min_score_cmd_vel;
    return true;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>
      collision_checker_;
  nav2_costmap_2d::CubeFootprintCollisionChecker<
      std::shared_ptr<nav2_costmap_2d::Costmap2D>>
      footprint_collision_checker_;
  Footprint footprint_;
  double simulate_ahead_time_;
  rclcpp::Service<aid_robot_msgs::srv::FindBestVelocity>::SharedPtr
      obstacle_srv_;
  std::string robot_base_frame_, global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("find_best_way_escape")};
  double transform_tolerance_;
  double wheelbase_;
  double max_steering_angle_;
  int diff_num_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FindBestWayEscape>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
