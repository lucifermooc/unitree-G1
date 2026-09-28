/******************************************************************************
 * Copyright (c) 2023 dongfang chen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of chassis_comm nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <boost/asio.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "aid_robot_msgs/srv/control_launch.hpp"
#include "aid_robot_msgs/srv/get_current_map.hpp"
#include "aid_robot_msgs/srv/get_string.hpp"
#include "aid_robot_msgs/srv/map_operation.hpp"
#include "aid_robot_msgs/srv/status_change.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lightning/srv/save_map.hpp"
#include <filesystem>

using namespace std::chrono_literals;
using namespace std::placeholders;

class StatusManagerNode : public rclcpp::Node {
 private:
  std::string maping_launch_file =
      "/robot_bringup/share/robot_bringup/launch/g1_mapping.launch.py";
  std::string localization_launch_file =
      "/robot_bringup/share/robot_bringup/launch/g1_localization.launch.py";
  std::string navigation_launch_file =
      "/g1_nav_bridge/share/g1_nav_bridge/launch/g1_navigation.launch.py";
  std::string map_filepath_;
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string set_status_;
  std::string control_model_;
  std::string slam_status_;
  std::string map_filename_;
  // true: 本节点通过 launch_manager 启停 SLAM/Nav2；
  // false: 顶层 robot.launch.py 已经拥有这些进程，本节点只上报状态。
  bool manage_stack_ = true;
  bool navigation_started_ = false;
  // 由顶层 robot.launch.py 透传给各子 launch 的附加参数（"k:=v k2:=v2"），
  // 保证经本节点拉起的定位/导航/建图与直接由顶层启动时配置一致。
  std::string localization_args_;
  std::string navigation_args_;
  std::string mapping_args_;
  // 地图数据库没有可用的当前地图时，定位使用的兜底地图目录。
  std::string default_map_dir_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Service<aid_robot_msgs::srv::StatusChange>::SharedPtr
      status_change_server_;
  rclcpp::Service<aid_robot_msgs::srv::MapOperation>::SharedPtr
      map_save_server_;
  rclcpp::Client<aid_robot_msgs::srv::ControlLaunch>::SharedPtr   
   start_launch_client_;
  rclcpp::Client<aid_robot_msgs::srv::ControlLaunch>::SharedPtr  
   stop_launch_client_;
  rclcpp::Client<lightning::srv::SaveMap>::SharedPtr lightning_save_map_client_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;

  rclcpp::Client<aid_robot_msgs::srv::GetCurrentMap>::SharedPtr get_current_map_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      init_pose_sub_;
  rclcpp::Service<aid_robot_msgs::srv::GetString>::SharedPtr ip_server_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr change_map_client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr
      lifecycle_manager_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      lifecycle_navigation_is_active_client_;
  void GetIpHandleRequest(
      const std::shared_ptr<aid_robot_msgs::srv::GetString::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::GetString::Response> response) {
    (void)request;

    std::string ip_address = GetIpAddress();

    if (ip_address.empty()) {
      response->success = false;
      response->message = "Failed to get IP address";
      RCLCPP_ERROR(get_logger(), "Failed to get IP address");
    } else {
      response->success = true;
      response->message = "Success";
      response->result = ip_address;
      RCLCPP_INFO(get_logger(), "IP Address: %s", ip_address.c_str());
    }
  }

  bool ChangeMap(const std::string& map_filepath) {

    if (!change_map_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(node_->get_logger(), "change map service not available");
      return false;
    }
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_filepath;

    auto future = change_map_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(node_, future);

    auto status = future.get()->result;
    if (status != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Change map request failed!");
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Change map request was successful!");
      return true;
    }
    return true;
  }

  std::string GetIpAddress() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    std::string ip;
    if (sockfd == -1) {
      perror("socket");
      return "";
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ - 1);

    if (ioctl(sockfd, SIOCGIFADDR, &ifr) == -1) {
      perror("ioctl");
      close(sockfd);
      return "";
    }

    close(sockfd);

    struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
    char ip_address[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr->sin_addr, ip_address, INET_ADDRSTRLEN);
    ip.append(ip_address);
    return ip;
  }

  bool GetCurrentMap(std::string &filename) {

    auto request =
        std::make_shared<aid_robot_msgs::srv::GetCurrentMap::Request>();

    if (false == get_current_map_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(
          rclcpp::get_logger("get_current_map_client"),
          "Wait finish_trajectory failed,service not available, return ...");
      return false;
    }
    auto result = get_current_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("get_current_map_client"),
                         "message:success");
      auto service_response = result.get();
      if(service_response->success==false){
        RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                   "get current map failed");
        return false;
      }

      if(service_response->map_file.empty()){
        RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                   "map file name is empty");
        return false;
      }
      filename = ResolveLightningMapDirectory(service_response->map_file);
      if (filename.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                     "selected map is incomplete: %s",
                     service_response->map_file.c_str());
        return false;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                   "Failed to call service get_current_map_idp");
      return false;
    }
    return true;
  }

  static std::string JoinArgs(const std::string &a, const std::string &b) {
    if (a.empty()) return b;
    if (b.empty()) return a;
    return a + " " + b;
  }

  // 定位地图：优先用地图数据库的当前地图（与前端一致），否则用 default_map_dir。
  bool SelectLocalizationMap(std::string &directory) {
    if (GetCurrentMap(directory) && IsLightningMapComplete(directory)) {
      return true;
    }
    if (!default_map_dir_.empty()) {
      const std::string fallback = ResolveLightningMapDirectory(default_map_dir_);
      if (!fallback.empty()) {
        RCLCPP_WARN(get_logger(), "No usable current map in database, falling back to %s",
                    fallback.c_str());
        directory = fallback;
        return true;
      }
    }
    return false;
  }

  bool IsLightningMapComplete(const std::string &directory) const {
    const std::filesystem::path path(directory);
    return std::filesystem::exists(path / "index.txt") &&
           std::filesystem::exists(path / "map.yaml");
  }

  std::string ResolveLightningMapDirectory(const std::string &directory) const {
    if (IsLightningMapComplete(directory)) {
      return directory;
    }
    const std::filesystem::path legacy_directory =
        std::filesystem::path(directory) / "lightning";
    if (IsLightningMapComplete(legacy_directory.string())) {
      RCLCPP_WARN(get_logger(), "Using legacy Lightning map directory: %s",
                  legacy_directory.c_str());
      return legacy_directory.string();
    }
    return "";
  }

  bool LightningSaveMap(const std::string &requested_path) {
    std::filesystem::path path(requested_path);
    std::string map_id = path.filename().string();
    if (map_id.empty()) {
      map_id = path.parent_path().filename().string();
    }
    if (map_id.empty() || map_id == "." || map_id == "..") {
      RCLCPP_ERROR(get_logger(), "Invalid map path: %s", requested_path.c_str());
      return false;
    }

    if (!lightning_save_map_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service /lightning/save_map is unavailable");
      return false;
    }
    auto request = std::make_shared<lightning::srv::SaveMap::Request>();
    request->map_id = map_id;
    auto result = lightning_save_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(60)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Timed out while saving Lightning map %s", map_id.c_str());
      return false;
    }
    if (result.get()->response != 0) {
      RCLCPP_ERROR(get_logger(), "Lightning map save failed, code=%u", result.get()->response);
      return false;
    }

    const char *home = std::getenv("HOME");
    if (home == nullptr || path.parent_path() != "/maps") {
      RCLCPP_ERROR(get_logger(), "Map path must be /maps/<map_id>: %s",
                   requested_path.c_str());
      return false;
    }
    const std::filesystem::path target =
        std::filesystem::path(home) / "maps" / map_id;
    if (!IsLightningMapComplete(target.string())) {
      RCLCPP_ERROR(get_logger(), "Lightning map files are incomplete: %s",
                   target.c_str());
      return false;
    }
    return true;
  }
  // Legacy map loading helpers were removed. Lightning localization loads its
  // tiled map and Nav2's map_server owns the 2D occupancy map.

  void TimerCallback() {  // RCLCPP_INFO(this->get_logger(), "Timer callback");
    std_msgs::msg::String status;
    status.data = slam_status_ + "+" + control_model_;
    status_pub_->publish(status);
  }

  void InitPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header = pose->header;
    initial_pose.header.frame_id = "map";
    initial_pose.pose.pose = pose->pose;
    initial_pose.pose.covariance[0] = 0.25;
    initial_pose.pose.covariance[7] = 0.25;
    initial_pose.pose.covariance[35] = 0.0685;
    initial_pose_pub_->publish(initial_pose);
  }

  bool ControlNavigation(uint8_t command) {
    auto request =
        std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = command;

    if (!lifecycle_manager_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
      return false;
    }

    auto result = lifecycle_manager_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result,
                                          std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed");
      return false;
    }
    return result.get()->success;
  }

  bool PauseNavigation() {
    return ControlNavigation(
        nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE);
  }

  bool ResumeNavigation() {
    return ControlNavigation(
        nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME);
  }
  bool IsNavigationActive() {
    auto request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    if (!lifecycle_navigation_is_active_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
      return false;
    }

    auto result = lifecycle_navigation_is_active_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result,
                                          std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed");
      return false;
    }
    return result.get()->success;
  }

  // Nav2 由 launch_manager 刚拉起时 lifecycle 仍在 configure/activate（autostart，Thor 上约 5~15 s，
  // 且 costmap 要等到定位发出 map->base_link 才能激活）。此时立即 RESUME 会失败，
  // 前端（遥控后进导航、定位复位后立刻导航）就会报"状态切换失败"。
  bool WaitNavigationActive(std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (IsNavigationActive()) {
        return true;
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    return false;
  }

  bool StartNavigation(const std::string& map_directory) {
    const std::filesystem::path map_yaml =
        std::filesystem::path(map_directory) / "map.yaml";
    if (!std::filesystem::exists(map_yaml)) {
      RCLCPP_ERROR(get_logger(), "Navigation map does not exist: %s",
                   map_yaml.c_str());
      return false;
    }
    if (navigation_started_) {
      return true;
    }
    navigation_started_ = StartLaunch(
        navigation_launch_file,
        JoinArgs("map:=" + map_yaml.string() + " floor_z:=0.0 base_floor_z:=0.0",
                 navigation_args_));
    return navigation_started_;
  }

  bool StopNavigation() {
    if (!navigation_started_) {
      return true;
    }
    if (!StopLaunch(navigation_launch_file)) {
      return false;
    }
    navigation_started_ = false;
    return true;
  }

  bool StartLocalizationAndNavigation(const std::string& map_directory) {
    if (!StartLaunch(localization_launch_file,
                     JoinArgs("map_dir:=" + map_directory, localization_args_))) {
      return false;
    }
    if (StartNavigation(map_directory)) {
      return true;
    }
    StopLaunch(localization_launch_file);
    return false;
  }

  void SaveMapCallback(
      const std::shared_ptr<aid_robot_msgs::srv::MapOperation::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::MapOperation::Response> response) {
    if (slam_status_ == "mapping") {
      if (!LightningSaveMap(request->map_file_name)) {
        response->message = "lightning map save failed";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           response->message);
        response->success = false;
        return;
      }

      if (!StopLaunch(maping_launch_file)) {
        response->message = "failed to stop mapping after saving";
        response->success = false;
        return;
      }

      slam_status_ = "idle";
      response->success = true;
      return;
    }

    response->message = "must change mode to mapping first";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                       response->message);
    response->success = false;
  }
  /**
   * @brief 设置机器人工作模式
   *
   * 根据传入的模式字符串设置机器人的工作模式，并启动或停止相应的服务。
   *
   * @param status 模式字符串，可选值为 "mapping", "localization", "patrol", "remote_control", "idle"
   * @return 如果模式设置成功，则返回 true；否则返回 false
   */
  bool ModeSet(std::string status) {
    bool run_status = false;
    set_status_ = status;

    // 由顶层 launch 持有进程时，不能再通过 launch_manager 启停第二套
    // Lightning/Nav2。遥控/巡航只改变控制状态，不改变进程所有权。
    if (!manage_stack_) {
      if (set_status_ == "remote_control") {
        control_model_ = set_status_;
        return true;
      }
      if (set_status_ == "patrol" && slam_status_ == "localization") {
        if (!IsNavigationActive()) {
          RCLCPP_ERROR(get_logger(),
                       "Cannot enter patrol: the parent-owned Nav2 stack is not active");
          return false;
        }
        control_model_ = set_status_;
        return true;
      }
      if (set_status_ == slam_status_) {
        return true;
      }
      RCLCPP_ERROR(
          get_logger(),
          "Cannot switch process mode to '%s': SLAM/Nav2 are owned by robot.launch.py; "
          "restart the top-level launch with the requested mode",
          set_status_.c_str());
      return false;
    }

    if (set_status_ == "mapping") {
      if (!StopNavigation()) {
        return false;
      }
      if (slam_status_ == "localization") {
        if (!StopLaunch(localization_launch_file)) {
          return false;
        }
      } else if (slam_status_ == "mapping" && !StopLaunch(maping_launch_file)) {
        return false;
      }

      run_status = StartLaunch(maping_launch_file, mapping_args_);
      if (run_status == true) {
        slam_status_ = set_status_;
      }

    } else if (set_status_ == "localization") {
      if (slam_status_ == "mapping") {
        if (!StopLaunch(maping_launch_file)) {
          return false;
        }
      } else if (slam_status_ == "localization" &&
                 !StopLaunch(localization_launch_file)) {
        return false;
      }
      if (!StopNavigation()) {
        return false;
      }

      if (!SelectLocalizationMap(map_filename_)) {
        RCLCPP_ERROR(get_logger(),
                     "No complete Lightning map is selected; mapping must be saved first");
        return false;
      }
      run_status = StartLocalizationAndNavigation(map_filename_);
      if (run_status == true) {
        slam_status_ = set_status_;
      }
      
    } else if (set_status_ == "patrol") {
      
      if (slam_status_ == "localization") {
        if (!navigation_started_ && !StartNavigation(map_filename_)) {
          return false;
        }
        // 先等 autostart 完成；超时仍未激活才尝试 RESUME（兜底被暂停的情况）。
        if (!WaitNavigationActive(std::chrono::seconds(60)) &&
            !(ResumeNavigation() && IsNavigationActive())) {
          RCLCPP_ERROR(get_logger(), "Nav2 is not active (check localization TF map->base_link)");
          return false;
        }
        run_status = ChangeMap(map_filename_+"/map.yaml");
        if (run_status == true) {
          control_model_ = set_status_;
        }
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           "can not set to patrol model");
      }
    } else if (set_status_ == "remote_control") {
      control_model_ = set_status_;
      run_status = true;
    } else if (set_status_ == "idle") {
      if (!StopNavigation()) {
        return false;
      }
      if (slam_status_ == "mapping") {
        if (!StopLaunch(maping_launch_file)) {
          return false;
        }
        slam_status_ = "idle";  // 退出建图（未保存）后不再处于建图状态
      }
      control_model_ = set_status_;
      run_status = true;
    }
    return run_status;
  }
  /**
   * @brief 设置模式回调函数
   *
   * 根据传入的请求，设置机器人的运行状态，并更新响应信息。
   *
   * @param request 模式设置请求，包含要设置的状态
   * @param response 模式设置响应，包含操作结果信息
   */
  void ModeSetCallback(
      const std::shared_ptr<aid_robot_msgs::srv::StatusChange::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::StatusChange::Response> response) {
    bool run_status = false;
    std::string set_status = request->action;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                       "set model to" << set_status);
    run_status = ModeSet(set_status);
    if (run_status)
      response->message = "ok";
    else
      response->message = "err";
    return;
  }

 public:
  StatusManagerNode() : Node("robot_status_manager_node") {
    control_model_ = "idle";
    manage_stack_ = declare_parameter<bool>("manage_stack", true);
    const std::string startup_mode =
        declare_parameter<std::string>("startup_mode", "base");
    localization_args_ = declare_parameter<std::string>("localization_launch_args", "");
    navigation_args_ = declare_parameter<std::string>("navigation_launch_args", "");
    mapping_args_ = declare_parameter<std::string>("mapping_launch_args", "");
    default_map_dir_ = declare_parameter<std::string>("default_map_dir", "");
    // launch_ros 通过全局 __node 重映射主节点名称。若辅助节点也读取全局
    // 参数，它会被重映射成与主节点相同的名称，并触发重复 rosout publisher。
    // 辅助节点只用于同步 service client，必须保留独立且稳定的节点名。
    const auto client_node_options =
        rclcpp::NodeOptions().use_global_arguments(false);
    node_ = rclcpp::Node::make_shared(
        "robot_status_server_manager_node", client_node_options);
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    callback_group_executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
    nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    const std::string bringup_share =
        ament_index_cpp::get_package_share_directory("robot_bringup");
    const char *home = std::getenv("HOME");
    map_filepath_ = home != nullptr ? std::string(home) + "/maps"
                    : bringup_share + "/maps";
    maping_launch_file = bringup_share + "/launch/g1_mapping.launch.py";
    localization_launch_file = bringup_share + "/launch/g1_localization.launch.py";
    navigation_launch_file =
      ament_index_cpp::get_package_share_directory("g1_nav_bridge") +
      "/launch/g1_navigation.launch.py";
    status_change_server_ =
        nh_->create_service<aid_robot_msgs::srv::StatusChange>(
            "mode_set",
            std::bind(&StatusManagerNode::ModeSetCallback, this, _1, _2));
    ip_server_ = nh_->create_service<aid_robot_msgs::srv::GetString>(
        "get_ip_addresses",
        std::bind(&StatusManagerNode::GetIpHandleRequest, this, _1, _2));
    map_save_server_ = nh_->create_service<aid_robot_msgs::srv::MapOperation>(
        "aid_save_map",
        std::bind(&StatusManagerNode::SaveMapCallback, this, _1, _2));
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&StatusManagerNode::TimerCallback, this));

    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "aid_init_pose", 10,
        std::bind(&StatusManagerNode::InitPoseCallback, this,
                  std::placeholders::_1));

    status_pub_ =
        nh_->create_publisher<std_msgs::msg::String>("robot_status", 10);
    initial_pose_pub_ = nh_->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    lifecycle_manager_client_ =
      node_->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
          "/lifecycle_manager_navigation/manage_nodes");
    lifecycle_navigation_is_active_client_ = 
      node_->create_client<std_srvs::srv::Trigger>(
          "/lifecycle_manager_navigation/is_active");
    start_launch_client_ = node_->create_client<aid_robot_msgs::srv::ControlLaunch>(
      "start_launch",
      rmw_qos_profile_services_default, 
      callback_group_
    );

    stop_launch_client_ =
        node_->create_client<aid_robot_msgs::srv::ControlLaunch>("stop_launch");  
    lightning_save_map_client_ =
        node_->create_client<lightning::srv::SaveMap>("/lightning/save_map");
    get_current_map_client_ =
        node_->create_client<aid_robot_msgs::srv::GetCurrentMap>("get_current_map_id");
    change_map_client_ =
        node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    slam_status_ = "idle";

    if (!manage_stack_) {
      if (startup_mode == "mapping") {
        slam_status_ = "mapping";
      } else if (startup_mode == "localization" ||
                 startup_mode == "navigation") {
        slam_status_ = "localization";
      }
      RCLCPP_INFO(
          get_logger(),
          "SLAM/Nav2 process ownership belongs to robot.launch.py; "
          "automatic nested launch is disabled (startup_mode=%s)",
          startup_mode.c_str());
    } else if (startup_mode == "mapping") {
      // 本节点拥有进程：前端可随时经 mode_set 在建图/定位间切换。
      if (StartLaunch(maping_launch_file, mapping_args_)) {
        slam_status_ = "mapping";
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to start Lightning mapping");
      }
    } else if (startup_mode == "localization" || startup_mode == "navigation") {
      if (SelectLocalizationMap(map_filename_)) {
        RCLCPP_INFO(get_logger(), "Starting localization and navigation on %s",
                    map_filename_.c_str());
        if (StartLocalizationAndNavigation(map_filename_)) {
          slam_status_ = "localization";
        } else {
          RCLCPP_ERROR(
              get_logger(),
              "Failed to start Lightning localization and navigation: %s",
              map_filename_.c_str());
        }
      } else {
        RCLCPP_WARN(
            get_logger(),
            "No complete Lightning map is selected; starting idle for first mapping");
        map_filename_ = map_filepath_ + "/default";
      }
    } else {
      RCLCPP_INFO(get_logger(), "startup_mode=%s: SLAM/Nav2 stay idle until mode_set",
                  startup_mode.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "robot_status init success");

  }

  bool StartLaunch(std::string file, std::string param = "") {
  while (!start_launch_client_->wait_for_service(std::chrono::seconds(5))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_status_manager_node"), "Interruped while waiting for the server.");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("robot_status_manager_node"), "Server not available, waiting again...");
    }
    auto request =
        std::make_shared<aid_robot_msgs::srv::ControlLaunch::Request>();
    request->launch_file = file;
    request->parameter = param;
    auto future = start_launch_client_->async_send_request(request);
    auto result = callback_group_executor_->spin_until_future_complete(future);
    //处理返回结果
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager_node"), "Received response from start_launch service. result: "<< response->success
                << ", message: " << response->message);
      if(response->success){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager_node"), "start launch success!:  " << response->message);
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager_node"), "start launch failed!" << response->message);
        return false;
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("robot_status_manager_node"), "start launch time out!");
      return false;
    }
    return true;
  }
  bool StopLaunch(std::string file) {

    auto request =
        std::make_shared<aid_robot_msgs::srv::ControlLaunch::Request>();
    request->launch_file = file;

    if (!stop_launch_client_->wait_for_service(5s)) {
      RCLCPP_INFO(rclcpp::get_logger("stop_launch_client"),
                  "service stop_launch not available");
      return false;
    }

    auto result = stop_launch_client_->async_send_request(request);
    // launch_manager 要等被停的 launch 真正退出才回复（Nav2/Lightning 正常退出常需 5~13 s，
    // 其停止流程上限约 13 s）。原先 5 s 超时会让模式切换半途失败：本节点判定失败并中止，
    // 而进程其实仍在被停，状态从此与实际不一致，前端要反复点几次才能进入建图。
    if (rclcpp::spin_until_future_complete(node_, result,
                                           std::chrono::seconds(45)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("stop_launch_client"),
                         "message:" << result.get()->message);
      rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("stop_launch_client"),
                   "Failed to call service stop_launch");
      return false;
    }
    return true;
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<StatusManagerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
