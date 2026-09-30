//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "core/system/ros_io.h"

namespace lightning {

namespace loc {
class Localization;
struct LocalizationResult;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    void PublishBaseTF(const geometry_msgs::msg::TransformStamped& lidar_pose);

    /// 腿式里程计（g1_nav_bridge/sport_to_odom 的 /odom）：估计本体与本机的时钟差后换到雷达系送给守护
    void ProcessOdom(const nav_msgs::msg::Odometry::SharedPtr& msg);

    /// 人工重定位（网页 / RViz 的 /initialpose，map 系 base_link 平面位姿）
    void ProcessInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg);

    /// 每次激光定位后发布一致性守护状态（JSON）
    void PublishLocStatus(const loc::LocalizationResult& res);

    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<BaseTFPublisher> base_tf_ = nullptr;  // map->base_link / base_link->body_link
    double map_z_offset_ = 0.0;  // map 帧竖直偏移，使 z=0 落在地面（仅 6DoF 模式与 /base_link_pose 使用）
    std::string lidar_frame_ = "mid360_link";

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    SensorSubscriptions sensor_subs_;

    /// 诊断用：配准后的 map 系点云。默认不创建，由 pub_registered_scan 参数开启。
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_ = nullptr;

    /// 一致性守护 / 重定位
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loc_status_pub_ = nullptr;
    std::vector<double> odom_offset_samples_;  // 接收时刻 - 消息时间戳
    bool odom_offset_ready_ = false;
    double odom_offset_ = 0.0;                 // 加到 /odom 时间戳上换成本机时钟
    double last_odom_time_ = -1.0;             // 已送出的最后一条（本机时钟）
    std::atomic<double> last_odom_wall_{-1.0}; // 最近一次送出里程计的本机时间，状态里报告里程计是否在线
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
