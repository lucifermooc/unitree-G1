//
// 建图（SlamSystem）与定位（LocSystem）共用的 ROS 接口：传感器订阅、机器人 TF 发布
//

#ifndef LIGHTNING_ROS_IO_H
#define LIGHTNING_ROS_IO_H

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"

namespace lightning {

/// IMU / PointCloud2 / Livox CustomMsg 三路订阅
struct SensorSubscriptions {
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox = nullptr;
};

SensorSubscriptions SubscribeSensors(const rclcpp::Node::SharedPtr& node, const std::string& imu_topic,
                                     const std::string& cloud_topic, const std::string& livox_topic,
                                     std::function<void(const IMUPtr&)> on_imu,
                                     std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr&)> on_cloud,
                                     std::function<void(const livox_ros_driver2::msg::CustomMsg::SharedPtr&)> on_livox);

/**
 * 由算法输出的 T_map_lidar 发布机器人 TF。
 * 平面模式（URDF 以 body_link 为根）：map->base_link 只含位置与航向、贴地 z=0，躯干倾斜由 base_link->body_link 发布；
 * URDF 无 body_link 时退回 6DoF map->base_link。雷达安装外参只从 /tf_static 读一次，未就绪时不发 TF。
 */
class BaseTFPublisher {
   public:
    /// 声明帧名参数（base_frame / lidar_frame / body_frame / footprint_frame）并订阅 /tf_static
    explicit BaseTFPublisher(const rclcpp::Node::SharedPtr& node);

    /// header 提供时间戳与 map 帧名；z_offset_6dof 只在 6DoF 模式下加到 z 上
    void Publish(const SE3& map_to_lidar, const std_msgs::msg::Header& header, double z_offset_6dof = 0.0);

    const std::string& LidarFrame() const { return lidar_frame_; }

   private:
    void CacheStaticExtrinsic(const tf2_msgs::msg::TFMessage& msg);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
    // 只读 /tf_static，避免把动态 map/body 路径当成外参缓存
    std::shared_ptr<tf2_ros::Buffer> static_tf_buffer_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_sub_;

    std::mutex extrinsic_mutex_;
    bool extrinsic_ready_ = false;
    bool planar_base_ = false;
    SE3 lidar_to_parent_;  // T_lidar_parent，parent 为 body_link（平面模式）或 base_link

    std::string base_frame_ = "base_link";
    std::string lidar_frame_ = "mid360_link";
    std::string body_frame_ = "body_link";
    std::string footprint_frame_ = "base_footprint";
};

}  // namespace lightning

#endif  // LIGHTNING_ROS_IO_H
