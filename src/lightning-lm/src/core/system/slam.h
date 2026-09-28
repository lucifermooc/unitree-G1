//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "lightning/srv/save_map.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "core/system/ros_io.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测

namespace ui {
class PangolinWindow;
}

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否需要可视化UI
        bool with_2dvisualization_ = true;  // 是否需要2D可视化UI

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
    };

    using SaveMapService = srv::SaveMap;

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    /// 保存地图，默认保存至./data/地图名/ 下方
    void SaveMap(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    /// ros端保存地图的实现
    void SaveMap(const SaveMapService::Request::SharedPtr request, SaveMapService::Response::SharedPtr response);

    Options options_;
    std::atomic_bool running_ = false;

    rclcpp::Service<SaveMapService>::SharedPtr savemap_service_ = nullptr;

    std::string map_name_;  // 地图名
    std::string map_save_root_;
    bool gravity_aligned_map_ = false;  // LIO 世界系已对齐重力（fasterlio.gravity_align_init）

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // ui
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;

    /// 把 LIO 当前帧转到世界系后发布为 PointCloud2（诊断用，不影响建图结果）
    void PublishRegisteredScan();

    /// 建图时发布 map->base_link（平面）与 base_link->body_link（躯干倾斜），与定位模式（LocSystem）一致，
    /// 前端 /base_link_pose 由 robot_pose_pub 查该 TF 得到。位姿 = 最新关键帧的回环修正 × LIO 当前位姿。
    void PublishBaseTF();

    /// LIO 处理完一帧后的共同流程：诊断输出、TF、新关键帧送回环/栅格/UI
    void AfterLidarProcessed();

    std::shared_ptr<BaseTFPublisher> base_tf_ = nullptr;
    double last_tf_stamp_ = -1;

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    /// 诊断用：LIO 去畸变并转到世界系的当前帧点云。只读取算法结果，不参与任何计算。
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
    bool pub_registered_scan_ = false;
    double registered_scan_leaf_ = 0.0;
    std::string registered_scan_frame_ = "map";
    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    SensorSubscriptions sensor_subs_;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
