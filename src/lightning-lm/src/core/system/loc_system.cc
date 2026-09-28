//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    sensor_subs_ = SubscribeSensors(
        node_, imu_topic_, cloud_topic_, livox_topic_, [this](const IMUPtr &imu) { ProcessIMU(imu); },
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) { ProcessLidar(cloud); },
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) { ProcessLidar(cloud); });

    if (options_.pub_tf_) {
        base_tf_ = std::make_shared<BaseTFPublisher>(node_);
        lidar_frame_ = base_tf_->LidarFrame();
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &pose) { PublishBaseTF(pose); });
    }

    // SLAM 建图以雷达起始位置为 map 原点，地面因此位于 z = g2p5.floor_height（约 -1.2 m），
    // 导致 base_link 落在 map 平面下方约 1.2 m。这里把 map 的 z=0 平移到地面，
    // 使 base_link 贴合地面、与 map_server 发布的 2D 栅格（origin.z 恒为 0）对齐。
    // 平面 base_link 模式下 TF 不再使用该偏移（base_link 恒贴地 z=0），仅用于 /base_link_pose 与旧 6DoF 模式。
    double floor_height = 0.0;
    yaml.GetOptional("g2p5", "floor_height", floor_height);
    map_z_offset_ = -floor_height;
    loc_->SetMapZOffset(map_z_offset_);
    LOG(INFO) << "map z offset (ground-referenced map frame): " << map_z_offset_;

    // 诊断话题：把配准后的点云（LIO 去畸变 + 定位位姿，与 UI 同源）发布到 map 系，
    // 便于在 RViz 里直接看 SLAM 用的是什么点云、配准结果落在哪。默认关闭。
    //
    // 开关放在 yaml 的 system.pub_registered_scan，而不是只靠 ROS 参数：
    // 本可执行文件用 gflags 解析 argv（run_loc_online.cc:20 ParseCommandLineFlags），
    // 遇到 --ros-args 这类未知 flag 会直接报错退出，所以 launch 侧无法用
    // "-p name:=value" 传参，只能经 --config 的 yaml 下发。
    // 仍保留 ROS 参数声明，便于 ros2 param get 查看当前值。
    bool pub_scan = false;  // 老配置文件没有该键，保持关闭
    yaml.GetOptional("system", "pub_registered_scan", pub_scan);
    pub_scan = node_->declare_parameter<bool>("pub_registered_scan", pub_scan);
    if (pub_scan) {
        scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lightning/registered_scan", rclcpp::QoS(2));
        loc_->SetPointcloudWorldCallback([this](const sensor_msgs::msg::PointCloud2 &cloud) {
            auto msg = cloud;
            msg.header.frame_id = lidar_frame_;  // 雷达系 + 扫描时刻，下游按该时刻查 TF
            scan_pub_->publish(msg);
        });
        LOG(INFO) << "publishing deskewed scan on /lightning/registered_scan (frame=" << lidar_frame_ << ")";
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::PublishBaseTF(const geometry_msgs::msg::TransformStamped &lidar_pose) {
    // The algorithm output is T_map_lidar, regardless of its legacy child-frame label.
    const auto &t = lidar_pose.transform.translation;
    const auto &r = lidar_pose.transform.rotation;
    const Quatd q(r.w, r.x, r.y, r.z);
    const Vec3d translation(t.x, t.y, t.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !translation.allFinite()) return;
    base_tf_->Publish(SE3(q.normalized(), translation), lidar_pose.header, map_z_offset_);
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
