//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

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

    // 一致性守护：腿式里程计输入 + 状态输出；人工重定位
    std::string odom_topic = "/odom";  // 老配置没有该键：订阅 /odom，收不到时守护只用 LIO
    yaml.GetOptional("loc_guard", "odom_topic", odom_topic);
    if (!odom_topic.empty()) {
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::SharedPtr msg) { ProcessOdom(msg); });
        LOG(INFO) << "loc guard leg odometry: " << odom_topic;
    }
    loc_status_pub_ = node_->create_publisher<std_msgs::msg::String>("/lightning/loc_status", rclcpp::QoS(10));
    loc_->SetLidarLocResultCallback([this](const loc::LocalizationResult &res) { PublishLocStatus(res); });
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { ProcessInitialPose(msg); });

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

void LocSystem::ProcessOdom(const nav_msgs::msg::Odometry::SharedPtr &msg) {
    if (!loc_started_ || base_tf_ == nullptr) return;
    const double now = node_->now().seconds();
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();

    // /odom 的时间戳是 G1 本体时钟，不一定与本机（雷达时间戳）对齐：先用前 50 条估计时钟差
    if (!odom_offset_ready_) {
        odom_offset_samples_.push_back(now - stamp);
        if (odom_offset_samples_.size() < 50) return;
        std::nth_element(odom_offset_samples_.begin(), odom_offset_samples_.begin() + 25,
                         odom_offset_samples_.end());
        odom_offset_ = odom_offset_samples_[25];
        odom_offset_samples_.clear();
        odom_offset_ready_ = true;
        if (std::fabs(odom_offset_) > 0.05) {
            LOG(WARNING) << "leg odometry clock offset " << odom_offset_ << " s (robot clock vs this host), compensated";
        } else {
            LOG(INFO) << "leg odometry clock offset " << odom_offset_ << " s";
        }
    }
    const double t = stamp + odom_offset_;
    if (std::fabs(now - t) > 1.0) {
        // 本体重启 / 时钟跳变：重新估计
        LOG(WARNING) << "leg odometry clock jumped (" << now - t << " s), re-estimating offset";
        odom_offset_ready_ = false;
        last_odom_time_ = -1.0;
        return;
    }
    if (t < last_odom_time_ + 0.02) return;  // 降到 <=50 Hz，守护只需要窗口两端

    SE3 base_to_lidar;
    if (!base_tf_->PlanarBaseToLidar(base_to_lidar)) return;
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw)) return;
    const SE3 odom_to_base(SO3::rotZ(yaw), Vec3d(p.x, p.y, 0.0));
    loc_->ProcessLegOdom(t, odom_to_base * base_to_lidar);
    last_odom_time_ = t;
    last_odom_wall_ = now;
}

void LocSystem::ProcessInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg) {
    if (!loc_started_) return;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
        LOG(WARNING) << "ignore /initialpose in frame " << msg->header.frame_id << " (expect map)";
        return;
    }
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw)) {
        LOG(WARNING) << "ignore invalid /initialpose";
        return;
    }
    SE3 map_to_lidar;
    if (base_tf_ == nullptr || !base_tf_->BasePoseToLidar(p.x, p.y, yaw, map_to_lidar)) {
        LOG(WARNING) << "ignore /initialpose: LiDAR extrinsic not ready";
        return;
    }
    LOG(WARNING) << "relocalize from /initialpose: base " << p.x << ", " << p.y << ", yaw " << yaw * 180.0 / M_PI
                 << " deg -> lidar " << map_to_lidar.translation().transpose();
    loc_->SetExternalPose(map_to_lidar.unit_quaternion(), map_to_lidar.translation(), true);
}

void LocSystem::PublishLocStatus(const loc::LocalizationResult &res) {
    if (loc_status_pub_ == nullptr) return;
    const auto &g = res.guard_status_;
    const double last_odom = last_odom_wall_.load();
    const bool odom_online = last_odom > 0 && node_->now().seconds() - last_odom < 1.0;
    char buf[640];
    std::snprintf(buf, sizeof(buf),
                  "{\"stamp\": %.3f, \"state\": \"%s\", \"frozen\": %s, \"drift_m\": %.3f, \"drift_deg\": %.2f, "
                  "\"odom_online\": %s, \"odom_agree\": %d, \"ndt_residual_m\": %.3f, "
                  "\"ndt_residual_deg\": %.2f, \"score\": %.3f, \"rollbacks\": %d, \"last_rollback_m\": %.3f, "
                  "\"need_reloc\": %s, \"loc_valid\": %s}",
                  res.timestamp_, loc::LocGuard::StateName(g.state_), g.frozen_ ? "true" : "false", g.drift_m_,
                  g.drift_deg_, odom_online ? "true" : "false", g.odom_agree_, g.residual_m_, g.residual_deg_,
                  res.confidence_, g.rollbacks_, g.last_rollback_m_, g.need_reloc_ ? "true" : "false",
                  res.lidar_loc_valid_ ? "true" : "false");
    std_msgs::msg::String msg;
    msg.data = buf;
    loc_status_pub_->publish(msg);
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
