//
// 建图（SlamSystem）与定位（LocSystem）共用的 ROS 接口
//

#include "core/system/ros_io.h"
#include "utils/timer.h"
#include "wrapper/ros_utils.h"

namespace lightning {

SensorSubscriptions SubscribeSensors(const rclcpp::Node::SharedPtr& node, const std::string& imu_topic,
                                     const std::string& cloud_topic, const std::string& livox_topic,
                                     std::function<void(const IMUPtr&)> on_imu,
                                     std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr&)> on_cloud,
                                     std::function<void(const livox_ros_driver2::msg::CustomMsg::SharedPtr&)> on_livox) {
    rclcpp::QoS qos(10);
    // IMU 200 Hz。本进程发布大消息（/map 约 1 MB）时 DDS 接收会被拖住 0.3~1 s，之后积压的
    // IMU 一次性灌入订阅队列；深度 100（0.5 s）会覆盖掉最旧的，快转时旋转预测缺失导致航向
    // 跳变、地图多层墙。1000 = 5 s 缓冲（约 350 KB）。内核 UDP 缓冲另见
    // robot_bringup/system/60-dds-buffers.conf。
    rclcpp::QoS imu_qos(1000);

    SensorSubscriptions subs;
    subs.imu = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, imu_qos, [on_imu](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            on_imu(imu);
        });

    subs.cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, qos, [on_cloud](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { on_cloud(cloud); }, "Proc Lidar", true);
        });

    subs.livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic, qos, [on_livox](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
            Timer::Evaluate([&]() { on_livox(cloud); }, "Proc Lidar", true);
        });
    return subs;
}

BaseTFPublisher::BaseTFPublisher(const rclcpp::Node::SharedPtr& node) : node_(node) {
    base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = node_->declare_parameter<std::string>("lidar_frame", "mid360_link");
    body_frame_ = node_->declare_parameter<std::string>("body_frame", "body_link");
    footprint_frame_ = node_->declare_parameter<std::string>("footprint_frame", "base_footprint");
    if (base_frame_ == lidar_frame_ || base_frame_.empty() || lidar_frame_.empty()) {
        throw std::runtime_error("base_frame and lidar_frame must be distinct, nonempty frame IDs");
    }
    static_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    static_tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { CacheStaticExtrinsic(*msg); });
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

void BaseTFPublisher::CacheStaticExtrinsic(const tf2_msgs::msg::TFMessage& msg) {
    std::lock_guard<std::mutex> lock(extrinsic_mutex_);
    if (extrinsic_ready_) return;  // 安装外参固定，每个进程读一次
    for (const auto& transform : msg.transforms) {
        static_tf_buffer_->setTransform(transform, "static_extrinsic", true);
    }
    try {
        // 不设超时，且只在收到静态 TF 时查询，不在定位回调里查。
        // 优先 body_link（平面 base_link 模式）；旧 URDF 没有 body_link 时退回 base_link。
        const bool has_body = !body_frame_.empty() && body_frame_ != base_frame_ &&
                              static_tf_buffer_->canTransform(body_frame_, lidar_frame_, tf2::TimePointZero);
        const std::string parent = has_body ? body_frame_ : base_frame_;
        const auto transform = static_tf_buffer_->lookupTransform(parent, lidar_frame_, tf2::TimePointZero);
        const auto& t = transform.transform.translation;
        const auto& r = transform.transform.rotation;
        Quatd q(r.w, r.x, r.y, r.z);
        const Vec3d translation(t.x, t.y, t.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !translation.allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid static LiDAR extrinsic; TF output disabled");
            return;
        }
        lidar_to_parent_ = SE3(q.normalized(), translation).inverse();
        planar_base_ = has_body;
        extrinsic_ready_ = true;
        if (planar_base_) {
            // base_footprint 与平面 base_link 重合（collision_monitor 使用）
            geometry_msgs::msg::TransformStamped fp;
            fp.header.stamp = node_->now();
            fp.header.frame_id = base_frame_;
            fp.child_frame_id = footprint_frame_;
            fp.transform.rotation.w = 1.0;
            static_tf_broadcaster_->sendTransform(fp);
        }
        RCLCPP_INFO(node_->get_logger(), "Cached static extrinsic %s <- %s; %s TF output enabled", parent.c_str(),
                    lidar_frame_.c_str(), planar_base_ ? "planar base_link + dynamic body tilt" : "6DoF base_link");
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Waiting for static base/LiDAR extrinsic: %s", error.what());
    }
}

void BaseTFPublisher::Publish(const SE3& map_to_lidar, const std_msgs::msg::Header& header, double z_offset_6dof) {
    SE3 extrinsic;
    {
        std::lock_guard<std::mutex> lock(extrinsic_mutex_);
        if (!extrinsic_ready_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Static extrinsic unavailable: NOT publishing map -> base_link");
            return;
        }
        extrinsic = lidar_to_parent_;
    }
    // planar_base_ 时 extrinsic 为 T_lidar_body，否则为 T_lidar_base。
    const SE3 map_to_parent = map_to_lidar * extrinsic;
    auto fill = [](geometry_msgs::msg::TransformStamped& msg, const SE3& T, double dz) {
        msg.transform.translation.x = T.translation().x();
        msg.transform.translation.y = T.translation().y();
        msg.transform.translation.z = T.translation().z() + dz;
        const auto r = T.unit_quaternion();
        msg.transform.rotation.x = r.x();
        msg.transform.rotation.y = r.y();
        msg.transform.rotation.z = r.z();
        msg.transform.rotation.w = r.w();
    };
    geometry_msgs::msg::TransformStamped output;
    output.header = header;
    output.child_frame_id = base_frame_;
    if (!planar_base_) {
        fill(output, map_to_parent, z_offset_6dof);
        tf_broadcaster_->sendTransform(output);
        return;
    }
    // 平面导航系：躯干 x 轴在地图水平面上的投影作为航向，位置取躯干原点（站立旋转中心的地面投影）。
    const Mat3d R = map_to_parent.rotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    const SE3 map_to_base(SO3::rotZ(yaw), map_to_parent.translation());
    const SE3 base_to_body(map_to_base.so3().inverse() * map_to_parent.so3(), Vec3d::Zero());
    // base_link 贴地：body_link 原点按 URDF 即机器人脚下地面，因此 map 系中 z 恒为 0。
    // 固定的 map_z_offset 依赖建图起点 IMU 高度，每张图不同（实测 1789713472324 偏 0.19m，
    // 整片地面被 costmap 当成障碍），不能用于导航高度基准。
    fill(output, map_to_base, -map_to_base.translation().z());
    auto tilt = output;
    tilt.header.frame_id = base_frame_;
    tilt.child_frame_id = body_frame_;
    fill(tilt, base_to_body, 0.0);
    tf_broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{output, tilt});
}

}  // namespace lightning
