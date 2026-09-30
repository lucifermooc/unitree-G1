//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "core/localization/localization.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_path, "./data/new_map/", "地图路径");
DEFINE_string(init_pose, "", "初始位姿 \"x y yaw_deg\"（地图系雷达位姿）；缺省按原逻辑从地图功能点初始化");
DEFINE_string(odom_topic, "", "腿式里程计话题（nav_msgs/Odometry，如 /odom），供一致性守护；空 = 不用");
DEFINE_double(odom_time_offset, 0.0, "加到里程计时间戳上的秒数（本体时钟与雷达时钟之差）");
DEFINE_string(odom_lidar_offset, "0 0 0", "雷达相对 base 的平面安装偏移 \"x y yaw_deg\"，把里程计位姿换到雷达系");
DEFINE_string(inject_ndt_bias, "", "仅测试：\"bx by t_start t_end\"，在该时间段（相对第一帧，秒）让 NDT 始终偏离预测 (bx, by)");
DEFINE_string(tf_out, "", "逐条记录定位输出（即在线模式交给 TF 发布的 T_map_lidar）：stamp x y z qx qy qz qw");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    loc::Localization::Options options;
    options.online_mode_ = false;

    loc::Localization loc(options);
    loc.Init(FLAGS_config, FLAGS_map_path);

    if (!FLAGS_init_pose.empty()) {
        double x = 0, y = 0, yaw_deg = 0;
        std::istringstream(FLAGS_init_pose) >> x >> y >> yaw_deg;
        loc.SetExternalPose(Quatd(Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Vec3d::UnitZ())), Vec3d(x, y, 0));
    }

    if (!FLAGS_inject_ndt_bias.empty()) {
        double bx = 0, by = 0, t0 = 0, t1 = 0;
        std::istringstream(FLAGS_inject_ndt_bias) >> bx >> by >> t0 >> t1;
        loc.SetDebugNdtBias(bx, by, t0, t1);
        LOG(WARNING) << "TEST ONLY: NDT bias (" << bx << ", " << by << ") injected during +" << t0 << " ~ +" << t1
                     << " s";
    }

    std::ofstream tf_out;
    if (!FLAGS_tf_out.empty()) {
        tf_out.open(FLAGS_tf_out);
        tf_out << std::fixed << std::setprecision(6);
        loc.SetTFCallback([&tf_out](const geometry_msgs::msg::TransformStamped& tf) {
            const auto& t = tf.transform.translation;
            const auto& q = tf.transform.rotation;
            tf_out << ToSec(tf.header.stamp) << " " << t.x << " " << t.y << " " << t.z << " " << q.x << " " << q.y
                   << " " << q.z << " " << q.w << "\n";
        });
    }

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");

    if (!FLAGS_odom_topic.empty()) {
        double ox = 0, oy = 0, oyaw_deg = 0;
        std::istringstream(FLAGS_odom_lidar_offset) >> ox >> oy >> oyaw_deg;
        const SE3 base_to_lidar(SO3::rotZ(oyaw_deg * M_PI / 180.0), Vec3d(ox, oy, 0));
        rosbag.AddRosOdomHandle(FLAGS_odom_topic, [&loc, base_to_lidar](nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto& p = msg->pose.pose.position;
            const auto& q = msg->pose.pose.orientation;
            const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            loc.ProcessLegOdom(ToSec(msg->header.stamp) + FLAGS_odom_time_offset,
                               SE3(SO3::rotZ(yaw), Vec3d(p.x, p.y, 0.0)) * base_to_lidar);
            return true;
        });
    }

    rosbag
        .AddImuHandle(imu_topic,
                      [&loc](IMUPtr imu) {
                          loc.ProcessIMUMsg(imu);
                          usleep(1000);
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  loc.ProcessLidarMsg(cloud);
                                  usleep(1000);
                                  return true;
                              })
        .AddLivoxCloudHandle("/livox/lidar",
                             [&loc](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
                                 loc.ProcessLivoxLidarMsg(cloud);
                                 usleep(1000);
                                 return true;
                             })
        .Go();

    Timer::PrintAll();
    loc.Finish();

    LOG(INFO) << "done";

    return 0;
}