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