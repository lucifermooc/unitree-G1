#include <algorithm>
#include <execution>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/registration/ndt.h>

#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"

#include "core/localization/lidar_loc/lidar_loc.h"

#include <opencv2/highgui.hpp>

#include "glog/logging.h"
#include "io/file_io.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"

namespace lightning::loc {

LidarLoc::LidarLoc(LidarLoc::Options options) : options_(options) {
    pcl_ndt_.reset(new NDTType());
    pcl_ndt_->setResolution(1.0);
    pcl_ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    pcl_ndt_->setOulierRatio(0.45);
    pcl_ndt_->setStepSize(0.1);
    pcl_ndt_->setTransformationEpsilon(0.01);
    pcl_ndt_->setMaximumIterations(20);
    pcl_ndt_->setNumThreads(4);

    pcl_ndt_rough_.reset(new NDTType());
    pcl_ndt_rough_->setResolution(5.0);
    pcl_ndt_rough_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    pcl_ndt_rough_->setStepSize(0.1);
    pcl_ndt_rough_->setMaximumIterations(4);
    pcl_ndt_rough_->setNumThreads(4);

    pcl_icp_.reset(new ICPType());
    pcl_icp_->setMaximumIterations(4);
    pcl_icp_->setTransformationEpsilon(0.01);

    LOG(INFO) << "match name is NDT_OMP"
              << ", MaximumIterations is: " << pcl_ndt_->getMaximumIterations();
}

LidarLoc::~LidarLoc() {
    if (update_map_thread_.joinable()) {
        update_map_quit_ = true;
        update_map_thread_.join();
    }

    recover_pose_out_.close();
}

bool LidarLoc::Init(const std::string& config_path) {
    YAML_IO yaml(config_path);
    options_.map_option_.enable_dynamic_polygon_ = yaml.GetValue<bool>("maps", "with_dyn_area");
    options_.map_option_.max_pts_in_dyn_chunk_ = yaml.GetValue<int>("maps", "max_pts_dyn_chunk");
    options_.map_option_.load_map_size_ = yaml.GetValue<int>("maps", "load_map_size");
    options_.map_option_.unload_map_size_ = yaml.GetValue<int>("maps", "unload_map_size");

    options_.update_kf_dis_ = yaml.GetValue<double>("lidar_loc", "update_kf_dis");
    options_.update_lidar_loc_score_ = yaml.GetValue<double>("lidar_loc", "update_lidar_loc_score");
    options_.min_init_confidence_ = yaml.GetValue<float>("lidar_loc", "min_init_confidence");

    // options_.filter_z_min_ = yaml.GetValue<double>("lidar_loc", "filter_z_min");
    // options_.filter_z_max_ = yaml.GetValue<double>("lidar_loc", "filter_z_max");
    // options_.filter_intensity_min_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_min");
    // options_.filter_intensity_max_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_max");
    options_.lidar_loc_odom_th_ = yaml.GetValue<double>("lidar_loc", "lidar_loc_odom_th");

    options_.init_with_fp_ = yaml.GetValue<bool>("lidar_loc", "init_with_fp");
    options_.enable_parking_static_ = yaml.GetValue<bool>("lidar_loc", "enable_parking_static");
    options_.enable_icp_adjust_ = yaml.GetValue<bool>("lidar_loc", "enable_icp_adjust");
    options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
    options_.try_self_extrap_ = yaml.GetValue<bool>("lidar_loc", "try_self_extrap");
    options_.gravity_constrain_ = false;  // 老配置无该键，保持原行为
    yaml.GetOptional("lidar_loc", "gravity_constrain", options_.gravity_constrain_);
    LOG(INFO) << "gravity constrain: " << options_.gravity_constrain_;
    // 地图坐标系中的重力"上"方向，来自 <map>/gravity_up.txt（三个数，地图系）。
    // 旧地图建图时世界系=首帧 IMU 系，会随建图起点姿态倾斜，方向需实测后写入；
    // 开启 fasterlio.gravity_align_init 建出的地图保存时自动写 "0 0 1"。
    // 没有该文件时无法确定地图的重力方向，不做重力约束（保持原行为）。
    {
        std::ifstream fin(options_.map_option_.map_path_ + "/gravity_up.txt");
        Vec3d up;
        if (fin >> up[0] >> up[1] >> up[2] && up.norm() > 0.5) {
            map_up_ = up.normalized();
            map_up_known_ = true;
            LOG(INFO) << "map gravity up: " << map_up_.transpose() << " (tilt "
                      << std::acos(std::min(1.0, map_up_.z())) * 180.0 / M_PI << " deg)";
        } else if (options_.gravity_constrain_) {
            options_.gravity_constrain_ = false;
            LOG(WARNING) << "map has no gravity_up.txt, gravity constrain disabled for this map";
        }
    }

    // 融合比例 / 快速收敛参数：可选键，缺省保持默认值（老配置行为不变，除快速收敛默认开启外）
    yaml.GetOptional("lidar_loc", "balance_factor", options_.balance_factor_);
    yaml.GetOptional("lidar_loc", "fast_converge_factor", options_.fast_converge_factor_);
    yaml.GetOptional("lidar_loc", "fast_converge_min_score", options_.fast_converge_min_score_);
    yaml.GetOptional("lidar_loc", "fast_converge_ang_deg", options_.fast_converge_ang_deg_);
    yaml.GetOptional("lidar_loc", "fast_converge_pos", options_.fast_converge_pos_);
    yaml.GetOptional("lidar_loc", "fast_converge_ok_frames", options_.fast_converge_ok_frames_);
    yaml.GetOptional("lidar_loc", "fast_converge_max_frames", options_.fast_converge_max_frames_);
    yaml.GetOptional("lidar_loc", "track_min_score", options_.track_min_score_);
    yaml.GetOptional("lidar_loc", "log_ndt_hessian", options_.log_ndt_hessian_);
    yaml.GetOptional("lidar_loc", "ndt_dump_dir", options_.ndt_dump_dir_);
    options_.balance_factor_ = std::clamp(options_.balance_factor_, 0.0, 1.0);
    options_.fast_converge_factor_ = std::clamp(options_.fast_converge_factor_, 0.0, 1.0);
    LOG(INFO) << "lidar loc balance factor: " << options_.balance_factor_
              << ", fast converge factor: " << options_.fast_converge_factor_
              << " (score >= " << options_.fast_converge_min_score_ << ", until " << options_.fast_converge_ok_frames_
              << "x residual < " << options_.fast_converge_ang_deg_ << " deg / " << options_.fast_converge_pos_
              << " m, max " << options_.fast_converge_max_frames_ << " frames), track min score "
              << options_.track_min_score_;

    // 一致性守护：独立的 loc_guard 段（老配置没有该段时按默认值开启）
    {
        auto& g = options_.guard_;
        yaml.GetOptional("loc_guard", "enabled", g.enabled_);
        std::string action = g.freeze_ ? "freeze" : "warn";
        yaml.GetOptional("loc_guard", "action", action);
        g.freeze_ = action != "warn";
        yaml.GetOptional("loc_guard", "rollback", g.rollback_);
        yaml.GetOptional("loc_guard", "window_sec", g.window_sec_);
        yaml.GetOptional("loc_guard", "window_dist", g.window_dist_);
        yaml.GetOptional("loc_guard", "max_drift_m", g.max_drift_m_);
        yaml.GetOptional("loc_guard", "max_drift_deg", g.max_drift_deg_);
        yaml.GetOptional("loc_guard", "static_travel_m", g.static_travel_m_);
        yaml.GetOptional("loc_guard", "max_drift_static_m", g.max_drift_static_m_);
        yaml.GetOptional("loc_guard", "recover_frames", g.recover_frames_);
        yaml.GetOptional("loc_guard", "recover_res_m", g.recover_res_m_);
        yaml.GetOptional("loc_guard", "recover_res_deg", g.recover_res_deg_);
        yaml.GetOptional("loc_guard", "reloc_hint_frames", g.reloc_hint_frames_);
        yaml.GetOptional("loc_guard", "odom_tol_m", g.odom_tol_m_);
        yaml.GetOptional("loc_guard", "odom_tol_ratio", g.odom_tol_ratio_);
        yaml.GetOptional("loc_guard", "odom_tol_deg", g.odom_tol_deg_);
        yaml.GetOptional("loc_guard", "odom_persist_frames", g.odom_persist_frames_);
        yaml.GetOptional("loc_guard", "max_ndt_jump_m", options_.max_ndt_jump_m_);
        yaml.GetOptional("loc_guard", "max_ndt_jump_deg", options_.max_ndt_jump_deg_);
        yaml.GetOptional("loc_guard", "init_yaw_search_deg", options_.init_yaw_search_deg_);
        yaml.GetOptional("loc_guard", "init_yaw_search_step_deg", options_.init_yaw_search_step_deg_);
        guard_ = LocGuard(g);
        LOG(INFO) << "loc guard: " << (g.enabled_ ? (g.freeze_ ? "freeze" : "warn only") : "disabled")
                  << (g.freeze_ && g.rollback_ ? " + rollback" : "") << ", window " << g.window_sec_ << " s / "
                  << g.window_dist_ << " m, max drift " << g.max_drift_m_ << " m (static " << g.max_drift_static_m_
                  << " m when path < " << g.static_travel_m_ << " m) / " << g.max_drift_deg_ << " deg, odom tol "
                  << g.odom_tol_m_ << " m + " << g.odom_tol_ratio_ << " x travel / " << g.odom_tol_deg_ << " deg x "
                  << g.odom_persist_frames_ << " frames; ndt jump gate " << options_.max_ndt_jump_m_ << " m / "
                  << options_.max_ndt_jump_deg_ << " deg; init yaw search +-" << options_.init_yaw_search_deg_ << " deg";
    }

    lidar_loc::grid_search_angle_step = yaml.GetValue<double>("lidar_loc", "grid_search_angle_step");
    lidar_loc::grid_search_angle_range = yaml.GetValue<double>("lidar_loc", "grid_search_angle_range");

    LOG(INFO) << "min init confidence: " << options_.min_init_confidence_;

    std::string map_policy = yaml.GetValue<std::string>("maps", "dyn_cloud_policy");
    if (map_policy == "short") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::SHORT;
    } else if (map_policy == "long") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::LONG;
    } else if (map_policy == "persistent") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::PERSISTENT;
    }

    options_.map_option_.delete_when_unload_ = yaml.GetValue<bool>("maps", "delete_when_unload");
    options_.map_option_.load_dyn_cloud_ = yaml.GetValue<bool>("maps", "load_dyn_cloud");
    options_.map_option_.save_dyn_when_quit_ = yaml.GetValue<bool>("maps", "save_dyn_when_quit");
    options_.map_option_.save_dyn_when_unload_ = yaml.GetValue<bool>("maps", "save_dyn_when_unload");

    map_ = std::make_shared<TiledMap>(options_.map_option_);
    map_->LoadMapIndex();

    auto fps = map_->GetAllFP();
    if (!fps.empty()) {
        map_->LoadOnPose(fps.front().pose_);
        /// 更新一次地图，保证有初始数据
        UpdateGlobalMap();
    }

    /// load recover pose if exist
    if (PathExists(options_.recover_pose_path_)) {
        std::ifstream fin(options_.recover_pose_path_);
        double data[7] = {0, 0, 0, 0, 0, 0, 1};
        for (int i = 0; i < 7; ++i) {
            fin >> data[i];
        }

        SE3 pose(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2]));
        FunctionalPoint fp_recover;
        fp_recover.name_ = "recover";
        fp_recover.pose_ = pose;
        map_->AddFP(fp_recover);
    }

    update_map_thread_ = std::thread([this]() { LidarLoc::UpdateMapThread(); });

    return true;
}

bool LidarLoc::ProcessCloud(CloudPtr cloud_input) {
    assert(cloud_input != nullptr);

    if (cloud_input->empty() || cloud_input->size() < 50) {
        LOG(WARNING) << "loc input is empty or invalid, sz: " << cloud_input->size();
        return false;
    }

    // CloudPtr cloud(new PointCloudType);
    // pcl::VoxelGrid<PointType> voxel;

    // float sz = 0.1;
    // voxel.setLeafSize(sz, sz, sz);
    // voxel.setInputCloud(cloud_input);
    // voxel.filter(*cloud);

    current_scan_ = cloud_input;

    Align(cloud_input);
    return true;
}

NavState LidarLoc::GetState() {
    UL lock_res(result_mutex_);
    NavState ns;
    ns.SetPose(current_abs_pose_);
    ns.timestamp_ = current_timestamp_;
    ns.confidence_ = current_score_;

    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        auto s = lo_pose_queue_.back();
        ns.SetVel(s.GetVel());
    }

    return ns;
}

bool LidarLoc::ProcessDR(const NavState& state) {
    // 未初始化成功的数据不接收
    if (!state.pose_is_ok_) {
        return false;
    }

    // DR数据check
    UL lock(dr_pose_mutex_);
    if (!dr_pose_queue_.empty()) {
        const double last_stamp = dr_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            return false;
        }
    }

    dr_pose_queue_.emplace_back(state);
    while (dr_pose_queue_.size() >= 1000) {
        dr_pose_queue_.pop_front();
    }

    return true;
}

void LidarLoc::ProcessLegOdom(double timestamp, const SE3& pose) {
    UL lock(leg_odom_mutex_);
    if (!leg_odom_queue_.empty() && timestamp <= leg_odom_queue_.back().timestamp_) {
        return;
    }
    leg_odom_queue_.emplace_back(timestamp, pose);
    while (leg_odom_queue_.size() > 2000) {  // /odom 20~500 Hz，只需覆盖一次激光定位的延迟
        leg_odom_queue_.pop_front();
    }
}

bool LidarLoc::LegOdomAt(double timestamp, SE3& pose) {
    UL lock(leg_odom_mutex_);
    TimedPose match;
    return math::PoseInterp<TimedPose>(
        timestamp, leg_odom_queue_, [](const TimedPose& p) { return p.timestamp_; },
        [](const TimedPose& p) { return p.pose_; }, pose, match, 0.2);
}

bool LidarLoc::ProcessLO(const NavState& state) {
    /// 理论上相对定位是按时间顺序到达的
    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        const double last_stamp = lo_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            LOG(WARNING) << "当前相对定位的结果的时间戳应当比上一个时间戳数值大，实际相减得"
                         << state.timestamp_ - last_stamp;
            return false;
        }
    }

    lo_pose_queue_.emplace_back(state);

    while (lo_pose_queue_.size() >= 50) {
        lo_pose_queue_.pop_front();
    }

    if (state.lidar_odom_reliable_ == false) {
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
    } else {
        if (state.lidar_odom_reliable_ && lo_reliable_cnt_ > 0) {
            lo_reliable_cnt_--;
        }

        if (lo_reliable_cnt_ == 0) {
            lo_reliable_ = true;
        }
    }

    return true;
}

bool LidarLoc::YawSearch(SE3& pose, double& confidence, CloudPtr input, CloudPtr output, double range_deg,
                         double step_deg) {
    SE3 init_pose = pose;
    auto RPYXYZ = math::SE3ToRollPitchYaw(init_pose);
    double init_yaw = RPYXYZ.yaw;

    confidence = 0;
    bool yaw_search_success = false;

    // 含两端点、以给定航向为中心对称（上游写法是 [-range, range) 且不含中心）
    const int half = std::max(0, static_cast<int>(std::round(range_deg / std::max(step_deg, 0.1))));
    const int step = 2 * half + 1;

    std::vector<double> searched_yaw;
    std::vector<double> scores(step);
    std::vector<int> index;
    std::vector<SE3> pose_opti(step);

    for (int i = 0; i < step; ++i) {
        double search_yaw = init_yaw + (i - half) * step_deg * constant::kDEG2RAD;
        searched_yaw.emplace_back(search_yaw);
        index.emplace_back(i);
    }

    LOG(INFO) << "init yaw: " << init_yaw << ", p: " << RPYXYZ.pitch << ", ro: " << RPYXYZ.roll << ", search from "
              << searched_yaw.front() << " to " << searched_yaw.back();

    /// 粗分辨率
    std::for_each(index.begin(), index.end(), [&](int i) {
        double fitness_score = 0;
        RPYXYZ.yaw = searched_yaw[i];
        SE3 pose_esti = math::XYZRPYToSE3(RPYXYZ);

        Localize(pose_esti, fitness_score, input, output, true);

        scores[i] = fitness_score;
        pose_opti[i] = pose_esti;
    });

    /// 高分辨率：粗分辨率（5 m 体素）分值与精匹配不在一个量级，不能拿它和 min_init_confidence 比；
    /// 对粗分最高的前 3 个候选都做精匹配，按精匹配分值取最好的
    std::sort(index.begin(), index.end(), [&](int a, int b) { return scores[a] > scores[b]; });
    confidence = 0;
    for (size_t k = 0; k < std::min<size_t>(3, index.size()); ++k) {
        SE3 candidate = pose_opti[index[k]];
        double fine_score = 0;
        Localize(candidate, fine_score, input, output, false);
        LOG(INFO) << "yaw search candidate " << searched_yaw[index[k]] * constant::kRAD2DEG << " deg: rough "
                  << scores[index[k]] << ", fine " << fine_score;
        if (fine_score > confidence) {
            confidence = fine_score;
            pose = candidate;
        }
    }

    if (confidence > options_.min_init_confidence_) {
        LOG(INFO) << "init success, score: " << confidence << ", th=" << options_.min_init_confidence_;
        Eigen::Vector3d suc_translation = pose.translation();
        Eigen::Matrix3d suc_rotation_matrix = pose.rotationMatrix();
        double suc_x = suc_translation.x();
        double suc_y = suc_translation.y();
        double suc_yaw = atan2(suc_rotation_matrix(1, 0), suc_rotation_matrix(0, 0));
        LOG(INFO) << "localization init success, pose: " << suc_x << ", " << suc_y << ", " << suc_yaw
                  << ", conf: " << confidence;
        yaw_search_success = true;
    }

    return yaw_search_success;
}

bool LidarLoc::InitWithFP(CloudPtr input, const SE3& fp_pose, bool yaw_search) {
    assert(input != nullptr && !input->empty());

    // 使用功能点的位置进行定位初始化
    double fitness_score;
    SE3 pose_esti = fp_pose;
    CloudPtr output_cloud(new PointCloudType);
    // 上游全范围 YawSearch 会选错角度（issue #123）；只在人工给了大致位姿时做窄范围搜索
    if (yaw_search && options_.init_yaw_search_deg_ > 0) {
        loc_inited_ = YawSearch(pose_esti, fitness_score, input, output_cloud, options_.init_yaw_search_deg_,
                                options_.init_yaw_search_step_deg_);
    } else {
        loc_inited_ = Localize(pose_esti, fitness_score, input, output_cloud);
    }
    if (loc_inited_ && options_.gravity_constrain_) {
        pose_esti = ApplyGravityConstraint(pose_esti, "init");
    }

    if (loc_inited_) {
        current_timestamp_ = math::ToSec(input->header.stamp);
        localization_result_.confidence_ = fitness_score;
        current_abs_pose_ = pose_esti;
        localization_result_.pose_ = pose_esti;
        localization_result_.timestamp_ = current_timestamp_;
        localization_result_.lidar_loc_valid_ = true;
        localization_result_.status_ = LocalizationStatus::GOOD;

        last_abs_pose_set_ = true;
        last_abs_pose_ = pose_esti;

        current_score_ = fitness_score;
        LOG(INFO) << "fitness_score is: " << fitness_score << ", global_pose is: " << fp_pose.translation().transpose();
        LOG(INFO) << " [Loc init pose]: " << last_abs_pose_.translation().transpose();
        map_height_ = fp_pose.translation()[2];

        if (current_lo_pose_set_) {
            // 设置上一次的相对定位结果
            last_lo_pose_ = current_lo_pose_;
            last_lo_pose_set_ = true;

            last_dr_pose_ = current_dr_pose_;
            last_dr_pose_set_ = true;
        }

        //  定位成功，则清空失败记录
        fp_init_fail_pose_vec_.clear();

        // 重新初始化后守护从头开始
        guard_.Reset();

        // 进入快速收敛阶段
        fast_converge_frames_ = 0;
        fast_converge_ok_cnt_ = 0;
        converged_ = options_.fast_converge_factor_ <= options_.balance_factor_;
    } else {
        // 添加失败历史记录
        LOG(INFO) << "init failed, score: " << fitness_score;
        fp_init_fail_pose_vec_.emplace_back(fp_pose);
        fp_last_tried_time_ = 1e-6 * static_cast<double>(input->header.stamp);
    }
    return loc_inited_;
}

void LidarLoc::ResetLastPose(const SE3& last_pose) {
    last_abs_pose_ = last_pose;

    // TODO：清空动态图层

    return;
}

bool LidarLoc::TryOtherSolution(CloudPtr input, SE3& pose) {
    double fitness_score;
    SE3 pose_esti = pose;
    CloudPtr output_cloud(new PointCloudType);

    bool loc_success = Localize(pose_esti, fitness_score, input, output_cloud);

    if (loc_success) {
        // 激光重置逻辑
        float score_th = std::min(1.5 * current_score_, current_score_ + 0.3);
        if (fitness_score > score_th && fitness_score > 1.0) {
            // 显著好于现在的估计
            LOG(WARNING) << "rtk solution is significantly better: " << fitness_score << " " << current_score_;
            pose = pose_esti;
            localization_result_.lidar_loc_smooth_flag_ = false;
            return true;
        } else {
            LOG(INFO) << "not using rtk solution: " << fitness_score << " " << current_score_;
            return false;
        }
    }
    return false;
}

bool LidarLoc::UpdateGlobalMap() {
    NDTType::Ptr ndt(new NDTType());
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setStepSize(0.1);
    ndt->setMaximumIterations(4);
    ndt->setNumThreads(4);
    // 与构造时一致（原先替换对象时漏设，默认 0.1）：线搜索步长下限 = epsilon/2，0.1 时每步至少走 0.05、退不回小步长
    ndt->setTransformationEpsilon(0.01);

    map_->SetNewTargetForNDT(ndt);
    ndt->initCompute();

    UL lock(match_mutex_);
    pcl_ndt_ = ndt;

    if (!loc_inited_) {
        NDTType::Ptr ndt_rough(new NDTType());
        ndt_rough->setResolution(5.0);
        ndt_rough->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_rough->setStepSize(0.1);
        ndt_rough->setMaximumIterations(4);
        ndt_rough->setNumThreads(4);

        map_->SetNewTargetForNDT(ndt_rough);
        // ndt_rough->initCompute();

        pcl_ndt_rough_ = ndt_rough;
    }

    if (options_.enable_icp_adjust_) {
        ICPType::Ptr icp(new ICPType());
        CloudPtr map_cloud(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel;
        auto sz = 0.5;
        voxel.setLeafSize(sz, sz, sz);
        voxel.setInputCloud(map_->GetAllMap());
        voxel.filter(*map_cloud);
        icp->setInputTarget(map_cloud);
        icp->setMaximumIterations(4);
        icp->setTransformationEpsilon(0.01);
        pcl_icp_ = icp;
    }

    return true;
}

void LidarLoc::UpdateMapThread() {
    LOG(INFO) << "UpdateMapThread thread is running";
    while (!update_map_quit_) {
        if (map_->MapUpdated() || map_->DynamicMapUpdated()) {
            UpdateGlobalMap();

            if (ui_) {
                ui_->UpdatePointCloudGlobal(map_->GetStaticCloud());
                ui_->UpdatePointCloudDynamic(map_->GetDynamicCloud());
            }

            map_->CleanMapUpdate();
        }
        usleep(10000);
    }
}

void LidarLoc::SetInitialPose(SE3 init_pose, bool yaw_search) {
    UL lock(initial_pose_mutex_);
    loc_inited_ = false;
    // map_->ClearMap();

    initial_pose_set_ = true;
    initial_pose_ = init_pose;
    initial_yaw_search_ = yaw_search;
    LOG(INFO) << "Set initial pose is: " << initial_pose_.translation().transpose()
              << (yaw_search ? " (with yaw search)" : "");
}

void LidarLoc::Align(const CloudPtr& input) {
    // 输入必须非空
    assert(input != nullptr);

    // 点云去畸变定到了结束时间，所以该点云的定位也是到结束时间的
    double current_time = math::ToSec(input->header.stamp) + lo::lidar_time_interval;
    current_timestamp_ = current_time;
    if (first_align_time_ < 0) {
        first_align_time_ = current_time;
    }

    LOG(INFO) << "current time: " << std::fixed << std::setprecision(12) << current_timestamp_;

    /// 设置当前帧对应的rel_pose
    if (!AssignLOPose(current_time)) {
        LOG(WARNING) << "assign LO pose failed";
    }

    if (!AssignDRPose(current_time)) {
        LOG(WARNING) << "assign DR pose failed";
    }

    /// 1. 车辆静止处理
    if (parking_ && loc_inited_) {
        LOG(INFO) << "车辆静止，不做匹配";

        UpdateState(input);
        current_abs_pose_ = last_abs_pose_;
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);

        UL lock(result_mutex_);
        localization_result_.timestamp_ = current_time;
        localization_result_.pose_ = current_abs_pose_;

        if (options_.enable_parking_static_) {
            localization_result_.is_parking_ = true;
            localization_result_.valid_ = true;
        }
        return;
    }

    /// 2. 初始化处理
    if (!loc_inited_) {
        UL lock_init(initial_pose_mutex_);
        LOG(INFO) << "initing lidarloc";
        SetInitRltState();

        if (initial_pose_set_) {
            /// 尝试在给定点初始化；重定位时给定点可能不在当前已加载的地图块里
            map_->LoadOnPose(initial_pose_);
            if (InitWithFP(input, initial_pose_, initial_yaw_search_)) {
                LOG(INFO) << "init with external pose: " << initial_pose_.translation().transpose();
                initial_pose_set_ = false;
                external_reinit_done_ = true;
                return;
            }
            if (initial_yaw_search_) {
                // 人工重定位：不要退回功能点初始化（会落到地图原点），下一帧继续在给定位姿附近试
                LOG(WARNING) << "relocalization at given pose not confident yet, retry next scan";
                return;
            }
        }

        if (options_.init_with_fp_) {
            /// 从功能点初始化
            /// 如果之前尝试过，那么需要间隔一段时间再进行搜索
            if (!fp_init_fail_pose_vec_.empty() && current_dr_pose_set_) {
                SE3 last_tried_pose = fp_init_fail_pose_vec_.back();
                bool should_try =
                    (current_time - fp_last_tried_time_) > 2.0 ||
                    (current_dr_pose_.translation() - last_tried_pose.translation()).norm() > 0.3 ||
                    (current_dr_pose_.so3().inverse() * last_tried_pose.so3()).log().norm() > 10 * M_PI / 180.0;
                if (!should_try) {
                    LOG(INFO) << "skip trying init, please move to another place.";
                    return;
                }
            } else {
                LOG(INFO) << "fp tried pose: " << fp_init_fail_pose_vec_.size()
                          << ", dr pose set: " << current_dr_pose_set_;
            }

            auto all_fps = map_->GetAllFP();
            bool fp_init_success = false;
            for (const auto& fp : all_fps) {
                map_->LoadOnPose(fp.pose_);
                if (InitWithFP(input, fp.pose_)) {
                    LOG(INFO) << "init with fp: " << fp.name_;
                    fp_init_success = true;
                    break;
                }
            }

            if (!fp_init_success) {
                LOG(INFO) << "FP init failed.";
                if (current_dr_pose_set_) {
                    LOG(INFO) << "record fp failed time: " << std::setprecision(12) << current_time
                              << ", pose: " << current_dr_pose_.translation().transpose();
                    fp_last_tried_time_ = current_time;
                    fp_init_fail_pose_vec_.emplace_back(current_dr_pose_);
                }
            } else {
                fp_last_tried_time_ = 0;
                fp_init_fail_pose_vec_.clear();
            }
        }

        /// 初始化未成功时，不往下走流程
        return;
    }

    /// 4. 设置当前帧对应的 pose guess
    /// NOTE: LO设置预测的位置和LidarLoc自身递推设置预测的方法并不完全一致，自身外推容易受噪声影响

    SE3 guess_from_lo = last_abs_pose_;
    if (last_lo_pose_set_ && current_lo_pose_set_) {
        // 如果有里程计，则用两个时刻的相对定位来递推，估计一个当前pose的初值
        const SE3 delta = last_lo_pose_.inverse() * current_lo_pose_;
        guess_from_lo = last_abs_pose_ * delta;

        LOG(INFO) << "current lo pose: " << current_lo_pose_.translation().transpose();
        LOG(INFO) << "last lo pose: " << last_lo_pose_.translation().transpose();
        LOG(INFO) << "lo motion: " << delta.translation().transpose();
        LOG(INFO) << "last abs pose: " << last_abs_pose_.translation().transpose();
        // guess_from_lo.translation()[2] = 0;
        LOG(INFO) << "loc using lo guess: " << guess_from_lo.translation().transpose();
    }

    SE3 guess_from_self = guess_from_lo;
    if (lidar_loc_pose_queue_.size() >= 2) {
        SE3 pred;
        TimedPose match;
        if (math::PoseInterp<TimedPose>(
                current_time, lidar_loc_pose_queue_, [](const TimedPose& p) { return p.timestamp_; },
                [](const TimedPose& p) { return p.pose_; }, pred, match, 2.0)) {
            guess_from_self = pred;
        }
    }

    // SE3 guess_from_dr = guess_from_lo;
    // if (last_dr_pose_set_ && current_dr_pose_set_) {
    //     const SE3 delta = last_dr_pose_.inverse() * current_dr_pose_;
    //     guess_from_dr = last_abs_pose_ * delta;
    //     // guess_from_dr.translation()[2] = 0;
    // }

    // bool try_dr = false;
    // if (((guess_from_dr.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //      (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //     LOG(INFO) << "trying dr pose: " << guess_from_dr.translation().transpose() << ", "
    //               << (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm()
    //               << ", vel_norm: " << current_vel_b_.norm();
    //     try_dr = true;
    // }

    bool try_self = false;
    // if (options_.try_self_extrap_) {
    //     if (((guess_from_self.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_) &&
    //         ((guess_from_dr.translation() - guess_from_self.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_dr.so3().inverse() * guess_from_self.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //         LOG(INFO) << "trying self extrap pose: " << guess_from_self.translation().transpose() << ", "
    //                   << (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm();
    //         try_self = true;
    //     }
    // }

    /// 5. 载入地图, 与地图匹配定位
    /// 尝试各种初始估计
    CloudPtr output_cloud(new PointCloudType);
    double fitness_score = 0;
    SE3 current_pose_esti = guess_from_lo;
    bool loc_success_lo, loc_success_self, loc_success_dr;
    loc_success_lo = loc_success_self = loc_success_dr = false;
    bool loc_success = false;

    /// 注意load on pose存在滞后，优先load on DR
    map_->LoadOnPose(guess_from_lo);

    loc_success_lo = Localize(current_pose_esti, fitness_score, input, output_cloud);  // LO 那个肯定会算
    double score_lo = fitness_score;

    if (!options_.ndt_dump_dir_.empty()) {
        // 诊断导出：<时刻>.pcd 为 NDT 输入（雷达系，前 header.seq 个点是当前帧，其余是投影的历史关键帧）；
        // index.txt 每行：时刻 当前帧点数 总点数 初值(xyz qxyzw) NDT结果(xyz qxyzw) 分值
        static std::ofstream dump_index(options_.ndt_dump_dir_ + "/index.txt");
        char name[64];
        std::snprintf(name, sizeof(name), "/%.6f.pcd", current_time);
        pcl::io::savePCDFileBinary(options_.ndt_dump_dir_ + name, *input);
        auto put = [](std::ostream& os, const SE3& T) {
            os << " " << T.translation().transpose() << " " << T.unit_quaternion().coeffs().transpose();
        };
        dump_index << std::fixed << std::setprecision(6) << current_time << " " << input->header.seq << " "
                   << input->size();
        put(dump_index, guess_from_lo);
        put(dump_index, current_pose_esti);
        dump_index << " " << fitness_score << "\n";
        dump_index.flush();
    }

    SE3 res_of_lo = current_pose_esti;
    SE3 res_of_dr = current_pose_esti;
    SE3 res_of_self = current_pose_esti;
    double score_dr = 0;

    // 先尝试外部预测，最后用自身
    // if (try_dr) {
    //     /// 尝试DR外推的pose
    //     res_of_dr = guess_from_dr;
    //     loc_success_dr = Localize(res_of_dr, score_dr, input, output_cloud);
    //     if (score_dr > (fitness_score - 0.1)) {
    //         current_pose_esti = res_of_dr;
    //         fitness_score = score_dr;
    //         LOG(INFO) << "take dr guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_dr << ", v_norm: " << current_vel_b_.norm();
    //     }
    // }

    if (debug_bias_on_ && converged_) {
        const double rel = current_time - first_align_time_;
        if (rel >= debug_bias_start_ && rel <= debug_bias_end_) {
            current_pose_esti = SE3(SO3(), debug_bias_) * guess_from_lo;
            LOG_EVERY_N(WARNING, 20) << "DEBUG: injected NDT bias " << debug_bias_.head<2>().transpose() << " at +"
                                     << rel << " s";
        }
    }

    // 用纯激光定位有点太抖了，加一些权重：每次只修正 NDT 相对 LO 预测残差的 balance_factor_（默认 10%）。
    // 定位每 ~2 s 才跑一次（loc_on_kf）时，10° 的初始航向误差要 ~45 s 才降到 1°（2026-09-19 实测 τ≈19 s）。
    // 初始化后的快速收敛阶段：NDT 分值足够高时用 fast_converge_factor_，残差连续达标后恢复平滑。
    const SE3 ndt_residual = guess_from_lo.inverse() * current_pose_esti;
    double balance = options_.balance_factor_;
    if (!converged_) {
        ++fast_converge_frames_;
        const double res_ang_deg = ndt_residual.so3().log().norm() * 180.0 / M_PI;
        const double res_pos = ndt_residual.translation().head<2>().norm();
        if (fitness_score >= options_.fast_converge_min_score_) {
            balance = std::max(balance, options_.fast_converge_factor_);
        }
        if (res_ang_deg < options_.fast_converge_ang_deg_ && res_pos < options_.fast_converge_pos_) {
            ++fast_converge_ok_cnt_;
        } else {
            fast_converge_ok_cnt_ = 0;
        }
        LOG(INFO) << "fast converge #" << fast_converge_frames_ << ": residual " << res_ang_deg << " deg / "
                  << res_pos << " m, score " << fitness_score << ", balance " << balance;
        if (fast_converge_ok_cnt_ >= options_.fast_converge_ok_frames_) {
            converged_ = true;
            LOG(INFO) << "lidar loc converged after " << fast_converge_frames_ << " frames, balance back to "
                      << options_.balance_factor_;
        } else if (fast_converge_frames_ >= options_.fast_converge_max_frames_) {
            converged_ = true;
            LOG(WARNING) << "lidar loc not converged within " << fast_converge_frames_
                         << " frames (low score or unstable match), fall back to balance " << options_.balance_factor_;
        }
    }
    // 低分匹配（走出地图覆盖区、遮挡、NDT 落入错误极小值）不融合，只沿 LO 递推，避免把错误结果逐帧累积成漂移。
    if (converged_ && fitness_score < options_.track_min_score_) {
        LOG(INFO) << "low score " << fitness_score << " < " << options_.track_min_score_ << ", follow LO only";
        balance = 0.0;
    }
    // 单帧跳变检查（2026-09-30）：LIO 一帧内的误差只有厘米级，NDT 结果离 LIO 预测太远多半是错配（走廊沿轴向退化，
    // 分值照样高）。本帧不采信，只按 LO 递推。代价：累计误差一旦超过阈值，NDT 就拉不回来（等同纯 LIO）。
    if (converged_ && options_.max_ndt_jump_m_ > 0) {
        const double jump_m = ndt_residual.translation().head<2>().norm();
        const Mat3d R = ndt_residual.rotationMatrix();
        const double jump_deg = std::fabs(std::atan2(R(1, 0), R(0, 0))) * 180.0 / M_PI;
        if (jump_m > options_.max_ndt_jump_m_ ||
            (options_.max_ndt_jump_deg_ > 0 && jump_deg > options_.max_ndt_jump_deg_)) {
            ++ndt_jump_rejects_;
            LOG(INFO) << "ndt jump rejected: " << jump_m << " m / " << jump_deg << " deg (limit "
                      << options_.max_ndt_jump_m_ << " m / " << options_.max_ndt_jump_deg_ << " deg), score "
                      << fitness_score << ", total rejects " << ndt_jump_rejects_;
            balance = 0.0;
        }
    }
    // 一致性守护：窗口内累积修正超限且 LIO / 腿式里程计不支持时冻结修正并退回（快速收敛阶段不管）
    SE3 guard_pose;
    bool guard_rollback = false;
    if (converged_ && current_lo_pose_set_) {
        LocGuard::Frame gf;
        gf.timestamp_ = current_time;
        gf.lo_ = current_lo_pose_;
        gf.guess_ = guess_from_lo;
        gf.ndt_ = current_pose_esti;
        gf.score_ = fitness_score;
        gf.has_odom_ = LegOdomAt(current_time, gf.odom_);
        balance = guard_.Update(gf, balance);
        guard_rollback = guard_.GetRollback(guard_pose);
    }
    current_pose_esti = guess_from_lo * SE3::exp(ndt_residual.log() * balance);
    if (guard_rollback) {
        // 输出跳回被拉偏之前；PGO 窗口里还是拉偏的位姿，不重置会把回退平滑成慢慢拖回，按人工重定位同样重置 PGO
        current_pose_esti = guard_pose;
        external_reinit_done_ = true;
    }

    // double score_self = 0;
    // if (try_self) {
    //     /// 尝试自身外推的pose
    //     LOG(INFO) << "localize with extrap";

    //     res_of_self = guess_from_self;
    //     loc_success_self = Localize(res_of_self, score_self, input, output_cloud);

    //     // 避免分值接近但长时间采信自身预测，此处更相信外部预测源
    //     if (score_self > (fitness_score + 0.1)) {
    //         current_pose_esti = res_of_self;
    //         fitness_score = score_self;
    //         LOG(INFO) << "take self guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_self;
    //     }
    // }

    /// NOTE 如果LO, DR出发点和收敛点不同，但分值相近，说明场景可能处在退化状态，此时使用DR预测的Pose
    // if (try_dr && (res_of_lo.translation() - res_of_dr.translation()).head<2>().norm() > 0.2 &&
    //     fabs(score_lo - score_dr) < 0.2 && score_lo < 1.2) {
    //     LOG(WARNING) << "判定激光定位进入退化状态，现在会使用DR递推pose而不是激光定位位置";
    //     current_pose_esti = guess_from_dr;
    // }

    if (options_.gravity_constrain_) {
        current_pose_esti = ApplyGravityConstraint(current_pose_esti, "track");
    }

    if (options_.force_2d_) {
        PoseRPYD RPYXYZ = math::SE3ToRollPitchYaw(current_pose_esti);
        RPYXYZ.roll = 0;
        RPYXYZ.pitch = 0;
        RPYXYZ.z = 0;
        current_pose_esti = math::XYZRPYToSE3(RPYXYZ);
    }

    // if (options_.with_height_) {
    //     current_pose_esti.translation()[2] = map_height_;
    //     LOG(INFO) << "adjust current pose to : " << current_pose_esti.translation().transpose();
    // }

    current_abs_pose_ = current_pose_esti;
    current_score_ = fitness_score;
    double delta_rel_abs_pose = 0;
    bool lidar_loc_odom_valid = true;

    if (loc_success_lo || loc_success_self || loc_success_dr) {
        loc_success = true;
    } else {
        LOG(INFO) << "loc success is false.";
    }

    if (loc_success) {
        lidar_loc_odom_valid = CheckLidarOdomValid(current_pose_esti, delta_rel_abs_pose);
        last_timestamp_ = current_timestamp_;  // 成功时，更新上一时刻激光定位时间
        match_fail_count_ = 0;
    } else {
        current_score_ = fitness_score;
        LOG(WARNING) << "localization failed! score: " << current_score_;

        ///  若连续3帧匹配失败就设一个大分值
        ++match_fail_count_;
    }

    /// 确定激光定位是否满足平滑性要求
    Vec3d dpred = current_abs_pose_.translation() - guess_from_self.translation();
    if (fabs(dpred[0]) < 0.5 && fabs(dpred[1]) < 0.5 &&
        (current_abs_pose_.so3().inverse() * guess_from_self.so3()).log().norm() < 2.0 * M_PI / 180.0) {
        localization_result_.lidar_loc_smooth_flag_ = true;
    } else {
        localization_result_.lidar_loc_smooth_flag_ = false;
    }

    localization_result_.lidar_loc_odom_reliable_ = lo_reliable_;
    localization_result_.is_parking_ = false;

    // if (ui_) {
    //     ui_->UpdatePredictPose(guess_from_lo);
    // }

    /// 7. 输出结果
    {
        UL lock(result_mutex_);
        localization_result_.timestamp_ = current_timestamp_;
        localization_result_.confidence_ = fitness_score;
        localization_result_.guard_status_ = guard_.GetStatus();
        if (match_fail_count_ < 100) {
            localization_result_.lidar_loc_valid_ = true;
            localization_result_.status_ = LocalizationStatus::GOOD;
        } else if (match_fail_count_ >= 100 && match_fail_count_ < 300) {
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FOLLOWING_DR;
        } else {
            match_fail_count_ = 300;
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FAIL;
        }

        localization_result_.lidar_loc_odom_delta_ = delta_rel_abs_pose;
        localization_result_.lidar_loc_odom_error_normal_ = lidar_loc_odom_valid;
        localization_result_.pose_ = current_pose_esti;
    }

    UpdateState(input);

    /// 8. 更新动态图层
    /// 条件：1. 定位成功 2. 与上次更新间隔一定距离 3. RTK与激光定位横纵向误差都小于0.3，或者匹配分值大于1.0
    bool score_cond = current_score_ > options_.update_lidar_loc_score_;

    if (options_.update_dynamic_cloud_ && loc_success &&
        (((current_pose_esti.translation() - last_dyn_upd_pose_.pose_.translation()).norm() >
          options_.update_kf_dis_) ||
         fabs(current_time - last_dyn_upd_pose_.timestamp_) > options_.update_kf_time_)) {
        if (score_cond /*  || update_cache_dis_ < options_.max_update_cache_dis_ */) {
            // LOG(INFO) << "passing through z filter, input:" << input->size();
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(input);

            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.5, options_.filter_z_max_);

            CloudPtr input_z_filter(new PointCloudType());
            pass.filter(*input_z_filter);

            if (!input_z_filter->empty()) {
                CloudPtr cloud_t(new PointCloudType());
                pcl::transformPointCloud(*input_z_filter, *cloud_t, current_pose_esti.matrix());

                // 以现在的scan来更新地图
                map_->UpdateDynamicCloud(cloud_t, true);

                last_dyn_upd_pose_.timestamp_ = current_time;
                last_dyn_upd_pose_.pose_ = current_pose_esti;
            }
        }
    }

    if (lidar_loc_pose_queue_.empty()) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    } else if (current_time > lidar_loc_pose_queue_.back().timestamp_) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    }

    while (lidar_loc_pose_queue_.size() > 1000) {
        lidar_loc_pose_queue_.pop_front();
    }

    ave_scores_.emplace_back(current_score_);
    while (ave_scores_.size() > 20) {
        ave_scores_.pop_front();
    }

    /// 9. save for recover pose
    recover_pose_out_.open(options_.recover_pose_path_);
    if (recover_pose_out_) {
        Vec3d t = current_pose_esti.translation();
        Quatd q = current_pose_esti.unit_quaternion();
        recover_pose_out_ << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " "
                          << q.w();
        recover_pose_out_.close();
    }
}

bool LidarLoc::CheckLidarOdomValid(const SE3& current_pose_esti, double& delta_posi) {
    delta_posi = ((last_lo_pose_.inverse() * current_lo_pose_).translation() -
                  (last_abs_pose_.inverse() * current_pose_esti).translation())
                     .head(2)
                     .norm();

    bool valid = true;

    if (delta_posi > options_.lidar_loc_odom_th_ && last_lo_pose_set_) {
        LOG(INFO) << "delta_rel_abs_pose is: " << delta_posi;
        LOG(INFO) << "LO相对pose: " << last_lo_pose_.translation().transpose() << " "
                  << current_lo_pose_.translation().transpose() << " "
                  << (last_lo_pose_.inverse() * current_lo_pose_).translation().transpose();
        LOG(INFO) << "Lidar Loc计算相对pose: " << last_abs_pose_.translation().transpose() << " "
                  << current_pose_esti.translation().transpose() << " "
                  << (last_abs_pose_.inverse() * current_pose_esti).translation().transpose();
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
        valid = false;
    }

    last_abs_pose_ = current_pose_esti;
    last_lo_pose_ = current_lo_pose_;
    last_lo_pose_set_ = true;
    last_dr_pose_ = current_dr_pose_;
    last_dr_pose_set_ = true;

    return valid;
}

bool LidarLoc::Localize(SE3& pose, double& confidence, CloudPtr input, CloudPtr output, bool use_rough_res) {
    Eigen::Matrix4f trans;
    bool loc_success = false;
    Eigen::Matrix4f guess_pose = pose.matrix().cast<float>();

    LOG(INFO) << "loc from: " << pose.translation().transpose();

    // pcl_ndt_ 会被 UpdateMapThread 在 match_mutex_ 下整体替换（启动后首轮必定替换一次），
    // 读取它也必须持锁，否则可能解引用刚被释放的旧 NDT 对象。
    UL lock(match_mutex_);
    if (pcl_ndt_->getInputTarget() == nullptr) {
        LOG(INFO) << "lidar loc target is null, skip";
        return false;
    }

    NDTType::Ptr ndt = nullptr;

    if (use_rough_res) {
        ndt = pcl_ndt_rough_;
    } else {
        ndt = pcl_ndt_;
    }

    ndt->setInputSource(input);
    ndt->align(*output, guess_pose);
    trans = ndt->getFinalTransformation();
    confidence = ndt->getTransformationProbability();
    if (!trans.allFinite() || !std::isfinite(confidence)) {
        // NaN 位姿交给 Sophus 会直接 abort 整个定位进程；本帧当作匹配失败，保持初值、分值记 0
        LOG(WARNING) << "ndt result not finite, keep guess";
        confidence = 0;
        return false;
    }

    if (options_.log_ndt_hessian_) {
        // 得分是最大化，A = -H 在解附近半正定；变量顺序 x y z roll pitch yaw（地图系）。
        // xy：只动平移时的曲率；schur：其余 4 维各自取最优后，xy 剩下的曲率（含航向-平移耦合）。
        // 特征值单位是 得分/m²，与点数成正比，n 一并输出便于归一化。ang 为弱方向在地图系的角度（度，[-90, 90)）。
        Eigen::Matrix<double, 6, 6> A = -ndt->getHessionMatrix();
        A = 0.5 * (A + A.transpose()).eval();
        auto eig2 = [](const Eigen::Matrix2d& m, double& l0, double& l1, double& ang) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(m);
            l0 = es.eigenvalues()(0);
            l1 = es.eigenvalues()(1);
            const Eigen::Vector2d v = es.eigenvectors().col(0);
            ang = std::atan2(v.y(), v.x()) * 180.0 / M_PI;
            if (ang >= 90) ang -= 180;
            if (ang < -90) ang += 180;
        };
        double b0, b1, ba, s0, s1, sa;
        eig2(A.block<2, 2>(0, 0), b0, b1, ba);
        const Eigen::Matrix4d A_oo = A.block<4, 4>(2, 2);
        const Eigen::Matrix<double, 2, 4> A_xo = A.block<2, 4>(0, 2);
        const Eigen::Matrix2d S = A.block<2, 2>(0, 0) - A_xo * A_oo.completeOrthogonalDecomposition().solve(A_xo.transpose());
        eig2(0.5 * (S + S.transpose()), s0, s1, sa);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es6(A);
        LOG(INFO) << std::fixed << std::setprecision(3) << "ndt hess: n " << input->size() << " it "
                  << ndt->getFinalNumIteration() << " conf " << confidence << " t " << trans(0, 3) << " " << trans(1, 3)
                  << " | xy " << b0 << " " << b1 << " ang " << ba << " | schur " << s0 << " " << s1 << " ang " << sa
                  << " | eig6 " << es6.eigenvalues().transpose();
    }

    if (loc_inited_ == false && confidence > options_.min_init_confidence_) {
        loc_success = true;
    } else {
        loc_success = true;
    }

    if (options_.enable_icp_adjust_ && loc_inited_) {
        Eigen::Matrix4f adjust_trans;
        CloudPtr input_voxel(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel_icp;

        double ls = 0.2;
        voxel_icp.setLeafSize(ls, ls, ls);
        voxel_icp.setInputCloud(input);
        voxel_icp.filter(*input_voxel);
        pcl_icp_->setInputSource(input_voxel);
        Timer::Evaluate([&]() { pcl_icp_->align(*output, trans); }, "pcl_icp adjust", true);
        adjust_trans = pcl_icp_->getFinalTransformation();

        Eigen::Matrix3f rotation_diff = trans.block<3, 3>(0, 0).transpose() * adjust_trans.block<3, 3>(0, 0);
        Eigen::AngleAxisf angle_axis(rotation_diff);
        float a = angle_axis.angle();
        float d = (trans.block<3, 1>(0, 3) - adjust_trans.block<3, 1>(0, 3)).norm();
        LOG(INFO) << "icp adjust d: " << d << ", a: " << a;

        if (pcl_icp_->hasConverged() && std::fabs(d) <= 0.05 && std::fabs(a) <= 0.05) {
            LOG(INFO) << "icp ajust trans set success";
            trans = adjust_trans;
        }
    }

    Eigen::Matrix3d rot = trans.block<3, 3>(0, 0).cast<double>();
    Quatd q_3d = Quatd(rot);
    Vec3d t_3d = trans.block<3, 1>(0, 3).cast<double>();
    q_3d.normalize();
    pose = SE3(q_3d, t_3d);

    {
        auto rpy = math::SE3ToRollPitchYaw(pose);  // 诊断：NDT 结果姿态（度）
        LOG(INFO) << "confidence: " << confidence << ", t: " << t_3d.transpose() << ", succ: " << loc_success
                  << ", rpy(deg): " << rpy.roll * 180 / M_PI << " " << rpy.pitch * 180 / M_PI << " "
                  << rpy.yaw * 180 / M_PI;
    }

    return loc_success;
}

bool LidarLoc::CheckStatic(double timestamp) {
    if (parking_) {
        // if (current_vel_b_.norm() < common::options::lo::parking_speed) {
        // LOG(INFO) << "car is in static mode";
        static_count_++;
        if (static_count_ >= lo::parking_count) {
            static_count_ = 0;
            return false;
        }
        return true;
    } else {
        static_count_ = 0;
        return false;
    }
}

void LidarLoc::UpdateState(const CloudPtr& input) { last_timestamp_ = current_timestamp_; }

void LidarLoc::SetInitRltState() {
    UL lock(result_mutex_);
    localization_result_.confidence_ = 0.0;
    localization_result_.timestamp_ = current_timestamp_;
    localization_result_.lidar_loc_valid_ = false;
    localization_result_.status_ = LocalizationStatus::INITIALIZING;
}

bool LidarLoc::LocInited() {
    // UL lock( data_mutex_);
    return loc_inited_;
}

bool LidarLoc::AssignLOPose(double timestamp) {
    UL lock(lo_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    // 无法拿到最新的，插值结果都是外推出来的，和真实有时偏差会很大（╯﹏╰）
    // if (!lo_pose_queue_.empty()) {
    //     LOG(INFO) << "lo interp: " << timestamp << " " << lo_pose_queue_.back().timestamp_ << " "
    //               << lo_pose_queue_.back().pose_.translation().transpose();
    // }

    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, lo_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        current_lo_pose_ = interp_pose;
        current_lo_pose_set_ = true;

        current_vel_b_ = best_match.GetRot().inverse() * best_match.GetVel();
        if (best_match.grav_.norm() > 1e-3) {
            current_up_body_ = -(best_match.GetRot().inverse() * best_match.grav_).normalized();
            current_up_body_set_ = true;
        }
        current_vel_ = best_match.GetVel();

        // if (options_.with_height_) {
        //     current_lo_pose_.translation()[2] = map_height_;
        // }

        return true;
    } else {
        current_lo_pose_set_ = false;
        return false;
    }
}

SE3 LidarLoc::ApplyGravityConstraint(const SE3& pose, const char* tag) const {
    if (!current_up_body_set_) {
        LOG(WARNING) << "gravity constrain(" << tag << "): LIO gravity not ready, skip";
        return pose;
    }
    const Vec3d up_map = pose.so3() * current_up_body_;  // 按当前估计，重力上方向在 map 系中的指向
    const Vec3d axis = up_map.cross(map_up_);           // 目标：地图自身的重力上方向
    const double s = axis.norm();
    const double angle = std::atan2(s, up_map.dot(map_up_));
    if (s < 1e-9) {
        return pose;
    }
    if (angle * 180.0 / M_PI > options_.gravity_constrain_max_deg_) {
        LOG(WARNING) << "gravity constrain(" << tag << "): tilt error " << angle * 180.0 / M_PI
                     << " deg too large, skip";
        return pose;
    }
    // 绕水平轴的最小旋转：一阶上不改变航向，平移（雷达位置）保持不变
    const SE3 out(SO3::exp(axis / s * angle) * pose.so3(), pose.translation());
    const auto rpy = math::SE3ToRollPitchYaw(out);
    LOG(INFO) << "gravity constrain(" << tag << "): corrected " << angle * 180.0 / M_PI
              << " deg, rpy(deg): " << rpy.roll * 180 / M_PI << " " << rpy.pitch * 180 / M_PI << " "
              << rpy.yaw * 180 / M_PI;
    return out;
}

bool LidarLoc::AssignDRPose(double timestamp) {
    UL lock(dr_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, dr_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        parking_ = best_match.is_parking_;
        current_dr_pose_ = interp_pose;
        current_dr_pose_set_ = true;

        // if (options_.with_height_) {
        //     current_dr_pose_.translation()[2] = map_height_;
        // }

        return true;
    } else {
        parking_ = false;
        current_dr_pose_set_ = false;
        return false;
    }
}

void LidarLoc::Finish() {
    if (map_) {
        LOG(INFO) << "saving maps";
        update_map_quit_ = true;
        update_map_thread_.join();

        /// 永久保存时，再存储地图
        if (options_.map_option_.policy_ == TiledMap::DynamicCloudPolicy::PERSISTENT &&
            options_.map_option_.save_dyn_when_quit_ && !has_set_pose_) {
            map_->SaveToBin(true);
            LOG(INFO) << "dynamic maps saved";
        }
    }
}

}  // namespace lightning::loc
