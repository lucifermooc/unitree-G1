#include "core/localization/lidar_loc/loc_guard.h"

#include <glog/logging.h>
#include <cmath>
#include <cstdio>
#include <iomanip>

namespace lightning::loc {

const char* LocGuard::StateName(State s) {
    switch (s) {
        case State::GOOD:
            return "GOOD";
        case State::NDT_SUSPECT:
            return "NDT_SUSPECT";
        case State::LIO_SUSPECT:
            return "LIO_SUSPECT";
    }
    return "UNKNOWN";
}

void LocGuard::Planar(const SE3& T, double& trans, double& yaw_deg) {
    trans = T.translation().head<2>().norm();
    const Mat3d R = T.rotationMatrix();
    yaw_deg = std::fabs(std::atan2(R(1, 0), R(0, 0))) * 180.0 / M_PI;
}

void LocGuard::Reset() {
    history_.clear();
    recover_cnt_ = 0;
    suspect_frames_ = 0;
    odom_bad_cnt_ = 0;
    odom_ok_cnt_ = 0;
    rollback_now_ = false;
    const int rollbacks = status_.rollbacks_;
    status_ = Status();
    status_.rollbacks_ = rollbacks;  // 累计值跨重定位保留，便于看一次运行里回退了几次
}

const LocGuard::Entry* LocGuard::PickReference(double now, double dist) const {
    // 取满足"时间 >= window_sec 或 行程 >= window_dist"的最新一帧；历史不够长时用最老的一帧
    for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
        if (now - it->timestamp_ >= options_.window_sec_ || dist - it->dist_ >= options_.window_dist_) {
            return &(*it);
        }
    }
    return history_.empty() ? nullptr : &history_.front();
}

double LocGuard::Update(const Frame& frame, double proposed_balance) {
    rollback_now_ = false;
    if (!options_.enabled_) {
        return proposed_balance;
    }

    status_.timestamp_ = frame.timestamp_;
    status_.score_ = frame.score_;
    const SE3 residual = frame.guess_.inverse() * frame.ndt_;
    Planar(residual, status_.residual_m_, status_.residual_deg_);

    double dist = 0;
    if (!history_.empty()) {
        dist = history_.back().dist_ + (frame.lo_.translation() - history_.back().lo_.translation()).head<2>().norm();
    }

    const Vec6d residual_log = residual.log();
    auto output_with = [&](double balance) { return frame.guess_ * SE3::exp(residual_log * balance); };

    // 窗口内：输出相对运动 与 LIO 相对运动 之差 = NDT 注入的累积修正
    double drift_m = 0, drift_deg = 0, odom_err_m = 0, odom_err_deg = 0, travel = 0, path = 0;
    int odom_agree = -1;
    const Entry* ref = PickReference(frame.timestamp_, dist);
    if (ref) {
        const SE3 lo_rel = ref->lo_.inverse() * frame.lo_;
        const SE3 out_rel = ref->out_.inverse() * output_with(proposed_balance);
        Planar(out_rel.inverse() * lo_rel, drift_m, drift_deg);
        travel = lo_rel.translation().head<2>().norm();
        path = dist - ref->dist_;  // 路程（原地转圈、走过去又走回来时位移小但路程不小）

        if (ref->has_odom_ && frame.has_odom_) {
            const SE3 odom_rel = ref->odom_.inverse() * frame.odom_;
            Planar(lo_rel.inverse() * odom_rel, odom_err_m, odom_err_deg);
            odom_agree = (odom_err_m < options_.odom_tol_m_ + options_.odom_tol_ratio_ * travel &&
                          odom_err_deg < options_.odom_tol_deg_)
                             ? 1
                             : 0;
        }
    }
    status_.drift_m_ = drift_m;
    status_.drift_deg_ = drift_deg;
    status_.odom_agree_ = odom_agree;
    status_.odom_err_m_ = odom_err_m;
    status_.odom_err_deg_ = odom_err_deg;
    status_.travel_m_ = travel;

    // 里程计投票要连续若干帧才算数：09-30 导航转身时里程计 y 向误差达 0.5 m，单帧判断让状态来回切了 ~70 次
    if (odom_agree == 0) {
        ++odom_bad_cnt_;
        odom_ok_cnt_ = 0;
    } else if (odom_agree == 1) {
        ++odom_ok_cnt_;
        odom_bad_cnt_ = 0;
    } else {
        odom_bad_cnt_ = odom_ok_cnt_ = 0;
    }
    const bool odom_blames_lio = odom_bad_cnt_ >= options_.odom_persist_frames_;
    const bool odom_backs_lio = odom_ok_cnt_ >= options_.odom_persist_frames_;

    const double max_m = path < options_.static_travel_m_ ? options_.max_drift_static_m_ : options_.max_drift_m_;
    const bool over = drift_m > max_m || drift_deg > options_.max_drift_deg_;
    const bool residual_ok =
        status_.residual_m_ < options_.recover_res_m_ && status_.residual_deg_ < options_.recover_res_deg_;
    const State prev = status_.state_;
    bool clear_history = false;

    switch (prev) {
        case State::GOOD:
            if (over) {
                // 里程计持续说 LIO 错了才怪 LIO；没有里程计或里程计同意 LIO 时怪 NDT
                status_.state_ = odom_blames_lio ? State::LIO_SUSPECT : State::NDT_SUSPECT;
            }
            break;
        case State::LIO_SUSPECT:
            if (!over) {
                status_.state_ = State::GOOD;
            } else if (odom_backs_lio) {
                status_.state_ = State::NDT_SUSPECT;
            }
            break;
        case State::NDT_SUSPECT:
            recover_cnt_ = residual_ok ? recover_cnt_ + 1 : 0;
            if (recover_cnt_ >= options_.recover_frames_) {
                // NDT 重新与 LIO 一致；清空窗口，否则窗口里冻结前的漂移会立刻再次触发
                status_.state_ = State::GOOD;
                clear_history = true;
            } else if (odom_blames_lio) {
                // 冻结期间里程计持续反对 LIO：更可能是 LIO 在漂，解冻让地图匹配去拉
                status_.state_ = State::LIO_SUSPECT;
            }
            break;
    }

    if (status_.state_ != State::NDT_SUSPECT) {
        recover_cnt_ = 0;
        suspect_frames_ = 0;
    } else {
        ++suspect_frames_;
    }
    status_.need_reloc_ = status_.state_ == State::NDT_SUSPECT && suspect_frames_ > options_.reloc_hint_frames_;

    if (status_.state_ != prev) {
        LOG(WARNING) << "loc guard: " << StateName(prev) << " -> " << StateName(status_.state_)
                     << ", drift " << drift_m << " m / " << drift_deg << " deg (limit " << max_m << " m, path " << path
                     << " m), ndt residual " << status_.residual_m_ << " m / " << status_.residual_deg_
                     << " deg, odom_agree " << odom_agree << " (bad " << odom_bad_cnt_ << " ok " << odom_ok_cnt_
                     << "), score " << frame.score_;
    } else if (status_.state_ != State::GOOD) {
        LOG_EVERY_N(WARNING, 10) << "loc guard: still " << StateName(status_.state_) << ", drift " << drift_m
                                 << " m / " << drift_deg << " deg, ndt residual " << status_.residual_m_ << " m / "
                                 << status_.residual_deg_ << " deg" << (status_.need_reloc_ ? ", NEED RELOC" : "");
    }

    const bool frozen = status_.state_ == State::NDT_SUSPECT && options_.freeze_;
    const double balance = frozen ? 0.0 : proposed_balance;
    SE3 out = output_with(balance);

    // 刚进入冻结：只冻结会停在已被拉偏的位置（阈值 + 当帧注入量），把窗口内注入的修正一并退回：
    // 输出 = 参考帧的修正量（map <- LIO）× 当前 LIO 位姿
    if (frozen && prev != State::NDT_SUSPECT && options_.rollback_ && ref != nullptr) {
        const SE3 rolled = (ref->out_ * ref->lo_.inverse()) * frame.lo_;
        status_.last_rollback_m_ = (rolled.translation() - out.translation()).head<2>().norm();
        ++status_.rollbacks_;
        LOG(WARNING) << "loc guard: rollback " << status_.last_rollback_m_ << " m to the correction of "
                     << frame.timestamp_ - ref->timestamp_ << " s ago (rollback #" << status_.rollbacks_ << ")";
        out = rolled;
        rollback_now_ = true;
        rollback_pose_ = rolled;
        clear_history = true;  // 窗口里是被拉偏的输出，留着会让之后的漂移量失真
    }
    status_.frozen_ = frozen;

    if (clear_history) {
        history_.clear();
        dist = 0;
    }
    // 逐帧记录（离线分析阈值、里程计精度用）：平面位姿 x y yaw(deg)
    auto xyyaw = [](const SE3& T) {
        const Mat3d R = T.rotationMatrix();
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%.3f %.3f %.2f", T.translation().x(), T.translation().y(),
                      std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI);
        return std::string(buf);
    };
    LOG(INFO) << std::fixed << std::setprecision(3) << "loc guard frame: t " << frame.timestamp_ << " lo "
              << xyyaw(frame.lo_) << " out " << xyyaw(out) << " odom " << frame.has_odom_ << " "
              << xyyaw(frame.odom_) << " drift " << drift_m << " " << drift_deg << " odom_err " << odom_err_m << " "
              << odom_err_deg << " travel " << travel << " res " << status_.residual_m_ << " "
              << status_.residual_deg_ << " state " << StateName(status_.state_) << " balance " << balance
              << " path " << path << " rollback " << rollback_now_;

    Entry entry;
    entry.timestamp_ = frame.timestamp_;
    entry.dist_ = dist;
    entry.lo_ = frame.lo_;
    entry.out_ = out;
    entry.has_odom_ = frame.has_odom_;
    entry.odom_ = frame.odom_;
    history_.push_back(entry);
    while (history_.size() > 2 &&
           (history_.front().timestamp_ < frame.timestamp_ - 3 * options_.window_sec_ || history_.size() > 5000)) {
        history_.pop_front();
    }

    return balance;
}

}  // namespace lightning::loc
