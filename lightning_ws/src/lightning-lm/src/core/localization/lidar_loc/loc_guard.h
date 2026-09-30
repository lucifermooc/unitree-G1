#pragma once

#include <deque>
#include <string>

#include "common/eigen_types.h"

namespace lightning::loc {

/**
 * 定位一致性守护：发现地图匹配（NDT）把位姿慢慢拉离 LIO / 腿式里程计时，暂停修正并告警。
 *
 * 背景（2026-09-29 实测）：NDT 每帧只注入 10% 残差，单帧永远只错几厘米，
 * 所有单帧检查都看不出来；走廊里 NDT 在 ±0.55 m 两个局部最优间跳，几分钟累积 ~2 m / 19°，
 * 而 LIO 85 分钟回到起点只差 9 cm。这里在一个窗口（最近 window_sec 秒或 LIO 走过 window_dist 米）内
 * 比较"输出位姿的相对运动"与"LIO 的相对运动"，差值即 NDT 注入的累积修正。
 *
 * 所有位姿都是雷达系（腿式里程计由调用方按安装外参换到雷达系），只比较平面分量 x/y/yaw。
 * 纯逻辑，不依赖 ROS，便于单测。
 *
 * 2026-09-30 新图导航实测：边走边快速转身时 NDT 把输出沿走廊拉走 1.5~2.2 m，全程 LIO 与腿式里程计互差 ~0.1 m。
 * 因此判为 NDT_SUSPECT 时不只冻结，还把窗口内已注入的修正退回（rollback），输出回到"参考帧修正量 × 当前 LIO"。
 */
class LocGuard {
   public:
    struct Options {
        Options() {}  // 用作类内默认参数时 GCC 需要用户定义的构造函数（同 LidarLoc::Options）
        bool enabled_ = true;
        bool freeze_ = true;             // true: NDT_SUSPECT 时冻结修正（balance=0）；false: 只告警
        bool rollback_ = true;           // 冻结时把窗口内累积的修正退回，而不是停在已被拉偏的位置（需 freeze）
        double window_sec_ = 60.0;       // 比较窗口：时间（静止时的慢拉要靠长窗口才看得出）
        double window_dist_ = 5.0;       // 比较窗口：LIO 行程（先满足哪个用哪个）
        double max_drift_m_ = 0.20;      // 窗口内累积修正超过该值视为异常
        double max_drift_deg_ = 5.0;
        double static_travel_m_ = 0.30;  // 窗口内 LIO 路程小于该值视为静止，改用 max_drift_static_m
        double max_drift_static_m_ = 0.10;  // 静止时 LIO 几乎不漂，NDT 不该有修正（09-30 14:51 静止被拉 0.19 m）
        int recover_frames_ = 10;        // 连续多少帧 NDT 残差达标后恢复
        double recover_res_m_ = 0.15;
        double recover_res_deg_ = 2.0;
        int reloc_hint_frames_ = 30;     // 冻结后仍不恢复超过该帧数，提示需要重定位
        double odom_tol_m_ = 0.15;       // LIO 与腿式里程计一致：平移误差 < odom_tol_m + odom_tol_ratio * 行程
        double odom_tol_ratio_ = 0.15;   // 09-29 实测里程计误差/行程 p95 10.9%、航向 p95 6°
        double odom_tol_deg_ = 10.0;
        int odom_persist_frames_ = 5;    // 里程计连续不一致（或一致）这么多帧才据此改判：转身时里程计单帧误差大，会来回跳
    };

    enum class State { GOOD = 0, NDT_SUSPECT = 1, LIO_SUSPECT = 2 };

    struct Status {
        State state_ = State::GOOD;
        double drift_m_ = 0;      // 窗口内累积修正（平移）
        double drift_deg_ = 0;    // 窗口内累积修正（航向）
        int odom_agree_ = -1;     // -1 无里程计 / 0 LIO 与里程计不一致 / 1 一致
        double residual_m_ = 0;   // 本帧 NDT 相对 LIO 预测的残差
        double residual_deg_ = 0;
        double score_ = 0;
        bool need_reloc_ = false; // 冻结后长时间不恢复
        double odom_err_m_ = 0;   // 窗口内 LIO 与里程计相对运动之差（有里程计时）
        double odom_err_deg_ = 0;
        double travel_m_ = 0;     // 窗口内 LIO 行程
        double timestamp_ = 0;
        bool frozen_ = false;     // 当前是否冻结修正（NDT_SUSPECT 且 freeze）
        int rollbacks_ = 0;       // 本次运行累计回退次数
        double last_rollback_m_ = 0;
    };

    /// 一次激光定位的输入
    struct Frame {
        double timestamp_ = 0;
        SE3 lo_;              // LIO 位姿（雷达系）
        SE3 guess_;           // 由 LIO 递推的预测
        SE3 ndt_;             // NDT 结果
        double score_ = 0;
        bool has_odom_ = false;
        SE3 odom_;            // 同一时刻腿式里程计位姿（已换到雷达系）
    };

    explicit LocGuard(Options options = Options()) : options_(options) {}

    /**
     * 处理一帧，返回实际使用的 balance（NDT_SUSPECT 且 freeze 时为 0，否则为 proposed_balance）。
     * 必须在每次跟踪阶段的激光定位后调用一次。
     */
    double Update(const Frame& frame, double proposed_balance);

    /// 上一次 Update 是否决定回退；是则 pose 为本帧应输出的位姿（调用方直接用它替代 balance 插值结果）
    bool GetRollback(SE3& pose) const {
        if (rollback_now_) pose = rollback_pose_;
        return rollback_now_;
    }

    /// 重新初始化（外部设置位姿、定位重启）后清空历史
    void Reset();

    const Status& GetStatus() const { return status_; }
    const Options& GetOptions() const { return options_; }

    static const char* StateName(State s);

   private:
    struct Entry {
        double timestamp_ = 0;
        double dist_ = 0;  // LIO 累积行程
        SE3 lo_;
        SE3 out_;
        bool has_odom_ = false;
        SE3 odom_;
    };

    /// 平面分量：平移长度 / 航向角（度）
    static void Planar(const SE3& T, double& trans, double& yaw_deg);
    const Entry* PickReference(double now, double dist) const;

    Options options_;
    Status status_;
    std::deque<Entry> history_;
    int recover_cnt_ = 0;
    int suspect_frames_ = 0;
    int odom_bad_cnt_ = 0;  // 里程计连续不一致帧数
    int odom_ok_cnt_ = 0;   // 里程计连续一致帧数
    bool rollback_now_ = false;
    SE3 rollback_pose_;
};

}  // namespace lightning::loc
