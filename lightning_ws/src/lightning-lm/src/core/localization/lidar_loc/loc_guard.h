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
        int relocks_ = 0;         // 单帧跳变检查锁死后被 JumpRelock 解除的累计次数
        int jump_reject_run_ = 0; // 当前连续被跳变检查拒绝的帧数
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

/**
 * 单帧跳变检查（lidar_loc max_ndt_jump_m/deg）的锁死解除。
 *
 * 背景（2026-10-24，Orin 30524）：输出与地图的偏差一旦超过跳变门限（0.2 m），之后 NDT 的每次修正都被当作"跳变"拒绝，
 * 输出只能跟 LIO，偏差永远拉不回来，守护还报 GOOD（10-23 17:53 偏 0.55 m 锁 25 min；10-24 15:00 偏 0.5 m 锁到结束）。
 * 门限本身是为挡住走廊里 NDT 的单帧错配，不能简单放宽。这里在"连续被拒"时检查被拒的 NDT 结果是否可信：
 *   - 连续 frames 帧被拒（中间没有残差 < reset_m 的正常帧）；
 *   - 这几帧换到地图系的修正量 C = ndt * guess^-1 彼此一致（偏离均值 <= consistency_m / consistency_deg）——
 *     输出整体偏了一个固定量时 C 恒定，NDT 在两个局部最优间跳时 C 不一致；
 *   - 期间 LIO 净位移 >= min_travel_m：换了视角仍落在同一修正量上，才不是同一个错误极小值反复出现
 *     （只看净位移不看转角：G1 步态躯干每步摆 ±5°，原地转也不改变平移上的歧义，起点开阔区就是这样）；
 *   - 分值都不低于 max(min_score, score_ratio × 最近 score_hist 帧分值中位数)（各张图正常分值差很多：1.5~2.8）；
 *   - 修正量不超过 max_m / max_deg（更大的交给人工重定位）。
 * 满足时返回 kAccept，调用方本帧整帧采信 NDT 并重置守护 / PGO。
 * 采信后验证：之后 verify_frames 帧内又被拒 >= verify_fail 帧，说明 NDT 是"相对当前位姿往一个方向拉"（09-29 走廊那种，
 * 5 帧窗口内修正量同样一致、和真锁死分不开），返回 kRevert：调用方用 RevertCorrection() 撤回本次修正，之后停用解除，
 * 直到 Reset()（重新初始化 / 人工重定位）。连续被拒 >= hint_frames 帧或已停用时 NeedReloc() 为 true。
 * 用 10-21~10-24 共 16 段 glog 离线复算（tools/loc_debug/sim_relock.py）：上面两次锁死分别在 17:56:43、15:01:05 解除，
 * 起点静止 / 原地转的歧义（10-23 上午、10-24 10:39、12:14、14:12、14:53）都不解除。
 */
class JumpRelock {
   public:
    struct Options {
        Options() {}
        int frames_ = 5;
        double min_score_ = 1.2;
        double score_ratio_ = 0.95;
        int score_hist_ = 50;
        double consistency_m_ = 0.15;
        double consistency_deg_ = 2.0;
        double min_travel_m_ = 0.5;
        double max_m_ = 1.0;
        double max_deg_ = 10.0;
        double reset_m_ = 0.10;  // 未被拒且残差小于该值的帧打断"连续被拒"
        int hint_frames_ = 15;   // 0 = 不提示
        int verify_frames_ = 5;  // 采信后验证的帧数
        int verify_fail_ = 3;    // 验证期内被拒达到该帧数即撤回
    };

    enum class Action { kNone = 0, kAccept = 1, kRevert = 2 };

    explicit JumpRelock(Options options = Options()) : options_(options) {}

    /**
     * 每个跟踪帧（快速收敛结束后）调用一次。rejected：本帧是否被跳变检查拒绝；guess / ndt：地图系雷达位姿
     * （LIO 递推的预测 / NDT 结果）；lo：LIO 位姿；score：NDT 分值。返回 true = 本帧应整帧采信 NDT。
     */
    Action Update(bool rejected, const SE3& guess, const SE3& ndt, const SE3& lo, double score);

    /// kRevert 时：把输出左乘它即撤回最近一次采信的修正（地图系）
    SE3 RevertCorrection() const { return correction_.inverse(); }

    bool NeedReloc() const { return disabled_ || (options_.hint_frames_ > 0 && run_count_ >= options_.hint_frames_); }
    bool Disabled() const { return disabled_; }
    int Reverts() const { return reverts_; }
    int RunLength() const { return run_count_; }
    int Relocks() const { return relocks_; }
    const Options& GetOptions() const { return options_; }
    /// 最近一次采信时的平均修正量（日志用）
    double LastCorrectionM() const { return last_corr_m_; }
    double LastCorrectionDeg() const { return last_corr_deg_; }
    /// 重新初始化 / 人工重定位后调用：清空连续被拒记录、结束验证、恢复启用
    void Reset() {
        ResetRun();
        verify_n_ = -1;
        disabled_ = false;
    }

   private:
    struct Item {
        double cx_ = 0, cy_ = 0, cyaw_ = 0;  // 地图系修正量（平面）
        double jump_m_ = 0, jump_deg_ = 0;   // 相对 LIO 预测的残差（与跳变检查同口径）
        double lo_x_ = 0, lo_y_ = 0;
        double score_ = 0;
    };
    Options options_;
    std::deque<Item> run_;  // 最近 frames 帧被拒的记录
    int run_count_ = 0;     // 当前连续被拒帧数
    std::deque<double> scores_;
    int relocks_ = 0;
    int reverts_ = 0;
    double last_corr_m_ = 0, last_corr_deg_ = 0;
    SE3 correction_;        // 最近一次采信的地图系修正
    int verify_n_ = -1;     // >= 0：采信后验证中，已过帧数
    int verify_fail_n_ = 0;
    bool disabled_ = false; // 撤回过一次：停用解除直到 Reset()

    void ResetRun() {
        run_.clear();
        run_count_ = 0;
    }
};

}  // namespace lightning::loc
