// LocGuard 单测：用闭环模拟复现 2026-09-29 的走廊漂移（NDT 在 ±0.55 m 两个局部最优间跳，
// 每帧按 balance 0.1 注入残差），验证守护能挡住、能恢复、不误报，并能区分"NDT 错"和"LIO 错"。
// 2026-09-30 补：冻结时退回已注入的修正、静止慢拉（14:51）、导航转身时里程计单帧跳变不来回切。
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "core/localization/lidar_loc/loc_guard.h"

using lightning::SE3;
using lightning::SO3;
using lightning::Vec3d;
using lightning::loc::LocGuard;
using State = LocGuard::State;

namespace {

constexpr double kDt = 0.5;       // 激光定位 2 Hz（走路时关键帧频率量级）
constexpr double kSpeed = 0.5;    // 0.5 m/s
constexpr double kBalance = 0.1;  // lidar_loc 的 balance_factor
// 回退到参考帧的修正量，参考帧自身那一帧已注入的量（balance × 0.55 m）不在退回范围内
constexpr double kOneFrame = kBalance * 0.55 + 0.005;

SE3 Pose(double x, double y = 0, double yaw = 0) { return SE3(SO3::rotZ(yaw), Vec3d(x, y, 0)); }

double PosErr(const SE3& a, const SE3& b) { return (a.translation() - b.translation()).head<2>().norm(); }

/// 闭环模拟 lidar_loc 跟踪：guess = 上次输出 * LIO 增量；输出 = guess * exp(balance * 残差)
struct Sim {
    LocGuard guard;
    SE3 out;
    SE3 last_lo;
    bool first = true;
    double t = 0;
    double last_balance = kBalance;

    explicit Sim(LocGuard::Options o = LocGuard::Options()) : guard(o) {}

    /// lo: LIO 位姿；ndt_fn: 由 guess 给出 NDT 结果；odom: 腿式里程计（可选）
    template <typename F>
    void Step(const SE3& lo, F&& ndt_fn, const SE3* odom = nullptr) {
        if (first) {
            out = lo;
            last_lo = lo;
            first = false;
        }
        const SE3 guess = out * (last_lo.inverse() * lo);
        const SE3 ndt = ndt_fn(guess);
        LocGuard::Frame f;
        f.timestamp_ = t;
        f.lo_ = lo;
        f.guess_ = guess;
        f.ndt_ = ndt;
        f.score_ = 2.0;
        if (odom) {
            f.has_odom_ = true;
            f.odom_ = *odom;
        }
        last_balance = guard.Update(f, kBalance);
        SE3 rolled;
        out = guard.GetRollback(rolled) ? rolled : guess * SE3::exp((guess.inverse() * ndt).log() * last_balance);
        last_lo = lo;
        t += kDt;
    }
};

/// 走廊里 NDT 的表现：70% 落到 -0.55 m、30% 落到 +0.55 m 的局部最优（相对当前 guess）
struct CorridorNdt {
    std::mt19937 rng{42};
    SE3 operator()(const SE3& guess) {
        std::bernoulli_distribution back(0.7);
        return guess * Pose(back(rng) ? -0.55 : 0.55);
    }
};

}  // namespace

TEST(LocGuard, StationaryNoiseDoesNotTrigger) {
    Sim sim;
    std::mt19937 rng(1);
    std::normal_distribution<double> n(0, 0.03);
    for (int i = 0; i < 400; ++i) {
        const SE3 truth = Pose(0);
        sim.Step(truth, [&](const SE3&) { return Pose(n(rng), n(rng), n(rng) * 0.02); });
        EXPECT_EQ(sim.guard.GetStatus().state_, State::GOOD) << "frame " << i;
    }
    EXPECT_DOUBLE_EQ(sim.last_balance, kBalance);
}

TEST(LocGuard, NormalWalkingWithSmallLioDriftDoesNotTrigger) {
    // LIO 少算 1% 行程，NDT 正确（给真值附近），这是正常的地图修正，不应触发
    Sim sim;
    std::mt19937 rng(2);
    std::normal_distribution<double> n(0, 0.03);
    for (int i = 0; i < 400; ++i) {
        const double x = i * kDt * kSpeed;
        sim.Step(Pose(0.99 * x), [&](const SE3&) { return Pose(x + n(rng), n(rng)); });
        EXPECT_EQ(sim.guard.GetStatus().state_, State::GOOD) << "frame " << i;
    }
}

TEST(LocGuard, CorridorDriftReproducesWithoutGuard) {
    // 对照：不开守护时，同样的走廊行为会漂 >1.5 m（这就是 09-29 的现象）
    LocGuard::Options o;
    o.enabled_ = false;
    Sim sim(o);
    CorridorNdt ndt;
    SE3 truth;
    for (int i = 0; i < 300; ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, ndt);
    }
    EXPECT_GT(PosErr(sim.out, truth), 1.5);
}

TEST(LocGuard, CorridorDriftIsFrozenWithoutOdom) {
    Sim sim;
    CorridorNdt ndt;
    SE3 truth;
    bool triggered = false;
    for (int i = 0; i < 300; ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, ndt);
        triggered |= sim.guard.GetStatus().state_ == State::NDT_SUSPECT;
    }
    EXPECT_TRUE(triggered);
    EXPECT_LT(PosErr(sim.out, truth), kOneFrame);  // 触发时退回到漂移前，之后只跟 LIO
    EXPECT_EQ(sim.guard.GetStatus().state_, State::NDT_SUSPECT);
    EXPECT_TRUE(sim.guard.GetStatus().frozen_);
    EXPECT_DOUBLE_EQ(sim.last_balance, 0.0);
    EXPECT_TRUE(sim.guard.GetStatus().need_reloc_);  // 一直不恢复 -> 提示重定位
}

TEST(LocGuard, CorridorDriftIsFrozenWhenOdomAgreesWithLio) {
    Sim sim;
    CorridorNdt ndt;
    SE3 truth;
    for (int i = 0; i < 300; ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, ndt, &truth);  // 里程计 = 真值 = LIO
    }
    EXPECT_EQ(sim.guard.GetStatus().state_, State::NDT_SUSPECT);
    EXPECT_EQ(sim.guard.GetStatus().odom_agree_, 1);
    EXPECT_LT(PosErr(sim.out, truth), kOneFrame);
}

TEST(LocGuard, WarnOnlyModeDoesNotChangeBalance) {
    LocGuard::Options o;
    o.freeze_ = false;
    Sim sim(o);
    CorridorNdt ndt;
    bool triggered = false;
    for (int i = 0; i < 300; ++i) {
        sim.Step(Pose(i * kDt * kSpeed), ndt);
        triggered |= sim.guard.GetStatus().state_ == State::NDT_SUSPECT;
        EXPECT_DOUBLE_EQ(sim.last_balance, kBalance);
    }
    EXPECT_TRUE(triggered);
    EXPECT_EQ(sim.guard.GetStatus().rollbacks_, 0);  // 只告警时不动输出
}

TEST(LocGuard, LioDriftWithOdomDisagreeingIsNotFrozen) {
    // LIO 在走廊退化、只算出 50% 行程；NDT 给真值；里程计给真值 -> 应判 LIO_SUSPECT，继续让 NDT 修。
    // （里程计容差按实测放宽到 0.15 m + 15% 行程后，20% 这种量级的 LIO 误差要走 1.5 m 以上才分得出来）
    Sim sim;
    bool lio_suspect = false;
    for (int i = 0; i < 200; ++i) {
        const double x = i * kDt * kSpeed;
        const SE3 truth = Pose(x);
        sim.Step(Pose(0.5 * x), [&](const SE3&) { return truth; }, &truth);
        if (sim.guard.GetStatus().state_ == State::LIO_SUSPECT) {
            lio_suspect = true;
            EXPECT_DOUBLE_EQ(sim.last_balance, kBalance);
        }
        EXPECT_NE(sim.guard.GetStatus().state_, State::NDT_SUSPECT) << "frame " << i;
    }
    EXPECT_TRUE(lio_suspect);
}

TEST(LocGuard, RecoversWhenNdtAgreesAgain) {
    Sim sim;
    CorridorNdt ndt;
    SE3 truth;
    int i = 0;
    for (; i < 150; ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, ndt);
    }
    ASSERT_EQ(sim.guard.GetStatus().state_, State::NDT_SUSPECT);
    // 走出走廊：NDT 与 LIO 预测一致（冻结期间输出跟 LIO，所以 NDT 给 guess 附近）
    int recovered_at = -1;
    for (int k = 0; k < 20; ++k, ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, [](const SE3& guess) { return guess * Pose(0.02); });
        if (recovered_at < 0 && sim.guard.GetStatus().state_ == State::GOOD) recovered_at = k;
    }
    EXPECT_EQ(recovered_at, LocGuard::Options().recover_frames_ - 1);
    EXPECT_FALSE(sim.guard.GetStatus().need_reloc_);
    // 恢复后窗口已清空，不应被冻结前的漂移立即再次触发
    for (int k = 0; k < 60; ++k, ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, [](const SE3& guess) { return guess * Pose(0.02); });
        EXPECT_EQ(sim.guard.GetStatus().state_, State::GOOD);
    }
}

TEST(LocGuard, YawDriftTriggers) {
    // 航向被一点点拧偏（每帧 NDT 残差 3°，注入 0.3°），纯航向漂移也要能抓到
    Sim sim;
    bool triggered = false;
    for (int i = 0; i < 200; ++i) {
        sim.Step(Pose(i * kDt * kSpeed), [](const SE3& guess) { return guess * Pose(0, 0, 3.0 * M_PI / 180); });
        triggered |= sim.guard.GetStatus().state_ == State::NDT_SUSPECT;
    }
    EXPECT_TRUE(triggered);
    EXPECT_LT(sim.guard.GetStatus().drift_deg_, 10.0);
}

TEST(LocGuard, ResetClearsState) {
    Sim sim;
    CorridorNdt ndt;
    for (int i = 0; i < 150; ++i) sim.Step(Pose(i * kDt * kSpeed), ndt);
    ASSERT_NE(sim.guard.GetStatus().state_, State::GOOD);
    sim.guard.Reset();
    EXPECT_EQ(sim.guard.GetStatus().state_, State::GOOD);
    EXPECT_FALSE(sim.guard.GetStatus().need_reloc_);
}

namespace {
/// NDT 始终落在 guess 后方 0.55 m（09-30 导航时 NDT 沿走廊往原点拉）
SE3 PullBack(const SE3& guess) { return guess * Pose(-0.55); }
}  // namespace

TEST(LocGuard, RollbackUndoesInjectedDrift) {
    Sim sim;
    SE3 truth;
    int rolled_frames = 0;
    for (int i = 0; i < 100; ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, PullBack);
        SE3 p;
        rolled_frames += sim.guard.GetRollback(p) ? 1 : 0;
    }
    EXPECT_EQ(rolled_frames, 1);
    EXPECT_EQ(sim.guard.GetStatus().rollbacks_, 1);
    EXPECT_GT(sim.guard.GetStatus().last_rollback_m_, 0.10);
    EXPECT_TRUE(sim.guard.GetStatus().frozen_);
    EXPECT_LT(PosErr(sim.out, truth), kOneFrame);
}

TEST(LocGuard, FreezeWithoutRollbackStopsWhereItWasPulled) {
    // 对照：只冻结不退回，误差停在阈值附近
    LocGuard::Options o;
    o.rollback_ = false;
    Sim sim(o);
    SE3 truth;
    for (int i = 0; i < 100; ++i) {
        truth = Pose(i * kDt * kSpeed);
        sim.Step(truth, PullBack);
    }
    EXPECT_EQ(sim.guard.GetStatus().state_, State::NDT_SUSPECT);
    EXPECT_GT(PosErr(sim.out, truth), 0.10);
    EXPECT_LT(PosErr(sim.out, truth), 0.35);
}

TEST(LocGuard, StationarySlowCreepIsCaught) {
    // 09-30 14:51：静止时 NDT 突然落到 0.28 m 外并一直待在那，每帧注入 10%，40 s 拉走 0.19 m
    Sim sim;
    const SE3 truth = Pose(0);
    for (int i = 0; i < 40; ++i) sim.Step(truth, [&](const SE3&) { return truth; });
    for (int i = 0; i < 200; ++i) sim.Step(truth, [](const SE3&) { return Pose(0.28); });
    EXPECT_EQ(sim.guard.GetStatus().state_, State::NDT_SUSPECT);
    EXPECT_LT(PosErr(sim.out, truth), 0.02);
}

TEST(LocGuard, StationarySlowCreepMissedByOldThresholds) {
    // 对照：09-29 的参数（20 s 窗口、0.30 m、无静止阈值）抓不到，会被拉满 0.28 m
    LocGuard::Options o;
    o.window_sec_ = 20.0;
    o.max_drift_m_ = 0.30;
    o.max_drift_static_m_ = 0.30;
    Sim sim(o);
    const SE3 truth = Pose(0);
    for (int i = 0; i < 40; ++i) sim.Step(truth, [&](const SE3&) { return truth; });
    for (int i = 0; i < 200; ++i) sim.Step(truth, [](const SE3&) { return Pose(0.28); });
    EXPECT_EQ(sim.guard.GetStatus().state_, State::GOOD);
    EXPECT_GT(PosErr(sim.out, truth), 0.25);
}

TEST(LocGuard, OdomSpikesDuringTurnsDoNotFlap) {
    // 导航转身时里程计单帧误差大：每 3 帧有 1 帧横向偏 1 m。不应因此在 NDT_SUSPECT / LIO_SUSPECT 间来回切
    Sim sim;
    CorridorNdt ndt;
    SE3 truth;
    int transitions = 0;
    State prev = State::GOOD;
    for (int i = 0; i < 300; ++i) {
        truth = Pose(i * kDt * kSpeed);
        const SE3 odom = (i % 3 == 0) ? truth * Pose(0, 1.0) : truth;
        sim.Step(truth, ndt, &odom);
        const State s = sim.guard.GetStatus().state_;
        transitions += s != prev ? 1 : 0;
        prev = s;
    }
    EXPECT_EQ(prev, State::NDT_SUSPECT);
    EXPECT_EQ(transitions, 1);
    EXPECT_LT(PosErr(sim.out, truth), kOneFrame);
}
