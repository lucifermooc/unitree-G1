//
// Created by xiang on 25-3-18.
//

#include <execinfo.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <unistd.h>
#include <csignal>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

namespace {
/// 致命信号（SIGSEGV 等）时把原始调用栈写到 stderr（launch_manager 日志可见），再按默认动作重新触发，
/// 退出码 / apport core 不变。输出形如 liblightning.libs.so(+0x1234)，用 addr2line -e <so> 0x1234 解析。
/// 只用 async-signal-safe 的 write/backtrace_symbols_fd（backtrace 已在启动时预热，避免信号里首次 dlopen）。
void FatalSignalHandler(int sig) {
    static const char kMsg[] = "\n*** run_loc_online caught fatal signal, raw backtrace:\n";
    (void)!write(STDERR_FILENO, kMsg, sizeof(kMsg) - 1);
    void* frames[64];
    const int n = backtrace(frames, 64);
    backtrace_symbols_fd(frames, n, STDERR_FILENO);
    std::signal(sig, SIG_DFL);
    raise(sig);
}

void InstallFatalSignalBacktrace() {
    void* warmup[1];
    backtrace(warmup, 1);  // 预加载 libgcc_s，信号处理里不再分配内存
    static char alt_stack[64 * 1024];
    stack_t ss{};
    ss.ss_sp = alt_stack;
    ss.ss_size = sizeof(alt_stack);
    sigaltstack(&ss, nullptr);  // 主线程栈溢出时也能打印（其他线程在自身栈上处理）
    struct sigaction sa{};
    sa.sa_handler = FatalSignalHandler;
    sa.sa_flags = SA_ONSTACK;
    sigemptyset(&sa.sa_mask);
    for (int sig : {SIGSEGV, SIGBUS, SIGFPE, SIGILL}) {
        sigaction(sig, &sa, nullptr);
    }
}
}  // namespace

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::ERROR;  // 终端只打 ERROR，完整日志见 /tmp/<程序名>.INFO
    // glog 默认缓冲 INFO（最长 30 s 才落盘），进程被信号杀死时缓冲丢失——
    // 2026-09-19 10:03:47 的段错误日志只剩 loc_system.cc:28 一行就是这个原因。逐条落盘开销可忽略（~100 行/s）。
    FLAGS_logbuflevel = -1;
    InstallFatalSignalBacktrace();

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    rclcpp::init(argc, argv);

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    /// 默认起点开始定位
    loc.SetInitPose(SE3());
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}