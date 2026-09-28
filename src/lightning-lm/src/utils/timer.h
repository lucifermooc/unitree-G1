//
// Created by gx on 23-11-23.
//
#pragma once

#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <glog/logging.h>

namespace lightning {

/// 统计时间工具
class Timer {
   public:
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& name, double time_usage) {
            func_name_ = name;
            time_usage_in_ms_.emplace_back(time_usage);
        }
        std::string func_name_;
        std::deque<double> time_usage_in_ms_;
    };

    /**
     * 评价并记录函数用时
     * @tparam F
     * @param func
     * @param func_name
     */
    template <class F>
    static void Evaluate(F&& func, const std::string& func_name, bool print = false) {
        auto t1 = std::chrono::steady_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

        // records_ 是进程级全局 std::map，定位在线模式下 ROS 回调线程（"Proc Lidar"）与
        // LIO 异步线程（"Preprocess (Standard)" 等）会同时首次插入新 key，无锁时红黑树被并发改写，
        // 启动瞬间偶发 SIGSEGV。只锁记账部分，不锁被测函数本身。
        std::lock_guard<std::mutex> lock(mutex_);
        if (records_.find(func_name) != records_.end()) {
            records_[func_name].time_usage_in_ms_.emplace_back(time_used);
            while (records_[func_name].time_usage_in_ms_.size() > 2000) {
                records_[func_name].time_usage_in_ms_.pop_front();
            }
        } else {
            records_.insert({func_name, TimerRecord(func_name, time_used)});
        }

        if (print) {
            LOG(INFO) << "func <" << func_name << "> timer: " << time_used << " ms";
        }
    }

    /// 打印记录的所有耗时
    static void PrintAll();

    /// 写入文件，方便作图分析
    static void DumpIntoFile(const std::string& file_name);

    /// 获取某个函数的平均执行时间
    static double GetMeanTime(const std::string& func_name);

    /// 清理记录
    static void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        records_.clear();
    }

   private:
    static std::map<std::string, TimerRecord> records_;
    inline static std::mutex mutex_;  // 保护 records_（多线程调用 Evaluate）
};
}  // namespace lightning
