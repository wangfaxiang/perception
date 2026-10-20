/******************************************************************************
 * ThreadPool.h —— 通用线程池（仅依赖 C++11 标准库，无第三方依赖）
 *
 * 主要特性（满足需求第 5 步）：
 *   - 可配置线程数（0 表示使用 std::thread::hardware_concurrency()）。
 *   - 任务队列可设上限，队列满时支持 4 种背压策略：
 *       Block(阻塞) / DiscardNew(丢新) / DiscardOld(丢旧) / Throw(抛异常)。
 *   - 线程复用，避免频繁创建 / 销毁。
 *   - enqueue 返回 std::future，可获取结果与捕获任务异常。
 *   - 工作线程捕获所有异常，单任务异常不会导致线程退出。
 *   - 支持运行时动态调整线程数 resize()（增 / 减线程，安全且不崩溃）。
 *   - 支持优雅关闭 shutdown()（等待已提交任务完成）与 shutdownNow()（丢弃排队任务）。
 *   - 提供状态监控 stats()（线程数、活跃线程、排队任务数）。
 *   - 兼容旧接口：ThreadPool(size_t) 构造 + enqueue(...)，C++14/17 均可编译。
 *
 * 说明：本实现为头文件内联实现，所有非模板成员均以 inline 修饰以避免多重定义。
 *****************************************************************************/
#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <atomic>
#include <string>
#include <algorithm>

// std::result_of 在 C++17 中被弃用、C++20 中被移除，这里按标准版本选择特征类，避免告警。
#if defined(__cpp_lib_is_invocable) || (defined(__cplusplus) && __cplusplus >= 201703L)
    #define LS_THREADPOOL_RESULT_OF(F, ...) typename std::invoke_result<F, ##__VA_ARGS__>::type
#else
    #define LS_THREADPOOL_RESULT_OF(F, ...) typename std::result_of<F(__VA_ARGS__)>::type
#endif

class ThreadPool {
public:
    // 队列满时的背压策略
    enum class OverflowPolicy {
        Block,       // 阻塞调用者直到队列有空位（默认）
        DiscardNew,  // 丢弃新任务（返回的 future 处于 broken_promise 状态）
        DiscardOld,  // 丢弃队首最旧任务后入队新任务
        Throw        // 抛出 std::runtime_error
    };

    // 线程池运行状态快照（用于监控）
    struct Stats {
        size_t thread_count;    // 目标工作线程数
        size_t active_threads;  // 正在执行任务的线程数
        size_t queued_tasks;    // 排队等待的任务数
        size_t max_queue_size;  // 队列上限（0 表示无上限）
    };

    // threads==0 时使用硬件并发数；max_queue_size==0 表示队列无上限
    explicit ThreadPool(size_t threads = 0,
                        size_t max_queue_size = 0,
                        OverflowPolicy policy = OverflowPolicy::Block);
    ~ThreadPool();

    ThreadPool(const ThreadPool &) = delete;
    ThreadPool &operator=(const ThreadPool &) = delete;

    // 提交任务，返回 std::future 以获取结果 / 异常
    template<class F, class... Args>
    auto enqueue(F &&f, Args &&... args) -> std::future<LS_THREADPOOL_RESULT_OF(F, Args...)>;

    void resize(size_t threads);   // 动态调整线程数（最小为 1）
    void shutdown();               // 优雅关闭：等待已排队任务全部完成
    void shutdownNow();            // 立即关闭：丢弃排队任务，仅等待在执行任务
    void waitIdle();               // 阻塞直到队列为空且无活跃任务

    size_t threadCount() const;
    size_t activeThreads() const;
    size_t queuedTasks() const;
    Stats  stats() const;

    void setOverflowPolicy(OverflowPolicy p);
    void setMaxQueueSize(size_t n);
    // 注入日志回调（默认无）；丢弃任务等事件会通过它输出警告，使线程池与具体日志框架解耦
    void setLogger(std::function<void(const std::string &)> logger);

private:
    void launchWorkers(size_t from, size_t to);
    void workerLoop(size_t id);
    void warn(const std::string &msg);

    std::vector<std::thread> workers_;
    std::vector<std::thread> retired_workers_;  // resize 缩容时移出、延迟到 shutdown 统一回收
    std::queue<std::function<void()>> tasks_;

    mutable std::mutex mutex_;
    std::condition_variable cv_task_;  // 唤醒工作线程
    std::condition_variable cv_room_;  // 唤醒等待空位的生产者
    std::condition_variable cv_idle_;  // 唤醒 waitIdle()

    size_t target_threads_;
    size_t max_queue_size_;
    OverflowPolicy policy_;
    std::atomic<size_t> active_;
    bool stop_;

    std::mutex resize_mutex_;          // 串行化 resize/shutdown
    std::function<void(const std::string &)> logger_;
    std::mutex logger_mutex_;
};

inline ThreadPool::ThreadPool(size_t threads, size_t max_queue_size, OverflowPolicy policy)
        : target_threads_(0), max_queue_size_(max_queue_size), policy_(policy), active_(0), stop_(false) {
    if (threads == 0) {
        threads = static_cast<size_t>(std::thread::hardware_concurrency());
        if (threads == 0) threads = 1;  // hardware_concurrency 可能返回 0
    }
    target_threads_ = threads;
    launchWorkers(0, threads);
}

inline void ThreadPool::launchWorkers(size_t from, size_t to) {
    for (size_t i = from; i < to; ++i) {
        workers_.emplace_back([this, i] { this->workerLoop(i); });
    }
}

inline void ThreadPool::workerLoop(size_t id) {
    for (;;) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_task_.wait(lock, [this, id] {
                return stop_ || id >= target_threads_ || !tasks_.empty();
            });
            if (stop_ && tasks_.empty()) return;   // 关闭且队列已清空
            if (id >= target_threads_) return;      // 被 resize 缩减掉的线程及时退出
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        cv_room_.notify_one();   // 取走一个任务，通知可能在等待空位的生产者

        active_.fetch_add(1, std::memory_order_relaxed);
        // packaged_task 会把异常存入 future，这里再加一层保护，确保任何情况下工作线程都不退出
        try {
            task();
        } catch (const std::exception &e) {
            warn(std::string("ThreadPool task threw exception: ") + e.what());
        } catch (...) {
            warn("ThreadPool task threw unknown exception");
        }
        active_.fetch_sub(1, std::memory_order_relaxed);

        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (tasks_.empty() && active_.load(std::memory_order_relaxed) == 0) {
                cv_idle_.notify_all();
            }
        }
    }
}

template<class F, class... Args>
auto ThreadPool::enqueue(F &&f, Args &&... args) -> std::future<LS_THREADPOOL_RESULT_OF(F, Args...)> {
    using return_type = LS_THREADPOOL_RESULT_OF(F, Args...);

    auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) throw std::runtime_error("enqueue on stopped ThreadPool");

        if (max_queue_size_ != 0 && tasks_.size() >= max_queue_size_) {
            switch (policy_) {
                case OverflowPolicy::Block:
                    cv_room_.wait(lock, [this] {
                        return stop_ || tasks_.size() < max_queue_size_;
                    });
                    if (stop_) throw std::runtime_error("enqueue on stopped ThreadPool");
                    break;
                case OverflowPolicy::Throw:
                    throw std::runtime_error("ThreadPool task queue is full");
                case OverflowPolicy::DiscardNew:
                    warn("ThreadPool queue full, discarding NEW task");
                    return res;  // task 析构后 future 处于 broken_promise，调用方 get() 可感知
                case OverflowPolicy::DiscardOld:
                    if (!tasks_.empty()) tasks_.pop();
                    warn("ThreadPool queue full, discarding OLD task");
                    break;
            }
        }
        tasks_.emplace([task]() { (*task)(); });
    }
    cv_task_.notify_one();
    return res;
}

inline void ThreadPool::resize(size_t threads) {
    if (threads < 1) threads = 1;
    std::lock_guard<std::mutex> rz(resize_mutex_);

    size_t old_count;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) return;
        old_count = target_threads_;
        target_threads_ = threads;
    }

    if (threads > old_count) {
        launchWorkers(old_count, threads);            // 扩容：追加新线程
    } else if (threads < old_count) {
        cv_task_.notify_all();                        // 唤醒，使 id>=target 的线程退出
        // 不在此处 join：被缩减的线程可能正在执行长任务（如永不返回的轮询任务），
        // 立即 join 会阻塞 resize。改为移入退休列表，统一在 shutdown() 时回收，保证 resize 不阻塞。
        for (size_t i = threads; i < workers_.size(); ++i) {
            retired_workers_.emplace_back(std::move(workers_[i]));
        }
        workers_.erase(workers_.begin() + threads, workers_.end());
    }
}

inline void ThreadPool::shutdown() {
    std::lock_guard<std::mutex> rz(resize_mutex_);
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) return;
        stop_ = true;
    }
    cv_task_.notify_all();
    cv_room_.notify_all();
    for (std::thread &w : workers_) {
        if (w.joinable()) w.join();
    }
    for (std::thread &w : retired_workers_) {
        if (w.joinable()) w.join();
    }
    workers_.clear();
    retired_workers_.clear();
}

inline void ThreadPool::shutdownNow() {
    std::lock_guard<std::mutex> rz(resize_mutex_);
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) return;
        stop_ = true;
        std::queue<std::function<void()>> empty;
        std::swap(tasks_, empty);   // 丢弃所有排队任务
    }
    cv_task_.notify_all();
    cv_room_.notify_all();
    for (std::thread &w : workers_) {
        if (w.joinable()) w.join();
    }
    for (std::thread &w : retired_workers_) {
        if (w.joinable()) w.join();
    }
    workers_.clear();
    retired_workers_.clear();
}

inline void ThreadPool::waitIdle() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_idle_.wait(lock, [this] {
        return tasks_.empty() && active_.load(std::memory_order_relaxed) == 0;
    });
}

inline size_t ThreadPool::threadCount() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return target_threads_;
}

inline size_t ThreadPool::activeThreads() const {
    return active_.load(std::memory_order_relaxed);
}

inline size_t ThreadPool::queuedTasks() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return tasks_.size();
}

inline ThreadPool::Stats ThreadPool::stats() const {
    std::unique_lock<std::mutex> lock(mutex_);
    Stats s;
    s.thread_count   = target_threads_;
    s.active_threads = active_.load(std::memory_order_relaxed);
    s.queued_tasks   = tasks_.size();
    s.max_queue_size = max_queue_size_;
    return s;
}

inline void ThreadPool::setOverflowPolicy(OverflowPolicy p) {
    std::unique_lock<std::mutex> lock(mutex_);
    policy_ = p;
}

inline void ThreadPool::setMaxQueueSize(size_t n) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        max_queue_size_ = n;
    }
    cv_room_.notify_all();  // 放宽上限后唤醒可能阻塞的生产者
}

inline void ThreadPool::setLogger(std::function<void(const std::string &)> logger) {
    std::lock_guard<std::mutex> lock(logger_mutex_);
    logger_ = std::move(logger);
}

inline void ThreadPool::warn(const std::string &msg) {
    std::function<void(const std::string &)> logger;
    {
        std::lock_guard<std::mutex> lock(logger_mutex_);
        logger = logger_;
    }
    if (logger) logger(msg);
}

inline ThreadPool::~ThreadPool() {
    shutdown();
}

#endif  // THREAD_POOL_H
