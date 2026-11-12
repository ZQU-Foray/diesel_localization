// Copyright 2026 ZQU-Foray
//
// 定位健康状态机。安全关键：任何失效都必须显式进入 DEGRADED/LOST，
// 不允许下游静默消费“看起来还在更新”的位姿。

#ifndef DIESEL_LOCALIZATION_HEALTH_HPP_
#define DIESEL_LOCALIZATION_HEALTH_HPP_

#include "diesel/localization/pose.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace diesel::localization {

/// 健康判定阈值。所有距离单位为米，时间单位为秒。
/// 初值来自 2027 赛季 12 m x 8 m 场地 + MID360 的调试经验，
/// 赛前必须用录包标定后写入 bringup 配置（见 decision.md D3/D9）。
struct HealthConfig {
    /// 进入 OK 的平均残差上限。
    /// 不能低于先验地图自身的离散度：地图按 0.25 m 体素降采样后，
    /// 即使配准完全收敛，最近邻残差也有约 0.10 m（仿真实测 0.1015–0.1026 m）。
    /// 旧默认值 0.10 恰好在基线之下，会把完美配准判成不合格（decision.md D22）。
    double ok_mean_residual_m{0.15};
    /// DEGRADED 可接受的平均残差上限；超过即视为不可用。
    double degraded_mean_residual_m{0.30};
    /// 进入 OK 的内点率下限。
    double ok_inlier_ratio{0.55};
    /// DEGRADED 的内点率下限。
    double degraded_inlier_ratio{0.35};
    /// 无有效配准超过该时长进入 LOST。
    double lost_timeout_sec{1.0};
    /// 里程计断流超过该时长直接进入 LOST。
    double odom_timeout_sec{0.5};
    /// 点云断流超过该时长即视为无有效配准（由节点喂入不可用样本）。
    /// 取 0.2 s = README「点云断流 ≤ 200 ms 进 DEGRADED」的验收上限，
    /// 同时高于 MID360 的 10 Hz 帧间隔（100 ms），避免正常周期抖动误判。
    double lidar_timeout_sec{0.2};
    /// 从 INITIALIZING 进入 OK 所需的连续好帧数。
    int warmup_frames{3};
    /// 从 DEGRADED 恢复 OK 所需的连续好帧数。
    int recover_frames{3};
    /// 单帧残差/内点率越界后进入 DEGRADED 的保持时间（用于抑制抖动）。
    double degraded_hold_sec{0.0};
    /// 单帧源点数下限。低于它时即使残差与内点率都合格，也不足以约束 6 个自由度
    /// （decision.md D23）。0 表示不设下限，即保持改造前行为。
    std::size_t min_source_points{0};
    /// 允许的不可观自由度上限（0..6）。超过即降级。默认 6 表示不启用该项。
    int max_unobservable_dofs{6};
};

/// 对外发布的状态快照。
struct HealthStatus {
    HealthState state{HealthState::kInitializing};
    /// 0..1，越大越可信；LOST/INITIALIZING 为 0。
    double confidence{0.0};
    /// 距最近一次有效配准的秒数；从未有效时为 +inf。
    double age_sec{0.0};
    /// 当前状态已持续的秒数。
    double state_age_sec{0.0};
    /// 人类可读原因，写入日志与诊断。
    std::string reason;
};

/// 有状态健康监视器。非线程安全，由节点在单线程内调用。
class HealthMonitor {
  public:
    explicit HealthMonitor(HealthConfig config = {});

    /// 输入最近一帧配准结果、当前时间与最近一次里程计时间。
    /// `last_odom_ns == 0` 表示尚未收到里程计。
    /// `odom_backlog_sec` 是最近一个窗口内「里程计帧在队列里等了多久」的最大值：
    /// 它把「根本没有里程计」和「里程计来了但 executor 忙着配准、没人处理」区分开，
    /// 否则两种完全不同的故障会共用同一句 reason（decision.md D30）。
    HealthStatus update(const RegistrationSample& sample, Timestamp now_ns, Timestamp last_odom_ns,
                        double odom_backlog_sec = 0.0);

    /// 显式进入 LOST，幂等。用于没有配准帧可喂的失效（地图没加载成功、TF 查不到等）：
    /// 这类故障在 update() 里永远进不来，健康状态会永久停在 INITIALIZING。
    /// 若此时还没成功过（`ever_ok() == false`），一旦合格帧开始到来，update() 会按
    /// `warmup_frames` 自恢复——否则「启动瞬间就缺件」会变成永久失锁（decision.md D57）。
    HealthStatus enter_lost(const std::string& reason, Timestamp now_ns);

    /// 显式重定位：回到 INITIALIZING，清空去抖计数。
    /// 同时清掉 `ever_ok_`：重定位是一次新的启动，新的启动期同样允许自恢复（decision.md D57）。
    void reset();

    const HealthConfig& config() const { return config_; }
    HealthState state() const { return state_; }
    /// 是否成功进入过 OK。`false` 表示这个监视器还处在「启动期」。
    bool ever_ok() const { return ever_ok_; }

  private:
    void transition(HealthState next, const std::string& reason, Timestamp now_ns);
    HealthStatus snapshot(Timestamp now_ns) const;
    double compute_confidence(const RegistrationSample& sample) const;

    HealthConfig config_;
    HealthState state_{HealthState::kInitializing};
    Timestamp state_since_ns_{0};
    Timestamp start_ns_{0};
    Timestamp last_valid_ns_{0};
    Timestamp first_bad_ns_{0};
    int good_frames_{0};
    /// 是否曾经进入过 OK。它把两种 LOST 分开（decision.md D57）：
    /// 从未成功过 = 缺的是「还没跑起来」（TF 晚到、点云晚到、地图晚加载），
    /// 允许按 `warmup_frames` 连续好帧自恢复；成功过又丢 = 真的跑丢了，只能显式 Relocalize。
    /// 旧行为是一律粘滞，于是启动早期几秒的缺帧会变成永久失锁。
    bool ever_ok_{false};
    double last_confidence_{0.0};
    std::string reason_{"waiting for first registration"};
};

/// 选择位姿源。`auto` 优先激光配准，无激光时回退外部位姿。
PoseSource select_pose_source(PoseSourcePolicy policy, bool lidar_available,
                              bool external_available);

} // namespace diesel::localization

#endif // DIESEL_LOCALIZATION_HEALTH_HPP_
