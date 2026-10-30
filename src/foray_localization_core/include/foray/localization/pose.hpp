// Copyright 2026 ZQU-Foray
//
// L3 定位层核心值类型。此文件不依赖 ROS，可被离线工具与单测直接复用。

#ifndef FORAY_LOCALIZATION_POSE_HPP_
#define FORAY_LOCALIZATION_POSE_HPP_

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace foray::localization {

/// 时间戳统一使用 ROS 的纳秒整数，避免系统时钟/稳态时钟混用。
using Timestamp = std::int64_t;

/// 一帧位姿及其坐标系。变换语义为 `parent_frame -> child_frame`。
struct Pose3D {
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    std::string parent_frame;
    std::string child_frame;
};

/// 位姿来源。下游只依赖 `LocalizationState`，不按来源分支。
enum class PoseSource : std::uint8_t {
    kUnknown = 0,
    kSentryLidar = 1,
    kExternalOdom = 2,
};

/// 定位健康状态。数值与 `LocalizationState.msg` 的常量保持一致。
enum class HealthState : std::uint8_t {
    kInitializing = 0,
    kOk = 1,
    kDegraded = 2,
    kLost = 3,
};

/// 返回稳定的英文小写标识，用于日志与诊断。
const char* to_string(HealthState state);

/// 返回稳定的英文小写标识，用于日志与诊断。
const char* to_string(PoseSource source);

/// 一次配准的观测结果。`valid=false` 表示求解失败/被拒绝，不代表位姿为 0。
struct RegistrationSample {
    Timestamp stamp_ns{0};
    bool valid{false};
    /// 内点到先验地图的平均距离（米）。由配准后端按同一口径计算。
    double mean_residual_m{0.0};
    /// 有有效对应的点占比，范围 [0, 1]。
    double inlier_ratio{0.0};
    /// 后端原始 fitness（越小越好）；仅用于日志，不参与状态判定。
    double fitness{0.0};
    /// 参与配准的源点总数；与 `inlier_ratio` 相除可还原内点绝对数。
    /// 点数太少时即使内点率 100% 也不足以约束 6 个自由度（decision.md D23）。
    std::size_t source_points{0};
    /// 点面残差信息矩阵的条件数（最大 / 最小特征值）。越大说明越接近退化。
    double condition_number{0.0};
    /// 不可观的自由度个数（0..6）。大于 0 表示这一帧只约束了一部分方向。
    int unobservable_count{0};
};

/// 位姿源选择策略。
enum class PoseSourcePolicy : std::uint8_t {
    kAuto = 0,
    kLidar = 1,
    kExternal = 2,
};

} // namespace foray::localization

#endif // FORAY_LOCALIZATION_POSE_HPP_
