// Copyright 2026 ZQU-Foray
//
// 融合层 6 维误差状态 EKF：只平滑 map -> odom，不改变里程计本身（decision.md D24）。
// 配准是 5 Hz 的离散量测，两次量测之间位姿完全靠里程计外推；滤波器的作用是把
// 单帧配准的抖动压下去，并让几何上不可观的方向少听配准的话。

#ifndef FORAY_LOCALIZATION_POSE_EKF_HPP_
#define FORAY_LOCALIZATION_POSE_EKF_HPP_

#include <Eigen/Geometry>

#include <array>

namespace foray::localization {

/// 位姿自由度：平移 xyz 在前、旋转 rx/ry/rz 在后。
inline constexpr int kPoseDofCount = 6;

/// 融合层参数。平移单位米，旋转单位弧度。
struct PoseEkfConfig {
    /// 总开关。false 时 update() 直接透传量测，与引入滤波前逐字节等价。
    bool enabled{false};
    /// 过程噪声标准差（每秒）：常值预测下协方差按 dt 线性增长。
    double process_noise_translation_m{0.05};
    double process_noise_rotation_rad{0.02};
    /// 量测噪声标准差基准。0.25 m 体素先验图的静止配准误差实测约 0.05 m（D22/D26）。
    double measurement_noise_translation_m{0.05};
    double measurement_noise_rotation_rad{0.01};
    /// 不可观方向的量测噪声倍数。按**标准差**倍数解释，方差按平方放大：
    /// 取 10 表示该方向的卡尔曼增益只有约 1 %——接近不信，但不硬切断（硬切断会让
    /// 恰好成立的观测也进不来，D23 已否定「整体拒绝」的做法）。
    ///
    /// 该倍数单独用不足以保证上面的 1 %（审计 L12）：协方差随外推时间单调增长，
    /// 分子追上来之后增益会自己爬回去（100 s 外推后实测 0.5）。所以不可观方向的
    /// 增益额外锁死在 `kMaxUnobservableGain` 上——倍数决定「多少」，锁决定「最多多少」。
    double unobservable_noise_scale{10.0};
};

/// 不可观方向的增益上限：README 参数表承诺「该方向仅约 1 % 权重」，这里把它写成硬上限，
/// 否则协方差增长会让增益重新爬回 0.5（审计 L12）。
inline constexpr double kMaxUnobservableGain = 0.01;

/// 协方差对角上限：只防外推无限进行时溢出成 inf/NaN，不是物理断言。
/// 平移 σ = 1 m、旋转 σ = 0.5 rad 已远超「1 s 无配准即 LOST」能容忍的量级。
inline constexpr double kMaxVarianceTranslationM2 = 1.0;
inline constexpr double kMaxVarianceRotationRad2 = 0.25;

/// 误差状态 EKF。非线程安全，由节点在单线程内调用。
class PoseEkf {
  public:
    explicit PoseEkf(PoseEkfConfig config = {});

    /// 硬切：直接跳到给定位姿并重置协方差。重定位与滤波器首次启动都走这里。
    /// 给定位姿非有限时直接忽略（保持原状态）。
    void reset(const Eigen::Isometry3d& estimate);

    /// 常值预测：估计值不变，协方差按 `dt_sec` 增长。`dt_sec <= 0` 时不做任何事。
    const Eigen::Isometry3d& predict(double dt_sec);

    /// 量测更新。`observable` 为空表示六个方向都视为可观；
    /// 否则未标记的方向按 `unobservable_noise_scale` 放大量测方差，且增益不超过
    /// `kMaxUnobservableGain`。
    ///
    /// 非有限（NaN/inf）的量测会被**丢弃**：估计值与协方差保持不变，也不触发 reset——
    /// 丢一帧比让 NaN 污染此后每一帧便宜得多（decision.md D47）。
    const Eigen::Isometry3d& update(const Eigen::Isometry3d& measurement,
                                    const std::array<bool, kPoseDofCount>* observable = nullptr);

    const Eigen::Isometry3d& estimate() const { return estimate_; }

    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount>& covariance() const {
        return covariance_;
    }

    bool enabled() const { return config_.enabled; }

  private:
    PoseEkfConfig config_;
    Eigen::Isometry3d estimate_{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> covariance_{
        Eigen::Matrix<double, kPoseDofCount, kPoseDofCount>::Identity()};
};

} // namespace foray::localization

#endif // FORAY_LOCALIZATION_POSE_EKF_HPP_
