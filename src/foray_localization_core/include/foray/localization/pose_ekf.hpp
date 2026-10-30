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
    /// 取 10 表示该方向只有约 1 % 的权重——接近不信，但不硬切断（硬切断会让
    /// 恰好成立的观测也进不来，D23 已否定「整体拒绝」的做法）。
    double unobservable_noise_scale{10.0};
};

/// 误差状态 EKF。非线程安全，由节点在单线程内调用。
class PoseEkf {
  public:
    explicit PoseEkf(PoseEkfConfig config = {});

    /// 硬切：直接跳到给定位姿并重置协方差。重定位与滤波器首次启动都走这里。
    void reset(const Eigen::Isometry3d& estimate);

    /// 常值预测：估计值不变，协方差按 `dt_sec` 增长。`dt_sec <= 0` 时不做任何事。
    const Eigen::Isometry3d& predict(double dt_sec);

    /// 量测更新。`observable` 为空表示六个方向都视为可观；
    /// 否则未标记的方向按 `unobservable_noise_scale` 放大量测方差。
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
