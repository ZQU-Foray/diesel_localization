// Copyright 2026 ZQU-Foray
//
// 融合层 EKF 实现（decision.md D24）。误差状态取「左乘小增量」：
// estimate <- exp(dx) * estimate，残差 r = log(measurement * estimate^-1)。
// 这样旋转力臂与平移分量都在 map 系下，和配准输出的语义一致。

#include "foray/localization/pose_ekf.hpp"

#include <algorithm>
#include <cmath>

namespace foray::localization {
namespace {

// 位姿的每个分量都必须是有限数：配准失败、上游里程计被除零污染、TF 里混进 NaN
// 都会走到这里，而一旦 NaN 进了估计值或协方差，之后每一帧都被它污染（审计 L13）。
bool is_finite(const Eigen::Isometry3d& pose) {
    if (!pose.matrix().allFinite()) {
        return false;
    }
    const Eigen::AngleAxisd rotation(pose.linear());
    return std::isfinite(rotation.angle());
}

Eigen::Matrix<double, kPoseDofCount, 1> to_delta(const Eigen::Isometry3d& delta) {
    Eigen::Matrix<double, kPoseDofCount, 1> vector;
    vector.head<3>() = delta.translation();
    const Eigen::AngleAxisd rotation(delta.linear());
    vector.tail<3>() = rotation.angle() * rotation.axis();
    return vector;
}

Eigen::Isometry3d from_delta(const Eigen::Matrix<double, kPoseDofCount, 1>& vector) {
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.translation() = vector.head<3>();
    const Eigen::Vector3d omega = vector.tail<3>();
    const double angle = omega.norm();
    // 角度为 0 时轴没有定义，此时旋转就是单位阵，直接跳过。
    if (angle > 1e-12) {
        delta.linear() = Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
    }
    return delta;
}

} // namespace

PoseEkf::PoseEkf(PoseEkfConfig config) : config_(config) {}

void PoseEkf::reset(const Eigen::Isometry3d& estimate) {
    if (!is_finite(estimate)) {
        return; // 硬切也不能把 NaN 装进状态：保持上一次的估计与协方差。
    }
    estimate_ = estimate;
    // 硬切之后的不确定度按「等于一次量测的不确定度」起步：既不把刚注入的位姿当成
    // 绝对真值（那会让后续量测全被忽略），也不给它一个 1 m 量级的不确定度
    // （那会让第一帧量测就整体推翻重定位结果）。
    covariance_ = Eigen::Matrix<double, kPoseDofCount, kPoseDofCount>::Zero();
    const double translation_var = config_.measurement_noise_translation_m *
                                   config_.measurement_noise_translation_m;
    const double rotation_var =
        config_.measurement_noise_rotation_rad * config_.measurement_noise_rotation_rad;
    for (int i = 0; i < 3; ++i) {
        covariance_(i, i) = translation_var;
    }
    for (int i = 3; i < kPoseDofCount; ++i) {
        covariance_(i, i) = rotation_var;
    }
}

const Eigen::Isometry3d& PoseEkf::predict(double dt_sec) {
    // `dt_sec <= 0.0` 挡不住 NaN（NaN 的每次比较都为假），必须单独判有限性。
    if (!config_.enabled || !std::isfinite(dt_sec) || dt_sec <= 0.0) {
        return estimate_;
    }
    // 常值预测：map -> odom 在两帧配准之间就靠里程计维持，估计值不动，
    // 只有不确定性随时间长大——这才是「下一次量测该被信多少」的依据。
    const double translation_var = config_.process_noise_translation_m *
                                   config_.process_noise_translation_m * dt_sec;
    const double rotation_var =
        config_.process_noise_rotation_rad * config_.process_noise_rotation_rad * dt_sec;
    for (int i = 0; i < 3; ++i) {
        // 上限只管「别溢出」：外推到天荒地老时 P 会一直线性增长，而增益 P/(P+R) 跟着
        // 涨到 1，等于把量测当唯一真值（审计 L12）。物理上 1 s 没配准就该 LOST 了，
        // 所以 1 m² / 0.25 rad² 这两条线永远不会在正常流程里被踩到。
        covariance_(i, i) =
            std::min(covariance_(i, i) + translation_var, kMaxVarianceTranslationM2);
    }
    for (int i = 3; i < kPoseDofCount; ++i) {
        covariance_(i, i) = std::min(covariance_(i, i) + rotation_var, kMaxVarianceRotationRad2);
    }
    return estimate_;
}

const Eigen::Isometry3d& PoseEkf::update(const Eigen::Isometry3d& measurement,
                                         const std::array<bool, kPoseDofCount>* observable) {
    // 关闭时逐字节透传：这是「默认配置与改造前等价」的落点（D24）。
    if (!config_.enabled) {
        if (is_finite(measurement)) {
            estimate_ = measurement;
        }
        return estimate_;
    }

    // 非有限量测直接丢弃：不 reset、不透传、不写进协方差。宁可这一帧不更新，
    // 也不能让 NaN 从此刻起污染每一帧输出（审计 L13）。
    if (!is_finite(measurement) || !is_finite(estimate_)) {
        return estimate_;
    }

    const Eigen::Matrix<double, kPoseDofCount, 1> residual =
        to_delta(measurement * estimate_.inverse());

    const double translation_var = config_.measurement_noise_translation_m *
                                   config_.measurement_noise_translation_m;
    const double rotation_var =
        config_.measurement_noise_rotation_rad * config_.measurement_noise_rotation_rad;
    const double scale = config_.unobservable_noise_scale * config_.unobservable_noise_scale;
    Eigen::Matrix<double, kPoseDofCount, 1> variance;
    for (int i = 0; i < 3; ++i) {
        variance[i] = translation_var;
    }
    for (int i = 3; i < kPoseDofCount; ++i) {
        variance[i] = rotation_var;
    }
    if (observable != nullptr) {
        for (int i = 0; i < kPoseDofCount; ++i) {
            if (!(*observable)[i]) {
                variance[i] *= scale;
            }
        }
    }

    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> noise = variance.asDiagonal();
    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> innovation = covariance_ + noise;
    // 协方差恒为正定对角占优，这里直接求逆；规模固定为 6x6，开销可忽略（D25 的计时会覆盖）。
    Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> gain = covariance_ * innovation.inverse();
    if (observable != nullptr) {
        // 噪声倍数只决定「该方向的增益有多小」，不决定「最多有多大」——协方差增长会把
        // 增益重新推回 0.5（审计 L12），所以这里给不可观方向加一道硬上限。
        for (int i = 0; i < kPoseDofCount; ++i) {
            if (!(*observable)[i] && gain(i, i) > kMaxUnobservableGain) {
                gain.row(i).setZero();
                gain(i, i) = kMaxUnobservableGain;
            }
        }
    }
    estimate_ = from_delta(gain * residual) * estimate_;
    // Joseph 形式而不是 (I - K) P：前者对任意增益都保持对称半正定，后者只在增益恰好最优时
    // 等价——而 D46 的 1 % 上限本身就是「非最优增益」。之后再做一次对称化，消掉矩阵相乘
    // 与相减留下的最后一位漂移（审计 L13）。
    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> identity =
        Eigen::Matrix<double, kPoseDofCount, kPoseDofCount>::Identity();
    const Eigen::Matrix<double, kPoseDofCount, kPoseDofCount> residual_operator = identity - gain;
    covariance_ = residual_operator * covariance_ * residual_operator.transpose() +
                  gain * noise * gain.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    return estimate_;
}

} // namespace foray::localization
