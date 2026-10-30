// Copyright 2026 ZQU-Foray

#include "foray/localization/health.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace foray::localization {
namespace {

constexpr double kSecondsToNanoseconds = 1.0e9;

/// 时间戳反序容差：里程计 50–100 Hz、点云 10 Hz，不同回调源的时间戳天然存在
/// 不超过一个点云周期（100 ms）的先后抖动；保守取 2 个点云周期（200 ms），
/// 超过该容差的“now 早于 then”仍视为时间基准异常（不可用）。
constexpr Timestamp kTimestampSkewToleranceNs = 2 * 100'000'000;

double to_seconds(Timestamp nanoseconds) {
    return static_cast<double>(nanoseconds) / kSecondsToNanoseconds;
}

double age_seconds(Timestamp now_ns, Timestamp then_ns) {
    if (then_ns <= 0) {
        return std::numeric_limits<double>::infinity();
    }
    if (now_ns < then_ns) {
        const Timestamp skew_ns = then_ns - now_ns;
        if (skew_ns <= kTimestampSkewToleranceNs) {
            return 0.0;
        }
        return std::numeric_limits<double>::infinity();
    }
    return to_seconds(now_ns - then_ns);
}

double clamp01(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, 0.0, 1.0);
}

} // namespace

const char* to_string(HealthState state) {
    switch (state) {
    case HealthState::kInitializing:
        return "initializing";
    case HealthState::kOk:
        return "ok";
    case HealthState::kDegraded:
        return "degraded";
    case HealthState::kLost:
        return "lost";
    }
    return "unknown";
}

const char* to_string(PoseSource source) {
    switch (source) {
    case PoseSource::kSentryLidar:
        return "sentry_lidar";
    case PoseSource::kExternalOdom:
        return "external_odom";
    case PoseSource::kUnknown:
        return "unknown";
    }
    return "unknown";
}

HealthMonitor::HealthMonitor(HealthConfig config) : config_(config) {}

void HealthMonitor::reset() {
    state_ = HealthState::kInitializing;
    state_since_ns_ = 0;
    start_ns_ = 0;
    last_valid_ns_ = 0;
    first_bad_ns_ = 0;
    good_frames_ = 0;
    reason_ = "waiting for first registration after reset";
}

void HealthMonitor::transition(HealthState next, const std::string& reason, Timestamp now_ns) {
    state_ = next;
    state_since_ns_ = now_ns;
    reason_ = reason;
    if (next != HealthState::kDegraded) {
        first_bad_ns_ = 0;
    }
}

double HealthMonitor::compute_confidence(const RegistrationSample& sample) const {
    if (!sample.valid || state_ == HealthState::kInitializing || state_ == HealthState::kLost) {
        return 0.0;
    }

    const double denominator = std::max(2.0 * config_.degraded_mean_residual_m, 1e-6);
    const double residual_term = 1.0 - clamp01(sample.mean_residual_m / denominator);
    double confidence = clamp01(sample.inlier_ratio) * residual_term;
    if (state_ == HealthState::kDegraded) {
        confidence *= 0.5;
    }
    return clamp01(confidence);
}

HealthStatus HealthMonitor::snapshot(Timestamp now_ns) const {
    HealthStatus status;
    status.state = state_;
    status.confidence = last_confidence_;
    status.age_sec = age_seconds(now_ns, last_valid_ns_);
    status.state_age_sec = age_seconds(now_ns, state_since_ns_);
    status.reason = reason_;
    return status;
}

HealthStatus HealthMonitor::update(const RegistrationSample& sample, Timestamp now_ns,
                                   Timestamp last_odom_ns) {
    if (start_ns_ == 0) {
        start_ns_ = now_ns;
        state_since_ns_ = now_ns;
    }

    // 里程计断流是最高优先级失效：没有 odom 就没有 map->odom 的合成基础。
    const double odom_age = age_seconds(now_ns, last_odom_ns);
    if (!std::isfinite(odom_age) || odom_age > config_.odom_timeout_sec) {
        if (state_ != HealthState::kLost) {
            transition(HealthState::kLost, "odometry timeout", now_ns);
        }
        last_confidence_ = 0.0;
        return snapshot(now_ns);
    }

    // LOST 只能由显式 Relocalize（reset）恢复；期间不接受“看起来正常”的样本。
    if (state_ == HealthState::kLost) {
        last_confidence_ = 0.0;
        return snapshot(now_ns);
    }

    const bool valid = sample.valid && sample.stamp_ns > 0;
    if (valid) {
        last_valid_ns_ = std::max(last_valid_ns_, sample.stamp_ns);
        // 点数与可观性是「残差/IPR 之外」的两条独立降级理由（decision.md D23）：
        // 残差小只说明对上的点贴得紧，不说明这些点在几何上约束住了 6 个自由度。
        const bool points_ok = config_.min_source_points == 0 ||
                               sample.source_points >= config_.min_source_points;
        const bool observable_ok = sample.unobservable_count <= config_.max_unobservable_dofs;
        const bool good = sample.mean_residual_m <= config_.ok_mean_residual_m &&
                          sample.inlier_ratio >= config_.ok_inlier_ratio && points_ok &&
                          observable_ok;
        const bool degraded_ok = sample.mean_residual_m <= config_.degraded_mean_residual_m &&
                                 sample.inlier_ratio >= config_.degraded_inlier_ratio;

        if (good) {
            ++good_frames_;
            first_bad_ns_ = 0;
        } else {
            good_frames_ = 0;
            if (first_bad_ns_ == 0) {
                first_bad_ns_ = now_ns;
            }
        }

        if (state_ == HealthState::kInitializing) {
            if (good_frames_ >= config_.warmup_frames) {
                transition(HealthState::kOk, "warmup complete", now_ns);
            } else if (first_bad_ns_ != 0 &&
                       age_seconds(now_ns, first_bad_ns_) >= config_.lost_timeout_sec) {
                transition(HealthState::kLost, "initialization timeout", now_ns);
            }
        } else if (state_ == HealthState::kOk) {
            if (!good) {
                const double bad_age = age_seconds(now_ns, first_bad_ns_);
                if (!degraded_ok && std::isfinite(bad_age) && bad_age >= config_.lost_timeout_sec) {
                    transition(HealthState::kLost, "registration quality lost", now_ns);
                } else if (bad_age >= config_.degraded_hold_sec) {
                    // 降级理由要能直接指向原因：现场只看到「degraded」是排查不了的。
                    const char* why = !points_ok        ? "insufficient source points"
                                      : !observable_ok  ? "insufficient observability"
                                                        : "residual or inlier ratio out of bounds";
                    transition(HealthState::kDegraded, why, now_ns);
                }
            }
        } else if (state_ == HealthState::kDegraded) {
            if (good_frames_ >= config_.recover_frames) {
                transition(HealthState::kOk, "registration recovered", now_ns);
            } else if (!degraded_ok && first_bad_ns_ != 0 &&
                       age_seconds(now_ns, first_bad_ns_) >= config_.lost_timeout_sec) {
                transition(HealthState::kLost, "registration quality lost", now_ns);
            }
        }
    } else {
        good_frames_ = 0;
        if (first_bad_ns_ == 0) {
            first_bad_ns_ = now_ns;
        }
        if (state_ == HealthState::kInitializing) {
            if (age_seconds(now_ns, start_ns_) >= config_.lost_timeout_sec) {
                transition(HealthState::kLost, "no valid registration during initialization",
                           now_ns);
            }
        } else if (state_ == HealthState::kOk) {
            transition(HealthState::kDegraded, "registration rejected", now_ns);
        } else if (state_ == HealthState::kDegraded &&
                   age_seconds(now_ns, last_valid_ns_) >= config_.lost_timeout_sec) {
            transition(HealthState::kLost, "no valid registration", now_ns);
        }
    }

    last_confidence_ = compute_confidence(sample);
    return snapshot(now_ns);
}

PoseSource select_pose_source(PoseSourcePolicy policy, bool lidar_available,
                              bool external_available) {
    switch (policy) {
    case PoseSourcePolicy::kLidar:
        return lidar_available ? PoseSource::kSentryLidar : PoseSource::kUnknown;
    case PoseSourcePolicy::kExternal:
        return external_available ? PoseSource::kExternalOdom : PoseSource::kUnknown;
    case PoseSourcePolicy::kAuto:
        if (lidar_available) {
            return PoseSource::kSentryLidar;
        }
        if (external_available) {
            return PoseSource::kExternalOdom;
        }
        return PoseSource::kUnknown;
    }
    return PoseSource::kUnknown;
}

} // namespace foray::localization
