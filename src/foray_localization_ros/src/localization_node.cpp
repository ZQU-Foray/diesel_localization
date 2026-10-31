// Copyright 2026 ZQU-Foray

#include "foray/localization_ros/localization_node.hpp"

#include "foray/localization_ros/adapters.hpp"
#include "foray/localization_ros/conversion.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

namespace foray::localization_ros {

namespace {

/// 单调时钟差值（毫秒）。用 steady_clock 而不是节点时钟：这里量的是真实耗时，
/// 仿真 /clock 会跳，量它没有意义。
double elapsed_ms(const std::chrono::steady_clock::time_point& begin) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

} // namespace
namespace {

constexpr double kDefaultStateRateHz = 50.0;
constexpr double kDefaultTfRateHz = 20.0;
constexpr double kDefaultAlignRateHz = 5.0;
constexpr double kDefaultObstacleRateHz = 10.0;

foray::localization::PoseSourcePolicy parse_policy(const std::string& value) {
    if (value == "lidar") {
        return foray::localization::PoseSourcePolicy::kLidar;
    }
    if (value == "external") {
        return foray::localization::PoseSourcePolicy::kExternal;
    }
    return foray::localization::PoseSourcePolicy::kAuto;
}

} // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options,
                                   std::unique_ptr<foray::localization_map::Aligner> aligner)
    : rclcpp::Node("foray_localization", options), tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_), clusterer_(foray::localization_map::ResidualClustererConfig{}),
      health_() {
    map_manifest_ = declare_parameter<std::string>("map_manifest", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    voxel_leaf_m_ = declare_parameter<double>("voxel_leaf_m", 0.1);
    map_voxel_leaf_m_ = declare_parameter<double>("map_voxel_leaf_m", 0.25);
    residual_threshold_m_ = declare_parameter<double>("residual_threshold_m", 0.3);
    cluster_tolerance_m_ = declare_parameter<double>("cluster_tolerance_m", 0.3);
    verify_map_hashes_ = declare_parameter<bool>("verify_map_hashes", true);
    pose_source_param_ = declare_parameter<std::string>("pose_source", "auto");

    health_config_.ok_mean_residual_m = declare_parameter<double>("ok_mean_residual_m", 0.15);
    health_config_.degraded_mean_residual_m =
        declare_parameter<double>("degraded_mean_residual_m", 0.30);
    health_config_.ok_inlier_ratio = declare_parameter<double>("ok_inlier_ratio", 0.55);
    health_config_.degraded_inlier_ratio = declare_parameter<double>("degraded_inlier_ratio", 0.35);
    health_config_.lost_timeout_sec = declare_parameter<double>("lost_timeout_sec", 1.0);
    health_config_.odom_timeout_sec = declare_parameter<double>("odom_timeout_sec", 0.5);
    health_config_.lidar_timeout_sec = declare_parameter<double>("lidar_timeout_sec", 0.2);
    health_config_.warmup_frames = declare_parameter<int>("warmup_frames", 3);
    health_config_.recover_frames = declare_parameter<int>("recover_frames", 3);
    health_ = foray::localization::HealthMonitor(health_config_);

    // 融合层滤波（decision.md D24）。默认关闭；关闭时 update() 逐字节透传量测，
    // 位姿更新路径与引入滤波前完全一致。只暴露三个现场真正会调的旋钮，
    // 其余噪声参数按 D22 的实测基线取默认值。
    foray::localization::PoseEkfConfig filter_config;
    filter_config.enabled = declare_parameter<bool>("filter_enabled", false);
    filter_config.measurement_noise_translation_m =
        declare_parameter<double>("filter_measurement_noise_m", 0.05);
    filter_config.unobservable_noise_scale =
        declare_parameter<double>("filter_unobservable_noise_scale", 10.0);
    pose_filter_ = foray::localization::PoseEkf(filter_config);
    if (filter_config.enabled) {
        RCLCPP_INFO(get_logger(), "融合层 EKF 已启用: 量测噪声=%.3f m 不可观方向倍数=%.1f",
                    filter_config.measurement_noise_translation_m,
                    filter_config.unobservable_noise_scale);
    }

    foray::localization_map::AlignOptions align_options;
    align_options.max_iterations = declare_parameter<int>("align_max_iterations", 10);
    align_options.max_correspondence_distance_m =
        declare_parameter<double>("align_max_correspondence_m", 0.5);
    max_correspondence_distance_m_ = align_options.max_correspondence_distance_m;

    // 视场裁切与可观测性（decision.md D23）。默认值必须让改造前后行为等价：
    // 360x180 走 crop_by_fov 的全视场直通分支，阈值 0 表示不做方向约束。
    fov_horizontal_deg_ = declare_parameter<double>("fov_horizontal_deg", 360.0);
    fov_vertical_deg_ = declare_parameter<double>("fov_vertical_deg", 180.0);
    fov_center_yaw_deg_ = declare_parameter<double>("fov_center_yaw_deg", 0.0);
    observability_ratio_threshold_ =
        declare_parameter<double>("observability_ratio_threshold", 0.0);
    profile_enabled_ = declare_parameter<bool>("profile_enabled", false);
    const int min_source_points_param = declare_parameter<int>("min_source_points", 0);
    // 负值等价于「不设下限」：显式夹到 0，避免负值经 size_t 转换变成天文数字。
    health_config_.min_source_points =
        static_cast<std::size_t>(min_source_points_param > 0 ? min_source_points_param : 0);
    health_config_.max_unobservable_dofs =
        declare_parameter<int>("max_unobservable_dofs", 6);
    // 后端由编译期开关决定（decision.md D21）。启动时把实际后端打出来，
    // 免得出现「文档说 GICP、实际跑 ICP」这种只能在事后靠精度异常才发现的情况。
    // 注入后端只用于离线回放与节点级测试，日志必须明说，否则现场会误判成默认后端。
    if (aligner) {
        aligner_ = std::move(aligner);
        RCLCPP_INFO(get_logger(), "配准后端: 注入（测试）");
    } else {
        aligner_ = foray::localization_map::make_aligner(align_options);
        RCLCPP_INFO(get_logger(), "配准后端: %s",
                    foray::localization_map::default_aligner_name().c_str());
    }

    foray::localization_map::ResidualClustererConfig cluster_config;
    cluster_config.residual_threshold_m = residual_threshold_m_;
    cluster_config.cluster_tolerance_m = cluster_tolerance_m_;
    cluster_config.min_cluster_size = declare_parameter<int>("min_cluster_size", 5);
    cluster_config.max_cluster_size = declare_parameter<int>("max_cluster_size", 500);
    clusterer_ = foray::localization_map::ResidualClusterer(cluster_config);

    policy_ = parse_policy(pose_source_param_);
    active_source_ = policy_ == foray::localization::PoseSourcePolicy::kExternal
                         ? foray::localization::PoseSource::kExternalOdom
                         : foray::localization::PoseSource::kSentryLidar;

    const auto state_rate = declare_parameter<double>("state_rate_hz", kDefaultStateRateHz);
    const auto tf_rate = declare_parameter<double>("tf_rate_hz", kDefaultTfRateHz);
    const auto align_rate = declare_parameter<double>("align_rate_hz", kDefaultAlignRateHz);
    const auto obstacle_rate =
        declare_parameter<double>("obstacle_rate_hz", kDefaultObstacleRateHz);

    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    const auto external_topic =
        declare_parameter<std::string>("external_pose_topic", "/localization/external_pose");

    // 回调组按「谁阻塞谁」划分（decision.md D30）：配准与聚类能吃掉上百毫秒，
    // 若与 200 Hz 的里程计挤在一个 MutuallyExclusive 组里，即使 executor 是多线程的，
    // 里程计也只能等配准跑完——那正是 L1 里 odom 队列被打满、最后误报断流的成因。
    odom_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    align_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    obstacle_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    const auto state_qos = rclcpp::QoS(10).reliable().transient_local();
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto obstacle_qos = rclcpp::QoS(10).reliable();

    state_pub_ = create_publisher<foray_localization_ros::msg::LocalizationState>(
        "/localization/state", state_qos);
    map_meta_pub_ =
        create_publisher<foray_localization_ros::msg::MapMeta>("/localization/map_meta", map_qos);
    obstacle_pub_ = create_publisher<foray_localization_ros::msg::DynamicObstacleArray>(
        "/localization/dynamic_obstacles", obstacle_qos);

    if (policy_ != foray::localization::PoseSourcePolicy::kExternal) {
        rclcpp::SubscriptionOptions cloud_options;
        cloud_options.callback_group = cloud_group_;
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, rclcpp::SensorDataQoS(),
            std::bind(&LocalizationNode::on_cloud, this, std::placeholders::_1), cloud_options);
        rclcpp::SubscriptionOptions odom_options;
        odom_options.callback_group = odom_group_;
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::QoS(50),
            std::bind(&LocalizationNode::on_odometry, this, std::placeholders::_1), odom_options);
    }
    if (policy_ != foray::localization::PoseSourcePolicy::kLidar) {
        rclcpp::SubscriptionOptions external_options;
        external_options.callback_group = odom_group_;
        external_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            external_topic, rclcpp::QoS(20),
            std::bind(&LocalizationNode::on_external_pose, this, std::placeholders::_1),
            external_options);
    }

    relocalize_srv_ = create_service<foray_localization_ros::srv::Relocalize>(
        "/localization/relocalize",
        std::bind(&LocalizationNode::on_relocalize, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, service_group_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    load_map();

    state_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, state_rate)),
                          std::bind(&LocalizationNode::on_state_timer, this), publish_group_);
    tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, tf_rate)),
                                  std::bind(&LocalizationNode::on_tf_timer, this), publish_group_);
    align_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, align_rate)),
                          std::bind(&LocalizationNode::on_align_timer, this), align_group_);
    obstacle_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, obstacle_rate)),
                          std::bind(&LocalizationNode::on_obstacle_timer, this), obstacle_group_);
}

void LocalizationNode::load_map() {
    if (map_manifest_.empty()) {
        RCLCPP_ERROR(get_logger(), "map_manifest 未设置，定位节点无法启动地图配准");
        mark_map_unavailable("map_manifest 未设置");
        return;
    }
    foray::localization_map::PriorMap map;
    const auto result =
        foray::localization_map::load_prior_map(map_manifest_, &map, verify_map_hashes_);
    if (!result.ok) {
        RCLCPP_ERROR(get_logger(), "加载先验地图失败: %s", result.error.c_str());
        mark_map_unavailable("加载先验地图失败: " + result.error);
        return;
    }
    if (map_voxel_leaf_m_ > 0.0) {
        map.cloud = foray::localization_map::voxel_downsample(map.cloud, map_voxel_leaf_m_);
    }
    map_ = std::move(map);
    has_map_ = true;
    // 目标侧索引与法向只建一次：每帧重建是配准之外最贵的固定开销，
    // 而它们只依赖地图，不依赖当前帧（decision.md D25）。
    map_index_ = std::make_shared<foray::localization_map::PointCloudIndex>(
        map_.cloud, max_correspondence_distance_m_);
    // 法向邻域取 2 倍地图体素边长：单个体素内的点不足以定义一个面片。
    // 体素参数为 0（不降采样）时退回 2 m，覆盖多数室内墙面的局部曲率。
    const double normal_cell = 2.0 * (map_voxel_leaf_m_ > 0.0 ? map_voxel_leaf_m_ : 1.0);
    map_normals_ =
        std::make_shared<foray::localization_map::NormalCache>(map_.cloud, normal_cell, 6);
    if (observability_ratio_threshold_ > 0.0) {
        RCLCPP_INFO(get_logger(), "可观测性分析已启用: 阈值=%.3f 法向覆盖=%.1f%%",
                    observability_ratio_threshold_, 100.0 * map_normals_->coverage());
    }
    // 残差聚类的 target 哈希格同样只依赖地图（decision.md D35）：这里是唯一的地图装载点，
    // 所以也只在这里重建一次，10 Hz 的聚类路径直接复用。
    clusterer_.set_target(map_.cloud);

    RCLCPP_INFO(get_logger(), "先验地图已加载: version=%s pcd=%zu grid=%dx%d",
                map_.manifest.version.c_str(), map_.cloud.size(), map_.grid.width,
                map_.grid.height);

    map_meta_pub_->publish(to_msg_map_meta(map_.manifest, now().nanoseconds()));
}

void LocalizationNode::mark_map_unavailable(const std::string& reason) {
    // 失败必须变成一条可被下游读到的事实，而不是一行日志：父提交上这里只打 ERROR 就
    // return，健康状态停在 `INITIALIZING`、`publish_tf()` 的闸门又只挡 `LOST`，
    // 于是没有地图的节点照样以 20 Hz 广播单位阵 `map → odom`（decision.md D36）。
    std::lock_guard<std::mutex> lock(mutex_);
    map_error_ = reason;
    has_map_ = false;
    health_status_ = health_.enter_lost("map unavailable: " + reason, now().nanoseconds());
}

bool LocalizationNode::lookup_base_from(const std::string& child_frame,
                                       const rclcpp::Time& time,
                                       std::chrono::milliseconds timeout,
                                       Eigen::Isometry3d* base_from_child) {
    try {
        const auto transform = tf_buffer_.lookupTransform(base_frame_, child_frame, time, timeout);
        const auto& stamped = transform.transform;
        const Eigen::Quaterniond quaternion(stamped.rotation.w, stamped.rotation.x,
                                            stamped.rotation.y, stamped.rotation.z);
        base_from_child->setIdentity();
        if (quaternion.norm() > 1e-9) {
            base_from_child->linear() = quaternion.normalized().toRotationMatrix();
        }
        base_from_child->translation() = Eigen::Vector3d(
            stamped.translation.x, stamped.translation.y, stamped.translation.z);
        return true;
    } catch (const tf2::TransformException& exception) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF %s->%s 失败: %s",
                             child_frame.c_str(), base_frame_.c_str(), exception.what());
        return false;
    }
}

void LocalizationNode::on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    std::string error;
    const auto profile_begin = std::chrono::steady_clock::now();
    foray::localization_map::PointCloud points = to_point_cloud(*cloud, &error);
    if (points.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃无效点云: %s", error.c_str());
        return;
    }
    // 先裁切再降采样：裁切是纯空间过滤，先做可以省掉后续降采样的点数（decision.md D23/D25）。
    if (fov_horizontal_deg_ < 360.0 || fov_vertical_deg_ < 180.0) {
        points = foray::localization_map::crop_by_fov(points, fov_horizontal_deg_,
                                                      fov_vertical_deg_, fov_center_yaw_deg_);
        if (points.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "视场裁切后点云为空 (h=%.1f v=%.1f yaw=%.1f)，检查 fov_* 参数",
                                 fov_horizontal_deg_, fov_vertical_deg_, fov_center_yaw_deg_);
            return;
        }
    }
    if (profile_enabled_) {
        profile_.to_pcl_ms += elapsed_ms(profile_begin);
    }
    const auto profile_voxel_begin = std::chrono::steady_clock::now();
    if (voxel_leaf_m_ > 0.0) {
        points = foray::localization_map::voxel_downsample(points, voxel_leaf_m_);
    }
    if (profile_enabled_) {
        profile_.voxel_ms += elapsed_ms(profile_voxel_begin);
    }

    Eigen::Isometry3d base_from_cloud = Eigen::Isometry3d::Identity();
    if (!cloud->header.frame_id.empty() && cloud->header.frame_id != base_frame_) {
        if (!lookup_base_from(cloud->header.frame_id, rclcpp::Time(cloud->header.stamp),
                              std::chrono::milliseconds(100), &base_from_cloud)) {
            return;
        }
    }
    points = foray::localization_map::transform_points(points, base_from_cloud);

    std::int64_t stamp_ns = 0;
    if (!to_timestamp_ns(cloud->header.stamp, &stamp_ns)) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = std::move(points);
    latest_cloud_stamp_ns_ = stamp_ns;
    // 激光活性用“接收时刻”判定，不用消息 stamp：实车驱动的时间戳可能与节点时钟不同源
    // （use_sim_time、传感器上电时刻），跨时钟域比较会把健康状态直接判成断流。
    last_cloud_rx_ns_ = now().nanoseconds();
    has_cloud_ = true;
}

void LocalizationNode::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
    // 进入即计数，且不持锁：若在锁后才累加，「进入次数」就永远等于「处理次数」，
    // 反而看不出帧在队列里等过头（decision.md D29）。
    odom_received_.fetch_add(1, std::memory_order_relaxed);
    std::string error;
    if (!validate_odometry(*odometry, &error)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃无效里程计: %s", error.c_str());
        return;
    }
    std::int64_t stamp_ns = 0;
    if (!to_timestamp_ns(odometry->header.stamp, &stamp_ns)) {
        return;
    }
    Eigen::Isometry3d odom_to_child;
    if (!to_isometry(odometry->pose.pose, &odom_to_child)) {
        return;
    }
    // `Odometry.pose` 的语义是 `odom → child_frame_id`，下游要的是 `odom → base_frame`
    // （decision.md D37）。父提交直接把前者当后者用：只要上游发的是 `odom → livox_frame`
    // ——`README.md` 里 point_lio 就是这个约定——雷达与 base 之间的外参就被整块丢掉，
    // 而这条外参正好是 `map → odom` 的杠杆臂。这里和 `on_cloud()` 用同一套 TF 转换；
    // 查不到就丢帧，绝不拿单位阵把「不知道」装成「一致」。
    Eigen::Isometry3d odom_to_base = odom_to_child;
    if (odometry->child_frame_id != base_frame_) {
        Eigen::Isometry3d base_from_child = Eigen::Isometry3d::Identity();
        // 取「最新可用」而不是「本帧 stamp」：里程计 200 Hz，逐帧精确查 TF 一旦晚到就会被
        // 判成未来外插；雷达/base 的安装外参是刚体量，晚一个 TF 周期没有影响。
        // timeout 取 0：这条回调 5 ms 就来一帧，任何阻塞等待都会把队列堆起来（L1 的病）。
        if (!lookup_base_from(odometry->child_frame_id, rclcpp::Time(0, 0, RCL_ROS_TIME),
                             std::chrono::milliseconds(0), &base_from_child)) {
            return;
        }
        // T(odom←base) = T(odom←child) · T(child←base)，而 T(child←base) = T(base←child)⁻¹。
        odom_to_base = odom_to_child * base_from_child.inverse();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // 两次配准之间协方差随时间长大：里程计外推越久，下一帧量测就该越被重视。
    if (last_odom_ns_ > 0 && stamp_ns > last_odom_ns_) {
        pose_filter_.predict(static_cast<double>(stamp_ns - last_odom_ns_) / 1e9);
    }
    odom_to_base_ = odom_to_base;
    last_odom_ns_ = stamp_ns;
    has_odom_ = true;
    ++odom_processed_;
    // 处理时刻与消息时间戳同为节点时钟，可以直接相减：这个差就是「帧在队列里等了多久」。
    const double lag_sec = static_cast<double>(now().nanoseconds() - stamp_ns) / 1e9;
    if (lag_sec > max_odom_lag_sec_) {
        max_odom_lag_sec_ = lag_sec;
    }
}

void LocalizationNode::on_external_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose) {
    if (policy_ == foray::localization::PoseSourcePolicy::kLidar) {
        return;
    }
    std::string error;
    if (!validate_header(pose->header, map_frame_.c_str(), &error)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃外部位姿: %s", error.c_str());
        return;
    }
    std::int64_t stamp_ns = 0;
    if (!to_timestamp_ns(pose->header.stamp, &stamp_ns)) {
        return;
    }
    Eigen::Isometry3d map_to_base;
    if (!to_isometry(pose->pose.pose, &map_to_base)) {
        return;
    }

    foray::localization::RegistrationSample sample;
    sample.stamp_ns = stamp_ns;
    sample.valid = true;
    sample.mean_residual_m = 0.0;
    sample.inlier_ratio = 1.0;
    sample.fitness = 1.0;

    std::lock_guard<std::mutex> lock(mutex_);
    map_to_base_ = map_to_base;
    map_to_odom_ = map_to_base;
    odom_to_base_ = Eigen::Isometry3d::Identity();
    last_odom_ns_ = stamp_ns;
    has_odom_ = true;
    last_valid_ns_ = stamp_ns;
    active_source_ = foray::localization::PoseSource::kExternalOdom;
    health_status_ = health_.update(sample, stamp_ns, stamp_ns, max_odom_lag_sec_);
}

void LocalizationNode::maybe_flush_profile(std::int64_t now_ns) {
    if (!profile_enabled_ || profile_.samples == 0) {
        return;
    }
    if (profile_.window_start_ns == 0) {
        profile_.window_start_ns = now_ns;
        return;
    }
    // 5 s 一个统计窗口：太短会被单帧抖动带偏，太长看不出趋势（decision.md D25）。
    constexpr std::int64_t kWindowNs = 5'000'000'000LL;
    if (now_ns - profile_.window_start_ns < kWindowNs) {
        return;
    }
    const double frames = static_cast<double>(profile_.samples);
    const double seconds = static_cast<double>(now_ns - profile_.window_start_ns) / 1e9;
    RCLCPP_INFO(get_logger(), "耗时窗口: to_pcl=%.2fms voxel=%.2fms align=%.2fms 帧数=%.0f 配准率=%.1fHz",
                profile_.to_pcl_ms / frames, profile_.voxel_ms / frames,
                profile_.align_ms / frames, frames, frames / seconds);
    profile_ = ProfileTotals{};
}

void LocalizationNode::on_align_timer() {
    if (policy_ == foray::localization::PoseSourcePolicy::kExternal) {
        return;
    }

    foray::localization_map::PointCloud cloud;
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    std::int64_t stamp_ns = 0;
    std::int64_t odom_ns = 0;
    // 人工重定位注入的是完整先验位姿，此时不该做方向约束——用户明确要的就是这一帧的解。
    bool relocalizing = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_map_ || !has_cloud_ || !has_odom_) {
            return;
        }
        cloud = latest_cloud_;
        stamp_ns = latest_cloud_stamp_ns_;
        odom_ns = last_odom_ns_;
        if (relocalize_pending_.exchange(false)) {
            guess = pending_map_to_base_;
            relocalizing = true;
        } else {
            guess = map_to_odom_ * odom_to_base_;
        }
    }

    const auto profile_align_begin = std::chrono::steady_clock::now();
    const auto result = aligner_->align(cloud, map_.cloud, guess);
    // 配准耗时总是量：profile 只是「打印不打印」的开关，诊断计数与它无关（decision.md D29）。
    const double align_ms = elapsed_ms(profile_align_begin);
    if (profile_enabled_) {
        profile_.align_ms += align_ms;
        ++profile_.samples;
        maybe_flush_profile(now().nanoseconds());
    }
    // 配准失败（有效对应点不足 / 输入为空）时 inlier_ratio 为 0，健康状态会一路退到 LOST。
    // 这条日志是唯一能看出「为什么无效」的地方，不加就只能看到 registration rejected。
    if (result.inlier_ratio <= 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "配准无有效对应: %s (source=%zu target=%zu iterations=%d)",
                             result.error.c_str(), cloud.size(), map_.cloud.size(),
                             result.iterations);
    }

    // 可观测性分析（decision.md D23）：配准「收敛」不等于「被约束住」，
    // 视野一裁就可能只剩两三个方向真有约束。这里在收敛位姿上建信息矩阵并做特征分解，
    // 后面只把修正量投影到可观子空间，不可观方向继续由里程计外推承担。
    foray::localization_map::ObservabilityReport observability;
    const bool observability_enabled = observability_ratio_threshold_ > 0.0 && !relocalizing &&
                                       map_index_ != nullptr && map_normals_ != nullptr &&
                                       result.inlier_ratio > 0.0;
    if (observability_enabled) {
        observability = foray::localization_map::analyze_observability(
            cloud, result.transform, *map_index_, *map_normals_, max_correspondence_distance_m_,
            observability_ratio_threshold_);
        if (observability.degenerate()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "配准退化: %d/6 个方向不可观 (condition=%.1f 点面约束=%zu)",
                                 observability.unobservable_count, observability.condition_number,
                                 observability.inlier_count);
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    ++align_runs_;
    if (align_ms > max_align_ms_) {
        max_align_ms_ = align_ms;
    }
    foray::localization::RegistrationSample sample;
    sample.stamp_ns = stamp_ns;
    sample.valid = result.inlier_ratio > 0.0;
    sample.mean_residual_m = result.mean_residual_m;
    sample.inlier_ratio = result.inlier_ratio;
    sample.fitness = result.fitness;
    sample.source_points = cloud.size();
    sample.condition_number = observability_enabled ? observability.condition_number : 0.0;
    sample.unobservable_count = observability_enabled ? observability.unobservable_count : 0;
    // 健康评估以当前时间进行：odom_ns 是最近一次里程计时间戳，更新快于点云，
    // 与点云时间戳相比会天然反序，若直接传入会误判为里程计断流。
    const std::int64_t eval_ns = now().nanoseconds();
    // 把里程计积压一并交给状态机：同一句「odom 过期」必须能区分「没数据」和「来不及处理」。
    health_status_ = health_.update(sample, eval_ns, odom_ns, max_odom_lag_sec_);

    // LOST 粘滞：只有人工重定位 reset 后才重新接受配准结果。
    if (health_status_.state != foray::localization::HealthState::kLost &&
        result.inlier_ratio > 0.0) {
        if (observability_enabled) {
            // 左乘增量（map 系）：correction 只保留可观分量，不可观方向保持当前估计。
            const Eigen::Isometry3d delta = result.transform * map_to_base_.inverse();
            Eigen::Matrix<double, foray::localization_map::kDofCount, 1> delta_vector;
            delta_vector.head<3>() = delta.translation();
            const Eigen::AngleAxisd delta_rotation(delta.linear());
            delta_vector.tail<3>() = delta_rotation.angle() * delta_rotation.axis();
            const auto projected = foray::localization_map::project_delta_to_observable(
                delta_vector, observability);
            Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
            correction.translation() = projected.head<3>();
            const Eigen::Vector3d omega = projected.tail<3>();
            const double angle = omega.norm();
            if (angle > 1e-12) {
                correction.linear() = Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
            }
            map_to_base_ = correction * map_to_base_;
        } else {
            map_to_base_ = result.transform;
        }
        if (pose_filter_.enabled()) {
            // 只平滑 map -> odom（D24）：量测由几何修正后的位姿换算而来，输出再与当前
            // 里程计合回 map -> base，所以 TF 频率与坐标系语义都不变。
            const Eigen::Isometry3d measurement = map_to_base_ * odom_to_base_.inverse();
            map_to_odom_ = pose_filter_.update(
                measurement, observability_enabled ? &observability.observable : nullptr);
            map_to_base_ = map_to_odom_ * odom_to_base_;
        } else {
            map_to_odom_ = map_to_base_ * odom_to_base_.inverse();
        }
        last_valid_ns_ = stamp_ns;
    }
    has_cloud_ = false;
}

void LocalizationNode::publish_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::int64_t now_ns = now().nanoseconds();
    // 地图不可用是最靠前的成因：没有地图就没有配准，`on_align_timer()` 会在 `!has_map_`
    // 处提前返回、从不喂健康状态机，所以必须在这里按节拍显式喂 LOST（decision.md D36）。
    if (!has_map_) {
        health_status_ = health_.enter_lost(
            "map unavailable: " + (map_error_.empty() ? std::string("unknown") : map_error_),
            now_ns);
    } else if (policy_ == foray::localization::PoseSourcePolicy::kExternal) {
        foray::localization::RegistrationSample sample;
        sample.stamp_ns = now_ns;
        sample.valid = last_odom_ns_ > 0;
        sample.mean_residual_m = 0.0;
        sample.inlier_ratio = sample.valid ? 1.0 : 0.0;
        health_status_ = health_.update(sample, now_ns, last_odom_ns_, max_odom_lag_sec_);
    } else if (has_odom_ && health_config_.odom_timeout_sec > 0.0 &&
               static_cast<double>(now_ns - last_odom_ns_) / 1e9 >
                   health_config_.odom_timeout_sec) {
        foray::localization::RegistrationSample sample;
        sample.stamp_ns = now_ns;
        sample.valid = false;
        sample.mean_residual_m = 0.0;
        sample.inlier_ratio = 0.0;
        health_status_ = health_.update(sample, now_ns, last_odom_ns_, max_odom_lag_sec_);
    } else if (has_odom_ && health_config_.lidar_timeout_sec > 0.0 &&
               (last_cloud_rx_ns_ <= 0 || static_cast<double>(now_ns - last_cloud_rx_ns_) / 1e9 >
                                              health_config_.lidar_timeout_sec)) {
        // 激光断流：点云停下后配准回调不再运行（!has_cloud_ 提前 return），
        // 若这里不显式喂不可用样本，健康状态会冻结在 OK，TF 会按旧 map→odom 继续外推，
        // 正是 README 明令禁止的「静默外推」。状态机自身负责 DEGRADED→LOST 的时限。
        foray::localization::RegistrationSample sample;
        sample.stamp_ns = now_ns;
        sample.valid = false;
        sample.mean_residual_m = 0.0;
        sample.inlier_ratio = 0.0;
        health_status_ = health_.update(sample, now_ns, last_odom_ns_, max_odom_lag_sec_);
    }

    foray::localization::Pose3D pose;
    pose.parent_frame = map_frame_;
    pose.child_frame = base_frame_;
    pose.transform = policy_ == foray::localization::PoseSourcePolicy::kExternal
                         ? map_to_base_
                         : map_to_odom_ * odom_to_base_;

    foray::localization::HealthStatus status = health_status_;
    status.age_sec = last_valid_ns_ > 0 ? static_cast<double>(now_ns - last_valid_ns_) / 1e9 : 0.0;
    if (status.state == foray::localization::HealthState::kLost) {
        status.confidence = 0.0;
    }
    state_pub_->publish(to_msg_state(status, pose, active_source_, now_ns));
}

void LocalizationNode::publish_tf() {
    std::lock_guard<std::mutex> lock(mutex_);
    // 只放行 OK / DEGRADED（decision.md D36）：闸门原来写成「非 LOST」，等于把
    // `INITIALIZING` 也算成可信——可那时还没有任何一帧配准证据，发出去的
    // `map → odom` 只能是构造时那个恒等矩阵。README 健康契约要求「LOST 停发」，
    // 而这里比它更紧：只有被配准证明过的状态才允许发布。
    const foray::localization::HealthState state = health_status_.state;
    if ((state != foray::localization::HealthState::kOk &&
         state != foray::localization::HealthState::kDegraded) ||
        !has_odom_) {
        return;
    }
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    const Eigen::Isometry3d& transform = map_to_odom_;
    message.transform.translation.x = transform.translation().x();
    message.transform.translation.y = transform.translation().y();
    message.transform.translation.z = transform.translation().z();
    const Eigen::Quaterniond quaternion(transform.linear());
    message.transform.rotation.x = quaternion.x();
    message.transform.rotation.y = quaternion.y();
    message.transform.rotation.z = quaternion.z();
    message.transform.rotation.w = quaternion.w();
    tf_broadcaster_->sendTransform(message);
    ++tf_published_;
}

LocalizationNode::Diagnostics LocalizationNode::diagnostics() {
    std::lock_guard<std::mutex> lock(mutex_);
    Diagnostics snapshot;
    snapshot.odom_received = odom_received_.load(std::memory_order_relaxed);
    snapshot.odom_processed = odom_processed_;
    snapshot.max_odom_lag_sec = max_odom_lag_sec_;
    snapshot.align_runs = align_runs_;
    snapshot.max_align_ms = max_align_ms_;
    snapshot.tf_published = tf_published_;
    return snapshot;
}

void LocalizationNode::on_state_timer() {
    publish_state();
}

void LocalizationNode::on_tf_timer() {
    publish_tf();
}

void LocalizationNode::on_obstacle_timer() {
    foray::localization_map::PointCloud cloud;
    Eigen::Isometry3d current_map_to_base;
    std::int64_t stamp_ns = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_map_ || !has_cloud_) {
            return;
        }
        cloud = latest_cloud_;
        current_map_to_base = map_to_base_;
        stamp_ns = latest_cloud_stamp_ns_;
    }
    // has_cloud_ 会被配准线程消费，这里用时间戳判断数据是否仍然新鲜。
    if (now().nanoseconds() - stamp_ns > 1000000000LL) {
        return;
    }
    const auto in_map = foray::localization_map::transform_points(cloud, current_map_to_base);
    const auto clusters = clusterer_.cluster(in_map);
    obstacle_pub_->publish(to_msg_obstacles(clusters, map_frame_, stamp_ns));
}

void LocalizationNode::on_relocalize(
    const std::shared_ptr<foray_localization_ros::srv::Relocalize::Request> request,
    std::shared_ptr<foray_localization_ros::srv::Relocalize::Response> response) {
    if (request->mode == foray_localization_ros::srv::Relocalize::Request::MODE_PLACE) {
        response->accepted = false;
        response->message = "place 后端默认关闭（D10）；请使用 MODE_GEOMETRIC 给定初值";
        return;
    }

    Eigen::Isometry3d initial;
    if (!to_isometry(request->initial_pose.pose, &initial)) {
        response->accepted = false;
        response->message = "初始位姿含非法数值或零四元数";
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_map_to_base_ = initial;
        relocalize_pending_ = true;
        health_.reset();
        health_status_ = foray::localization::HealthStatus{};
        health_status_.state = foray::localization::HealthState::kInitializing;
        map_to_base_ = initial;
        map_to_odom_ = initial * odom_to_base_.inverse();
        // 硬切：重定位注入的是人工确认的位姿，滤波器必须整体重置而不是慢慢收敛。
        pose_filter_.reset(map_to_odom_);
    }
    response->accepted = true;
    response->message = "已接受初始位姿，下一帧按该初值配准";
    RCLCPP_INFO(get_logger(), "收到手动重定位请求");
}

} // namespace foray::localization_ros
