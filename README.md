# foray_localization

> **L3** · owner 角色：导航

定位服务 + 地图管理 + 配准残差动态物体检测

**安全关键**——激光只做定位，构成单点故障。

---

## 快速开始

```bash
# 1. 拉取（仓齐备后改为从元仓 foray_ws 一键拉取）
git clone git@github.com:ZQU-Foray/foray_localization.git
cd foray_localization

# 2. 依赖安装（PCL 由 package.xml 经 rosdep 带入；无 PCL 环境的降级构建见「依赖」）
rosdep install -y -r --from-paths src --ignore-src --rosdistro humble

# 3. 构建（仓根即 colcon 空间根，全部包在 src/ 下）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 运行（哨兵；步兵/重装用 config_name:=infantry.yaml / heavy.yaml）
source install/setup.bash
ros2 launch foray_localization_bringup localization.launch.py \
  config_name:=sentry.yaml \
  map_manifest:=/path/to/map/manifest.yaml
```

## 无实车验证

本仓**不内置**合成仿真与离线建图工具（2026-10-27 移出，见 `decision.md` D16）。
车下调试只保留一条路径：**同一个定位节点 + 外部数据源**，接口不变。

```bash
# 录制包重放：节点照常订阅 /livox/lidar 与 /aft_mapped_to_init
ros2 launch foray_localization_bringup localization.launch.py \
  config_name:=sentry.yaml map_manifest:=/path/to/map/manifest.yaml &
ros2 bag play <recorded_bag> --clock
```

- 需要已知初值时，向 `/localization/relocalize` 注入 `map → base_frame`，观察状态机是否
  回到 `INITIALIZING` 并收敛到 `OK`。
- 建图与地图验证在**本仓之外**完成；本仓只消费符合 `manifest.yaml` 契约的地图（见「地图」）。
- CI 只断言「可构建 + 可执行文件安装位置 + launch 参数可解析 + `colcon test` 通过」，
  **不对定位精度作断言**——精度验收在两处：实车录制包，与下游仿真仓 `foray_simulation`
  （它的需求 A2 就是本仓在仿真里的验收口径：10 s 内状态 `OK`、位置误差 < 0.30 m、
  朝向误差 < 5°）。
- 本地跑单元测试不需要装 ROS：容器 `foray_sim:humble-fortress`（Humble + Gazebo Fortress，
  含 PCL 1.12）把本仓挂到 `/ws/src/`，在里面 `colcon build` / `colcon test` 即可。

## 定位方案

内置**先验地图** + 激光配准；**赛时不跑在线 SLAM**。

```
MID360 点云 ──► point_lio（仅里程计）──► T_odom_lidar
      │                                      │
      └──► 5 Hz GICP ──► 先验点云地图        │
                    │                       │
                    ├──► T_map_lidar ───────┴──► T_map_odom ──► TF（20 Hz）
                    │                                   │
                    └──► 配准残差 {p} ──► 聚类 ──► 动态物体（无标号）
```

- **里程计**：`point_lio` 只提供 `odom → lidar`，不参与地图更新；赛时不做 SLAM。
- **配准**：`foray_localization_map` 以先验 PCD 地图为目标做 GICP（默认 PCL GICP，5 Hz），
  输出 `T_map_lidar`、fitness、内点率与残差点。后端在**编译期**选择：
  `-DFORAY_WITH_PCL=OFF` 时退回内置点到点 ICP（精度较低，仅用于无 PCL 环境与降级排查）。
- **融合**：`foray_localization_ros` 由 `T_map_lidar` 与 `T_odom_lidar` 合成
  `map → odom`，以 20 Hz 发布；`LocalizationState` 以 50 Hz 发布。
- **位姿滤波**：融合层对 `map → odom` 做 EKF（`decision.md` D24）。配准是 5 Hz 的离散量测，
  两次量测之间的 200 ms 全靠里程计外推；不滤波时每次配准都会让 `map → odom` 阶跃跳变。
  滤波器以「`map → odom` 在两次配准之间应保持常数」为预测模型，量测噪声按残差与各方向可观性
  **逐方向**放大。TF 与 `LocalizationState` 的频率、坐标系均不变，下游无感；
  可整体关闭退回直通（逃生口见 `decision.md` D24）。
- **可观测性与裁切**：定位必须在**视野被裁**时仍可用——既包括传感器视场受限、遮挡造成的被动
  裁切，也包括为省算力主动做的 ROI / FOV 裁切。配准是否可信不能用单一阈值概括：
  `foray_localization_map` 在配准收敛位姿上建立**点面残差信息矩阵**（6×6），特征分解得到各
  自由度的可观性，把不可观方向显式上报，并**只在可观方向上**接受配准修正（`decision.md` D23）；
  不可观方向的位姿由里程计外推承担，不被退化解拉走。可观性说的「绕原点转」必须是被修正的那个
  位姿的原点（滤波启用时是 `map → odom`），不是本体位置——两者相差机器人离里程计原点的距离
  （`decision.md` D49）。
- **健康**：状态机 `INITIALIZING → OK ⇄ DEGRADED → LOST`，失效显式上报，**禁止静默外推**。
  退化（输入点数不足、可观性不足）是显式降级理由，不是沉默接受的理由。
- **重定位**：启动时 `map → odom` 取单位阵（出生点即 `map` 原点，`odom` 与 `map` 重合）；
  丢失后调用 `/localization/relocalize`（`MODE_GEOMETRIC` + `map → base_frame` 初值）注入，
  状态机回到 `INITIALIZING` 直到收敛。自动全局重定位（ONNX Scan Context）为可选 P2，默认关闭。
- **动态物体**：配准残差聚类，输出质心/尺寸/点数/置信度，**无类别标号**；
  与「地图过期」通过持续时间、允许动态区域标注区分。聚类判「这帧点云还新鲜吗」用的是节点
  时钟上的收到时刻，与点云 header 的传感器时间戳无关（两者是不同时钟域，`decision.md` D51）。

## 目录布局

仓根**就是 colcon 空间根**，全部 ROS 2 包位于 `src/<package>/`，目录名与 `package.xml`
的 `<name>` 一致；C++ 小工具放 `apps/`、资产放 `data/`，不得出现 `tools/`。
完整骨架与命名后缀词表见 [`tree.md`](tree.md) 的「布局规则」「命名规则」，
CI 的 `dir-layout` 作业按同一规则校验。

## Package 布局

所有包均在 `src/` 下：

| package | 职责 | 是否依赖 ROS |
|---|---|---|
| `foray_localization_core` | 位姿/健康状态机/位姿源仲裁/地图 manifest/几何，纯函数 | 否 |
| `foray_localization_map` | PCD 先验地图、2D 栅格、PCL GICP 后端（内置 ICP 兜底）、残差聚类 | 否 |
| `foray_localization_ros` | 节点、TF、QoS、消息/服务、map_server、适配器 | 是 |
| `foray_localization_relocalization` | 可选 ONNX Runtime（CPU）全局重定位（原 `_place`，已更名），默认 OFF | 可选 |
| `foray_localization_bringup` | 三兵种 launch / params（`heavy.yaml` 为装甲兵种预留，2027 3v3 不使用） | 是 |

## 接口

### 话题

| 方向 | 名称 | 类型 | QoS / 频率 |
|---|---|---|---|
| 输入 | `/aft_mapped_to_init` | `nav_msgs/Odometry` | sensor_data / 100–200 Hz |
| 输入 | `/livox/lidar` | `sensor_msgs/PointCloud2` | sensor_data / 10 Hz |
| 输入 | `/localization/external_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 步兵/重装外部位姿，50 Hz |

`/aft_mapped_to_init` 的 `pose` 是 `odom → child_frame_id`：`child_frame_id` 等于
`base_frame` 时直接使用，否则按 TF 把安装外参换算进来（`decision.md` D37）；查不到 TF
就丢这一帧，不会拿单位阵冒充。

`/localization/external_pose` 是 `map → base_frame` 的量测，与配准结果同类：`pose_source`
为 `external` 时无条件采用；为 `auto` 时**只在雷达位姿断流超过 `lidar_timeout_sec` 期间**采用，
雷达恢复后位姿源自动翻回雷达（`decision.md` D38）。外部位姿没有可测的配准质量，
所以它不参与残差/内点率判据，健康状态只判它是否新鲜。
| 输出 | `/localization/state` | `foray_localization_ros/LocalizationState` | reliable, transient_local / 50 Hz |
| 输出 | `/localization/map_meta` | `foray_localization_ros/MapMeta` | transient_local / 1 Hz |
| 输出 | `/localization/dynamic_obstacles` | `foray_localization_ros/DynamicObstacleArray` | reliable / 10 Hz |

### 服务

| 名称 | 类型 | 说明 |
|---|---|---|
| `/localization/relocalize` | `foray_localization_ros/Relocalize` | 注入 `map → base_frame` 初值（默认 `base_footprint`），状态机回到 `INITIALIZING` 直到收敛 |

### TF

| 变换 | 发布者 | 频率 |
|---|---|---|
| `map → odom` | `foray_localization_ros` | 20 Hz |
| `odom → lidar` | `point_lio` | 100–200 Hz |
| `base_footprint → lidar` | URDF / `robot_state_publisher`（静态） | — |
| `base_footprint → <里程计 child_frame>` | 本体侧（URDF 静态或仿真链） | — |

**`base_footprint → <child_frame>` 这一层不是可选项。** 本仓按 D37 把 `/aft_mapped_to_init`
的 `pose` 当「`odom → child_frame_id`」用，`child_frame_id` 由上游决定（`point_lio` 硬编码
`body`，见其 `laserMapping.cpp:228` 与 `:270`），与本仓的 `base_frame` 往往不是同一个名字；
该层缺失时每一帧点云都在 `lookup_base_from()` 失败后被丢弃，状态机永远停在
`INITIALIZING`，而 `point_lio` 的 `body → camera_init` 那条 TF 树与 `odom` 树不连通，补不上这层。
补帧方向只能是 `base_footprint → body`：反成 `body → base_footprint` 会让
`base_footprint` 出现两个父（`odom` 与本层），tf2 直接断树（审计 L2）。

### 关键参数

以下两张表与实现**逐字对应**：`declare_parameter` 名字、默认值、`bringup/config/*.yaml` 键集合
三处必须一致。该约束由 CI 作业 `param-contract`（`scripts/ci/check_params.py`）看守，不一致直接红：
改动任一参数时，必须在同一提交内同步本表、`declare_parameter` 与兵种配置。
（历史：旧校验脚本随工具包删除后曾一度退回人工核对，`decision.md` D16；D18 以零依赖方式恢复。）
**本表是实现之外的唯一参数清单。**

#### 本地化节点参数（`foray_localization`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_manifest` | `""` | 先验地图 `manifest.yaml` 绝对路径；为空时启动失败。由 launch 的 `map_manifest:=` 传入 |
| `map_frame` | `map` | 地图坐标系名 |
| `odom_frame` | `odom` | 里程计坐标系名（`point_lio` 输出） |
| `base_frame` | `base_footprint` | 机器人本体坐标系名 |
| `voxel_leaf_m` | `0.10` | 实时点云降采样体素（m） |
| `map_voxel_leaf_m` | `0.25` | 先验地图降采样体素（m） |
| `residual_threshold_m` | `0.30` | 配准残差超过该距离的点判为残差（m） |
| `cluster_tolerance_m` | `0.30` | 残差点欧氏聚类容差（m） |
| `verify_map_hashes` | `true` | 加载地图时校验 SHA-256；由 launch 的 `verify_map_hashes:=` 覆盖 |
| `pose_source` | `auto` | `auto` / `lidar` / `external`；步兵/重装用 `external` |
| `ok_mean_residual_m` | `0.15` | 进入 `OK` 的平均残差上限（m）。不能低于先验地图自身离散度：0.25 m 体素图即便配准完全收敛，实测残差也有约 0.10 m（见 `decision.md` D22） |
| `degraded_mean_residual_m` | `0.30` | `DEGRADED` 可接受的平均残差上限（m） |
| `ok_inlier_ratio` | `0.55` | 进入 `OK` 的内点率下限 |
| `degraded_inlier_ratio` | `0.35` | `DEGRADED` 的内点率下限 |
| `lost_timeout_sec` | `1.0` | 无有效配准超过该时长进入 `LOST`（s） |
| `odom_timeout_sec` | `0.5` | 里程计断流超过该时长进入 `LOST`（s） |
| `lidar_timeout_sec` | `0.2` | 点云断流超过该时长即视为无有效配准（s） |
| `warmup_frames` | `3` | `INITIALIZING → OK` 所需连续好帧数 |
| `recover_frames` | `3` | `DEGRADED → OK` 所需连续好帧数 |
| `align_max_iterations` | `10` | 配准后端（默认 PCL GICP）最大迭代次数 |
| `align_max_correspondence_m` | `0.5` | 配准后端最大对应点距离（m） |
| `fov_horizontal_deg` | `360.0` | 水平视场开口角（度）。`360` 表示不裁切；小于 `360` 时按 `base_frame` 前向为 0° 裁切点云后再配准（见 `decision.md` D23） |
| `fov_vertical_deg` | `180.0` | 垂直视场开口角（度）。`180` 表示不裁切 |
| `fov_center_yaw_deg` | `0.0` | 视场中心相对 `base_frame` 前向的偏航（度），用于把裁切窗口转向感兴趣方向 |
| `observability_ratio_threshold` | `0.0` | 点面残差信息矩阵归一化特征值阈值，兼作可观测性总开关。`0` 表示不启用方向约束（行为与改造前一致） |
| `min_source_points` | `0` | 单帧源点数下限，低于即降级为 `DEGRADED(insufficient source points)`。`0` 表示不设下限 |
| `max_unobservable_dofs` | `6` | 允许的不可观自由度上限（0..6），超过即降级为 `DEGRADED(insufficient observability)`。`6` 表示不启用 |
| `profile_enabled` | `false` | 分段耗时统计开关：每 5 s 打印一行 `to_pcl / voxel / align` 平均耗时与配准率。默认关，关闭时只有一次判空开销（`decision.md` D25） |
| `filter_enabled` | `false` | 融合层 6 维 EKF 开关。`false` 时位姿更新与未引入滤波时逐字节一致（见 `decision.md` D24） |
| `filter_measurement_noise_m` | `0.05` | EKF 平移量测噪声标准差（m）。默认取 0.25 m 体素先验图静止配准误差的实测值（D22/D26） |
| `filter_unobservable_noise_scale` | `10.0` | EKF 对不可观方向的量测噪声倍数（标准差倍数），决定该方向权重有多小；权重本身另有 1 % 硬上限，不随外推时间回升（`decision.md` D46） |
| `min_cluster_size` | `5` | 有效残差簇的最小点数 |
| `max_cluster_size` | `500` | 有效残差簇的最大点数（更大视为地图差异） |
| `state_rate_hz` | `50.0` | `LocalizationState` 发布频率（Hz） |
| `tf_rate_hz` | `20.0` | `map → odom` TF 发布频率（Hz） |
| `align_rate_hz` | `5.0` | 配准频率上限（Hz） |
| `obstacle_rate_hz` | `10.0` | 动态物体话题发布频率（Hz） |
| `cloud_topic` | `/livox/lidar` | 输入点云话题 |
| `odom_topic` | `/aft_mapped_to_init` | 输入里程计话题 |
| `external_pose_topic` | `/localization/external_pose` | 输入外部位姿话题 |

#### 地图服务节点参数（`foray_map_server`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_manifest` | `""` | 先验地图 `manifest.yaml` 绝对路径；由 launch 的 `map_manifest:=` 传入 |
| `verify_map_hashes` | `true` | 加载地图时校验 SHA-256 |
| `publish_rate_hz` | `1.0` | `/localization/map_meta` 重复发布频率（Hz） |

> 阈值均以 2027 赛季 12 m × 8 m 场地与 MID360 噪声水平为初值，
> 赛前用录包标定后写入 `bringup/config/*.yaml`，**魔法数字必须注明来源**。
> 兵种配置只需给出「非 launch 提供」的那些参数；`map_manifest` 与 `verify_map_hashes`
> 由 launch 参数注入，配置里出现属于覆盖，不出现也合法。

## 地图

3D 先验点云（PCD）与 2D 栅格（PGM/YAML）由**同一次建图**投影得到，
**同源、同坐标系、同原点**。地图不入 git，按 manifest 版本化分发。
建图与地图校验工具**不在本仓**（`decision.md` D16），本仓只按下列契约加载地图。

```yaml
# manifest.yaml 示例
version: "2027.1"
frame_id: "map"
origin: [-6.0, -4.0, 0.0]   # 2D 栅格左下角在 map 系的位置 [x, y, yaw]
resolution: 0.05
width: 240
height: 160
pcd: "map.pcd"
grid: "map.pgm"
pcd_sha256: "<sha256>"
grid_sha256: "<sha256>"
created_at: "2026-10-25"
source: "RMUL 2027 field survey"
```

坐标系约定：`map` 原点为**场地中心**，`+X` 红方 → 蓝方，`+Y` 红方左侧，`+Z` 向上。
2D 栅格的 `origin` 是左下角像素在 `map` 系的位置，`resolution` 单位为 m/px。

加载器把 PCD 头当不可信输入：点数上限 2×10⁷、数据上限 512 MiB、单字段 ≤ 32 字节，
头部畸形或声明与实际不符一律加载失败并进入「地图不可用」——**宁可启动失败，也不静默
接受半张地图**（`decision.md` D41）。

ascii 段里字段数不足的行、以及 x/y/z 为 NaN/inf 的点会被跳过（这类行本来就不是有效点）：
装载仍然成功，但跳过行数会记进 `PcdLoadResult::skipped_lines` 并在装载时 **WARN 出来**
（`decision.md` D54）。「地图比声明少几个点」不能是没人知道的事实。

## 健康契约

| 状态 | 条件（持续去抖） | `pose_valid` | 下游行为 |
|---|---|---|---|
| `INITIALIZING` | 启动/重定位后，尚未连续 `warmup_frames` 帧有效配准 | `false` | 不得用于决策，等待 |
| `OK` | 平均残差 ≤ `ok_mean_residual_m` 且内点率 ≥ `ok_inlier_ratio` | `true` | 正常消费 |
| `DEGRADED` | 残差或内点率越界，但仍在 `degraded_*` 内；≤ 200 ms 公告 | `true` | 降级消费，规划减速 |
| `LOST` | 连续 `lost_timeout_sec` 无有效配准，或 odom 断流 > `odom_timeout_sec`、点云断流 > `lidar_timeout_sec` | `false` | 停止消费，显式失效 |

`LOST` 的 `reason` 区分三种成因：`no odometry received`（从未收到里程计）、
`odometry stale: <age>s (no samples)`（真的没有新帧）、
`odometry stale: <age>s (processing backlog <lag>s)`（帧在队列里积压，即本节点处理不及）。

`DEGRADED`/`OK` 的残差与内点率判据只适用于带配准质量的位姿源；外部位姿没有可测质量，
在有外部位姿的配置里健康状态只判新鲜度，`confidence` 按状态给 0.5/1.0（`decision.md` D38）。

地图不可用是 `LOST` 的另一条成因：`map_manifest` 未设置、sha256 校验失败、点数超限或
加载异常，都会把状态锁进 `LOST`，reason 以 `map unavailable:` 开头。此时 `map → odom`
**一帧都不发**（`decision.md` D36）。

算法路径（配准、残差聚类、可观测性）与里程计、点云、发布路径必须分属不同回调组并跑在
多线程执行器上：5 Hz 配准单帧可达上百毫秒，任何让里程计回调排队的实现都会把积压误报成
`odometry stale`（`decision.md` D30、D50）。

`LocalizationState` 的字段：`header`（`stamp` 为状态快照时刻）、`pose`（`map → base_frame`）、
**`pose_valid`**、`confidence`、`age_sec`（距上次有效配准）、`state`、`source`、`reason`。

**位姿消费约定（强制）**：下游**必须**先判 `pose_valid`，为 `false` 时禁止把 `pose`
用于闭环控制、路径规划或决策——`INITIALIZING`/`LOST` 时该字段仍会带着未经校正或已停更的
合成位姿，只为 rosbag 复盘与诊断保留。TF `map → odom` 在 `LOST` 下停发，下游若绕过
`pose_valid` 直接查 TF 会拿到过期结果。**失效标志不可被下游静默忽略。**

**协方差契约（强制）**：`pose.covariance` 是 6×6 行主序（`x, y, z, rotX, rotY, rotZ`），
只填对角——配准残差没有给出方向间的相关性。对角按最近一帧的残差与内点率给：位置方向
σ = `max(残差, 1 cm) / sqrt(内点率)`，姿态方向 σ = 位置 σ / 1 m 杠杆；该帧不可观的自由度
再按 `filter_unobservable_noise_scale` 的平方放大。**没有任何配准质量时（从未配准、
`INITIALIZING`/`LOST`、外部位姿源）对角写 1e3，绝不写 0**——0 方差在
`PoseWithCovariance` 里等于「无限权重」，会把未知量伪装成绝对真值（`decision.md` D43）。
按协方差加权的下游可以据此降权，但**不得**把它当成「位姿一定可用」的判据——那是
`pose_valid` 的职责。

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | X2（foray_interfaces）· L1（foray_platform）· L2 |
| **第三方** | Eigen3 · yaml-cpp · PCL（GICP 默认后端，必需，≥1.10）· point_lio（pin）· ONNX Runtime（可选，默认 OFF） |

- `point_lio` 以独立进程运行，按 `foray_ws` 的 pin 构建；本仓只消费其话题。
- PCL 由 `package.xml` 的 `libpcl-all-dev` 经 rosdep 安装，是默认配准后端 GICP 的依赖；
  `colcon build --cmake-args -DFORAY_WITH_PCL=OFF` 退回内置点到点 ICP（精度较低，仅用于
  无 PCL 的最小构建与降级排查）。这是**编译期**选择，不做运行时静默回退：
  找不到 PCL 时 configure 直接失败，而不是悄悄换成另一个后端（decision.md D21）。
- `ONNX Runtime` 仅 `foray_localization_relocalization` 使用，`FORAY_WITH_ONNX=ON` 时编译。
- 验证过的构建环境是 Linux（ROS 2 Humble；CI 用 `ros:humble` 容器）。Windows 上
  `scripts/ci/` 的四道门禁与 pytest 自检可以正常跑（`.gitattributes` 已把检出换行钉成 LF，
  门禁按字节判 CRLF，不依赖平台的换行翻译），但 **ROS 2 + PCL 的 Windows 构建从未验证过**；
  要在 Windows 上试，先用 `-DFORAY_WITH_PCL=OFF` 把 PCL 摘掉，别一上来就卡在找 PCL。

## 纪律

- **场地坐标系原点全队统一**，对齐规则定义的场地原点（**不是**机器人启动位置）——这是机间共享坐标的前提
- 2D 栅格与 3D 点云**必须同源同坐标系**，否则态势融合无法工作
- 需区分「**地图过期**」与「**动态物体**」：残差聚类 + 持续时间判定 + 先验「允许动态区域」标注
- 已删除：`terrain_analysis` / `terrain_analysis_ext` / `IntensityVoxelLayer` / `pointcloud_to_laserscan` / 在线 SLAM

## 上下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `foray_interfaces` · `foray_platform` |
| **下游**（谁依赖本仓） | `foray_vision` · `foray_navigation` · `foray_decision` · `foray_robots` · `foray_ws` |

## 参考

- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
