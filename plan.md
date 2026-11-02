# 开发计划

## 任务列表

- [x] **M0 骨架与文档**：package 目录/CMake/package.xml、CI 接入、README/tree/decision 对齐
  （M0 时 6 个包；仿真与离线工具包已于 2026-10-27 移除，现 5 个，见 `decision.md` D16）
- [x] **M1 里程计与地图管线**：point_lio 话题接入、`build_map`/`validate_map`、manifest 格式冻结
- [x] **M2 核心与配准**：健康状态机、位姿源仲裁、manifest 校验、GICP（PCL 后端 + ICP 兜底）、残差点
  （PCL GICP 已于 2026-10-30 实装并成为默认后端，`-DFORAY_WITH_PCL=OFF` 退回内置 ICP；见 `decision.md` D21）
- [x] **M3 ROS 层**：LocalizationState/MapMeta/DynamicObstacle/Relocalize 消息、主节点、map_server、TF/QoS
- [x] **M4 动态物体与重定位**：残差聚类发布、Relocalize 服务与状态机联动、允许动态区域
- [ ] **M5 HIL 验证**：录包回放、ATE/RPE、DEGRADED/LOST 时序验证
  - ~~**M5a 离车仿真闭环**~~：已删除（2026-10-27，见 `decision.md` D16）——合成点云回放、
    真值里程计注入、ATE/RPE 评估脚本与 CI `sim-smoke` 整体移出本仓
  - [x] **M5b Gazebo 联调**：由下游仿真仓 `foray_simulation` 承接并已取证（其需求 A2 就是本仓的
    仿真验收口径：10 s 内状态 `OK`、位置误差 < 0.30 m、朝向误差 < 5°）——场景与话题映射在
    **本仓之外**完成，本仓只提供可订阅的定位节点（`decision.md` D16）
  - [ ] **M5c 实车 bag 验证**：现场录包后复算阈值并写回 config
- [ ] **M6 调参与交付**：2027 场地地图、阈值标定、运维文档、兵种 launch 验收（3v3：步兵/哨兵）
- [x] **M7 布局与命名规范化**：包迁入 `src/`（仓根即 colcon 空间根，现 5 个包）、
  `foray_localization_place` → `foray_localization_relocalization`、CI `dir-layout` 作业、
  文档与配置同步（见 `decision.md` D12）
  - [x] **M7a CI 首次真实运行修复**：补仓根 `requirements.txt`（test-python 缺 PyYAML）、
    清 ruff / clang-format 违规、脚本记录 100755、`pyproject.toml` 固定 Python 3.10
    （见 `decision.md` D13）；`requirements.txt` 与 `test-python` 已于 2026-10-27 删除（D16）
- [x] **M7b 审计报告修复**（报告为一次性分析，本地留存、不入库；口径见 `decision.md` D14）
  - [x] **P0 A1** 可执行文件装到 `lib/<pkg>`，`ros2 run` / `launch` 恢复（`3538e2a`）
  - [x] **P0 A2** 健康状态对时间戳小幅反序容错，去掉 `age_seconds = +inf` 的永久 LOST（`25b622c`）
  - [x] **P0 A3** 仿真射线投射改用传感器系点云，合成点云随位姿变化（`99da264`）
  - [x] **P1 B1** 点云断流显式降级：`lidar_timeout_sec` + `publish_state()` 兜底（`42c13c9`）
  - [x] **P1 B2** `LocalizationState.pose_valid`：`LOST` / `INITIALIZING` 显式 `false`，
    README 写入「必须检查」履约约定（`decision.md` D14）
  - [x] **P1 B3** README 参数表按实现回写（原 `check_param_consistency.py` 随工具包删除，D16；
    该约束已由 CI `param-contract` 作业以零依赖方式恢复，D18），
    `tree.md` 修正节点 main 文件名（`decision.md` D15）
  - [ ] **P2 C1/C2/C3** converged 判据、首帧/重定位对应距离、launch 级集成测试 —— 未开工
- [x] **M8 文档与接口漂移门禁**：仓级 `scripts/ci/` 四个零依赖脚本（`check_style` /
  `check_contracts` / `check_params` / `check_tree_doc`）、`contracts.sha256` 接口快照、
  CI 四个作业（`file-hygiene` / `doc-sync` / `param-contract` / `contract-lock`），
  `tree.md` 双向一致（见 `decision.md` D18）
- [ ] **M9 小 FOV 可观测性定位 · EKF 融合 · 性能**（`decision.md` D23/D24/D25）——进行中
  - [x] **i1 需求入档**：README「定位方案」补齐「位姿滤波」「可观测性与裁切」两节；
    `decision.md` 新增 D23（可观测性与方向约束）、D24（融合层 EKF）、D25（性能与可选后端）
  - [x] **i2 可观测性基础设施**：点面残差 6×6 信息矩阵、特征分解、不可观方向标签化、
    target 法向与点云索引随地图生命周期缓存
  - [x] **i3 FOV 裁切与方向约束**：`crop_by_fov`、修正量投影到可观子空间、健康契约增加
    「输入点数不足」「可观性不足」两条显式降级判据
  - [x] **i4 FOV 极限扫描实验**：静止条件下扫 360/180/120/90/60/45/30 度共九组，
    全部 `100% OK`、误差 0.04–0.12 m（A2 判据 0.30 m）；方向约束只在 60° 及以下有收益
    （0.118 → 0.066），120° 反而劣化。结论见 `decision.md` D26
  - [ ] **i5 EKF 融合层**：常值预测、按可观性逐方向放大量测噪声、重定位硬切与重置、
    `filter_enabled: false` 时与改造前逐字节等价的直通路径
  - [x] **i6 性能零依赖优化**：逆矩阵缓存、`PointCloudIndex` 与 target 法向复用、
    `setInputTarget` 跨帧复用可行性验证；逐项独立提交并给分段耗时表
  - [x] **i7 可选 small_gicp 后端评估**：`FORAY_WITH_SMALL_GICP`（默认 OFF），
    实测提速 < 15% 则不引入
  - [ ] **i8 GUI 审查产物**：补 `odom → base_footprint` 后尝试 RViz；不可用则交付
    monitor 四联图 + 自包含 HTML 报告
- [ ] **M10 审计整改 L1–L18**（`audit/tasks/02-foray_localization.md`，原则见 `decision.md` D28）
  - [x] **测试基建**：节点实现进库、可注入配准后端、`Diagnostics` 计数
    （`decision.md` D29）
  - [x] **P0 L1 执行器与回调组**：`MultiThreadedExecutor` + 独立回调组、聚类索引随地图建一次、
    odom 超时拆成「没收到」与「处理不及」两种 reason（`decision.md` D30/D35；
    回执：注入 200 ms 后端 6 s，父提交停在 `no valid registration`、积压 201.4 ms、
    状态 27.5 Hz；现为 `LOST=0`、积压 11.8 ms、状态 50.0 Hz）
  - [ ] **P1 L2/L3/L4/L5/L9/L10**：地图失败转 LOST 且 TF 只放行 `OK`/`DEGRADED`、
    `child_frame_id` 校验与 TF 补转换、位姿源仲裁接线（`select_pose_source`）、
    参数声明顺序、配准量测锁内快照、PCD 加载上限与异常安全
    - [x] **L2 地图不可用即 LOST、TF 闸门收紧**（`decision.md` D36）：三种地图失败
      （manifest 缺失 / sha256 不符 / PCD 截断）在 `test_localization_node_map_errors`
      上都是 `state=LOST` + `pose_valid=false` + `tf_published=0`，订阅侧 TF 树也查不到
      `map → odom`；父提交同用例 `state=INITIALIZING`、`tf_published=20`、TF 树查得到。
    - [x] **L10 PCD 头校验与异常安全**（`decision.md` D41）：三条硬上限 + `from_chars`
      不抛异常解析 + 二进制段先量剩余字节再分配 + ascii 实际行数校验 + 加载调用点
      `try/catch`。`test_pcd_loader` 4→8 条，父提交四条全红；节点级
      `OversizedPointCountSendsNoTransform` 覆盖「20 亿点地图 → 一帧 TF 都不发」。
    - [x] **L9 配准量测时刻对齐**（`decision.md` D40）：锁内快照 `odom_to_base_at_cloud`，
      量测换算不再读配准结束时的里程计。`MapToOdomJitterUnderConstantVelocity`（1 m/s、
      200 ms 后端、8 s/20 次配准）`map → odom` 极差父提交 3.2110 m、本提交 0.0000 m。
    - [x] **L5 参数声明顺序**（`decision.md` D39）：`min_source_points` /
      `max_unobservable_dofs` 的 declare 移到 `HealthMonitor` 构造之前。集成用例
      `MinSourcePointsParameterTakesEffect`：250 点进 `OK`、降到 30 点落
      `DEGRADED / insufficient source points`；父提交同用例停在 `OK`（参数完全无效）。
    - [x] **L4 位姿源仲裁接线**（`decision.md` D38）：`select_pose_source()` 每拍重算，
      `auto` 只在雷达断流时接受外部位姿；外部位姿不再伪造 `inlier_ratio=1.0`（改由
      `has_registration_quality=false` 走「只判新鲜度」路径）。三条集成用例在父提交上
      全红（`source=2, x=5.000` / `state=DEGRADED` 未接管 / 恢复后仍 `source=2`），
      现为 `source=1 x=0.000` / `source=2 x=5.000 state=OK` / `source=1`。
    - [x] **L3 里程计子帧与外参换算**（`decision.md` D37）：`Odometry.pose` 按
      `odom → child_frame_id` 解释，`child != base` 时用 TF 补 `T(base←child)` 并取逆乘进去。
      `test_localization_node` 两条用例：0.11 m 外参 → state x=0.890（父提交 1.000）；
      查不到 TF → 收到 78 帧、处理 0 帧、`tf_published=0`（父提交全部照收并发 TF）。
  - [ ] **P2 L6/L7/L8/L11/L12/L13/L14/L15/L16 + 附带(a)(b)**：点云消费与 TF 时间戳、
    协方差真实填充、odom QoS 对齐 `sensor_data`、ICP 弧度阈值、EKF 增益饱和与
    Joseph 形式、GICP 缓存指纹、可观性掩码统一到 `map → odom`、新鲜度判定统一时钟域、
    删除死声明
    - [x] **L6 点云消费与 TF 时间戳**（`decision.md` D42）：`has_cloud_` 与消费戳在放锁
      之前确定，`/tf` 的 `map → odom` 用被消费点云的时刻。`TransformIsStampedWithTheConsumedCloudTime`
      父提交 TF 戳与最近点云戳最大偏差 50.150 ms、本提交 0.000 ms；
      `CloudArrivingDuringAlignmentIsNotDropped` 父提交 600 ms 内只配到老帧（源点数 `{10}`），
      本提交配到了配准期间到达的 77 点帧。
    - [x] **L7 协方差真实填充**（`decision.md` D43）：`PoseUncertainty` + `pose_covariance()`
      取代 `covariance.fill(0.0)`，按残差 / 内点率填对角、不可观方向乘
      `filter_unobservable_noise_scale²`、无配准质量时写 1e3。节点级读数父提交
      `INITIALIZING cov[x]=0.000 / OK cov[x]=0.000000 / 外部位姿 cov[x]=0.000`，
      本提交 `1000.000 / 0.000100 / 1000.000`；`StateMessageCarriesTheCovariance`
      在父提交行为下报 `covariance[0] evaluates to 0`。
    - [x] **L8 里程计 QoS 对齐 `sensor_data`**（`decision.md` D44）：订阅从 `QoS(50)`
      reliable 改成 `SensorDataQoS()`，启动打一行实际 QoS。`BestEffortOdometryUpstreamIsReceived`
      父提交 `odom_received=0`（DDS 报 `RELIABILITY_QOS_POLICY` 不兼容）、本提交 146 帧全收；
      13 条既有用例仍用 reliable 发布并通过，证明兼容方向没写反。
    - [x] **L11 ICP 收敛判据拆米/弧度**（`decision.md` D45）：`AlignOptions` 增
      `transformation_epsilon_rad`（默认 1e-3 rad），与 `transformation_epsilon_m`（1e-4 m）
      分别比较；预算耗尽时把迭代次数与两项增量写进 `error`。`test_icp`
      `MeterThresholdDoesNotSetRotationTolerance` 父提交 `iterations=1`、残余角度 0.1838 rad
      就宣布收敛（本提交 10 次迭代、0.0000 rad）；`ExhaustedBudgetExplainsItself` 父提交
      `converged=0 error=''`（本提交给出完整阈值与增量）。注：审计说的「几乎不收敛」在合成
      点云上没有复现，实测定性为「量纲混用 + 失败无说明」，已写进 D45。
    - [x] **L12 退化抑制不再随协方差增长失效**（`decision.md` D46）：不可观方向增益加
      1 % 硬上限 `kMaxUnobservableGain`，`predict()` 给协方差对角加 `1.0 m²` / `0.25 rad²`
      上限。`UnobservableDirectionWeightStaysCapped` 父提交外推 100 s 后 1 m 量测让病态方向
      动 0.5025 m（隐含增益 0.502）、本提交 0.0100 m（0.010）；`CovarianceStopsGrowing`
      父提交 `cov[x]=250.0025 cov[rx]=40.0001`、本提交 `1.0000 / 0.2500`。
    - [x] **L13 EKF 入口挡非有限值、Joseph + 对称化**（`decision.md` D47）：`reset/predict/update`
      全判 `is_finite()`，非有限量测丢弃（不 reset、不透传）；协方差改 Joseph 形式并
      `(P+Pᵀ)/2`。父提交 `估计 x=nan`、`协方差有限=0`、`估计 x=9.0000 y=inf` 三条红；
      `CovarianceStaysSymmetricAndPositive` 两侧都通过，作为回归守卫保留。
    - [x] **L14 GICP 缓存键与参数下发**（`decision.md` D48）：target 缓存改成「地址 + 点数 +
      16 点内容指纹」，PCL 参数每帧下发并新增 `set_options()`，`nr_iterations_` 收进
      `GicpWithIterationCount`。`TargetCacheInvalidatesWhenContentChanges` 父提交
      `converged=0 iterations=0 inlier=1.000 error='GICP 未收敛（iterations=0）'`
      （对着旧靶标优化），本提交 `converged=1 iterations=1 inlier=1.000 x=0.050`。
  - [ ] **P3 L17/L18**：未匹配残差系数与 ascii 丢点计数（魔法数字注释 + 可观测）

- [ ] **P2 可选**：ONNX Runtime CPU 全局重定位（默认 OFF）

## 当前 Agent State

DELIVERED：5 个 package 全部位于 `src/`（仓根即 colcon 空间根），重定位包已更名为
`foray_localization_relocalization`。2026-10-27 按「只保留核心定位算法」移除合成仿真与
离线工具（`decision.md` D16）：无 `sim-smoke`、无建图/评估脚本。车下验证改为录制包回放。
2026-10-28 补上文档与接口门禁（`decision.md` D18），CI 共 10 个作业：`dir-layout` /
`file-hygiene` / `doc-sync` / `param-contract` / `contract-lock` / `gate-selftest` /
`lint-python` / `lint-c` / `build-ros2` / `build-nopcl`（作业清单以 `.github/workflows/ci.yml`
的 `jobs` 为准，此处只是便于扫读的复述；此前写成「8 个」并漏列 `gate-selftest` 正是靠人眼
同步数字的产物）。第十个作业 `build-nopcl` 是 2026-10-30 实装 PCL GICP 时加的（`decision.md`
D21）：默认的 `build-ros2` 走 `FORAY_WITH_PCL=ON` 守护 GICP 主路径，`build-nopcl` 走 `OFF`
守护内置 ICP 回退路径——只验一条路径等于另一条迟早烂掉。
M5b 的承接方 `foray_simulation` 已建成并取证（首次 `OK` 3.78 s、位姿误差 0.083 m、
`yaw` 0.012°，95 s 采样 4737 条状态中 4423 条 `OK`、零 `DEGRADED`/`LOST`）；M5c（实车 bag）
仍待实车条件。
验证环境：宿主无 ROS 2 / colcon / PCL，`colcon build` 与 `colcon test` 一律在容器里跑——
镜像 `foray_sim:humble-fortress`（Humble + Gazebo Fortress，含 PCL 1.12、gtest、colcon），
本仓与仿真仓一起挂到 `/ws/src/`。2026-10-30 实测：装 PCL GICP 前 5 个包构建 51 s、
73 tests / 0 errors / 0 failures；装之后同一口径（本仓 5 个包，`colcon test-result` 汇总）
默认 ON 报 **75 tests**、`-DFORAY_WITH_PCL=OFF` 报 **70 tests**，两轮均 0 failures——
差的那 5 条就是 `test_pcl_gicp`（4 个用例 + 该文件在 CTest 里的汇总条目），map 包单独口径
是 ON 34 / OFF 29。CI 的 `build-ros2` 保留，防的是「只在容器里验过」。

## 模糊点与待确认项

- `foray_interfaces` 的 0.2.0 消息尚未落地；首版消息暂放 `foray_localization_ros`，迁移计划见 `decision.md` D8
- `small_gicp` 由 `decision.md` **D25 立为可选的编译期后端评估**（`FORAY_WITH_SMALL_GICP`，
  默认 OFF，实测提速 < 15% 则不引入）——此前「不再计划」的表述已作废；PCL GICP 仍是默认后端
  （`decision.md` D21），`D9` 里「首版用 PCL GICP + 内置 ICP 兜底」的设想已在 D21 落地
- `bringup/config/heavy.yaml` 对应装甲兵种，不在 2027 3v3 阵容（步兵×2 + 哨兵×1，
  见 `foray_docs/algorithm_structure.md` §12）；保留为预留配置，不参与验收
- 2027 场地地图需赛前场地窗口实测；当前以录制包回放验证（无实车，M5 未完成）
- 自动全局重定位（ONNX）为 P2，默认关闭；许可证决策不在本期范围

## Acceptance Criteria

- [ ] 本仓能独立 `colcon build`（无代码时跳过）
- [ ] CI 绿灯
- [ ] 每个新增模块带独立可执行测试
- [ ] 录包回放 ATE ≤ 5 cm、航向 ≤ 2°（相对起点；仿真回放能力已移出本仓，D16）
- [ ] 定位初始化 ≤ 5 s；`LocalizationState` ≥ 50 Hz；GICP 5 Hz 且 p99 ≤ 40 ms
  （p99 缺实车/实图数据，暂以容器基准为参考；GICP 后端已于 2026-10-30 实装，见 `decision.md` D21）
- [ ] 配准失效 ≤ 200 ms 输出 `DEGRADED`、≤ 1 s 输出 `LOST`，无静默外推
- [ ] 同源地图 2D/3D 一致率 ≥ 95%，manifest 强校验
- [ ] 动态物体检出率 ≥ 90%（标注集）
- [ ] 兵种 launch 可分别启动（3v3：步兵/哨兵），参数错误有明确报错
