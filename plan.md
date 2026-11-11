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
- [x] **M10 审计整改 L1–L18**（`audit/tasks/02-foray_localization.md`，原则见 `decision.md` D28）
  2026-11-03 全部 20 条落地：22 个提交在 `fix/audit-localization-02`（PR #7，base 为审计基线
  `feat/fov-observability-ekf`），每条都留了「父提交红 / 修复后绿」回执；容器内
  `FORAY_WITH_PCL=ON` 145 tests、`OFF` 138 tests 均 0 failures，四道门禁全绿。
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
    - [x] **L15 可观性力臂改用被修正位姿的原点**（`decision.md` D49）：`analyze_observability()`
      增可选 `lever_origin`（默认仍是 `transform.translation()`），节点传 `map_to_odom_` 原点。
      回执：机器人距里程计原点 40 m 的「地面 + 单墙」场景，父提交语义 `unobs=5 cond=5.04e+06`、
      判断翻转 0/6；本提交 `unobs=1 cond=616`、翻转 4/6。另加「不传 `lever_origin` 与父提交
      逐位相同」的默认值守卫（`RotationLeverDefaultsToTheBodyPose`）。
    - [x] **L16 算法路径不再独占执行器**（`decision.md` D50）：`MultiThreadedExecutor`（线程数
      `clamp(hw, 2, 4)`）+ 六个互斥回调组（odom / cloud / align / obstacle / publish / service），
      配准与聚类各占一组。回执（150 ms 忙等配准 @ 5 Hz，2 s 窗口）：多线程 `LOST=0 state_hz=50.0
      max_odom_lag_ms=1.3 align_runs=19`；对照组（同一负载放单线程执行器）`state_hz=15.0`——
      对照组已写进同一条用例，作为「负载足以证明执行器起作用」的证据。
    - [x] **附带 (a) 聚类新鲜度改用节点时钟的收到时刻**（`decision.md` D51）：`on_obstacle_timer()`
      不再用 `now - header.stamp` 判新鲜度，闸门也从 `has_cloud_` 改为「收到过点云」。
      回执：传感器时钟偏移 −10 s 时，父提交 2000 ms 内障碍帧数 `0`，本提交 `95`，且发布戳与
      点云戳偏差 `0 ms`。
    - [x] **附带 (b) 删掉死声明 `update_health()`**（`decision.md` D52）：全仓 1 处声明、0 处定义、
      0 处调用；删除后构建与 142 条测试不变（死声明没有可执行的「改前坏」证据，回执即调用图）。
  - [ ] **P3 L17/L18**：未匹配残差系数与 ascii 丢点计数（魔法数字注释 + 可观测）
    - [x] **L17 未匹配点的残差占位变成具名常量**（`decision.md` D53）：提
      `kUnmatchedResidualMultiple = 4.0` 到公开头文件并写明「为什么是 4、为什么不能是 0/inf」。
      回执：父提交该表达式是裸字面量（`4.0 * config_.residual_threshold_m`，无来源注释）、
      头文件里没有这个常量；本提交新增用例 `UnmatchedPointsReportTheDocumentedResidual`
      把值钉成契约（无邻格簇 `1.200 m`、有邻格簇 `0.545 m`）。
    - [x] **L18 ascii PCD 坏行不再静默丢点**（`decision.md` D54）：新增
      `PcdLoadResult::skipped_lines`、`PriorMapLoadResult::skipped_pcd_lines`、
      `LocalizationNode::Diagnostics::skipped_pcd_lines`，两个节点在装载期 WARN。
      回执：父提交（把计数逻辑退回）`L18 回执(NaN 行): ok=1 装载=2 点 跳过=0 行`、
      `L18 回执(字段数不足): ok=1 装载=2 点 跳过=0 行`、节点级
      `state=1 tf_published=6 跳过行数=0` 且日志无 WARN，三条全 FAILED；本提交三条回执
      `跳过=1 行`、节点 `跳过行数=1` 且装载期打印
      `[WARN] 先验地图有 1 行被跳过（字段数不足或 x/y/z 非有限），装载点数 3`。
  - [x] **验收清单入口**：新增 `scripts/ci/test_ros_results.py`，把 `colcon test` 写下的
    gtest XML 汇总成覆盖清单，并按名字断言 20 条发现的验收用例都在（L14 两条只在
    `FORAY_WITH_PCL=ON` 时要求）。回执：`python3 -m pytest scripts/ci/test_ros_results.py -q -s`
    在 PCL ON 构建上 `结果文件: 18 个，用例 123 个，failures=0 errors=0` + `3 passed`；在
    `FORAY_RESULTS_ROOT=/ws/build-nopcl` 上 `17 个 / 117 个` + `2 passed, 1 skipped`（`decision.md` D55）。

- [ ] **M11 联调缺陷整改 L1–L8**（2026-11-11 仿真联调复盘；设计与取舍见 `decision.md` D56–D59）
  结论先说：A2 失败与 `ready=FAIL` 的根因都**不在配准算法**，在**帧契约与状态机表达**——
  仿真链里没有 `body` 帧，定位节点一帧点云都吃不到；节点又把「吃不到帧」表达成永不结束的
  `INITIALIZING`。补上帧之后 A2 仍掉锁，那是另一条独立成因（点数漏斗塌缩，见 i6）。
  顺序：i1 解 `ready` → i2/i3 健康契约 → i4/i5 判据与口径可见性 → i6 精度 → i7 验收可读性 → i8 收口。
  **进度（2026-11-11，宿主直接部署 ROS 2 humble，两个 colcon 空间 `foray_sim_ws` +
  `foray_simulation`）**：i1–i8 全部落地并提交（仿真仓 `fix/m11-body-tf`，本仓
  `fix/m11-health-contract`）；下列回执取自**已提交**的二进制。
  回执五（终验，ROS_DOMAIN_ID=66，判据修正后的最终代码）：`ready=PASS ready_delay_s=0.93
  lidar_s=0.13 localization_s=0.93 bt_navigator=active controller_server=active
  last_state=OK state_reason="warmup complete"`；`a2 samples=8336 max_xy_err=0.039
  max_yaw=0.154 lost=0 verdict=PASS`；漏斗 `raw=18818 crop=18818 voxel=3197
  align_source=1697 align_target=4137`；`tf unavailable` 0 条、LOST 0 条（正向链路无
  误报）。
  回执一（整链就绪，`navigation_sim.launch.py headless:=true`）：
  `ready=PASS ready_delay_s=21.09 lidar_s=0.71 localization_s=1.05 bt_navigator=active
  controller_server=active last_state=OK state_reason="warmup complete"`；`tf2_echo
  base_footprint body` → `Translation: [0.000, 0.000, 0.000]`、四元数 `(0,0,0,1)`。
  回执二（`profile_enabled:=true`）：`点数漏斗: raw=18491 crop=18491 voxel=3141
  align_source=1695 align_target=4137`，全程无「配准无有效对应」，退化从 5/6
  （condition≈1.0e10）降到 2/6（condition≈3.3e3）。
  回执三（A2）：`samples=14394 first_ok_delay=0.00 max_xy_err=0.035 max_yaw=0.139
  last_state=OK lost=0 verdict=PASS`——对照整改前 `samples=4161 max_xy_err=0.076
  lost=5571 verdict=FAIL`。
  回执四：`colcon test --packages-select foray_localization_core` → 5/5 通过，
  `test_health.xml` 的 `HealthMonitor` 12 例（含 i3 新增两例）全绿。
  - [x] **i1 L1/L2 仿真 `body` 帧契约**（跨仓：`foray_simulation`）：`point_lio` 硬编码
    `body`（`laserMapping.cpp:228` 点云帧、`:270` 里程计子帧），而
    `localization_sim.launch.py` 只发了 `base_footprint→front_mid360`。修法：launch 增一条
    `base_footprint→body` 恒等静态 TF（**父 = `base_footprint`，方向不可反**，D56）。
    已验证的一条：`point_lio` 自己也广播 TF（`laserMapping.cpp:280-292`，受 `publish.tf_send_en`
    控制、默认 `true`），但发的是 `camera_init → aft_mapped`，**不是 `body`**——`body` 只作为
    odom 的 `child_frame_id`（`:270`）与点云 header（`:228`）出现，TF 树里根本没有它。所以补
    这一条不会给任何 frame 造出第二个父；`camera_init → aft_mapped` 是一对孤立的两节点树，
    没有订阅者跨树查询就无害。实车链**不是**「本来就有」：`point_lio` 从不发布 `body` 的 TF，
    实车同样缺这条父级（只是还没人在实车上跑过定位），谁补要在实车 launch 里落实。
    断言：仿真仓加静态用例「launch 里存在 `base_footprint→body`」；验收：`accept.py ready`
    由 `FAIL` 转 `PASS`（手工补帧实测 `ready_delay_s=1.51`）。
  - [x] **i2 L3 TF 缺失要进状态机、要带 reason**（本仓）：`lookup_base_from()`
    （`localization_node.cpp:302-322`）失败只 `WARN_THROTTLE` 后 `return`，两个回调都在
    「设 `has_cloud_`/`has_odom_` 之前」早退（`:354-360`、`:400-411`）；而 `publish_state()`
    （`:670-741`）四支成因全部要求「至少收到过里程计或点云」，于是一条都不命中，状态冻在
    `INITIALIZING`。修法：节点侧留一条 TF 失败记录（`child` / tf2 原文 / 时刻 / 次数），
    `publish_state()` 增一支「TF 缺失」→ `enter_lost("tf unavailable: <base>←<child>: <detail>")`；
    其余分支在 TF 失败未消除时把同一条成因追加进 reason。`LocalizationState` **不动**：
    `reason` 是既存字段（`msg/LocalizationState.msg:35-36`），`state_name` 也不加——下游本来就
    按常量映射名字，同一事实存两份正是 `AGENTS.md` 点名要防的分叉。README 同步补两句：
    `### TF` 表里点云帧那一层由上游给（本仓只发布 `map → odom`），以及「查不到就丢帧、并按
    TF 缺失报 `LOST`」——本仓需求文档现在只有 `odom → lidar` / `base_footprint → lidar`，
    缺的正是 `body` 这一条前提。验收：节点用例改「不发 TF」
    → `state=LOST` 且 reason 含 `tf unavailable`（现基线停在 `INITIALIZING`，红 → 绿）。
    回执：仿真仓 `body_tf:=false` 故障注入后 `state: 3`、
    `reason: 'tf unavailable: base_footprint<-body: "body" passed to lookupTransform argument
    source_frame does not exist.'`，`accept.py ready` → `last_state=LOST
    state_reason="tf unavailable: ..."`；对照整改前同一注入下 `last_state=INITIALIZING`。
    实现细节修正一处：判据只看「此刻 TF 是否通」，不看 `has_odom_`——里程计照常到时按
    激光断流报会把排查方向指错（D60）。
  - [x] **i3 L4 启动期 `LOST` 可自恢复**（本仓）：`health.cpp:172-176` 一进 `LOST` 就不再
    看样本，`reset()` 是唯一出口；而启动期 TF 晚到几秒就够判 `LOST`（`:241-245`），帧全到位后
    也回不来（实测 `last_state=LOST` 直到超时）。修法：区分「从未成功过」与「成功过又丢」——
    前者按 `warmup_frames` 连续好帧自恢复（reason 写明是启动期恢复），后者保持粘滞、只能走
    `Relocalize`（D57）。验收：`test_health` 增两条（启动期 `LOST` → 3 好帧 → `OK`；
    `OK` 过之后 `LOST` → 3 好帧 → 仍 `LOST`）。
  - [x] **i4 L5 退化判据必须真判**（本仓）：`max_unobservable_dofs` 默认 6 = `kDofCount`
    （`observability.hpp:30`），`health.cpp:189-190` 的比较恒真，`sentry.yaml:42` 又显式写 6，
    于是「6/6 全不可观」也算 `observable_ok`；实测 5/6 不可观、condition 1e10 仍报 `OK`，
    位姿靠方向冻结继续输出。修法：`sentry.yaml` → `max_unobservable_dofs: 3`、
    `min_source_points: 100`（64–70 点的帧不许静默进配准）；步兵/重装两份配置保留 6 但注明
    「方向约束未启用」；节点启动打一行有效判据；`check_params.py` 增语义断言
    「`observability_ratio_threshold > 0` ⇒ `max_unobservable_dofs < 6`」，防下一份配置再把它
    关掉（D58）。**默认值不动**（仍 6 = 不启用），参数表与 `param-contract` 的数值不受影响。
  - [x] **i5 L6 两套超时口径要能分辨**（本仓 + 跨仓）：`sentry.yaml:32-34` 是 1.0/0.5/0.2，
    `localization_sim.launch.py` 覆盖成 5.0/2.0/2.5（各有实测依据）。数值不改，改**可见性**：
    节点启动打一行「lost / odom / lidar 时限 + 来源」；仿真仓把三个覆盖值提成
    `DeclareLaunchArgument`（默认仍 2.0/2.5/5.0），复现实车口径不必改代码；参数表加脚注
    说明仿真链路由 `foray_simulation` 覆盖（D59）。
  - [x] **i6 L7 A2 掉锁的真因：先测量再改**（本仓 + 跨仓）：两条嫌疑已排除——
    **出生点不在图外**：先验图 `rmul_2027.pcd` 102433 点，`x∈[-0.30,12.30]`、`y∈[-0.30,8.30]`、
    `z∈[-0.10,0.40]`，spawn `(6.0,4.0)` 在场心，10 m 内 102433 点、3 m 内 26594 点、前向 30°
    楔形内 10036 点；**裁切逻辑没写错**：`crop_by_fov`（`point_cloud.cpp:83-131`）就是
    「相对中心 ±半个开口角」，四条单位测试覆盖。剩下的嫌疑全在点数漏斗：`/livox/lidar`
    （gz 1875×32 全 360°）→ `ign_sim_pointcloud_tool` 按环重排（`n_scan=32`、
    `ang_res_y=1.903225806451613`、`ang_bottom=7.0`，该工具会丢 row_id 越界的回波）→
    `to_point_cloud` → `crop_by_fov` 30° → `voxel` 0.10 m。修法：`profile_enabled` 那行扩成点数漏斗
    （raw / crop 后 / voxel 后 / 配准 source、target），**不新增参数**；仿真口径把
    `fov_horizontal_deg` 覆盖成 `360.0`（gz 雷达全视场，与实车 Livox 稀疏扫描不是一回事，D59）。
    验收：`accept.py a2` 的 `source` 回到四位数、`lost=0`、`verdict=PASS`；拿到漏斗数据前
    **不动** `voxel_leaf_m`：`align_max_correspondence_m: 0.5` 约等于 2× 地图体素 0.25 m，本身合理。
  - [x] **i7 L8 验收脚本带 reason**（跨仓：`foray_simulation`）：`accept.py:124-133` 只存
    `last_state` 名字，`:223/:275/:301` 也只打这一项，于是「为什么 LOST」得回翻定位节点日志。
    修法：`_on_state` 连 `msg.reason` 一起存，`ready`/`a2`/`a3` 输出 `state_reason`。
  - [x] **i8 验证与回执**：容器 `foray_sim:humble-fortress` 内 `FORAY_WITH_PCL=ON` / `OFF` 两轮
    `colcon test`（本仓现口径 145 / 138，新增用例计入）；`scripts/ci/test_ros_results.py`
    的名字清单同步登记新用例；四道门禁 + 仿真仓 `accept.py ready` / `a2` 回执贴进本条。
    回执（2026-11-11 宿主直跑，取代容器口径）：`colcon test --base-paths src/foray_localization`
    → ON 143 例、`--build-base build-nopcl -DFORAY_WITH_PCL=OFF` → OFF 119 例（含新增用例）。
    两轮都只剩 `test_localization_node` 的两条实时性断言（`max_odom_lag_sec < 0.05`）在宿主
    负载 ~12 时超线（60 ms / 83 ms）；**基线对照**：`git stash` 后同一份 dev 代码在同一负载下
    同样失败这 2 例，单独运行（负载回落）则两条全绿（35.2 ms / 40.7 ms）⇒ 属宿主机负载敏感的
    既有用例，不是本轮改动引入，容器 CI 口径不受影响。`test_ros_results.py` 已登记 i3 的三例。
    四道门禁：`check_style` / `check_tree_doc`（104 文件、24 目录）/ `check_params`
    （2 节点、43 参数、3 份配置）/ `check_contracts`（5 个接口文件）全 exit 0。

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
2026-11-03 完成审计整改 M10：节点级测试基建落地后，`colcon test` 口径从 73 条涨到
**145 条**（`FORAY_WITH_PCL=OFF` 138 条），20 条发现的验收用例全部按名字被
`scripts/ci/test_ros_results.py` 断言存在（`decision.md` D55）；健康契约、协方差契约、
执行器要求与地图硬上限都已写进 `README.md`，取舍记在 `decision.md` D30–D55。

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
