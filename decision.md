# 关键工程决策日志

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。
>
> 本文件自定的补充约定（2026-10-30）：条目之间用 `---` 分隔；`Context` / `Decision` / `Reason`
> 必填，`Alternatives` / `Rejected` / `Evidence` / `Consequences` / `Status` 按需。
> **编号只增不重排**：条目作废时保留原号并标 `Status: superseded by Dxx`，空号同样保留
> （D17 的缘由见 D18 编号说明）。决策只在这里写一次——`README.md` / `plan.md` / `tree.md`
> 需要时引用编号，不复述理由。

---

## Decision

Date: 2026-02-19

Context: 本仓从 0 初始化。需要确定它在整体架构中的位置与依赖边界。

Decision: 本仓定位为 **L3** 层，owner 角色为「导航」，
依赖层级 X2、L1、L2。

Reason:
- 切分判据 C1–C6 见 `foray_docs/repository_structure.md` §1
- 粒度结论：**一个仓 = 一个团队角色 + 一个变更率档位**
- **安全关键**——激光只做定位，构成单点故障

Alternatives:
- 并入相邻仓（减少仓库数量，但违反团队边界或变更率差异）

Rejected:
- 按 ROS 2 package 粒度切仓 —— 会导致版本矩阵地狱
- 按分层机械切仓 —— 会切断团队边界（Conway）

---

## D1 定位服务对位姿源解耦

Date: 2026-10-25

Context: 3v3 中哨兵装 MID360，步兵/重装无雷达，由里程计/视觉提供位姿；
后续还要接 UWB、视觉位姿。

Decision: 核心健康契约 `LocalizationState` 与位姿源无关，统一以
`source`（`SENTRY_LIDAR` / `EXTERNAL_ODOM`）区分；`foray_localization_core`
不依赖任何 ROS 类型。

Reason:
- 下游（导航/决策/视觉）只消费统一契约，不关心来源
- 步兵/重装可复用同一状态机与新鲜度/失效逻辑

Alternatives:
- 为每兵种单独定义消息/节点

Rejected:
- 让下游按话题名猜来源——会把兵种差异泄漏到所有消费者

---

## D2 point_lio 只做里程计，配准独立成层

Date: 2026-10-25

Context: point_lio 支持先验 PCD，但每帧在 LIO 内更新地图会与「固定先验地图」目标冲突，
且赛时不允许在线 SLAM。

Decision: `point_lio` 只输出 `odom → lidar`；`foray_localization_map` 独立做
先验地图 GICP（5 Hz），`foray_localization_ros` 合成 `map → odom`。

Reason:
- 先验地图保持只读，避免漂移污染与资源竞争
- 配准频率与 LIO 解耦，可单独限流/降级
- 失效时能明确区分「LIO 漂移」与「配准失败」

Alternatives:
- point_lio 内置先验 PCD 直接输出 map 位姿
- LIO-SAM / FAST-LIO2 整栈替换

Rejected:
- 内置先验 PCD 在长走廊/大场景退化，且无法独立观测配准质量
- 整栈替换会牵动点云预处理与 IMU 标定，超出本期

---

## D3 首版固定出生点 + 手动 Relocalize，不做自动全局重定位

Date: 2026-10-25

Context: 2027 3v3 场地 12 m × 8 m，出生点固定；自动全局重定位需要地点识别/描述子，
增加启动时间与误匹配风险。

Decision: 启动用 `initial_pose` 固定初值；丢失后通过
`/localization/relocalize` 服务注入初值并回到 `INITIALIZING`。
自动全局重定位（ONNX Scan Context）为可选 P2，默认关闭。

Reason:
- 赛时确定性优先；固定出生点可把「初始化」变成可复现步骤
- 手动重定位责任明确，便于操作手/裁判流程配合

Alternatives:
- 启动即自动全局重定位
- 依赖 UWB/视觉给初值

Rejected:
- 自动重定位误匹配会静默给出错误位姿，违背安全关键原则
- UWB/视觉初值作为增强，不作为首版依赖

---

## D4 地图资产 = 3D PCD + 2D 栅格 + manifest

Date: 2026-10-25

Context: 下游态势融合需要 2D 栅格，配准需要 3D 点云；两者若分别建图会坐标系不一致。

Decision: 同一次建图投影出 2D 栅格；manifest 记录版本、`frame_id`、`origin`、
`resolution`、`width/height`、PCD/栅格 sha256、建图时间与来源。
PCD/PGM 不入 git，按 manifest 分发。

Reason:
- 保证「同源、同坐标系、同原点」
- sha256 可在启动时强校验，防止地图/参数错配

Alternatives:
- 2D 与 3D 分别建图
- 地图直接入 git

Rejected:
- 分别建图会产生不可检测的错位
- 大文件入 git 违反仓库约定

---

## D5 场地坐标原点 = 场地中心

Date: 2026-10-25

Context: 规则未定义坐标原点；各队常以出生点/机器人启动位置为原点，导致机间坐标不共享。

Decision: `map` 原点为**场地中心**，`+X` 红方 → 蓝方，`+Y` 红方左侧，`+Z` 向上；
2D 栅格 `origin` 为左下角像素在 `map` 系的位置。

Reason:
- 机间共享坐标的前提；与规则场地尺寸（12 m × 8 m）对齐
- 红蓝对称，便于镜像/换边

Alternatives:
- 以红方出生点为原点
- 以机器人启动位置为原点

Rejected:
- 出生点/启动位置因兵种和操作而异，无法作为全队契约

---

## D6 动态物体 = 配准残差聚类，无标号

Date: 2026-10-25

Context: 赛时动态物体包括敌方机器人、裁判系统道具等；语义分割需要训练数据与算力。

Decision: 对配准后残差点做体素/欧氏聚类，输出质心、尺寸、点数、置信度，
**不输出类别**；用持续时间与「允许动态区域」标注抑制地图过期/反光噪声。

Reason:
- 无需训练、对任意障碍有效，CPU 可实时
- 无标号避免下游误用类别，由决策层结合视觉再分类

Alternatives:
- 训练点云语义分割
- 直接减去队友广播位姿

Rejected:
- 语义分割数据/算力成本高，且赛场外观域差异大
- 队友位姿只能去掉己方，不能覆盖敌方/道具

---

## D7 输入用标准 ROS 话题，平台 HAL 由 X3 桥接

Date: 2026-10-25

Context: `foray_platform` 目前只有 HAL C++ 库，没有 ROS 节点；本仓需要可独立构建。

Decision: 本仓订阅标准 `sensor_msgs/PointCloud2` / `nav_msgs/Odometry`；
平台 HAL → ROS 的桥接由 X3（foray_robots/装配根）提供。`scan_adapter` 负责
frame/stamp/字段校验，不直接链接 HAL。

Reason:
- 本仓可独立 colcon build/test，不被平台进度阻塞
- 标准消息便于录包回放与仿真

Alternatives:
- 直接链接 `foray_platform` HAL 并自带驱动节点

Rejected:
- 会把驱动/设备生命周期耦合进定位层，违反分层

---

## D8 消息过渡期放在 foray_localization_ros，后续迁 X1

Date: 2026-10-25

Context: `foray_interfaces` 当前无 `package.xml`/rosidl，且不在本仓可写范围；
定位层开发不能因此停摆。

Decision: 首版 `LocalizationState` / `MapMeta` / `DynamicObstacle(Array)` /
`Relocalize` 定义在 `foray_localization_ros/msg|srv`，字段与 X1 0.2.0 对齐；
X1 落地后迁移并保留兼容别名。

Reason:
- 解除跨仓阻塞，先打通技术链
- 字段设计按契约评审，迁移成本可控

Alternatives:
- 等 X1 提供消息后再开发
- 只用标准消息 + diagnostic_msgs 承载健康字段

Rejected:
- 阻塞关键路径
- 标准消息无法表达状态/置信度/age 的强类型契约

---

## D9 首版 GICP 后端用 PCL，内置 ICP 兜底

Date: 2026-10-25

Context: `small_gicp` 未在 `foray_ws` pin，且无 rosdep key；PCL 在 ROS 2 Humble
环境稳定可用，CI 可直接安装。

Decision: `foray_localization_map` 定义 `Aligner` 接口；
本次实现 `IcpAligner`（内置点到点 ICP：角点网格最近邻 + SVD 位姿更新，
带内点率/平均残差/fitness 评估）作为默认后端；
`small_gicp` 与 PCL GICP 作为后续可插拔后端，通过同一接口接入。

Revision (2026-10-25): 实现期本机与 CI 构建环境均无 PCL，`FORAY_WITH_PCL`
未落地。为避免未验证代码进入关键路径，先交付可测试的内置 ICP，并保留
`Aligner` 抽象；赛前实测若需要 GICP 精度，再接 PCL/small_gicp 后端。

Reason:
- 保持 CI 与本地可构建，不把未 pin 的第三方塞进关键路径
- 接口不变，后续换 small_gicp/PCL 不影响上层
- 内置 ICP 可被合成点云回归测试覆盖，当前测试全绿

Alternatives:
- 立即 vendor small_gicp 源码
- 只用内置 ICP

Rejected:
- 第三方源码入库增加升级/审计成本，且与 foray_ws pin 策略冲突
- 内置 ICP 精度不足，不能作为赛场默认

---

## D10 ONNX 重定位可选、默认 OFF；许可证不在本期范围

Date: 2026-10-25

Context: 用户要求计划考虑纯 CPU ONNX Runtime；许可证问题用户明确暂不考虑。

Decision: `foray_localization_place`（2026-10-25 更名为
`foray_localization_relocalization`，见 D12）用 ONNX Runtime CPU（`FORAY_WITH_ONNX=OFF` 默认），
不引入 CUDA/TensorRT EP；定位主链不依赖 ONNX。许可证决策移出本期，
`package.xml` 的 `<license>` 暂保持 `TODO`，待组织统一。

Reason:
- 主链确定性优先，可选能力不阻塞交付
- 纯 CPU 满足机载无 GPU 约束

Alternatives:
- 默认启用 ONNX 全局重定位
- 本期同时定许可证

Rejected:
- 默认启用会增加启动时间与误匹配面
- 许可证与代码交付解耦，避免阻塞

---

## D11 仿真闭环放在本仓、但不引入 Gazebo 构建依赖

Status: **superseded by D16**（2026-10-27：仿真闭环与离线工具已整体移出本仓）。
`foray_localization_sim` / `foray_localization_tools` / `bringup/config/sim.yaml` /
`docs/simulation.md` / CI `sim-smoke` 均已不存在，删除理由与失去的能力见 D16。

Date: 2026-10-25

Context: M5 HIL 验证需要录包/仿真回放与 ATE/RPE、DEGRADED/LOST 时序验证，但此时无实车、
机载无 GPU，且官方 Gazebo 场景（`rmu_gazebo_simulator` / `pb_rm_simulation`）版本演进快。

Decision: 本仓只提供 `foray_localization_sim`（真值里程计注入 + 模式化 launch）与
`foray_localization_tools` 的合成回放/评估脚本；Gazebo 场景与 Mid360 插件不入仓、不进构建
依赖，按 `docs/simulation.md` 外部复现。合成闭环走「先验 PCD 射线投射 → 回放 + 定位节点 →
`eval_trajectory.py` 断言 ATE/OK 占比」，CI `sim-smoke` 无 GPU 可跑；仿真阈值单放
`bringup/config/sim.yaml`，不参与默认实车启动。

Reason: 定位节点只认 `/livox/lidar`、`/aft_mapped_to_init`、TF，既不破 `.foray-layer` 的
依赖方向，也不破「定位层不感知数据来源」的接口约定；Gazebo 进仓会把渲染/物理依赖带进定位层
构建，与机载无 GPU 约束冲突；仿真阈值又只能初标（无动态物体、无遮挡、多为机械式扫描），
实车结论仍以现场标定为准。

Alternatives: 把 Gazebo 场景与插件 vendor 进本仓；只等实车、暂不做离车验证。
Rejected: vendor 增加升级/审计成本且与 `foray_ws` pin 策略冲突；无实车期间没有任何可重复
回归，M5 会长期空转。

---

## D12 包布局迁入 src/、重定位包更名

Date: 2026-10-25

Context: 组织统一「仓根 = colcon 空间根、全部包在 `src/`」的布局约定（`foray_docs`
`repository_structure.md` §7.5 待登记），并要求核心包用 `_core`、角色后缀封闭词表。
本仓 7 个包原先平铺在仓根，`foray_localization_place` 的后缀 `place` 不在词表内。

Decision:
- 7 个包全部迁入 `src/<package>/`，仓根不再有 `package.xml`；
- `foray_localization_place` → `foray_localization_relocalization`，同步
  `package.xml`、CMake `project`、命名空间 `foray::localization_relocalization`、
  头文件目录 `include/foray/localization_relocalization/`、include guard；
- 类名 `PlaceLocalizer` / `PlaceConfig` 与文件名保持（重定位手段仍是地点识别）；
- CI 新增 `dir-layout` 作业，与 `foray_platform` 使用同一套规则。

Reason:
- 与组织布局约定一致，`colcon build` / `rosdep install --from-paths src` 语义清晰；
- 「重定位」比「地点识别」更贴合对外语义（服务名已是 `Relocalize`），且 `place`
  不在封闭后缀词表内；
- 改名窗口只有现在：该包尚无下游消费方（已核对 `foray_localization_bringup`
  不 `find_package` 它），成本最低。

Alternatives:
- 保持平铺布局，只在 `foray_docs` 登记差异；
- 只迁 `src/`，不改包名。

Rejected:
- 平铺布局与组织约定冲突，后续每个仓都要写例外；
- `place` 与 `_relocalization` 并存会造成跨仓命名漂移，越晚改越贵。

---

## D13 Python 目标版本固定为 3.10

Date: 2026-10-25

Context: CI `test-python` 使用 `actions/setup-python` 的 3.10，与 ROS 2 humble 的
系统 Python 一致。首次真实运行中，`ruff --fix` 的 `UP017` 把
`datetime.timezone.utc` 改写成 `datetime.UTC`；后者是 Python 3.11 才引入的别名，
在 3.10 上直接 `AttributeError`，而本地开发机是 3.14，测试全绿，问题只在 CI 暴露。

Decision:
- 仓根新增 `pyproject.toml`，写 `[tool.ruff] target-version = "py310"`；
- 需要 UTC 时使用 `datetime.timezone.utc`，不得使用 `datetime.UTC`。

Reason:
- 让静态检查与自动修复也遵守运行时版本，而不是跟随开发机；
- 目标版本显式声明比逐个违规加 `noqa` 可维护。

Alternatives:
- 只在违规行加 `# noqa: UP017`；
- 把 CI 与运行时升到 3.11+。

Rejected:
- 逐行 `noqa` 无法阻止同类自动修复再次引入 3.11+ API；
- ROS 2 humble 的官方支持版本是 3.10，升级不属于本仓决策范围。

---

## D14 位姿可用性显式化：`LocalizationState.pose_valid`

Date: 2026-10-27

Context: 审计 `docs/audit-2026-10-27.md` B2 指出，`LOST` 下 `publish_state()` 照旧按
`map_to_odom_ * odom_to_base_` 合成一个「看起来很新」的位姿发出去。下游只要不盯 `state`
就会拿过期位姿闭环——正是 README「禁止静默外推」要防的事故。另一种做法是 `LOST` 期间
干脆不发 `LocalizationState`。

Decision:
- `LocalizationState` 新增 `bool pose_valid`；`to_msg_state()` 由健康状态纯映射：
  `OK` / `DEGRADED` 为 `true`，`INITIALIZING` / `LOST` 为 `false`。
- `LOST` 期间**照旧**以 `state_rate_hz` 发布状态（`confidence = 0`、TF `map → odom` 停发），
  但 `pose` 只是最后一次合成值，仅用于 rosbag 复盘。
- README「健康契约」把它写成**强制**约定：下游必须先判 `pose_valid`，`false` 时禁止消费 `pose`。

Reason:
- 停发消息会让下游失去「已经失效」这一事实，只能靠消息超时猜——诊断连续性更差，
  且 `transient_local` QoS 下「最后一次消息」语义不明确；
- 加字段是纯增量改动：消费方都在本仓、随包一起重建（ROS 2 消息类型哈希会变，
  但这是一次全仓统一改动，不像改发布时序那样要下游改控制逻辑）；
- 字段放在 `pose` 之后而非文件末尾，是为了「位姿 + 是否可用」在读定义时相邻；
- 把判据从「状态枚举的语义」压成一个布尔量，下游实现不会写错。

Alternatives:
- `LOST` 下停发 `LocalizationState`，由下游按超时判定失效；
- 只靠 `state != STATE_OK` 隐式表达（不新增字段）。

Rejected:
- 停发消息把「失效」变成不可观测事件，且与 `transient_local` 的语义纠缠；
- 隐式判据正是审计里出事的形式——文档写过「必须检查」，实现却没有单一的检查点。

---

## D15 参数文档由脚本看守：README ↔ `declare_parameter` ↔ 兵种配置

Status: **校验已恢复（D18，2026-10-28）**——原 `check_param_consistency.py` 与其 pytest、
CI `test-python` 作业已随工具包删除（D16），三处一致一度退回 PR 人工核对；
现由 `scripts/ci/check_params.py` + CI 作业 `param-contract` 看守，零第三方依赖，
README 参数表结构保持不变。

Date: 2026-10-27

Context: 审计 B3 实测 README「关键参数」表有 11 个参数名在源码中不存在
（`map_manifest_path`、`registration_rate_hz`、`voxel_size`、`min_cluster_points` …），
`base_frame` 默认值也写成 `base_link`，`tree.md` 还把两个节点入口写成一个文件。
README 是本仓唯一需求来源（AGENTS.md），这类漂移会让下游照文档写配置直接启动失败。

Decision:
- 以实现为准回写 README 参数表：拆成「本地化节点参数（`foray_localization`，30 项）」
  与「地图服务节点参数（`foray_map_server`，3 项）」，**一行一个参数名、默认值抄自
  `declare_parameter`**，便于机器解析；
- 新增 `src/foray_localization_tools/scripts/check_param_consistency.py`，在 CI 的
  `test-python` 作业里断言：README 参数名集合 == `declare_parameter` 集合；兵种配置键
  ⊆ 已声明参数；兵种配置键 ⊇ 已声明参数减去 launch 注入的 `{map_manifest, verify_map_hashes}`；
- 配套 pytest（`test_param_consistency.py`）既跑真实仓库，也用 `tmp_path` 合成仓库证明
  「README 多参数 / 缺参数 / 配置键漂移」三类问题都会让脚本 exit 1。

Reason:
- 文档漂移靠人眼 Review 抓不住（11 个名字错了很久没人发现），只有可执行断言能长期生效；
- 让 README 表格结构化（一行一名）同时降低了人读成本与解析成本；
- 配置允许「多声明 launch 注入项」，所以规则写成子集/超集而不是死板相等，避免过度约束。

Alternatives:
- README 直接由脚本从源码生成（`declare_parameter` 无法提供说明文本，反而更差）；
- 只在 Review checklist 里加一条「记得同步 README」。

Rejected:
- 自动生成会丢掉「为什么是这个默认值」的解释，README 会退化成接口清单；
- checklist 无强制力，审计已经证明这类漂移会长期存活。

---

## D16 移除合成仿真与离线工具，本仓只保留核心定位算法

Date: 2026-10-27

Context: 用户要求「删除本仓库无意义的合成仿真，只保留核心定位算法」（m00001）。
盘点发现仿真不是孤立脚本而是一条闭环：`foray_localization_sim`（真值里程计注入 +
模式化 launch）、`foray_localization_tools`（合成场地生成、回放、累积建图、
ATE/RPE 评估、参数一致性校验）、`bringup/config/sim.yaml`、`docs/simulation.md`，
以及 CI 的 `sim-smoke` / `test-python` 两个作业。同时查出 `sim-smoke` 自 M7 起就是
死作业：探测条件写的是仓根 `foray_localization_sim`，而包已迁入 `src/`，
`-d` 恒为假，作业从未执行过（证据：`local/evidence-strip-sim/ci-sim-smoke-skip-proof.txt`）。

Decision:
- 删除 `src/foray_localization_sim`（4 文件）、`src/foray_localization_tools`（18 文件）、
  `src/foray_localization_bringup/config/sim.yaml`、`docs/simulation.md`、仓根
  `requirements.txt`；`src/` 只保留 `_core` / `_map` / `_ros` / `_relocalization` / `_bringup`。
- CI 只保留真实执行的 4 个作业：`dir-layout` / `lint-python` / `lint-c` / `build-ros2`；
  删除 `sim-smoke`（恒跳过）与 `test-python`（已无 `test_*.py`）。
  `dir-layout` 的豁免词表只留 `*_bringup`。
  > 现况（2026-10-28 起）：CI 为 9 个作业。此处 4 个是当时删减后的最小集，D18 又补了
  > 四道零依赖门禁与 `gate-selftest`；各作业清单以 `.github/workflows/ci.yml` 为准。
- 保留 `pyproject.toml`：`localization.launch.py` 仍是 Python，ruff/black 仍需目标版本 3.10。
- 验证责任重新划分：本仓 CI 断言「可构建 + 可执行文件安装位置 + launch 参数可解析 +
  `colcon test`」，**不**断言定位精度；车下验证改为录制包回放（README「无实车验证」）。

Reason:
- 需求来源只有 README（AGENTS.md），仿真闭环在 README 里是成章节的需求，删除必须同步
  改 README/tree/plan/decision，否则文档成为第二套过时需求。
- `sim-smoke` 恒跳过、`test-python` 无测试可跑：保留只会制造「绿灯假象」，
  与用户要求「修正被全部跳过的 CI」冲突。
- 合成仿真从未驱动过真实决策——它的产物（合成场地、回放包）与实际比赛场地无关，
  维护成本却随源码演进持续存在（审计 A3 就专门修过仿真射线投射）。

Compensation（明确记下失去的能力）:
- 失去 **参数一致性 CI 门禁**（D15）：README ↔ `declare_parameter` ↔ 兵种配置三处
  一致性改由 PR 人工核对。基线已跑过一次（30 + 3 项全等：
  `local/evidence-strip-sim/check_param_consistency.txt`），恢复工具包时必须同时恢复该作业。
  **后续（2026-10-28）**：本欠账已由 D18 以仓级零依赖脚本 `scripts/ci/check_params.py`
  + CI 作业 `param-contract` 补上，不需要恢复已删除的 Python 工具包。
- 失去 **仓内建图与地图校验工具**（`make_map.py` / `validate_map.py` /
  `accumulate_pcd.py`）：地图改在仓外制作，本仓只按 `manifest.yaml` 契约加载并校验 SHA-256。
- 失去 **离车精度回归**（ATE/RPE）：精度验收移到实车与下游仿真仓，本仓不再提供自动化精度断言。
  > 现况（2026-10-30）：承接方 `foray_simulation` 已建成，并把这条精度验收写成自己的需求 A2
  > （本仓在仿真里 10 s 内状态 `OK`、位置误差 < 0.30 m、朝向误差 < 5°），已取证：首次 `OK`
  > 3.78 s、位姿误差 0.083 m、`yaw` 0.012°，95 s 采样 4737 条状态中 4423 条 `OK` 且零
  > `DEGRADED`/`LOST`。因此本仓的回归通道是「容器内 `colcon test` 跑全套单元测试（2026-10-30
  > 实测 73 tests / 0 failures）+ 需要时跑 A2 整链」，不需要把仿真能力搬回本仓。

Alternatives:
- 保留 `foray_localization_tools` 只删 `foray_localization_sim`（用户评估后否决）；
- 保留工具包但删掉其测试与 CI 作业（会留下无测试的 Python 代码，更差）；
- 只修 `sim-smoke` 的 `src/` 前缀让死作业复活（用户目标是删除仿真，不是修仿真）。

Rejected:
- 用户明确选择 C 方案：连离线工具包一起删除，只留五个核心包；
- 修补恒跳过的作业会把它从「假绿」变成「真跑合成闭环」，与删除目标相反。

---

## D18 文档与接口漂移改由零依赖 CI 门禁看守：`scripts/ci/`

Date: 2026-10-28

编号说明：D17 属于并行的另一条工作流（仿真引擎外移），尚未入库；本条目按既定顺序取 D18。

Context: 用户提出「AI 开发越来越快、文件雪崩式膨胀，人类开发者如何维护代码」，讨论后的
结论是：**读不完的代码不要再试图读懂，改为压缩「需要被理解的表面积」**。其中一层是
「文件之间的机械一致性」——这类事实不需要判断力，只需要有人反复核对，正是 AI 生成速度下
最先崩掉的部分。用户据此要求把门禁补上。

D15 已经用一次实证给出了依据：README「关键参数」表与源码、与三份兵种配置之间曾有 11 个
参数名长期不一致，D15、D16 两轮审计之后，tree.md 又漏记 `.clang-format`、`local/` 与
`log/` 还与 tree.md 自己的「记录约束」段互相矛盾。这些都是**人眼 Review 反复扫过却没发现**
的错误——不是因为粗心，而是因为没有人会逐行比对两份长清单。

Decision:
- 新增仓级门禁脚本 `scripts/ci/`，**只用 Python 标准库**，不新增任何依赖：
  - `check_style.py`——LF / UTF-8 / 文件末尾恰好一个换行 / 无行尾空白。
    缩进不在此重复实现：C/C++ 归 `clang-format`（`lint-c`），Python 归 ruff/black。
  - `check_contracts.py`——`msg`/`srv`/`action` 的 SHA-256 快照，`--update` 刷新。
  - `check_params.py`——README 参数表 ↔ `declare_parameter` ↔ `bringup/config/*.yaml`
    三方一致，含跨 `constexpr` 常量解析与缺省值归一化（`0.1` ≡ `0.10`）。
  - `check_tree_doc.py`——`tree.md` 与 `git ls-files` **双向**互为镜像。
- 新增 `contracts.sha256` 快照文件（复用标准 `sha256sum` 格式，可用 `sha256sum -c` 直验）。
- CI 新增 `file-hygiene` / `doc-sync` / `param-contract` / `contract-lock` 四个作业，
  各自独立成 job：红了立刻能从作业名看出是哪条规则被破。
- 新增 `scripts/ci/test_guards.py` 与 CI 作业 `gate-selftest`：向仓库注入 7 种已知缺陷
  （行尾空白 / CRLF / 末尾双空行 / 非法 UTF-8 / msg 字段改名 / README 默认值被改 /
  未记录的被跟踪文件），断言对应门禁真的非零退出，再断言恢复后全部复绿且 `git status`
  无残留。门禁必须能被证明「在坏输入上失败」，否则无法排除它已被改坏。
- 新增 `.gitattributes`（`* text=auto eol=lf` + 二进制后缀列表）：`check_style.py`
  断言 LF，就必须同时在**检出侧**保证 LF。只在检查侧强制，Windows 开发者会看到
  「自己没改过却报红」的 CRLF——文件在 git 里是 LF，工作区却是 CRLF。门禁一旦开始
  误报就会被当成噪音绕过，那还不如不装。二进制后缀列表与 `check_style.py` 的
  `BINARY_SUFFIXES` 一一对应，两处改动必须同步。
- 自检文件命名为 `test_guards.py` 并保持可被 `pytest` 直接收集；自跑（不经 pytest）时
  末尾打印 `[==========] N tests from N test suites ran.` 汇总行。理由：DSH 任务引擎与
  部分 CI 适配器只从 gtest / pytest / ctest 等已知运行器的输出里判定「确实跑了 N 个用例」，
  `python3 某个脚本.py` 不在识别名单内，退出码 0 会被当成「没有测试」。用标准汇总行
  而不是引入 pytest 依赖，既不增加安装步骤，又让自检结果可被自动消费。
- 同步修 tree.md：补 `.clang-format`、`scripts/`、`contracts.sha256`；
  删掉 `local/` 与 `log/` 两行（tree.md 的「记录约束」段本就规定它们不得记录）。

Reason:
- 这四类约束原文都写在 AGENTS.md / README / D15 里，但**只有能被机器执行的那部分才会
  长期生效**。写在文档里的规则，在 AI 高频提交下退化成「写的时候想过」。
- 选零依赖而非恢复 `foray_localization_tools`（D16 已删除）：门禁的价值在于**任何机器上
  一条命令就能跑**，包括没有装 pyproject 依赖的 CI runner 与 AI agent 的临时容器。
  恢复工具包会把「跑一下门禁」重新变成「先配环境」。
- 双向检查而非单向：只查「文档写了但文件不存在」会漏掉新增未记录（最常见），
  只查「文件存在但文档没写」会漏掉虚构条目。两个方向都要。
- 门禁本身也必须可被怀疑。`sim-smoke` 的先例说明「看起来在跑的检查」可能从未生效；
  一个被削弱成恒绿的门禁比没有门禁更危险——它制造虚假安全感。这就是 `test_guards.py`
  的存在理由：定期证明每个门禁仍会红。
- 明确范围边界：这些门禁只锁**事实与事实之间是否一致**，不锁**设计是否合理**。
  后者仍需要人，也仍需要文档承载意图——门禁是让人把注意力从「抄写是否准确」里解放出来，
  不是取代判断。

Evidence:
- 门禁上线当次即抓到 3 处真实漂移：`tree.md` 漏记 `.clang-format`、漏记 `scripts/` 自身、
  `local/`+`log/` 与其「记录约束」段自相矛盾。这些是 D15/D16 两轮人工审计都漏掉的。
- 故障注入验证（每个场景注入后应红、恢复后应绿，全部符合预期）：
  | 场景 | 门禁 | 结果 |
  |---|---|---|
  | 追加行尾空白 | `check_style.py` | exit=1 |
  | 首行改 CRLF | `check_style.py` | exit=1 |
  | 文件末尾双空行 | `check_style.py` | exit=1 |
  | `LocalizationState.msg` 改字段名 | `check_contracts.py` | exit=1 |
  | README 参数默认值 `0.10`→`0.99` | `check_params.py` | exit=1 |
  | 新增未记录的跟踪文件 | `check_tree_doc.py` | exit=1 |
  | 全部恢复后复跑 | 四个脚本 | exit=0 |
- 门禁自检（`python3 scripts/ci/test_guards.py`）：11 个用例全部符合预期——4 项基线绿、
  7 种缺陷全部被对应门禁抓住、恢复后复绿、测试前后 `git status` 完全一致。
- 耗时：四个门禁脚本合计不超过 10 秒（本机为慢速外挂盘，`user` CPU 时间仅 0.1–0.2 s，
  几乎全为文件读取等待）；`test_guards.py` 因重复调用门禁约 50 秒。
  GitHub Actions 的本地 SSD 上两者都远快于此。

Alternatives:
- 用 pre-commit 框架统一管理钩子——需要每台机器 `pre-commit install`，AI agent 的
  一次性容器里通常没有；CI 才是唯一无法绕过的关口。
- 恢复 `foray_localization_tools` 并用 pytest 承载这些断言——与 D16 的删除决定冲突，
  且把这些断言绑回一个本该只有算法的包里。
- 只做 Git 本地钩子——本地可 `--no-verify` 绕过，跨机器不同步，保护不了 PR。

Rejected:
- 「只告警不失败」（warning 模式）：D15 的教训正是没人看的规则等于没有规则。
- 让门禁自动修复格式（自动 `--update` 快照、自动重排 README）：会把「接口变了」这件
  需要人判断的事静默吞掉，快照失去全部意义。

状态：已执行（2026-10-28）。`param-contract` 同时补上了 D16 记下的那笔「失去参数一致性
门禁」的欠账，且不再依赖被删除的 Python 工具包。

---

## D19 一次性分析报告不入库，门禁自检改用专门的探针文件

Context: `docs/audit-2026-10-27.md` 是一次性代码审计快照（P0/P1/P2 问题与修复状态）。
它记录的是「某一天谁查出了什么」，问题修完之后就没有维护价值，却会随仓永久增重；
同时 `scripts/ci/test_guards.py` 一直拿它当 `STYLE_PROBE`（注入缺陷验证风格门禁会红），
于是「不入库」和「门禁自检依赖它」直接冲突。

Decision:
- 日期化的审计/复核报告一律**本地留存、不入库**：`.gitignore` 里按
  `docs/audit-*.md` 与 `src/*/docs/*_AUDIT.md` 这类模式排除；审计的**结论**留在
  `decision.md` 与 `plan.md`，**可复现的判据**留在脚本与 JSON 参数里（如
  `step_measurements.json`），报告文件本身不进仓。
- 门禁自检的注入点改用专门的 [`docs/probe-style.md`](docs/probe-style.md)：
  它没有需求语义，任何门禁都不读它的内容，只服务 `test_guards.py`。

Consequences:
- `tree.md` 与本文件被门禁双向校验，所以探针文件必须同时登记进 `tree.md`；
  删探针等于让风格门禁自测失去落点，CI 会红。
- 已发布的历史提交里仍有那份审计报告，不重写历史；只从当前树移除。

状态：已执行（2026-10-28）。

---

## D20 lint 红灯是契约问题，不是格式洁癖

Date: 2026-10-30

Context: PR #3 首次真实运行 CI（run 40913190187），7 个作业通过，`lint-c`（10 s）与
`lint-python`（13 s）红：
`src/foray_localization_ros/src/localization_node.cpp:345:42` 与 `:346:72` 报
`code should be clang-formatted [-Wclang-format-violations]`；`ruff` 报 5 次
`EXE001`（Shebang is present but file is not executable），指向 `scripts/ci/` 全部五个脚本。
值得记下的是本地当时的判断：用 clang-format 23.1.4 跑 `--dry-run` 发现 HEAD 版本同样报错，
于是判定「属版本差异、不是本次引入」——**这个判断是错的**。那两行本来就不符合
`.clang-format` 的 `ColumnLimit: 100`，只是本地工具版本的语气不同。

Decision:
- 按 `.clang-format` 重排 `localization_node.cpp:345-346`：条件表达式合并为一行、
  续行与括号对齐（2 insertions / 3 deletions），`clang-format --dry-run --Werror` 归零。
- `scripts/ci/*.py` 五个脚本统一 `chmod 100755`：`check_contracts.py` / `check_params.py` /
  `check_style.py` / `check_tree_doc.py` / `test_guards.py` 都带 shebang，可执行位是它们
  声明的调用方式。
- `test_guards.py` 按 black 格式化（`CONTRACT_PROBE` 注入块与两处 `_mutated(...)` 调用按
  88 列拆行，共 15 行变动），格式化后自检仍 11/11 通过、`black --check` 与 `ruff check .`
  全仓无告警。

Reason:
- **本地绿不等于 CI 绿**：格式判据是 CI 装的那套工具与版本，本地版本差异不能用来论证
  「不是我的问题」。这次误判差点让一条真违规被当成噪音放过——而门禁被当成噪音的那一刻，
  它就等于不存在（D18 的同一逻辑）。
- shebang 与可执行位是一件事的两半：只写 shebang 不给权限，等于声明了调用方式又拒绝被调用。
  同类隐患在门禁脚本上尤其危险，因为它们正是「别人要随手跑一下」的东西。
- 门禁代码也要过它自己的标准。自检脚本被自己管辖的工具格式化，是「门禁可被怀疑」的自然延伸。

Alternatives: 给 ruff 加 `per-file-ignores` 静音 `EXE001`；在那两行前后加 `// clang-format off`。
Rejected: 静音会把「脚本不可执行」这一真实缺陷永久藏起来；`clang-format off` 让该文件成为
格式飞地，下一个人改这段代码仍会踩同一处。

状态：已执行（2026-10-30）。修复提交 `2562ec3`，CI run 40915429011 九作业全绿。

---

## D21 默认配准后端改为 PCL GICP

Date: 2026-10-30

Context: README 从首版起就写「配准做 GICP（默认 PCL GICP）」，但仓内只有
`IcpAligner`（点到点 ICP）：全仓 `FORAY_WITH_PCL` / `find_package(PCL)` / `gicp` 一处都没有，
`src/foray_localization_ros/src/localization_node.cpp` 里直接构造的是
`foray::localization_map::IcpAligner`。D9 当年已记下「PCL 未安装、首版先用内置 ICP 兜底」，
而那只是**临时状态被写成了长期口径**：文档描述的能力实际不存在，属文档与实现不一致，
不是实现落后于文档。2026-10-30 用户裁定「实装 PCL GICP」，这条决策把它落地。

Decision:
- 新增 `PclGicpAligner`（`pcl::GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ>`），
  后端选择做成**编译期开关 + 工厂**：`option(FORAY_WITH_PCL ON)` 决定是否编译该后端，
  `make_aligner()` 返回 `std::unique_ptr<Aligner>`，`default_aligner_name()` 供节点打日志。
  `#if` 只出现在 `aligner_factory.cpp` 与 `pcl_gicp_aligner.cpp`，不泄进 ROS 包。
- 默认 ON；ON 时 `find_package(PCL 1.10 REQUIRED COMPONENTS common registration)`，
  **找不到 PCL 直接 configure 失败**，不静默退化成 ICP。OFF 是显式逃生口（无 PCL 环境、
  最小构建、降级排查），不是自动兜底。
- 质量口径抽成公共 `evaluate_alignment()`（`alignment_metrics.hpp`）：内点比例、平均残差、
  fitness 三字段由同一份代码算，两个后端都调它，连体素近邻索引 `PointCloudIndex` 也共用。
- 每帧新建 GICP 对象，不做 target 预处理缓存；`iterations` 直接读公开成员
  `nr_iterations_`；不新增任何 ROS 参数（`max_iterations` / `max_correspondence_distance_m` /
  `transformation_epsilon_m` 一对一映射到 PCL setter，`correspondence_randomness` 留 PCL 默认 20）。
- CI 增 `build-nopcl` 作业跑 `-DFORAY_WITH_PCL=OFF`，作业数 9 → 10。

Reason:
- **语义相反，绝不能直接填**：PCL 的 `getFitnessScore()` 是均方误差（越小越好），
  本项目的 `fitness` 是 0..1 且越大越好，`RegistrationSample.valid` 与健康状态机全靠后者。
  若两个后端各填一套，换后端就等于换了一套判定标准，`ok_inlier_ratio` 之类的阈值失去意义。
- **target 预处理无法安全跨帧缓存**：PCL 1.12 的 `setInputTarget()` 会
  `target_covariances_.reset()`，协方差仅在为空时重算，而 `setInputSource()` 不重置
  `input_covariances_`，且 PCL 没有 `getTargetCovariances()` 可回取。复用同一对象跨帧，
  等于让上一帧的 source 协方差与新一帧的 source 错配——这是静默的精度事故，比慢更糟。
- **静默降级会造假象**：若找不到 PCL 就悄悄用 ICP，产物会是「文档说 GICP、二进制跑 ICP」
  而没人知道。显式 configure 失败把这件事变成构建期可见的事实。

Alternatives: 用运行时 ROS 参数选后端；复用同一 GICP 对象缓存 target；直接把
`getFitnessScore()` 填进 `fitness`；实装 `small_gicp`。
Rejected: 运行时选后端要求二进制永远带 PCL，既没省依赖又多一个没人验证的参数，且健康阈值
会随后端漂移；对象复用见上（协方差错配）；直接填 `getFitnessScore()` 语义相反；
`small_gicp` 用户明确不在此次范围，文档已改为不再宣称它存在。

Consequences:
- 构建从「Eigen + yaml-cpp」变成**需要 PCL ≥ 1.10**（`package.xml` 用 rosdep key
  `libpcl-all-dev`，映射 apt `libpcl-dev`；容器与 CI 已验证 1.12）。
- `-DFORAY_WITH_PCL=OFF` 路径由 CI `build-nopcl` 守护；只验一条路径等于另一条迟早烂掉。
- **已知代价**：target 协方差每帧重算，地图点数大时可能吃掉 5 Hz 预算。节点已按
  `map_voxel_leaf_m`（默认 0.25）对地图降采样，实测本仓口径下 map 包 34 tests 全绿、
  单帧耗时未超预算，但**没有实车/实图数据**，所以 p99 ≤ 40 ms 的验收项保持未勾选，
  等 M5c 录包后再定缓存策略。
- 另一处重复开销（审核时补记）：质量评估调 `evaluate_alignment` 会对 target 再建一份
  体素索引，与 GICP 内部的 KD 树并存。这是为了让 `fitness`/`inlier_ratio` 与内置 ICP
  同口径而付的代价——换成 PCL 的 `getFitnessScore()` 能省掉这次建索引，但语义相反
  （见 Rejected）。是否把索引也做跨帧缓存，与上一条一起等实测数据。
- 平台面：默认 ON 让 PCL 成为硬依赖，而 PCL 在 Windows 上要另走 vcpkg。Windows/MSVC 构建
  本仓从未验证过（CI 全在 Linux 容器），因此 README 只声明「验证过的环境是 Linux」，
  并指出 Windows 上先用 `-DFORAY_WITH_PCL=OFF` 摘掉 PCL 再谈构建；`pcl_gicp_aligner.cpp`
  里的中文 error 字面量在 MSVC 下需要 `/utf-8`，这条留给真正做 Windows 构建时一并验证，
  不在此刻凭空加一个没人跑过的编译选项。检出侧与门禁侧不受影响：`.gitattributes` 已钉 LF，
  `check_style.py` 按字节判 CRLF，`scripts/ci/` 与 pytest 自检在 Windows 上可正常运行。

状态：已执行（2026-10-30）。容器实测（本仓 5 个包，`colcon test-result` 汇总）：默认 ON
75 tests / 0 failures，`-DFORAY_WITH_PCL=OFF` 70 tests / 0 failures；差的那 5 条正是
`test_pcl_gicp`（4 个用例 + 该文件在 CTest 里的汇总条目），map 包单独口径 ON 34 / OFF 29。

---

## D22 健康阈值必须高于先验地图自身离散度

Date: 2026-10-30

Context: 仿真里换成 PCL GICP 后端后，定位节点始终进不了 `OK`：先停在 `INITIALIZING`，
`lost_timeout_sec` 到点后转 `LOST`；`LOST` 是粘滞状态，只有显式 `Relocalize` 能清，
而重新注入真值后节点又回到 `INITIALIZING` 并再次超时，看起来像「GICP 起不来、
注入初值也没用」。实测每帧诊断（临时加在节点里，事后已撤）显示后端本身没问题：
`converged=1`、`iterations=1~5`、`inlier_ratio=1.000`、`fitness≈0.83`，但
**`mean_residual_m` 稳定在 0.1015~0.1026 m**，而 `ok_mean_residual_m` 是 0.10。

Decision: `ok_mean_residual_m` 0.10 → 0.15，`degraded_mean_residual_m` 0.20 → 0.30
（`heavy.yaml` 按同比例 0.12/0.24 → 0.16/0.32）。三处必须同步：`HealthConfig` 的成员
默认值（`health.hpp`）、节点的 `declare_parameter` 默认值、以及
`src/foray_localization_bringup/config/*.yaml` 三份兵种配置——`check_params.py`
就是来锁这三处一致性的，改一处漏一处 CI 会红。

Reason: 阈值量的东西和地图的离散度是同一个量纲。先验图按 `map_voxel_leaf_m`（默认
0.25）做过体素降采样，点间距本身就是 0.25 m 量级，于是即使配准完全收敛，每个源点
到最近的地图点也有约 0.10 m 的残差——这是**地图分辨率的下限**，不是配准误差。旧阈值
0.10 恰好压在这条基线上，实测值高出 1.5 mm 就永久不合格：`good` 永不成立 →
`warmup_frames` 永远攒不满 → `INITIALIZING` 超时 → `LOST` 粘滞 → 注入初值重来一遍
只是重播同一段剧情。同一实测里 `inlier_ratio` 是 1.000，说明内点率阈值 0.55 从来不是
瓶颈，问题只在残差这一侧。

Alternatives: 让 `ok_mean_residual_m` 自动取自 `map_voxel_leaf_m`（例如 0.6 × 体素边长）；
调小 `map_voxel_leaf_m` 让地图更细从而压低残差；放宽 `inlier_ratio`；改 GICP 的
`correspondence_randomness` 或 `transformation_epsilon`。
Rejected: 自动取值的想法更优雅，但实车地图的粒度未必等于本仓的降采样参数（地图是仓外
产物），把两个独立来源耦合成一条隐式规则，现场调参时反而看不出发生了什么；调小体素会让
地图点数与配准耗时一起涨，是拿性能换一个本来就不该卡的判据；`inlier_ratio` 已经是
1.000，放宽它解决不了任何问题；GICP 在精确初值下 1 次迭代就收敛，参数没有可挤的水分。

Consequences:
- 仿真实测（改动后重启节点，`--symlink-install` 下 yaml 是符号链接、改 src 即生效，
  但**节点只在启动时读参数**，必须重启才看到效果）：静态 25 s 采样 852 条，`pose_valid`
  100%、`state=OK` 100%、`confidence` 最小 0.791 / 均值 0.825。
- 短距运动（0.2 m/s 走 4 s）340 条样本全部 `OK`，无 `DEGRADED`/`LOST`。
- 判据仍然有效：残差 > 0.15 进 `DEGRADED`、> 0.30 视为不可用，`test_health` 里
  `DegradesAndRecovers` 用的样本残差同步从 0.15 抬到 0.20，避免测试挂在阈值边界上。
- 新增回归用例 `AcceptsResidualAboveLegacyThreshold`：0.12 m 残差在默认配置下必须是
  `OK`。这条用例的存在就是为了在有人把阈值调回 0.10 时立刻红。
- **风险**：调宽的是「判定合格」的那一侧，实车若真的出现 0.12~0.15 m 的系统性残差，
  现在会被判成 `OK` 而不是 `DEGRADED`。这个数值来自仿真（32 线、0.25 m 体素图），
  实车地图与雷达线数不同，**必须用 M5c 的实车数据复核**；在那之前不要再往上抬。
- 排查教训：issue 里「把 `ok_mean_residual_m` 放宽到 1.00 也没用」的观察无法复现。
  `install/` 下的 yaml 是符号链接，改 `src/` 立刻生效，但参数只在节点启动时读取，
  不重启就还是旧值——这类「改了没用」的现象优先怀疑没重启，而不是怀疑阈值机制。

状态：已执行（2026-10-30）。仿真静态与短距运动实测见上；实车复核待 M5c。

## D23 小 FOV 下用可观测性与方向约束取代整体拒绝

Context: 用户要求「将点云裁切优化方向定位用尽可能小 fov 实现稳定定位」——不是把 FOV 硬编码成
某个角度，而是把**可用 FOV 的下限尽可能压下去**。现有实现对此毫无准备：`src/` 里没有 ROI /
FOV 裁切（`grep crop|roi|max_range` 零命中），也没有任何可观性判断，配准失败只有一条路——
`inlier_ratio == 0` → 该帧整体否决 → 健康状态往下掉。视野一裁，内点变少、几何约束变弱，
这条「要么全信、要么全拒」的路径会直接退化成不可用。

Decision: 在 `foray_localization_map` 增加可观测性分析：在配准收敛位姿上，用**点面残差**构造
6×6 信息矩阵 `H = Σ Jᵢᵀ Jᵢ / σᵢ²`（`Jᵢ = [nᵢᵀ, (pᵢ × nᵢ)ᵀ]`），对 `H` 做特征分解，把特征值
归一化后低于阈值的特征向量标为「不可观方向」，并在更新 `map → odom` 时把修正量投影到可观
子空间——**不可观方向保留里程计外推，可观方向照常接受修正**。

Reason: 必须用点面残差而不是点对点。点对点下沿墙面切向平移时最近邻会重匹配到相邻点，残差
依旧很小，会把走廊、长直墙面判成「完全可观」——那正是我们最需要识别的失效场景。点面残差的
切向分量恒为 0，物理意义与「这一维到底有没有被约束住」一致。这条思路与 X-ICP
（localizability-aware registration）一致：先判可观测性，再在优化/更新层面对不可观方向加约束，
而不是把整帧丢掉。

Alternatives: 整体判 DEGRADED（实现最简单）；用固定 FOV 阈值一刀切；把 FOV 裁切做成纯性能开关
而不做可观性判断；用点对点信息矩阵。
Rejected: 整体降级会把「只丢一维」退化为「完全不可用」，与「压低下限」的目标直接相反；固定
角度阈值忽略了场景几何与行进方向——同一角度在开阔场地够用、在走廊里远远不够；只做裁切不做
判断等于把失败概率交给运气；点对点信息矩阵在走廊给出假性可观，是最危险的错法。

Consequences:
- 新增 target 侧法向估计（邻域 PCA）与信息矩阵计算，均为一次性/每帧 O(内点数) 的开销；
  法向与点云索引随地图生命周期缓存，顺带解决 D25 的一项性能热点。
- 健康契约多两条降级理由：输入点数不足、可观性不足（`reason` 里带具体自由度），
  只可能把 OK 往 DEGRADED 拉，不会反向放宽。
- **FOV 下限的结论必须由实验给出**：对若干张角逐档跑固定轨迹，输出「角度 → 状态分布/误差/
  条件数」曲线，并附「方向约束版 vs 无约束版」对照，证明约束确实把可用下限推低。
  照搬 *How Much LiDAR Field of View Is Enough?* 的实验设计（按开口角与方位裁切后测失效率）。
- 阈值不能用拍死的常数：点数少时特征值噪声大，判据需随点数自适应（参考 probabilistic
  degeneracy detection 的思路），否则小 FOV 下会频繁误报。

状态：需求已入档（2026-10-30），实现中。

## D24 融合层 EKF：只平滑 map → odom，不动里程计

Context: `localization_node.cpp` 现在是直通——每次配准成功就 `map_to_base_ = result.transform`、
`map_to_odom_ = map_to_base_ * odom_to_base_.inverse()`，随后 TF 按这个值发 20 Hz。配准 5 Hz，
意味着每 200 ms `map → odom` 阶跃一次，中间全靠点云里程计外推；跳变会直接传到下游的规划与
costmap。

Decision: 在 `foray_localization_core` 增加 6 维 EKF（位置 + 姿态），状态是 `map → odom`，
预测用**常值模型**（两次配准之间 `map → odom` 应当不变，运动由里程计承担），过程噪声随 dt 增长；
量测是每帧配准给出的 `map → odom`，量测噪声按残差项与 D23 的逐方向特征值放大。
配准与预测偏离超过阈值时判为重定位、直接硬切覆盖；`/localization/relocalize` 触发重置；
LOST 期间停止更新。

Reason: 把滤波放在融合层而不是 aligner 内部，是因为 aligner 目前是无状态的（每帧新建 GICP
对象正是为了不残留内部状态），且 ICP / PCL GICP / 未来的后端各自实现一遍滤波必然走样。
融合层只看到位姿，代价是无法做 IEKF 式的迭代重线性化——这个代价可以接受，因为我们本来
就只能在配准输出之后动手。

Alternatives: 在 aligner 里做 IEKF；用简单低通/滑动平均；直接用里程计做二级 EKF
（robot_localization 那种双实例结构）；完全不滤波只改配准频率。
Rejected: aligner 内滤波会污染后端抽象，且三种后端的噪声模型根本无法统一；低通没有协方差、
无法与可观性联动，也无法区分「平滑」与「跟随错误」；双 EKF 需要另一个里程计源且改动面过大，
本仓的里程计已经足够好，问题只在融合层；提高配准频率是拿 CPU 换平滑，与 D25 的性能目标冲突。

Consequences:
- `map → odom` 从阶跃变为连续；TF/state 频率与坐标系不变，下游无感。
- **必须提供 `filter_enabled: false` 的逃生口，且该路径要与改造前逐字节等价**——这是回归对照，
  也是实车出问题时的第一道开关。
- 退化方向上的量测噪声被放大后，滤波器在那个方向上会自然退化为「信里程计」，与 D23 的
  方向约束形成同一套语义的两种实现，不会互相打架。
- 阶跃抑制要有可测判据：同一段输入下相邻帧 `map → odom` 位置跳变 ≤ 0.02 m、朝向 ≤ 0.5°。

状态：需求已入档（2026-10-30），实现中。

## D25 性能：先测量再优化，第三方后端只能是可选

Context: 当前可指认的热点有两处——`pcl_gicp_aligner.cpp` 每帧新建 GICP 并 `setInputTarget`
（PCL 1.12 的 `setInputTarget` 会 `target_covariances_.reset()`，协方差仅在其为空时重算，所以
「每帧新建」等于每帧重算 target 协方差）；`alignment_metrics.cpp` 每帧重建 `PointCloudIndex`。
但先前的 D21 明确记过「不做 target 预处理缓存」是当时的权衡，不能凭猜推翻。

Decision: 先加分段计时（`profile_enabled`，默认关）拿到 `to_point_cloud / voxel / TF / align /
evaluate / cluster` 各阶段真实耗时，再按数据排序动手；每一项独立提交、可单独 revert。
第三方方面，`small_gicp`（MIT、header-only、只依赖 Eigen + bundled nanoflann/Sophus，
官方基准 GICP 比 `pcl::GICP` 快约 2.4×、并行降采样快约 3.2×，并直接暴露 6×6 Hessian `result.H`）
只作为**可选的编译期后端** `FORAY_WITH_SMALL_GICP`（默认 OFF）评估，填进现有 `make_aligner`
工厂，不改变默认构建。

Reason: 没有基线数据的「优化」无法验证，也无法在 PR 里自证；而引入第三方库必须能被收益证明
正当（AGENTS.md 禁止无必要依赖）。small_gicp 的价值不只在速度——它的 `result.H` 正好是 D23
需要的信息矩阵，若能复用可以省掉一遍自算；但要先证明它在本仓的 PCL 1.12 + 本场景下真的更快。

Alternatives: 直接换 small_gicp 为默认后端；只做零依赖优化不考虑第三方；把 target 换更粗的
体素以降低点数。
Rejected: 直接换默认等于让一个未在本仓验证的依赖承担主链路风险；只做零依赖可能够不到目标；
加粗体素是拿精度换速度，而精度恰恰是 D22 刚花力气修好的东西。

Consequences:
- 验收看**分段耗时表**与总耗时的改前/改后对照，目标下降 ≥ 20%；达不到就如实写，不许只报单项好转。
- small_gicp 实测提速 < 15% 则不引入，结论记入本条目。
- D23 的法向估计与索引缓存与这里的优化是同一批改动，不得各做一份（否则会出现两份索引）。

状态：需求已入档（2026-10-30），实现中。

## D26 FOV 扫描：静止条件下水平视场可压到 30°，方向约束在 60° 以下才有收益

Context: D23 立了「用可观测性与方向约束取代整体拒绝」，但没有数据说明视场到底能压到多小、
以及方向约束在什么档位真正值钱。arena 仿真场地 12 m x 8 m，0.25 m 体素先验图 512x352。

Decision: 在仿真里做水平视场扫描——保持仿真栈不动，只改 `fov_horizontal_deg` 后重启定位节点，
每组采集 25 s、取后 60% 稳态统计。九个配置全部 `100% OK`、`pose_valid_ratio = 1.0`：

| 水平 FOV | 可观测性阈值 | 位置误差均值 (m) | p90 (m) | confidence 均值 |
|---|---|---|---|---|
| 360° | 关闭 | 0.0417 | 0.0435 | 0.831 |
| 180° | 关闭 | 0.0687 | 0.0703 | 0.835 |
| 120° | 关闭 | 0.0909 | 0.0934 | 0.844 |
| 120° | 0.05 | 0.1078 | 0.2420 | 0.838 |
| 90° | 0.05 | 0.0794 | 0.0805 | 0.843 |
| 60° | 关闭 | 0.1181 | 0.1189 | 0.853 |
| 60° | 0.05 | 0.0661 | 0.0670 | 0.846 |
| 45° | 0.05 | 0.0637 | 0.0664 | 0.830 |
| 30° | 0.05 | 0.0905 | 0.0916 | 0.838 |

结论三条：① 静止定位对水平视场的容忍度远高于预期，压到 30° 仍全程 OK，误差 0.09 m 远在
A2 判据 0.30 m 之内；② 无约束时误差随视场收窄单调上升（360° 0.042 → 60° 0.118），因为可用
约束变少；③ 方向约束只在 60° 及以下有收益（60° 从 0.1181 降到 0.0661，降 44%），在 120° 时
反而制造抖动（p90 0.242）——视场够大时投影会把本来可观的修正量砍掉，阈值 0.05 偏保守。

Reason: 「小 FOV 能不能定位」是本仓必须能回答的问题，而答案依赖本场地几何与先验图粒度，
不能引用通用结论。方向约束的适用边界同样只能实测——它在小视场救场、在大视场捣乱，这一点
如果只凭直觉配置会直接变成负收益。

Alternatives: 只用理论分析（`H` 的特征值谱）给结论——已被本数据否证：30° 时信息矩阵明显
秩亏，但配准仍稳定收敛，说明「秩亏」不等于「不可用」，最终判据必须是闭环状态而不是矩阵条件数。

Rejected: 把方向约束默认打开——120° 档的抖动说明默认开会在正常视场下劣化定位；默认仍是关闭
（`observability_ratio_threshold: 0.0`），由兵种配置按实际视场决定是否启用。

Consequences: 兵种配置可以放心用小视场（例如仅保留下半视场避开车体遮挡），但要同时确认
`observability_ratio_threshold` 的取舍；本表只覆盖静止条件，运动条件下的视场下限仍是未测项，
需要按 D22 的方式用实车或录制数据复核。

状态：实验完成（2026-10-30）。

## D27 性能：GICP 对象跨帧复用 + 事后 profile，不引入 small_gicp

Context: i6 要求在不动依赖的前提下拿性能。离线基准（真实 `rmul_2027.pcd` 0.25 m 体素 target
4137 点、每帧换一块区域的 source、20 帧取均值，容器内 `foray_sim_gui`）暴露出 D21 的一条
判断是错的：D21 写「复用同一对象跨帧就等于让 source 协方差与新 source 错配」，据此选择了
每帧新建 GICP。

Decision:
1. `PclGicpAligner` 内部持有一个复用的 `pcl::GeneralizedIterativeClosestPoint`，**只在
   target 换成另一份点云时**调用 `setInputTarget()`；每帧只 `setInputSource()`。PCL 类型
   藏在 pimpl 后面，头文件仍不含任何 PCL 头（D21 的隔离约束继续成立）。
2. 节点新增 `profile_enabled`（默认 `false`），每 5 s 打印 `to_pcl / voxel / align` 平均值
   与配准率；关闭时只有一次判空。
3. 不引入 `small_gicp`（i7 评估结论）。

Reason: 三组基准的原始数据——

| 方案 | 单帧均耗 |
|---|---|
| A 每帧新建 + `setInputTarget`（改造前） | 73.37 ms（第二轮 61.77 ms） |
| B 复用对象但每帧仍 `setInputTarget` | 77.24 ms |
| C 复用对象、只 `setInputTarget` 一次 | 40.57 ms（第二轮 30.61 ms） |

B ≈ A 证明 `setInputTarget()` 在 PCL 1.12 里每次都会重算 target 协方差与 KdTree（占单帧
四成以上），而不是只在协方差为空时算一次；C 因此把这一半固定成本摊到了地图加载。**正确性
同样验证过**：C 与 A 的 `getFinalTransformation()` 最大差 `0.000e+00`、`getFitnessScore()`
最大差 `0.000e+00`——`setInputSource()` 会清 `input_covariances_`，逐帧换 source 不会沿用
上一帧协方差，D21 那句担心不成立，已在本条更正。实测提速 **2.02×**（61.77 → 30.61 ms）。

关于 `small_gicp`（调研：JOSS 2024、MIT、header-only，官方基准比 `pcl::GICP` 快 2.4×、并行
降采样快 3.2×、KdTree 构建快 6×，并直接暴露 6×6 Hessian `result.H`，对 D23 的可观测性
判定有额外价值）：本轮**不引入**。理由是收益口径不同——2.4× 是第三方自己的对照基准，与
本仓实测的 2.02× 不是同一把尺子；而引入它要新增一条依赖、要扩 CI 矩阵、要过许可证审查，
在没有「同场景同数据」对照数据前不值得。D25 里「提速 <15% 不引入」的门槛依然有效，本项
作为编译期可选后端继续留在评估清单上。

Alternatives: ① 继续每帧新建——已被基准与实测否定，白扔一半时间；② 把 GICP 参数调成
`correspondence_randomness` 等调低——没有实测支撑，等于凭空多一个没人验证过的旋钮（D21
同样的理由）；③ 在 `align()` 里改用 `align(output, input, guess)` 三参重载——PCL 1.12 没有
可用的三参签名，基准里试过直接编不过。

Rejected: 把 target 缓存的判定条件写成「target 点数相同即视为同一份地图」——两份不同的
地图完全可能点数相同，错用的后果是位姿静默错位，比慢几十毫秒严重得多。缓存判定只认
对象身份（指针），地图在 `load_map()` 里整份重建，换图必然换对象。

Consequences: 配准单帧从 ~73 ms 降到 ~31 ms（本机、Onboard model 量级），意味着 5 Hz 配准
的 CPU 占用实打实减半，为 D23 的 `NormalCache` 等额外计算腾出了预算。风险是「同一对象被
两条线程同时 align」——当前节点是单线程定时器，且头文件已删除拷贝构造，将来若改多线程必须
按实例拆分，不能共享。

状态：已实现（2026-10-30）。


---

## D28 审计整改只改根因：不新增参数、不改契约、不补静默兜底

Context: `audit/tasks/02-foray_localization.md` 对 `feat/fov-observability-ekf` @ `25e5bf9`
列出 20 条发现（1 blocker + 6 high + 11 medium + 2 low），审计方的一句话结论是
「算法本身写得不错，问题全在 ROS 边界，而且到处都在静默降级」。任务书同时给了硬约束：
一条发现一个提交、每条都要留下能复现「改前坏」的离线回执、不许改测试期望值来让测试变绿。

Decision: 本轮整改的五条自定纪律——
1. 只改根因。落到执行器/回调组/QoS/参数声明顺序/契约这些边界上，不在调用点加减速器。
2. **不新增 ROS 参数、不改 `.msg` / `.srv`**。需要新语义时用 `constexpr` 常量与内部结构承载
   （如 `RegistrationSample::has_registration_quality`、PCD 加载上限），`README.md` 的参数表
   44 项与 `contracts.sha256` 保持不动——接口面每动一次，下游与实际部署就多一次同步风险。
3. 要么正确，要么响亮地失败。TF 查不到就丢帧并 `WARN`，不拿单位阵冒充；地图加载失败就
   `LOST`，不广播零位姿。
4. 每条发现配一条能在父提交上复现失败的测试，先记录红、再改实现记录绿，回执进回报。
5. 只在本仓动手。跨仓（协议、TF 帧链、submodule）与无实车数据无法取证的项，写进回报的
   「未修」与「需要跨仓协调」，不越界改别人的仓库、不改 `audit/`。

Reason: 静默降级是这次审计最大的系统性问题——`odometry timeout` 把「我算太慢」写成
「里程计断了」、`pose_valid` 没人读、失败路径返回单位阵、参数声明顺序让两个安全旋钮永久
失效。这些都不是算术错误，而是「出错时假装没出错」的边界代码。要治它，整改本身就不能用
兜底式补丁：一旦在调用点加个单位阵或补一个 `if (!finite) return`，问题只是被搬到下一层，
审计的判据（LOST 一帧 TF 都不许发）也就永远无从验证。

Alternatives: 顺势把新语义做成 ROS 参数（可现场调）；顺带把 `LocalizationState` 加字段
承载协方差/来源明细；直接引第三方定位框架替换节点层。
Rejected: 加参数会把「安全旋钮有没有生效」这件事再外包给配置文件，本轮恰恰是要删掉这类
隐性开关（L5 就是两个参数声明晚了一行的后果）；改 msg 会触发 `contract-lock` 与下游
`foray_platform` 的同步，收益与风险不匹配，协方差用现有 `PoseWithCovariance` 就能表达；
替换框架不是整改，是重写。

Consequences:
- 后续 D30 起的整改条目都受本条约束：能用内部结构表达的一律不新增参数，能用现有 msg
  字段表达的一律不改契约。
- 每条整改都要连带一个测试；`foray_localization_ros` 的测试从 2 个文件涨到 3 个以上，
  构建时间与 `colcon test` 时长随之上升（节点级用例靠 `TIMEOUT` 兜底）。
- 回报里必须区分「已修（有回执）」「未修（有理由）」，不允许「看起来修好了」。

状态：实施中（2026-10-31）。

---

## D29 节点级可测试性：节点实现进库、可注入配准后端、诊断计数

Context: 整改的 P0（L1）与三条 high（L2/L4/L5）验收条件都是**集成级**的——
「注入 100–200 ms 配准耗时后不再出现 odom timeout」「地图加载失败时 `map → odom` 一帧都不发」
「`pose_source: auto` 且雷达断流时才接受外部位姿」「`min_source_points: 200` + 30 点云要
`DEGRADED`」。但 `foray_localization_ros` 只有一个 `conversion.cpp` + `adapters.cpp` 的库，
`localization_node.cpp` 直接挂在 `add_executable` 下，测试不但构造不到节点，连符号都拿不到：
父提交上把测试文件加进去、只把节点从库里摘掉，链接直接报
`undefined reference to 'foray::localization_ros::LocalizationNode::LocalizationNode(
rclcpp::NodeOptions const&, std::unique_ptr<foray::localization_map::Aligner>)'`。

Decision:
1. `localization_node.cpp` 移入 `foray_localization_ros_lib`，`localization_node` 可执行文件
   只保留 `localization_node_main.cpp`；安装产物（可执行位置、库名）不变。
2. 构造函数增加第二个参数 `std::unique_ptr<foray::localization_map::Aligner> aligner = nullptr`：
   非空即替代 `make_aligner()`，并打印「配准后端: 注入（测试）」。生产入口不传，行为不变。
3. 节点新增公开的 `Diagnostics` 结构（`odom_received / odom_processed / max_odom_lag_sec /
   align_runs / max_align_ms / tf_published`）与 `diagnostics()`；`max_odom_lag_sec` 是
   「节点时钟 `now()` − 消息 `header.stamp`」的窗口最大值——同一时钟域，用来把
   「没收到 odom」与「收到了但排队没处理」分开。
4. 测试在进程内构造节点，与观测节点一起跑在 `MultiThreadedExecutor` 上，输入输出全走真实
   话题与 `/tf`（`test/localization_node_test_support.hpp` 提供临时地图、假后端、驱动循环）。

Reason: 三条都要控制「配准耗时」这个自变量——真 GICP 的耗时与数据强相关，没法当控制变量；
注入一个睡眠固定时长的后端，才能把「执行器是否挡住 odom」与「配准本身慢不慢」分开。
断言也不能读日志：文本一变测试就废，而计数与时间量是同一份事实的另一种表示。

Alternatives: 用 `launch_testing` 起真进程 + `ros2 bag play`；把配准耗时做成参数；解析日志。
Rejected: `launch_testing` 拿不到节点内部计数，回放还依赖包体与时间源，比进程内构造慢一个
量级且更脆；参数化耗时要新增参数，违反 D28 ②；解析日志等于把断言绑在提示语上，审计禁的就是
这种「文案当契约」。

Consequences:
- `foray_localization_ros_lib` 的符号面变大（含节点），但它是仓内包、不对外承诺 ABI。
- `Diagnostics` 是「测试与运维共用」的观测面：读时持 `mutex_`，不能在回调内部再调用它。
- 夹具里刻意用 `MultiThreadedExecutor`：生产入口也是它，用单线程执行器就测不出 D30 的
  回调组划分是否真的解开了阻塞。

状态：已实现（2026-10-31）。

## D30 执行器与回调组划分：谁阻塞谁独占一组

Context: L1 的故障链不是「配准慢」，而是**阻塞位置错了**。父提交的
`localization_node_main.cpp` 用 `rclcpp::spin`（单线程），且所有订阅、定时器、服务都在节点的
默认回调组（`MutuallyExclusive`）里：5 Hz GICP 单次上百毫秒，同期 200 Hz 的
`/aft_mapped_to_init` 回调根本拿不到执行权，`localization_node.cpp` 里的 `QoS(50)` 队列在
0.25 s 内被打满、`last_odom_ns_` 停止推进，`health.cpp` 于是判 `odom_age > odom_timeout_sec`，
把「本节点算得太慢」写成 `odometry timeout` 并转 LOST、TF 停发。
实测（注入 200 ms 后端、5 Hz 配准 / 50 Hz 状态 / 200 Hz 里程计，6 s）：

| 指标 | 父提交 | 本决定之后 |
|---|---|---|
| 进入 `OK` | 否（停在 `no valid registration during initialization`） | 是 |
| 里程计积压峰值 | 201.4 ms | 11.8 ms |
| 实际配准次数 / 6 s | 15 | 30 |
| 状态发布频率 | 165 条 / 6 s（27.5 Hz） | 300 条 / 6 s（50 Hz） |

Decision:
1. 入口换 `MultiThreadedExecutor`，线程数 `clamp(hardware_concurrency, 2, 4)`。
2. 回调组按「谁阻塞谁独占」划分，六个组全部 `MutuallyExclusive`：`odom_group_`
   （odometry + external_pose）、`cloud_group_`（点云）、`align_group_`（配准定时器）、
   `obstacle_group_`（聚类定时器）、`publish_group_`（state + TF 定时器）、
   `service_group_`（重定位服务）。
3. **组间并行只解决「互不阻塞」**：共享状态仍由唯一的 `mutex_` 仲裁，配准、聚类、TF 查询
   一律在锁外执行；加锁窗口只做「取快照 / 写结果」。
4. 聚类索引随地图建一次：`ResidualClusterer::set_target()` 在 `load_map()` 里调用，
   10 Hz 的 `on_obstacle_timer` 改调 `cluster(source)`（见 D35）。
5. `HealthMonitor::update()` 增加 `odom_backlog_sec`，`LOST` 的 `reason` 拆成三支：
   `no odometry received` / `odometry stale: Xs (no samples)` /
   `odometry stale: Xs (processing backlog Xs)`。判据是「窗口内里程计帧的最大排队时长」
   （`Diagnostics::max_odom_lag_sec`，节点时钟同域）是否超过 `0.5 × odom_timeout_sec`。

Reason: 报错文案是现场的第一个分诊器。同一句 `odometry timeout` 同时覆盖「传感器没数据」
与「你算得太慢」，会让排查方向系统性地指向驱动而不是本节点——L1 之所以从 5 Hz GICP 一路
演化成「LOST 后靠重定位恢复」，就是因为每次都归因到里程计。

Alternatives: 调大 `odom_timeout_sec`；给配准降频；把配准搬到独立线程 + 自己管队列。
Rejected: 调大超时只是把误诊推后（L1 的验收就是不许出现该误诊）；降配准频率拿定位精度换
健康状态；自建线程要复制 executor 的信号、生命周期与取消语义，而 rclcpp 已经有回调组。

Consequences:
- 回调组只是「不互相阻塞」。任何在回调里做的长耗时都会提高对应组被占用期间的排队深度，
  新增慢回调必须同时说明它属于哪个组，或新增一个组。
- `Diagnostics::max_odom_lag_sec` 是窗口峰值、不自动清零，现场读数应配合 `profile_enabled_`
  的 5 s 窗口日志一起看。
- 状态机仍然只在「没有新鲜里程计」时判 LOST：本决定只改文案与诊断，不改失效判据。

状态：已实现（2026-10-31），回执见 `plan.md` M10 与提交 `L1`。

## D35 聚类索引生命周期：与地图同生共死，换图必须显式重设

Context: `ResidualClusterer::cluster(source, target)` 每次调用都遍历整张先验地图建哈希格
（`residual_clusterer.cpp` 原 `:55-59`），而 `on_obstacle_timer` 以 10 Hz 调它。这份索引只依赖
地图、不依赖当前帧，与 `map_index_` / `map_normals_`（D25）是同一类开销。

Decision:
1. `ResidualClusterer` 增加 `set_target(const PointCloud&)` 与 `cluster(const PointCloud&)`，
   索引（点 + 哈希格）一起存进内部 `TargetGrid`，由 `load_map()` 在装载完地图后**只调一次**。
2. 换地图不会自动重建索引：`set_target` 是显式动作，`has_target()` 只是查询。
   旧的两参数 `cluster(source, target)` 保留，供测试与一次性调用。
3. `TargetGrid` 持有目标点云的副本，而不是指向 `map_.cloud` 的指针：这样「用 A 地图的格子
   查 B 地图的点」不可能编译通过，也不会因为 `map_ = std::move(map)` 变成悬垂引用。

Reason: 静默复用旧地图判定残差，会把整片新地图报成动态物体，而故障形态是「障碍物突然变多」，
与传感器噪声不可区分。显式 `set_target` 让这条约束在调用点上可见。

Consequences:
- 地图点云多存一份（`map_voxel_leaf_m` 降采样后，8 万点约 2.6 MB）。这是不用悬垂指针换来的。
- **硬约束**：`load_map()` 是目前唯一的地图装载点；将来若增加换图/热更新路径，必须同时调
  `set_target` 与重建 `map_index_` / `map_normals_`，否则聚类用的是旧地图。

状态：已实现（2026-10-31）。

## D36 地图不可用即 `LOST`，TF 闸门只放行 `OK` / `DEGRADED`

Context: 父提交的 `load_map()` 三条失败路径（`map_manifest` 未设置、`load_prior_map`
失败、点数超限）都只 `RCLCPP_ERROR` 就 `return`：`has_map_` 保持 `false`，
`on_align_timer()` 在 `!has_map_` 处提前返回、从不喂健康状态机，于是状态永远停在
`INITIALIZING`；而 `publish_tf()` 的闸门写的是「`state == kLost || !has_odom_` 就返回」，
`INITIALIZING` 不在其中——一个没有地图、没有任何一帧配准证据的节点，照样以 20 Hz 广播
`map → odom`，内容就是构造时的恒等矩阵。

Decision:
1. 三条失败路径统一走 `mark_map_unavailable(reason)`：记下 `map_error_`，把健康状态压进
   `LOST`，reason 以 `map unavailable:` 开头（后接具体成因）。
2. `publish_state()` 在 `!has_map_` 时每拍继续喂 `enter_lost()`：状态机的「变坏」不能依赖
   「某个回调恰好运行」，否则回调越少、状态越健康——正好反过来。
3. `publish_tf()` 闸门改成**只放行 `kOk` / `kDegraded`**。比 README 的「LOST 停发」
   更紧：只有被配准证明过的状态才允许发布，`INITIALIZING` 一律不发。

Reason: 健康状态的唯一价值是「下游能据此决定要不要用这个位姿」。发一条恒等矩阵的
`map → odom`，下游拿到的是一条**语法正确、语义为零**的变换，而且它和真值长得一模一样——
这正是审计 §4.1 说的静默降级。改成「一帧都不发」，下游的 TF 查询会直接失败，故障在
第一秒就可见。

Alternatives: 发恒等矩阵 + `WARN`（审计明令禁止）；把 `map → odom` 的发布挪到
`on_align_timer()` 里（改动面更大，且 TF 频率被配准频率绑死）。
Rejected: 前者把「没有地图」降级成日志；后者会把 20 Hz TF 变成 5 Hz，违反 README 接口表。

Consequences:
- 没有地图时 `/tf` 上根本没有 `map → odom`，`foray_navigation` 的启动顺序必须容忍这一点
  （它本来就要求先判 `pose_valid`）。
- 地图加载失败后节点不再「看起来在正常工作」：这是有意为之，属于 D28 ①的「响亮地失败」。
- `load_map()` 只在构造期调用，`mark_map_unavailable()` 因此不需要额外的「重试/恢复」路径；
  若将来加地图热重载，必须同时清 `map_error_` 并 `health_.reset()`。

状态：已实现（2026-10-31）。
