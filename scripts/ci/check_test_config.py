#!/usr/bin/env python3
"""被测配置对齐生产门禁：生产值 ≠ 代码默认值的参数，必须在测试里被按名字设置过。

为什么需要它——`check_params.py` 校验的是「三处**声明**一致」：README 参数表、
`declare_parameter` 默认值、兵种配置的键集合。它对下面这种情形完全无感：

    declare_parameter<double>("observability_ratio_threshold", 0.0)   // 代码默认：投影关闭
    sentry.yaml: observability_ratio_threshold: 0.05                  // 生产：投影开启
    测试 harness: 从不设置它                                          // 被测：投影关闭

三处都写 0.0，`check_params.py` 全绿；而 sentry 兵种真正跑的那条分支
（`observability_enabled = threshold > 0.0`）在整个节点测试 suite 里**一次都没执行**。
2026-11-17 的实测：14 个参数的生产值不等于代码默认值，其中 13 个在 `src/*/test/`
里从未按名字出现过。可观性投影的两个米级缺陷（decision.md D62）就是在这个盲区里
活了 14 天、被「修」过两次。

断言（棘轮式，只许缩不许长）：
  1. 生产非默认参数 − 测试已覆盖 − 债务台账 == ∅   （不许新增未覆盖的生产开关）
  2. 债务台账 − 当前未覆盖集合 == ∅                （已补覆盖或已失效的条目必须删掉）

第 2 条是这套门禁的关键：没有它，台账会变成一个越写越长、永远没人回头清的垃圾场，
门禁也就退化成「只要登记一下就放行」。有了它，补上覆盖的那一次提交**必须**同时
删掉台账条目，否则红——债务只能减少。

「已覆盖」的判定：参数名以字符串字面量形式出现在 `src/*/test/*.cpp|*.hpp` 的
非注释行里。刻意不匹配具体写法（`rclcpp::Parameter("x", …)` 与
`overrides.emplace_back("x", …)` 都算），因为 harness 的注入方式会变，
而这条门禁要守的是「这个名字在测试里被真的用到过」，不是某种调用语法。

用法：
    python3 scripts/ci/check_test_config.py     # 校验，失败 exit 1
"""

from __future__ import annotations

import sys
from pathlib import Path

# 复用参数门禁已经写好的三个解析器，避免同一套 yaml/C++ 解析规则出现两份实现
# （那正是本仓 C1 类缺陷的成因）。以脚本方式运行时 sys.path[0] 就是本目录；
# 被 import 时（例如 pytest 直接收集本文件）需要显式补上。
sys.path.insert(0, str(Path(__file__).resolve().parent))

import check_params  # noqa: E402  - 必须先补 sys.path 再 import

CONFIG_DIR = Path("src/diesel_localization_bringup/config")
TEST_GLOB = "*/test/*"
TEST_SUFFIXES = frozenset({".cpp", ".hpp"})

# ---------------------------------------------------------------- 债务台账
# 每一条都是「已知没有测试覆盖的生产非默认参数」，必须写明**为什么现在测不到**
# 与**什么条件下必须补**。台账不是豁免清单：它是一份会到期的人工债，
# 条目一旦不再成立（补了覆盖、参数被删、生产值改回默认），门禁会要求删掉它。
#
# 分组依据是「为什么测不到」，不是「属于哪个兵种」——同一组共享同一条触发条件。
DEBT_LEDGER: dict[str, str] = {
    # 配准调参：节点测试用注入式 aligner（localization_node_test_support.hpp 的
    # BusyAligner / FixedDelayAligner）替掉真实配准后端，这四个参数在节点层是惰性的，
    # 设了也不改变任何被测行为。触发条件：节点测试改用真实 aligner，
    # 或 map 层新增按 ROS 参数构造 aligner 的用例时，必须把它们移出本台账。
    "align_max_correspondence_m": "节点测试注入假 aligner，配准调参在节点层惰性",
    "align_max_iterations": "节点测试注入假 aligner，配准调参在节点层惰性",
    "residual_threshold_m": "节点测试注入假 aligner，配准调参在节点层惰性",
    "voxel_leaf_m": "节点测试注入假 aligner，配准调参在节点层惰性",
    # 健康阈值：状态机本身有用例，但全部走代码默认值；heavy 兵种这四项与默认值不同。
    # 触发条件：改动 health.cpp 的判据、或新增 heavy 兵种验收时，必须补一条按 heavy
    # 口径构造节点的用例，并把对应条目移出本台账。
    "ok_inlier_ratio": "健康状态机用例只走代码默认阈值，缺 heavy 口径用例",
    "ok_mean_residual_m": "健康状态机用例只走代码默认阈值，缺 heavy 口径用例",
    "degraded_inlier_ratio": "健康状态机用例只走代码默认阈值，缺 heavy 口径用例",
    "degraded_mean_residual_m": "健康状态机用例只走代码默认阈值，缺 heavy 口径用例",
    # 动态障碍聚类：core 层用结构体参数直测，不经 ROS 参数通道。
    # 触发条件：聚类参数改为可运行期调整、或出现聚类相关的实车缺陷时，
    # 必须补节点层用例并把两条移出本台账。
    "cluster_tolerance_m": "聚类在 core 层按结构体参数直测，未过 ROS 参数通道",
    "max_cluster_size": "聚类在 core 层按结构体参数直测，未过 ROS 参数通道",
    # FOV 裁剪：sentry 用 30°，而仿真装配把它覆盖成 360°，两条路径都没有节点用例。
    # 触发条件：改动 FOV 裁剪逻辑、或新增窄 FOV 兵种时，必须补一条 30° 口径用例。
    "fov_horizontal_deg": "sentry 30° 与仿真 360° 两条 FOV 路径都无节点用例",
}


def production_non_defaults() -> dict[str, dict[str, str]]:
    """返回 {参数名: {配置文件名: 生产值}}，只含与代码默认值不等价的项。"""
    declared = {
        node: check_params.parse_source_params(source)
        for node, source in check_params.NODE_SOURCES.items()
    }

    result: dict[str, dict[str, str]] = {}
    for config in sorted(CONFIG_DIR.glob("*.yaml")):
        for node, values in check_params.parse_config_values(config).items():
            if node not in declared:
                # 未知节点由 check_params.py 负责报错，这里不重复判定。
                continue
            for name, value in values.items():
                default = declared[node].get(name)
                if default is None:
                    continue
                if not check_params.same_value(value, default):
                    result.setdefault(name, {})[config.name] = value
    return result


def covered_names() -> set[str]:
    """收集测试源里出现过的参数名字符串字面量（注释行不算）。"""
    names: set[str] = set()
    for path in sorted(Path("src").glob(TEST_GLOB)):
        if path.suffix not in TEST_SUFFIXES or not path.is_file():
            continue
        for line in path.read_text(encoding="utf-8").splitlines():
            code = line.split("//", 1)[0]
            # 只取成对双引号之间的内容；参数名不含空格，据此排除散文里的引用。
            for chunk in code.split('"')[1::2]:
                if chunk and " " not in chunk:
                    names.add(chunk)
    return names


def main() -> int:
    non_defaults = production_non_defaults()
    if not non_defaults:
        print(
            "::error::没有解析出任何生产非默认参数，检查兵种配置与 NODE_SOURCES 映射",
            file=sys.stderr,
        )
        return 1

    covered = covered_names()
    uncovered = {name for name in non_defaults if name not in covered}

    failures: list[str] = []

    # ---- 1. 不许新增未覆盖的生产开关 ----
    for name in sorted(uncovered - set(DEBT_LEDGER)):
        where = "、".join(
            f"{config}={value}" for config, value in sorted(non_defaults[name].items())
        )
        failures.append(
            f"参数「{name}」的生产值不等于代码默认值（{where}），"
            "但 src/*/test/ 里没有任何用例按名字设置它——"
            "生产跑的那条分支在测试里从未执行。"
            "请补一条按生产口径构造节点的具名用例；"
            "确实测不到时，在 check_test_config.py 的 DEBT_LEDGER 登记理由与触发条件"
        )

    # ---- 2. 台账只许缩不许长 ----
    for name in sorted(set(DEBT_LEDGER) - uncovered):
        if name not in non_defaults:
            reason = "它已不再是生产非默认参数（被删除或生产值改回默认）"
        else:
            reason = "它已被测试覆盖"
        failures.append(
            f"DEBT_LEDGER 里的「{name}」应当删除：{reason}。"
            "台账是棘轮，不是豁免清单——债务减少时必须同步删条目"
        )

    if failures:
        for failure in failures:
            print(f"::error::{failure}")
        print(
            f"\n被测配置对齐生产校验失败：{len(failures)} 处。\n"
            "依据 decision.md D63：门禁校验「声明一致」不等于「被测配置==生产配置」，\n"
            "后者必须单独断言，否则生产开启的分支可以长期零覆盖。",
            file=sys.stderr,
        )
        return 1

    print(
        f"被测配置对齐生产校验通过：{len(non_defaults)} 个生产非默认参数，"
        f"{len(non_defaults) - len(uncovered)} 个已有具名测试覆盖，"
        f"{len(DEBT_LEDGER)} 个在债务台账内（只许缩不许长）。"
    )
    for name in sorted(non_defaults):
        where = "、".join(
            f"{config}={value}" for config, value in sorted(non_defaults[name].items())
        )
        mark = "债务" if name in uncovered else "覆盖"
        print(f"  [{mark}] {name:32s} {where}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
