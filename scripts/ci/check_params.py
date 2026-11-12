#!/usr/bin/env python3
"""参数一致性门禁：README 参数表 ↔ declare_parameter ↔ 兵种配置三处必须一致。

为什么需要它——`decision.md` D15 的 Reason 写得很直白：
「文档漂移靠人眼 Review 抓不住（11 个名字错了很久没人发现），只有可执行断言能长期生效」。
D16 删掉原 `check_param_consistency.py` 后，这条约束退化成「PR 人工核对」，
等于把已经实证失效的防线又交回给人眼。本脚本把它恢复成 CI 断言，
但不恢复那个 Python 工具包——只用标准库，不引入包依赖。

三处定义（README「关键参数」段自称「本表是实现之外的唯一参数清单」）：
  1. README 两张表：参数名 + 默认值 + 所属节点（表头括号里）
  2. 节点源码 `declare_parameter<T>("名字", 默认值)`
  3. `bringup/config/*.yaml` 的 `ros__parameters` 键集合

断言：
  - README 名字集合 == declare_parameter 名字集合（按节点分组）
  - README 默认值 == 源码默认值（数值比较容忍 0.1 与 0.10 写法差异）
  - 兵种配置键 ⊆ 已声明参数（防拼错）
  - 兵种配置键 ⊇ 已声明参数 − launch 注入集合（防漏配）

用法：
    python3 scripts/ci/check_params.py          # 校验，失败 exit 1
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

README = Path("README.md")
CONFIG_DIR = Path("src/diesel_localization_bringup/config")

# README 表头括号里的节点名 → 该节点的源码文件。
# 没有机械规则能从 `diesel_localization` 推出 `localization_node.cpp`，所以显式列映射；
# README 出现未登记的新节点名时直接报错，而不是悄悄漏检。
NODE_SOURCES = {
    "diesel_localization": Path(
        "src/diesel_localization_ros/src/localization_node.cpp"
    ),
    "diesel_map_server": Path("src/diesel_localization_ros/src/map_server_node.cpp"),
}

# 由 launch 注入、允许不出现在兵种配置里的参数。
# 来源：README「关键参数」段末注 + src/diesel_localization_bringup/launch/localization.launch.py。
LAUNCH_INJECTED = frozenset({"map_manifest", "verify_map_hashes"})

# 位姿自由度总数，与 `src/diesel_localization_core/include/diesel/localization_map/observability.hpp`
# 的 `kDofCount` 相同。本脚本只看 yaml，不解析 C++，所以这里按事实重述一次；
# 两边漂移的后果是「判据悄悄失效」，因此改动时两处必须一起改（decision.md D58）。
K_DOF_COUNT = 6

# declare_parameter<T>("name", default)
DECLARE_RE = re.compile(
    r'declare_parameter<([^>]+)>\(\s*"([^"]+)"\s*,\s*(.+?)\s*\)\s*;'
)
# constexpr double kDefaultStateRateHz = 50.0;
CONSTEXPR_RE = re.compile(
    r"^\s*constexpr\s+(?:double|float|int|std::size_t|size_t|bool)\s+(\w+)\s*=\s*([^;]+);",
    re.MULTILINE,
)


def parse_readme_params() -> dict[str, dict[str, str]]:
    """解析 README「关键参数」段，返回 {节点名: {参数名: 默认值原文}}。"""
    lines = README.read_text(encoding="utf-8").splitlines()

    groups: dict[str, dict[str, str]] = {}
    in_section = False
    current: str | None = None

    for line in lines:
        if line.startswith("### 关键参数"):
            in_section = True
            continue
        if not in_section:
            continue
        if line.startswith(("## ", "### ")):
            break

        heading = re.search(r"（`([^`]+)`）", line)
        if heading and line.startswith("#### "):
            current = heading.group(1)
            groups[current] = {}
            continue

        if current and line.startswith("| `"):
            cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
            if len(cells) < 2:
                continue
            name = cells[0].strip("`").strip()
            default = cells[1].strip("`").strip()
            groups[current][name] = default

    return groups


def parse_source_params(path: Path) -> dict[str, str]:
    """解析源码里的 declare_parameter，返回 {参数名: 默认值表达式原文}。"""
    if not path.is_file():
        raise SystemExit(f"::error::节点源码不存在：{path}")

    text = path.read_text(encoding="utf-8")

    constants: dict[str, str] = {}
    for match in CONSTEXPR_RE.finditer(text):
        constants[match.group(1)] = match.group(2).strip()

    params: dict[str, str] = {}
    for line in text.splitlines():
        if "declare_parameter<" not in line:
            continue
        match = DECLARE_RE.search(line)
        if match is None:
            continue
        name = match.group(2)
        default = match.group(3).strip()
        # 默认值是命名常量时解析成实际值，避免「换了常量名就被当成默认值变更」。
        if default in constants:
            default = constants[default]
        params[name] = default

    return params


def parse_config(path: Path) -> dict[str, set[str]]:
    """解析兵种 yaml 的 ros__parameters 键集合，返回 {节点名: {键}}。

    手写解析而不是引 PyYAML：本文件结构固定为「节点名: / ros__parameters: / 键: 值」，
    且本仓 CI 不装第三方包。缩进按 2/4 空格判定，注释与空行跳过。
    """
    result: dict[str, set[str]] = {}
    node: str | None = None
    in_params = False

    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.split("#", 1)[0].rstrip()
        if not stripped.strip():
            continue
        indent = len(stripped) - len(stripped.lstrip())

        if indent == 0 and stripped.endswith(":"):
            node = stripped[:-1].strip()
            in_params = False
            continue
        if indent == 2 and stripped.strip() == "ros__parameters:":
            in_params = True
            if node is not None:
                result.setdefault(node, set())
            continue
        if indent == 4 and in_params and node is not None:
            result[node].add(stripped.strip().split(":", 1)[0].strip())

    return result


def parse_config_values(path: Path) -> dict[str, dict[str, str]]:
    """解析兵种 yaml 的「键: 值」原文，返回 {节点名: {键: 值}}。

    与 `parse_config()` 同一套手写规则，区别只是保留冒号右侧的值——
    键集合一致不代表值说得通：`max_unobservable_dofs: 6` 也是合法的键值对，
    但它等于把退化判据关掉（审计 L5），只有看值才拦得住。
    """
    result: dict[str, dict[str, str]] = {}
    node: str | None = None
    in_params = False

    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.split("#", 1)[0].rstrip()
        if not stripped.strip():
            continue
        indent = len(stripped) - len(stripped.lstrip())

        if indent == 0 and stripped.endswith(":"):
            node = stripped[:-1].strip()
            in_params = False
            continue
        if indent == 2 and stripped.strip() == "ros__parameters:":
            in_params = True
            if node is not None:
                result.setdefault(node, {})
            continue
        if indent == 4 and in_params and node is not None and ":" in stripped:
            key, value = stripped.strip().split(":", 1)
            result[node][key.strip()] = value.strip()

    return result


def as_number(raw: str) -> float | None:
    """尝试把默认值原文转成数字；失败返回 None。"""
    try:
        return float(raw)
    except ValueError:
        return None


def normalize(raw: str) -> tuple[str, object]:
    """把默认值原文规范化，用于跨处比较。

    返回 (类别, 值)。数字按数值比较（容忍 0.1 与 0.10），其余按去掉引号的字符串比较。
    """
    text = raw.strip().strip("`").strip()
    if text in {"true", "True"}:
        return "bool", True
    if text in {"false", "False"}:
        return "bool", False
    number = as_number(text)
    if number is not None:
        return "number", number
    return "string", text.strip('"')


def same_value(left: str, right: str) -> bool:
    """判断两处默认值是否等价。"""
    left_kind, left_value = normalize(left)
    right_kind, right_value = normalize(right)
    if left_kind == "number" and right_kind == "number":
        return abs(float(left_value) - float(right_value)) < 1e-12  # type: ignore[arg-type]
    return left_value == right_value


def main() -> int:
    failures: list[str] = []

    readme_groups = parse_readme_params()
    if not readme_groups:
        print(
            "::error::README 未解析出任何参数表，检查「### 关键参数」段与「| `名字` |」格式",
            file=sys.stderr,
        )
        return 1

    declared: dict[str, dict[str, str]] = {}
    for node, source in NODE_SOURCES.items():
        declared[node] = parse_source_params(source)

    for node in readme_groups:
        if node not in NODE_SOURCES:
            failures.append(
                f"README 出现未登记的节点「{node}」，请在 check_params.py 的 NODE_SOURCES 补映射"
            )

    # ---- 1. README 名字集合 == declare_parameter 名字集合 ----
    for node, table in readme_groups.items():
        if node not in declared:
            continue
        doc_names = set(table)
        code_names = set(declared[node])

        for name in sorted(doc_names - code_names):
            failures.append(f"[{node}] README 有参数「{name}」但源码未声明")
        for name in sorted(code_names - doc_names):
            failures.append(f"[{node}] 源码声明了「{name}」但 README 参数表没有")

        # ---- 2. 默认值一致 ----
        for name in sorted(doc_names & code_names):
            doc_default = table[name]
            code_default = declared[node][name]
            if not same_value(doc_default, code_default):
                failures.append(
                    f"[{node}] 「{name}」默认值不一致：README={doc_default!r} 源码={code_default!r}"
                )

    # ---- 3. 兵种配置与已声明参数 ----
    configs = sorted(CONFIG_DIR.glob("*.yaml"))
    if not configs:
        failures.append(f"{CONFIG_DIR} 下没有兵种配置，无法校验参数覆盖")

    for config in configs:
        parsed = parse_config(config)
        for node, keys in parsed.items():
            if node not in declared:
                failures.append(f"{config.name}: 出现未知节点「{node}」")
                continue
            code_names = set(declared[node])

            for key in sorted(keys - code_names):
                failures.append(
                    f"{config.name}: 键「{key}」不是 {node} 声明的参数（疑似拼写错误）"
                )

            required = code_names - LAUNCH_INJECTED
            for key in sorted(required - keys):
                failures.append(
                    f"{config.name}: 缺少必需参数「{key}」"
                    f"（非 launch 注入，兵种配置必须给出）"
                )

    # ---- 4. 退化判据不许被写成「等于关闭」 ----
    # `max_unobservable_dofs` 的上限是自由度总数 6（observability.hpp 的 kDofCount）。
    # 写成 6 时「六个方向全不可观」也满足 `<=`，判据永远为真——审计 L5 实测：
    # 配准退化 5/6 方向不可观时位姿照常输出，日志刷 WARN 但没有任何降级。
    # 语义断言：只要方向约束是开着的（threshold > 0），容忍数就必须小于全自由度。
    for config in configs:
        values = parse_config_values(config)
        params = values.get("diesel_localization", {})
        ratio = as_number(params.get("observability_ratio_threshold", ""))
        dofs = as_number(params.get("max_unobservable_dofs", ""))
        if ratio is None or dofs is None or ratio <= 0.0:
            continue
        if dofs >= K_DOF_COUNT:
            failures.append(
                f"{config.name}: observability_ratio_threshold={ratio} 已启用方向约束，"
                f"但 max_unobservable_dofs={dofs:g} ≥ {K_DOF_COUNT}（=全部自由度），"
                "退化判据等于关闭（审计 L5 / decision.md D58）"
            )

    if failures:
        for failure in failures:
            print(f"::error::{failure}")
        print(
            f"\n参数一致性校验失败：{len(failures)} 处。\n"
            "改动任一参数时，必须在同一提交内同步 README「关键参数」表、\n"
            "declare_parameter 与 bringup/config/*.yaml 三处（README 亦为唯一需求来源）。",
            file=sys.stderr,
        )
        return 1

    total = sum(len(table) for table in readme_groups.values())
    print(
        f"参数一致性校验通过：{len(readme_groups)} 个节点、{total} 个参数、"
        f"{len(configs)} 份兵种配置三处一致。"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
