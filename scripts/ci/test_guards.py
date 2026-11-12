#!/usr/bin/env python3
"""门禁自检：注入已知缺陷，证明四道门禁真的会红。

为什么需要自检——`decision.md` D16 的教训是「恒跳过的检查看起来是绿的」。
一个永远不会失败的检查比没有检查更危险：它给出虚假的安全感。所以本文件
对每道门禁注入一种本仓**真实发生过**的漂移，断言它返回 1；再断言干净树返回 0。

pytest 只是运行器，本文件只依赖标准库与被测脚本——CI 里 `pip install pytest`
是测试期工具，不进包依赖，也不改变门禁脚本自身零依赖的性质。

用法：
    python3 scripts/ci/test_guards.py            # 自跑（无第三方依赖），末尾打印汇总
    python3 -m pytest scripts/ci/test_guards.py  # 交给 pytest 运行（需已安装 pytest）

自跑模式下末尾会打印一行 `[==========] N tests from N test suites ran.`，
这是 gtest 的汇总格式：DSH 任务引擎与部分 CI 适配器靠它判定「确实跑了 N 个用例」，
而 `python3 xxx.py` 这样的自定义命令本身不在它们的识别名单里。
"""

from __future__ import annotations

import contextlib
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts" / "ci"

# 注入点选受跟踪的中性文本文件：改它不改变「哪些文件被记录」，
# 只影响各门禁对内容的判断，避免测试自身引发跨门禁的连锁反应。
# 风格探针用专门的 docs/probe-style.md：审计报告之类的一次性分析不入库，
# 门禁自检不能依赖一个被 .gitignore 排除的文件。
STYLE_PROBE = ROOT / "docs" / "probe-style.md"
CONTRACT_PROBE = (
    ROOT / "src" / "diesel_localization_ros" / "msg" / "LocalizationState.msg"
)
PARAM_PROBE = ROOT / "README.md"
TREE_PROBE = ROOT / "scripts" / "ci" / "probe_untracked.py"


def _run(script: str) -> int:
    """在仓根运行一个门禁脚本，返回退出码。"""
    proc = subprocess.run(
        [sys.executable, str(SCRIPTS / script)],
        capture_output=True,
        text=True,
        cwd=ROOT,
        check=False,
    )
    return proc.returncode


@contextlib.contextmanager
def _mutated(path: pathlib.Path, mutate):
    """临时改写文件内容，退出时无条件恢复原字节（含异常路径）。"""
    original = path.read_bytes()
    path.write_bytes(mutate(original))
    try:
        yield
    finally:
        path.write_bytes(original)


# ---------------------------------------------------------------- 基线：干净树必须全绿


def test_baseline_check_style_passes() -> None:
    assert _run("check_style.py") == 0


def test_baseline_check_contracts_passes() -> None:
    assert _run("check_contracts.py") == 0


def test_baseline_check_params_passes() -> None:
    assert _run("check_params.py") == 0


def test_baseline_check_tree_doc_passes() -> None:
    assert _run("check_tree_doc.py") == 0


# ------------------------------------------------- 注入：每种缺陷都必须被对应门禁抓住


def test_style_catches_trailing_whitespace() -> None:
    with _mutated(STYLE_PROBE, lambda b: b + b"trailing spaces here   \n"):
        assert _run("check_style.py") == 1


def test_style_catches_crlf() -> None:
    with _mutated(STYLE_PROBE, lambda b: b.replace(b"\n", b"\r\n", 1)):
        assert _run("check_style.py") == 1


def test_style_catches_extra_blank_line_at_eof() -> None:
    with _mutated(STYLE_PROBE, lambda b: b + b"\n"):
        assert _run("check_style.py") == 1


def test_style_catches_invalid_utf8() -> None:
    with _mutated(STYLE_PROBE, lambda b: b + b"\xff\xfe\n"):
        assert _run("check_style.py") == 1


def test_contracts_catches_field_rename() -> None:
    # pose_valid 是 452d58b 新增的显式位姿可用性位：改名而不同步审计签名，
    # 下游按旧字段名取值的代码会在编译期之外静默失败。
    with _mutated(
        CONTRACT_PROBE, lambda b: b.replace(b"pose_valid", b"pose_valid_x", 1)
    ):
        assert _run("check_contracts.py") == 1


def test_params_catches_readme_default_drift() -> None:
    # D15 实证的失效模式：README 默认值与 declare_parameter 漂移，11 个名字错了很久没人发现。
    with _mutated(
        PARAM_PROBE,
        lambda b: b.replace(
            b"| `map_frame` | `map` |", b"| `map_frame` | `mapframe` |", 1
        ),
    ):
        assert _run("check_params.py") == 1


def test_tree_doc_catches_untracked_listed_file() -> None:
    TREE_PROBE.write_text("# 自检探针，不应被提交\n", encoding="utf-8")
    subprocess.run(
        ["git", "add", "--intent-to-add", str(TREE_PROBE)],
        cwd=ROOT,
        capture_output=True,
        check=False,
    )
    try:
        assert _run("check_tree_doc.py") == 1
    finally:
        subprocess.run(
            ["git", "rm", "-q", "--cached", "--ignore-unmatch", str(TREE_PROBE)],
            cwd=ROOT,
            capture_output=True,
            check=False,
        )
        TREE_PROBE.unlink(missing_ok=True)


# ---------------------------------------------------------------- 自跑入口（无第三方依赖）

TEST_NAMES = sorted(name for name in globals() if name.startswith("test_"))


def _main() -> int:
    """按顺序运行全部用例，返回退出码。汇总行格式见模块 docstring。"""
    failures = 0
    for name in TEST_NAMES:
        try:
            globals()[name]()
        except Exception as exc:  # noqa: BLE001 - 任何异常都是用例失败，不应中断整轮
            failures += 1
            print(f"FAIL  {name}: {exc}")
        else:
            print(f"PASS  {name}")

    total = len(TEST_NAMES)
    print(f"[==========] {total} tests from {total} test suites ran.")
    if failures:
        print(f"[  FAILED  ] {failures} tests.")
        return 1
    print(f"[  PASSED  ] {total} tests.")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
