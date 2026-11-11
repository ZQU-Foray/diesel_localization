"""把 colcon test 写下的 gtest 结果汇总成可断言的覆盖清单。

审计整改（`audit/tasks/02-foray_localization.md`）要求每条发现的验收用例都
「跑过并且全绿」。`colcon test` 只把结果留在 `build/<pkg>/test_results/` 下，
而 CI 的 `build-ros2` 作业只看 colcon 自己的退出码——某个用例悄悄消失不会让它变红。
这里把结果文件汇总成一份清单：总数、逐包明细，以及一组必须存在的用例名。

用法（在 colcon 工作区里跑）：

    python3 -m pytest scripts/ci/test_ros_results.py -q

结果目录按顺序定位：环境变量 `FORAY_RESULTS_ROOT`、仓库根下的 `build/`、
以及从仓库根逐级向上的 `build/`（docker 里仓库挂在 `/ws/src/...`、构建在 `/ws/build`）。
"""

from __future__ import annotations

import os
from pathlib import Path
from xml.etree import ElementTree

import pytest

# 每条审计发现对应的验收用例名 → 发现编号。用例消失就等于该条发现失去证据，
# 所以这里按名字断言，而不是只看总数。
REQUIRED_CASES = {
    # L1：算法路径不再独占执行器（低速配准下健康仍是 OK）
    "SlowAlignmentKeepsHealthOk": "L1",
    # L16：同上，另用单线程对照组证明「改前坏」
    "AlgorithmPathDoesNotStarveTheOdomPath": "L16",
    # L9：配准量测与里程计时刻对齐
    "MapToOdomJitterUnderConstantVelocity": "L9",
    # L6：点云消费时刻与 /tf 打戳一致、配准期间到达的帧不被丢
    "TransformIsStampedWithTheConsumedCloudTime": "L6",
    "CloudArrivingDuringAlignmentIsNotDropped": "L6",
    # L8：best_effort 上游也能收到
    "BestEffortOdometryUpstreamIsReceived": "L8",
    # L7：协方差按残差与内点率填充
    "StateMessageCarriesTheCovariance": "L7",
    "CovarianceNeverClaimsZeroUncertaintyWithoutQuality": "L7",
    "CovarianceFollowsResidualAndInlierRatio": "L7",
    "CovarianceGrowsWhenRegistrationDegrades": "L7",
    "UnobservableDirectionVarianceIsInflatedByScaleSquared": "L7",
    "CovarianceTreatsNonFiniteResidualAsWorstCase": "L7",
    "StateCovarianceReflectsRegistrationQuality": "L7",
    "ExternalSourceCovarianceDoesNotReuseLidarQuality": "L7",
    # L4：位姿来源仲裁
    "AutoPolicyIgnoresExternalPoseWhileLidarIsFresh": "L4",
    "AutoPolicyAcceptsExternalPoseWhenLidarStalls": "L4",
    "AutoPolicySwitchesBackToLidarWhenItReturns": "L4",
    # L5：声明顺序修好后参数真的生效
    "MinSourcePointsParameterTakesEffect": "L5",
    # L4（decision.md D57）：LOST 分两档——从未成功过按 warmup 自恢复，成功过则粘滞到 Relocalize
    "LostBeforeFirstSuccessRecoversFromStartup": "L4",
    "EnterLostIsStickyAfterFirstSuccess": "L4",
    "EnterLostKeepsTheFirstReason": "L4",
    # L3：odom 的 child_frame_id 校验与 TF 补换算
    "OdometryChildFrameIsConvertedThroughTf": "L3",
    "OdometryWithoutChildFrameTfIsDropped": "L3",
    # L2 与 L10：地图装载失败时不发任何 /tf
    "MissingManifestSendsNoTransform": "L2",
    "Sha256MismatchSendsNoTransform": "L2",
    "TruncatedPcdSendsNoTransform": "L2",
    "OversizedPointCountSendsNoTransform": "L2/L10",
    "RejectsDeclaredPointCountAboveLimit": "L10",
    "RejectsNonNumericFieldSize": "L10",
    "RejectsTruncatedAsciiData": "L10",
    "RejectsOversizedFieldBytes": "L10",
    # L18：ascii 坏行被计数并告警
    "SkipsInvalidPoints": "L18",
    "CountsRowsWithMissingFields": "L18",
    "MapWithSkippedRowsStillLocalizes": "L18",
    # L11：收敛阈值拆成米与弧度、预算耗尽要说明原因
    "MeterThresholdDoesNotSetRotationTolerance": "L11",
    "ExhaustedBudgetExplainsItself": "L11",
    # L12：不可观方向权重上限与协方差饱和
    "UnobservableDirectionWeightStaysCapped": "L12",
    "CovarianceStopsGrowing": "L12",
    # L13：非有限量测被丢弃、协方差走 Joseph 形式
    "NonFiniteMeasurementIsDropped": "L13",
    "NonFinitePredictionIsDropped": "L13",
    "NonFiniteResetIsIgnored": "L13",
    "CovarianceStaysSymmetricAndPositive": "L13",
    # L15：可观性力臂取被修正位姿的原点
    "RotationLeverFollowsTheCorrectedFrame": "L15",
    "RotationLeverDefaultsToTheBodyPose": "L15",
    # 附带(a)：聚类新鲜度用节点时钟的收到时刻
    "ObstacleClusteringUsesReceiveTimeNotSensorStamp": "附带(a)",
    # L17：未匹配残差占位值是具名常量
    "UnmatchedPointsReportTheDocumentedResidual": "L17",
}

# L14 只在 PCL 后端编译进来时存在（`FORAY_WITH_PCL=ON`），因此单独断言。
PCL_CASES = {
    "TargetCacheInvalidatesWhenContentChanges": "L14",
    "OptionsTakeEffectAfterSetOptions": "L14",
}


def _candidate_roots() -> list[Path]:
    """按优先级列出可能存放 gtest 结果的工作区根目录。"""
    candidates: list[Path] = []
    override = os.environ.get("FORAY_RESULTS_ROOT")
    if override:
        candidates.append(Path(override))
    repo_root = Path(__file__).resolve().parents[2]
    for ancestor in (repo_root, *repo_root.parents):
        candidates.append(ancestor / "build")
    return candidates


def find_results_root() -> Path:
    """返回第一个含 gtest 结果文件的构建目录。"""
    for candidate in _candidate_roots():
        if any(candidate.glob("*/test_results/*/*.gtest.xml")):
            return candidate
    searched = ", ".join(str(path) for path in _candidate_roots())
    raise AssertionError(f"没有找到 gtest 结果文件（已尝试：{searched}）")


def load_results(root: Path) -> dict[str, list[str]]:
    """读出 `{结果文件名: [用例名, ...]}`，并顺带校验每个文件的汇总计数。"""
    results: dict[str, list[str]] = {}
    for path in sorted(root.glob("*/test_results/*/*.gtest.xml")):
        suites = ElementTree.parse(path).getroot()
        tests = int(suites.get("tests", "0"))
        failures = int(suites.get("failures", "0"))
        errors = int(suites.get("errors", "0"))
        assert (
            failures == 0 and errors == 0
        ), f"{path.name}: {failures} failures / {errors} errors"
        cases = [case.get("name", "") for case in suites.iter("testcase")]
        assert tests == len(
            cases
        ), f"{path.name}: 汇总写 {tests} 个用例，实际列出 {len(cases)} 个"
        results[path.name] = cases
    return results


@pytest.fixture(scope="module")
def results() -> dict[str, list[str]]:
    root = find_results_root()
    loaded = load_results(root)
    total = sum(len(cases) for cases in loaded.values())
    print(f"结果目录: {root}")
    print(f"结果文件: {len(loaded)} 个，用例 {total} 个，failures=0 errors=0")
    return loaded


def test_results_are_not_empty(results: dict[str, list[str]]) -> None:
    assert results, "没有任何 gtest 结果文件"
    assert sum(len(cases) for cases in results.values()) > 0


def test_required_acceptance_cases_present(results: dict[str, list[str]]) -> None:
    present = {case for cases in results.values() for case in cases}
    missing = sorted(name for name in REQUIRED_CASES if name not in present)
    assert not missing, "缺少审计验收用例: " + ", ".join(
        f"{name}（{REQUIRED_CASES[name]}）" for name in missing
    )


def test_pcl_cases_present_when_pcl_results_exist(
    results: dict[str, list[str]],
) -> None:
    pcl_recorded = any("pcl_gicp" in name for name in results)
    if not pcl_recorded:
        pytest.skip("本次构建没有 PCL 后端（FORAY_WITH_PCL=OFF），L14 用例不参与")
    present = {case for cases in results.values() for case in cases}
    missing = sorted(name for name in PCL_CASES if name not in present)
    assert not missing, "缺少 L14 验收用例: " + ", ".join(missing)
