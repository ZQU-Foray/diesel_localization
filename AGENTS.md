# AGENTS.md — AI 协作规范

本仓遵循组织 [贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)，
以及 [`foray_docs`](https://github.com/ZQU-Foray/foray_docs) 的算法结构与仓库结构约定。

## 需求来源

**`README.md` 是本仓唯一的需求来源。** 不得实现 `README.md` 中不存在的需求。

## 必读

| 文档 | 内容 |
|---|---|
| `README.md` | 本仓定位、接口、上下游 |
| `tree.md` | 目录结构与各目录职责 |
| `plan.md` | 当前计划与进度 |
| `decision.md` | 关键工程决策（含破坏性接口变更留痕） |
| `foray_docs/algorithm_structure.md` | 全局算法结构 |
| `foray_docs/repository_structure.md` | 全局仓库结构 |

## 文档职责

**每条事实只有一个权威出处。** 其余位置需要它时一律**引用**（文件名 + 标题或 `D` 编号），
不复述数值、清单与理由。理由很直接：同一件事写在两处的第一天就开始分叉，而分叉只由
有判断力的人发现——`README.md` 的参数表曾经错了 11 个名字，`tree.md` 与自己的「记录约束」
段互相矛盾过（`decision.md` D15/D18 都是这么来的）。

| 文档 | 只回答 | 不写 |
|---|---|---|
| `README.md` | 本仓**做什么、怎么做**：定位、方案、接口、参数、地图与健康契约（唯一需求来源） | 进度、决策理由、目录清单 |
| `plan.md` | **做到哪了、下一步做什么**：里程碑勾选、当前状态、模糊点、验收条件 | 接口细节、决策理由、目录清单 |
| `decision.md` | **为什么这么定**：取舍、被否决的方案、破坏性变更留痕 | 现状描述、参数清单、进度 |
| `tree.md` | **仓库里有什么**：目录与文件职责，与 `git ls-files` 双向镜像 | 需求、进度、决策理由 |
| `AGENTS.md` | **怎么协作**：规范、必读清单、文档分工、Git 流程 | 技术细节（指向 `README.md` / `decision.md`） |
| `docs/probe-style.md` | 仅供门禁自检注入缺陷（`decision.md` D19） | 任何需求语义；门禁与人都不读其内容 |

判据一句话：**这份文档删掉，谁会失去信息？** 只有一处会失去的那条事实，就该归那一处所有。

- 参数默认值、速率、阈值只写在 `README.md` 参数表；CI 作业清单只写在 `.github/workflows/ci.yml`
  （其他文档说「以该文件为准」，不抄数字）。
- 决策条目只在 `decision.md` 写一次；`plan.md` 完成里程碑时引用 `D` 编号，不复述理由。
- 新增或删除文件必须同一次改动更新 `tree.md`，否则 CI 的 `doc-sync` 作业会红。
- 改文档前先确认「这条事实的权威出处是哪里」，再决定改哪一份——**不要在两处各改一半**。

## 必须

- **Conventional Commits**——scope 见 `foray_docs/repository_structure.md` §6.6
- 按逻辑单元**小步提交**，不得一次性提交全部代码
- 维护 `plan.md` 最新状态，任意时刻可中断恢复
- 每完成一个逻辑单元，同步更新 `tree.md` 与 `decision.md`
- 公共 API（ROS 2 话题/服务/参数）必须有 Docstring / Doxygen
- **魔法数字必须注释来源**（规则手册页码、标定值、赛季）
- 4 空格缩进 · LF · UTF-8 · 文件末尾一个空行

## 禁止

- 修改与任务无关的文件
- 删除用户提供的源材料
- 破坏性 Git 操作（`reset --hard` / `push --force` / 非必要 `rebase`）
- 引入无必要依赖
- **违反 `.diesel-layer` 声明的依赖方向**

## Git Workflow

```
main          ← 稳定版本，经过测试与 Review
  └─ dev      ← 开发分支，所有更改在此分支调试
       └─ feat/xxx · fix/xxx · docs/xxx
```

- **所有 PR 的目标分支是 `dev`，不是 `main`**（文档类仓库除外）
- 功能分支从 `dev` 切出，用 `git rebase dev` 同步上游
