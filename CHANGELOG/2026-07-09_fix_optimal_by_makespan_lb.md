# 修改记录：修正最优性判定（optimal_by_makespan_lb）

- **日期**：2026-07-09
- **涉及文件**：`lacam2/src/planner.cpp`、`lacam2/src/post_processing.cpp`
- **影响范围**：仅「是否声明最优」的**报告字段**，不影响搜索过程、解的质量或任何 makespan / soc 数值。

---

## 1. 问题描述

判定「解是否达到理论下界从而可判为最优」的分支 `optimal_by_makespan_lb` 存在两处叠加缺陷：

1. **量纲不匹配**：用 **makespan**（时间步数）去和 **sum-of-costs 的下界** 比较。两者量纲不同，等式几乎永远不成立。
2. **off-by-one**：直接用 `solution.size()` 当作 makespan，但真实 makespan 为 `solution.size() - 1`（见 `get_makespan()`）。

后果：`optimal_by_makespan_lb` 实际上**恒为 false**，「解触到下界即判最优」这条捷径失效。最优性判定退化为只依赖另一条路径 `optimal_by_open_exhaustion`（OPEN 搜索列表被穷尽）。

> 注意：这不会导致**误报**（不会把非最优解报成最优），只会导致**漏报 / 迟报**——某些解已等于下界、但 OPEN 尚未穷尽的算例本可立即判最优，却要等到 OPEN 穷尽（或超时后仍为非最优）。

### 出错代码

`planner.cpp`（约第 826–832 行，修改前）：

```cpp
int soc_lower_bound = 0;
for (size_t i = 0; i < ins->N; ++i) {
  soc_lower_bound += D.get(i, ins->starts[i]);
}
const bool optimal_by_open_exhaustion = open_exhausted;
const bool optimal_by_makespan_lb =
    (static_cast<int>(solution.size()) == soc_lower_bound);   // ← makespan(+1) 比 soc 下界
```

`post_processing.cpp`（约第 308–311 行，修改前）：

```cpp
const bool optimal_by_makespan_lb =
    (!solution.empty() &&
     static_cast<int>(solution.size()) ==
         get_sum_of_costs_lower_bound(ins, dist_table));       // ← 同样的量纲/off-by-one 问题
```

---

## 2. 修改内容

将比较改为「makespan 与 makespan 下界」这对同量纲、语义与变量名一致的量。

### planner.cpp

```cpp
int soc_lower_bound = 0;
int makespan_lower_bound = 0;
for (size_t i = 0; i < ins->N; ++i) {
  const int d = static_cast<int>(D.get(i, ins->starts[i]));
  soc_lower_bound += d;
  makespan_lower_bound = std::max(makespan_lower_bound, d);
}
const bool optimal_by_open_exhaustion = open_exhausted;
// makespan = (number of configs) - 1. Certify makespan-optimality only
// when the solution's makespan equals its lower bound.
const int makespan =
    solution.empty() ? 0 : static_cast<int>(solution.size()) - 1;
const bool optimal_by_makespan_lb =
    (!solution.empty() && makespan == makespan_lower_bound);
```

其中 makespan 下界 = 各 agent 单体（忽略其他 agent、含转向代价）最短路距离的最大值，与 `post_processing.cpp::get_makespan_lower_bound()` 定义一致。

### post_processing.cpp

```cpp
const bool optimal_by_makespan_lb =
    (!solution.empty() &&
     get_makespan(solution) == get_makespan_lower_bound(ins, dist_table));
```

复用已有的 `get_makespan()` 与 `get_makespan_lower_bound()`，同时消除 off-by-one。

---

## 3. 语义说明

- 修复后 `optimal_by_makespan_lb` 的含义是：**解的 makespan 已等于其 makespan 下界，故 makespan 不可能更优**——这是针对 **makespan 目标** 的一个有效最优性证书。
- `optimal = optimal_by_open_exhaustion || optimal_by_makespan_lb` 中，`open_exhaustion` 仍是与目标无关的通用证书；`makespan_lb` 现在能作为「快速捷径」正常生效。
- **注意（潜在后续项）**：若某次实验优化的目标不是 makespan（如 sum-of-loss / sum-of-costs），仅凭 makespan 触底并不能证明该目标最优。当前实现未按 objective 区分；如需严格的目标感知判定，可后续将该分支与 `objective` 绑定。通用证书 `open_exhaustion` 不受此限制。

---

## 4. 验证

重新编译后运行：

| 算例 | makespan | makespan_lb | optimal | open_exhaustion | makespan_lb 分支 |
|---|---|---|---|---|---|
| loop.scen, N=3, `-t 60` | 14 | 4 | ✅ true | true | false（14≠4，符合预期） |
| empty-32-32, N=1, `-s 3` | 25 | 25 | ✅ true | true | **true（触底捷径现已生效）** |
| empty-32-32, N=2, `-s 3` | 25 | 25 | ✅ true | true | **true** |
| empty-32-32, N=3, `-s 3` | 25 | 25 | ✅ true | true | **true** |

- 回归：makespan≠lb 时 `optimal_by_makespan_lb` 仍为 false，最优性靠 `open_exhaustion` 正确判定，无回归。
- 修复生效：makespan==lb 时 `optimal_by_makespan_lb` 正确变为 true（修改前恒为 false）。

---

## 5. 对历史实验结果的影响评估

- 所有 makespan / soc / 各下界的**数值本身不受影响**（bug 只在错误的比较处，不改变各量的计算）。
- 若此前某张「不同 agent 数下求得最优的算例数」的图是**读求解器输出的 `optimal=` 字段**统计的：该图的最优计数偏**保守**（真实值 ≥ 图上值），修复后重跑可能略增，趋势方向不变。
- 若该图是**分析脚本中直接用 cost 与下界比较**得到的：不受本 bug 影响。
- 建议：确认画图脚本中「最优」的来源；若来自 `optimal=` 字段，用修复后的二进制重跑一遍以获得更准确（可能略高）的最优计数。
