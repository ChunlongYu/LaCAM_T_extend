# 修改记录：pairwise makespan 静态根下界

- **日期**：2026-07-09
- **涉及文件**：`main.cpp`、`lacam2/include/lacam2.hpp`、`lacam2/src/lacam2.cpp`、
  `lacam2/include/planner.hpp`、`lacam2/src/planner.cpp`
- **理论依据**：`docs/LaCAM_star_global_lower_bound_discussion_updated.md` §10.2、§21。
- **前置结论**：动作模型保持不变（不加「保持朝向后退」动作），故 `g`、
  报告 `makespan`、`solution.size()-1` 三者同单位，pairwise 直接按统一单位算。

---

## 1. 做了什么

在 **makespan 目标** 下，开局一次性计算一个比 `max_i dist(s_i,g_i)` 更紧的
**静态全局下界**，抬高 `best_lb` 的地板：

$$LB^{pair}=\max\Big(\max_i d_i,\ \max_{(i,j)\in\text{候选}} M_2(i,j)\Big)$$

- $d_i = D.get(i, s_i)$：单体（含朝向、忽略他人）最短距离。
- $M_2(i,j)$：只考虑 i、j 两个 agent（忽略其他）的**最优 makespan**。

**可采纳性**：把完整解投影到 (i,j) 得到一个合法的 2-agent 解，其 makespan
$\le$ 完整 makespan，故 $M_2(i,j)\le OPT$；对所有对取 max 仍 $\le OPT$。

只在搜索**开始前**算一次（$\Delta$ 与搜索状态无关），作为独立全局界与
`best_lb` 取 max，**不改 h、不进热循环**。

## 2. $M_2$ 的求法：2-agent 联合空间精确 A\*

- **状态**：$((v_i,o_i),(v_j,o_j))$，含朝向；编码为 `id*4+o` 再拼成 64 位 key。
- **每 agent 单步动作**（与主搜索动作模型一致）：等待 / 转 ±90° / 沿朝向前进一格，
  各 1 步。联合转移 = 两者笛卡尔积，代价 1（一个时间步）。
- **冲突**：剔除顶点冲突（同格）与交换冲突（互换位置）。
- **启发式**：$h=\max(D.get(i,\cdot),\,D.get(j,\cdot))$，用现成距离表，可采纳且一致。
- **目标**：两者同时到达各自目标顶点（朝向任意，与 `D` 约定一致）。

## 3. 成本控制

- **包围盒重叠剪枝**：两 agent 的 start/goal 包围盒（加 margin=1）不相交 ⇒
  基本无交互 ⇒ $M_2\approx\max(d_i,d_j)\le$ 基线，直接跳过，不搜。
- **每对扩展上限** `PER_PAIR_EXP_CAP = 200000`；超限放弃该对（其贡献退化为基线，安全）。
- **总时间预算** `--pair_lb_ms`；超预算或触及主 deadline 即停。**只算部分对仍是
  合法（更松）下界**，因为 max 的项少了只会更小。

## 4. 新增参数与输出

| 参数 | 含义 | 默认 |
|---|---|---|
| `--pair_lb_ms <ms>` | pairwise 根下界的时间预算；`0`=关闭；仅 `--objective 1` 生效 | `0` |

输出日志新增字段 `lb_pair`（关闭或非 makespan 目标时为 0）。默认关闭，
**不改变原有行为**。

用法：
```sh
build/main -m assets/loop.map -i assets/loop.scen -N 3 -O 1 -t 30 --pair_lb_ms 2000
```

## 5. 验证

`baseline = max_i d_i`（原 makespan_lb），要求 `baseline <= lb_pair <= makespan`
（下界不得超过任何可行解）：

| 算例 | baseline | lb_pair | makespan | 结论 |
|---|---|---|---|---|
| loop N=3 | 4 | **14** | 14 | 大幅收紧到最优；界有效 |
| random-32-32-20 N=40 | 56 | 56 | 56 | 无交互增益；界有效 |
| warehouse N=40 | 190 | 190 | 190 | 界有效 |
| room-64-64-8 N=30 | 125 | 125 | 126 | 界有效（`<= makespan`） |

- 全部满足 `baseline <= lb_pair <= makespan`，**下界从未超过最优**。
- **loop** 是典型收益场景：cheap LB 长期卡在 8，而 pairwise 直接给出 14（=最优
  makespan）。收敛图（`plot_convergence.py` 叠加 with/without）显示：开启 pairwise 后
  LB 从 t=0 起就是 14，gap 全程显著更小——配合 bounded-gap 早停可提前约一个数量级停止。
- 默认 `--pair_lb_ms 0`：`lb_pair=0`，行为与之前完全一致。

## 6. 局限与后续

- **静态根界**：不随搜索收紧。对「UB 下降是瓶颈」的实例（如 loop 精确最优）不会
  加速——那里的耗时花在把 UB 压到最优，与随机性主导；pairwise 的价值在于**立刻给出
  紧 LB**，从而利好 bounded-gap 早停与 anytime 证书。
- **无交互实例**（random/warehouse 上述）pairwise 无增益，属正常：这些实例
  makespan 下界本已被 $\max_i d_i$ 打紧。
- **后续**（§21）：动态 per-frontier selective tightening、corridor/cut、EWMVC
  组合等，可让下界随搜索进一步收紧。本次仅实现静态根界。
