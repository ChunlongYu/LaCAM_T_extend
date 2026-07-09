# 修改记录：cheap 全局下界 + bounded-gap 早停 + 收敛日志/画图

- **日期**：2026-07-09
- **涉及文件**：
  - `main.cpp`（新增 CLI 参数）
  - `lacam2/include/lacam2.hpp`、`lacam2/src/lacam2.cpp`（`solve()` 接口透传）
  - `lacam2/include/planner.hpp`、`lacam2/src/planner.cpp`（核心实现）
  - `plot_convergence.py`（新增：收敛曲线画图脚本）
- **理论依据**：`docs/LaCAM_star_global_lower_bound_discussion_updated.md` §15–18。

---

## 1. 新增功能概述

1. **cheap 全局下界（LB）**：在搜索过程中维护一个便宜、理论安全的全局下界
   $$LB_t^{cheap}=\min\Big\{g_t(G),\ \min_{X\in\mathcal{F}_t}[g_t(X)+h(X)]\Big\}$$
   其中 $\mathcal{F}_t=\{X:\text{successor 尚未全部生成}\}$（即 `search_tree` 非空的配置）。
2. **bounded-gap 早停**：新增 `-e/--epsilon`。当 $(UB-LB)/UB\le\epsilon$ 时带证书提前停止，返回一个满足 $UB\le(1+\epsilon)\,OPT$ 的解。`epsilon=0`（默认）时退化为「跑到精确最优」，行为与原来一致。
3. **收敛日志**：新增 `--conv_log <path>`。把 `(time_ms, ub, lb, gap)` 随搜索过程写成 CSV，供画图。
4. **画图脚本**：`plot_convergence.py` 读取该 CSV，画 UB/LB 随时间收敛曲线 + gap(%) 面板。

---

## 2. 实现要点与正确性论证

实现前对照 `planner.cpp` 核对了 cheap LB 成立所需的两个前提，均满足：

### 前提 (a)：$g_t$ 为已知图精确最短距离 —— 满足
- 新节点建立时 `g = parent->g + edge_cost`，且该配置此前不在 `EXPLORED`，唯一已知入边来自父节点 ⟹ 建立即精确。
- 命中已知配置时调用 `rewrite()`，它是 label-correcting（Bellman-Ford 式）传播，正边权、每次调用内收敛到精确最短距离。
- 每条生成的边都记入 `neighbor` 图（构造函数 + `rewrite`），无漏边。
⟹ 任意时刻取 LB 都安全，不会因 stale/偏高的 $g_t$ 而失效。

### 前提 (b)：$\mathcal{F}_t\neq$ OPEN —— 已正确规避
- `planner.cpp` 中 `if (H_goal != nullptr && H->f >= H_goal->f) continue;` 的剪枝会把「未完成但 $f\ge UB$」的节点移出 OPEN。
- **但**这些节点 $f\ge UB$，不会把 $\min$ 压到 $UB$ 以下；且 `rewrite()` 会在某节点 $f$ 降到 $UB$ 以下时把它重新 `push_back` 进 OPEN。
- ⟹ 数值上 $\min_{X\in OPEN,\ \text{未完成}} f \;=\; \min_{X\in\mathcal{F}_t} f$。因此实现里**只扫描 OPEN 中 `search_tree` 非空的节点**即可，无需遍历全部已发现节点，开销为 $O(|OPEN|)$。

> 注：代码里对每个采样点都显式跳过 `search_tree` 为空的节点（已完成配置不属于 $\mathcal{F}_t$），保证下界有效。

### 有效下界取历史最大
每次采样得到的 cheap LB 都 $\le OPT$，故对其取历史最大值 `best_lb = max(best_lb, cur)` 仍 $\le OPT$，且**单调不减**，得到更紧、更平滑的下界曲线用于 gap 判定与画图。

### admissibility
`get_h_value` 已是目标感知的（makespan 取 $\max_i D.get$、sum-of-loss 取 $\sum_i D.get$），`D` 为精确单体（忽略他人、含转向）距离，是 admissible 下界，且与 `g` 的代价模型一致 ⟹ 下界有效。

---

## 3. 采样策略与开销

- 每 `CONV_SAMPLE_INTERVAL = 64` 次主循环迭代采样一次；额外在**首个可行解出现时**和**搜索结束时**各采样一次，确保关键事件被记录。
- 早停判定只在采样点进行；`epsilon>0` 且 `gap<=epsilon` 时置 `stopped_by_gap=true` 并 `break`。
- OPEN 为空（穷尽）时 `LB_cheap` 自然等于 `UB`，gap→0，与 `optimal_by_open_exhaustion` 一致。

---

## 4. 新增命令行参数

| 参数 | 含义 | 默认 |
|---|---|---|
| `-e, --epsilon <f>` | bounded-gap 早停容差，`(UB-LB)/UB<=eps` 即停；`0`=跑到精确最优 | `0.0` |
| `--conv_log <path>` | 写 UB/LB 收敛日志（CSV）；空=关闭 | `""` |

### 输出日志（`-o` 指定的 .txt）新增字段
`epsilon`、`stopped_by_gap`、`optimal_by_bound`、`cheap_lb`、`cheap_ub`、`final_gap`。
（`stopped_by_gap=true` 表示带证书的**次优**解，不等于精确最优，除非 `final_gap=0`。）

### 按界证明最优（optimal_by_bound）—— 2026-07-09 追加
当某个采样点检测到 `best_lb >= UB`（即 gap=0）时，直接停止并置
`optimal_by_bound=true`，**与 `epsilon` 无关**。理由：

$$LB=UB \implies UB\le OPT\ \wedge\ UB\ge OPT \implies UB=OPT$$

这是分支定界式的「按界证明最优」——**不需要把 OPEN 搜穷**，只要证明剩余
未完成节点全部满足 $f\ge UB$（不可能更优）即可。它是比 OPEN 穷尽更早可用的
最优性证书：

- `optimal = optimal_by_open_exhaustion || optimal_by_makespan_lb || optimal_by_bound`。
- 关系：`OPEN 空 ⟹ gap=0`，但 `gap=0 ⇏ OPEN 空`。故 `optimal_by_bound` 可在
  OPEN 仍非空时触发。
- 验证：loop N=3（`-O 1`）此前报 `optimal_by_open_exhaustion=true`；加入后
  多数 seed 改报 `optimal_by_bound=true`（OPEN 非空即证明最优）。当 64 次
  迭代的采样粒度恰好错过 gap 闭合瞬间时，OPEN 穷尽仍会正确兜底终止。
- 注意：本 loop 实例 cheap LB 偏松（长期停在 8），LB 只在末尾才追上 UB，故
  `optimal_by_bound` 与穷尽在时间上几乎重合；真正的加速要靠更紧的下界
  （§21 selective tightening）。此改动主要保证「一旦能按界证明就立刻停」，
  且默认（`epsilon=0`）也生效。

### 收敛日志格式（`--conv_log` 的 CSV）
```
time_ms,ub,lb,gap
9,,1715,            <- 首个可行解之前：ub/gap 为空
9,2548,1715,0.326923
...
```

---

## 5. 用法示例

求解并记录收敛日志：
```sh
build/main -m assets/loop.map -i assets/loop.scen -N 3 -O 1 -t 60 \
           --conv_log conv_loop.csv -o out.txt
```

带 5% gap 早停：
```sh
build/main -m assets/random-32-32-20.map -i assets/random-32-32-20-random-1.scen \
           -N 60 -O 2 -t 30 -e 0.05 --conv_log conv.csv
```

画收敛图（单个 / 多个叠加对比）：
```sh
python plot_convergence.py conv_loop.csv -o conv_loop.png
python plot_convergence.py runA.csv runB.csv -o compare.png
```

---

## 6. 验证

重新编译后：

| 场景 | 观察 | 结论 |
|---|---|---|
| loop N=3, `-O 1 -t 60` | UB 29→14，cheap LB 4→8 长期偏松，末尾 OPEN 穷尽时 LB 跳到 14，gap→0，`optimal=1` | 收敛正确；印证 §20「cheap LB 可能长期很松」 |
| random-32-32-20 N=60, `-O 2 -t 10` | UB 2548→2544，LB 停在 1715，`final_gap≈0.326` | 松下界如预期；日志/gap 计算正确 |
| 同上 `-e 0.35` | `stopped_by_gap=true`，`final_gap≈0.327`，提前返回 | 早停判定正确 |

收敛曲线图（`plot_convergence.py` 生成）清晰展示 UB 单调下降、LB 单调上升、gap 面板收敛到 0。

---

## 6b. 采样开销与总开关（2026-07-09 追加）

UB/LB 采样(每 `CONV_SAMPLE_INTERVAL=64` 次迭代扫一遍 OPEN 算 cheap LB)本身有
少量开销。实测 random-32-32-20 N=70 跑满时限:采样开启比关闭少约 **2.8%** 迭代。

因此加了总开关,**默认零开销**:

- 采样只在 `track_bounds` 为真时才跑。`track_bounds = 显式 --track_bounds ||
  设了 --conv_log || --epsilon>0 || --pair_lb_ms>0`。
- 默认(什么都不加):**不采样、不扫 OPEN、不累积历史**,行为与加入 LB 功能之前
  完全一致(精确最优仍由 OPEN 穷尽终止)。
- 写盘只在 `--conv_log` 非空时发生;历史向量也只在此时累积(避免无谓内存)。
- 代价:关闭采样时 `optimal_by_bound` / `stopped_by_gap` 不会触发(需要 LB)。
  一旦用 `--conv_log` / `--epsilon>0` / `--pair_lb_ms>0` / `--track_bounds`
  任一开启,它们即恢复工作。

新增参数 `--track_bounds`(implicit true,默认 false):在不写日志、不设 epsilon
的情况下也强制开启 LB 追踪(从而启用 optimal-by-bound 早停证书)。

## 7. 已知局限（与理论一致）

- cheap LB 常被起点 frontier 卡住（$LB\le h(S)$），gap 可能长期偏大——见 §20。若需要更紧的 gap，应按 §21 对「当前决定 LB 的少数 frontier」做 selective tightening（corridor / pairwise / flow 等），本次未实现。
- 早停只在采样点触发，故实际停止可能比精确达到 gap 的时刻晚最多 `CONV_SAMPLE_INTERVAL` 次迭代——方向保守、安全。
