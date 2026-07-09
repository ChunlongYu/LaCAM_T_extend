# 修改记录：逐拓展的 (优先级继承数, 奖励) 日志

- **日期**：2026-07-09
- **涉及文件**：`main.cpp`、`lacam2/include/lacam2.hpp`、`lacam2/src/lacam2.cpp`、
  `lacam2/include/planner.hpp`、`lacam2/src/planner.cpp`
- **背景**：见 `docs/priority_inheritance_reward_measurement_issue.md`。原有埋点
  （`initial_priority_inheritance_sum`）在时间/粒度上与奖励错位，无法做因果分析。

---

## 1. 做了什么

为研究「高层节点拓展时的优先级继承次数」与「该次拓展获得的奖励」的关系，新增
**逐拓展**日志：每次**阶段 2** 的高层节点拓展记录一行同点同时的
（继承数, 奖励, 上下文）。

新增参数 `--inherit_log <path>`（默认空=关闭，不写盘）。

## 2. 关键改动

1. **打开阶段 2 的继承计数**（`planner.cpp` funcPIBT）：去掉 `!in_optimization_phase`
   这道闸，使优化阶段每次拓展也统计继承数。
   - 安全性核对：`current_priority_inheritance_count` 仅在建节点处
     （`H_goal==nullptr` 时）被读入 `HNode::priority_inheritance_count`，阶段 2 恒取 0；
     其余仅用于计数自身的重置/回滚。故此改动**不改变搜索行为**，也不改变
     `initial_priority_inheritance_sum` 语义。
2. **逐拓展写行**：在阶段 2 三个奖励结算点（fail / known-config / new-config）各写一行。
3. **收尾写 CSV**：仅当 `--inherit_log` 非空时写盘。默认零 I/O。

## 3. CSV 格式

```
loop_cnt, region, inherit, reward, outcome, h, g, f
```
- `inherit`：本次拓展的优先级继承次数（X）。
- `reward`：本次拓展奖励（Y）：fail=0 / known=0.05 / new=0.1 / f_improve=0.5 / better=1.0。
- `outcome`：0=fail,1=known,2=new,3=f_improve,4=better。
- `h,g,f`：被拓展节点的启发式/代价（控制混淆用）。
- 每行 = 一次拓展事件；同一节点可多次出现。

## 4. 用法

```sh
build/main -m assets/random-32-32-20.map -i assets/random-32-32-20-random-1.scen \
           -N 50 -O 2 -t 8 --inherit_log inherit.csv
```

## 5. 验证

random-32-32-20 N=50 `-O 2 -t 8`：产出 21.9 万行，阶段 2 继承数确为非零
（样例 6/9/3/1…），确认闸已打开且计数生效。快速分析（单次运行）：

- `Pearson r(inherit, reward) ≈ 0.037`（弱，单次噪声大）。
- 按 outcome 看平均继承数：known=0.27（低）、fail=1.42、new=1.12 —— 继承越多越
  不易撞到已知配置，符合直觉（推挤越多→构型越新）。
- 默认（不给 `--inherit_log`）：不写盘；打开闸带来的计数开销可忽略，搜索行为不变。

## 6. 分析提示（给学生）

奖励是离散/即时量，别只算裸相关；建议「按 inherit 分桶看平均 reward」「inherit →
各 outcome 概率」，并用 `h/g/f/region` 控制混淆，多 seed/多算例汇总以降噪。
