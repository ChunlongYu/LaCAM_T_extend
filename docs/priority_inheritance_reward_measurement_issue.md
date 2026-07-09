# 关于「优先级继承计数 vs 高层节点奖励」统计的问题说明

## 0. 结论先行

统计优先级继承次数的代码**已经在仓库里**,并且会连同各 UCB arm 的奖励一起写进日志。
但当前的埋点方式**无法回答「继承次数是否影响高层节点拓展奖励」这个问题**,原因是
两者在**时间窗**和**统计粒度**上都错位了。要得到能做因果分析的数据,需要按下面第 5 节改。

---

## 1. 想研究的问题

在一个高层节点内,底层 PIBT 会发生若干次「优先级继承」(高优先级 agent 递归推挤
低优先级 agent 让路)。想探究:**一个高层节点生成/拓展时发生的继承次数,和它被拓展时
获得的奖励分数,有没有关系。**

---

## 2. 现有代码在哪里数、怎么存、怎么输出

### 2.1 计数(`funcPIBT`,`planner.cpp:1596`)
```cpp
if (!is_initial && !in_optimization_phase && !recursively_called_agents[ai->id]) {
    recursively_called_agents[ai->id] = true;
    recursive_call_history.push_back(ai->id);
    ++current_priority_inheritance_count;
}
```
- 每递归推挤一个「首次被推」的 agent，`current_priority_inheritance_count` +1。
- `recursively_called_agents` 保证同一 agent 只计一次；回滚时（`planner.cpp:1784-1785`）会减回。
- `current_priority_inheritance_count` 在每次生成配置开始时清零（`planner.cpp:1357`），
  因此它表示「生成这一个配置时的推挤数」。

### 2.2 存进高层节点（建节点，`planner.cpp:814`）
```cpp
const int priority_inheritance_count =
    (H_goal == nullptr) ? current_priority_inheritance_count : 0;
auto H_new = new HNode(..., priority_inheritance_count);
```
存进 `HNode::priority_inheritance_count`（`planner.hpp:74`）。

### 2.3 聚合成区域和（首解时刻，`planner.cpp:739-743`）
```cpp
int sum_priority_inheritance = 0;
for (idx in 区域 i) sum_priority_inheritance += OPEN[idx]->priority_inheritance_count;
phases[i].initial_priority_inheritance_sum = sum_priority_inheritance;
```

### 2.4 输出
`initial_priority_inheritance_sum` 与各 arm 的奖励统计一起写进 `mab_arm_stats`
日志（`planner.cpp:895/912`、`post_processing.cpp:257/278`），落在 `-o` 输出旁边的
配套 `.csv`。每个 arm 一行，列含
`num_selected, total_reward, reward_better_solution, reward_f_improve,
reward_new_config, reward_known_config, reward_fail_new_config,
initial_priority_inheritance_sum`。

---

## 3. 两个阶段与两道闸

**两个阶段：**
- **阶段 1（找首解）**：`H_goal == nullptr`，纯 DFS 下潜直到第一个可行解出现；
  之后立即 `in_optimization_phase = true`（`planner.cpp:655`）切入阶段 2。
- **阶段 2（优化）**：`H_goal != nullptr`，UCB 反复选区、扩展、改进解。
  **奖励全部在这个阶段累积。**

**两道闸让继承计数只发生在阶段 1：**
- **闸 1**（`planner.cpp:1596` 的 `!in_optimization_phase`）：进入阶段 2 后此条件恒假，
  `current_priority_inheritance_count` **在阶段 2 完全不再增加**。
- **闸 2**（`planner.cpp:814` 的 `H_goal == nullptr ? ... : 0`）：已有解后新建的节点，
  继承数**硬编码为 0**。

⇒ **只有阶段 1 造出的节点带非零继承数；阶段 2 出生的节点继承数一律为 0。**

---

## 4. 为什么现在的配对无法回答问题（时间 + 粒度错位）

时间线：
```
t=0…T0   阶段1(DFS):造节点 A,B,C…, 各记录生成时的推挤数(如 A=3,B=0,C=5)
t=T0     首解出现!把 OPEN 切 6 区, 对每区求和继承数并【冻结】进
         initial_priority_inheritance_sum(如 区域0={A,C} → 8)
t=T0…end 阶段2(UCB优化):生成大量新节点(继承数全=0)塞进各区,
         同时给区域累积奖励(如 区域0 → total_reward=12.5)
end      日志写: 区域0: inheritance_sum=8, total_reward=12.5
```

于是每个区域配对的 `(inheritance_sum, total_reward)`：
- **X（继承和）**：阶段 1 里、首解那一刻恰好落在该区域的那几个节点的推挤数之和。
- **Y（奖励）**：整个阶段 2 该区域挣到的奖励——而这段时间真正被拓展、真正产生奖励的，
  大多是阶段 2 新生的、继承数=0 的节点。

问题：
1. **时间窗错位**：X 描述初始 DFS 的结构，Y 描述优化阶段的表现，测的是两段不同时间。
2. **节点集错位**：随着阶段 2 不断塞入继承数=0 的新节点，冻结的 `sum` 越来越不能代表
   区域当前实际内容。
3. **粒度太粗**：只有 6 个区域 ⇒ 每个实例仅 6 个数据点，且是区域级聚合，不是节点级。
4. **奖励公式本身不含继承数**：奖励只由结果决定（更优解 +1.0、`f` 改善 +0.5、
   撞已知配置 +0.05、失败 0），继承计数**不进入奖励或 UCB**。当前埋点纯属观测，无因果耦合。

结论：即便画出散点看到相关，也无法解释成「继承多 → 奖励高」，因为 X 量的根本不是
「正在被拓展、正在拿奖励的那些节点的继承程度」。

---

## 5. 要做成能回答问题的形式（建议改法）

目标：在**阶段 2**、以**高层节点**为单位，记录同一节点上同一时间的两个量。

1. **让优化阶段也统计继承数**：放开闸 1、闸 2 的限制（或新增一个不受
   `in_optimization_phase` / `H_goal` 约束的计数器），使阶段 2 每次生成/扩展节点时
   也记录其真实继承数，存进该 `HNode`。
2. **把奖励归因到节点**：当前奖励累加到区域（arm）。改成在每次拓展某个具体节点、
   算出这次奖励时，记录一条 `(该节点的继承数, 这次奖励)`。
3. **逐节点输出 CSV**：dump 两列（必要时附节点 id、`g`、`h`、所在区域、时间戳），
   直接用于散点图 / 回归。

这样 X、Y 才是同一时间、同一节点上的量，相关/回归才有因果解释力。

---

## 5b. 已实现：`--inherit_log`（2026-07-09）

上面第 5 节的改法已经落地，可直接用。

**做了什么**
- 打开了阶段 2 的继承计数（去掉 `funcPIBT` 里 `!in_optimization_phase` 这道闸）。
  已核对：`current_priority_inheritance_count` 只在建节点处被读入 node 字段，且阶段 2
  恒取 0，故此改动**不影响搜索行为**，只是让该计数在阶段 2 也有效。
- 每次**阶段 2 的高层节点拓展**都记录一行：当次继承数 + 当次奖励 + 上下文列。
- 用 `--inherit_log <path>` 开启（默认关、不写盘）。

**命令**
```sh
build/main -m assets/random-32-32-20.map -i assets/random-32-32-20-random-1.scen \
           -N 50 -O 2 -t 8 --inherit_log inherit.csv
```

**CSV 列**
```
loop_cnt, region, inherit, reward, outcome, h, g, f
```
- `loop_cnt`：迭代序号（时间轴）。
- `region`：该节点所在的 UCB 区域（arm）。
- `inherit`：**这次拓展**发生的优先级继承次数（= 同点同时的 X）。
- `reward`：**这次拓展**获得的奖励（同点同时的 Y）：fail=0 / known=0.05 / new=0.1 /
  f_improve=0.5 / better=1.0。
- `outcome`：0=fail、1=known、2=new、3=f_improve、4=better（便于分层）。
- `h, g, f`：被拓展节点的启发式/代价（用于控制混淆变量）。

每行是**一次拓展事件**（同一节点可多次出现）。这就是第 5 节要的「同点同时」数据。

**分析建议**
- 奖励基本是离散/即时的（取值 {0,0.05,0.1,0.5,1.0}），别只算裸皮尔逊相关；
  更适合「按 `inherit` 分桶看平均 `reward`」或「`inherit` → 各 `outcome` 概率」。
- 用 `h/g/f/region` 做分层或回归控制，排除拥堵/深度等混淆。
- 单次运行噪声大，建议多 seed / 多算例汇总。

## 6. 一句话给学生

现有代码记录的是「**阶段 1 的继承和（区域级、首解时冻结）**」对「**阶段 2 的区域奖励**」，
时间窗、节点集、粒度都对不上，且继承数并不进入奖励公式；要研究「继承 → 奖励」，
需改成**阶段 2 逐高层节点**同时记录（继承数, 该次拓展奖励）再输出。
