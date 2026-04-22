# LaCAM_T_extend 项目文档

## 项目概述

本项目是论文 [*Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding*](https://kei18.github.io/lacam2/)（IJCAI-23）所提出的 **LaCAM2** 算法的扩展实现。在原版 LaCAM2 的基础上，本项目引入了**方向约束（Orientation Constraint）**，将智能体从无朝向的点扩展为有朝向的实体，从而更贴近真实机器人的物理运动模型（例如差速驱动机器人）。

### 核心扩展内容

| 方面 | 原 LaCAM2 | 本扩展版 |
|------|-----------|----------|
| 智能体状态 | 仅位置 `Vertex*` | 位置 + 朝向 `State{Vertex*, Orientation}` |
| 动作空间 | 移动到邻居 / 原地等待 | 向前移动 / 顺时针转 / 逆时针转 / 原地等待 |
| 距离表 | 按顶点 BFS | 按（顶点, 朝向）二元组 BFS |
| 目标检测 | `is_same_config` | `is_same_config_pos`（只检查位置） |
| 高层搜索 | DFS（栈） | 找解阶段 DFS + 优化阶段 UCB 三区域采样（双端队列） |

---

## 目录结构

```
LaCAM_T_extend/
├── main.cpp                    # 程序入口
├── CMakeLists.txt
├── lacam2/
│   ├── include/
│   │   ├── orientation.hpp     # 朝向枚举定义
│   │   ├── graph.hpp           # 图结构、State、Config
│   │   ├── instance.hpp        # 问题实例
│   │   ├── dist_table.hpp      # 启发式距离表
│   │   ├── planner.hpp         # 规划器（Agent、LNode、HNode、Planner）
│   │   ├── post_processing.hpp # 后处理与结果验证
│   │   ├── lacam2.hpp          # 对外接口 solve()
│   │   └── utils.hpp           # 工具函数（计时、随机数等）
│   └── src/
│       ├── graph.cpp
│       ├── instance.cpp
│       ├── dist_table.cpp
│       ├── planner.cpp         # 核心算法实现（最重要）
│       ├── lacam2.cpp
│       ├── post_processing.cpp
│       └── utils.cpp
├── tests/                      # 单元测试
├── assets/                     # 地图与场景文件
└── third_party/                # argparse、googletest
```

---

## 核心数据结构

### `Orientation`（`orientation.hpp`）

```
X_PLUS  = 0  // 向东（x+1）
Y_PLUS  = 1  // 向南（y+1）
X_MINUS = 2  // 向西（x-1）
Y_MINUS = 3  // 向北（y-1）
```

智能体在任意时刻都拥有一个朝向。只有面朝目标方向时才能向前移动；转向需要额外时间步。

---

### `Vertex`（`graph.hpp`）

```cpp
struct Vertex {
    uint id;           // 在图中的紧凑编号（从0开始连续）
    uint index;        // 在地图网格中的位置编号（width * y + x）
    vector<Vertex*> neighbor;
};
```

- `V`：仅包含可通行顶点的紧凑列表。
- `U`：包含 `nullptr` 的全网格列表，按 `index` 随机访问。

---

### `State`（`graph.hpp`）

```cpp
struct State {
    Vertex* v;    // 位置
    Orientation o; // 朝向
};
using Config = vector<State>;   // 所有智能体在某时刻的状态集合
using Solution = vector<Config>; // 时间序列上的配置列表
```

`Config` 是算法状态空间的核心单元。一个 `Solution` 就是从初始配置到目标配置的配置序列。

---

### `Graph`（`graph.hpp` / `graph.cpp`）

从 `.map` 文件（MovingAI 格式）解析构建网格图。`T` 或 `@` 字符表示障碍物。

---

### `Instance`（`instance.hpp` / `instance.cpp`）

封装一个 MAPF 问题实例：图 `G`、起始配置 `starts`、目标配置 `goals`、智能体数量 `N`。

提供三种构造方式：
1. 从索引列表直接构造（用于测试）
2. 从 `.scen` 场景文件构造（MAPF benchmark）
3. 随机生成（用于性能测试）

所有智能体的初始朝向统一设为 `Y_MINUS`（向北）。

---

### `DistTable`（`dist_table.hpp` / `dist_table.cpp`）

带朝向的启发式距离表。索引为 `v->id * 4 + static_cast<uint>(o)`，即每个顶点存储 4 个方向的距离。

使用**逆向 BFS（Reverse BFS）**从每个智能体的目标位置出发，向前反推：
- **逆向转向**：若 `(u, dir_u)` 可以通过旋转到达 `(u, dir_u)`，则前驱方向距离 = 当前 + 1。
- **逆向移动**：若从 `v` 朝 `dir_u` 方向向前移动一步可到 `u`，则 `(v, dir_u)` 距离 = 当前 + 1。

目标位置的**所有 4 个朝向**距离都初始化为 0（到达目标位置即完成，无需特定方向）。

---

### `Agent`（`planner.hpp`）

```cpp
struct Agent {
    uint id;
    Vertex* v_now;    Orientation o_now;  // 当前状态
    Vertex* v_next;   Orientation o_next; // 下一步状态
    bool swap_completed;                  // Swap 操作是否完成
};
```

---

### `LNode`（低层节点，`planner.hpp`）

低层约束树的节点，记录对部分智能体的强制约束：
- `who`：被约束的智能体 ID 序列。
- `where`：对应的目标 `State` 序列。
- `depth`：已约束的智能体数量。

---

### `HNode`（高层节点，`planner.hpp`）

高层搜索树的节点，代表搜索空间中的一个配置：
- `C`：当前配置（`Config`）。
- `g`、`h`、`f`：路径代价、启发值、总估值。
- `parent`：父节点（用于回溯路径）。
- `neighbor`：邻居高层节点集合（用于 `rewrite` 时的 Dijkstra 传播）。
- `priorities`：各智能体的优先级（动态更新，优先处理未到达目标的智能体）。
- `order`：按优先级排序的智能体顺序。
- `search_tree`：低层约束树的待展开队列。

---

### `Planner`（`planner.hpp` / `planner.cpp`）

主规划器，持有所有必要状态：
- `D`：`DistTable` 实例。
- `A`：`Agents` 列表。
- `occupied_now` / `occupied_next`：按顶点 ID 索引的占用快速查询表。
- `reserved_nodes`：各智能体的保留节点（转向等待期间使用）。
- `push_count_table`：`N×N` 的推挤计数矩阵（用于 k-Push Escape Trigger）。
- `validation_table` / `modified_indices`：用于新配置合法性检查的临时表与回滚列表。
- `request_chain`：当前 PIBT 递归的请求链（用于环形冲突检测）。

---

## 算法流程

### 整体流程图

```
main()
  └─ solve()                          [lacam2.cpp]
       └─ Planner::solve()            [planner.cpp]
            ├─ 初始化 HNode、OPEN
            ├─ 主循环（DFS / UCB）
            │    ├─ expand_lowlevel_tree()   ← 展开低层约束树
            │    ├─ get_new_config()         ← 生成新配置
            │    │    └─ funcPIBT()          ← PIBT 低层规划
            │    │         ├─ computeAction()
            │    │         ├─ swap_possible_and_required()
            │    │         │    ├─ is_swap_required()
            │    │         │    └─ is_swap_possible()
            │    │         ├─ handleCycleWithOrientation()
            │    │         ├─ PushEscapeTrigger()
            │    │         └─ （递归）funcPIBT()
            │    └─ rewrite()                ← 更新已探索节点的代价
            └─ 回溯路径，返回 Solution
```

---

### 第一阶段：找初始解（纯 DFS）

搜索使用 `std::deque<HNode*>` 作为 OPEN 表。**找到初始解之前**，从队列**尾部**弹出节点（DFS 语义），以快速深入找到可行解。

```
while (!OPEN.empty() && !is_expired(deadline)):
    H = OPEN.back(); OPEN.pop_back()   // DFS
    expand_lowlevel_tree(H, L)
    get_new_config(H, L)
    → 生成 C_new，若已探索则 rewrite，否则创建 HNode 并 push_back
    若 is_same_config_pos(C_new, goals) → 找到初始解，切换阶段
```

---

### 第二阶段：优化阶段（UCB 三区域采样）

找到初始解后，将 OPEN 三等分为三个"区域"（Phase 0/1/2）：

```
OPEN: [ Phase 0 | Phase 1 | Phase 2 ]
       0       b[0]      b[1]       end
```

每轮使用 **UCB（Upper Confidence Bound）** 公式选择最有潜力的区域，再在该区域内**随机**选取节点：

```
UCB(i) = avg_reward(i) + C * sqrt(ln(total_sel + 1) / sel_count(i))
```

奖励规则：
- 生成新节点且找到更优解：`reward = 1.0`
- 生成新节点且 f 值下降：`reward = 0.5`
- 生成新节点（普通）：`reward = 0.1`
- rewrite 命中旧节点：`reward = 0.05`
- 生成配置失败：`reward = 0`

---

### `expand_lowlevel_tree`（低层约束树展开）

对第 `L->depth` 个待约束智能体（按优先级顺序），枚举其所有合法后继 `State`，并作为新的 `LNode` 加入队列：

- **等待**：保持当前位置与朝向不变。
- **右转**：位置不变，朝向顺时针转 90°。
- **左转**：位置不变，朝向逆时针转 90°。
- **向前移动**：只有正前方邻居（与当前朝向一致）才可移动，朝向保持不变。

随机打乱候选顺序后入队，引入随机性。

---

### `get_new_config`（生成新配置）

1. 将所有智能体的 `v_now` 注册到 `occupied_now`，清空 `v_next`。
2. 将低层节点 `L` 中记录的约束（已确定的部分智能体的下一步 `State`）预分配到 `occupied_next`，同时做**顶点冲突**和**交换冲突**的提前过滤。
3. 对剩余未分配智能体，按优先级顺序调用 `funcPIBT`。
4. 全部分配完成后，进行**完整性验证**（顶点冲突、交换冲突、物理合法性），失败则回滚。

---

### `funcPIBT`（PIBT 低层规划，核心递归函数）

每次为单个智能体 `ai` 分配下一步状态。

```
funcPIBT(ai, pusher=null, is_initial=true):
1. 生成候选节点 P = ai 的所有邻居 + ai 当前位置（等待）
2. 按"转向代价 + 到目标距离"排序（贪心）
3. k-Push Escape Trigger：若被同一 pusher 推超过 k 次，随机打乱 P
4. 检测是否需要 Swap（is_swap_required + is_swap_possible）
5. 若有 reserved_node，将其提到 P 最前面
6. 遍历候选节点 u:
   a. 若 u 已被占用（occupied_next[u]≠null）或 u 是 pusher 当前位置 → 跳过
   b. 预定 u（occupied_next[u] = ai）
   c. 检测环形冲突（u == initial_requester->v_now）→ handleCycleWithOrientation
   d. 若 u 被 ak 占用且 ak 未分配 → 递归 funcPIBT(ak, ai)
   e. 调用 computeAction 计算实际动作（可能是转向步而非移动步）
   f. 若 computeAction 返回原地（需要转向）：记录 reserved_node，等待下轮移动
   g. 若可以前进：更新 v_next、o_next，返回 true
   h. 若 swap 对象也需要处理：计算 swap_agent 的动作
7. 若所有候选均失败：原地等待，返回 false
```

---

### `computeAction`（计算物理动作）

给定当前位置、目标位置、当前朝向，返回下一步实际执行的 `(位置, 朝向)`：

| 角度差 | 动作 |
|--------|------|
| 0° | 向前移动到目标节点，朝向不变 |
| 90° | 原地转向至目标方向 |
| 180° | 原地逆时针转 90°（不能一步掉头） |

---

### `rewrite`（代价传播）

当发现已探索节点 `T` 可以通过当前节点 `H_from` 以更低代价到达时，用 Dijkstra BFS 向邻居传播更低 g 值，并更新父节点指针（仅当物理合法时）。被更新且 f < H_goal->f 的节点重新加入 OPEN（Phase 2 区域）。

---

### Swap 机制

为解决两个智能体需要互换位置（死锁）的问题：

- **`is_swap_required`**：模拟 pusher 沿 puller 的路径前进，若路径是死路（度为 1）或被阻塞，则需要交换。
- **`is_swap_possible`**：从 puller 出发模拟前进，若存在分叉路（度 ≥ 2），则可以交换。
- **`swap_possible_and_required`**：综合判断后返回需要交换的 `swap_agent`。若触发交换，反转候选列表 P 并为 swap_agent 直接分配动作。

---

### k-Push Escape Trigger

记录每对 `(pushed_agent, pusher)` 的推挤次数。当同一 pusher 连续推挤同一智能体 ≥ 2 次时（可配置），随机打乱该智能体的候选节点列表，强制其探索其他路径，避免局部循环。

---

### 环形冲突处理（`handleCycleWithOrientation`）

当 PIBT 递归链形成环（某智能体想占用 `initial_requester` 的当前位置）时：
- 若环中有智能体尚未对准目标方向 → 所有人原地等待，未对准的人转向。
- 若所有人已对准 → 所有人同时向前移动，完成旋转。

---

## 主要函数调用关系总结

```
solve() [lacam2.cpp]
  └── Planner::solve() [planner.cpp]
        ├── HNode() 构造                  → DistTable::get() 计算初始优先级
        ├── get_h_value()                 → DistTable::get()
        ├── expand_lowlevel_tree()        （生成低层候选动作）
        ├── get_new_config()
        │    ├── funcPIBT()
        │    │    ├── DistTable::get()   （启发距离查询）
        │    │    ├── computeAction()
        │    │    ├── swap_possible_and_required()
        │    │    │    ├── is_swap_required()  → getMinDistAllDirections() → DistTable::get()
        │    │    │    └── is_swap_possible()
        │    │    ├── PushEscapeTrigger()  → updatePushCount() / getPushCount()
        │    │    ├── handleCycleWithOrientation() → get_direction()
        │    │    └── funcPIBT()（递归）
        │    └── is_valid_transition()    （物理合法性验证）
        ├── rewrite()                     → get_edge_cost() → get_direction()
        └── is_same_config_pos()

main() [main.cpp]
  ├── Instance() 构造                    → Graph() 构造（解析 .map 文件）
  ├── solve()
  ├── is_feasible_solution()             → is_same_config() / is_same_config_pos()
  ├── print_stats()                      → DistTable() / get_makespan() / get_sum_of_costs() / get_sum_of_loss()
  └── make_log()                         → DistTable()
```

---

## 目标函数

通过 `-O` 参数指定：

| 参数值 | 枚举 | 含义 |
|--------|------|------|
| 0 | `OBJ_NONE` | 找到可行解即停止 |
| 1 | `OBJ_MAKESPAN` | 最小化最大完成时间步 |
| 2 | `OBJ_SUM_OF_LOSS` | 最小化所有智能体的额外等待代价之和 |

---

## 合法性约束

一个合法的 `Solution` 必须满足：
1. **起点一致**：`solution[0]` 与 `ins.starts` 完全相同（含朝向）。
2. **终点一致**：`solution.back()` 所有智能体位置（不含朝向）与 `ins.goals` 相同。
3. **连通性**：相邻时间步的位置变化必须在图的边上。
4. **物理合法性**：
   - 移动时朝向不能改变（移动方向必须与当前朝向一致）。
   - 不能在移动的同时转向。
5. **顶点冲突**：不同智能体在同一时间步不能占用同一位置。
6. **交换冲突**：不允许两智能体在相邻时间步互换位置。

---

## 构建与运行

```sh
# 构建
cmake -B build && make -C build

# 运行（随机地图，400个智能体）
./build/main -m assets/random-32-32-20.map -N 400 -v 1

# 运行（场景文件，makespan 优化）
./build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 1 -O 1

# 查看所有参数
./build/main --help
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-m` | 必填 | 地图文件路径（`.map`） |
| `-i` | `""` | 场景文件路径（`.scen`），为空则随机生成 |
| `-N` | 必填 | 智能体数量 |
| `-s` | `0` | 随机种子 |
| `-v` | `0` | 输出详细程度（0=静默，1=基本，2+=详细） |
| `-t` | `3` | 时间限制（秒） |
| `-o` | `./build/result.txt` | 输出文件路径 |
| `-O` | `0` | 目标函数（0=无，1=makespan，2=sum_of_loss） |
| `-r` | `0.001` | 随机重启概率（暂未使用） |
| `-l` | `false` | 简短日志（不输出路径详情） |

---

## 输出格式

结果文件（`-o` 指定）包含：
```
agents=<N>
map_file=<地图文件名>
solver=planner
solved=<0|1>
soc=<sum_of_costs>
soc_lb=<lower_bound>
makespan=<makespan>
makespan_lb=<lower_bound>
sum_of_loss=<sum_of_loss>
comp_time=<毫秒>
seed=<随机种子>
starts=(x0,y0),(x1,y1),...
goals=(x0,y0),(x1,y1),...
solution=
0:(x0,y0,方向),(x1,y1,方向),...
1:(x0,y0,方向),...
...
```

---

## 关键设计决策与注意事项

1. **距离表使用 Eager BFS**：在初始化时一次性完成所有可达状态的距离计算，不使用懒惰求值（`setup()` 函数已废弃）。这保证了查询时间为 O(1)，但初始化时间略长。

2. **方向对 Config 哈希的影响**：`ConfigHasher` 同时哈希位置和方向，保证不同朝向的相同位置集合不会被认为是同一高层节点。但目标检测 `is_same_config_pos` 只比较位置，避免朝向差异导致找不到目标。

3. **`reserved_nodes` 机制**：当 `computeAction` 判断需要转向时，智能体原地等待并记录目标节点。下一轮 `funcPIBT` 时优先尝试该节点，确保转向后能及时移动。

4. **`swap_completed` 标志**：在 Swap 完成之前，智能体保留对目标位置的预定，防止其他智能体抢占。

5. **注释代码保留**：`planner.cpp` 中保留了大量注释代码，记录了多个历史版本（纯 DFS、DFS+BFS 双队列等），便于理解算法演化过程，不建议删除。

6. **`get_edge_cost` 的逆向移动惩罚**：若某智能体需要向非正前方移动（等效于先掉头再移动再复位），代价设为 5（其余情况为 1）。这影响高层节点的 f 值计算与剪枝。
