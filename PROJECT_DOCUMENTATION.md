# LaCAM_T_extend 项目文档

## 1. 项目简介

本项目是论文 [_Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding_](https://kei18.github.io/lacam2/)（IJCAI-23）开源代码的扩展版本。

**核心扩展点：** 在原版 LaCAM2 基础上，为每个智能体（Agent）引入了**朝向（Orientation）**状态。智能体不再是无向的点，而是像真实机器人一样，拥有面朝的方向。移动之前必须先旋转对准目标方向，才能前进。这使问题从经典 MAPF（Multi-Agent Path Finding，多智能体路径规划）升级为 **MAPF-R**（带旋转约束的 MAPF）。

### 运动模型约束
| 动作 | 规则 |
|------|------|
| **向前移动** | 位置改变，朝向不变，且移动方向必须与当前朝向一致 |
| **原地旋转** | 位置不变，朝向旋转 ±90°（每步只能转 90°） |
| **等待** | 位置与朝向均不变 |

---

## 2. 项目目录结构

```
LaCAM_T_extend/
├── main.cpp                   # 程序入口：参数解析、调用求解器
├── CMakeLists.txt             # CMake 构建配置
├── assets/                    # 地图文件与场景文件（来自 MAPF benchmark）
├── tests/                     # 单元测试（早期开发，非完整覆盖）
├── third_party/               # 第三方库（argparse, googletest）
└── lacam2/
    ├── CMakeLists.txt
    ├── include/               # 头文件
    │   ├── orientation.hpp    # 朝向枚举定义
    │   ├── utils.hpp          # 工具函数（计时、随机数、日志）
    │   ├── graph.hpp          # 图结构、State、Config 定义
    │   ├── instance.hpp       # 问题实例定义（起点、终点、地图）
    │   ├── dist_table.hpp     # 带朝向的距离表
    │   ├── planner.hpp        # 规划器（Agent、LNode、HNode、Planner）
    │   ├── lacam2.hpp         # 对外暴露的 solve() 接口
    │   └── post_processing.hpp# 解的质量评估与日志输出
    └── src/                   # 源文件（实现）
        ├── graph.cpp
        ├── instance.cpp
        ├── dist_table.cpp
        ├── planner.cpp
        ├── lacam2.cpp
        └── post_processing.cpp
```

---

## 3. 核心数据结构

### 3.1 Orientation（朝向）
```cpp
// include/orientation.hpp
enum class Orientation {
    X_PLUS,  // (1, 0)  向右（East）
    Y_PLUS,  // (0, 1)  向下（South）
    X_MINUS, // (-1, 0) 向左（West）
    Y_MINUS  // (0, -1) 向上（North）
};
```
四个方向按整数 0~3 编码，相差 1 为 90° 旋转，相差 2 为 180° 掉头。

### 3.2 Vertex（顶点）
```cpp
struct Vertex {
    const uint id;     // 在 V（无障碍节点列表）中的稠密索引
    const uint index;  // 网格中的绝对位置：width * y + x
    std::vector<Vertex*> neighbor; // 四连通邻居
};
```

### 3.3 State（状态）
```cpp
struct State {
    Vertex* v;   // 位置
    Orientation o; // 朝向
};
```
**State** 是本扩展的核心新增结构，将"位置"与"朝向"绑定为一个整体。

### 3.4 Config（配置）
```cpp
using Config = std::vector<State>; // 所有 N 个智能体的状态集合（某一时刻的快照）
```

### 3.5 Solution（解）
```cpp
using Solution = std::vector<Config>; // 从起点到终点的 Config 时间序列
```

### 3.6 Graph（地图图结构）
| 字段 | 含义 |
|------|------|
| `V` | 所有可通行顶点的稠密列表（不含 nullptr）|
| `U` | 按 `width*y+x` 索引的完整列表（障碍处为 nullptr）|
| `width/height` | 地图宽高 |

通过读取 `.map` 文件（MovingAI 格式）构建，`T` 和 `@` 为障碍。

### 3.7 Instance（问题实例）
| 字段 | 含义 |
|------|------|
| `G` | 地图 |
| `starts` | 各智能体起始 State（位置+朝向，初始朝向均为 Y_MINUS）|
| `goals` | 各智能体目标 State（仅位置有意义）|
| `N` | 智能体数量 |

三种构造方式：测试用直接指定索引、从 `.scen` 文件加载、随机生成。

### 3.8 DistTable（带朝向的距离表）
```
table[agent_id][vertex_id * 4 + orientation_int]
```
- 大小：`N × (V_size × 4)`
- 值：从状态 `(vertex, orientation)` 到该智能体目标的**最短步数**
- **构建方式**：对每个智能体，从目标位置（所有4个朝向，距离=0）出发做**反向 BFS**，同时建模旋转（代价 1）和移动（代价 1，且只允许正向移动）两类动作。
- **目标处所有朝向距离均为 0**，意味着到达目标位置后无论朝向均视为完成任务。

### 3.9 Agent（智能体）
| 字段 | 含义 |
|------|------|
| `id` | 智能体编号 |
| `v_now / o_now` | 当前时刻位置与朝向 |
| `v_next / o_next` | 下一时刻位置与朝向（PIBT 规划中填写）|
| `swap_completed` | 当前是否已完成 swap 操作 |

### 3.10 LNode（低层约束节点）
```
who[k]   → 第 k 个已确定动作的智能体 ID
where[k] → 第 k 个已确定动作的目标 State
depth    → 已确定的智能体数量
```
低层搜索树（Constraint Tree）的一个节点，记录已为哪些智能体分配了什么动作。

### 3.11 HNode（高层搜索节点）
| 字段 | 含义 |
|------|------|
| `C` | 此节点代表的 Config（多智能体状态快照）|
| `parent` | 父节点（用于回溯路径）|
| `neighbor` | 邻居节点集合（用于 rewrite/Dijkstra 优化）|
| `g / h / f` | 代价（从起点到此的累积代价 / 启发值 / 总估计代价）|
| `priorities` | 每个智能体的优先级（动态更新）|
| `order` | 优先级排序后的智能体顺序 |
| `search_tree` | 低层搜索队列（LNode 队列）|

---

## 4. 算法整体流程

```
main()
  └─ solve(ins, ...)                  [lacam2.cpp]
       └─ Planner::solve()            [planner.cpp]
            ├─ 初始化 HNode 根节点
            ├─ 主搜索循环 (DFS + BFS 双队列)
            │    ├─ expand_lowlevel_tree(H, L)   → 生成候选动作
            │    ├─ get_new_config(H, L)         → 用 PIBT 求下一 Config
            │    │    └─ funcPIBT(agent, ...)    → 递归分配智能体动作
            │    │         ├─ computeAction()    → 计算单步物理动作
            │    │         ├─ swap_possible_and_required()
            │    │         └─ handleCycleWithOrientation()
            │    └─ rewrite(H, T, H_goal, OPEN) → Dijkstra 优化代价
            └─ 回溯 H_goal 输出 Solution
```

---

## 5. 函数详解

### 5.1 lacam2.cpp

#### `solve(ins, additional_info, verbose, deadline, MT, objective, restart_rate)`
- **功能**：对外暴露的求解接口。创建 `Planner` 对象并调用其 `solve()` 方法。
- **调用关系**：`main()` → `solve()` → `Planner::solve()`

---

### 5.2 planner.cpp — Planner 类核心方法

#### `Planner::solve(additional_info)`
- **功能**：LaCAM2 主搜索循环（双队列版本）。
- **阶段划分**：
  - **Phase 1（找初始解）**：100% 使用 DFS 栈，快速找到一个可行解。
  - **Phase 2（优化解）**：DFS 30% + BFS 70%，广度优先覆盖更多节点以寻找更优解。
- **主要步骤**：
  1. 初始化所有智能体的起始状态，创建根 HNode。
  2. 循环直至超时或队列为空：
     - 取出 HNode，检查是否为目标配置。
     - 从低层搜索树取 LNode，调用 `expand_lowlevel_tree` 扩展候选。
     - 调用 `get_new_config` 用 PIBT 生成新的 Config。
     - 若 Config 已探索：调用 `rewrite` 尝试更新代价；否则创建新 HNode。
  3. 回溯 H_goal 的 parent 链生成 Solution 并反转。

#### `Planner::expand_lowlevel_tree(H, L)`
- **功能**：为低层搜索树生成下一层候选动作。按优先级顺序取下一个待分配的智能体 `i`，生成其所有合法动作（等待、左转、右转、向前移动），打乱顺序后封装为新的 LNode 加入 `H->search_tree`。
- **候选动作**（按运动模型）：
  - 等待（位置和方向不变）
  - 左转 / 右转（位置不变，方向 ±90°）
  - 向前移动（仅当正前方有邻居时有效，位置变，方向不变）

#### `Planner::get_new_config(H, L)`
- **功能**：根据 LNode 中的约束（已固定的智能体动作），为剩余未分配智能体调用 PIBT，生成完整的下一时刻 Config。
- **步骤**：
  1. 重置所有智能体状态缓存（`occupied_now` / `occupied_next`）。
  2. 应用 LNode 中的约束（检查冲突后标记 `occupied_next`）。
  3. 按优先级顺序对未分配智能体调用 `funcPIBT`。
  4. 对生成的 Config 进行合法性验证（顶点冲突、交换冲突、朝向一致性）。

#### `Planner::funcPIBT(ai, pusher, is_initial)`
- **功能**：核心低层规划算法（Priority Inheritance with Backtracking，带朝向扩展版本）。递归地为智能体 `ai` 找到一个无冲突的下一状态。
- **主要逻辑**：
  1. 生成候选节点列表 P（邻居 + 当前位置），按 `cost1 + D.get()` 排序（cost1 为旋转代价，D 为启发距离）。
  2. 检查 k-Push Escape Trigger，防止被同一智能体反复推挤而陷入循环。
  3. 检查是否需要 Swap（调用 `swap_possible_and_required`），如需 Swap 则反转候选列表。
  4. 若有 `reserved_nodes`（因旋转而预定的节点），将其提到优先级最高位。
  5. 遍历候选节点 u：
     - 若 u 已被预定则跳过。
     - 若 u 处有其他智能体 ak 未分配，递归调用 `funcPIBT(ak, ai, false)`（推挤动作）。
     - 若检测到环形等待（u 是最初发起者的当前位置），调用 `handleCycleWithOrientation` 处理。
     - 调用 `computeAction` 决定实际物理动作（移动/转向），填写 `v_next/o_next`。
     - 若目标智能体 ak 因需要转向而原地不动，则当前智能体也需等待并预留节点。
  6. 处理 swap agent 的动作分配。

#### `Planner::computeAction(current, target, current_orient)`
- **功能**：根据当前位置、目标位置和当前朝向，计算下一步的物理动作。
- **返回**：`(下一步位置, 下一步朝向)`
- **规则**：
  - 若 current == target → 等待，保持朝向不变。
  - 若朝向差为 0°（已对准）→ 向前移动到 target，保持朝向。
  - 若朝向差为 90° → 原地旋转对准目标方向。
  - 若朝向差为 180°（正对面）→ 原地逆时针旋转 90°（需两步才能掉头）。

#### `Planner::handleCycleWithOrientation()`
- **功能**：处理环形等待（多个智能体互相等待对方让位）。
- **逻辑**：检查链条中每个智能体是否已对准其请求的节点。若有未对准的智能体，所有人原地等待并各自转向；若所有人都已对准，则同步向前移动，完成环形旋转。

#### `Planner::rewrite(H_from, T, H_goal, OPEN)`
- **功能**：当新路径可以降低已探索节点代价时，用 Dijkstra 传播式更新图中节点的 g 值。只有物理合法的转移（由 `is_valid_transition` 验证）才会更新 parent 指针。

#### `Planner::get_edge_cost(C1, C2)`
- **功能**：计算两个配置之间的转移代价。
- 若有智能体需要"非正向移动"（朝向与移动方向不一致），代价为 5（代表需要转身+移动+复位的多步代价）。
- `OBJ_SUM_OF_LOSS` 模式下，计算所有仍未到达目标的智能体数量之和。
- 其他情况（正向移动/旋转/等待）代价为 1。

#### `Planner::get_h_value(C)`
- **功能**：计算启发值。
- `OBJ_MAKESPAN`：取所有智能体 DistTable 距离的最大值。
- `OBJ_SUM_OF_LOSS`：取所有智能体 DistTable 距离的总和。

#### Swap 相关函数

| 函数 | 功能 |
|------|------|
| `swap_possible_and_required(ai, P)` | 综合判断：在当前候选中是否存在需要且可行的 Swap；返回 swap 对象的指针，否则返回 nullptr |
| `is_swap_required(pusher, puller, v_pusher_origin, v_puller_origin)` | 模拟推挤过程，判断如果不互换位置，pusher 将永远无法前进（陷阱判断）|
| `is_swap_possible(v_pusher_origin, v_puller_origin)` | 判断 puller 是否有足够的空间绕行，使得 Swap 操作物理可行 |

#### k-Push Escape Trigger 相关函数

| 函数 | 功能 |
|------|------|
| `updatePushCount(pushed_agent_id, pusher_id)` | 记录 pusher 对 pushed 的推挤次数 |
| `getPushCount(pushed_agent_id, pusher_id)` | 查询推挤次数 |
| `PushEscapeTrigger(C, pushed_agent_id, pusher_id)` | 若推挤次数 ≥ 2，随机打乱候选列表 C 并重置计数，避免陷入循环推挤 |

---

### 5.3 dist_table.cpp

#### `DistTable::createDistanceTableWithOrientation(ins)`
- **功能**：用反向 BFS 为每个智能体构建带朝向的距离表。
- **初始状态**：目标位置的所有 4 个朝向的距离设为 0（到达目标无论朝向均算完成）。
- **扩展动作**（反向）：
  - **逆向旋转**：若从 `(u, dir_prev)` 旋转 90° 可达 `(u, dir_u)`，则 `dist(u, dir_prev) = dist(u, dir_u) + 1`。
  - **逆向移动**：若从 `(v, dir_u)` 前进一步可达 `(u, dir_u)`（要求 v→u 的方向恰好等于 dir_u），则 `dist(v, dir_u) = dist(u, dir_u) + 1`。

---

### 5.4 graph.cpp

#### `Graph::Graph(filename)`
- 解析 MovingAI `.map` 格式，构建无障碍顶点列表 `V` 和全图索引列表 `U`，并建立四连通边。

#### `is_same_config(C1, C2)` / `is_same_config_pos(C1, C2)`
- 前者比较位置和朝向均相同；后者仅比较位置（用于终止条件判断）。

#### `ConfigHasher::operator()`
- 将 Config 哈希化（同时混合位置 ID 和朝向整数），用于 `EXPLORED` 哈希表。

---

### 5.5 post_processing.cpp

| 函数 | 功能 |
|------|------|
| `is_feasible_solution(ins, solution, verbose)` | 完整验证解的合法性：起终点一致性、连通性、朝向约束（移动时朝向与移动方向一致、移动时不能转向）、顶点冲突、边冲突 |
| `get_makespan(solution)` | 解的时间跨度（步数）|
| `get_path_cost(solution, i)` | 单个智能体的路径代价（到达目标后等待不计入）|
| `get_sum_of_costs(solution)` | 所有智能体路径代价之和 |
| `get_sum_of_loss(solution)` | Sum of Loss（每个智能体离开目标的步数之和）|
| `get_makespan_lower_bound` / `get_sum_of_costs_lower_bound` | 利用 DistTable 计算代价下界 |
| `print_stats(...)` | 控制台输出解的统计信息（代价、下界、上界比）|
| `make_log(...)` | 将解写入 `.txt` 文件，包含每步每个智能体的坐标和朝向 |

---

## 6. 函数调用关系图

```
main()
├── Instance(...)               [instance.cpp]    构建问题实例
│    └── Graph(filename)        [graph.cpp]        读取地图
├── ins.is_valid()
├── solve(ins, ...)             [lacam2.cpp]
│    └── Planner(ins, ...)      [planner.cpp]
│         ├── DistTable(ins)    [dist_table.cpp]   构建距离表
│         └── Planner::solve()
│              ├── HNode(...)                      创建高层节点
│              │    └── DistTable::get()           计算启发值
│              ├── [主循环]
│              │    ├── expand_lowlevel_tree(H, L) 扩展低层搜索树
│              │    ├── get_new_config(H, L)        生成新配置
│              │    │    ├── funcPIBT(a, ...)       PIBT 递归分配
│              │    │    │    ├── computeAction()    计算物理动作
│              │    │    │    ├── swap_possible_and_required()
│              │    │    │    │    ├── is_swap_required()
│              │    │    │    │    └── is_swap_possible()
│              │    │    │    ├── PushEscapeTrigger()
│              │    │    │    └── handleCycleWithOrientation()
│              │    │    └── [合法性验证]
│              │    ├── get_edge_cost(C1, C2)
│              │    ├── get_h_value(C)
│              │    └── rewrite(H, T, H_goal, OPEN)
│              └── [回溯生成 Solution]
├── is_feasible_solution(...)   [post_processing.cpp]
├── print_stats(...)            [post_processing.cpp]
└── make_log(...)               [post_processing.cpp]
```

---

## 7. 高层搜索策略详解

LaCAM2 原版采用纯 DFS（深度优先搜索）快速找解，本项目扩展为**双队列混合策略**：

| 阶段 | 条件 | 策略 |
|------|------|------|
| Phase 1 | 尚未找到可行解（H_goal == nullptr）| 100% DFS，快速深入 |
| Phase 2 | 已找到可行解（H_goal != nullptr）| 30% DFS + 70% BFS（概率采样）|

- **OPEN_DFS**（栈）：后进先出，深度优先。
- **OPEN_BFS**（队列）：先进先出，广度优先。
- 新生成的 HNode **同时**加入两个队列，以便 Phase 2 时 BFS 队列有"存档"可用。
- `rewrite` 发现代价可优化的节点时，重新加入 OPEN_DFS 队列（优先继续优化）。

---

## 8. 朝向扩展的关键设计决策

### 8.1 目标状态的朝向处理
- `goals` 中的朝向设为默认值 `Y_MINUS`，但**终止条件**使用 `is_same_config_pos`（仅比较位置），因此智能体到达目标位置时无论朝向均视为完成任务。
- `DistTable` 中目标位置的所有4个朝向距离均初始化为 0，与此一致。

### 8.2 启发距离的计算
- 距离表以 `(vertex_id * 4 + orientation_int)` 为索引，能准确反映从某个带朝向的状态出发到达目标所需的最少步数（含旋转代价）。
- PIBT 中排序候选节点时使用 `cost1 + D.get(i, target, next_orient)`，其中 `cost1` 为到达目标邻居所需的旋转步数，`D.get` 为从目标邻居出发到终点的距离，二者之和保证了综合代价的估计精度。

### 8.3 多步旋转的保留节点机制
- 当智能体需要先旋转再移动（需多步动作）时，使用 `reserved_nodes[agent_id]` 记录其目标节点，防止后续时间步中该智能体"忘记"自己原本要去哪里。

---

## 9. 构建与运行

### 构建
```sh
cmake -B build && make -C build
```

### 运行示例
```sh
# 无优化目标，随机生成 400 个智能体实例
build/main -v 1 -m assets/random-32-32-20.map -N 400

# makespan 优化，从场景文件加载
build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 1 --objective 1

# 所有参数说明
build/main --help
```

### 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-m` | 地图文件路径 | 必填 |
| `-i` | 场景文件路径（可选，不填则随机生成）| 空 |
| `-N` | 智能体数量 | 必填 |
| `-s` | 随机数种子 | 0 |
| `-v` | 日志详细级别 | 0 |
| `-t` | 时间限制（秒）| 3 |
| `-o` | 输出文件路径 | ./build/result.txt |
| `-O` | 优化目标（0=无，1=makespan，2=sum_of_loss）| 0 |
| `-r` | 随机重启概率 | 0.001 |
| `-l` | 短日志模式（不输出路径）| false |

---

## 10. 与原版 LaCAM2 的主要差异

| 模块 | 原版 LaCAM2 | 本项目扩展 |
|------|-------------|------------|
| **Config 定义** | `vector<Vertex*>` | `vector<State>`（位置+朝向）|
| **DistTable** | 每格 1 个值 | 每格 4 个值（×4 朝向）|
| **距离表构建** | 懒惰 BFS | 预计算完整反向 BFS（含旋转） |
| **低层动作集** | 移动到邻居/等待 | 向前移动/左旋/右旋/等待 |
| **PIBT** | 纯位置冲突处理 | 额外处理旋转等待、reserved node、朝向同步 |
| **Swap 检测** | 基于位置距离 | 基于各朝向最小距离（`getMinDistAllDirections`）|
| **搜索策略** | 纯 DFS | DFS（找解）+ BFS（优化）双队列混合 |
| **边代价** | 1 或 sum_of_loss | 含逆向移动惩罚（代价 5）|
| **可行性验证** | 位置冲突 + 连通性 | 额外验证朝向一致性和移动时不转向 |

---

## 11. 已知局限与待改进点

1. 注释中保留了大量被注释掉的旧版代码，可读性有待提升。
2. `is_swap_required` / `is_swap_possible` 中的模拟逻辑未考虑朝向约束，可能在极端场景下误判。
3. `rewrite` 中对邻居关系的双向添加（`T->neighbor.insert(H_from)`）与物理合法性约束之间存在注释提示的潜在矛盾，仍需进一步验证。
4. 测试用例（`tests/`）为早期开发版本，尚未覆盖带朝向的所有边界场景。
5. `handleCycleWithOrientation` 中的 180° 掉头逻辑采用了简化处理（始终逆时针转 90°），可能不是最优转向路径。
