/*
 * src/planner.cpp
 */
#include "../include/planner.hpp"

#include <array>
#include <chrono>
#include <fstream>
#include <queue>
#include <tuple>
#include <unordered_map>

// LNode::LNode(LNode* parent, uint i, Vertex* v)
//     : who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1)
// {
//   if (parent != nullptr) {
//     who = parent->who;
//     who.push_back(i);
//     where = parent->where;
//     where.push_back(v);
//   }
// }

// [修改] 接收 State 而不是 Vertex*
LNode::LNode(LNode* parent, uint i, State s)
    : who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1)
{
  if (parent != nullptr) {
    who = parent->who;
    who.push_back(i);
    where = parent->where;
    where.push_back(s); // 存入状态
  }
}

uint HNode::HNODE_CNT = 0;

// for high-level
HNode::HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
             const uint _h, const int _priority_inheritance_count)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      priority_inheritance_count(_priority_inheritance_count),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<LNode*>())
{
  ++HNODE_CNT;

  //std::cout<<"HNODE_CNT="<<HNODE_CNT<<std::endl;

  search_tree.push(new LNode());
  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) parent->neighbor.insert(this);

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (uint i = 0; i < N; ++i) priorities[i] = (float)D.get(i, C[i]) / N;
  } else {
    // dynamic priorities, akin to PIBT
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0); // 先把order填成[0,1,2,...,n-1]
  std::sort(order.begin(), order.end(),
            [&](uint i, uint j) { return priorities[i] > priorities[j]; }); 
}

HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
                 const int _verbose, const Objective _objective,
                 const float _restart_rate, const float _epsilon,
                 const std::string& _conv_log, const long _pair_lb_ms,
                 const bool _track_bounds, const std::string& _inherit_log)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      RESTART_RATE(_restart_rate),
      EPSILON(_epsilon),
      CONV_LOG(_conv_log),
      PAIR_LB_MS(_pair_lb_ms),
      TRACK_BOUNDS(_track_bounds),
      INHERIT_LOG(_inherit_log),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      current_priority_inheritance_count(0),
      recursively_called_agents(N, false),
      recursive_call_history(),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr),
      validation_table(V_size, nullptr), //测试：删除错误配置（可以删了？
      reserved_nodes(N, nullptr),
      push_count_table(N, std::vector<int>(N, 0))
{
  modified_indices.reserve(N); // 测试：删除错误配置
  recursive_call_history.reserve(N);
  for (uint i = 0; i < N; ++i) A[i] = new Agent(i);
}

Planner::~Planner()
{
  for (auto a : A) delete a;
}


// Solution Planner::solve(std::string& additional_info)
// {
//   solver_info(1, "start search");

//   // setup agents
//   for (uint i = 0; i < N; ++i) {
//     A[i]->v_now = ins->starts[i].v;
//     A[i]->o_now = ins->starts[i].o;

//     A[i]->swap_completed = true; // 测试：swap(reset)
//     reserved_nodes[i] = nullptr; // 测试：swap(reset)
//   }

//   // setup search
//   auto H_init = new HNode(ins->starts, D, nullptr, 0, get_h_value(ins->starts));
//   std::stack<HNode*> OPEN;
//   std::unordered_map<Config, HNode*, ConfigHasher> EXPLORED;
//   std::vector<HNode*> GC;  // garbage collection
//   OPEN.push(H_init);
//   EXPLORED[H_init->C] = H_init;
//   GC.push_back(H_init);

//   // int restart_threshold = 5000; // 测试：每探索1000个节点重启一次
//   // int iter_since_last_restart = 0; // 测试：每探索1000个节点重启一次

//   // DFS
//   HNode* H_goal = nullptr;
//   while (!OPEN.empty() && !is_expired(deadline)) {
//     loop_cnt += 1;

//     // iter_since_last_restart++; // 记录这一轮干活了（不管有没有成果）
//     // if (H_goal != nullptr && iter_since_last_restart > restart_threshold) {
        
//     //     // solver_info(1, "Deep trap detected at loop ", loop_cnt, 
//     //     //                ". Restarting search with bound: ", H_goal->f);

//     //     // 1. 清空 OPEN 表 (放弃当前所有深层待办事项)
//     //     // 注意：不要 delete 里面的指针，GC (垃圾回收) 向量里存着呢，这里只是清空待办列表
//     //     while (!OPEN.empty()) OPEN.pop();
        
//     //     // 2. 重新加入起点
//     //     // 关键：利用 H_goal->f 作为新的天花板，从头开始搜
//     //     OPEN.push(H_init); 
        
//     //     // 3. 重置计数器
//     //     iter_since_last_restart = 0;
        
//     //     // 4. 立即进入下一次循环，开始处理 H_init
//     //     continue;
//     // }

//     //if(loop_cnt % 10000 == 0){std::cout<<"loop_cnt="<<loop_cnt<<std::endl;}

//     // do not pop here!
//     auto H = OPEN.top();

//     // check goal condition
//     if (H_goal == nullptr && is_same_config_pos(H->C, ins->goals)) {
//       H_goal = H;
//       solver_info(1, "found solution, cost: ", H->g);
//       if (objective == OBJ_NONE) break;
//       in_optimization_phase = true; // [开启]
//       continue;
//     }

//     // check invalid nodes: 如果预估总代价比目标的f都大，直接去掉
//     if (H_goal != nullptr && H->f >= H_goal->f) {
//       OPEN.pop();
//       continue;
//     }

//     // check lack of nodes：尝试所有的低层约束
//     if (H->search_tree.empty()) {
//       OPEN.pop();
//       continue;
//     }

//     // expand low-level search tree
//     auto L = H->search_tree.front();
//     H->search_tree.pop();
//     expand_lowlevel_tree(H, L);

//     // create new configuration
//     if (!get_new_config(H, L)) {
//       delete L;
//       continue;
//     }
//     delete L;

//     // create new configuration vector
//     auto C_new = Config(N, State{nullptr});
//     for (auto a : A) {
//       C_new[a->id] = State{a->v_next, a->o_next};
//     }

//     // check explored
//     auto iter = EXPLORED.find(C_new);
//     if (iter != EXPLORED.end()) {

//       // known config
//       rewrite(H, iter->second, H_goal, OPEN); // iter->second 哈希表中的旧节点指针
//       // re-insert
//       auto H_known = iter->second;
//       if (H_known->search_tree.empty()) continue;
//       OPEN.push(H_known);
//     } else {
//       // new config
//       auto H_new = new HNode(C_new, D, H, H->g + get_edge_cost(H->C, C_new),
//                              get_h_value(C_new));
//       EXPLORED[H_new->C] = H_new;
//       GC.push_back(H_new);
//       OPEN.push(H_new);
//     }
    
//   }

//   solver_info(1, "end search, node_num: ", GC.size());

//   // backtrack
//   if (H_goal == nullptr) {
//     for (auto h : GC) delete h;
//     return Solution();
//   }
//   auto solution = Solution();
//   auto H = H_goal;
//   while (H != nullptr) {
//     solution.push_back(H->C);
//     H = H->parent;
//   }
//   std::reverse(solution.begin(), solution.end());

//   // stats
//   for (auto h : GC) delete h;
//   return solution;
// }


// 20260311初步测试可行版，无ucb
// Solution Planner::solve(std::string& additional_info)
// {
//   solver_info(1, "start search");

//   // setup agents
//   for (uint i = 0; i < N; ++i) {
//     A[i]->v_now = ins->starts[i].v;
//     A[i]->o_now = ins->starts[i].o;
//     A[i]->swap_completed = true; 
//     reserved_nodes[i] = nullptr; 
//   }

//   // setup search
//   auto H_init = new HNode(ins->starts, D, nullptr, 0, get_h_value(ins->starts));
  
//   // 定义双队列
//   std::stack<HNode*> OPEN_DFS; 
//   std::queue<HNode*> OPEN_BFS; 
  
//   std::unordered_map<Config, HNode*, ConfigHasher> EXPLORED;
//   std::vector<HNode*> GC;  

//   // 初始节点同时加入
//   OPEN_DFS.push(H_init);
//   OPEN_BFS.push(H_init);

//   EXPLORED[H_init->C] = H_init;
//   GC.push_back(H_init);

//   HNode* H_goal = nullptr;
  
//   // [变量] 定义不同阶段的 DFS 比例
//   const float RATIO_PHASE_1 = 1.0f; // 找解阶段：100% DFS
//   const float RATIO_PHASE_2 = 0.3f; // 优化阶段：50% DFS / 50% BFS

//   while ((!OPEN_DFS.empty() || !OPEN_BFS.empty()) && !is_expired(deadline)) {
//     loop_cnt += 1;

//     // -------------------------------------------------------------
//     // [逻辑] 动态决定当前策略
//     // -------------------------------------------------------------
//     float current_dfs_ratio;
//     if (H_goal == nullptr) {
//         current_dfs_ratio = RATIO_PHASE_1; // 还没找到解，只用DFS
//     } else {
//         current_dfs_ratio = RATIO_PHASE_2; // 找到解了，开始广度覆盖
//     }

//     // -------------------------------------------------------------
//     // [逻辑] 从队列取节点
//     // -------------------------------------------------------------
//     HNode* H = nullptr;
//     bool is_dfs_step = true;

//     // 1. 如果 DFS 空了，强制用 BFS；如果 BFS 空了，强制用 DFS
//     if (OPEN_DFS.empty()) {
//         is_dfs_step = false;
//     } else if (OPEN_BFS.empty()) {
//         is_dfs_step = true;
//     } else {
//         // 2. 都有货，按概率决定
//         // float r = get_random_float(MT); 
//         // if (r < current_dfs_ratio) {
//         //     is_dfs_step = true;
//         // } else {
//         //     is_dfs_step = false;
//         // }
//         if (current_dfs_ratio >= 0.999f) {
//             is_dfs_step = true;
//         } 
//         else if (current_dfs_ratio <= 0.001f) {
//             is_dfs_step = false;
//         } 
//         else {
//             // 只有进入 Phase 2 (0.5) 时才消耗随机数
//             float r = get_random_float(MT); 
//             if (r < current_dfs_ratio) {
//                 is_dfs_step = true;
//             } else {
//                 is_dfs_step = false;
//             }
//         }
//     }

//     // 执行取操作
//     if (is_dfs_step) {
//         H = OPEN_DFS.top();
//         OPEN_DFS.pop();
//     } else {
//         H = OPEN_BFS.front();
//         OPEN_BFS.pop();
//     }

//     // 检查节点是否枯竭 (因为节点可能同时存在于两个队列，可能已经被另一个队列处理完了)
//     if (H->search_tree.empty()) {
//         continue;
//     }

//     // check goal condition
//     if (H_goal == nullptr && is_same_config_pos(H->C, ins->goals)) {
//       H_goal = H;
//       solver_info(1, "found solution, cost: ", H->g);
      
//       // [关键] 找到解后，H_goal 不再是 nullptr
//       // 下一次循环开始，current_dfs_ratio 就会自动变成 0.5
      
//       if (objective == OBJ_NONE) break;
//       continue;
//     }

//     // check invalid nodes (剪枝)
//     // 无论是哪个队列拿出来的，只要比当前解差，就剪掉
//     if (H_goal != nullptr && H->f >= H_goal->f) {
//       continue;
//     }

//     // expand low-level search tree
//     auto L = H->search_tree.front();
//     H->search_tree.pop();
//     expand_lowlevel_tree(H, L);

//     // [回放策略] 如果还有剩余约束，放回原队列，保持该队列的连续性
//     if (!H->search_tree.empty()) {
//         if (is_dfs_step) OPEN_DFS.push(H);
//         else OPEN_BFS.push(H);
//     }

//     // create new configuration
//     if (!get_new_config(H, L)) {
//       delete L;
//       continue;
//     }
//     delete L;

//     auto C_new = Config(N, State{nullptr});
//     for (auto a : A) {
//       C_new[a->id] = State{a->v_next, a->o_next};
//     }

//     // check explored
//     auto iter = EXPLORED.find(C_new);
//     if (iter != EXPLORED.end()) {
//       // known config
//       // Rewrite 通常依然优先使用 DFS 队列，因为我们希望优化能尽快生效
//       rewrite(H, iter->second, H_goal, OPEN_DFS); 
      
//       auto H_known = iter->second;
//       if (!H_known->search_tree.empty()) {
//           OPEN_DFS.push(H_known);
//           // 旧节点复活时，也可以选择加入 BFS，视内存情况而定
//           // OPEN_BFS.push(H_known); 
//       }
//     } else {
//       // new config
//       auto H_new = new HNode(C_new, D, H, H->g + get_edge_cost(H->C, C_new),
//                              get_h_value(C_new));
//       EXPLORED[H_new->C] = H_new;
//       GC.push_back(H_new);
      
//       // [关键] 新节点必须同时加入两个队列
//       // 即使现在是 DFS 阶段 (Phase 1)，我们也必须把它放入 BFS 队列。
//       // 这样一旦找到解切换到 Phase 2，BFS 队列里才有“存档”可以读取。
//       OPEN_DFS.push(H_new);
//       OPEN_BFS.push(H_new);
//     }
//   }

//   solver_info(1, "end search, node_num: ", GC.size());

//   // backtrack
//   if (H_goal == nullptr) {
//     for (auto h : GC) delete h;
//     return Solution();
//   }
//   auto solution = Solution();
//   auto H = H_goal;
//   while (H != nullptr) {
//     solution.push_back(H->C);
//     H = H->parent;
//   }
//   std::reverse(solution.begin(), solution.end());

//   for (auto h : GC) delete h;
//   return solution;
// }

Solution Planner::solve(std::string& additional_info)
{
    solver_info(1, "start search");
    in_optimization_phase = false;

    // setup agents
    for (uint i = 0; i < N; ++i) {
        A[i]->v_now = ins->starts[i].v;
        A[i]->o_now = ins->starts[i].o;
        A[i]->swap_completed = true;
        reserved_nodes[i] = nullptr;
    }

    // setup search
    auto H_init = new HNode(ins->starts, D, nullptr, 0, get_h_value(ins->starts));

    std::deque<HNode*> OPEN;
    std::unordered_map<Config, HNode*, ConfigHasher> EXPLORED;
    std::vector<HNode*> GC;

    OPEN.push_back(H_init);
    EXPLORED[H_init->C] = H_init;
    GC.push_back(H_init);

    HNode* H_goal = nullptr;

    // ============================================================
    // [新增] cheap 全局下界 + bounded-gap 早停 + 收敛日志
    // 参考 docs/LaCAM_star_global_lower_bound_discussion_updated.md §15-18:
    //   LB_cheap = min( g_t(G), min_{X in F_t}[g_t(X)+h(X)] )
    //   其中 F_t = { X : X 的 successor 还没有全部生成完 } = search_tree 非空。
    // 实现要点(已对照本文件核对):
    //   1) g_t 全程为已知图精确最短距离(新节点建立即精确; rewrite() 为
    //      label-correcting 传播,每次调用内收敛; 所有生成边都记入 neighbor),
    //      因此任意时刻取 LB 都安全。
    //   2) :660 的剪枝会把未完成节点(此时 f>=UB)移出 OPEN,但这些节点不会
    //      把 min 压到 UB 以下; 且 rewrite() 会在节点 f 降到 UB 以下时重新
    //      入 OPEN。故 min_{X in OPEN, 未完成} f 与 min_{X in F_t} f 数值一致,
    //      可安全地只扫描 OPEN(比遍历全部已发现节点便宜得多)。
    // ============================================================
    long best_lb = 0;               // 单调不减的有效下界(取历史 max)
    bool stopped_by_gap = false;
    bool optimal_by_bound = false;  // LB 追上 UB => 按界证明最优(无需搜穷 OPEN)
    double final_gap = 1.0;

    // [新增] 逐拓展记录 (继承数, 奖励) 供相关性分析(仅 --inherit_log 开启时)
    // 列: loop_cnt, region, inherit, reward, outcome, h, g, f
    // outcome: 0=fail 1=known 2=new 3=f_improve 4=better_solution
    std::vector<std::array<double, 8>> inherit_history;
    auto log_expansion = [&](long lc, int region, int inherit, double reward,
                             int outcome, HNode* node) {
      if (INHERIT_LOG.empty() || region < 0) return;
      inherit_history.push_back({(double)lc, (double)region, (double)inherit,
                                 reward, (double)outcome, (double)node->h,
                                 (double)node->g, (double)node->f});
    };

    // [新增] pairwise makespan 静态根下界(仅 makespan 目标; 开局算一次抬高地板)
    const long lb_pair = compute_pairwise_makespan_lb(PAIR_LB_MS);
    if (lb_pair > best_lb) best_lb = lb_pair;
    if (lb_pair > 0)
      solver_info(1, "pairwise root LB = ", lb_pair);
    // 每行: (time_ms, ub, lb, gap); ub<0 表示尚未找到可行解
    std::vector<std::array<double, 4>> conv_history;
    const long CONV_SAMPLE_INTERVAL = 64;
    // 采样总开关: 只在需要时才扫 OPEN 算 LB(默认零开销)。
    // 需要 = 写收敛日志 / 开了 bounded-gap / 显式 --track_bounds。
    const bool track_bounds = TRACK_BOUNDS || !CONV_LOG.empty() ||
                              (EPSILON > 0.0f) || (PAIR_LB_MS > 0);

    auto compute_cheap_lb = [&]() -> long {
        long m = -1;  // min over 未完成 OPEN 节点的 f
        for (auto n : OPEN) {
            if (n->search_tree.empty()) continue;  // 仅未完成 frontier F_t
            if (m < 0 || (long)n->f < m) m = (long)n->f;
        }
        if (H_goal != nullptr) {
            long ub = (long)H_goal->g;         // g_t(G)
            if (m < 0 || ub < m) m = ub;       // LB_cheap = min(UB, minOPEN)
        }
        return m;  // <0 表示 OPEN 空且尚无解
    };

    auto sample_convergence = [&]() {
        long cur = compute_cheap_lb();
        if (cur < 0) return;
        if (cur > best_lb) best_lb = cur;      // 有效下界取历史最大,单调不减
        double ub = (H_goal != nullptr) ? (double)H_goal->g : -1.0;
        double gap = (ub > 0) ? (ub - (double)best_lb) / ub : -1.0;
        if (gap >= 0.0) final_gap = gap;
        if (!CONV_LOG.empty())  // 只有要写盘时才累积历史,避免无谓内存增长
          conv_history.push_back(
              {(double)elapsed_ms(deadline), ub, (double)best_lb, gap});
    };

    // ============================================================
    // UCB 阶段统计
    // ============================================================
    struct PhaseStats {
        int num_selected = 0;
        double total_reward = 0.0;
        int selection_count = 0;
        int reward_fail_new_config = 0;
        int reward_known_config = 0;
        int reward_new_config = 0;
        int reward_f_improve = 0;
        int reward_better_solution = 0;
        int initial_priority_inheritance_sum = 0;
        
        double getUCB(int total_sel, double c = 1.414) const {
            if (selection_count == 0) return 1e9;
            return (total_reward / selection_count) + 
                   c * std::sqrt(std::log(total_sel + 1) / selection_count);
        }
    };
    
    constexpr int NUM_PHASES = 6;
    constexpr int NUM_BOUNDARIES = NUM_PHASES - 1;
    std::array<PhaseStats, NUM_PHASES> phases;
    int total_ucb_selections = 0;

    // ============================================================
    // [新增] 记录每个区域的边界索引
    // ============================================================
    // 区域划分：
    // Phase 0: [0, boundary[0))
    // Phase i: [boundary[i-1], boundary[i)) for 1 <= i < NUM_PHASES - 1
    // Phase N-1: [boundary[N-2], size)
    std::array<size_t, NUM_BOUNDARIES> boundary{};

    auto normalize_boundaries = [&](size_t n) {
        for (int i = 0; i < NUM_BOUNDARIES; ++i) {
            if (boundary[i] > n) boundary[i] = n;
        }
        for (int i = 1; i < NUM_BOUNDARIES; ++i) {
            if (boundary[i] < boundary[i - 1]) boundary[i] = boundary[i - 1];
        }
    };

    while (!OPEN.empty() && !is_expired(deadline)) {
        loop_cnt += 1;

        // [新增] 周期采样收敛曲线 + 按界证明最优 + bounded-gap 早停
        if (track_bounds && loop_cnt % CONV_SAMPLE_INTERVAL == 0) {
            sample_convergence();
            if (H_goal != nullptr) {
                // 按界证明最优:LB 追上 UB => UB=OPT,无需搜穷 OPEN(与 epsilon 无关)
                if (best_lb >= (long)H_goal->g) {
                    optimal_by_bound = true;
                    solver_info(1, "optimal by bound: LB=UB=", H_goal->g);
                    break;
                }
                // bounded-gap 早停(带证书的次优解)
                if (EPSILON > 0.0f && final_gap >= 0.0 &&
                    final_gap <= (double)EPSILON) {
                    stopped_by_gap = true;
                    solver_info(1, "bounded-gap stop: gap=", final_gap,
                                " <= eps=", EPSILON);
                    break;
                }
            }
        }

        HNode* H = nullptr;
        int selected_phase = -1;

        if (H_goal == nullptr) {
            // ============================================================
            // 阶段1：寻找初始解，纯 DFS
            // ============================================================
            H = OPEN.back();
            OPEN.pop_back();
            
            // 更新边界（后端减少了一个）
            normalize_boundaries(OPEN.size());
        } 
        else {
            // ============================================================
            // 阶段2：优化阶段，UCB 选择
            // ============================================================
            size_t n = OPEN.size();
            if (n == 0) break;

            // 确保边界合法
            normalize_boundaries(n);

            // 选择 UCB 最大且非空的区域
            double best_ucb = -1.0;
            for (int i = 0; i < NUM_PHASES; ++i) {
                size_t start_idx = (i == 0) ? 0 : boundary[i - 1];
                size_t end_idx = (i == NUM_PHASES - 1) ? n : boundary[i];
                size_t phase_size = end_idx - start_idx;
                if (phase_size > 0) {
                    double ucb = phases[i].getUCB(total_ucb_selections);
                    if (ucb > best_ucb) {
                        best_ucb = ucb;
                        selected_phase = i;
                    }
                }
            }

            // 如果没选中任何区域（不应该发生）
            if (selected_phase < 0) {
                H = OPEN.back();
                OPEN.pop_back();
                normalize_boundaries(OPEN.size());
            } else {
                // 确定区间
                size_t start_idx = (selected_phase == 0) ? 0 : boundary[selected_phase - 1];
                size_t end_idx = (selected_phase == NUM_PHASES - 1) ? n : boundary[selected_phase];

                // 方案1:在区间内随机选择
                // std::uniform_int_distribution<size_t> dist(start_idx, end_idx - 1);
                // size_t chosen_idx = dist(*MT);

                // 方案2：在区间内随机采样 K 个节点，再选择其中 f 最小的节点
                // constexpr size_t K_SAMPLE = 8;
                // const size_t phase_size = end_idx - start_idx;
                // const size_t sample_cnt = std::min(K_SAMPLE, phase_size);

                // std::vector<size_t> sampled_indices;
                // sampled_indices.reserve(phase_size);
                // for (size_t idx = start_idx; idx < end_idx; ++idx) {
                //     sampled_indices.push_back(idx);
                // }
                // std::shuffle(sampled_indices.begin(), sampled_indices.end(), *MT);
                // sampled_indices.resize(sample_cnt);

                // size_t chosen_idx = sampled_indices[0];
                // uint best_f = OPEN[chosen_idx]->f;
                // for (size_t i = 1; i < sampled_indices.size(); ++i) {
                //     size_t cand_idx = sampled_indices[i];
                //     uint cand_f = OPEN[cand_idx]->f;
                //     if (cand_f < best_f) {
                //         best_f = cand_f;
                //         chosen_idx = cand_idx;
                //     }
                // }
                
                // 方案3：在区间内随机采样 K 个节点，再选择其中 h 最小的节点
                constexpr size_t K_SAMPLE = 8;
                const size_t phase_size = end_idx - start_idx;
                const size_t sample_cnt = std::min(K_SAMPLE, phase_size);

                std::vector<size_t> sampled_indices;
                sampled_indices.reserve(phase_size);
                for (size_t idx = start_idx; idx < end_idx; ++idx) {
                    sampled_indices.push_back(idx);
                }
                std::shuffle(sampled_indices.begin(), sampled_indices.end(), *MT);
                sampled_indices.resize(sample_cnt);

                size_t chosen_idx = sampled_indices[0];
                uint best_h = OPEN[chosen_idx]->h;
                for (size_t i = 1; i < sampled_indices.size(); ++i) {
                    size_t cand_idx = sampled_indices[i];
                    uint cand_h = OPEN[cand_idx]->h;
                    if (cand_h < best_h) {
                        best_h = cand_h;
                        chosen_idx = cand_idx;
                    }
                }

                H = OPEN[chosen_idx];
                OPEN.erase(OPEN.begin() + chosen_idx);
                phases[selected_phase].num_selected++;

                // 更新边界（删除了一个元素）
                for (int i = 0; i < NUM_BOUNDARIES; ++i) {
                    if (chosen_idx < boundary[i]) boundary[i]--;
                }
            }
        }

        // 检查节点是否枯竭
        if (H->search_tree.empty()) {
            continue;
        }

        // check goal condition
        if (H_goal == nullptr && is_same_config_pos(H->C, ins->goals)) {
            H_goal = H;
            solver_info(1, "found initial solution, cost: ", H->g);
            sample_convergence();  // [新增] 记录首个可行解出现的时刻

            // 初始化边界：将当前 OPEN 六等分
            size_t n = OPEN.size();
            for (int i = 1; i <= NUM_BOUNDARIES; ++i) {
                boundary[i - 1] = (static_cast<size_t>(i) * n) / NUM_PHASES;
            }

            for (int i = 0; i < NUM_PHASES; ++i) {
                size_t start_idx = (i == 0) ? 0 : boundary[i - 1];
                size_t end_idx = (i == NUM_PHASES - 1) ? n : boundary[i];
                int sum_priority_inheritance = 0;
                for (size_t idx = start_idx; idx < end_idx; ++idx) {
                    sum_priority_inheritance += OPEN[idx]->priority_inheritance_count;
                }
                phases[i].initial_priority_inheritance_sum = sum_priority_inheritance;
            }
            
            if (objective == OBJ_NONE) break;
            in_optimization_phase = true;
            continue;
        }

        // 剪枝
        if (H_goal != nullptr && H->f >= H_goal->f) {
            continue;
        }

        // expand low-level search tree
        auto L = H->search_tree.front();
        H->search_tree.pop();
        expand_lowlevel_tree(H, L);

        // ============================================================
        // 放回策略：放回原区域
        // ============================================================
        if (!H->search_tree.empty()) {
            if (H_goal == nullptr) {
                OPEN.push_back(H);
            } else {
                // 放回原来的区域
                if (selected_phase < 0 || selected_phase == NUM_PHASES - 1) {
                    OPEN.push_back(H);
                } else {
                    size_t insert_pos = boundary[selected_phase];
                    OPEN.insert(OPEN.begin() + insert_pos, H);
                    for (int i = selected_phase; i < NUM_BOUNDARIES; ++i) {
                        boundary[i]++;
                    }
                }
            }
        }

        // create new configuration
        if (!get_new_config(H, L)) {
            delete L;
            if (selected_phase >= 0) {
                phases[selected_phase].selection_count++;
                total_ucb_selections++;
                phases[selected_phase].reward_fail_new_config++;
                // 失败，0 奖励
                log_expansion(loop_cnt, selected_phase,
                              current_priority_inheritance_count, 0.0, 0, H);
            }
            continue;
        }
        delete L;

        auto C_new = Config(N, State{nullptr});
        for (auto a : A) {
            C_new[a->id] = State{a->v_next, a->o_next};
        }

        // check explored
        auto iter = EXPLORED.find(C_new);
        if (iter != EXPLORED.end()) {
            rewrite(H, iter->second, H_goal, OPEN, boundary.data());

            if (selected_phase >= 0) {
                phases[selected_phase].selection_count++;
                total_ucb_selections++;
                phases[selected_phase].total_reward += 0.05;
                phases[selected_phase].reward_known_config++;
                log_expansion(loop_cnt, selected_phase,
                              current_priority_inheritance_count, 0.05, 1, H);
            }
        } else {
            // ============================================================
            // [核心修改] 新节点插入父节点所在的区域
            // ============================================================
            const int priority_inheritance_count =
                (H_goal == nullptr) ? current_priority_inheritance_count : 0;
            auto H_new = new HNode(C_new, D, H, H->g + get_edge_cost(H->C, C_new),
                                   get_h_value(C_new), priority_inheritance_count);
            EXPLORED[H_new->C] = H_new;
            GC.push_back(H_new);

            if (H_goal == nullptr) {
                // 阶段1：直接放后端
                OPEN.push_back(H_new);
            } else {
                // 阶段2：插入到父节点所在的区域
                if (selected_phase < 0 || selected_phase == NUM_PHASES - 1) {
                    OPEN.push_back(H_new);
                } else {
                    size_t insert_pos = boundary[selected_phase];
                    OPEN.insert(OPEN.begin() + insert_pos, H_new);
                    for (int i = selected_phase; i < NUM_BOUNDARIES; ++i) {
                        boundary[i]++;
                    }
                }
            }

            // 更新 UCB 奖励
            if (selected_phase >= 0) {
                phases[selected_phase].selection_count++;
                total_ucb_selections++;

                double reward = 0.1;
                int outcome = 2;  // new_config
                phases[selected_phase].reward_new_config++;

                if (is_same_config_pos(H_new->C, ins->goals)) {
                    if (H_new->g < H_goal->g) {
                        solver_info(1, "found better solution: ", H_goal->g,
                                   " -> ", H_new->g);
                        H_goal = H_new;
                        reward = 1.0;
                        outcome = 4;  // better_solution
                        phases[selected_phase].reward_better_solution++;
                    }
                } else if (H_new->f < H->f) {
                    reward = 0.5;
                    outcome = 3;  // f_improve
                    phases[selected_phase].reward_f_improve++;
                }

                phases[selected_phase].total_reward += reward;
                log_expansion(loop_cnt, selected_phase,
                              current_priority_inheritance_count, reward, outcome,
                              H);
            }
        }
    }

    const bool open_exhausted = OPEN.empty();

    // [新增] 写逐拓展 (继承数, 奖励) 日志
    if (!INHERIT_LOG.empty()) {
        std::ofstream ilog(INHERIT_LOG);
        ilog << "loop_cnt,region,inherit,reward,outcome,h,g,f\n";
        for (auto& r : inherit_history) {
            ilog << (long)r[0] << "," << (long)r[1] << "," << (long)r[2] << ","
                 << r[3] << "," << (long)r[4] << "," << (long)r[5] << ","
                 << (long)r[6] << "," << (long)r[7] << "\n";
        }
    }

    // [新增] 收尾采样 + 写收敛日志
    if (track_bounds) sample_convergence();
    if (!CONV_LOG.empty()) {
        std::ofstream cf(CONV_LOG);
        cf << "time_ms,ub,lb,gap\n";
        for (auto& r : conv_history) {
            cf << r[0] << ",";
            if (r[1] >= 0) cf << (long)r[1];  // ub (空 = 尚无解)
            cf << "," << (long)r[2] << ",";
            if (r[3] >= 0) cf << r[3];         // gap (空 = 尚无解)
            cf << "\n";
        }
    }

    // 输出统计
    solver_info(1, "end search, node_num: ", GC.size());
    if (H_goal != nullptr) {
        for (int i = 0; i < NUM_PHASES; ++i) {
            double avg = (phases[i].selection_count > 0) ? 
                         phases[i].total_reward / phases[i].selection_count : 0.0;
            solver_info(1, "Phase ", i, ": selections=", phases[i].selection_count,
                       " avg_reward=", avg);
        }
    }

    additional_info += "mab_num_arms=" + std::to_string(NUM_PHASES) + "\n";
    additional_info +=
        "mab_arm_stats_header=arm_id\tnum_selected\ttotal_reward\tavg_reward\t"
        "final_ucb_score\treward_better_solution\treward_f_improve\t"
        "reward_new_config\treward_known_config\treward_fail_new_config\t"
        "initial_priority_inheritance_sum\n";
    for (int i = 0; i < NUM_PHASES; ++i) {
      const auto& arm = phases[i];
      const double avg_reward =
          (arm.num_selected > 0) ? arm.total_reward / arm.num_selected : 0.0;
      const double final_ucb_score = arm.getUCB(total_ucb_selections);
      additional_info +=
          "mab_arm_stats=" + std::to_string(i) + "\t" +
          std::to_string(arm.num_selected) + "\t" +
          std::to_string(arm.total_reward) + "\t" +
          std::to_string(avg_reward) + "\t" +
          std::to_string(final_ucb_score) + "\t" +
          std::to_string(arm.reward_better_solution) + "\t" +
          std::to_string(arm.reward_f_improve) + "\t" +
          std::to_string(arm.reward_new_config) + "\t" +
          std::to_string(arm.reward_known_config) + "\t" +
          std::to_string(arm.reward_fail_new_config) + "\t" +
          std::to_string(arm.initial_priority_inheritance_sum) + "\n";
    }

    // backtrack
    if (H_goal == nullptr) {
        additional_info += "optimal=false\n";
        additional_info += "optimal_by_open_exhaustion=false\n";
        additional_info += "optimal_by_makespan_lb=false\n";
        for (auto h : GC) delete h;
        return Solution();
    }
    
    auto solution = Solution();
    auto H = H_goal;
    while (H != nullptr) {
        solution.push_back(H->C);
        H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());

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
    const bool optimal = optimal_by_open_exhaustion || optimal_by_makespan_lb ||
                         optimal_by_bound;
    additional_info +=
        std::string("optimal=") + (optimal ? "true" : "false") + "\n";
    additional_info += std::string("optimal_by_open_exhaustion=") +
                       (optimal_by_open_exhaustion ? "true" : "false") + "\n";
    additional_info += std::string("optimal_by_makespan_lb=") +
                       (optimal_by_makespan_lb ? "true" : "false") + "\n";
    additional_info += std::string("optimal_by_bound=") +
                       (optimal_by_bound ? "true" : "false") + "\n";

    // [新增] bounded-gap 证书信息
    const long final_ub = (H_goal != nullptr) ? (long)H_goal->g : -1;
    additional_info += "lb_pair=" + std::to_string(lb_pair) + "\n";
    additional_info += "epsilon=" + std::to_string(EPSILON) + "\n";
    additional_info += std::string("stopped_by_gap=") +
                       (stopped_by_gap ? "true" : "false") + "\n";
    additional_info += "cheap_lb=" + std::to_string(best_lb) + "\n";
    additional_info += "cheap_ub=" + std::to_string(final_ub) + "\n";
    additional_info += "final_gap=" + std::to_string(final_gap) + "\n";

    for (auto h : GC) delete h;
    return solution;
}


// void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
// {
//   if (L->depth >= N) return;
//   const auto i = H->order[L->depth];
  
//   auto C = H->C[i].v->neighbor;
//   C.push_back(H->C[i].v);  // wait

//   // random shuffle
//   std::shuffle(C.begin(), C.end(), *MT);
//   // insert
//   for (auto v : C) H->search_tree.push(new LNode(L, i, v));
// }

void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
{
  if (L->depth >= N) return;
  
  // 1. 确定当前轮到哪个智能体（同样是根据优先级）
  const auto i = H->order[L->depth];
  
  // 2. 获取该智能体在上一时刻的状态 (位置 + 朝向)
  State curr = H->C[i]; 
  
  // 3. 生成候选动作 (Next States)
  std::vector<State> candidates;

  // --- 动作 A: 原地等待 (Wait) ---
  // 保持位置不变，保持方向不变
  candidates.push_back(curr);

  // --- 动作 B: 旋转 (Rotate) ---
  // 位置不变，方向改变 +/- 90度
  // 定义左转和右转的映射
  auto get_rotation = [](Orientation o, bool clockwise) -> Orientation {
      // 假设定义顺序: X_PLUS(0), Y_PLUS(1), X_MINUS(2), Y_MINUS(3)
      int idx = static_cast<int>(o);
      int next_idx = clockwise ? (idx + 1) % 4 : (idx + 3) % 4;
      return static_cast<Orientation>(next_idx);
  };

  candidates.push_back({curr.v, get_rotation(curr.o, true)});  // 右转 (顺时针)
  candidates.push_back({curr.v, get_rotation(curr.o, false)}); // 左转 (逆时针)

  // --- 动作 C: 向前移动 (Move Forward) ---
  // 只能移动到当前朝向正对面的邻居
  for (auto neighbor_v : curr.v->neighbor) {
      // 计算从 curr.v 到 neighbor_v 的向量方向
      int diff_con = (int)neighbor_v->index - (int)curr.v->index;
      //int diff = (int)curr.v->index - (int)neighbor_v->index;
      Orientation move_dir;
      bool is_valid_neighbor = false;

      // 假设地图宽度为 width (从 ins 获取)
      int w = ins->G.width;

      if (diff_con == 1) { move_dir = Orientation::X_PLUS; is_valid_neighbor = true; }
      else if (diff_con == -1) { move_dir = Orientation::X_MINUS; is_valid_neighbor = true; }
      else if (diff_con == w) { move_dir = Orientation::Y_PLUS; is_valid_neighbor = true; }
      else if (diff_con == -w) { move_dir = Orientation::Y_MINUS; is_valid_neighbor = true; }

      // 只有当邻居的方向 == 当前朝向时，才能移动（不能斜着走，也不能平移）
      if (is_valid_neighbor && move_dir == curr.o) {
          // 移动后：位置变了，但朝向保持不变（因为是直线移动）
          candidates.push_back({neighbor_v, curr.o});
          break; // 网格图中正前方只有一个邻居
      }
  }

  // 4. 随机打乱候选顺序
  std::shuffle(candidates.begin(), candidates.end(), *MT);

  // 5. 将候选状态插入搜索树 (Constraint Tree)
  // 这里的 LNode 构造函数已经按照第2步修改过，接受 State 类型
  for (const auto& next_state : candidates) {
      H->search_tree.push(new LNode(L, i, next_state));
  }
}

// 辅助函数：计算从 v_from 到 v_to 的所需方向
Orientation get_direction(Vertex* v_from, Vertex* v_to, int width) {
    int diff = (int)v_to->index - (int)v_from->index;
    if (diff == 1) return Orientation::X_PLUS;
    if (diff == -1) return Orientation::X_MINUS;
    if (diff == width) return Orientation::Y_PLUS;
    if (diff == -width) return Orientation::Y_MINUS;

    return Orientation::X_PLUS;
}

// 检查从上一时刻配置 C_from 到当前配置 C_to 是否物理合法
// 必须放在 rewrite 函数之前
bool is_valid_transition(const Config& C_from, const Config& C_to, int width) {
    for (size_t i = 0; i < C_from.size(); ++i) {
        const auto& s1 = C_from[i]; // t-1
        const auto& s2 = C_to[i];   // t

        if (s1.v != s2.v) {
            // 发生了位移：
            // 1. 必须保持朝向不变 (模型约束)
            if (s1.o != s2.o) return false;
            
            // 2. 移动方向必须与出发时的朝向一致
            Orientation move_dir = get_direction(s1.v, s2.v, width);
            if (move_dir != s1.o) return false;
        }
        // 如果 s1.v == s2.v (原地)，则是等待或转向，总是合法的
    }
    return true;
}

// ---初版rewrite，无法考虑配置间不双向互联的问题
// void Planner::rewrite(HNode* H_from, HNode* T, HNode* H_goal,
//                       std::stack<HNode*>& OPEN)
// {
//   // update neighbors
//   T->neighbor.insert(H_from);
//   H_from->neighbor.insert(T);

//   // Dijkstra
//   std::queue<HNode*> Q;
//   Q.push(H_from);
//   while (!Q.empty()) {
//     auto n_from = Q.front();
//     Q.pop();
//     for (auto n_to : n_from->neighbor) {
//       auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
//       if (g_val < n_to->g) {
//         if (n_to == H_goal)
//           solver_info(1, "cost update: ", H_goal->g, " -> ", g_val);
//         n_to->g = g_val;
//         n_to->f = n_to->g + n_to->h;
//         n_to->parent = n_from;
//         Q.push(n_to);
//         if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
//       }
//     }
//   }
// }

// [新增] k-Push Escape Trigger 实现
void Planner::updatePushCount(int pushed_agent_id, int pusher_id) {
    if (pushed_agent_id >= 0 && pushed_agent_id < (int)push_count_table.size() &&
        pusher_id >= 0 && pusher_id < (int)push_count_table[0].size()) {
        push_count_table[pushed_agent_id][pusher_id]++;
    }
}

int Planner::getPushCount(int pushed_agent_id, int pusher_id) const {
    if (pushed_agent_id >= 0 && pushed_agent_id < (int)push_count_table.size() &&
        pusher_id >= 0 && pusher_id < (int)push_count_table[0].size()) {
        return push_count_table[pushed_agent_id][pusher_id];
    }
    return 0;
}

void Planner::PushEscapeTrigger(std::vector<Vertex*>& C, int pushed_agent_id, int pusher_id) {
    int push_time = getPushCount(pushed_agent_id, pusher_id);
    // k 值设定为 2，可以根据需要调整
    if (push_time >= 2 && C.size() > 1) { 
        // 使用 Planner 类成员变量 MT 进行随机打乱
        std::shuffle(C.begin(), C.end(), *MT);
        push_count_table[pushed_agent_id][pusher_id] = 0; // 重置计数
        // std::cout << "Triggered Escape for " << pushed_agent_id << " pushed by " << pusher_id << std::endl;
    }
}

// 20260311初步测试可行，无ucb
// void Planner::rewrite(HNode* H_from, HNode* T, HNode* H_goal,
//                       std::stack<HNode*>& OPEN) //本轮搜索从H_from触发，生成了new_config对应老节点T
// {
//   // update neighbors
//   // [修改] 原版是双向插入，现在我们只知道 H_from -> T 是刚才探索到的路径，肯定是合法的。
//   // T -> H_from 未必合法。
//   // 但为了保守起见，我们可以保留双向插入，但在下面的循环里严格检查 transition。
//   // 不过更严谨的做法是只插入单向，但 rewrite 需要反向传播优化，所以通常保留双向关系，
//   // 依靠 is_valid_transition 或修改 get_edge_cost 来过滤非法边。
  
//   // -------------------------------------------------------
//   // 第一步：建立图的连接关系 (Update Neighbors)
//   // -------------------------------------------------------
//   // 意义：LaCAM 即使是基于树的搜索，本质上也是在探索一个图。
//   // 这里记录 H_from 和 T 是互为邻居的。
//   // T 是我们在 EXPLORED 表里找到的“旧节点”，H_from 是我们刚刚生成的新节点。
//   // 图结构上，从 H_from 可以到 T，从 T 也可以到 H_from（假设是无向图，get_edge_cost控制局部搜索方向）
//   //T->neighbor.insert(H_from); 
//   H_from->neighbor.insert(T); //双向添加邻居3关系
// //   if (is_valid_transition(T->C, H_from->C, ins->G.width)) {
// //     T->neighbor.insert(H_from);
// //   }
//   T->neighbor.insert(H_from);

//   // Dijkstra

//   // -------------------------------------------------------
//   // 第二步：初始化传播队列 (Dijkstra/BFS Initialization)
//   // -------------------------------------------------------
//   // 意义：我们需要从 H_from 开始，像波纹一样向外检查，看看有没有节点的 G 值（从起点到该点的代价）可以被更新。
//   // 使用队列 Q 来存储需要检查的节点。
//   std::queue<HNode*> Q;
//   Q.push(H_from);
//   Q.push(T); 
  
//   // [新增] 如果我们要从 H_from 开始优化，我们应该也可以把 T 放进去，
//   // 因为 T 是刚刚发现的节点，它的 g 值可能也能优化别人。
//   // Q.push(T); 

//   while (!Q.empty()) {
//     auto n_from = Q.front();
//     Q.pop();
    
//     // 遍历 n_from 的所有邻居 n_to
//     for (auto n_to : n_from->neighbor) {

//       // ----------------------------------------------------------------
//       // 只有当 n_from -> n_to 是物理合法的移动时，才允许更新
//       // ----------------------------------------------------------------
//       // if (!is_valid_transition(n_from->C, n_to->C, ins->G.width)) {
//       //     continue; // 物理不可达（例如倒着走），跳过！
//       // }

//       // 意义：计算“如果走 n_from 这条路到达 n_to，总代价是多少？”
//       // n_from->g 是起点到 n_from 的代价。
//       // get_edge_cost 是 n_from 到 n_to 这一步的代价。
//       auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
//       // 如果发现通过n_from走代价更少，那么久更新
//       if (g_val < n_to->g) {
//         if (n_to == H_goal)
//           solver_info(1, "cost update: ", H_goal->g, " -> ", g_val);
        
//         n_to->g = g_val;
//         n_to->f = n_to->g + n_to->h;
//         // 重连父节点，以后回溯是n_to的前一步一定是n_from
        
//         //n_to->parent = n_from;

//         // ============================================================
//         // [关键] 只有物理合法时才更新 parent
//         // ============================================================
//         if (is_valid_transition(n_from->C, n_to->C, ins->G.width)) {
//             n_to->parent = n_from;
//         }
   
//         // 意义：既然 n_to 的代价变小了，那么 n_to 的邻居（以及邻居的邻居）
//         // 如果通过 n_to 走，代价可能也会变小。
//         // 所以把 n_to 加入队列，下一轮循环去检查它的邻居。
//         Q.push(n_to);

//         // reinsert
//         // 意义：如果 n_to 的 F 值变得足够小（比当前找到的最好解 H_goal 还小），
//         // 说明它变成了一个很有潜力的节点，应该让高层搜索再次关注它。
//         // 把它放回 OPEN 表，让主循环有机会再次从它开始扩展。
//         if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
//       }
//     }
//   }
// }

void Planner::rewrite(HNode* H_from, HNode* T, HNode* H_goal,
                      std::deque<HNode*>& OPEN, size_t* boundary)
{
    H_from->neighbor.insert(T);
    T->neighbor.insert(H_from);

    std::queue<HNode*> Q;
    Q.push(H_from);
    Q.push(T);

    while (!Q.empty()) {
        auto n_from = Q.front();
        Q.pop();

        for (auto n_to : n_from->neighbor) {
            auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
            
            if (g_val < n_to->g) {
                if (n_to == H_goal)
                    solver_info(1, "cost update: ", H_goal->g, " -> ", g_val);

                n_to->g = g_val;
                n_to->f = n_to->g + n_to->h;

                if (is_valid_transition(n_from->C, n_to->C, ins->G.width)) {
                    n_to->parent = n_from;
                }

                Q.push(n_to);
                
                // 被更新的节点如果有潜力，加入 Phase 2
                if (H_goal != nullptr && n_to->f < H_goal->f) {
                    OPEN.push_back(n_to);
                    // 不更新边界，因为加到了 Phase 2
                }
            }
        }
    }
}

// original edge cost
uint Planner::get_edge_cost(const Config& C1, const Config& C2)
{
//   // 测试：如果高层节点是单向连接的，把逆向移动边权设置得非常大
//   for (size_t i = 0; i < N; ++i) {
//       // 检查位移
//       if (C1[i].v != C2[i].v) {
//           // 规则1: 移动模型约束 —— 移动时不能改变朝向
//           // (如果您允许边走边转，可以去掉这个，但通常 MAPF-R 不允许)
//           if (C1[i].o != C2[i].o) {
//               return 100000000; // 返回一个足够大的数 (INF)
//           }

//           // 规则2: 方向一致性约束 —— 只能向当前朝向的前方移动
//           // 这能有效拦截 "倒着走" (B -> A) 的情况
//           Orientation move_dir = get_direction(C1[i].v, C2[i].v, ins->G.width);
//           if (move_dir != C1[i].o) {
//               return 100000000; // INF
//           }
//       }
//       // 如果没有位移 (C1==C2)，则是原地等待或转向，总是合法的，继续检查下一个智能体
//   }

  bool is_reverse_move = false;
  
  // 遍历所有智能体
  for (size_t i = 0; i < N; ++i) {
    // 检查是否发生了位置位移
    if (C1[i].v != C2[i].v) {
        
        // 计算移动方向
        Orientation move_dir = get_direction(C1[i].v, C2[i].v, ins->G.width);
        
        // 检查移动方向是否与当前朝向一致
        if (C1[i].o != move_dir) {
            // [情况 A] 逆向移动 (Misaligned Move)
            // 意味着智能体试图向非正前方移动。
            // 在您的模型中，这代表需要：调头(2) + 移动(1) + 调头复位(2) = 5
            is_reverse_move = true;
        } 
        // [情况 B] 正向移动 (Aligned Move)
        // C1[i].o == move_dir，这是正常的向前走，消耗 1
    }
    // [情况 C] 原地不动 (Wait/Rotate)
    // 消耗 1，不需要额外标记
  }

  // 结算 Cost
  // 只要有一个人需要由“调头-移动-调头”组成的 5 步操作，
  // 整个系统状态转移的 Makespan 代价就是 5。
  if (is_reverse_move) {
      return 5;
  }

  // 原有部分：
  if (objective == OBJ_SUM_OF_LOSS) {
    uint cost = 0;
    for (uint i = 0; i < N; ++i) {
      if (C1[i].v != ins->goals[i].v || C2[i].v != ins->goals[i].v) {
        cost += 1;
      }
    }
    return cost;
  }

  // 否则，全是正向移动或原地旋转，标准代价为 1
  return 1;
}

uint Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
  return get_edge_cost(H_from->C, H_to->C);
}

uint Planner::get_h_value(const Config& C)
{
  uint cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (uint i = 0; i < N; ++i) cost = std::max(cost, D.get(i, C[i]));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (uint i = 0; i < N; ++i) cost += D.get(i, C[i]);
  }
  return cost;
}

bool Planner::get_new_config(HNode* H, LNode* L)
{
  current_priority_inheritance_count = 0;
  std::fill(recursively_called_agents.begin(), recursively_called_agents.end(),
            false);
  recursive_call_history.clear();

  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      //a->v_now = nullptr;
      occupied_now[a->v_now->id] = nullptr;
      //a->o_now = Orientation();
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
      a->o_next = Orientation();
    }

    // set occupied now
    a->v_now = H->C[a->id].v;
    a->o_now = H->C[a->id].o;
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (uint k = 0; k < L->depth; ++k) {
    const auto i = L->who[k];        // agent
    //const auto l = L->where[k]->id;  // loc
    const auto l = L->where[k].v->id;

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto y = occupied_now[l];
    if (y != nullptr && y->v_next != nullptr && y->v_next == A[i]->v_now)
      return false;

    // set occupied_next
    A[i]->v_next = L->where[k].v;
    A[i]->o_next = L->where[k].o;
    // if (A[i]->id==1){std::cout<<"a1 is constrained"<<std::endl;}

    occupied_next[l] = A[i];
    
    // 清空未执行完的节点请求信息
    if (reserved_nodes[i] != nullptr) {
        reserved_nodes[i] = nullptr;
    }
  }

  // perform PIBT
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a,nullptr,true)) return false;
  }

  // 测试：生成完new_config之后如果有冲突，就去掉

  // 第一版写法，无法正常求解（留前一半解不了大规模random算例）
  // for (auto a : A) {
  //     if (a->v_next == nullptr) return false; // 异常情况
      
  //     // 检查 Vertex Conflict: 
  //     // 确认 occupied_next 记录的人确实是自己 (防止被别人覆盖了)

  //     if (occupied_next[a->v_next->id] != a) {
  //         return false; 
  //     }

  //     // 检查 Swap Conflict:
  //     // a 从 v_now -> v_next
  //     // b 从 v_next -> v_now (即 b = occupied_now[v_next])
  //     auto b = occupied_now[a->v_next->id];
  //     if (b != nullptr && b != a) {
  //         if (b->v_next == a->v_now) {
  //             return false; // 发生互换
  //         }
  //     } 
  // }

  // 第二版写法
  bool is_valid_config = true;

  for (auto a : A) {
      // 1. 基础检查
      if (a->v_next == nullptr) {
          is_valid_config = false; break;
      }

      int target_id = a->v_next->id;

      // 2. 精准 Vertex Conflict 检查 (不依赖脏的 occupied_next)
      if (validation_table[target_id] != nullptr) {
          is_valid_config = false; break; // 发现两人去同一位置
      }
      validation_table[target_id] = a;       // 登记
      modified_indices.push_back(target_id); // 记录以便回滚

      // 3. Swap Conflict 检查
      auto b = occupied_now[target_id];
      if (b != nullptr && b != a) {
          if (b->v_next == a->v_now) {
              is_valid_config = false; break;
          }
      }

      // 4. 物理合法性检查 (防止倒着走/平移)
      if (a->v_next != a->v_now) {
          // 移动时不能同时改变方向 (除非模型允许)
          if (a->o_next != a->o_now) { is_valid_config = false; break; }
          
          // 移动方向必须与当前朝向一致
          Orientation move_dir = get_direction(a->v_now, a->v_next, ins->G.width);
          if (move_dir != a->o_now) { is_valid_config = false; break; }
      }
  }

  // [重要] 快速回滚清理，供下一次使用
  for (int idx : modified_indices) {
      validation_table[idx] = nullptr;
  }
  modified_indices.clear();

  return is_valid_config;
  
}

void Planner::handleCycleWithOrientation() {
    if (request_chain.empty()) return;

    bool all_oriented_correctly = true;
    std::vector<bool> correct_orientations(request_chain.size(), false);
    int width = ins->G.width;

    // 1. 检查链条中每个智能体是否已经对准了它请求的节点
    for (size_t i = 0; i < request_chain.size(); ++i) {
        Agent* ai = request_chain[i].first;
        Vertex* u = request_chain[i].second;

        // 计算目标节点 u 相对于 ai 当前位置的方向
        Orientation target_dir = get_direction(ai->v_now, u, width);

        // 检查朝向是否一致
        if (ai->o_now == target_dir) {
            correct_orientations[i] = true;
        } else {
            all_oriented_correctly = false;
            correct_orientations[i] = false;
        }
    }

    // 2. 根据检查结果分配动作
    if (!all_oriented_correctly) {
        // [情况 A]: 至少有一个智能体没对准 -> 所有人原地不动，没对准的人进行转向
        for (size_t i = 0; i < request_chain.size(); ++i) {
            Agent* ai = request_chain[i].first;
            Vertex* u = request_chain[i].second;

            // 占用下一时刻的当前位置 (原地等待)
            // 注意：如果之前有人预定了其他位置，这里会覆盖，这是正确的
            if (occupied_next[u->id] == ai) occupied_next[u->id] = nullptr;
            occupied_next[ai->v_now->id] = ai;
            ai->v_next = ai->v_now;

            if (!correct_orientations[i]) {
                // 如果没对准，计算转向
                Orientation target_dir = get_direction(ai->v_now, u, width);
                
                // 简单的转向逻辑：优先顺时针或逆时针转90度
                // 这里为了简单，如果不是180度掉头，就直接转过去；如果是掉头，先转90度
                if ((int)ai->o_now % 2 == (int)target_dir % 2) { 
                    // 180度情况 (0 vs 2 或 1 vs 3) -> 顺时针转 90
                    ai->o_next = (Orientation)(((int)ai->o_now + 3) % 4); 
                } else {
                    // 90度情况 -> 直接对准
                    ai->o_next = target_dir;
                }
            } else {
                // 如果已经对准了，但必须等前面的人转过来，所以保持朝向等待
                ai->o_next = ai->o_now;
            }
        }
    } else {
        // [情况 B]: 所有人都对准了 -> 所有人向前移动，完成闭环旋转
        for (size_t i = 0; i < request_chain.size(); ++i) {
            Agent* ai = request_chain[i].first;
            Vertex* u = request_chain[i].second;

            // 移动到请求的节点
            occupied_next[u->id] = ai;
            ai->v_next = u;
            ai->o_next = ai->o_now; // 移动时保持朝向不变
        }
    }
}

// 辅助函数：Softmax 采样
// 输入：candidates (顶点列表), costs (对应的代价列表), MT (随机数生成器)
// 输出：被选中的那个“首选节点”在 candidates 中的下标
// int softmax_selection(const std::vector<uint>& costs, std::mt19937* MT, float temperature = 1.0f) {
//     if (costs.empty()) return -1;
//     if (costs.size() == 1) return 0;

//     std::vector<double> exp_values;
//     double sum_exp = 0.0;
    
//     // 找到最小 cost 以防止指数爆炸 (数值稳定性)
//     uint min_cost = *std::min_element(costs.begin(), costs.end());

//     for (uint c : costs) {
//         // 使用负 cost，因为我们希望 cost 越小概率越大
//         // (c - min_cost) 保证指数部分是负数或0
//         double val = std::exp(-(double)(c - min_cost) / temperature);
//         exp_values.push_back(val);
//         sum_exp += val;
//     }

//     // 生成随机数进行采样
//     std::uniform_real_distribution<double> dist(0.0, sum_exp);
//     double r = dist(*MT);
    
//     double current_sum = 0.0;
//     for (size_t i = 0; i < exp_values.size(); ++i) {
//         current_sum += exp_values[i];
//         if (r <= current_sum) return i;
//     }
//     return exp_values.size() - 1;
// }

bool Planner::funcPIBT(Agent* ai, Agent* pusher, bool is_initial)
{
  // 1. 初始化
  if (is_initial) {
      request_chain.clear();
      cycle_handled = false;
      initial_requester = ai;
  }

  // [修改] 去掉 !in_optimization_phase 闸,使优化阶段(阶段2)每次拓展也统计
  // 继承数(仅用于 --inherit_log 观测)。注意: current_priority_inheritance_count
  // 只在建节点处(H_goal==nullptr 时)被读入 node 字段,阶段2 恒取 0,故此改动
  // 不影响搜索行为与 initial_priority_inheritance_sum 语义。
  if (!is_initial && !recursively_called_agents[ai->id]) {
      recursively_called_agents[ai->id] = true;
      recursive_call_history.push_back(ai->id);
      ++current_priority_inheritance_count;
  }

  const auto i = ai->id;

  // if(ai->id == 1 or ai->id == 8){std::cout<<"-----start PIBT for a"<<ai->id<<"-----"<<std::endl;}
  
  // 准备候选节点
  std::vector<Vertex*> P = ai->v_now->neighbor;
  P.push_back(ai->v_now);

  // 优化阶段改用softmax前的经典排序操作
  // 随机因子
  for (auto u : P) tie_breakers[u->id] = get_random_float(MT)* 0.001f;

  // 排序
  std::sort(P.begin(), P.end(), [&](Vertex* u, Vertex* v) {
      // (保留你原有的 Cost 计算逻辑)
      auto get_total_cost = [&](Vertex* target) -> uint {
          uint cost1 = 0;
          Orientation next_o = ai->o_now; 
          if (target == ai->v_now) { cost1 = 1; next_o = ai->o_now; } 
          else {
              int diff = (int)target->index - (int)ai->v_now->index;
              Orientation move_dir = get_direction(ai->v_now, target, ins->G.width);
              int turns = 0;
              if (ai->o_now == move_dir) turns = 0;
              else if ((int)ai->o_now % 2 == (int)move_dir % 2) turns = 2; 
              else turns = 1; 
              cost1 = 1 + turns; 
              next_o = move_dir;
          }
          uint cost2 = D.get(i, target, next_o);
          return cost1 + cost2;
      };
      return get_total_cost(u) + tie_breakers[u->id] < get_total_cost(v) + tie_breakers[v->id];
  });
  
  /*
  // 2. 计算每个候选节点的 Cost
  // 我们需要把 Cost 先算出来，供 Softmax 使用
  struct Candidate {
      Vertex* v;
      uint cost;
      float tie_breaker;
  };
  std::vector<Candidate> candidates;
  std::vector<uint> costs_for_softmax; // 仅用于 Softmax 计算

  for (auto u : P) {
      // 计算 Cost (保留您原有的逻辑)
      uint cost1 = 0;
      Orientation next_o = ai->o_now; 
      if (u == ai->v_now) { cost1 = 1; next_o = ai->o_now; } 
      else {
          Orientation move_dir = get_direction(ai->v_now, u, ins->G.width);
          int turns = 0;
          if (ai->o_now == move_dir) turns = 0;
          else if ((int)ai->o_now % 2 == (int)move_dir % 2) turns = 2; 
          else turns = 1; 
          cost1 = 1 + turns; 
          next_o = move_dir;
      }
      uint total_cost = cost1 + D.get(i, u, next_o);
      
      // 生成随机 Tie-breaker
      float tb = get_random_float(MT) * 0.001f;
      
      candidates.push_back({u, total_cost, tb});
      costs_for_softmax.push_back(total_cost);
  }

  // 3. 排序策略分支
  // [新增] 检查是否处于优化阶段 (比如根据 solve 中是否已找到 H_goal)
  // 这里假设您能通过某种方式（如成员变量）访问到 stage2 状态
  // 或者您直接在这里判断: bool use_softmax = (objective != OBJ_NONE); // 简化判断，实际需传入
  
  // 为了演示，假设您已经添加了成员变量 bool in_optimization_phase = false;
  if (in_optimization_phase) {
      // --- Softmax 策略 ---
      
      // 这里的 temperature 可以调节：
      // 0.1: 非常接近贪婪 (几乎只选最好的)
      // 1.0: 标准
      // 5.0: 非常随机 (不仅选好的，差的也经常选)
      int chosen_idx = softmax_selection(costs_for_softmax, MT, 0.000f); 
      
      // 将选中的那个“幸运儿”放到 P 的最前面
      // P 已经被 candidates 替代了，我们需要重构 P
      P.clear();
      
      // 1. 先放被选中的那个
      P.push_back(candidates[chosen_idx].v);
      
      // 2. 剩下的节点怎么排？
      // 策略 A: 剩下的按贪婪排序 (推荐，保持一定理性)
      // 策略 B: 剩下的也按概率排 (太乱了，不推荐)
      
      // 移除已被选中的，剩下的临时列表
      std::vector<Candidate> remains;
      for (size_t k = 0; k < candidates.size(); ++k) {
          if ((int)k != chosen_idx) remains.push_back(candidates[k]);
      }
      
      // 对剩下的按 Cost 从小到大排序
      std::sort(remains.begin(), remains.end(), 
          [](const Candidate& a, const Candidate& b) {
              return a.cost + a.tie_breaker < b.cost + b.tie_breaker;
          });
          
      // 加入 P
      for (const auto& c : remains) P.push_back(c.v);

  } else {
      // --- 原有贪婪策略 (找初始解) ---
      std::sort(candidates.begin(), candidates.end(), 
          [](const Candidate& a, const Candidate& b) {
              return a.cost + a.tie_breaker < b.cost + b.tie_breaker;
          });
      
      P.clear();
      for (const auto& c : candidates) P.push_back(c.v);
  }
  */

  // k-Push Escape Trigger
  if (!is_initial && pusher != nullptr) {
      PushEscapeTrigger(P, ai->id, pusher->id);
  }

  // [新增] Swap 逻辑：检测是否需要 Swap
  Agent* swap_agent = swap_possible_and_required(ai, P);
  if (swap_agent != nullptr){
    std::reverse(P.begin(), P.end());
    // std::cout << "Swap agent :" << swap_agent->id << std::endl;    
  }

  int m = 0;
  // [新增] Reserved Node 处理
  if (reserved_nodes[ai->id] != nullptr) {
    auto it = std::find(P.begin(), P.end(), reserved_nodes[ai->id]);
    if (it != P.end()) {
        Vertex* reserved = *it;
        P.erase(it);
        P.insert(P.begin(), reserved);
    }
  }

  // 遍历候选
  for (size_t k = 0; k < P.size(); ++k) {
    const auto history_size_before_candidate = recursive_call_history.size();
    auto u = P[k];
    
    auto ak = occupied_now[u->id];
    if (occupied_next[u->id]!=nullptr){
      m++; 
      continue;
    }
    if (pusher != nullptr && u == pusher->v_now){
      m++;
      continue;
    }

    // 预定 (Reserve)
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // cycle检测
    if (!is_initial && u == initial_requester->v_now) {
        request_chain.push_back({ai, u}); 
        handleCycleWithOrientation();     
        cycle_handled = true;             
        return true;                      
    }

    // 递归 (Recursion)
    if (ak != nullptr && ak->v_next == nullptr) {
        request_chain.push_back({ai, u});
        
        // [关键修正 2] 将 'ai' 作为 pusher 传给 'ak'
        if (!funcPIBT(ak, ai, false)) { 
            request_chain.pop_back(); 
            while (recursive_call_history.size() > history_size_before_candidate) {
                auto rolled_back_id = recursive_call_history.back();
                recursive_call_history.pop_back();
                recursively_called_agents[rolled_back_id] = false;
                --current_priority_inheritance_count;
            }
            
            // 恢复状态
            ai->v_next = nullptr;
            if (occupied_next[u->id] == ai) {
                occupied_next[u->id] = nullptr;
            }
            m++;
            continue;
        }
    }

    if (cycle_handled) return true;

    // if (ai->id==1 || ai->id == 8){std::cout<<"u_id="<<u->index<<std::endl;}
    
    auto [next_node, next_orientation] = computeAction(ai->v_now, u, ai->o_now);

    // 分支 A: 无法移动到新节点
    if (next_node == ai->v_now) {
        // 确认 v_now 没被 High-Level 约束抢占
        // if (occupied_next[ai->v_now->id] != nullptr && occupied_next[ai->v_now->id] != ai) {
        //      ai->v_next = nullptr;
        //      continue; 
        // }
        ai->v_next = ai->v_now;
        ai->o_next = next_orientation;
        occupied_next[u->id] = nullptr;
        occupied_next[ai->v_now->id] = ai;

        if(ai->swap_completed){reserved_nodes[ai->id] = nullptr;} // reserve the node before swap is completed
        if (next_orientation != ai->o_now){
            reserved_nodes[ai->id] = u;    
        }
    }
    // 分支 B: 可以前进
    else{
        // if agent can moving forward then do so
        ai->v_next = next_node;
        ai->o_next = next_orientation;
        occupied_next[ai->v_next->id] = ai;
        reserved_nodes[ai->id] = nullptr;

        if (!is_initial && pusher != nullptr && next_node != ai->v_now) {
          updatePushCount(ai->id, pusher->id);
        }
    }
    // 分支 C: 自己可以移动，但被别的智能体堵住了
    auto al = occupied_now[u->id];
    if (al != nullptr && al->v_next == al->v_now) {
        // other agent must stay because it will adjust orientation, current agent must also stay
        if(next_node!=ai->v_now){ //if current agent wants to moving forward
        occupied_next[ai->v_now->id] = ai;
        ai->v_next = ai->v_now; // reserve current vertex
        ai->o_next = ai->o_now; 

        reserved_nodes[ai->id] = u; // reserve action
        }
    }

    // [新增] 计算 Swap Agent 的动作
    if (m == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr && 
        (occupied_next[ai->v_now->id] == nullptr or occupied_next[ai->v_now->id] == ai)) {
        
        swap_agent->swap_completed = false;
        swap_agent->v_next = ai->v_now;
        occupied_next[swap_agent->v_next->id] = swap_agent;
        
        auto [next_node_swap_agent, next_orientation_swap_agent] = computeAction(
            swap_agent->v_now,
            swap_agent->v_next,           
            swap_agent->o_now  
        );

        if (next_node_swap_agent == swap_agent->v_now) {
            occupied_next[swap_agent->v_next->id] = nullptr;
            swap_agent->v_next = swap_agent->v_now;
            occupied_next[swap_agent->v_next->id] = swap_agent;
            swap_agent->o_next = next_orientation_swap_agent;
            reserved_nodes[swap_agent->id] = nullptr;
            if (next_orientation_swap_agent != swap_agent->o_now){
                reserved_nodes[swap_agent->id] = ai->v_now; 
            }
        }
        else {
            swap_agent->v_next = next_node_swap_agent;
            swap_agent->o_next = next_orientation_swap_agent;
            occupied_next[swap_agent->v_next->id] = swap_agent;
            reserved_nodes[swap_agent->id] = nullptr;
            swap_agent->swap_completed = true;
        }

        if (ai->v_next == ai->v_now) {
            if(next_node_swap_agent != swap_agent->v_now){
                occupied_next[swap_agent->v_now->id] = swap_agent;
                swap_agent->v_next = swap_agent->v_now;
                swap_agent->o_next = swap_agent->o_now; 
                reserved_nodes[swap_agent->id] = ai->v_now;
            }
        }         
    }

    // // 安全检查：退回的位置是否可行
    // if (occupied_next[ai->v_now->id] != nullptr && occupied_next[ai->v_now->id] != ai) {
    //     ai->v_next = nullptr;
    //     continue; 
    // }
    return true;
  }

  // Failed
  ai->v_next = ai->v_now;
  ai->o_next = ai->o_now;
  occupied_next[ai->v_now->id] = ai;

  return false;
}

// [新增] 辅助函数：获取所有方向中的最小距离
float Planner::getMinDistAllDirections(int agent_id, Vertex* v) {
    uint min_d = 1000000;
    for (int i = 0; i < 4; ++i) {
        uint d = D.get(agent_id, v, static_cast<Orientation>(i));
        if (d < min_d) min_d = d;
    }
    return (float)min_d;
}


Agent* Planner::swap_possible_and_required(Agent* ai, const std::vector<Vertex*>& P)
{
  const auto i = ai->id;
    if (P.empty() || P[0] == ai->v_now) return nullptr;

    auto aj = occupied_now[P[0]->id];
    if (aj != nullptr && aj->v_next == nullptr &&
        is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
        is_swap_possible(aj->v_now, ai->v_now)) {
        return aj;
    }

    for (auto u : ai->v_now->neighbor) {
        auto ak = occupied_now[u->id];
        if (ak == nullptr || P[0] == ak->v_now) continue;
        if (is_swap_required(ak->id, ai->id, ai->v_now, P[0]) &&
            is_swap_possible(P[0], ai->v_now)) {
            return ak;
        }
    }
  return nullptr;
}

// simulate whether the swap is required
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
    auto v_puller = v_puller_origin;
    Vertex* tmp = nullptr;

    while (getMinDistAllDirections(pusher, v_puller) < 
           getMinDistAllDirections(pusher, v_pusher)) {
        auto n = v_puller->neighbor.size();
        for (auto u : v_puller->neighbor) {
            auto a = occupied_now[u->id];
            // 注意：这里检查目标位置是否是该智能体的目标，需 Planner 中有 goals 信息
            // ins->goals[a->id].v 获取目标
            if (u == v_pusher ||
                (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id].v == u)) {
                --n;
            } else {
                tmp = u;
            }
        }
        if (n >= 2) return false;
        if (n <= 0) break;
        v_pusher = v_puller;
        v_puller = tmp;
    }

    return (getMinDistAllDirections(puller, v_pusher) < 
            getMinDistAllDirections(puller, v_puller)) &&
           (getMinDistAllDirections(pusher, v_pusher) == 0 ||
            getMinDistAllDirections(pusher, v_puller) < 
            getMinDistAllDirections(pusher, v_pusher));
}

// simulate whether the swap is possible
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;

  while (v_puller != v_pusher_origin) {
      auto n = v_puller->neighbor.size();
      for (auto u : v_puller->neighbor) {
          auto a = occupied_now[u->id];
          if (u == v_pusher ||
              (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id].v == u)) {
              --n;
          } else {
              tmp = u;
          }
      }
      if (n >= 2) return true;
      if (n <= 0) return false;
      v_pusher = v_puller;
      v_puller = tmp;
  }
  return false;
}

// 参照 plan.cpp 逻辑实现 
std::pair<Vertex*, Orientation> Planner::computeAction(Vertex* current, Vertex* target, Orientation current_orient) {
    if (current == target) {
        return {current, current_orient}; // 原地不动，方向不变
    }

    // 计算目标节点相对于当前位置的方向
    Orientation relative_pos = get_direction(current, target, ins->G.width);
    
    // 计算角度差 (0, 90, 180)
    int dir1 = static_cast<int>(current_orient);
    int dir2 = static_cast<int>(relative_pos);
    int diff = std::abs(dir1 - dir2);
    int angle_diff = (diff % 2 == 0 && diff != 0) ? 180 : (diff == 0 ? 0 : 90);

    if(current == target){
      return {current, current_orient};
    }

    if (angle_diff == 0) {
        // 1. 已经对准：向前移动，保持朝向
        return {target, current_orient};
    } else if (angle_diff == 90) {
        // 2. 90度差：原地转向目标方向
        return {current, relative_pos};
    } else {
        // 3. 180度差：原地逆时针转90度 (参照 plan.cpp rotateCounterClockwise )
        Orientation next_o = static_cast<Orientation>((dir1 + 3) % 4);
        return {current, next_o};
    }
}

// ============================================================
// [新增] pairwise makespan 静态根下界
//   LB^pair = max( max_i d_i,  max_{候选对(i,j)} M2(i,j) )
//   M2(i,j) = 只考虑 i、j(忽略其他 agent)的 2-agent 最优 makespan。
// 可采纳性: 完整解投影到 (i,j) 是一个合法 2-agent 解, 其 makespan <= 完整
//   makespan, 故 M2 <= OPT。单位与主搜索一致(每个原语转移 = 1)。
// 只在 objective == OBJ_MAKESPAN 时有效。受 budget_ms + deadline + 每对扩展
//   上限约束; 只算部分对仍是合法(更松)下界。
// ============================================================
long Planner::compute_pairwise_makespan_lb(long budget_ms)
{
  if (objective != OBJ_MAKESPAN || N < 2 || budget_ms <= 0) return 0;

  const int W = ins->G.width;
  const uint64_t V4 = (uint64_t)V_size * 4;
  auto oint = [](Orientation o) { return static_cast<int>(o); };

  // 单体距离基线
  std::vector<long> d(N);
  long best = 0;
  for (uint i = 0; i < N; ++i) {
    d[i] = (long)D.get(i, ins->starts[i]);
    best = std::max(best, d[i]);
  }

  const auto t0 = std::chrono::steady_clock::now();
  auto over_budget = [&]() {
    if (is_expired(deadline)) return true;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0)
                  .count();
    return ms >= budget_ms;
  };

  // (v,o) 沿朝向 o 前进一格的邻居; 无则 nullptr
  auto forward = [&](Vertex* v, Orientation o) -> Vertex* {
    for (auto u : v->neighbor)
      if (get_direction(v, u, W) == o) return u;
    return nullptr;
  };
  auto enc = [&](Vertex* v, Orientation o) -> uint {
    return v->id * 4u + (uint)oint(o);
  };
  // 每个 agent 单步候选: wait / 转 +-90 / 前进
  auto succ = [&](Vertex* v, Orientation o, std::array<State, 4>& out) -> int {
    int n = 0;
    out[n++] = State{v, o};
    out[n++] = State{v, (Orientation)((oint(o) + 1) % 4)};
    out[n++] = State{v, (Orientation)((oint(o) + 3) % 4)};
    Vertex* f = forward(v, o);
    if (f != nullptr) out[n++] = State{f, o};
    return n;
  };

  const long PER_PAIR_EXP_CAP = 200000;

  struct Nd {
    int f, g;
    Vertex *vi, *vj;
    Orientation oi, oj;
  };
  auto cmp = [](const Nd& a, const Nd& b) { return a.f > b.f; };

  for (uint i = 0; i < N; ++i) {
    if (over_budget()) break;
    const int ax0 = ins->starts[i].v->index % W, ay0 = ins->starts[i].v->index / W;
    const int agx = ins->goals[i].v->index % W, agy = ins->goals[i].v->index / W;
    const int ix0 = std::min(ax0, agx), ix1 = std::max(ax0, agx);
    const int iy0 = std::min(ay0, agy), iy1 = std::max(ay0, agy);
    Vertex* gi = ins->goals[i].v;

    for (uint j = i + 1; j < N; ++j) {
      if (over_budget()) break;

      // 便宜的包围盒重叠剪枝: 不相交则基本无交互, M2 ~ max(d_i,d_j) <= best, 跳过
      const int bx0 = ins->starts[j].v->index % W,
                by0 = ins->starts[j].v->index / W;
      const int bgx = ins->goals[j].v->index % W,
                bgy = ins->goals[j].v->index / W;
      const int jx0 = std::min(bx0, bgx), jx1 = std::max(bx0, bgx);
      const int jy0 = std::min(by0, bgy), jy1 = std::max(by0, bgy);
      const int M = 1;  // margin
      if (ix1 + M < jx0 || jx1 + M < ix0 || iy1 + M < jy0 || jy1 + M < iy0)
        continue;

      Vertex* gj = ins->goals[j].v;
      auto h2 = [&](Vertex* vi, Orientation oi, Vertex* vj, Orientation oj) {
        return std::max((int)D.get(i, State{vi, oi}),
                        (int)D.get(j, State{vj, oj}));
      };

      std::unordered_map<uint64_t, int> g_seen;
      std::priority_queue<Nd, std::vector<Nd>, decltype(cmp)> pq(cmp);
      Vertex* si = ins->starts[i].v;
      Orientation soi = ins->starts[i].o;
      Vertex* sj = ins->starts[j].v;
      Orientation soj = ins->starts[j].o;
      pq.push(Nd{h2(si, soi, sj, soj), 0, si, sj, soi, soj});
      g_seen[(uint64_t)enc(si, soi) * V4 + enc(sj, soj)] = 0;

      long expansions = 0;
      while (!pq.empty()) {
        if ((expansions & 1023) == 0 && over_budget()) break;
        Nd cur = pq.top();
        pq.pop();
        uint64_t ck = (uint64_t)enc(cur.vi, cur.oi) * V4 + enc(cur.vj, cur.oj);
        auto it = g_seen.find(ck);
        if (it != g_seen.end() && it->second < cur.g) continue;  // stale
        if (cur.vi == gi && cur.vj == gj) {
          if (cur.g > best) best = cur.g;  // M2(i,j)
          break;
        }
        if (++expansions > PER_PAIR_EXP_CAP) break;

        std::array<State, 4> ai_s, aj_s;
        int ni = succ(cur.vi, cur.oi, ai_s);
        int nj = succ(cur.vj, cur.oj, aj_s);
        for (int a = 0; a < ni; ++a) {
          for (int b = 0; b < nj; ++b) {
            Vertex* nvi = ai_s[a].v;
            Vertex* nvj = aj_s[b].v;
            if (nvi == nvj) continue;                        // vertex collision
            if (nvi == cur.vj && nvj == cur.vi) continue;    // swap collision
            int ng = cur.g + 1;
            uint64_t nk =
                (uint64_t)enc(nvi, ai_s[a].o) * V4 + enc(nvj, aj_s[b].o);
            auto f2 = g_seen.find(nk);
            if (f2 == g_seen.end() || ng < f2->second) {
              g_seen[nk] = ng;
              pq.push(Nd{ng + h2(nvi, ai_s[a].o, nvj, aj_s[b].o), ng, nvi,
                         nvj, ai_s[a].o, aj_s[b].o});
            }
          }
        }
      }
    }
  }
  return best;
}

std::ostream& operator<<(std::ostream& os, const Objective objective)
{
  if (objective == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (objective == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}
