/*
 * src/planner.cpp
 */
#include "../include/planner.hpp"

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
             const uint _h)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
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
                 const float _restart_rate)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      RESTART_RATE(_restart_rate),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr),
      validation_table(V_size, nullptr), //测试：删除错误配置（可以删了？
      reserved_nodes(N, nullptr),
      push_count_table(N, std::vector<int>(N, 0))
{
  modified_indices.reserve(N); // 测试：删除错误配置
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
    // UCB 阶段统计
    // ============================================================
    struct PhaseStats {
        double total_reward = 0.0;
        int selection_count = 0;
        
        double getUCB(int total_sel, double c = 1.414) const {
            if (selection_count == 0) return 1e9;
            return (total_reward / selection_count) + 
                   c * std::sqrt(std::log(total_sel + 1) / selection_count);
        }
    };
    
    PhaseStats phases[3];
    int total_ucb_selections = 0;

    // ============================================================
    // [新增] 记录每个区域的边界索引
    // ============================================================
    // 区域划分：[0, boundary[0]) [boundary[0], boundary[1]) [boundary[1], size)
    // 即：      Phase 0           Phase 1                   Phase 2
    size_t boundary[2] = {0, 0};  // 两个分界点

    while (!OPEN.empty() && !is_expired(deadline)) {
        loop_cnt += 1;

        HNode* H = nullptr;
        int selected_phase = -1;

        if (H_goal == nullptr) {
            // ============================================================
            // 阶段1：寻找初始解，纯 DFS
            // ============================================================
            H = OPEN.back();
            OPEN.pop_back();
            
            // 更新边界（后端减少了一个）
            if (boundary[1] > OPEN.size()) boundary[1] = OPEN.size();
            if (boundary[0] > boundary[1]) boundary[0] = boundary[1];
        } 
        else {
            // ============================================================
            // 阶段2：优化阶段，UCB 选择
            // ============================================================
            size_t n = OPEN.size();
            if (n == 0) break;

            // 确保边界合法
            if (boundary[0] > n) boundary[0] = n;
            if (boundary[1] > n) boundary[1] = n;
            if (boundary[1] < boundary[0]) boundary[1] = boundary[0];

            // 计算各区域大小
            size_t size_phase0 = boundary[0];
            size_t size_phase1 = boundary[1] - boundary[0];
            size_t size_phase2 = n - boundary[1];

            // 选择 UCB 最大且非空的区域
            double best_ucb = -1.0;
            for (int i = 0; i < 3; ++i) {
                size_t phase_size = (i == 0) ? size_phase0 : 
                                    (i == 1) ? size_phase1 : size_phase2;
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
                if (boundary[1] > OPEN.size()) boundary[1] = OPEN.size();
                if (boundary[0] > boundary[1]) boundary[0] = boundary[1];
            } else {
                // 确定区间
                size_t start_idx, end_idx;
                if (selected_phase == 0) {
                    start_idx = 0;
                    end_idx = boundary[0];
                } else if (selected_phase == 1) {
                    start_idx = boundary[0];
                    end_idx = boundary[1];
                } else {
                    start_idx = boundary[1];
                    end_idx = n;
                }

                // 在区间内随机选择
                std::uniform_int_distribution<size_t> dist(start_idx, end_idx - 1);
                size_t chosen_idx = dist(*MT);

                H = OPEN[chosen_idx];
                OPEN.erase(OPEN.begin() + chosen_idx);

                // 更新边界（删除了一个元素）
                if (chosen_idx < boundary[0]) {
                    boundary[0]--;
                    boundary[1]--;
                } else if (chosen_idx < boundary[1]) {
                    boundary[1]--;
                }
                // 如果在 Phase 2 删除，边界不变
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
            
            // 初始化边界：将当前 OPEN 三等分
            size_t n = OPEN.size();
            boundary[0] = n / 3;
            boundary[1] = 2 * n / 3;
            
            if (objective == OBJ_NONE) break;
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
                if (selected_phase == 0) {
                    OPEN.insert(OPEN.begin(), H);
                    boundary[0]++;
                    boundary[1]++;
                } else if (selected_phase == 1) {
                    OPEN.insert(OPEN.begin() + boundary[0], H);
                    boundary[1]++;
                } else {
                    OPEN.push_back(H);
                }
            }
        }

        // create new configuration
        if (!get_new_config(H, L)) {
            delete L;
            if (selected_phase >= 0) {
                phases[selected_phase].selection_count++;
                total_ucb_selections++;
                // 失败，0 奖励
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
            rewrite(H, iter->second, H_goal, OPEN, boundary);

            if (selected_phase >= 0) {
                phases[selected_phase].selection_count++;
                total_ucb_selections++;
                phases[selected_phase].total_reward += 0.05;
            }
        } else {
            // ============================================================
            // [核心修改] 新节点插入父节点所在的区域
            // ============================================================
            auto H_new = new HNode(C_new, D, H, H->g + get_edge_cost(H->C, C_new),
                                   get_h_value(C_new));
            EXPLORED[H_new->C] = H_new;
            GC.push_back(H_new);

            if (H_goal == nullptr) {
                // 阶段1：直接放后端
                OPEN.push_back(H_new);
            } else {
                // 阶段2：插入到父节点所在的区域
                if (selected_phase == 0) {
                    // 插入 Phase 0 的末尾
                    OPEN.insert(OPEN.begin() + boundary[0], H_new);
                    boundary[0]++;
                    boundary[1]++;
                } else if (selected_phase == 1) {
                    // 插入 Phase 1 的末尾
                    OPEN.insert(OPEN.begin() + boundary[1], H_new);
                    boundary[1]++;
                } else {
                    // 插入 Phase 2 的末尾
                    OPEN.push_back(H_new);
                }
            }

            // 更新 UCB 奖励
            if (selected_phase >= 0) {
                phases[selected_phase].selection_count++;
                total_ucb_selections++;

                double reward = 0.1;
                
                if (is_same_config_pos(H_new->C, ins->goals)) {
                    if (H_new->g < H_goal->g) {
                        solver_info(1, "found better solution: ", H_goal->g, 
                                   " -> ", H_new->g);
                        H_goal = H_new;
                        reward = 1.0;
                    }
                } else if (H_new->f < H->f) {
                    reward = 0.5;
                }
                
                phases[selected_phase].total_reward += reward;
            }
        }
    }

    // 输出统计
    solver_info(1, "end search, node_num: ", GC.size());
    if (H_goal != nullptr) {
        for (int i = 0; i < 3; ++i) {
            double avg = (phases[i].selection_count > 0) ? 
                         phases[i].total_reward / phases[i].selection_count : 0.0;
            solver_info(1, "Phase ", i, ": selections=", phases[i].selection_count,
                       " avg_reward=", avg);
        }
    }

    // backtrack
    if (H_goal == nullptr) {
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

std::ostream& operator<<(std::ostream& os, const Objective objective)
{
  if (objective == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (objective == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}
