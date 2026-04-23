/*
 * lacam-star
 */

#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
std::ostream& operator<<(std::ostream& os, const Objective objective);

// PIBT agent
// struct Agent {
//   const uint id;
//   Vertex* v_now;   // current location
//   Vertex* v_next;  // next location
//   Agent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
// };
struct Agent {
  const uint id;
  Vertex* v_now;   
  Orientation o_now; // 新增：当前方向
  Vertex* v_next;  
  Orientation o_next; // 新增：下一时刻方向

  // [新增] 标记 Swap 是否完成
  bool swap_completed;

  // 构造函数初始化
  Agent(uint _id) 
    : id(_id), 
      v_now(nullptr), o_now(Orientation::Y_MINUS), 
      v_next(nullptr), o_next(Orientation::Y_MINUS),
      swap_completed(true) 
  {}
};
using Agents = std::vector<Agent*>;

// low-level node
// struct LNode {
//   std::vector<uint> who;
//   Vertices where;
//   const uint depth;
//   LNode(LNode* parent = nullptr, uint i = 0,
//         Vertex* v = nullptr);  // who and where
// };
struct LNode {
  std::vector<uint> who;
  // [修改] 原先是 Vertices where; 现在改为存储 State
  std::vector<State> where; 
  const uint depth;
  
  // [修改] 构造函数参数改为 const State& s
  LNode(LNode* parent = nullptr, uint i = 0, State s = {nullptr, Orientation::Y_MINUS});
};

// high-level node
struct HNode {
  static uint HNODE_CNT;  // count #(high-level node)
  const Config C;

  // tree
  HNode* parent;
  std::set<HNode*> neighbor;

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)

  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
        const uint _h);
  ~HNode();
};
using HNodes = std::vector<HNode*>;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // hyper parameters
  const Objective objective;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  DistTable D;
  uint loop_cnt;      // auxiliary

  // used in PIBT
  std::vector<std::array<Vertex*, 5> > C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;              // random values, used in PIBT
  Agents A;
  Agents occupied_now;                          // for quick collision checking
  Agents occupied_next;                         // for quick collision checking
  std::vector<Vertex*> reserved_nodes;          // 用于swap保留节点
  // [新增] k-Push Escape Trigger 表
  // 记录 pusher 推挤 pushed_agent 的次数
  std::vector<std::vector<int>> push_count_table;

  // [修改/新增] 核心规划函数
  bool funcPIBT(Agent* ai, Agent* pusher = nullptr, bool is_initial = true);
  
  /**
   * @brief 计算从当前状态前往目标邻居所需的物理动作
   * @param current         当前节点
   * @param target          期望前往的目标节点（通常是 neighbor 或 current 本身）
   * @param current_orient  当前朝向
   * @return std::pair<Vertex*, Orientation> 返回下一步的 (位置, 朝向)
   */
  std::pair<Vertex*, Orientation> computeAction(Vertex* current, 
                                                Vertex* target, 
                                                Orientation current_orient);

  // [新增] 辅助转向逻辑：逆时针旋转 90 度
  Orientation rotateCounterClockwise(Orientation orient);
  
  // 测试：去除添加约束后出现冲突的高层节点
  std::vector<Agent*> validation_table; // 映射: 节点ID -> 占用它的Agent
  std::vector<int> modified_indices;    // 记录本轮修改过的节点ID，用于快速回滚

  // cycle
  std::vector<std::pair<Agent*, Vertex*>> request_chain;
  Agent* initial_requester = nullptr;
  bool cycle_handled = false;

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001);
  ~Planner();
  Solution solve(std::string& additional_info);
  void expand_lowlevel_tree(HNode* H, LNode* L);
  // 已测试可行rewrite，无ucb
  // void rewrite(HNode* H_from, HNode* T, HNode* H_goal,
  //              std::stack<HNode*>& OPEN);
  void rewrite(HNode* H_from, HNode* T, HNode* H_goal,
             std::deque<HNode*>& OPEN, size_t* boundary);
  uint get_edge_cost(const Config& C1, const Config& C2);
  uint get_edge_cost(HNode* H_from, HNode* H_to);
  uint get_h_value(const Config& C);
  bool get_new_config(HNode* H, LNode* L);
  bool funcPIBT(Agent* ai);
  
  // [新增] 声明处理闭环转向的函数
  void handleCycleWithOrientation();

  // swap operation
  Agent* swap_possible_and_required(Agent* ai, const std::vector<Vertex*>& P);
  bool is_swap_required(const uint pusher, const uint puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);
  // [新增] 获取所有方向最小距离的辅助函数
  float getMinDistAllDirections(int agent_id, Vertex* v);

  // [新增] k-Push Escape Trigger 相关函数声明
  void updatePushCount(int pushed_agent_id, int pusher_id);
  int getPushCount(int pushed_agent_id, int pusher_id) const;
  // 注意：这里用 std::vector<Vertex*>& 对应 planner.cpp 中的类型
  void PushEscapeTrigger(std::vector<Vertex*>& C, int pushed_agent_id, int pusher_id);

  bool in_optimization_phase = false; // [新增]

  // utilities
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << HNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};


