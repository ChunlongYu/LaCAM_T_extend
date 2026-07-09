#include "../include/post_processing.hpp"

#include "../include/dist_table.hpp"
#include <sstream>

// 辅助函数：根据节点索引差计算方向
Orientation get_move_dir(Vertex* v_from, Vertex* v_to, int width) {
    int diff = (int)v_to->index - (int)v_from->index;
    if (diff == 1) return Orientation::X_PLUS;       // Right
    if (diff == -1) return Orientation::X_MINUS;      // Left
    if (diff == width) return Orientation::Y_PLUS;    // Down
    if (diff == -width) return Orientation::Y_MINUS;  // Up
    return Orientation::X_PLUS; // Should not happen for valid neighbors
}

bool is_feasible_solution(const Instance& ins, const Solution& solution,
                          const int verbose)
{
  if (solution.empty()) return true;

  // check start locations
  if (!is_same_config(solution.front(), ins.starts)) {
    info(1, verbose, "invalid starts");
    return false;
  }

  // check goal locations
  if (!is_same_config_pos(solution.back(), ins.goals)) {
    info(1, verbose, "invalid goals");
    return false;
  }

  for (size_t t = 1; t < solution.size(); ++t) {
    for (size_t i = 0; i < ins.N; ++i) {
      auto v_i_from = solution[t - 1][i];
      auto v_i_to = solution[t][i];
      auto state_from = solution[t - 1][i];
      auto state_to = solution[t][i];
      // check connectivity
      if (v_i_from.v != v_i_to.v &&
          std::find(v_i_to.v->neighbor.begin(), v_i_to.v->neighbor.end(),
                    v_i_from.v) == v_i_to.v->neighbor.end()) {
        info(1, verbose, "invalid move");
        return false;
      }

      // 检查是否出现不沿当前朝向的移动
      if (state_from.v != state_to.v) { // 如果发生了移动
          // 计算从 from 到 to 的物理方向
          Orientation move_dir = get_move_dir(state_from.v, state_to.v, ins.G.width);

          // 检查 A: 移动方向必须与出发时的朝向一致 (防倒车/横移)
          if (state_from.o != move_dir) {
              if (verbose) {
                  std::cout << "Invalid Move Direction at t=" << t << " for agent " << i << std::endl;
                  std::cout << "  Pos: " << state_from.v->index << " -> " << state_to.v->index << std::endl;
                  std::cout << "  Agent Facing: " << (int)state_from.o << ", Move Dir: " << (int)move_dir << std::endl;
              }
              info(1, verbose, "invalid move (orientation mismatch)");
              return false;
          }

          // 检查 B: 移动过程中不能转向
          // 只有原地等待时才能转向，移动时 o_from 必须等于 o_to
          if (state_from.o != state_to.o) {
              if (verbose) {
                  std::cout << "Invalid Action at t=" << t << " for agent " << i << ": Rotating while moving!" << std::endl;
              }
              info(1, verbose, "invalid move (rotate while moving)");
              return false;
          }
      }

      // check conflicts
      for (size_t j = i + 1; j < ins.N; ++j) {
        auto v_j_from = solution[t - 1][j];
        auto v_j_to = solution[t][j];
        // vertex conflicts
        if (v_j_to.v == v_i_to.v) {
          info(1, verbose, "vertex conflict");
          return false;
        }
        // swap conflicts
        if (v_j_to.v == v_i_from.v && v_j_from.v == v_i_to.v) {
          info(1, verbose, "edge conflict");
          return false;
        }
      }
    }
  }

  return true;
}

int get_makespan(const Solution& solution)
{
  if (solution.empty()) return 0;
  return solution.size() - 1;
}

// int get_path_cost(const Solution& solution, uint i)
// {
//   const auto makespan = solution.size();
//   const auto g = solution.back()[i];
//   auto c = makespan;
//   while (c > 0 && solution[c - 1][i] == g) --c;
//   return c;
// }

int get_path_cost(const Solution& solution, uint i)
{
  const auto makespan = solution.size();
  const auto g = solution.back()[i]; // g 是 State 类型
  auto c = makespan;
  // 修改：比较时只看位置 g.v
  while (c > 0 && solution[c - 1][i].v == g.v) --c;
  return c;
}

int get_sum_of_costs(const Solution& solution)
{
  if (solution.empty()) return 0;
  int c = 0;
  const auto N = solution.front().size();
  for (size_t i = 0; i < N; ++i) c += get_path_cost(solution, i);
  return c;
}

// int get_sum_of_loss(const Solution& solution)
// {
//   if (solution.empty()) return 0;
//   int c = 0;
//   const auto N = solution.front().size();
//   const auto T = solution.size();
//   for (size_t i = 0; i < N; ++i) {
//     auto g = solution.back()[i];
//     for (size_t t = 1; t < T; ++t) {
//       if (solution[t - 1][i] != g || solution[t][i] != g) ++c;
//     }
//   }
//   return c;
// }

int get_sum_of_loss(const Solution& solution)
{
  if (solution.empty()) return 0;
  int c = 0;
  const auto N = solution.front().size();
  const auto T = solution.size();
  for (size_t i = 0; i < N; ++i) {
    auto g = solution.back()[i]; // 目标 State
    for (size_t t = 1; t < T; ++t) {
      // 修改：只比较位置 .v
      if (solution[t - 1][i].v != g.v || solution[t][i].v != g.v) ++c;
    }
  }
  return c;
}

int get_makespan_lower_bound(const Instance& ins, DistTable& dist_table)
{
  uint c = 0;
  for (size_t i = 0; i < ins.N; ++i) {
    c = std::max(c, dist_table.get(i, ins.starts[i]));
  }
  return c;
}

int get_sum_of_costs_lower_bound(const Instance& ins, DistTable& dist_table)
{
  int c = 0;
  for (size_t i = 0; i < ins.N; ++i) {
    c += dist_table.get(i, ins.starts[i]);
  }
  return c;
}

void print_stats(const int verbose, const Instance& ins,
                 const Solution& solution, const double comp_time_ms)
{
  auto ceil = [](float x) { return std::ceil(x * 100) / 100; };
  auto dist_table = DistTable(ins);
  const auto makespan = get_makespan(solution);
  const auto makespan_lb = get_makespan_lower_bound(ins, dist_table);
  const auto sum_of_costs = get_sum_of_costs(solution);
  const auto sum_of_costs_lb = get_sum_of_costs_lower_bound(ins, dist_table);
  const auto sum_of_loss = get_sum_of_loss(solution);
  info(1, verbose, "solved: ", comp_time_ms, "ms", "\tmakespan: ", makespan,
       " (lb=", makespan_lb, ", ub=", ceil((float)makespan / makespan_lb), ")",
       "\tsum_of_costs: ", sum_of_costs, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_costs / sum_of_costs_lb), ")",
       "\tsum_of_loss: ", sum_of_loss, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_loss / sum_of_costs_lb), ")");
}

// for log of map_name
static const std::regex r_map_name = std::regex(R"(.+/(.+))");

static bool parse_optimal_from_info(const std::string& additional_info,
                                    bool& optimal_value)
{
  if (additional_info.find("optimal=true") != std::string::npos) {
    optimal_value = true;
    return true;
  }
  if (additional_info.find("optimal=false") != std::string::npos) {
    optimal_value = false;
    return true;
  }
  return false;
}

static std::string sanitize_additional_info_for_log(
    const std::string& additional_info)
{
  std::stringstream in(additional_info);
  std::stringstream out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("optimal=", 0) == 0) continue;
    if (line.rfind("mab_", 0) == 0) continue;
    out << line << "\n";
  }
  return out.str();
}

static std::vector<std::string> split_string(const std::string& s, char delim)
{
  std::vector<std::string> tokens;
  std::stringstream ss(s);
  std::string token;
  while (std::getline(ss, token, delim)) tokens.push_back(token);
  return tokens;
}

static std::string get_mab_csv_name(const std::string& output_name)
{
  const size_t sep_pos = output_name.find_last_of("/\\");
  const size_t dot_pos = output_name.find_last_of('.');
  const bool has_ext =
      (dot_pos != std::string::npos &&
       (sep_pos == std::string::npos || dot_pos > sep_pos));
  if (has_ext) return output_name.substr(0, dot_pos) + ".csv";
  return output_name + ".csv";
}

static void write_mab_csv(const std::string& output_name,
                          const std::string& additional_info)
{
  const auto csv_name = get_mab_csv_name(output_name);
  std::ofstream csv(csv_name, std::ios::out);
  if (!csv) return;

  csv << "arm_id,num_selected,total_reward,avg_reward,final_ucb_score,"
         "reward_better_solution,reward_f_improve,reward_new_config,"
         "reward_known_config,reward_fail_new_config,"
         "initial_priority_inheritance_sum\n";

  std::stringstream in(additional_info);
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("mab_arm_stats=", 0) != 0) continue;

    auto fields = split_string(line.substr(std::string("mab_arm_stats=").size()),
                               '\t');
    if (fields.size() < 11) continue;

    csv << fields[0] << ","   // arm_id
        << fields[1] << ","   // num_selected
        << fields[2] << ","   // total_reward
        << fields[3] << ","   // avg_reward
        << fields[4] << ","   // final_ucb_score
        << fields[5] << ","   // reward_better_solution
        << fields[6] << ","   // reward_f_improve
        << fields[7] << ","   // reward_new_config
        << fields[8] << ","   // reward_known_config
        << fields[9] << ","   // reward_fail_new_config
        << fields[10] << "\n"; // initial_priority_inheritance_sum
  }
}

void make_log(const Instance& ins, const Solution& solution,
              const std::string& output_name, const double comp_time_ms,
              const std::string& map_name, const int seed,
              const std::string& additional_info, const bool log_short)
{
  // map name
  std::smatch results;
  const auto map_recorded_name =
      (std::regex_match(map_name, results, r_map_name)) ? results[1].str()
                                                        : map_name;

  // for instance-specific values
  auto dist_table = DistTable(ins);

  // log for visualizer
  auto get_x = [&](int k) { return k % ins.G.width; };
  auto get_y = [&](int k) { return k / ins.G.width; };
  std::ofstream log;
  log.open(output_name, std::ios::out);
  log << "agents=" << ins.N << "\n";
  log << "map_file=" << map_recorded_name << "\n";
  log << "solver=planner\n";
  log << "solved=" << !solution.empty() << "\n";
  bool optimal_from_info = false;
  const bool has_optimal_in_info =
      parse_optimal_from_info(additional_info, optimal_from_info);
  const bool optimal_by_makespan_lb =
      (!solution.empty() &&
       static_cast<int>(solution.size()) ==
           get_sum_of_costs_lower_bound(ins, dist_table));
  const bool optimal =
      !solution.empty() && (optimal_by_makespan_lb ||
                            (has_optimal_in_info && optimal_from_info));
  log << "optimal=" << optimal << "\n";
  log << "soc=" << get_sum_of_costs(solution) << "\n";
  log << "soc_lb=" << get_sum_of_costs_lower_bound(ins, dist_table) << "\n";
  log << "makespan=" << get_makespan(solution) << "\n";
  log << "makespan_lb=" << get_makespan_lower_bound(ins, dist_table) << "\n";
  log << "sum_of_loss=" << get_sum_of_loss(solution) << "\n";
  log << "sum_of_loss_lb=" << get_sum_of_costs_lower_bound(ins, dist_table)
      << "\n";
  log << "comp_time=" << comp_time_ms << "\n";
  log << "seed=" << seed << "\n";
  log << sanitize_additional_info_for_log(additional_info);
  write_mab_csv(output_name, additional_info);
  if (log_short) return;
  log << "starts=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.starts[i].v->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  log << "\ngoals=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.goals[i].v->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  log << "\nsolution=\n";
  for (size_t t = 0; t < solution.size(); ++t) {
    log << t << ":";
    auto C = solution[t];
    for (auto s : C) {
      log << "(" << get_x(s.v->index) << "," << get_y(s.v->index) << "," 
          << orientationToString(s.o) << "),";
    }
    log << "\n";
  }
  log.close();
}
