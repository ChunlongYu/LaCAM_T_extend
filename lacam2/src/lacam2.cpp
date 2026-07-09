#include "../include/lacam2.hpp"

Solution solve(const Instance& ins, std::string& additional_info,
               const int verbose, const Deadline* deadline, std::mt19937* MT,
               const Objective objective, const float restart_rate,
               const float epsilon, const std::string& conv_log,
               const long pair_lb_ms, const bool track_bounds,
               const std::string& inherit_log)
{
  auto planner =
      Planner(&ins, deadline, MT, verbose, objective, restart_rate, epsilon,
              conv_log, pair_lb_ms, track_bounds, inherit_log);
  return planner.solve(additional_info);
}
