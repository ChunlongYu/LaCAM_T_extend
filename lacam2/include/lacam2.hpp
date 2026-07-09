#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "utils.hpp"

// main function
Solution solve(const Instance& ins, std::string& additional_info,
               const int verbose = 0, const Deadline* deadline = nullptr,
               std::mt19937* MT = nullptr, const Objective objective = OBJ_NONE,
               const float restart_rate = 0.001, const float epsilon = 0.0,
               const std::string& conv_log = "", const long pair_lb_ms = 0,
               const bool track_bounds = false,
               const std::string& inherit_log = "");
