#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include "graph.h"
#include "timer.h"
#include "path_info.h"

namespace game_engine {

  struct AStar2D {
    PathInfo Run(const Graph2D& graph, 
                 const std::shared_ptr<Node2D> start_ptr, 
                 const std::shared_ptr<Node2D> end_ptr);
  };
}
