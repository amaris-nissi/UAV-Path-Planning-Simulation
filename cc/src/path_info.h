#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include "node_eigen.h"

namespace game_engine {

  // Structure that encapsulates information about a path
  struct PathInfo {
    struct Details {
      // Number of nodes explored
      size_t num_nodes_explored{0};

      // Length of the final path
      size_t path_length{0};

      // Cost to reach the final node
      double path_cost{0};

      // Wall-clock time it took the path-finding algorithm
      std::chrono::duration<double> run_time{0};
  
      // Print the details
      void Print() const {
        std::cout << "===== DETAILS =====" << std::endl;
        std::cout << "  " << "Number of Nodes Explored: " << num_nodes_explored << std::endl;
        std::cout << "  " << "Number of Nodes in Path: " << path_length << std::endl;
        std::cout << "  " << "Cost of path: " << path_cost << std::endl;
        std::cout << "  " << "Run Time: " 
          << std::chrono::duration_cast<std::chrono::microseconds>(run_time).count() 
          << " microseconds" << std::endl;
        std::cout << std::endl;
      }
    };
   
    // These must be filled
    std::vector<std::shared_ptr<Node2D>> path;
    Details details;
  };
}
