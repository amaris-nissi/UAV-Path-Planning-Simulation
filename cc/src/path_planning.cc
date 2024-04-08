// Author: PUT YOUR NAME HERE

#include <cstdlib>
#include <iostream>

#include "dijkstra2d.h"
#include "depth_first_search2d.h"
#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"
#include "gui2d.h"

using namespace game_engine;

///////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
void RunDepthFirstSearch(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node);

void RunDijkstra(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node);

void RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node);

///////////////////////////////////////////////////////////////////
// MAIN FUNCTION
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  if(argc != 6) {
    std::cerr << "Usage: ./path_planning occupancy_grid_file row1 col1 row2 col2" << std::endl;
    return EXIT_FAILURE;
  }

  // Parsing input
  const std::string occupancy_grid_file = argv[1];
  const std::shared_ptr<Node2D> start_node = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[2]),std::stoi(argv[3])));
  const std::shared_ptr<Node2D> end_node = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[4]),std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  // Run the path planning algorithms
  RunDepthFirstSearch(graph, &occupancy_grid, start_node, end_node);
  RunDijkstra(graph, &occupancy_grid, start_node, end_node);
  RunAStar(graph, &occupancy_grid, start_node, end_node);

  return EXIT_SUCCESS;
}

///////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
void RunDepthFirstSearch(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node) {
  
  
  std::cout << "============================================" << std::endl;
  std::cout << "=============    RUNNING DFS   =============" << std::endl;
  std::cout << "============================================" << std::endl;

  // Run DFS
  DepthFirstSearch2D dfs;
  PathInfo path_info = dfs.Run(graph, start_node, end_node);

  // Display the solution
  Gui2D gui;
  gui.LoadOccupancyGrid(occupancy_grid);
  gui.LoadPath(path_info.path);
  gui.Display();

  // Print the solution
  path_info.details.Print();

  std::cout << "===== PATH =====" << std::endl;
  for(const std::shared_ptr<Node2D>& node: path_info.path) {
    std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  }

  std::cout << std::endl;
}

void RunDijkstra(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node) {
  
  std::cout << "============================================" << std::endl;
  std::cout << "============= RUNNING DIJKSTRA =============" << std::endl;
  std::cout << "============================================" << std::endl;

  // Run Dijkstra
  Dijkstra2D dijkstra;
  PathInfo path_info = dijkstra.Run(graph, start_node, end_node);

  // Display the solution
  Gui2D gui;
  gui.LoadOccupancyGrid(occupancy_grid);
  gui.LoadPath(path_info.path);
  gui.Display();

  // Print the solution
  path_info.details.Print();

  std::cout << "===== PATH =====" << std::endl;
  for(const std::shared_ptr<Node2D>& node: path_info.path) {
    std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  }

  std::cout << std::endl;
}

void RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node) {
  
  std::cout << "============================================" << std::endl;
  std::cout << "=============  RUNNING A Star  =============" << std::endl;
  std::cout << "============================================" << std::endl;

  // Run A*
  AStar2D a_star;
  PathInfo path_info = a_star.Run(graph, start_node, end_node);

  // Display the solution
  Gui2D gui;
  gui.LoadOccupancyGrid(occupancy_grid);
  gui.LoadPath(path_info.path);
  gui.Display();

  // Print the solution
  path_info.details.Print();

  std::cout << "=====  PATH   =====" << std::endl;
  for(const std::shared_ptr<Node2D>& node: path_info.path) {
    std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  }

  std::cout << std::endl;

}
