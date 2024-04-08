#include <vector>
#include <memory>

#include "graph.h"
#include "occupancy_grid2d.h"

/* This file contains example uses of the MediationLayer library. Examples
 * include: reading an occupancy grid file, constructing a graph, extracting
 * data from the graph, printing objects, and evaluating object equality.
 */

using namespace game_engine;

int main(int argc, char** argv) {
  if(argc != 2) {
    std::cerr << "Usage: ./examples ${occupancy_grid_file}" << std::endl;
    return EXIT_FAILURE;
  }

  // Load an occupancy grid from a file
  // There are two example grids at: data/labs/grid2d_*
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(argv[1]);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  // Define the start and end points
  // Create a node at (0,0) and a node at (4,4)
  const std::shared_ptr<Node2D> start_node = std::make_shared<Node2D>(Eigen::Vector2d(0,0));
  const std::shared_ptr<Node2D> end_node = std::make_shared<Node2D>(Eigen::Vector2d(4,4));

  // Access directed edges eminating from a node
  const std::vector<DirectedEdge2D> edges = graph.Edges(start_node);
  
  // Iterate through the list of edges
  for(const DirectedEdge2D& edge: edges) {
    const std::shared_ptr<Node2D>& source_ptr = edge.Source();
    const std::shared_ptr<Node2D>& sink_ptr = edge.Sink();
    const double cost = edge.Cost();

    // Print relevant data
    // std::shared_ptr<Node2D>->Data() is an Eigen::Vector2d. If you want to use
    // functions/accessors other than x() and y(), google Eigen::Vector2d
    std::cout 
      << "DirectedEdge2D from " 
      << "[" << source_ptr->Data().x() << ", " << source_ptr->Data().y() << "]"
      << " to "
      << "[" << sink_ptr->Data().x() << ", " << sink_ptr->Data().y() << "]"
      << " with cost " 
      << cost
      << std::endl;
  }

  // Access neighbors of a given node
  const std::vector<std::shared_ptr<Node2D>> neighbors = graph.Neighbors(start_node);

  // Iterate through the list of neighbors and print
  for(const std::shared_ptr<Node2D> neighbor: neighbors) {
    std::cout 
      << "[" << neighbor->Data().transpose() << "]"
      << " is a neighbor of "
      << "[" << start_node->Data().transpose() << "]"
      << std::endl;
  }

  // Check node equality
  std::cout << "Are the start and end nodes equal: " << (*start_node == *end_node) << std::endl;
  std::cout << "Is the start node equal to itself: " << (*start_node == *start_node) << std::endl;

  return EXIT_SUCCESS;
}
