#include <queue>

#include "a_star2d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;

      // True cost to this node
      double cost;

      // Heuristic to end node
      double heuristic;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
    bool is_present(NodeWrapperPtr nwp, std::vector<NodeWrapperPtr> nwpVec){
      for(int j=0;j<nwpVec.size();j++){
        if( *(nwp) == *(nwpVec[j]) )
          return true;
      }
      return false;
    }

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper>& lhs, 
        const std::shared_ptr<NodeWrapper>& rhs) {
      return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
    }

    double Heuristic(
        const std::shared_ptr<Node2D>& current_ptr,
        const std::shared_ptr<Node2D>& end_ptr) {
        return std::sqrt( std::pow( end_ptr->Data()[0] - current_ptr->Data()[0] ,2.0 ) +
                          std::pow( end_ptr->Data()[1] - current_ptr->Data()[1] ,2.0 ) ); // Euclidean Distance, always underestimates
        //return std::abs(current_ptr->Data()[0]-end_ptr->Data()[0]) + 
        //       std::abs(current_ptr->Data()[1]-end_ptr->Data()[1]); // Manhattan Distance, always overestimates
        //return 0; // zero heuristic function
    }
  }

  PathInfo AStar2D::Run(
      const Graph2D& graph, 
      const std::shared_ptr<Node2D> start_ptr, 
      const std::shared_ptr<Node2D> end_ptr) {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    ///////////////////////////////////////////////////////////////////
    // SETUP
    // DO NOT MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Timer timer;
    timer.Start();

    // Use these data structures
    std::priority_queue<
      NodeWrapperPtr,
      std::vector<NodeWrapperPtr>,
      std::function<bool(
          const NodeWrapperPtr&, 
          const NodeWrapperPtr& )>> 
        to_explore(NodeWrapperPtrCompare);

    std::vector<NodeWrapperPtr> explored;

    // Creating the Path information in while loop to have access to the 
    // explored nodes and the last explored node.
    
    NodeWrapperPtr start_node = std::make_shared<NodeWrapper>(); // Define Start node
    NodeWrapperPtr end_node = std::make_shared<NodeWrapper>(); // Define End node
    start_node->parent = nullptr; // Initialize as null pointer
    start_node->node_ptr = start_ptr; // Initialize pointer to the given start pointer
    start_node->cost = 0; // Initialize cost as zero
    start_node->heuristic = Heuristic(start_ptr, end_ptr); // Call Heuristic Function
    end_node->node_ptr = end_ptr; // Initialize pointer to the given end pointer

    to_explore.push(start_node); // Push the first note to explore, start node
    
    while(!to_explore.empty()){ // Explore every node in the graph
      NodeWrapperPtr node_to_explore = to_explore.top(); // Exploring the top node in the stack
      to_explore.pop(); // Removing the top node of the stack

      if(is_present(node_to_explore,explored)) // Check if the the node has been explored before
        continue;

      if(*node_to_explore == *end_node){ // Check if the node to be explore is the last node
        PathInfo path_info; // Create a PathInfo
        NodeWrapperPtr current_node = node_to_explore;  // Create a current node
        path_info.details.num_nodes_explored = 0;
        path_info.details.path_length = 0;
        path_info.details.path_cost = 0;
        path_info.details.run_time = timer.Stop();
        path_info.path = {};

        while(current_node){ // while current node is true
          //path_info.details.path_length++; // Increse the size of path Vec by 1 for each iteration
          path_info.path.push_back(current_node->node_ptr); // Push back the current node
          current_node = current_node->parent; // set current node to be the previous node's parent
        };
        
        std::reverse(path_info.path.begin(),path_info.path.end()); // Reverse the order of the path vector
        
        path_info.details.path_cost = node_to_explore->cost; // Adds costs of pushed parent nodes
        path_info.details.path_length = path_info.path.size(); // Set the path Length
        path_info.details.num_nodes_explored = explored.size(); // Set the number of Nodes explored
        path_info.details.run_time = timer.Stop(); // Stop Run time

        if(start_node->heuristic > path_info.details.path_cost)
          std::cout << "Overestimated" << std::endl;
        else
          std::cout << "Underestimating" << std::endl;

        return path_info; // Ending the code and returning the path inforamtion

      }else{ // If it's not the last node or not explored before
        explored.push_back(node_to_explore); // Push the node to explored stack
        auto edges = graph.Edges(node_to_explore->node_ptr); //Define edges, Edges' vector of edges that are eminating from node
        for(auto edge : edges){ // Automatically asign the data type of edge. For loop every edge in the Edges vector
          NodeWrapperPtr neighbor = std::make_shared<NodeWrapper>(); // Define every iteration, memory location doesn't change
          neighbor->parent = node_to_explore; // Define the Parent of the neighbor to be the current node
          neighbor->node_ptr = edge.Sink() ; // Define the node pointer of the neighbor as the "neighbor node"
          neighbor->cost = node_to_explore->cost + edge.Cost(); // Define the cost of reaching that neighnor
          neighbor->heuristic = Heuristic(neighbor->node_ptr, end_ptr); // Call Heuristic Function
          to_explore.push(neighbor); // Push neighbor so it can be explored the same way
      };};
    };
  };
};