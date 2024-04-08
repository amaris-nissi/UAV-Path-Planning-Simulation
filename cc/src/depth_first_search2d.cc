#include <stack>
#include <iostream>
#include "depth_first_search2d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node and a cost to reach
    // that node.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;
      double cost;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    // Function to check if one node is part of a vector of nodes
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
    bool is_present(NodeWrapperPtr nwp, std::vector<NodeWrapperPtr> nwpVec){
      for(int j=0;j<nwpVec.size();j++){
        if( *(nwp) == *(nwpVec[j]) )
          return true;
      }
      return false;
    }
  }

  PathInfo DepthFirstSearch2D::Run(
      const Graph2D& graph, 
      const std::shared_ptr<Node2D> start_ptr, 
      const std::shared_ptr<Node2D> end_ptr)
  {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    ///////////////////////////////////////////////////////////////////
    // SETUP
    // DO NOT MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Timer timer;
    timer.Start();

    // Use these data structures
    std::stack<NodeWrapperPtr> to_explore;
    std::vector<NodeWrapperPtr> explored;
    
    // Creating the Path information in while loop to have access to the 
    // explored nodes and the last explored node.
    
    NodeWrapperPtr start_node = std::make_shared<NodeWrapper>(); 
    NodeWrapperPtr end_node = std::make_shared<NodeWrapper>(); 
    start_node->parent = nullptr; 
    start_node->node_ptr = start_ptr; 
    start_node->cost = 0; 
    end_node->parent = nullptr; 
    end_node->node_ptr = end_ptr; 
    end_node->cost = 0; 
    to_explore.push(start_node); 

    while(!to_explore.empty()){ 
      NodeWrapperPtr node_to_explore = to_explore.top(); 
      to_explore.pop(); 

      if(is_present(node_to_explore,explored)) 
        continue;

      if(*node_to_explore == *end_node){ 
        PathInfo path_info; 
        NodeWrapperPtr current_node = node_to_explore;  

        while(current_node){ 
          path_info.details.path_length++; 
          path_info.path.push_back(current_node->node_ptr); 
          current_node = current_node->parent; 
        };
        
        std::reverse(path_info.path.begin(),path_info.path.end()); 
        
        path_info.details.path_cost = node_to_explore->cost; 
        path_info.details.path_length = path_info.path.size(); 
        path_info.details.num_nodes_explored = explored.size(); 
        path_info.details.run_time = timer.Stop(); 
        return path_info; 

      }else{ 
        explored.push_back(node_to_explore); 
        auto edges = graph.Edges(node_to_explore->node_ptr); 
        for(auto edge : edges){ 
          NodeWrapperPtr neighbor = std::make_shared<NodeWrapper>(); 
          neighbor->parent = node_to_explore; 
          neighbor->node_ptr = edge.Sink() ; 
          neighbor->cost = node_to_explore->cost + edge.Cost(); 
          to_explore.push(neighbor); 
      };};
    };
  };
};