// Author: Tucker Haydon

#include <stack>

#include "depth_first_search2d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here

  namespace {
    // Helper struct. Functions as a linked list with data. The linked list
    // represents a path. Data contained includes a node and a cost to reach
    // that node. 
    struct NodeWrapper {
      std::shared_ptr<NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;
      double cost;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };                                         
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
    // Put the function
    bool is_present
    (std::vector<NodeWrapperPtr> explored, NodeWrapperPtr node_to_explore)
      {
        for(auto node:explored)
        {
          if(*node == *node_to_explore)
            return true;
        }
        return false;
      }


  }



  PathInfo DepthFirstSearch2D::Run(
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
    std::stack<NodeWrapperPtr> to_explore;
    std::vector<NodeWrapperPtr> explored;

    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES HERE
    // SOME EXAMPLE CODE INCLUDED BELOW
    ///////////////////////////////////////////////////////////////////

    // Create a NodeWrapperPtr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
    to_explore.push(nw_ptr);  
    NodeWrapperPtr node_to_explore;

    PathInfo path_info;

    int counter = 0;

    while(counter < 2000)
    {  
      counter++;
      node_to_explore = to_explore.top();
      // std::cout << "\n What is in the node_to_explore. " << to_explore.top()->node_ptr->Data().transpose();
      to_explore.pop();
      //std::cout << "\n Popped node " << to_explore.top()->node_ptr->Data().transpose();
      if(is_present(explored, node_to_explore))
      {
          continue;
      }
      if(*(node_to_explore->node_ptr) == *(end_ptr)) //end_ptr = end_node
      {
        path_info.details.num_nodes_explored = counter;
        //path_info.details.path_length = 0;
        path_info.details.path_cost = node_to_explore->cost;
        path_info.details.run_time = timer.Stop();
        path_info.path.insert(path_info.path.begin(), node_to_explore->node_ptr);
          int pathlength = 1;
        while (!(*(node_to_explore->node_ptr) == *(start_ptr))) {
          pathlength++;
          node_to_explore = node_to_explore->parent;
          path_info.path.insert(path_info.path.begin(), node_to_explore->node_ptr);
        }
        path_info.details.path_length = pathlength;
        break; //breaks out of while loop
      } 

	    
      else
      {
         // std::cout << "\n Testing 2";
         explored.push_back(node_to_explore);  //node_to_explore_ptr
         for(const DirectedEdge2D &this_Edge: graph.Edges(node_to_explore->node_ptr))
         {
             //std::cout << "\n What has been pushed back " << explored.back()->node_ptr->Data().transpose();
             NodeWrapperPtr temp = std::make_shared<NodeWrapper>(); //std::make_shared declares; temp is the name of the pointer that we made.
             temp->node_ptr = this_Edge.Sink(); // Tells me where the sink is.
             temp->cost = node_to_explore->cost + this_Edge.Cost();// node_to_explore_ptr [86%]  // Built target student_autonomy_protocol
             temp->parent = node_to_explore; // node_to_explore_ptr
            to_explore.push(temp);
         }
      
      }
       
     }
    // Create a PathInfo
      return path_info;  
   }

 }  

