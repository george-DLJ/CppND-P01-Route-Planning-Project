#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance((*end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* neighbor_node : current_node->neighbors){
        neighbor_node->parent = current_node;
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value + current_node->distance((*neighbor_node));  //value of pointer (Node)
        open_list.emplace_back(neighbor_node);
        neighbor_node->visited = true;   
    }
}

/**
 * Helper STL function for std::sort 
 *Compares the f values of two nodes to sort nodes, being f = g + h  
 */
bool Compare( const RouteModel::Node * a, const RouteModel::Node * b ) {
    return (a->g_value + a->h_value) > (b->g_value + b->h_value);
} 

/**
 * This function sorts RouteMolde nodes by its f(h + g) value in decending order 
 * and returns the node with lowest  value.
 * There are two alternatives to sort the list:
 *   Alternative 1: using STL function as Compare (above)
 *                  in this case sort should look like this:
 *                   std::sort(open_list.begin(), open_list.end(), Compare);  
 *   Alternative 2: use an inline Lambda function instead of the STL Helper function.
 */
 RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), 
        []( const RouteModel::Node * a, const RouteModel::Node * b ) { 
            return (a->g_value + a->h_value) > (b->g_value + b->h_value);
        });
    RouteModel::Node* p_nextNode = open_list.back(); //lowest sum f(h+g) node
    open_list.pop_back();
    return p_nextNode;
}

/**
 * This method takes the current (final) node as an argument and iteratively follow the
 * chain of parents of nodes until the starting node is found.
 *  - For each node in the chain, add the distance from the node to its parent to the distance variable.
 *  - The returned vector is in the correct order: the start node should be the first element
 *    of the vector, the end node should be the last element.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // NOTE: Check if final node(current) is the same as start node
    //     Then: path has only one node and distance is 0.
    if(current_node == start_node){ 
        path_found.push_back((*current_node));
        return path_found;
    }

    RouteModel::Node* node = current_node; 
    while(node != start_node){ //NOTE: parent node is not start and not nullptr
        distance += node->distance((*node->parent));
        path_found.push_back((*node)); // NOTE:push back node object element; the vector will be rotated afterwards.
        node = node->parent; 
    }
    path_found.push_back((*start_node));
    // NOTE:change vector to correct order:
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); 
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->h_value = CalculateHValue(start_node);
    start_node->visited = true;
    open_list.push_back(start_node);
    while(open_list.size() > 0) {
        current_node = NextNode();
        if(current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}