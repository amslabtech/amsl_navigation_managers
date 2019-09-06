#include "amsl_navigation_managers/node_edge_map_interface.h"

NodeEdgeMapInterface::NodeEdgeMapInterface(void)
{

}

void NodeEdgeMapInterface::set_map(amsl_navigation_msgs::NodeEdgeMap& _map)
{
    map = _map;
}

void NodeEdgeMapInterface::get_node_from_id(int id, amsl_navigation_msgs::Node& node)
{
    for(const auto n : map.nodes){
        if(n.id == id){
            node = n;
            return;
        }
    }
}

void NodeEdgeMapInterface::get_edge_from_node_id(int node0_id, int node1_id, amsl_navigation_msgs::Edge& edge)
{
    for(const auto e : map.edges){
        if(e.node0_id == node0_id && e.node1_id == node1_id){
            edge = e;
            return;
        }
    }
}

int NodeEdgeMapInterface::get_edge_index_from_node_id(int node0_id, int node1_id)
{
    int index = 0;
    for(const auto e : map.edges){
        if(e.node0_id == node0_id && e.node1_id == node1_id){
            return index;
        }
        index++;
    }
}

int NodeEdgeMapInterface::get_node_index_from_id(int id)
{
    int i = 0;
    for(const auto n : map.nodes){
        if(n.id == id){
            return i;
        }
        i++;
    }
    return -1;
}

amsl_navigation_msgs::Edge NodeEdgeMapInterface::get_edge_from_index(int edge_index)
{
    return map.edges[edge_index];
}

std::string NodeEdgeMapInterface::get_map_header_frame_id(void)
{
    return map.header.frame_id;
}

int NodeEdgeMapInterface::get_edge_num(void)
{
    return map.edges.size();
}
