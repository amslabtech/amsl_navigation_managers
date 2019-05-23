#ifndef __NODE_EDGE_MAP_INTERFACE_H
#define __NODE_EDGE_MAP_INTERFACE_H

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"

class NodeEdgeMapInterface
{
public:
	NodeEdgeMapInterface(void);

	void set_map(amsl_navigation_msgs::NodeEdgeMap&);
	void get_node_from_id(int id, amsl_navigation_msgs::Node&);
	void get_edge_from_node_id(int, int, amsl_navigation_msgs::Edge&);
	int get_node_index_from_id(int);
	int get_edge_index_from_node_id(int, int);
	amsl_navigation_msgs::Edge get_edge_from_index(int);

private:
	amsl_navigation_msgs::NodeEdgeMap map;
};

#endif// __NODE_EDGE_MAP_INTERFACE_H
