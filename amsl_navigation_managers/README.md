# amsl_navigation_managers

## Environments
- Ubuntu 20.04
- ROS Noetic

## Requirements
- pyyaml

## node_edge_map_manager
- This node loads map data from yaml(sample at ./sample/map/sample_map.yaml), and publishes map data for path planning, localization, etc.
### published topics
  - /node_edge_map/map (amsl_navigation_msgs/NodeEdgeMap)
### subscribed topics
  - this node doesn't subscribe topics
### services
  - /node_edge_map/update_node (amsl_navigation_msgs/UpdateNode)
  - /node_edge_map/update_edge (amsl_navigation_msgs/UpdateNode)
### params
- HZ
  - loop rate (default: 10)
- MAP_PATH
  - ABSOLUTE path to map yaml file (default: $(find amsl_navigation_managers)/sample/map/sample_map.yaml)

## checkpoint_manager
- This node loads checkpoint(node) id from yaml.
### published topics
  - /node_edge_map/checkpoint (std_msgs/Int32MultiArray)
### subscribed topics
  - this node doesn't subscribe topics

## task_manager
- This node loads task list from yaml. This node sends commands to the nodes involved in the execution of each task.
