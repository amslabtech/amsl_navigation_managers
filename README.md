# amsl_navigation_managers

## node_edge_map_manager
- This node loads map data from yaml(sample at ./sample/map/sample_map.yaml), and publishes map data for path planning, localization, etc.

## checkpoint_manager
- This node loads checkpoint(node) id from yaml. 

## task_manager
- This node loads task list from yaml. This node sends commands to the nodes involved in the execution of each task.
