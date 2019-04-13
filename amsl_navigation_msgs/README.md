# amsl_navigation_msgs

## amsl_navigation_msgs/Edge
- float32 distance
  - Distance between node0 and node1 ([m])
- float32 progress
  - Progress rate from node0 to node1 (no unit)
- int32 node0_id
- int32 node1_id

## amsl_navigation_msgs/Node
- int32 id
  - Node id (>=0)
- string type
  - Property for node recognition. "intersection", "landmark", etc.
- string label
  - Name of node. "start", "goal", etc.
- geometry_msgs/Point
  - Position of node
  

## amsl_navigation_msgs/NodeEdgeMap
- std_msgs/Header header
- amsl_navigation_msgs/Node[] nodes
- amsl_navigation_msgs/Edge[] edges
