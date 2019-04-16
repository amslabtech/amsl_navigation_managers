# amsl_navigation_msgs

## messages

### amsl_navigation_msgs/Edge
- float32 distance
  - Distance between node0 and node1 ([m])
- float32 progress
  - Progress rate from node0 to node1 (no unit)
- int32 node0_id
- int32 node1_id

### amsl_navigation_msgs/Node
- int32 id
  - Node id (>=0)
- string type
  - Property for node recognition. "intersection", "landmark", etc.
- string label
  - Name of node. "start", "goal", etc.
- geometry_msgs/Point
  - Position of node
  

### amsl_navigation_msgs/NodeEdgeMap
- std_msgs/Header header
- amsl_navigation_msgs/Node[] nodes
- amsl_navigation_msgs/Edge[] edges

## services

### amsl_navigation_msgs/UpdateNode
- request
  - uint8 ADD=0
  - uint8 MODIFY=0
  - uint8 DELETE=1
  - uitn8 operation
    - set one of the above three constants
  - amsl_navigation_msgs/Node node
- response
  - bool succeeded
  
### amsl_navigation_msgs/UpdateEdge
- request
  - uint8 ADD=0
  - uint8 MODIFY=0
  - uint8 DELETE=1
  - uitn8 operation
    - set one of the above three constants
  - amsl_navigation_msgs/Edge edge
- response
  - bool succeeded
  
### amsl_navigation_msgs/UpdateCheckpoint
- request
  - uint8 ADD=0
  - uint8 DELETE=1
  - uitn8 operation
    - set one of the above two constants
  - int32 id
- response
  - bool succeeded
  
### amsl_navigation_msgs/RunStop
- request
  - bool RUN=1
  - bool STOP=0
  - uitn8 operation
    - set one of the above two constants
- response
  - bool succeeded
