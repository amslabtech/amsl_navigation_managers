#! /usr/bin/env python3

import rospy
import yaml

from std_msgs.msg import Int32MultiArray, Int32
from amsl_navigation_msgs.msg import NodeEdgeMap, Node, Edge , Road
from geometry_msgs.msg import PoseWithCovarianceStamped ,Point

class RoadPublisher:
    def __init__(self):
        rospy.init_node('road_publisher')
        print('=== road_publisher ===')

        ## check params
        if rospy.has_param('~WIDTH_YAML'):
            self.width_yaml = rospy.get_param('~WIDTH_YAML')
        if rospy.has_param('~CHECKPOINT_YAML'):
            self.checkpoint_yaml = rospy.get_param('~CHECKPOINT_YAML')
        if rospy.has_param('~HZ'):
            self.hz = rospy.get_param('~HZ')

        ## load yaml
        self.width_yaml = self.load_width_yaml()
        self.checkpoint_yaml = self.load_checkpoint_yaml()

        ## make lists
        self.make_checkpoint_list()
        self.make_width_list()

        ## check array size
        if len(self.checkpoint_list) != len(self.width_list):
            print('!==width_yaml and checkpoint_yaml size is not same==!')
            exit()

        self.node_edge_map = NodeEdgeMap()
        self.road = Road()
        self.current_edge = Edge()
        self.current_checkpoint = 0
        self.next_checkpoint = 0
        self.current_node_point = Point()
        self.next_node_point = Point()
        self.current_width = 0
        self.closed_checkpoint_num = -1
        self.old_checkpoint = 0

        ## subscriber
        self.node_edge_map_sub = rospy.Subscriber("/node_edge_map/map", NodeEdgeMap, self.node_edge_map_callback)
        # self.current_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.current_pose_callback)
        self.current_checkpoint_sub = rospy.Subscriber("/current_checkpoint", Int32, self.current_checkpoint_callback)

        ## publisher
        self.road_pub = rospy.Publisher("/road", Road, queue_size=1)

    ## callback
    def node_edge_map_callback(self, msg):
        self.node_edge_map = msg
        print('get_node_edge_map')

    # def current_pose_callback(self, msg):
        # self.current_pose = msg

    def current_checkpoint_callback(self, msg):
        self.old_checkpoint = self.current_checkpoint
        self.current_checkpoint = msg.data
        if self.old_checkpoint != self.current_checkpoint:
            self.closed_checkpoint_num += 1
            print('closed_checkpoint_num', self.closed_checkpoint_num)

    ## load yaml
    def load_width_yaml(self):
        with open(self.width_yaml, 'r') as file:
            width_yaml = yaml.safe_load(file)
        return width_yaml

    def load_checkpoint_yaml(self):
        with open(self.checkpoint_yaml, 'r') as file:
            checkpoint_yaml = yaml.safe_load(file)
        return checkpoint_yaml

    def make_checkpoint_list(self):
        self.checkpoint_list = []
        for i in self.checkpoint_yaml['checkpoints']:
            self.checkpoint_list.append(i)

    def make_width_list(self):
        self.width_list = []
        for i in self.width_yaml['widths']:
            self.width_list.append(i)


    def get_next_checkpoint(self):
        for i in range(self.closed_checkpoint_num,len(self.checkpoint_list)):
            if i == len(self.checkpoint_list)-1:
                break

            if self.checkpoint_list[i] == self.current_checkpoint:
                self.current_width = self.width_list[i]
                self.next_checkpoint = self.checkpoint_list[i+1]
                print('current_width', self.current_width)
                return 1
        return 0


    def get_current_edge(self):
        is_get_next_checkpoint = self.get_next_checkpoint()
        edge = Edge()
        if is_get_next_checkpoint == 1:
            for edge in self.node_edge_map.edges:
                if edge.node0_id == self.current_checkpoint and edge.node1_id == self.next_checkpoint:
                    print('node0', edge.node0_id)
                    print('node1', edge.node1_id)
                    self.current_edge = edge
                    node0_index = self.get_index_from_id(edge.node0_id)
                    node1_index = self.get_index_from_id(edge.node1_id)
                    self.current_node_point = self.node_edge_map.nodes[node0_index].point
                    self.next_node_point = self.node_edge_map.nodes[node1_index].point
                    return 1
            return 0
        else:
            return -1

    def get_index_from_id(self, id):
        index = 0
        for node in self.node_edge_map.nodes:
            if node.id == id:
                return index
            index += 1
        return -1

    def pub_road(self):
        is_get_current_edge = self.get_current_edge()
        if is_get_current_edge == 1:
            self.road.point0 = self.current_node_point
            self.road.point1 = self.next_node_point
            self.road.width = self.current_width
            self.road.direction = self.current_edge.direction
            self.road_pub.publish(self.road)
        elif is_get_current_edge == 0:
            print('!==not found edge==!')
        elif is_get_current_edge == -1 and self.closed_checkpoint_num >= 1:
            print ('!===GOOOOOOAL===!')
            exit()
        else:
            print('!==not found next checkpoint==!')
        print ('')

    def main(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.pub_road()
            rate.sleep()

if __name__ == '__main__':
    road_publisher = RoadPublisher()
    road_publisher.main()
