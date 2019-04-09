#!/usr/bin/env python
#! coding:utf-8

import numpy as np
import math
import threading
import yaml
import pprint

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

np.set_printoptions(linewidth=200)

class NodeEdgeMapManager:
    def __init__(self):
        rospy.init_node('node_edge_map_manager')

        MAP_PATH = rospy.get_param('~MAP_PATH')
        with open(MAP_PATH) as file:
            self.map_data = yaml.load(file)

        pprint.pprint(self.map_data)

        self.node_marker = MarkerArray()
        self.node_marker_pub = rospy.Publisher('/node_edge_map/node/viz', MarkerArray, queue_size=1, latch=True)
        self.lock = threading.Lock()

        print '=== node edge map manager ==='

    def process(self):
        r = rospy.Rate(10)
        self.make_node_marker()
        while not rospy.is_shutdown():
            self.node_marker_pub.publish(self.node_marker)
            r.sleep()

    def make_node_marker(self):
        self.node_marker = MarkerArray()
        time = rospy.get_rostime()
        for node in self.map_data['NODE']:
            n = Marker()
            n.header.frame_id = self.map_data['MAP_FRAME']
            n.header.stamp = time
            n.id = node['id']
            n.action = Marker().ADD
            n.type = Marker().CUBE
            n.lifetime = rospy.Duration()
            n.scale.x = 0.5
            n.scale.y = 0.5
            n.scale.z = 0.5
            n.color.r = 0.0
            n.color.g = 1.0
            n.color.b = 0.0
            n.color.a = 0.7
            n.pose.position.x = node['point']['x']
            n.pose.position.y = node['point']['y']
            self.node_marker.markers.append(n)
            print node

if __name__ == '__main__':
    node_edge_map_manager = NodeEdgeMapManager()
    node_edge_map_manager.process()
