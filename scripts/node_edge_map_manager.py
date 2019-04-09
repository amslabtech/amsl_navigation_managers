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

        self.marker_pub = rospy.Publisher('/node_edge_map/viz', MarkerArray, queue_size=1)
        self.lock = threading.Lock()

        print '=== node edge map manager ==='

    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()


    def landmark_callback(self, lm):
        with self.lock:
            pass

if __name__ == '__main__':
    node_edge_map_manager = NodeEdgeMapManager()
    node_edge_map_manager.process()
