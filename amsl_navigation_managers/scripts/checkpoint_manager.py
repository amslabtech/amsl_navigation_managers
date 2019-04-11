#!/usr/bin/env python
#! coding:utf-8

import numpy as np
import math
import threading
import yaml
import pprint
import os
import time
import datetime
from stat import *

import rospy
import tf
from std_msgs.msg import ColorRGBA, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap

class CheckpointManager:
    def __init__(self):
        rospy.init_node('checkpoint_manager')

        self.CHECKPOINT_PATH = rospy.get_param('~CHECKPOINT_PATH')

        self.cp_data = self.load_cp_from_yaml()
        pprint.pprint(self.cp_data)

        self.cp_marker = MarkerArray()
        self.cp_marker_pub = rospy.Publisher('/node_edge_map/viz/node/checkpoint', MarkerArray, queue_size=1, latch=True)

        self.checkpoint_list = Int32MultiArray()
        self.checkpoint_list_pub = rospy.Publisher('/node_edge_map/checkpoint', Int32MultiArray, queue_size=1, latch=True)

        self.node_markers = MarkerArray()
        self.node_sub = rospy.Subscriber('/node_edge_map/viz/node', MarkerArray, self.node_callback)

        self.lock = threading.Lock()

        print '=== checkpoint manager ==='

    def process(self):
        r = rospy.Rate(10)

        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.CHECKPOINT_PATH)

        while not rospy.is_shutdown():
            self.make_and_publish_checkpoint()
            for file in os.listdir(dir_name):
                if 0 is file.find('.'):
                    continue
                file_timestamp = os.stat(dir_name+'/'+file)[ST_MTIME]
                if timestamp < file_timestamp:
                    timestamp = file_timestamp
                    try:
                        self.cp_data = self.load_cp_from_yaml()
                        print 'checkpoint updated!'
                    except:
                        print 'failed to update checkpoint'
            r.sleep()

    def load_cp_from_yaml(self):
        with open(self.CHECKPOINT_PATH) as file:
            cp_data = yaml.load(file)
        return cp_data

    def make_and_publish_checkpoint(self):
        self.make_checkpoint()
        self.checkpoint_list_pub.publish(self.checkpoint_list)
        self.cp_marker_pub.publish(self.cp_marker)

    def make_checkpoint(self):
        self.checkpoint_list = Int32MultiArray()
        for cp in self.cp_data['checkpoints']:
            self.checkpoint_list.data.append(cp)

    def node_callback(self, node):
        with self.lock:
            self.node_markers = node

        self.cp_marker = MarkerArray()
        for cp in self.checkpoint_list.data:
            for node in self.node_markers.markers:
                if cp == node.id:
                    self.cp_marker.markers.append(node)
                    break
        for cp in self.cp_marker.markers:
            self.set_marker_rgb(cp, 1.0, 0.0, 0.0)
            
    def set_marker_scale(self, marker, x, y, z):
        marker.scale.x = x
        marker.scale.y = y
        marker.scale.z = z

    def set_marker_position(self, marker, x, y, z):
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

    def set_marker_rgb(self, marker, r, g, b, a=0.7):
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

if __name__ == '__main__':
    checkpoint_manager = CheckpointManager()
    checkpoint_manager.process()
