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
import copy

import rospy
import tf
from std_msgs.msg import ColorRGBA, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')

        self.map = NodeEdgeMap()
        self.map_sub = rospy.Subscriber('/node_edge_map/map', NodeEdgeMap, self.map_callback)

        self.lock = threading.Lock()

        print '=== task manager ==='

    def process(self):
        r = rospy.Rate(10)

        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.TASK_LIST_PATH)

        while not rospy.is_shutdown():
            for file in os.listdir(dir_name):
                if 0 is file.find('.'):
                    continue
                file_timestamp = os.stat(dir_name+'/'+file)[ST_MTIME]
                if timestamp < file_timestamp:
                    timestamp = file_timestamp
                    try:
                        self.task_data = self.load_task_from_yaml()
                        print 'task updated!'
                    except:
                        print 'failed to update task'
            r.sleep()

    def load_task_from_yaml(self):
        with open(self.TASK_LIST_PATH) as file:
            task_data = yaml.load(file)
        return task_data

    def map_callback(self, node_edge_map):
        self.map = node_edge_map

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
