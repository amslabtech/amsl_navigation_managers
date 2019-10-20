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
from pprint import pprint

import rospy
import tf
from std_msgs.msg import ColorRGBA, Int32MultiArray, Bool, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap

class Task:
    def __init__(self):
        self.trigger = None
        self.task_type = None
        self.node0_id = None
        self.node1_id = None
        self.progress_min = None
        self.progress_max = None
        self.repeat = None

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')

        self.map = NodeEdgeMap()
        self.estimated_pose = Odometry()
        self.estimated_edge = Edge()
        self.goal_flag = Empty()
        self.map_sub = rospy.Subscriber('/node_edge_map/map', NodeEdgeMap, self.map_callback)
        self.pose_sub = rospy.Subscriber('/estimated_pose/pose', Odometry, self.pose_callback)
        self.edge_sub = rospy.Subscriber('/estimated_pose/edge', Edge, self.edge_callback)
        self.goal_flag_sub = rospy.Subscriber('/node_edge_navigator/goal_flag', Empty, self.goal_flag_callback)

        self.stop_pub = rospy.Publisher('/task/stop', Bool, queue_size=1)
        self.stop_line_sub = rospy.Subscriber('/recognition/stop_line', Bool, self.stop_line_callback)
        self.line_detected = False

        self.lock = threading.Lock()

        self.task_data = self.load_task_from_yaml()

    def process(self):
        r = rospy.Rate(10)

        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.TASK_LIST_PATH)

        while not rospy.is_shutdown():
            print '=== task manager ==='
            count = 0
            for task in self.task_data['task']:
                pprint(task)
                if (task['edge']['node0_id'] == self.estimated_edge.node0_id) and (task['edge']['node1_id'] == self.estimated_edge.node1_id):
                    print "task ", count, " is related to this edge"
                    if (task['edge']['progress_min'] < self.estimated_edge.progress) and (self.estimated_edge.progress < task['edge']['progress_max']):
                        print "task ", count, " is enabled"
                        if task['trigger'] == 'recognition/stop_line':
                            if self.line_detected:
                                if 'performed' in task:
                                    pass
                                else:
                                    print "task is performed"
                                    self.stop_pub.publish(Bool(self.line_detected))
                                    task['performed'] = True
                count += 1


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
        pprint(task_data)
        return task_data

    def map_callback(self, node_edge_map):
        self.map = node_edge_map

    def pose_callback(self, pose):
        self.estimated_pose = pose

    def edge_callback(self, edge):
        self.estimated_edge = edge

    def goal_flag_callback(self, goal_flag):
        self.goal_flag = goal_flag
        self.stop_pub.publish(Bool(True))

    def stop_line_callback(self, detection):
        self.line_detected = detection.data

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
