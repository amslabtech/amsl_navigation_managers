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
import subprocess

import rospy
import tf
from std_msgs.msg import ColorRGBA, Int32MultiArray, Bool, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap
from amsl_navigation_msgs.srv import UpdateEdge, Replan

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

        self.map = None
        self.estimated_pose = Odometry()
        self.estimated_edge = Edge()
        self.goal_flag = Empty()
        self.map_sub = rospy.Subscriber('/node_edge_map/map', NodeEdgeMap, self.map_callback)
        self.pose_sub = rospy.Subscriber('/estimated_pose/pose', Odometry, self.pose_callback)
        self.edge_sub = rospy.Subscriber('/estimated_pose/edge', Edge, self.edge_callback)
        self.goal_flag_sub = rospy.Subscriber('/node_edge_navigator/goal_flag', Empty, self.goal_flag_callback)

        self.stop_pub = rospy.Publisher('/task/stop', Bool, queue_size=1)
        self.stop_line_sub = rospy.Subscriber('/recognition/stop_line', Bool, self.stop_line_callback)
        self.closed_sign_sub = rospy.Subscriber('/recognition/closed_sign', Bool, self.closed_sign_callback)

        self.line_detected = False
        self.road_closed = False
        self.first_park_flag = True
        self.process_terminated = False

        self.subprocess1 = "road_closed_sign_detector"
        self.lock = threading.Lock()
        self.task_data = self.load_task_from_yaml()

    def process(self):
        r = rospy.Rate(10)
        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.TASK_LIST_PATH)
        while not rospy.is_shutdown():
            # print '=== task manager ==='
            if self.map is not None:
                # print("node id :{}".format(self.map.nodes[self.estimated_edge.node0_id].id))
                if (self.map.nodes[self.estimated_edge.node0_id].label == 'park'):
                    if self.first_park_flag:
                        print("----- booting %s -----" % self.subprocess1)
                        cmd = "roslaunch %s %s.launch" % (self.subprocess1, self.subprocess1)
                        p1 = subprocess.Popen(cmd.split())
                        self.first_park_flag = False
                    if not self.process_terminated:
                        if self.road_closed:
                            self.set_impassable_edge(self.estimated_edge)
                            rospy.sleep(0.1)
                            self.request_replan()
                            self.road_closed = False
                else:
                    if (not self.first_park_flag and not self.process_terminated):
                        print("killing %s" % self.subprocess1)
                        node1 = "/navigation_managers/%s/%s" % (self.subprocess1, self.subprocess1)
                        self.kill_node(node1)

            for count, task in enumerate(self.task_data['task']):
                # pprint(task)
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
                                    self.line_detected = False
                                    task['performed'] = True

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

    def kill_node(self, nodename):
        node_list_cmd = "rosnode list"
        ros_node_process = subprocess.Popen(node_list_cmd.split(), stdout=subprocess.PIPE)
        ros_node_process.wait()
        nodetuple = ros_node_process.communicate()
        nodelist = nodetuple[0]
        nodelist = nodelist.split("\n")
        for nd in nodelist:
            idx = nd.find(nodename)
            if idx==0:
                kill_cmd = "rosnode kill %s" % nd
                subprocess.call(kill_cmd.split())
                self.process_terminated = True
                print("success to kill the process")
                return
        print("failed to kill the process")

    def set_impassable_edge(self, edge):
        rospy.wait_for_service('/node_edge_map/update_edge')
        try:
            client = rospy.ServiceProxy('/node_edge_map/update_edge', UpdateEdge)
            req = UpdateEdge()
            req.edge = edge
            req.edge.impassable = True
            req.operation = 0
            res = client(req.edge, req.operation)
            if res.succeeded:
                print "success to update the edge"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def request_replan(self):
        rospy.wait_for_service('/global_path/replan')
        try:
            client = rospy.ServiceProxy('/global_path/replan', Replan)
            req = Replan()
            req.edge = self.estimated_edge
            res = client(req.edge)
            if res.succeeded:
                print "success to request replan" 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

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

    def closed_sign_callback(self, detection):
        self.road_closed = detection.data

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
