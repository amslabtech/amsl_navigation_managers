#!/usr/bin/env python
#! coding:utf-8

import numpy as np
import math
import threading
import yaml
import os
import time
import datetime
from stat import *
import copy
from pprint import pprint
import subprocess

import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion, rotation_matrix
from std_msgs.msg import ColorRGBA, Int32MultiArray, Bool, Empty
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap, StopLine
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
        self.LINE_DIST_THRESHOLD = rospy.get_param('LINE_DIST_THRESHOLD', 3.0)
        self.ROBOT_FRAME = rospy.get_param('ROBOT_FRAME', "base_link")
        self.REST_TIME = rospy.get_param('REST_TIME', 0.0)

        self.map = None
        self.line_detected_pose = None
        self.line_info = StopLine()
        self.odom = Odometry()
        self.estimated_pose = Odometry()
        self.estimated_edge = Edge()
        self.goal_flag = Empty()
        self.map_sub = rospy.Subscriber('/node_edge_map/map', NodeEdgeMap, self.map_callback)
        self.pose_sub = rospy.Subscriber('/estimated_pose/pose', Odometry, self.pose_callback)
        self.edge_sub = rospy.Subscriber('/estimated_pose/edge', Edge, self.edge_callback)
        self.odom_sub = rospy.Subscriber('/odom/complement', Odometry, self.odom_callback)
        self.goal_flag_sub = rospy.Subscriber('/node_edge_navigator/goal_flag', Empty, self.goal_flag_callback)

        self.target_velocity_pub = rospy.Publisher('/target_velocity', Twist, queue_size=1)
        self.stop_pub = rospy.Publisher('/task/stop', Bool, queue_size=1)
        self.ignore_pub = rospy.Publisher('/task/ignore_intensity', Bool, queue_size=1)
        self.grassy_pub = rospy.Publisher('/task/grassy', Bool, queue_size=1)
        self.local_goal_pub = rospy.Publisher('/local_goal', PoseStamped, queue_size=1)
        self.stop_line_sub = rospy.Subscriber('/recognition/stop_line', StopLine, self.stop_line_callback)
        self.closed_sign_sub = rospy.Subscriber('/recognition/closed_sign', Bool, self.closed_sign_callback)

        self.line_detected = False
        self.road_closed = False
        self.first_park_flag = True
        self.t_flag = False
        self.process_terminated = False
        self.ignore_intensity_flag = False
        self.grassy_flag = False
        self.is_line_trace = False

        self.subprocess1 = "road_closed_sign_detector"
        self.lock = threading.Lock()
        self.task_data = self.load_task_from_yaml()

    def process(self):
        r = rospy.Rate(10)
        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.TASK_LIST_PATH)
        while not rospy.is_shutdown():
            # print('=== task manager ===')
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
                    # print("task ", count, " is related to this edge")
                    if (task['edge']['progress_min'] < self.estimated_edge.progress) and (self.estimated_edge.progress < task['edge']['progress_max']):
                        # print("task ", count, " is enabled")
                        if task['trigger'] == 'enter/edge':
                            if task['task_type'] == "ignore_intensity" :
                                if not self.ignore_intensity_flag:
                                    self.ignore_intensity_flag = True
                                    self.ignore_pub.publish(Bool(self.ignore_intensity_flag))
                            elif task['task_type'] == "use_intensity" :
                                if self.ignore_intensity_flag:
                                    self.ignore_intensity_flag = False
                                    self.ignore_pub.publish(Bool(self.ignore_intensity_flag))
                            if task['task_type'] == "grassy_area" :
                                if not self.grassy_flag:
                                    self.grassy_flag = True
                                    self.grassy_pub.publish(Bool(self.grassy_flag))
                            elif task['task_type'] == "not_grassy_area" :
                                if self.grassy_flag:
                                    self.grassy_flag = False
                                    self.grassy_pub.publish(Bool(self.grassy_flag))
                        elif task['trigger'] == 'recognition/stop_line':
                            if self.line_detected:
                                line_angle = self.line_info.angle
                                line_direction = self.calc_line_direction(line_angle)
                                # print("direction :{} angle :{}".format(line_direction, line_angle))
                                if (line_direction > math.pi*0.25 and line_direction < math.pi*0.75) or self.t_flag:
                                    line_dist = self.calc_line_dist()
                                    if line_dist > self.LINE_DIST_THRESHOLD:
                                        # print("line dist :{}".format(line_dist))
                                        self.line_detected_pose = self.odom
                                        if 'performed' in task:
                                            if task['repeat']:
                                                print("task is performed")
                                                rospy.sleep(self.REST_TIME)
                                                self.stop_pub.publish(Bool(self.line_detected))
                                                self.line_detected = False
                                                if self.t_flag:
                                                    self.interrupt_local_goal(False)
                                                    self.publish_target_velocity(0.6)
                                                    self.t_flag = False
                                            else:
                                               pass
                                        else:
                                            if task['after_t']:
                                                if self.t_flag:
                                                    print("task is performed")
                                                    rospy.sleep(self.REST_TIME)
                                                    self.stop_pub.publish(Bool(self.line_detected))
                                                    self.line_detected = False
                                                    self.interrupt_local_goal(False)
                                                    self.publish_target_velocity(0.8)
                                                    self.t_flag = False
                                                    task['performed'] = True
                                            else:
                                                print("task is performed")
                                                rospy.sleep(self.REST_TIME)
                                                self.stop_pub.publish(Bool(self.line_detected))
                                                self.line_detected = False
                                                task['performed'] = True

                        elif task['trigger'] == 'recognition/stop_line/T':
                            if self.line_detected:
                                if self.is_line_trace is True:
                                    line_angle = self.line_info.angle
                                    line_direction = self.calc_line_direction(line_angle)
                                    if (line_direction > math.pi*0.25 and line_direction < math.pi*0.75):
                                        # if self.line_info.is_t_shape:
                                        if 'performed' in task:
                                            pass
                                        else:
                                            print("task is performed")
                                            abs_local_goal = np.array((task['local_goal']['x'], task['local_goal']['y'], 0))
                                            if abs(line_angle) > math.pi*0.75:
                                                line_angle += -np.sign(line_angle) * math.pi
                                            Rz = rotation_matrix(-line_angle, (0,0,1))[:3,:3]
                                            rel_local_goal = Rz.dot(abs_local_goal)
                                            # print("line angle :{}".format(line_angle))
                                            # print("absolute local goal :{}\nrelative local goal :{}".format(abs_local_goal, rel_local_goal))
                                            rospy.sleep(self.REST_TIME)
                                            self.stop_pub.publish(Bool(self.line_detected))
                                            self.interrupt_local_goal(True)
                                            rospy.sleep(0.1)
                                            self.publish_local_goal(rel_local_goal[:2], line_angle)
                                            self.line_detected_pose = self.odom
                                            self.publish_target_velocity(0.5)
                                            self.line_detected = False
                                            self.t_flag = True
                                            self.is_line_trace = False
                                            task['performed'] = True
                        elif task['trigger'] == 'recognition/stop_line/line_trace':
                            if self.line_detected and  not self.t_flag:
                                if self.line_info.center_point.x > 0.1:
                                    print("task is performed")
                                    self.interrupt_local_goal(True)
                                    rospy.sleep(0.1)
                                    rel_local_goal = np.array((self.line_info.center_point.x, self.line_info.center_point.y, 0))
                                    self.publish_local_goal(rel_local_goal[:2], abs(self.line_info.angle))
                                    self.publish_target_velocity(0.1)
                                    self.is_line_trace = True

            self.line_detected = False
            for file in os.listdir(dir_name):
                if file.find('.') == 0:
                    continue
                file_timestamp = os.stat(dir_name+'/'+file)[ST_MTIME]
                if timestamp < file_timestamp:
                    timestamp = file_timestamp
                    try:
                        self.task_data = self.load_task_from_yaml()
                        print('task updated!')
                    except:
                        print('failed to update task')
            r.sleep()

    def get_yaw(self, orientation):
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw

    def create_quaternion_from_yaw(self, yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def calc_line_direction(self, line_angle):
        robot_direction = self.get_yaw(self.estimated_pose.pose.pose.orientation)
        edge_direction = self.estimated_edge.direction
        direction_diff = robot_direction - edge_direction
        # print("robot :{} edge :{} diff :{}".format(robot_direction, edge_direction, direction_diff))
        line_direction = line_angle - math.pi*0.5 + direction_diff
        line_direction = abs(math.atan2(math.sin(line_direction), math.cos(line_direction)))
        return line_direction

    def calc_line_dist(self):
        curr_pose = self.odom.pose.pose.position
        past_pose = self.line_detected_pose.pose.pose.position
        dist = math.sqrt((past_pose.x - curr_pose.x)**2 + (past_pose.y - curr_pose.y)**2)
        return dist

    def kill_node(self, nodename):
        node_list_cmd = "rosnode list"
        ros_node_process = subprocess.Popen(node_list_cmd.split(), stdout=subprocess.PIPE)
        ros_node_process.wait()
        nodetuple = ros_node_process.communicate()
        nodelist = nodetuple[0]
        nodelist = nodelist.split("\n")
        for nd in nodelist:
            if nd.find(nodename) == 0:
                kill_cmd = "rosnode kill %s" % nd
                subprocess.call(kill_cmd.split())
                self.process_terminated = True
                print("success to kill the process")
                return
        print("failed to kill the process")

    def interrupt_local_goal(self, flag):
        rospy.wait_for_service('/local_goal/interruption')
        try:
            client = rospy.ServiceProxy('/local_goal/interruption', SetBool)
            req = SetBool()
            req.data = flag
            res = client(req.data)
            if res.success:
                if flag:
                    print("success to stop local_goal_creator")
                else:
                    print("success to restart local_goal_creator")

        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

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
                print("success to update the edge")
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def request_replan(self):
        rospy.wait_for_service('/global_path/replan')
        try:
            client = rospy.ServiceProxy('/global_path/replan', Replan)
            req = Replan()
            req.edge = self.estimated_edge
            res = client(req.edge)
            if res.succeeded:
                print("success to request replan")
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def load_task_from_yaml(self):
        with open(self.TASK_LIST_PATH) as file:
            task_data = yaml.load(file)
        pprint(task_data)
        return task_data

    def publish_local_goal(self, pose, line_angle):
        local_goal = PoseStamped()
        local_goal.header.stamp = rospy.get_rostime()
        local_goal.header.frame_id = self.ROBOT_FRAME
        local_goal.pose.position.x = pose[0]
        local_goal.pose.position.y = pose[1]
        local_goal.pose.position.z = 0.0
        line_direction = math.pi*0.5 - line_angle
        local_goal.pose.orientation = self.create_quaternion_from_yaw(line_direction)
        self.local_goal_pub.publish(local_goal)
        print("----- published local goal -----")
        print("target angle :{}".format(line_direction))
        print("line angle :{}".format(self.line_info.angle))
        print(local_goal)

    def publish_target_velocity(self, vel):
        velocity = Twist()
        velocity.linear.x = vel
        self.target_velocity_pub.publish(velocity)
        print("----- published target velocity -----")
        print(velocity)

    def map_callback(self, node_edge_map):
        self.map = node_edge_map

    def pose_callback(self, pose):
        self.estimated_pose = pose
        # if self.line_detected_pose == None:
        #     self.line_detected_pose = pose
        # print("line detected pose x:{}, y:{}".format(self.line_detected_pose.pose.pose.position.x, self.line_detected_pose.pose.pose.position.y))

    def odom_callback(self, odom):
        self.odom = odom
        if self.line_detected_pose == None:
            self.line_detected_pose = odom
        # print("pose x:{}, y:{}".format(odom.pose.pose.position.x, odom.pose.pose.position.y))


    def edge_callback(self, edge):
        self.estimated_edge = edge

    def goal_flag_callback(self, goal_flag):
        self.goal_flag = goal_flag
        self.stop_pub.publish(Bool(True))

    def stop_line_callback(self, line_info):
        self.line_detected = True
        self.line_info = line_info

    def closed_sign_callback(self, detection):
        self.road_closed = detection.data

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
