#!/usr/bin/env python3
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
        print('=== task manager ===')

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')
        self.LINE_DIST_THRESHOLD = rospy.get_param('LINE_DIST_THRESHOLD', 3.0)
        self.ROBOT_FRAME = rospy.get_param('ROBOT_FRAME', "base_link")
        self.REST_TIME = rospy.get_param('REST_TIME', 0.0)

        self.map = None
        # self.odom = Odometry()
        # self.estimated_pose = Odometry()
        self.estimated_edge = Edge()
        self.current_edge = Edge()
        self.goal_flag = Empty()

        # subscriber
        self.map_sub = rospy.Subscriber('/node_edge_map/map', NodeEdgeMap, self.map_callback) # from node_edge_manager
        self.edge_sub = rospy.Subscriber('/estimated_pose/edge', Edge, self.edge_callback) # node_edge_localizerの”get_edge_from_estimated_pose”で算出できそう
        self.goal_flag_sub = rospy.Subscriber('/node_edge_navigator/goal_flag', Empty, self.goal_flag_callback) # どこからpubされるかまだ謎
        self.closed_sign_sub = rospy.Subscriber('/recognition/closed_sign', Bool, self.closed_sign_callback) # 通行止め看板を見つけたら飛んでくる情報
        self.traffic_sign_sub = rospy.Subscriber('/recognition/traffic_light', Bool, self.traffic_sign_callback) # 通行止め看板を見つけたら飛んでくる情報
        # self.stop_line_sub = rospy.Subscriber('/recognition/stop_line', StopLine, self.stop_line_callback) # 白線を見つけたら飛んでくる情報
        
        # self.odom_sub = rospy.Subscriber('/odom/complement', Odometry, self.odom_callback) # なくしたい

        # publisher
        self.stop_pub = rospy.Publisher('/task/stop', Bool, queue_size=1) # とまれ指示 bool 
        self.traffic_launch_pub = rospy.Publisher('/task/traffic_light', Bool, queue_size=1)
        self.measurement_pub = rospy.Publisher('/task/measurement_update', Bool, queue_size=1)
        
        # self.ignore_pub = rospy.Publisher('/task/ignore_intensity', Bool, queue_size=1) # intensity使う領域かどうか bool
        # self.grassy_pub = rospy.Publisher('/task/grassy', Bool, queue_size=1) # 芝生だよ bool

        # self.target_velocity_pub = rospy.Publisher('/target_velocity', Twist, queue_size=1) # いる？
        # self.local_goal_pub = rospy.Publisher('/local_goal', PoseStamped, queue_size=1) # いる？
        
        self.road_closed = False
       
        self.first_park_flag = True
        self.first_traffic_flag = True
        self.first_traffic_flag_2 = True
        

        self.process_terminated = False
        self.process_terminated_2 = False
        self.process_terminated_3 = False

        self.ignore_intensity_flag = False
        self.grassy_flag = False
        self.traffic_flag = False # false: 青  true: 赤

        self.subprocess1 = "road_closed_sign_detector"
        self.subprocess2 = "darknet_ros"
        self.subprocess2_1 = "old"
        self.lock = threading.Lock()
        self.task_data = self.load_task_from_yaml()

    def process(self):
        r = rospy.Rate(10)
        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.TASK_LIST_PATH)
        # label: 場所関係 trigger: 場所に関係なし
        while not rospy.is_shutdown():
            if self.map is not None: # map情報がある
               # ----------------信号認識＋通行止め看板認識------------------------------ 
                if (self.map.nodes[self.estimated_edge.node0_id].label == 'traffic_detector_launch'): # 信号認識範囲内なら
                    # print("----- see traffic!!! -----")
                    self.traffic_launch_pub.publish(True) # launch darknet
               
                # if (self.map.nodes[self.estimated_edge.node0_id].label == 'traffic_sign_outbound'): # 信号認識範囲内なら
                #     # print("traffic_sign area!!!")
                #     if self.first_traffic_flag:
                #         print("----- booting %s -----" % self.subprocess2)
                #         cmd2 = "roslaunch %s %s.launch" % (self.subprocess2, self.subprocess2_1) # launch起動
                #         p2 = subprocess.Popen(cmd2.split())
                #         self.first_traffic_flag = False
                #     if not self.process_terminated_2:
                #         if self.traffic_flag: # 赤信号
                #             # print("stop ccv!!!!!!")
                #             self.stop_pub.publish(True) # stop sign
                #             self.traffic_flag = False
                # # elif(self.map.nodes[self.estimated_edge.node0_id].label == '' or self.map.nodes[self.estimated_edge.node0_id].label == 'park' ): # 信号認識範囲外
                # elif(self.map.nodes[self.estimated_edge.node0_id].label != 'traffic_sign_outbound' ): # 信号認識範囲外
                #     if (not self.first_traffic_flag and not self.process_terminated_2): 
                #         print("killing %s" % self.subprocess2) # darknet_ros kill
                #         node2 = "/navigation_managers/%s" % (self.subprocess2)
                #         self.kill_node_2(node2)
                # # 復路 
                # if (self.map.nodes[self.estimated_edge.node0_id].label == 'traffic_sign_inbound'): # 信号認識範囲内なら
                #     # print("traffic_sign area!!!")
                #     if self.first_traffic_flag_2:
                #         print("----- booting %s -----" % self.subprocess2)
                #         cmd2 = "roslaunch %s %s.launch" % (self.subprocess2, self.subprocess2) # launch起動
                #         p2 = subprocess.Popen(cmd2.split())
                #         self.first_traffic_flag_2 = False
                #     if not self.process_terminated_3:
                #         if self.traffic_flag: # 赤信号
                #             print("stop ccv!!!!!!")
                #             # self.stop_pub.publish(True) # stop sign
                #             self.traffic_flag = False
                # # elif(self.map.nodes[self.estimated_edge.node0_id].label == '' or self.map.nodes[self.estimated_edge.node0_id].label == 'park' ): # 信号認識範囲外
                # elif(self.map.nodes[self.estimated_edge.node0_id].label != 'traffic_sign_inbound' ): # 信号認識範囲外
                #     if (not self.first_traffic_flag_2 and not self.process_terminated_3): 
                #         print("killing %s" % self.subprocess2) # darknet_ros kill
                #         node2 = "/navigation_managers/%s" % (self.subprocess2)
                #         self.kill_node(node2)
                #         self.process_terminated_3 = True
                
                if (self.map.nodes[self.estimated_edge.node0_id].label == 'park'): # 公園内なら
                    # print("in the park!!!")
                    if self.first_park_flag:
                        print("----- booting %s -----" % self.subprocess1)
                        cmd = "roslaunch %s %s.launch" % (self.subprocess1, self.subprocess1) # launch起動
                        p1 = subprocess.Popen(cmd.split())
                        self.first_park_flag = False # 重複launch防止
                    if not self.process_terminated:
                        if self.road_closed:
                            print("closed sign detect!!!!")
                            # self.set_impassable_edge(self.estimated_edge)
                            # rospy.sleep(0.1)
                            # self.request_replan() # replan要請
                            self.road_closed = False
                # else: # 公園外
                # elif(self.map.nodes[self.estimated_edge.node0_id].label == '' or self.map.nodes[self.estimated_edge.node0_id].label == 'traffic_sign' ):
                elif(self.map.nodes[self.estimated_edge.node0_id].label != 'park' ):
                    if (not self.first_park_flag and not self.process_terminated): # node切る
                        print("killing %s" % self.subprocess1)
                        node1 = "/navigation_managers/%s/%s" % (self.subprocess1, self.subprocess1)
                        self.kill_node(node1)
                        self.process_terminated = True
            # ------------------------------------------------------------------------
            for count, task in enumerate(self.task_data['task']):
                 if (task['edge']['node0_id'] == self.estimated_edge.node0_id) and (task['edge']['node1_id'] == self.estimated_edge.node1_id):
                     if task['trigger'] == 'bool/measurement_update':
                        self.measurement_pub.publish(True) # 

            # road_recognizerとの連携
            # いろいろとintensityの傾向をさぐれたら入れる予定
            # for count, task in enumerate(self.task_data['task']):
            #     if (task['edge']['node0_id'] == self.estimated_edge.node0_id) and (task['edge']['node1_id'] == self.estimated_edge.node1_id):
            #         # if (task['edge']['progress_min'] < self.estimated_edge.progress) and (self.estimated_edge.progress < task['edge']['progress_max']): # もし今いるedgeのお仕事領域に達したら  廃止？
            #         if task['trigger'] == 'enter/edge': 
            #             if task['task_type'] == "ignore_intensity" :
            #                 if not self.ignore_intensity_flag:
            #                     self.ignore_intensity_flag = True
            #                     self.ignore_pub.publish(Bool(self.ignore_intensity_flag))
            #             elif task['task_type'] == "use_intensity" : # intensityを使う場所
            #                 if self.ignore_intensity_flag:
            #                     self.ignore_intensity_flag = False
            #                     self.ignore_pub.publish(Bool(self.ignore_intensity_flag))
            #             if task['task_type'] == "grassy_area" : # 芝生エリア
            #                 if not self.grassy_flag:
            #                     self.grassy_flag = True
            #                     self.grassy_pub.publish(Bool(self.grassy_flag))
            #             elif task['task_type'] == "not_grassy_area" : 
            #                 if self.grassy_flag:
            #                     self.grassy_flag = False
            #                     self.grassy_pub.publish(Bool(self.grassy_flag))
                    
                    # 強制停止
                    # elif task['trigger'] == 'recognition/must_stop':
                    #     self.stop_pub.publish(True)

                    #白線検知やるなら

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

    # def get_yaw(self, orientation):
    #     quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    #     _, _, yaw = euler_from_quaternion(quaternion)
    #     return yaw

    
    def create_quaternion_from_yaw(self, yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    # 白線検知系関数は消しました

    def kill_node(self, nodename):
        node_list_cmd = "rosnode list"
        ros_node_process = subprocess.Popen(node_list_cmd.split(), stdout=subprocess.PIPE)
        ros_node_process.wait()
        nodetuple = ros_node_process.communicate()
        nodelist = nodetuple[0]
        nodelist = nodelist.decode() # add
        nodelist = nodelist.split("\n")
        for nd in nodelist:
            if nd.find(nodename) == 0:
                kill_cmd = "rosnode kill %s" % nd
                subprocess.call(kill_cmd.split())
                # self.process_terminated = True
                print("success to kill the process")
                return
        print("failed to kill the process")
    
    def kill_node_2(self, nodename):
        node_list_cmd = "rosnode list"
        ros_node_process_2 = subprocess.Popen(node_list_cmd.split(), stdout=subprocess.PIPE)
        ros_node_process_2.wait()
        nodetuple = ros_node_process_2.communicate()
        nodelist = nodetuple[0]
        nodelist = nodelist.decode() # add
        nodelist = nodelist.split("\n")
        for nd in nodelist:
            if nd.find(nodename) == 0:
                kill_cmd = "rosnode kill %s" % nd
                subprocess.call(kill_cmd.split())
                self.process_terminated_2 = True
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
                print("success to update the edge")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def request_replan(self):
        rospy.wait_for_service('/global_path/replan')
        try:
            client = rospy.ServiceProxy('/global_path/replan', Replan)
            req = Replan()
            req.edge = self.estimated_edge # 今のedgeを送信
            res = client(req.edge) # request 送信 & responseくる dijkstraへ送信
            if res.succeeded:
                print("success to request replan")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def load_task_from_yaml(self):
        with open(self.TASK_LIST_PATH) as file:
            task_data = yaml.safe_load(file)
        # pprint(task_data)
        return task_data
    
    def map_callback(self, node_edge_map):
        self.map = node_edge_map

    def edge_callback(self, edge):
        self.estimated_edge = edge

    def goal_flag_callback(self, goal_flag):
        self.goal_flag = goal_flag
        self.stop_pub.publish(Bool(True))

    def closed_sign_callback(self, detection):
        self.road_closed = detection.data
    
    def traffic_sign_callback(self, line_info):
        self.traffic_flag = True

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
    
    
    # def interrupt_local_goal(self, flag):
    #     rospy.wait_for_service('/local_goal/interruption')
    #     try:
    #         client = rospy.ServiceProxy('/local_goal/interruption', SetBool)
    #         req = SetBool()
    #         req.data = flag
    #         res = client(req.data)
    #         if res.success:
    #             if flag:
    #                 print("success to stop local_goal_creator")
    #             else:
    #                 print("success to restart local_goal_creator")

    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

    # def publish_local_goal(self, pose, line_angle):
    #     local_goal = PoseStamped()
    #     local_goal.header.stamp = rospy.get_rostime()
    #     local_goal.header.frame_id = self.ROBOT_FRAME
    #     local_goal.pose.position.x = pose[0]
    #     local_goal.pose.position.y = pose[1]
    #     local_goal.pose.position.z = 0.0
    #     line_direction = math.pi*0.5 - line_angle
    #     local_goal.pose.orientation = self.create_quaternion_from_yaw(line_direction)
    #     # self.local_goal_pub.publish(local_goal)
    #     print("----- published local goal -----")
    #     print("target angle :{}".format(line_direction))
    #     print("line angle :{}".format(self.line_info.angle))
    #     print(local_goal)

    # def publish_target_velocity(self, vel):
    #     velocity = Twist()
    #     velocity.linear.x = vel
    #     self.target_velocity_pub.publish(velocity)
    #     print("----- published target velocity -----")
    #     print(velocity)


    # def pose_callback(self, pose):
    #     self.estimated_pose = pose
        # ---------------------------------------------
        # 〜poseから今いるcurrent_edgeを算出する〜
        # ---------------------------------------------
        # self.current_edge = estimated_edge

        # if self.line_detected_pose == None:
        #     self.line_detected_pose = pose
        # print("line detected pose x:{}, y:{}".format(self.line_detected_pose.pose.pose.position.x, self.line_detected_pose.pose.pose.position.y))

    # def odom_callback(self, odom):
    #     self.odom = odom
    #     if self.line_detected_pose == None:
    #         self.line_detected_pose = odom
        

        # self.estimated_edge = ~~~~~
        # print("pose x:{}, y:{}".format(odom.pose.pose.position.x, odom.pose.pose.position.y))
