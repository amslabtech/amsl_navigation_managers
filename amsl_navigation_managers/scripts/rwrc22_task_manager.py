#!/usr/bin/env python3
#! coding:utf-8

#注：rwrc21_task_managerとは仕様が全く違います
import yaml
import math

import rospy
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
import tf2_ros

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')
        print('=== task manager ===')

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')
        # self.STOP_LIST_PATH = rospy.get_param('~STOP_LIST_PATH')
        self.turn_rate = rospy.set_param('~turn_rate', 0.5)

        # ros
        self.current_checkpoint_id_sub = rospy.Subscriber('/current_checkpoint', Int32, self.checkpoint_id_callback)
        self.stop_line_flag_sub = rospy.Subscriber('/stop_line_flag', Bool, self.stop_line_flag_callback)
        self.local_planner_cmd_vel_sub = rospy.Subscriber('/local_planner/cmd_vel', Twist, self.local_planner_cmd_vel_callback)
        self.local_goal_sub = rospy.Subscriber('/local_goal', PoseStamped, self.local_goal_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        self.detect_line_flag_pub = rospy.Publisher('/detect_line', Bool, queue_size=1)
        # self.task_stop_flag_pub = rospy.Publisher('/task/stop' , Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/local_path/cmd_vel', Twist, queue_size=1)

        # params in callback function
        self.current_checkpoint_id = self.last_checkpoint_id = -1
        self.stop_line_flag = False
        self.local_planner_cmd_vel = Twist()
        self.local_goal = PoseStamped()
        self.joy = Joy()
        self.amcl_pose = PoseWithCovarianceStamped()

        # params
        self.get_task = False
        self.task_data = self.load_task_from_yaml()
        self.reached_checkpoint = False
        # self.get_stop_list = False
        # self.stop_list = self.load_stop_list_from_yaml()

        # msg update flags
        self.local_planner_cmd_vel_updated = False
        self.local_goal_updated = False
        self.amcl_pose_updated = False


    def process(self):
        print('=== process started ===')
        if self.get_task == False:
            rospy.logerr('task is not get')
            exit(-1)
        # if self.get_task == False or self.get_stop_list == False:
        #     rospy.logerr('task or stop list is not loaded')
        #     exit(-1)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.local_planner_cmd_vel_updated and self.local_goal_updated and self.amcl_pose_updated):
                rospy.loginfo('================================================================')
                task_type = self.search_task_from_node_id(self.last_checkpoint_id, self.current_checkpoint_id)

                ##### white line detector & stop behind robot #####
                enable_detect_line = Bool()
                if task_type == 'detect_line':
                    enable_detect_line.data = True
                else:
                    enable_detect_line.data = False
                self.detect_line_flag_pub.publish(enable_detect_line)
                rospy.loginfo('detect_line_flag_pub = %s' % enable_detect_line.data)
                ##### white line detector & stop behind robot #####

                cmd_vel = Twist()
                cmd_vel.linear.x = self.local_planner_cmd_vel.linear.x
                cmd_vel.angular.z = self.local_planner_cmd_vel.angular.z

                ##### turn at node #####
                if(self.reached_checkpoint):
                    cmd_vel, self.reached_checkpoint = self.get_turn_cmd_vel(self.local_goal, self.local_planner_cmd_vel)
                ##### turn at node #####

                    ##### stop at designated node #####
                    # if(task_type == 'stop'):
                    ##### stop at designated node #####

                ##### stop at white line #####
                # if(self.stop_line_flag):
                #     cmd_vel.linear.x = 0.0
                #     cmd_vel.angular.z = 0.0
                ##### stop at white line #####

                self.cmd_vel_pub.publish(cmd_vel)
                self.local_planner_cmd_vel_updated = False
                self.local_goal_updated = False
                self.amcl_pose_updated = False
            else:
                if self.local_planner_cmd_vel_updated == False:
                    rospy.logwarn('local_planner_cmd_vel is not updated')
                if self.local_goal_updated == False:
                    rospy.logwarn('local_goal is not updated')
                if self.amcl_pose_updated == False:
                    rospy.logwarn('amcl_pose is not updated')
            r.sleep()

    def get_turn_cmd_vel(self, goal, local_planner_cmd_vel):
        goal_yaw = math.atan2(goal.pose.position.y, goal.pose.position.x) # base_link to goal
        if(abs(goal_yaw) < 0.2):
            return local_planner_cmd_vel, False
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        if local_planner_cmd_vel.angular.z != 0.0:
            cmd_vel.angular.z = (local_planner_cmd_vel.angular.z / abs(local_planner_cmd_vel.angular.z)) * self.turn_rate
        else:
            cmd_vel.angular.z = 0.0
        return cmd_vel, True

    def load_task_from_yaml(self):
        with open(self.TASK_LIST_PATH) as file:
            task_data = yaml.safe_load(file)
            self.get_task = True
            # print('get task')
        return task_data

    # def load_stop_list_from_yaml(self):
    #     with open(self.STOP_LIST_PATH) as file:
    #         stop_list = yaml.safe_load(file)
    #         self.get_stop_list = True
    #         # print('get stop list')
    #     return stop_list

    def checkpoint_id_callback(self, checkpoint_id):
        if self.current_checkpoint_id != int(checkpoint_id.data):
            self.last_checkpoint_id = self.current_checkpoint_id
            self.current_checkpoint_id = int(checkpoint_id.data)
            self.reached_checkpoint = True

    def stop_line_flag_callback(self, flag):
        self.stop_line_flag = flag.data

    def local_planner_cmd_vel_callback(self, msg):
        self.local_planner_cmd_vel = msg
        self.local_planner_cmd_vel_updated = True

    def local_goal_callback(self, msg):
        self.local_goal = msg
        self.local_goal_updated = True

    def joy_callback(self, msg):
        self.joy = msg

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg
        self.amcl_pose_updated = True

    def search_task_from_node_id(self, node0_id, node1_id):
        if self.get_task == True:
            # print('get_task is true')
            for count, task in enumerate(self.task_data['task']):
                # print(task)
                if (task['edge']['node0_id'] == node0_id) and (task['edge']['node1_id'] == node1_id):
                    return task['task_type']
        else:
            return ''

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
