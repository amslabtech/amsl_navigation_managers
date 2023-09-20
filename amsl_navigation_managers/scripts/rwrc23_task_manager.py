#!/usr/bin/env python3
#! coding:utf-8

#注：rwrc22_task_managerをベースにしています
import yaml
import math

import rospy
from std_msgs.msg import Bool, Int32, Int32MultiArray, Float64
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
import tf2_ros
import subprocess

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')
        print('=== task manager ===')

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')
        self.STOP_LIST_PATH = rospy.get_param('~STOP_LIST_PATH')
        self.ANNOUNCE_SOUND_PATH = rospy.get_param('~ANNOUNCE_SOUND_PATH', "../sounds/announcement_long_plus2sec.wav")
        self.USE_DETECT_WHITE_LINE = rospy.get_param('~USE_DETECT_WHITE_LINE')
        self.STOP_LINE_THRESHOLD = rospy.get_param('~STOP_LINE_THRESHOLD')
        self.turn_rate = rospy.get_param('~turn_rate', 0.5)
        # self.enable_announce = rospy.get_param('~enable_announce', False)
        self.enable_announce = rospy.get_param('~enable_announce', False)
        self.sound_volume = rospy.get_param('~sound_volume', 100)
        self.dwa_target_velocity = rospy.get_param('~dwa_target_velocity', 1.0)
        self.pfp_target_velocity = rospy.get_param('~pfp_target_velocity', 1.0)
        self.detect_line_pfp_target_velocity = rospy.get_param('~detect_line_pfp_target_velocity', 0.3)

        # params in callback function
        self.current_checkpoint_id = self.next_checkpoint_id = -1
        self.stop_line_flag = False
        self.skip_node_flag = False
        self.local_planner_cmd_vel = Twist()
        self.local_goal = PoseStamped()
        self.joy = Joy()
        self.localized_pose = PoseWithCovarianceStamped()
        # self.checkpoint_array = []
        self.target_velocity = Twist()

        # params
        self.get_task = False
        self.task_data = self.load_task_from_yaml()
        self.reached_checkpoint = False
        self.get_stop_list = False
        self.stop_list = self.load_stop_list_from_yaml()
        self.stop_node_flag = False
        self.cross_traffic_light_flag = False
        # self.get_checkpoint_array = False
        self.ignore_flag = False
        self.has_stopped = False
        self.switch_detect_line = False
        self.local_goal_dist = 7.0
        self.start_announce_flag = False
        self.announce_pid = 0
        self.last_planner = "dwa"
        self.task_stop_flag = Bool()

        # msg update flags
        self.local_planner_cmd_vel_updated = False
        self.local_goal_updated = False
        self.localized_pose_updated = False
        self.joy_updated = False

        # ros
        self.current_checkpoint_id_sub = rospy.Subscriber('/current_checkpoint', Int32, self.current_checkpoint_id_callback)
        self.current_nextpoint_id_sub = rospy.Subscriber('/next_checkpoint', Int32, self.next_checkpoint_id_callback)
        self.stop_line_flag_sub = rospy.Subscriber('/stop_line_detector/stop_flag', Bool, self.stop_line_flag_callback)
        self.local_planner_cmd_vel_sub = rospy.Subscriber('/local_planner/cmd_vel', Twist, self.local_planner_cmd_vel_callback)
        self.local_goal_sub = rospy.Subscriber('/local_goal', PoseStamped, self.local_goal_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.localized_pose_sub = rospy.Subscriber('/localized_pose', PoseWithCovarianceStamped, self.localized_pose_callback)
        # self.checkpoint_array_sub = rospy.Subscriber('/node_edge_map/checkpoint', Int32MultiArray, self.checkpoint_array_callback)
        self.skip_node_flag_sub = rospy.Subscriber('/skip_node_flag', Bool, self.skip_node_flag_callback)
        self.cross_traffic_light_flag_sub = rospy.Subscriber('/cross_traffic_light_flag', Bool, self.cross_traffic_light_flag_callback)

        self.detect_line_flag_pub = rospy.Publisher('~request_detect_line', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/local_path/cmd_vel/no_use', Twist, queue_size=1)
        self.is_stop_node_pub = rospy.Publisher('/is_stop_node_flag', Bool, queue_size=1)
        self.local_goal_dist_pub = rospy.Publisher('/local_goal_dist', Float64, queue_size=1)
        self.target_velocity_pub = rospy.Publisher('/target_velocity', Twist, queue_size=1)
        self.task_stop_pub = rospy.Publisher('/task/stop', Bool, queue_size=1)


    def process(self):
        if self.get_task == False or self.get_stop_list == False:
            rospy.logerr('task or stop list is not loaded')
            exit(-1)

        prev_task_type = ''
        r = rospy.Rate(10)
        # self.set_sound_volume(0)
        # proc = announce_once()
        self.set_sound_volume()
        while not rospy.is_shutdown():
            if(self.local_goal_updated and self.localized_pose_updated):
                rospy.loginfo('================================================================')
                task_type = self.search_task_from_node_id(self.current_checkpoint_id, self.next_checkpoint_id)
                print("task_type: ", task_type)
                print("last_planner: ", self.last_planner)
                self.target_velocity.linear.x = self.dwa_target_velocity
                # self.task_stop_flag.data = False

                ##### enable white line detector & stop behind robot #####
                enable_detect_line = Bool()
                if task_type == 'detect_line' and self.USE_DETECT_WHITE_LINE:
                    enable_detect_line.data = True
                    # self.use_point_follow_planner()
                    # target_velocity = Twist()
                    # self.target_velocity.linear.x = self.detect_line_pfp_target_velocity
                    # self.target_velocity_pub.publish(target_velocity)
                else:
                    enable_detect_line.data = False
                self.detect_line_flag_pub.publish(enable_detect_line)

                if task_type == 'detect_line' and prev_task_type != task_type:
                    self.use_point_follow_planner()
                    self.target_velocity.linear.x = self.detect_line_pfp_target_velocity

                # rospy.loginfo('detect_line_flag_pub = %s' % enable_detect_line.data)
                ##### enable white line detector & stop behind robot #####

                ##### announce #####
                # if task_type == 'announce':
                #     self.announce_once()
                ##### announce #####

                ##### shorten goal dist #####

                if task_type == "traffic_light" and prev_task_type != task_type:
                    if(self.cross_traffic_light_flag and self.get_go_signal(joy)):
                        self.ignore_flag = True
                        self.has_stopped = False


                ##### autodoor #####
                if task_type == 'autodoor' and prev_task_type != task_type:
                    # print("task_type: ", task_type)
                    self.use_point_follow_planner()
                    # target_velocity = Twist()
                    self.target_velocity.linear.x = self.pfp_target_velocity
                    # self.target_velocity_pub.publish(target_velocity)
                    # self.announce_once()
                    # self.local_goal_dist = 3.0
                # else:
                    # self.local_goal_dist = 7.0
                if task_type == '' and prev_task_type != task_type:
                    self.use_local_planner()
                    # target_velocity = Twist()
                    self.target_velocity.linear.x = self.dwa_target_velocity
                    # self.target_velocity_pub.publish(target_velocity)
                # self.local_goal_dist_pub.publish(self.local_goal_dist)
                ##### shorten goal dist #####

                #### skip node announce ####
                if self.skip_node_flag:
                    self.skip_node_flag = False
                    # self.set_sound_volume()
                    # self.announce_once()
                #### skip node announce ####

                ##### select cmd_vel #####
                # if task_type == 'change_local_planner':
                #     self.use_local_planner()
                # elif task_type == 'change_point_follow_planner':
                #     self.use_point_follow_planner()
                ##### select cmd_vel #####

                cmd_vel = Twist()
                cmd_vel.linear.x = self.local_planner_cmd_vel.linear.x
                cmd_vel.angular.z = self.local_planner_cmd_vel.angular.z

                ##### turn at node #####
                # if(self.reached_checkpoint):
                #     cmd_vel, self.reached_checkpoint = self.get_turn_cmd_vel(self.local_goal, self.local_planner_cmd_vel)
                ##### turn at node #####

                ##### stop at designated node #####
                # self.stop_node_flag = self.is_stop_node(self.stop_list, self.current_checkpoint_id)
                # if(self.stop_node_flag):
                #     cmd_vel, is_not_toward = self.get_turn_cmd_vel(self.local_goal, self.local_planner_cmd_vel)
                #     if is_not_toward == False: # toward goal
                #         cmd_vel.linear.x = 0.0
                #         cmd_vel.angular.z = 0.0
                #     if self.get_go_signal(self.joy):
                #         del self.stop_list[0]
                ##### stop at designated node #####

                ##### stop at white line #####
                if self.switch_detect_line != enable_detect_line.data:
                    self.switch_detect_line = enable_detect_line.data
                    self.ignore_flag = False

                if enable_detect_line.data:

                    # self.ignore_count += 1
                    # if self.ignore_count > 100:
                        # self.ignore_count = 100

                    # rospy.loginfo("=======================")
                    # rospy.loginfo('self.stop_node_flag = %s' % self.stop_node_flag)

                    if self.stop_line_flag and self.ignore_flag == False:
                        self.has_stopped = True

                    if self.has_stopped:
                        cmd_vel, is_not_toward = self.get_turn_cmd_vel(self.local_goal, self.local_planner_cmd_vel)
                        if is_not_toward == False: # toward goal
                            self.target_velocity.linear.x = 0.0
                            cmd_vel.linear.x = 0.0
                            cmd_vel.angular.z = 0.0
                            self.task_stop_flag.data = True
                            self.task_stop_pub.publish(self.task_stop_flag)

                    # if cmd_vel.linear.x < 0.01 and cmd_vel.angular.z < 0.01 and self.get_go_signal(self.joy):
                    if self.has_stopped and self.get_go_signal(self.joy):
                        # self.ignore_count = 0
                        # if self.stop_node_flag or self.ignore_flag == False:
                        # del self.stop_list[0]
                        self.ignore_flag = True
                        self.has_stopped = False
                        # if self.stop_node_flag:
                        #     del self.stop_list[0]
                        self.task_stop_flag.data = False
                        self.task_stop_pub.publish(self.task_stop_flag)

                    # self.task_stop_pub.publish(self.task_stop_flag)

                self.stop_node_flag = self.is_stop_node(self.stop_list, self.current_checkpoint_id)

                print(f"current_checkpoint : {self.current_checkpoint_id}")
                print(f"next_checkpoint : {self.next_checkpoint_id}")

                if(self.stop_node_flag):
                    cmd_vel, is_not_toward = self.get_turn_cmd_vel(self.local_goal, self.local_planner_cmd_vel)
                    if is_not_toward == False: # toward goal
                        self.target_velocity.linear.x = 0.0
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.task_stop_flag.data = True
                        self.task_stop_pub.publish(self.task_stop_flag)
                    if self.get_go_signal(self.joy):
                        self.ignore_flag = True
                        self.has_stopped = False
                        del self.stop_list[0]
                        self.task_stop_flag.data = False
                        self.task_stop_pub.publish(self.task_stop_flag)

                    # self.task_stop_pub.publish(self.task_stop_flag)

                # rospy.loginfo('self.stop_list = %s' % self.stop_list)
                # rospy.loginfo('self.current_checkpoint_id = %s' % self.current_checkpoint_id)
                # rospy.loginfo('self.ignore_flag = %s' % self.ignore_flag)
                # rospy.loginfo('self.stop_node_flag = %s' % self.stop_node_flag)
                # rospy.loginfo('self.has_stopped = %s' % self.has_stopped)
                # rospy.loginfo('ignore_flag = %s' % self.ignore_flag)
                ##### stop at white line #####

                ##### linear.x limit #####
                # if(enable_detect_line.data == True):
                #     cmd_vel.linear.x = min(cmd_vel.linear.x, 0.3)
                ##### linear.x limit #####

                self.cmd_vel_pub.publish(cmd_vel)
                self.local_planner_cmd_vel_updated = False
                self.local_goal_updated = False
                self.localized_pose_updated = False
                self.target_velocity_pub.publish(self.target_velocity)

                self.is_stop_node_flag_publish(self.next_checkpoint_id, self.stop_list)
                prev_task_type = task_type
            else:
                if self.local_planner_cmd_vel_updated == False:
                    rospy.logwarn('local_planner_cmd_vel is not updated')
                if self.local_goal_updated == False:
                    rospy.logwarn('local_goal is not updated')
                if self.localized_pose_updated == False:
                    rospy.logwarn('localized_pose is not updated')
            r.sleep()

    def get_turn_cmd_vel(self, goal, local_planner_cmd_vel):
        goal_yaw = math.atan2(goal.pose.position.y, goal.pose.position.x) # base_link to goal
        if(abs(goal_yaw) < 0.2):
            return local_planner_cmd_vel, False
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        if local_planner_cmd_vel.angular.z != 0.0:
            cmd_vel.angular.z = (local_planner_cmd_vel.angular.z / abs(local_planner_cmd_vel.angular.z)) * float(self.turn_rate)
        else:
            cmd_vel.angular.z = 0.0
        return cmd_vel, True

    def load_task_from_yaml(self):
        with open(self.TASK_LIST_PATH) as file:
            task_data = yaml.safe_load(file)
            self.get_task = True
            # print('get task')
        return task_data

    def load_stop_list_from_yaml(self):
        with open(self.STOP_LIST_PATH) as file:
            stop_list_from_yaml = yaml.safe_load(file)
            self.get_stop_list = True
        stop_list = []
        for count, stop in enumerate(stop_list_from_yaml['stop_list']):
            stop_list.append(stop['node_id'])
        return stop_list

    # def checkpoint_id_callback(self, msg):
    #     if self.current_checkpoint_id != int(msg.data):
    #         self.current_checkpoint_id, self.next_checkpoint_id = self.search_current_edge(int(msg.data))
    #         self.reached_checkpoint = True

    def current_checkpoint_id_callback(self, msg):
        self.current_checkpoint_id = int(msg.data)

    def next_checkpoint_id_callback(self, msg):
        self.next_checkpoint_id = int(msg.data)

    def stop_line_flag_callback(self, flag):
        self.stop_line_flag = flag.data

    def cross_traffic_light_flag_callback(self, flag):
        self.cross_traffic_light_flag = flag.data

    def skip_node_flag_callback(self, flag):
        self.skip_node_flag = flag.data

    def local_planner_cmd_vel_callback(self, msg):
        self.local_planner_cmd_vel = msg
        self.local_planner_cmd_vel_updated = True

    def local_goal_callback(self, msg):
        self.local_goal = msg
        self.local_goal_updated = True

    def joy_callback(self, msg):
        self.joy = msg
        self.joy_updated = True

    def localized_pose_callback(self, msg):
        self.localized_pose = msg
        self.localized_pose_updated = True

    # def checkpoint_array_callback(self, msg):
    #     if self.get_checkpoint_array == False:
    #         self.checkpoint_array.clear()
    #         for id in msg.data:
    #             self.checkpoint_array.append(id)
    #         self.get_checkpoint_array = True

    def search_task_from_node_id(self, node0_id, node1_id):
        if self.get_task == True:
            # print('get_task is true')
            for count, task in enumerate(self.task_data['task']):
                # print(task)
                if (task['edge']['node0_id'] == node0_id) and (task['edge']['node1_id'] == node1_id):
                    return task['task_type']
            return ''
        else:
            return ''

    def is_stop_node(self, stop_list, current_checkpoint_id):
        if self.get_stop_list == False:
            return False
        if len(stop_list) == 0:
            return False
        if stop_list[0] == current_checkpoint_id:
            return True
        return False

    def get_go_signal(self, joy):
        if self.joy_updated == True:
            self.joy_updated = False
            if (joy.buttons[11] == 1 and joy.buttons[12] == 1) or joy.buttons[1] == 1:
                return True
            return False
        else:
            return False

    # def search_current_edge(self, current_checkpoint_id):
    #     if len(self.checkpoint_array) == 0:
    #         return -1, -1
    #     current_id = self.checkpoint_array[0]
    #     next_id = self.checkpoint_array[1]
    #     while self.checkpoint_array[0] == current_checkpoint_id:
    #         current_id = self.checkpoint_array[0]
    #         next_id = self.checkpoint_array[1]
    #         del self.checkpoint_array[0]
    #         if len(self.checkpoint_array) < 1:
    #             return -1, -1
    #     return current_id, next_id

    def is_stop_node_flag_publish(self, next_node_id, stop_list):
        is_stop_node_flag = Bool()
        if(len(stop_list) == 0) or self.get_stop_list == False:
            is_stop_node_flag.data = False
        elif next_node_id == stop_list[0]:
            is_stop_node_flag.data = True
        else:
            is_stop_node_flag.data = False
        self.is_stop_node_pub.publish(is_stop_node_flag)

    def set_sound_volume(self):
        if self.enable_announce == True :
            volume_cmd = "amixer -c1 sset Speaker " + str(self.sound_volume) + "%," + str(self.sound_volume) + "% unmute"
            subprocess.Popen(volume_cmd.split())

    def announce_once(self):
        if self.enable_announce == True :
            announce_cmd = "aplay " + self.ANNOUNCE_SOUND_PATH
            # announce_proc = subprocess.Popen(announce_cmd.split())
            announce_proc = subprocess.call(announce_cmd.split())

    def use_local_planner(self):
        subprocess.Popen(['rosrun','topic_tools','mux_select','/local_planner/cmd_vel','/local_planner/dwa_planner/cmd_vel'])
        subprocess.Popen(['rosrun','topic_tools','mux_select','/local_planner/candidate_trajectories','/local_planner/dwa_planner/candidate_trajectories'])
        subprocess.Popen(['rosrun','topic_tools','mux_select','/local_planner/selected_trajectory','/local_planner/dwa_planner/selected_trajectory'])
        self.last_planner = "dwa"

    def use_point_follow_planner(self):
        subprocess.Popen(['rosrun','topic_tools','mux_select','/local_planner/cmd_vel','/local_planner/point_follow_planner/cmd_vel'])
        subprocess.Popen(['rosrun','topic_tools','mux_select','/local_planner/candidate_trajectories','/local_planner/point_follow_planner/candidate_trajectories'])
        subprocess.Popen(['rosrun','topic_tools','mux_select','/local_planner/selected_trajectory','/local_planner/point_follow_planner/best_trajectory'])
        self.last_planner = "pfp"

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
