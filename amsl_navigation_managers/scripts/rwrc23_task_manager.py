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
        rospy.loginfo("=== task manager ===")

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')
        self.STOP_LIST_PATH = rospy.get_param('~STOP_LIST_PATH')
        self.ANNOUNCE_SOUND_PATH = rospy.get_param('~ANNOUNCE_SOUND_PATH', "../sounds/announcement_long.wav")
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
        self.local_goal = PoseStamped()
        self.joy = Joy()
        self.target_velocity = Twist()

        # params
        self.get_task = False
        self.task_data = self.load_task_from_yaml()
        self.reached_checkpoint = False
        self.get_stop_list = False
        self.stop_list = self.load_stop_list_from_yaml()
        self.stop_node_flag = False
        self.cross_traffic_light_flag = False
        self.has_stopped = False
        self.local_goal_dist = 7.0
        self.start_announce_flag = False
        self.announce_pid = 0
        self.last_planner = "dwa"
        self.task_stop_flag = Bool()

        # msg update flags
        self.local_goal_updated = False
        self.joy_updated = False

        # ros
        self.current_checkpoint_id_sub = rospy.Subscriber('/current_checkpoint', Int32, self.current_checkpoint_id_callback)
        self.current_nextpoint_id_sub = rospy.Subscriber('/next_checkpoint', Int32, self.next_checkpoint_id_callback)
        self.stop_line_flag_sub = rospy.Subscriber('/stop_line_detector/stop_flag', Bool, self.stop_line_flag_callback)
        self.local_goal_sub = rospy.Subscriber('/local_goal', PoseStamped, self.local_goal_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.skip_node_flag_sub = rospy.Subscriber('/skip_node_flag', Bool, self.skip_node_flag_callback)
        self.cross_traffic_light_flag_sub = rospy.Subscriber('/cross_traffic_light_flag', Bool, self.cross_traffic_light_flag_callback)

        self.detect_line_flag_pub = rospy.Publisher('~request_detect_line', Bool, queue_size=1)
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
        self.target_velocity.linear.x = self.dwa_target_velocity
        enable_detect_line = Bool()
        while not rospy.is_shutdown():
            if self.local_goal_updated:
                rospy.loginfo_throttle(1, '================================================================')
                task_type = self.search_task_from_node_id(self.current_checkpoint_id, self.next_checkpoint_id)
                rospy.loginfo_throttle(1, f"task_type : {task_type}")
                rospy.loginfo_throttle(1, f"last_planner : {self.last_planner}")
                rospy.loginfo_throttle(1, f"current_checkpoint : {self.current_checkpoint_id}")
                rospy.loginfo_throttle(1, f"next_checkpoint : {self.next_checkpoint_id}")

                ##### enable white line detector #####
                if task_type == 'detect_line' and prev_task_type != task_type and self.USE_DETECT_WHITE_LINE:
                    enable_detect_line.data = True
                    self.target_velocity.linear.x = self.detect_line_pfp_target_velocity
                    self.use_point_follow_planner()
                self.detect_line_flag_pub.publish(enable_detect_line)

                ##### announce #####
                # if task_type == 'announce':
                #     self.announce_once()

                ##### traffic_light #####
                if task_type == "traffic_light" and prev_task_type != task_type:
                    if self.cross_traffic_light_flag and self.get_go_signal(joy):
                        self.has_stopped = False
                        enable_detect_line.data = False

                ##### point_follow_planner #####
                if task_type == 'autodoor' and prev_task_type != task_type:
                    self.use_point_follow_planner()
                    self.target_velocity.linear.x = self.pfp_target_velocity
                    enable_detect_line.data = False

                ##### no task #####
                if task_type == '' and prev_task_type != task_type:
                    self.use_local_planner()
                    self.target_velocity.linear.x = self.dwa_target_velocity
                    enable_detect_line.data = False

                #### skip node announce ####
                if self.skip_node_flag:
                    self.skip_node_flag = False
                    # self.set_sound_volume()
                    # self.announce_once()

                ##### stop at white line #####
                if enable_detect_line.data:
                    if self.stop_line_flag:
                        self.has_stopped = True

                    if self.has_stopped:
                        self.task_stop_flag.data = True
                        self.task_stop_pub.publish(self.task_stop_flag)

                    if self.has_stopped and self.get_go_signal(self.joy):
                        self.has_stopped = False
                        self.task_stop_flag.data = False
                        self.task_stop_pub.publish(self.task_stop_flag)
                        enable_detect_line.data = False
                        self.stop_line_flag = False

                ##### stop node #####
                self.stop_node_flag = self.is_stop_node(self.stop_list, self.current_checkpoint_id)
                if self.stop_node_flag:
                    self.task_stop_flag.data = True
                    self.task_stop_pub.publish(self.task_stop_flag)

                    if self.get_go_signal(self.joy):
                        self.has_stopped = False
                        del self.stop_list[0]
                        self.task_stop_flag.data = False
                        self.task_stop_pub.publish(self.task_stop_flag)

                self.local_goal_updated = False
                self.target_velocity_pub.publish(self.target_velocity)
                self.is_stop_node_flag_publish(self.next_checkpoint_id, self.stop_list)
                prev_task_type = task_type
            else:
                rospy.logwarn_throttle(1, "local_goal is not updated")
            r.sleep()

    def load_task_from_yaml(self):
        with open(self.TASK_LIST_PATH) as file:
            task_data = yaml.safe_load(file)
            self.get_task = True
        return task_data

    def load_stop_list_from_yaml(self):
        with open(self.STOP_LIST_PATH) as file:
            stop_list_from_yaml = yaml.safe_load(file)
            self.get_stop_list = True
        stop_list = []
        for count, stop in enumerate(stop_list_from_yaml['stop_list']):
            stop_list.append(stop['node_id'])
        return stop_list

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

    def local_goal_callback(self, msg):
        self.local_goal = msg
        self.local_goal_updated = True

    def joy_callback(self, msg):
        self.joy = msg
        self.joy_updated = True

    def search_task_from_node_id(self, node0_id, node1_id):
        if self.get_task == True:
            for count, task in enumerate(self.task_data['task']):
                if task['edge']['node0_id'] == node0_id and task['edge']['node1_id'] == node1_id:
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

    def is_stop_node_flag_publish(self, next_node_id, stop_list):
        is_stop_node_flag = Bool()
        if len(stop_list) == 0 or self.get_stop_list == False:
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
