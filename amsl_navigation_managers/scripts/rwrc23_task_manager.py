#!/usr/bin/env python3
#! coding:utf-8

#注：rwrc22_task_managerをベースにしています
import yaml
import math

import rospy
from std_msgs.msg import Bool, Int32, Int32MultiArray, Float64, String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
import tf2_ros
import subprocess

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')
        rospy.loginfo('=== task manager ===')

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')
        self.STOP_LIST_PATH = rospy.get_param('~STOP_LIST_PATH')
        self.ANNOUNCE_SOUND_PATH = rospy.get_param('~ANNOUNCE_SOUND_PATH', '../sounds/announcement_long.wav')
        self.USE_DETECT_WHITE_LINE = rospy.get_param('~USE_DETECT_WHITE_LINE')
        self.STOP_LINE_THRESHOLD = rospy.get_param('~STOP_LINE_THRESHOLD')
        self.start_node_id = rospy.get_param('~start_node_id', 0)
        self.turn_rate = rospy.get_param('~turn_rate', 0.5)
        # self.enable_announce = rospy.get_param('~enable_announce', False)
        self.enable_announce = rospy.get_param('~enable_announce', False)
        self.sound_volume = rospy.get_param('~sound_volume', 100)
        self.dwa_target_velocity = rospy.get_param('~dwa_target_velocity', 1.0)
        self.pfp_target_velocity = rospy.get_param('~pfp_target_velocity', 1.0)
        self.detect_line_pfp_target_velocity = rospy.get_param('~detect_line_pfp_target_velocity', 0.3)
        self.slow_target_velocity = rospy.get_param('~slow_target_velocity', 0.6)
        self.sleep_time_after_finish = rospy.get_param('~sleep_time_after_finish', 0.5)

        # mux topic
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '')
        self.cand_traj_topic = rospy.get_param('~cand_traj_topic', '')
        self.sel_traj_topic = rospy.get_param('~sel_traj_topic', '')
        self.footprint_topic = rospy.get_param('~footprint_topic', '')
        self.finish_flag_topic = rospy.get_param('~finish_flag_topic', '')
        self.dwa_cmd_vel = rospy.get_param('~dwa_cmd_vel', '')
        self.dwa_cand_traj = rospy.get_param('~dwa_cand_traj', '')
        self.dwa_sel_traj = rospy.get_param('~dwa_sel_traj', '')
        self.dwa_footprint = rospy.get_param('~dwa_footprint', '')
        self.dwa_finish_flag = rospy.get_param('~dwa_finish_flag', '')
        self.pfp_cmd_vel = rospy.get_param('~pfp_cmd_vel', '')
        self.pfp_cand_traj = rospy.get_param('~pfp_cand_traj', '')
        self.pfp_best_traj = rospy.get_param('~pfp_best_traj', '')
        self.pfp_footprint = rospy.get_param('~pfp_footprint', '')
        self.pfp_finish_flag = rospy.get_param('~pfp_finish_flag', '')

        # params in callback function
        self.current_checkpoint_id = self.next_checkpoint_id = -1
        self.stop_line_flag = False
        self.joy = Joy()
        self.target_velocity = Twist()

        # params
        self.get_task = False
        self.task_data = self.load_task_from_yaml()
        self.reached_checkpoint = False
        self.get_stop_list = False
        self.stop_list = self.load_stop_list_from_yaml()
        self.cross_traffic_light_flag = False
        self.has_stopped = False
        self.start_announce_flag = False
        self.announce_pid = 0
        self.current_planner = ''
        self.task_stop_flag = Bool()
        self.stop_node_flag = False
        self.checkpoint_list = Int32MultiArray()
        self.finish_flag = Bool()
        self.skip_mode_flag = Bool()
        self.recovery_mode_flag = Bool()
        self.enable_detect_line = Bool()

        # msg update flags
        self.checkpoint_list_subscribed = False
        self.checkpoint_id_subscribed = False
        self.joy_updated = False
        self.stop_node_flag_updated = False
        self.exec_traffic_light_detector = False

        # ros
        self.current_checkpoint_id_sub = rospy.Subscriber('/current_checkpoint', Int32, self.current_checkpoint_id_callback)
        self.next_checkpoint_id_sub = rospy.Subscriber('/next_checkpoint', Int32, self.next_checkpoint_id_callback)
        self.stop_line_flag_sub = rospy.Subscriber('/stop_line_detector/stop_flag', Bool, self.stop_line_flag_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cross_traffic_light_flag_sub = rospy.Subscriber('/cross_traffic_light_flag', Bool, self.cross_traffic_light_flag_callback)
        self.checkpoint_sub = rospy.Subscriber('/checkpoint', Int32MultiArray, self.checkpoint_callback)
        self.select_topic_sub = rospy.Subscriber('/select_topic', String, self.select_topic_callback)
        self.finish_flag_sub = rospy.Subscriber('/local_planner/finish_flag', Bool, self.finish_flag_callback)

        self.detect_line_flag_pub = rospy.Publisher('~request_detect_line', Bool, queue_size=1)
        self.detect_traffic_light_flag_pub = rospy.Publisher('/request_detect_traffic_light', Bool, queue_size=1)
        self.target_velocity_pub = rospy.Publisher('/target_velocity', Twist, queue_size=1)
        self.task_stop_pub = rospy.Publisher('/task/stop', Bool, queue_size=1)
        self.finish_flag_pub = rospy.Publisher('/finish_flag', Bool, queue_size=1)
        self.skip_mode_flag_pub = rospy.Publisher('/skip_mode_flag', Bool, queue_size=1)
        self.recovery_mode_flag_pub = rospy.Publisher('/recovery_mode_flag', Bool, queue_size=1)

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
        while not rospy.is_shutdown():
            if self.checkpoint_id_subscribed:
                rospy.loginfo_throttle(1, '=====')
                task_type = self.search_task_from_node_id(self.current_checkpoint_id, self.next_checkpoint_id)
                rospy.loginfo_throttle(1, f'task_type : {task_type}')
                rospy.loginfo_throttle(1, f'planner : {self.current_planner}')
                rospy.loginfo_throttle(1, f'current_checkpoint : {self.current_checkpoint_id}')
                rospy.loginfo_throttle(1, f'next_checkpoint : {self.next_checkpoint_id}')

                ##### enable white line detector #####
                if task_type == 'detect_line' and prev_task_type != task_type and self.USE_DETECT_WHITE_LINE:
                    self.stop_line_flag = False
                    self.enable_detect_line.data = True
                    self.use_point_follow_planner()
                    self.target_velocity.linear.x = self.detect_line_pfp_target_velocity

                if task_type != 'detect_line':
                    self.enable_detect_line.data = False

                ##### traffic_light #####
                if task_type == 'traffic_light':
                    self.exec_traffic_light_detector = True
                    if prev_task_type != task_type:
                        self.use_dwa_planner()
                else:
                    self.exec_traffic_light_detector = False

                ##### point_follow_planner #####
                if task_type == 'in_line' and prev_task_type != task_type:
                    self.use_point_follow_planner()

                ##### slow #####
                if task_type == 'slow' and prev_task_type != task_type:
                    self.target_velocity.linear.x = self.slow_target_velocity

                ##### no task #####
                if task_type == '' and prev_task_type != task_type:
                    self.use_dwa_planner()

                ##### skip_mode #####
                if task_type == '' and not self.is_stop_node(self.stop_list, self.next_checkpoint_id):
                    self.skip_mode_flag.data = True

                ##### no task #####
                if task_type == '':
                    self.recovery_mode_flag.data = True

                ##### stop at white line #####
                if self.enable_detect_line.data:
                    if self.stop_line_flag:
                        self.has_stopped = True

                    if self.has_stopped:
                        self.task_stop_flag.data = True
                        self.task_stop_pub.publish(self.task_stop_flag)

                    if self.has_stopped and self.get_go_signal(self.joy):
                        self.has_stopped = False
                        self.task_stop_flag.data = False
                        self.task_stop_pub.publish(self.task_stop_flag)
                        self.enable_detect_line.data = False
                        self.stop_line_flag = False
                        self.finish_flag.data = True
                        self.target_velocity.linear.x = self.pfp_target_velocity

                ##### stop node #####
                if self.stop_node_flag_updated:
                    rospy.logwarn_throttle(1, '=== stop_node ===')
                    self.task_stop_flag.data = True
                    self.task_stop_pub.publish(self.task_stop_flag)

                    if self.get_go_signal(self.joy) or self.cross_traffic_light_flag:
                        self.has_stopped = False
                        self.stop_node_flag_updated = False
                        del self.stop_list[0]
                        self.task_stop_flag.data = False
                        self.task_stop_pub.publish(self.task_stop_flag)

                self.target_velocity_pub.publish(self.target_velocity)
                self.detect_line_flag_pub.publish(self.enable_detect_line)
                self.detect_traffic_light_flag_pub.publish(self.exec_traffic_light_detector)
                self.finish_flag_pub.publish(self.finish_flag.data)
                self.skip_mode_flag_pub.publish(self.skip_mode_flag.data)
                self.recovery_mode_flag_pub.publish(self.recovery_mode_flag.data)
                if self.finish_flag.data:
                    rospy.sleep(self.sleep_time_after_finish)

                prev_task_type = task_type
                self.skip_mode_flag.data = False
                self.recovery_mode_flag.data = False
                self.finish_flag.data = False
            else:
                rospy.logwarn_throttle(1, 'Checkpoint id is not updated')
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
        self.checkpoint_id_subscribed = True
        self.stop_node_flag = self.is_stop_node(self.stop_list, self.current_checkpoint_id)
        if self.stop_node_flag:
            self.stop_node_flag_updated = True

    def next_checkpoint_id_callback(self, msg):
        self.next_checkpoint_id = int(msg.data)

    def stop_line_flag_callback(self, flag):
        self.stop_line_flag = flag.data

    def cross_traffic_light_flag_callback(self, flag):
        self.cross_traffic_light_flag = flag.data

    def joy_callback(self, msg):
        self.joy = msg
        self.joy_updated = True

    def checkpoint_callback(self, msg):
        self.checkpoint_list = msg
        if self.checkpoint_list_subscribed == False:
            self.init_stop_list()
        self.checkpoint_list_subscribed = True

    def select_topic_callback(self, msg):
        self.current_planner = msg.data.split('/')[-2].replace('_planner', '').replace('point_follow', 'pfp')

    def finish_flag_callback(self, flag):
        self.finish_flag.data = flag.data

    def init_stop_list(self):
        for id in self.checkpoint_list.data:
            if id == self.start_node_id:
                return
            if id == self.stop_list[0]:
                del self.stop_list[0]

    def search_task_from_node_id(self, node0_id, node1_id):
        if self.get_task == True:
            for count, task in enumerate(self.task_data['task']):
                if task['edge']['node0_id'] == node0_id and task['edge']['node1_id'] == node1_id:
                    return task['task_type']
            return ''
        else:
            return ''

    def is_stop_node(self, stop_list, checkpoint_id):
        if self.get_stop_list == False:
            return False
        if len(stop_list) == 0:
            return False
        if stop_list[0] == checkpoint_id:
            return True
        return False

    def get_go_signal(self, joy):
        if self.joy_updated == True:
            self.joy_updated = False
            if joy.buttons[1] == 1:
                return True
            return False
        else:
            return False

    def set_sound_volume(self):
        if self.enable_announce == True :
            volume_cmd = 'amixer -c1 sset Speaker ' + str(self.sound_volume) + '%,' + str(self.sound_volume) + '% unmute'
            subprocess.Popen(volume_cmd.split())

    def announce_once(self):
        if self.enable_announce == True :
            announce_cmd = 'aplay ' + self.ANNOUNCE_SOUND_PATH
            announce_proc = subprocess.call(announce_cmd.split())

    def use_dwa_planner(self):
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.cmd_vel_topic),str(self.dwa_cmd_vel)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.cand_traj_topic),str(self.dwa_cand_traj)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.sel_traj_topic),str(self.dwa_sel_traj)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.footprint_topic),str(self.dwa_footprint)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.finish_flag_topic),str(self.dwa_finish_flag)])
        self.target_velocity.linear.x = self.dwa_target_velocity

    def use_point_follow_planner(self):
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.cmd_vel_topic),str(self.pfp_cmd_vel)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.cand_traj_topic),str(self.pfp_cand_traj)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.sel_traj_topic),str(self.pfp_best_traj)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.footprint_topic),str(self.pfp_footprint)])
        subprocess.Popen(['rosrun','topic_tools','mux_select',str(self.finish_flag_topic),str(self.pfp_finish_flag)])
        self.target_velocity.linear.x = self.pfp_target_velocity

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
