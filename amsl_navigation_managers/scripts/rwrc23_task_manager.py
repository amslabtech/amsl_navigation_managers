#!/usr/bin/env python3

# 注：rwrc22_task_managerをベースにしています

import math
import subprocess

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse


class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager")
        rospy.loginfo("=== task manager ===")

        self.TASK_LIST_PATH = rospy.get_param("~TASK_LIST_PATH")
        self.STOP_LIST_PATH = rospy.get_param("~STOP_LIST_PATH")
        self.ANNOUNCE_SOUND_PATH = rospy.get_param(
            "~ANNOUNCE_SOUND_PATH", "../sounds/announcement_long.wav"
        )
        self.USE_DETECT_WHITE_LINE = rospy.get_param(
            "~USE_DETECT_WHITE_LINE", False
        )
        self.USE_TRAFFIC_LIGHT = rospy.get_param("~USE_TRAFFIC_LIGHT", False)
        self.STOP_LINE_THRESHOLD = rospy.get_param("~STOP_LINE_THRESHOLD")
        self.debug = rospy.get_param("~debug", False)
        self.start_node_id = rospy.get_param("~start_node_id", 0)
        self.turn_rate = rospy.get_param("~turn_rate", 0.5)
        self.sound_volume = rospy.get_param("~sound_volume", 100)
        self.dwa_target_velocity = rospy.get_param("~dwa_target_velocity", 1.0)
        self.pfp_target_velocity = rospy.get_param("~pfp_target_velocity", 1.0)
        self.detect_line_pfp_target_velocity = rospy.get_param(
            "~detect_line_pfp_target_velocity", 0.3
        )
        self.slow_target_velocity = rospy.get_param(
            "~slow_target_velocity", 0.6
        )
        self.sleep_time_after_finish = rospy.get_param(
            "~sleep_time_after_finish", 0.5
        )

        # mux topic
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "")
        self.cand_traj_topic = rospy.get_param("~cand_traj_topic", "")
        self.sel_traj_topic = rospy.get_param("~sel_traj_topic", "")
        self.footprint_topic = rospy.get_param("~footprint_topic", "")
        self.finish_flag_topic = rospy.get_param("~finish_flag_topic", "")
        self.dwa_cmd_vel = rospy.get_param("~dwa_cmd_vel", "")
        self.dwa_cand_traj = rospy.get_param("~dwa_cand_traj", "")
        self.dwa_sel_traj = rospy.get_param("~dwa_sel_traj", "")
        self.dwa_footprint = rospy.get_param("~dwa_footprint", "")
        self.dwa_finish_flag = rospy.get_param("~dwa_finish_flag", "")
        self.pfp_cmd_vel = rospy.get_param("~pfp_cmd_vel", "")
        self.pfp_cand_traj = rospy.get_param("~pfp_cand_traj", "")
        self.pfp_best_traj = rospy.get_param("~pfp_best_traj", "")
        self.pfp_footprint = rospy.get_param("~pfp_footprint", "")
        self.pfp_finish_flag = rospy.get_param("~pfp_finish_flag", "")

        # params in callback function
        self.current_checkpoint_id = None
        self.next_checkpoint_id = None
        self.target_velocity = Twist()

        # params
        self.task_data = self.load_task_from_yaml()
        self.stop_list = self.load_stop_list_from_yaml()
        self.current_planner = None
        self.checkpoint_list = None
        self.finish_flag = Bool()
        self.skip_mode_flag = Bool()
        self.recovery_mode_flag = Bool()

        # msg update flags
        self.checkpoint_list_subscribed = False

        # ros
        self.current_checkpoint_id_sub = rospy.Subscriber(
            "/current_checkpoint", Int32, self.current_checkpoint_id_callback
        )
        self.next_checkpoint_id_sub = rospy.Subscriber(
            "/next_checkpoint", Int32, self.next_checkpoint_id_callback
        )
        self.checkpoint_sub = rospy.Subscriber(
            "/checkpoint", Int32MultiArray, self.checkpoint_callback
        )
        self.select_topic_sub = rospy.Subscriber(
            "/select_topic", String, self.select_topic_callback
        )
        self.finish_flag_sub = rospy.Subscriber(
            "/local_planner/finish_flag", Bool, self.finish_flag_callback
        )

        self.target_velocity_pub = rospy.Publisher(
            "/target_velocity", Twist, queue_size=1
        )
        self.finish_flag_pub = rospy.Publisher(
            "/finish_flag", Bool, queue_size=1
        )
        self.skip_mode_flag_pub = rospy.Publisher(
            "/skip_mode_flag", Bool, queue_size=1
        )
        self.recovery_mode_flag_pub = rospy.Publisher(
            "/recovery_mode_flag", Bool, queue_size=1
        )
        self.stop_line_detected_server = rospy.Service(
            "/stop_line_detector/stop",
            SetBool,
            self.stop_line_detected_callback,
        )
        self.task_stop_client = rospy.ServiceProxy("/task/stop", SetBool)
        self.stop_line_detector_client = rospy.ServiceProxy(
            "/stop_line_detector/request", SetBool
        )
        self.traffic_light_detector_client = rospy.ServiceProxy(
            "/traffic_light_detector/request", SetBool
        )

    def process(self):
        if self.debug:
            rospy.logwarn("waiting for services")
            rospy.wait_for_service("/task/stop")
            if self.USE_DETECT_WHITE_LINE:
                rospy.wait_for_service("/stop_line_detector/request")
            if self.USE_TRAFFIC_LIGHT:
                rospy.wait_for_service("/traffic_light_detector/request")

        prev_task_type = "init"
        r = rospy.Rate(10)
        self.target_velocity.linear.x = self.dwa_target_velocity
        while not rospy.is_shutdown():
            if self.current_checkpoint_id is None:
                rospy.logwarn_throttle(1, "Checkpoint id is not updated")
                r.sleep()
                continue
            elif self.next_checkpoint_id is None:
                rospy.logwarn_throttle(1, "Next checkpoint id is not updated")
                r.sleep()
                continue
            elif self.checkpoint_list is None:
                rospy.logwarn_throttle(1, "Checkpoint list is not updated")
                r.sleep()
                continue
            elif self.current_planner is None:
                rospy.logwarn_throttle(1, "Cannot get current planner")
                r.sleep()
                continue

            rospy.loginfo_throttle(1, "=====")
            task_type = self.search_task_from_node_id(
                self.current_checkpoint_id, self.next_checkpoint_id
            )
            rospy.loginfo_throttle(1, f"task_type : {task_type}")
            rospy.loginfo_throttle(1, f"planner : {self.current_planner}")
            rospy.loginfo_throttle(
                1, f"current_checkpoint : {self.current_checkpoint_id}"
            )
            rospy.loginfo_throttle(
                1, f"next_checkpoint : {self.next_checkpoint_id}"
            )

            # update task
            if prev_task_type != task_type:
                rospy.logwarn(f"task updated : {task_type}")
                # detect_line
                if task_type == "detect_line" and self.USE_DETECT_WHITE_LINE:
                    try:
                        resp = self.stop_line_detector_client(True)
                        rospy.logwarn(resp.message)
                    except rospy.ServiceException as e:
                        rospy.logwarn(e)
                    self.use_point_follow_planner()
                    self.target_velocity.linear.x = (
                        self.detect_line_pfp_target_velocity
                    )
                else:
                    try:
                        resp = self.stop_line_detector_client(False)
                        rospy.logwarn(resp.message)
                    except rospy.ServiceException as e:
                        rospy.logwarn(e)

                # traffic_light
                if task_type == "traffic_light" and self.USE_TRAFFIC_LIGHT:
                    try:
                        resp = self.traffic_light_detector_client(True)
                        rospy.logwarn(resp.message)
                    except rospy.ServiceException as e:
                        rospy.logwarn(e)
                    self.use_dwa_planner()
                else:
                    try:
                        resp = self.traffic_light_detector_client(False)
                        rospy.logwarn(resp.message)
                    except rospy.ServiceException as e:
                        rospy.logwarn(e)

                # point_follow_planner
                if task_type == "in_line":
                    self.use_point_follow_planner()

                # slow
                if task_type == "slow":
                    self.target_velocity.linear.x = self.slow_target_velocity

                # no task
                if task_type == "":
                    self.use_dwa_planner()

                # skip_mode
                if task_type == "" and not self.is_stop_node(
                    self.stop_list, self.next_checkpoint_id
                ):
                    self.skip_mode_flag.data = True
                else:
                    self.skip_mode_flag.data = False

                # stop node
                if self.is_stop_node(
                    self.stop_list, self.current_checkpoint_id
                ):
                    try:
                        resp = self.task_stop_client(True)
                        rospy.logwarn(resp.message)
                    except rospy.ServiceException as e:
                        rospy.logwarn(e)
                    del self.stop_list[0]

                # recovery_mode
                if task_type == "" or task_type == "slow":
                    self.recovery_mode_flag.data = True
                else:
                    self.recovery_mode_flag.data = False

            # publish
            self.target_velocity_pub.publish(self.target_velocity)
            self.finish_flag_pub.publish(self.finish_flag.data)
            self.skip_mode_flag_pub.publish(self.skip_mode_flag.data)
            self.recovery_mode_flag_pub.publish(self.recovery_mode_flag.data)

            if self.finish_flag.data:
                rospy.sleep(self.sleep_time_after_finish)

            prev_task_type = task_type
            self.finish_flag.data = False

            r.sleep()

    def load_task_from_yaml(self):
        while not rospy.is_shutdown():
            task_data = None
            try:
                with open(self.TASK_LIST_PATH) as file:
                    task_data = yaml.safe_load(file)
                if task_data is not None:
                    break
            except Exception as e:
                rospy.logerr_throttle(5.0, e)
                rospy.sleep(1.0)
        return task_data

    def load_stop_list_from_yaml(self):
        while not rospy.is_shutdown():
            try:
                with open(self.STOP_LIST_PATH) as file:
                    stop_list_from_yaml = yaml.safe_load(file)
                stop_list = []
                for count, stop in enumerate(stop_list_from_yaml["stop_list"]):
                    stop_list.append(stop["node_id"])
                if len(stop_list) != 0:
                    break
            except Exception as e:
                rospy.logerr_throttle(5.0, e)
                rospy.sleep(1.0)
        return stop_list

    def current_checkpoint_id_callback(self, msg):
        self.current_checkpoint_id = int(msg.data)
        self.checkpoint_id_subscribed = True

    def next_checkpoint_id_callback(self, msg):
        self.next_checkpoint_id = int(msg.data)

    def checkpoint_callback(self, msg):
        self.checkpoint_list = msg
        if self.checkpoint_list_subscribed == False:
            self.init_stop_list()
        self.checkpoint_list_subscribed = True

    def select_topic_callback(self, msg):
        self.current_planner = (
            msg.data.split("/")[-2]
            .replace("_planner", "")
            .replace("point_follow", "pfp")
        )

    def finish_flag_callback(self, flag):
        self.finish_flag.data = flag.data

    def stop_line_detected_callback(self, req):
        try:
            resp = self.task_stop_client(req.data)
            rospy.logwarn(resp.message)
        except rospy.ServiceException as e:
            rospy.logwarn(e)
        self.target_velocity.linear.x = self.pfp_target_velocity
        return SetBoolResponse(True, "success")

    def init_stop_list(self):
        for id in self.checkpoint_list.data:
            if id == self.start_node_id:
                return
            if id == self.stop_list[0]:
                if len(self.stop_list) == 1:
                    self.stop_list[0] = -1
                    return
                else:
                    del self.stop_list[0]

    def search_task_from_node_id(self, node0_id, node1_id):
        for count, task in enumerate(self.task_data["task"]):
            if (
                task["edge"]["node0_id"] == node0_id
                and task["edge"]["node1_id"] == node1_id
            ):
                return task["task_type"]
        return ""

    def is_stop_node(self, stop_list, checkpoint_id):
        if len(stop_list) == 0:
            return False
        if stop_list[0] == checkpoint_id:
            return True
        return False

    def announce_once(self):
        announce_cmd = "aplay " + self.ANNOUNCE_SOUND_PATH
        announce_proc = subprocess.call(announce_cmd.split())

    def use_dwa_planner(self):
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.cmd_vel_topic),
                str(self.dwa_cmd_vel),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.cand_traj_topic),
                str(self.dwa_cand_traj),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.sel_traj_topic),
                str(self.dwa_sel_traj),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.footprint_topic),
                str(self.dwa_footprint),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.finish_flag_topic),
                str(self.dwa_finish_flag),
            ]
        )
        self.target_velocity.linear.x = self.dwa_target_velocity

    def use_point_follow_planner(self):
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.cmd_vel_topic),
                str(self.pfp_cmd_vel),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.cand_traj_topic),
                str(self.pfp_cand_traj),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.sel_traj_topic),
                str(self.pfp_best_traj),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.footprint_topic),
                str(self.pfp_footprint),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.finish_flag_topic),
                str(self.pfp_finish_flag),
            ]
        )
        self.target_velocity.linear.x = self.pfp_target_velocity


if __name__ == "__main__":
    task_manager = TaskManager()
    task_manager.process()
