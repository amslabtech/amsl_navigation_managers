#!/usr/bin/env python3

# 注：rwrc22_task_managerをベースにしています

import subprocess
from dataclasses import dataclass

import rospy
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger

from amsl_navigation_msgs.msg import Edge


@dataclass(frozen=True)
class TaskManagerParam:
    task_list_path: str
    announce_sound_path: str
    use_detect_white_line: bool
    use_traffic_light: bool
    start_node_id: int
    debug: bool


@dataclass(frozen=True)
class TopicConfig:
    cmd_vel_topic: str
    cand_traj_topic: str
    sel_traj_topic: str
    footprint_topic: str
    finish_flag_topic: str


@dataclass(frozen=True)
class PlannerConfig:
    target_velocity: float
    cmd_vel: str
    cand_traj: str
    sel_traj: str
    footprint: str
    finish_flag: str


@dataclass(frozen=True)
class PlannerParam:
    detect_line_pfp_target_velocity: float
    slow_target_velocity: float
    sleep_time_after_finish: float


@dataclass
class TaskManagerState:
    task_type: str = "_init"
    planner: str = ""
    assigned_planner: str = ""
    edge: Edge = None

    def print(self):
        rospy.loginfo_throttle(1, "=====")
        rospy.loginfo_throttle(1, f"task_type : {self.task_type}")
        rospy.loginfo_throttle(1, f"planner : {self.planner}")
        if self.edge is not None:
            rospy.loginfo_throttle(
                1, f"current_checkpoint : {self.edge.node0_id}"
            )
            rospy.loginfo_throttle(
                1, f"next_checkpoint : {self.edge.node1_id}"
            )


class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager")
        rospy.loginfo("=== task manager ===")

        # Param
        self.load_param()
        self.state = TaskManagerState()
        self.prev_edge = None
        self.checkpoint_list = None
        self.task_list = self.load_task_from_yaml()
        self.finish_flag = Bool()
        self.target_velocity = Twist()
        self.target_velocity.linear.x = self.dwa_config.target_velocity

        if not self.task_manager_param.debug:
            self.wait_for_service()

        # Publisher
        self.target_velocity_pub = rospy.Publisher(
            "/target_velocity", Twist, queue_size=1
        )
        # Subscriber
        self.checkpoint_sub = rospy.Subscriber(
            "/checkpoint", Int32MultiArray, self.checkpoint_callback
        )
        self.edge_sub = rospy.Subscriber("/edge", Edge, self.edge_callback)
        self.select_topic_sub = rospy.Subscriber(
            "/select_topic", String, self.select_topic_callback
        )
        self.finish_flag_sub = rospy.Subscriber(
            "/local_planner/finish_flag", Bool, self.finish_flag_callback
        )
        # Service
        self.stop_line_detected_server = rospy.Service(
            "/stop_line_detector/stop",
            SetBool,
            self.stop_line_detected_callback,
        )
        self.checkpoint_update_client = rospy.ServiceProxy(
            "/local_goal_creator/update", Trigger
        )
        self.recovery_mode_client = rospy.ServiceProxy(
            "/recovery/available", SetBool
        )
        self.skip_mode_client = rospy.ServiceProxy(
            "/local_goal_creator/skip_mode/avaliable", SetBool
        )
        self.stop_line_detector_client = rospy.ServiceProxy(
            "/stop_line_detector/request", SetBool
        )
        self.task_stop_client = rospy.ServiceProxy("/task/stop", SetBool)
        self.traffic_light_detector_client = rospy.ServiceProxy(
            "/traffic_light_detector/request", SetBool
        )

    def load_param(self):
        # TaskManagerParams
        self.task_manager_param = TaskManagerParam(
            task_list_path=rospy.get_param("~TASK_LIST_PATH"),
            announce_sound_path=rospy.get_param(
                "~ANNOUNCE_SOUND_PATH", "../sounds/announcement_long.wav"
            ),
            use_detect_white_line=rospy.get_param(
                "~USE_DETECT_WHITE_LINE", False
            ),
            use_traffic_light=rospy.get_param("~USE_TRAFFIC_LIGHT", False),
            start_node_id=rospy.get_param("~start_node_id", 0),
            debug=rospy.get_param("~debug", False),
        )

        # PlannerParams
        self.topic_config = TopicConfig(
            cmd_vel_topic=rospy.get_param("~cmd_vel_topic", ""),
            cand_traj_topic=rospy.get_param("~cand_traj_topic", ""),
            sel_traj_topic=rospy.get_param("~sel_traj_topic", ""),
            footprint_topic=rospy.get_param("~footprint_topic", ""),
            finish_flag_topic=rospy.get_param("~finish_flag_topic", ""),
        )
        self.dwa_config = PlannerConfig(
            target_velocity=rospy.get_param("~dwa_target_velocity", 1.0),
            cmd_vel=rospy.get_param("~dwa_cmd_vel", ""),
            cand_traj=rospy.get_param("~dwa_cand_traj", ""),
            sel_traj=rospy.get_param("~dwa_sel_traj", ""),
            footprint=rospy.get_param("~dwa_footprint", ""),
            finish_flag=rospy.get_param("~dwa_finish_flag", ""),
        )
        self.pfp_config = PlannerConfig(
            target_velocity=rospy.get_param("~pfp_target_velocity", 1.0),
            cmd_vel=rospy.get_param("~pfp_cmd_vel", ""),
            cand_traj=rospy.get_param("~pfp_cand_traj", ""),
            sel_traj=rospy.get_param("~pfp_best_traj", ""),
            footprint=rospy.get_param("~pfp_footprint", ""),
            finish_flag=rospy.get_param("~pfp_finish_flag", ""),
        )
        self.elevator_config = PlannerConfig(
            target_velocity=rospy.get_param("~pfp_target_velocity", 1.0),
            cmd_vel=rospy.get_param("~elevator_cmd_vel", ""),
            cand_traj=rospy.get_param("~pfp_cand_traj", ""),
            sel_traj=rospy.get_param("~pfp_best_traj", ""),
            footprint=rospy.get_param("~pfp_footprint", ""),
            finish_flag=rospy.get_param("~elevator_finish_flag", ""),
        )
        self.planner_param = PlannerParam(
            detect_line_pfp_target_velocity=rospy.get_param(
                "~detect_line_pfp_target_velocity", 0.3
            ),
            slow_target_velocity=rospy.get_param("~slow_target_velocity", 0.6),
            sleep_time_after_finish=rospy.get_param(
                "~sleep_time_after_finish", 0.5
            ),
        )

    def load_task_from_yaml(self):
        while not rospy.is_shutdown():
            task_list = None
            try:
                with open(self.task_manager_param.task_list_path) as file:
                    task_list = yaml.safe_load(file)
                if task_list is not None:
                    break
            except Exception as e:
                rospy.logerr_throttle(5.0, e)
                rospy.sleep(1.0)
        return task_list

    def wait_for_service(self):
        rospy.logwarn("waiting for services")
        rospy.wait_for_service("/recovery/available")
        rospy.wait_for_service("/local_goal_creator/skip_mode/avaliable")
        rospy.wait_for_service("/local_goal_creator/update")
        if self.task_manager_param.use_detect_white_line:
            rospy.wait_for_service("/stop_line_detector/request")
        rospy.wait_for_service("/task/stop")
        if self.task_manager_param.use_traffic_light:
            rospy.wait_for_service("/traffic_light_detector/request")

    def checkpoint_callback(self, msg):
        self.checkpoint_list = msg

    def edge_callback(self, msg):
        self.state.edge = msg

        if self.checkpoint_list is None:
            rospy.logwarn_throttle(1, "Checkpoint list is not updated")
            return

        if (
            self.prev_edge is None
            or self.state.edge.node0_id != self.prev_edge.node0_id
        ):
            self.state.task_type = self.search_task_from_node_id(
                self.state.edge
            )
            self.update_task(self.state.task_type)
            self.prev_edge = self.state.edge

    def select_topic_callback(self, msg):
        self.state.planner = (
            msg.data.split("/")[-2]
            .replace("_planner", "")
            .replace("point_follow", "pfp")
        )

    def finish_flag_callback(self, flag):
        self.finish_flag.data = flag.data

    def stop_line_detected_callback(self, req):
        self.service_call(self.task_stop_client, req.data)
        self.target_velocity.linear.x = self.pfp_config.target_velocity
        return SetBoolResponse(True, "success")

    def update_task(self, task_type):
        rospy.logwarn(f"task updated : {task_type}")

        # stop
        if task_type == "stop":
            if self.state.assigned_planner != "":
                self.select_planner(self.state.assigned_planner)
            self.service_call(self.task_stop_client, True)

        # detect_line
        if self.task_manager_param.use_detect_white_line:
            if task_type == "detect_line":
                self.service_call(self.stop_line_detector_client, True)
                self.select_planner("pfp")
                self.target_velocity.linear.x = (
                    self.planner_param.detect_line_pfp_target_velocity
                )
            else:
                self.service_call(self.stop_line_detector_client, False)

        # traffic_light
        if self.task_manager_param.use_traffic_light:
            if task_type == "traffic_light":
                self.service_call(self.task_stop_client, True)
                self.service_call(self.traffic_light_detector_client, True)
                self.select_planner("dwa")
            else:
                self.service_call(self.traffic_light_detector_client, False)

        # point_follow_planner
        if task_type == "in_line":
            self.select_planner("pfp")

        # elevator_task
        if task_type == "elevator":
            self.select_planner("elevator")

        # slow
        if task_type == "slow":
            self.target_velocity.linear.x = (
                self.planner_param.slow_target_velocity
            )

        # skip_mode
        if task_type == "" and not self.is_stop_node(self.state.edge.node0_id):
            self.service_call(self.skip_mode_client, True)
        else:
            self.service_call(self.skip_mode_client, False)

        # recovery_mode
        if (
            task_type == "" or task_type == "slow" or task_type == "stop"
        ) and self.state.assigned_planner != "pfp":
            self.service_call(self.recovery_mode_client, True)
        else:
            self.service_call(self.recovery_mode_client, False)

        # no task
        if task_type == "":
            self.select_planner("dwa")

    def search_task_from_node_id(self, edge):
        self.state.assigned_planner = ""
        for count, task in enumerate(self.task_list["task"]):
            if (
                task["edge"]["node0_id"] == edge.node0_id
                and task["edge"]["node1_id"] == edge.node1_id
            ):
                if "planner" in task:
                    self.state.assigned_planner = task["planner"]
                return task["task_type"]
        return ""

    def service_call(self, service_name, req=None):
        while not rospy.is_shutdown():
            try:
                if req is None:
                    resp = service_name()
                    rospy.logwarn(resp.message)
                else:
                    resp = service_name(req)
                    rospy.logwarn(resp.message)
                break
            except rospy.ServiceException as e:
                rospy.logwarn(e)
                rospy.sleep(0.5)

    def is_stop_node(self, node_id):
        for count, task in enumerate(self.task_list["task"]):
            if (
                task["edge"]["node0_id"] == node_id
                and task["task_type"] == "stop"
            ):
                return True

        return False

    def announce_once(self):
        announce_cmd = "aplay " + self.task_manager_param.announce_sound_path
        announce_proc = subprocess.call(announce_cmd.split())

    def select_planner(self, planner_name: str):
        if planner_name == "dwa":
            self.select_topic(self.dwa_config)
        elif planner_name == "pfp":
            self.select_topic(self.pfp_config)
        elif planner_name == "elevator":
            self.select_topic(self.elevator_config)
        else:
            rospy.logwarn("Invalid planner")

    def select_topic(self, planner_config: PlannerConfig):
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.topic_config.cmd_vel_topic),
                str(planner_config.cmd_vel),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.topic_config.cand_traj_topic),
                str(planner_config.cand_traj),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.topic_config.sel_traj_topic),
                str(planner_config.sel_traj),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.topic_config.footprint_topic),
                str(planner_config.footprint),
            ]
        )
        subprocess.Popen(
            [
                "rosrun",
                "topic_tools",
                "mux_select",
                str(self.topic_config.finish_flag_topic),
                str(planner_config.finish_flag),
            ]
        )
        self.target_velocity.linear.x = planner_config.target_velocity

    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.state.print()
            self.target_velocity_pub.publish(self.target_velocity)

            if self.finish_flag.data:
                self.service_call(self.checkpoint_update_client)
                rospy.sleep(self.planner_param.sleep_time_after_finish)
                self.finish_flag.data = False
            else:
                r.sleep()


if __name__ == "__main__":
    task_manager = TaskManager()
    task_manager.process()
