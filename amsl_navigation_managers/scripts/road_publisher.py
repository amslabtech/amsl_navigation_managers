#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional

import rospy
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import Int32

from amsl_navigation_msgs.msg import NodeEdgeMap, Road


@dataclass(frozen=True)
class Params:
    """data class for parameters

    Attributes:
        hz (int): publish rate
        path (str): path to yaml file
    """

    hz: int = 10
    path: str = "road_width.yaml"


class RoadPublisher:
    """class for road publisher

    Attributes:
        _params (Params): parameters
        _road_pub (rospy.Publisher): road publisher
        _current_checkpoint_id_sub (rospy.Subscriber): current checkpoint id
          subscriber
        _next_checkpoint_id_sub (rospy.Subscriber): next checkpoint id
          subscriber
        _node_edge_map_sub (rospy.Subscriber): node edge map subscriber
        _current_checkpoint_id (int): current checkpoint id
        _next_checkpoint_id (int): next checkpoint id
        _node_edge_map (NodeEdgeMap): node edge map
        _widths (dict): road widths
    """

    def __init__(self) -> None:
        """constructor"""
        rospy.init_node("road_publisher")

        self._params = Params(
            hz=rospy.get_param("~hz", 10),
            path=rospy.get_param("~path", "road_width.yaml"),
        )

        self._road_pub = rospy.Publisher(
            "/node_edge_map/road", Road, queue_size=1
        )
        self._current_checkpoint_id_sub = rospy.Subscriber(
            "/current_checkpoint", Int32, self._current_checkpoint_id_callback
        )
        self._next_checkpoint_id_sub = rospy.Subscriber(
            "/next_checkpoint", Int32, self._next_checkpoint_id_callback
        )
        self._node_edge_map_sub = rospy.Subscriber(
            "/node_edge_map/map", NodeEdgeMap, self._node_edge_map_callback
        )

        self._current_checkpoint_id: Optional[int] = None
        self._next_checkpoint_id: Optional[int] = None
        self._node_edge_map: Optional[NodeEdgeMap] = None
        self._widths: dict = self._load_width_from_yaml(self._params.path)

    def _node_edge_map_callback(self, msg: NodeEdgeMap) -> None:
        """node edge map callback

        Args:
            msg (NodeEdgeMap): node edge map
        """
        self._node_edge_map = msg

    def _current_checkpoint_id_callback(self, msg: Int32) -> None:
        """current checkpoint id callback

        Args:
            msg (Int32): current checkpoint id
        """
        self._current_checkpoint_id = msg.data

    def _next_checkpoint_id_callback(self, msg: Int32) -> None:
        """next checkpoint id callback

        Args:
            msg (Int32): next checkpoint id
        """
        self._next_checkpoint_id = msg.data

    def _load_width_from_yaml(self, file_path: str) -> dict:
        """load road widths from yaml file

        Args:
            file_path (str): path to yaml file

        Returns:
            list: road widths
        """
        while not rospy.is_shutdown():
            try:
                with open(file_path, "r") as file:
                    widths: dict = yaml.safe_load(file)
                    if widths is not None:
                        break
            except Exception as e:
                rospy.logerr_throttle(5.0, e)
                rospy.sleep(1.0)

        if "widths" not in widths:
            rospy.logerr("Widths not found in yaml file")
            rospy.signal_shutdown("Widths not found in yaml file")

        rospy.logwarn("Loaded road widths from yaml file")

        return widths

    def process(self) -> None:
        """process"""
        r: rospy.Rate = rospy.Rate(self._params.hz)
        while not rospy.is_shutdown():
            if (
                self._node_edge_map is not None
                and self._current_checkpoint_id is not None
                and self._next_checkpoint_id is not None
            ):
                road: Optional[Road] = self._make_road(
                    self._node_edge_map,
                    self._current_checkpoint_id,
                    self._next_checkpoint_id,
                    self._widths,
                )
                if road is not None:
                    self._road_pub.publish(road)
            else:
                rospy.logwarn_throttle(1.0, "Waiting for data")

            r.sleep()

    def _make_road(
        self,
        map: NodeEdgeMap,
        current_checkpoint: int,
        next_checkpoint: int,
        widths: dict,
    ) -> Optional[Road]:
        """make road

        Args:
            map (NodeEdgeMap): node edge map
            current_checkpoint (int): current checkpoint id
            next_checkpoint (int): next checkpoint id
            widths (dict): road widths

        Returns:
            Optional[Road]: road
        """
        current_right_width: Optional[float] = None
        current_width: Optional[float] = None

        for width in widths["widths"]:
            if (
                width["edge"]["node0_id"] == current_checkpoint
                and width["edge"]["node1_id"] == next_checkpoint
            ):
                current_right_width = width["distance_to_right"]
                current_width = width["width"]
            elif (
                width["edge"]["node0_id"] == next_checkpoint
                and width["edge"]["node1_id"] == current_checkpoint
            ):
                current_right_width = (
                    width["width"] - width["distance_to_right"]
                )
                current_width = width["width"]

        if current_width is None or current_right_width is None:
            return None

        for node in map.nodes:
            if node.id == current_checkpoint:
                current_node_point: Point = node.point
            elif node.id == next_checkpoint:
                next_node_point: Point = node.point

        return Road(
            point0=current_node_point,
            point1=next_node_point,
            length=None,
            distance_to_right=current_right_width,
            width=current_width,
            direction=None,
        )


if __name__ == "__main__":
    road_publisher = RoadPublisher()
    road_publisher.process()
