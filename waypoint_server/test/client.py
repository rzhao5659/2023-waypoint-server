from array import array
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from waypoint_server_interfaces.srv import (
    OperateWaypoint,
    CreateWaypoint,
    EditWaypoint,
    MoveWaypoint,
)
from waypoint_server_interfaces.msg import Waypoint, WaypointArray
from std_msgs.msg import Bool
from enum import IntEnum


class Client(Node):
    def __init__(self):
        super().__init__("minimal_client_async")
        self.advance_client = self.create_client(Trigger, "/waypoints/mission/advance")
        self.revert_client = self.create_client(Trigger, "/waypoints/mission/revert")
        self.set_client = self.create_client(OperateWaypoint, "/waypoints/mission/set")
        self.remove_client = self.create_client(OperateWaypoint, "/waypoints/remove")
        self.create_wp_client = self.create_client(CreateWaypoint, "/waypoints/create")
        self.edit_wp_client = self.create_client(EditWaypoint, "/waypoints/edit")
        self.move_client = self.create_client(MoveWaypoint, "/waypoints/mission/move")
        self.clear_mission_client = self.create_client(
            Trigger, "/waypoints/mission/clear"
        )
        self.clear_misc_client = self.create_client(Trigger, "/waypoints/misc/clear")
        self.clear_home_client = self.create_client(Trigger, "/waypoints/home/clear")
        self.clear_all_client = self.create_client(Trigger, "/waypoints/clear")
        self.begin_mission_client = self.create_client(
            Trigger, "/waypoints/mission/begin"
        )
        self.go_home_client = self.create_client(
            OperateWaypoint, "/waypoints/home/begin"
        )

        # Subscribers
        self.home_waypoints_subscriber = self.create_subscription(
            WaypointArray, "/waypoints/home/all", self.home_waypoints_subscriber_cb, 1
        )
        self.misc_waypoints_subscriber = self.create_subscription(
            WaypointArray, "/waypoints/misc/all", self.misc_waypoints_subscriber_cb, 1
        )
        self.mission_waypoints_subscriber = self.create_subscription(
            WaypointArray,
            "/waypoints/mission/all",
            self.mission_waypoints_subscriber_cb,
            1,
        )
        self.ready_subscriber = self.create_subscription(
            Bool, "/waypoints/ready", self.ready_subscriber_cb, 1
        )
        self.curr_waypoint_subscriber = self.create_subscription(
            Waypoint, "/waypoints/current", self.curr_waypoint_subscriber_cb, 1
        )

        # Stores messages received through subscribers.
        self.mission_waypoints_msg = None
        self.home_waypoints_msg = None
        self.misc_waypoints_msg = None
        self.curr_waypoint_msg = None
        self.ready_msg = None

    def send_advance_req(self):
        request_msg = Trigger.Request()
        self.advance_future = self.advance_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.advance_future)
        return self.advance_future.result()

    def send_revert_req(self):
        request_msg = Trigger.Request()
        self.revert_future = self.revert_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.revert_future)
        return self.revert_future.result()

    def send_clear_mission_req(self):
        request_msg = Trigger.Request()
        self.clear_mission_future = self.clear_mission_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.clear_mission_future)
        return self.clear_mission_future.result()

    def send_clear_misc_req(self):
        request_msg = Trigger.Request()
        self.clear_misc_future = self.clear_misc_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.clear_misc_future)
        return self.clear_misc_future.result()

    def send_clear_home_req(self):
        request_msg = Trigger.Request()
        self.clear_home_future = self.clear_home_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.clear_home_future)
        return self.clear_home_future.result()

    def send_clear_all_req(self):
        request_msg = Trigger.Request()
        self.clear_all_future = self.clear_all_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.clear_all_future)
        return self.clear_all_future.result()

    def send_begin_mission_req(self):
        request_msg = Trigger.Request()
        self.begin_mission_future = self.begin_mission_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.begin_mission_future)
        return self.begin_mission_future.result()

    def send_create_wp_req(self, name, latitude, longitude, type_, markers):
        request_msg = CreateWaypoint.Request()
        request_msg.waypoint = Waypoint()
        request_msg.waypoint.name = name
        request_msg.waypoint.latitude = latitude
        request_msg.waypoint.longitude = longitude
        request_msg.waypoint.type = type_
        request_msg.waypoint.markers = markers

        self.create_wp_future = self.create_wp_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.create_wp_future)
        return self.create_wp_future.result()

    def send_edit_wp_req(self, current_name, name, latitude, longitude, type_, markers):
        request_msg = EditWaypoint.Request()
        request_msg.current_name = current_name
        request_msg.waypoint = Waypoint()
        request_msg.waypoint.name = name
        request_msg.waypoint.latitude = latitude
        request_msg.waypoint.longitude = longitude
        request_msg.waypoint.type = type_
        request_msg.waypoint.markers = markers

        self.edit_wp_future = self.edit_wp_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.edit_wp_future)
        return self.edit_wp_future.result()

    def send_move_req(self, index, name):
        request_msg = MoveWaypoint.Request()
        request_msg.index = index
        request_msg.name = name

        self.move_future = self.move_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.move_future)
        return self.move_future.result()

    def send_set_req(self, name):
        request_msg = OperateWaypoint.Request()
        request_msg.name = name

        self.set_future = self.set_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.set_future)
        return self.set_future.result()

    def send_remove_req(self, name):
        request_msg = OperateWaypoint.Request()
        request_msg.name = name

        self.remove_future = self.remove_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.remove_future)
        return self.remove_future.result()

    def send_go_home_req(self, name):
        request_msg = OperateWaypoint.Request()
        request_msg.name = name

        self.go_home_future = self.go_home_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self, self.go_home_future)
        return self.go_home_future.result()

    # Subscribers
    def home_waypoints_subscriber_cb(self, msg):
        self.home_waypoints_msg = [
            Point.from_waypoint(waypoint) for waypoint in msg.waypoints
        ]

    def mission_waypoints_subscriber_cb(self, msg):
        self.mission_waypoints_msg = [
            Point.from_waypoint(waypoint) for waypoint in msg.waypoints
        ]

    def misc_waypoints_subscriber_cb(self, msg):
        self.misc_waypoints_msg = [
            Point.from_waypoint(waypoint) for waypoint in msg.waypoints
        ]

    def curr_waypoint_subscriber_cb(self, msg):
        self.curr_waypoint_msg = Point.from_waypoint(msg)

    def ready_subscriber_cb(self, msg):
        self.ready_msg = msg.data

    # Functions that helps with testing
    def spin_until_msg(self):
        self._clear_msg()
        # It doesn't wait for current waypoint msg to be some value because it can be None
        # intentionally.
        # These other messages are the only ones that can't be None if received.
        while (
            self.home_waypoints_msg is None
            or self.mission_waypoints_msg is None
            or self.misc_waypoints_msg is None
            or self.ready_msg is None
        ):
            rclpy.spin_once(self)

        if self.ready_msg is True:
            # If true, then current waypoint should be published.
            # Spin until we receive a value for it.
            while self.curr_waypoint_msg is None:
                rclpy.spin_once(self)

    def _clear_msg(self):
        self.mission_waypoints_msg = None
        self.home_waypoints_msg = None
        self.home_waypoints_msg = None
        self.curr_waypoint_msg = None
        self.ready_msg = None


class WaypointType(IntEnum):
    CUSTOM = 0
    AUTONOMOUS = 1
    EXTREME_DELIVERY = 2
    HOME = 3
    MISCELLANEOUS = 4


# Mostly the same as source code Point class, except the equal method checks every fields equality.
# Note that by default, any "array" in ROS messages such as int32[] are python arrays.
@dataclass
class Point:
    name: str
    latitude: float
    longitude: float
    type_: int
    markers: array

    def into_waypoint(self) -> Waypoint:
        wp = Waypoint()
        wp.name = self.name
        wp.latitude = self.latitude
        wp.longitude = self.longitude
        wp.type = self.type_
        wp.markers = self.markers
        return wp

    @classmethod
    def from_waypoint(cls, w: Waypoint):
        return cls(w.name, w.latitude, w.longitude, w.type, w.markers)

    def __eq__(self, other: object):
        if isinstance(other, Point):
            if (
                other.name == self.name
                and is_close(other.latitude, self.latitude)
                and is_close(other.longitude, self.longitude)
                and other.type_ == self.type_
                and other.markers == self.markers
            ):
                return True
        return False


def is_close(a, b, tol=0.0001):
    return abs(a - b) < tol
