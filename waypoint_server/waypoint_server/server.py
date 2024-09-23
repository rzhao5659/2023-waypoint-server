#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from array import array
from typing import List, Optional, Tuple
from geodesy import utm
from tf2_ros import TransformBroadcaster
import sqlite3
from waypoint_server.waypoint import WaypointType, Point

from waypoint_server_interfaces.msg import Waypoint, WaypointArray
from waypoint_server_interfaces.srv import (
    OperateWaypoint,
    MoveWaypoint,
    CreateWaypoint,
    EditWaypoint,
)
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped


class WaypointServer(Node):
    """
    Server that manages all waypoints.

    Internally, these are stored in three separate list:
        - Mission waypoints is an ordered list of waypoints that the rover needs to reach
          as part of the mission. Contains Extreme Delivery, Autonomous, and Custom type waypoints.
        - Home waypoints is an unordered list of waypoints that represent homes / bases.
        - Miscellaneous waypoints is an unordered list of waypoints that is of interest only
          to the operator. The rover don't need to reach these waypoints.
    """

    def __init__(self, disable_tf=False):
        super().__init__("waypoint_server")

        self.mission_waypoints: List[Point] = []
        self.misc_waypoints: List[Point] = []
        self.home_waypoints: List[Point] = []
        self.mission_started: bool = False
        self.current_mission_waypoint_index = None
        self.current_waypoint = None

        # Services:
        # Services that moves the mission waypoint index. The current waypoint moves accordingly.
        self.advance_mission_waypoint_srv = self.create_service(
            Trigger, "/waypoints/mission/advance", self.advance_mission_waypoint
        )
        self.revert_mission_waypoint_srv = self.create_service(
            Trigger, "/waypoints/mission/revert", self.revert_mission_waypoint
        )
        self.set_mission_waypoint_srv = self.create_service(
            OperateWaypoint, "/waypoints/mission/set", self.set_mission_waypoint
        )
        # Services that changes the content of waypoint lists.
        # The current waypoint may be changed implicitly as a result of some operations.
        self.create_waypoint_srv = self.create_service(
            CreateWaypoint, "/waypoints/create", self.create_waypoint
        )
        self.edit_waypoint_srv = self.create_service(
            EditWaypoint, "/waypoints/edit", self.edit_waypoint
        )
        self.move_mission_waypoint_srv = self.create_service(
            MoveWaypoint, "/waypoints/mission/move", self.move_mission_waypoint
        )
        self.remove_waypoint_srv = self.create_service(
            OperateWaypoint, "/waypoints/remove", self.remove_waypoint
        )
        self.clear_mission_waypoints_srv = self.create_service(
            Trigger, "/waypoints/mission/clear", self.clear_mission_waypoints
        )
        self.clear_misc_waypoints_srv = self.create_service(
            Trigger, "/waypoints/misc/clear", self.clear_misc_waypoints
        )
        self.clear_home_waypoints_srv = self.create_service(
            Trigger, "/waypoints/home/clear", self.clear_home_waypoints
        )
        self.clear_all_srv = self.create_service(
            Trigger, "/waypoints/clear", self.clear_all
        )
        # Services that changes operation mode of rover:
        # The rover may be going home, progressing mission. Initially its doing nothing.
        # The current waypoint is changed accordingly.
        self.begin_mission_srv = self.create_service(
            Trigger, "/waypoints/mission/begin", self.begin_mission
        )
        self.go_home_srv = self.create_service(
            OperateWaypoint, "/waypoints/home/begin", self.go_home
        )

        # Publishers:
        self.current_publisher = self.create_publisher(
            Waypoint, "/waypoints/current", 1
        )
        # Ready publisher tells whether current publisher is publishing anything.
        # If it's not, then it means there is no current waypoint.
        self.ready_publisher = self.create_publisher(Bool, "/waypoints/ready", 1)
        self.home_waypoints_publisher = self.create_publisher(
            WaypointArray, "/waypoints/home/all", 1
        )
        self.misc_waypoints_publisher = self.create_publisher(
            WaypointArray, "/waypoints/misc/all", 1
        )
        self.mission_waypoints_publisher = self.create_publisher(
            WaypointArray, "/waypoints/mission/all", 1
        )

        # TF broadcaster (UTM->Waypoint)
        # This parameter disables the TF broadcaster for testing purposes.
        # The package used to transform GPS to UTM coords throws error for arbitrary (lat, long)
        self.declare_parameter("disable_tf", disable_tf)
        if self.get_parameter("disable_tf").value is False:
            self.waypoints_tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.declare_parameter("publish_period", 1)
        timer_period = self.get_parameter("publish_period").value
        self.timer = self.create_timer(timer_period, self.publish_timer_cb)

        # Persistence through a sqlite3 local database
        self.declare_parameter("persist_path", "")
        self.db_path = self.get_parameter("persist_path").value
        if self.db_path != "":
            try:
                # Sqlite3 Setup
                # Tells sqlite3 how to stringify an "array" type object.
                # Required when inserting waypoints into the database.
                def sqlite_adapt_array(arr: array):
                    sep = ";"
                    return sep.join((str(number) for number in arr))

                sqlite3.register_adapter(array, sqlite_adapt_array)

                # Connects to the database. If it doesn't exist, it creates one.
                self.db_connection = sqlite3.connect(self.db_path)
                self.db_cursor = self.db_connection.cursor()

                # Create a new table for waypoints if it doesn't exist
                result = self.db_cursor.execute(
                    "SELECT name FROM sqlite_master WHERE name='waypoints'"
                )
                if result.fetchone() is None:
                    self.db_cursor.execute(
                        (
                            "CREATE TABLE waypoints(index_arr, list_type, name, latitude,"
                            "longitude, type, markers)"
                        )
                    )
                # Load values from database to the waypoint lists.
                self._load_waypoints_from_db()
                self.get_logger().info(
                    f"Loaded waypoints from the database (PATH = {self.db_path})."
                )
            except Exception:
                self.get_logger().error("Invalid persist path.")

        self.get_logger().info("Waypoint server has started.")

    def publish_timer_cb(self):
        """Publish all topics and transforms in this callback function."""
        # Ready Publisher
        is_ready = self.current_waypoint is not None
        bool_msg = Bool()
        bool_msg.data = is_ready
        self.ready_publisher.publish(bool_msg)

        # Current Waypoint Publisher
        if is_ready:
            self.current_publisher.publish(self.current_waypoint.into_waypoint())

        # All Waypoints Publisher
        waypoints_msg = WaypointArray()
        waypoints_msg.waypoints = [p.into_waypoint() for p in self.home_waypoints]
        self.home_waypoints_publisher.publish(waypoints_msg)

        waypoints_msg.waypoints = [p.into_waypoint() for p in self.misc_waypoints]
        self.misc_waypoints_publisher.publish(waypoints_msg)

        waypoints_msg.waypoints = [p.into_waypoint() for p in self.mission_waypoints]
        self.mission_waypoints_publisher.publish(waypoints_msg)

        # Transform Broadcaster: position of waypoints relative to UTM.
        if self.get_parameter("disable_tf").value is False:
            for waypoint in self.mission_waypoints + self.home_waypoints:
                self._create_and_send_transform(waypoint)

    def advance_mission_waypoint(self, _, response) -> None:
        """
        Set the current waypoint to be the next mission waypoint.

        Note: Only works if mission is in progress.
        """
        response.success = False
        # The mission can't begin with empty mission waypoint list, so it includes the case of
        # empty waypoint list.
        if self.mission_started is False:
            error_msg = "Can't advance waypoint: Needs to start mission."
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        if self.current_mission_waypoint_index == len(self.mission_waypoints) - 1:
            error_msg = "Can't advance waypoint: This is the last waypoint"
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response

        previous_waypoint = self.current_waypoint
        self.current_mission_waypoint_index += 1
        self.current_waypoint = self.mission_waypoints[
            self.current_mission_waypoint_index
        ]

        success_msg = (
            f"Success: Advancing waypoint from {previous_waypoint}"
            f"to {self.current_waypoint}"
        )
        self.get_logger().info(success_msg)
        response.success = True
        response.message = success_msg
        return response

    def revert_mission_waypoint(self, _, response) -> None:
        """
        Set the current waypoint to be the previous mission waypoint.

        Note: Only works if mission is in progress.
        """
        response.success = False
        # The mission can't begin with empty mission waypoint list, so it includes the case of
        # empty waypoint list.
        if self.mission_started is False:
            error_msg = "Can't revert waypoint: Needs to start mission."
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response
        if self.current_mission_waypoint_index == 0:
            error_msg = "Can't revert waypoint: No waypoint before this"
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response

        previous_waypoint = self.current_waypoint
        self.current_mission_waypoint_index -= 1
        self.current_waypoint = self.mission_waypoints[
            self.current_mission_waypoint_index
        ]

        success_msg = (
            f"Success: Reverting waypoint from {previous_waypoint}"
            f"to {self.current_waypoint}"
        )
        self.get_logger().info(success_msg)
        response.success = True
        response.message = success_msg
        return response

    def set_mission_waypoint(self, request, response) -> None:
        """
        Set the current waypoint to be the mission waypoint with the given name.

        Note: Only works if mission is in progress.
        """
        response.success = False
        # The mission can't begin with empty mission waypoint list, so it includes the case of
        # empty waypoint list.
        if self.mission_started is False:
            error_msg = "Can't set waypoint: Needs to start mission."
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response

        new_mission_waypoint_index = self._find_waypoint(
            request.name, self.mission_waypoints
        )
        if new_mission_waypoint_index is None:
            error_msg = "Can't set waypoint: No waypoint with such name."
            self.get_logger().warn(error_msg)
            response.message = error_msg
            return response

        prev_waypoint = self.current_waypoint
        self.current_mission_waypoint_index = new_mission_waypoint_index
        self.current_waypoint = self.mission_waypoints[new_mission_waypoint_index]

        success_msg = (
            f"Success: Setting waypoint from {prev_waypoint} to {self.current_waypoint}"
        )
        self.get_logger().info(success_msg)
        response.success = True
        response.message = success_msg
        return response

    def clear_mission_waypoints(self, _, response) -> None:
        """Clear mission waypoint list and its tracker."""
        MISSION_WAYPOINT_TYPE = [
            WaypointType.AUTONOMOUS,
            WaypointType.EXTREME_DELIVERY,
            WaypointType.CUSTOM,
        ]

        # Clear the current waypoint as well if it's a mission waypoint type.
        if (
            self.current_waypoint is not None
            and self.current_waypoint.type_ in MISSION_WAYPOINT_TYPE
        ):
            self.current_waypoint = None
            self.mission_started = False

        self.current_mission_waypoint_index = None
        self.mission_waypoints = []

        self.get_logger().info("Success: Cleared mission waypoints")
        response.success = True
        response.message = "Success: Cleared mission waypoints"

        # If persistence feature is enabled, update the database.
        if self.db_path != "":
            self.db_cursor.execute("DELETE FROM waypoints WHERE type BETWEEN 0 AND 2")
            self.db_connection.commit()
        return response

    def clear_misc_waypoints(self, _, response) -> None:
        """Clear miscellaneous waypoint list."""
        self.misc_waypoints = []

        self.get_logger().info("Success: Cleared miscellaneous waypoints")
        response.success = True
        response.message = "Success: Cleared miscellaneous waypoints"

        # If persistence feature is enabled, update the database.
        if self.db_path != "":
            self.db_cursor.execute("DELETE FROM waypoints WHERE type = 4")
            self.db_connection.commit()
        return response

    def clear_home_waypoints(self, _, response) -> None:
        """Clear home waypoint list."""
        if (
            self.current_waypoint is not None
            and self.current_waypoint.type_ == WaypointType.HOME
        ):
            self.current_waypoint = None

        self.home_waypoints = []

        self.get_logger().info("Success: Cleared home waypoints")
        response.success = True
        response.message = "Success: Cleared home waypoints"

        # If persistence feature is enabled, update the database.
        if self.db_path != "":
            self.db_cursor.execute("DELETE FROM waypoints WHERE type = 3")
            self.db_connection.commit()
        return response

    def clear_all(self, _, response) -> None:
        """Clear all waypoints."""
        self.current_mission_waypoint_index = None
        self.current_waypoint = None
        self.mission_started = False
        self.mission_waypoints = []
        self.misc_waypoints = []
        self.home_waypoints = []

        self.get_logger().info("Success: Cleared all waypoints")
        response.success = True
        response.message = "Success: Cleared all waypoints"

        # If persistence feature is enabled, update the database.
        if self.db_path != "":
            self.db_cursor.execute("DELETE FROM waypoints")
            self.db_connection.commit()
        return response

    def move_mission_waypoint(self, request, response) -> None:
        """
        Move the position of a mission waypoint in the list.

        The behavior of this operation is similar to bring forward or move backward. This replaces
        the old bring forward service.

        The current waypoint may change:
        It doesn't change the current mission waypoint index,
        but if this index ends up pointing to a different waypoint
        as a result of this function, the current waypoint will change.
        """
        if not (0 <= request.index < len(self.mission_waypoints) + 1):
            error_msg = "Can't move waypoint: The given index is out of bound."
            self.get_logger().warn(error_msg)
            response.success = False
            response.message = error_msg
            return response

        waypoint_index = self._find_waypoint(request.name, self.mission_waypoints)
        if waypoint_index is None:
            error_msg = "Can't move waypoint: No waypoint with such name exist."
            self.get_logger().warn(error_msg)
            response.success = False
            response.message = error_msg
            return response

        # Record the current waypoint, as this may change due to this operation.
        previous_waypoint = self.current_waypoint

        # Removes the waypoint and reinsert it in the requested position in the list.
        waypoint_to_be_moved = self.mission_waypoints[waypoint_index]
        if waypoint_index != request.index:
            self.mission_waypoints.pop(waypoint_index)
            self.mission_waypoints.insert(request.index, waypoint_to_be_moved)

        move_success_msg = (
            f"Success: Moving waypoint {waypoint_to_be_moved}"
            f"from index {waypoint_index} to {request.index}"
        )

        self.get_logger().info(move_success_msg)
        response.success = True
        response.message = move_success_msg

        # Inform the user if this operation changed the current waypoint.
        # Only possible if the rover is currently going to some mission waypoint.
        if (
            self.current_waypoint is not None
            and self.current_waypoint.type_ != WaypointType.HOME
        ):
            # Update the current waypoint.
            self.current_waypoint = self.mission_waypoints[
                self.current_mission_waypoint_index
            ]
            # Check if this operation changed the current waypoint.
            if self.current_waypoint != previous_waypoint:
                changed_waypoint_msg = (
                    "Current waypoint changed due to rearrangement of the list"
                )
                self.get_logger().info(changed_waypoint_msg)

        # If persistence feature is enabled, update the database.
        if self.db_path != "" and waypoint_index != request.index:
            if waypoint_index > request.index:
                # Bring forward scenario
                # Add +1 to indices to those with indices between [requested_index, wp.index - 1].
                self.db_cursor.execute(
                    (
                        "UPDATE waypoints "
                        "SET index_arr = index_arr + 1 "
                        "WHERE index_arr BETWEEN ? AND ? AND list_type = 'mission'"
                    ),
                    (request.index, waypoint_index - 1),
                )
            else:
                # Move backward scenario
                # Add -1 to indices to those with indices between [wp.index + 1, requested_index].
                self.db_cursor.execute(
                    (
                        "UPDATE waypoints "
                        "SET index_arr = index_arr - 1 "
                        "WHERE index_arr BETWEEN ? AND ? AND list_type = 'mission'"
                    ),
                    (waypoint_index + 1, request.index),
                )
            # Set wp.index = requested_index
            self.db_cursor.execute(
                ("UPDATE waypoints SET index_arr = ? WHERE name = ?"),
                (request.index, waypoint_to_be_moved.name),
            )
            self.db_connection.commit()

        return response

    def begin_mission(self, _, response) -> None:
        """
        Set the current waypoint to the currently tracked mission waypoint (if valid).

        The current mission waypoint is the mission waypoint tracked by
        current_mission_waypoint_index. It may become invalid if the mission waypoint list is edited
        , such that this index exceeds the size of list.  If it's not valid,
        it's set to the initial mission waypoint.

        This operation represent beginning or continuing the mission.
        """
        if len(self.mission_waypoints) == 0:
            error_msg = "Can't begin mission: No waypoint exists."
            self.get_logger().warn(error_msg)
            response.success = False
            response.message = error_msg
            return response

        if (
            self.current_mission_waypoint_index is not None
            and self.current_mission_waypoint_index < len(self.mission_waypoints)
        ):
            self.current_waypoint = self.mission_waypoints[
                self.current_mission_waypoint_index
            ]
            success_msg = (
                f"Continuing mission: Setting waypoint to {self.current_waypoint}"
            )
        else:
            self.current_mission_waypoint_index = 0
            self.current_waypoint = self.mission_waypoints[
                self.current_mission_waypoint_index
            ]
            success_msg = (
                f"Beginning mission: Setting waypoint to {self.current_waypoint}"
            )

        self.mission_started = True
        self.get_logger().info(success_msg)
        response.success = True
        response.message = success_msg
        return response

    def go_home(self, request, response) -> None:
        """
        Set the current waypoint to the requested home.

        The user may continue the mission by requesting the begin mission service.
        """
        home_waypoint_index = self._find_waypoint(request.name, self.home_waypoints)
        if home_waypoint_index is None:
            error_msg = "Can't go home: No home waypoint with such name."
            self.get_logger().warn(error_msg)
            response.success = False
            response.message = error_msg
            return response

        self.current_waypoint = self.home_waypoints[home_waypoint_index]
        self.mission_started = False

        success_msg = f"Success: Going home ({self.current_waypoint})"
        self.get_logger().info(success_msg)
        response.success = True
        response.message = success_msg
        return response

    def create_waypoint(self, request, response) -> None:
        """
        Create a new waypoint based on request.

        This is used to set the home waypoint, add a mission waypoint, or add a misc waypoint.
        """
        point = Point.from_waypoint(request.waypoint)

        # Checks all waypoint list to avoid identical name but different types.
        if point in self.home_waypoints + self.misc_waypoints + self.mission_waypoints:
            error_msg = "Can't create waypoint: Waypoint with such name already exists."
            self.get_logger().warn(error_msg)
            response.success = False
            response.message = error_msg
            return response

        waypoint_list_type, waypoint_list = self._get_associated_waypoint_list(point)
        waypoint_list.append(point)
        success_message = f"Success: Created the waypoint {point}."
        self.get_logger().info(success_message)
        response.success = True
        response.message = success_message

        # If persistence feature is enabled, update the database.
        if self.db_path != "":
            index = len(waypoint_list) - 1
            self.db_cursor.execute(
                "INSERT INTO waypoints VALUES(?,?,?,?,?,?,?)",
                (index, waypoint_list_type, *point),
            )
            self.db_connection.commit()
        return response

    def edit_waypoint(self, request, response) -> None:
        """
        Edit the waypoint with the requested name.

        Notes
        -----
        - Editing type only works if the type is originally CUSTOM, AUTONOMOUS, OR EXTREME_DELIVERY,
        and it's edited to either of these, because it only searches one specific list.
        - Current waypoint can be edited.

        """
        point = Point.from_waypoint(request.waypoint)

        # Check existence of waypoint by name.
        wp_list_name, wp_list = self._get_associated_waypoint_list(point)
        waypoint_index = self._find_waypoint(request.current_name, wp_list)
        if waypoint_index is None:
            error_msg = (
                "Can't edit waypoint: No waypoint with such name"
                " in the corresponding waypoint list"
            )
            self.get_logger().warn(error_msg)
            response.success = False
            response.message = error_msg
            return response

        # If the waypoint is being renamed, verify that the new name is unique.
        if point.name != request.current_name:
            new_name = point.name
            for wp in (
                self.home_waypoints + self.mission_waypoints + self.misc_waypoints
            ):
                if wp.name == new_name:
                    error_msg = "Can't edit waypoint: The new name is already taken."
                    self.get_logger().warn(error_msg)
                    response.success = False
                    response.message = error_msg
                    return response

        # Edit the requested waypoint.
        wp_list[waypoint_index] = point
        success_message = "Success in editing the waypoint"
        self.get_logger().info(success_message)
        response.success = True
        response.message = success_message

        # Update the current waypoint if it's the one being edited.
        if (
            self.current_waypoint is not None
            and self.current_waypoint.name == request.current_name
        ):
            self.current_waypoint = point
            changed_waypoint_msg = "The current waypoint was edited."
            self.get_logger().info(changed_waypoint_msg)

        # If persistence feature is enabled, update the database.
        if self.db_path != "":
            self.db_cursor.execute(
                (
                    "UPDATE waypoints "
                    "SET name = ?, latitude = ?, longitude = ?, type = ?, markers = ? "
                    "WHERE name = ?"
                ),
                (
                    point.name,
                    point.latitude,
                    point.longitude,
                    point.type_,
                    point.markers,
                    str(request.current_name),
                ),
            )
            self.db_connection.commit()
        return response

    def remove_waypoint(self, request, response) -> None:
        """
        Remove an existing waypoint (any type) based on request.

        Removing the current waypoint will stop the mission.
        """
        # Search through all waypoints list and find the waypoint.
        waypoint_list_types = ["home", "mission", "misc"]
        for i, waypoint_list in enumerate(
            [
                self.home_waypoints,
                self.mission_waypoints,
                self.misc_waypoints,
            ]
        ):
            waypoint_index = self._find_waypoint(request.name, waypoint_list)
            if waypoint_index is not None:
                waypoint_list_type = waypoint_list_types[i]
                break

        if waypoint_index is None:
            error_msg = "Can't remove waypoint: No waypoint with such name."
            self.get_logger().warn(error_msg)
            response.success = False
            response.message = error_msg
            return response

        wp_to_be_removed = waypoint_list[waypoint_index]

        if self.current_waypoint == wp_to_be_removed:
            msg = "The current waypoint was removed. Stopping mission."
            self.mission_started = False
            self.current_waypoint = None
            self.get_logger().info(msg)

        waypoint_list.pop(waypoint_index)
        success_message = f"Removed the waypoint {wp_to_be_removed}"
        self.get_logger().info(success_message)
        response.success = True
        response.message = success_message

        # If persistence feature is enabled, update the database.
        if self.db_path != "":
            # Removes the waypoint from database
            self.db_cursor.execute(
                "DELETE FROM waypoints WHERE name = ?", (request.name,)
            )
            # Decrease the index column for all waypoints in the same list after the removed point.
            self.db_cursor.execute(
                (
                    "UPDATE waypoints "
                    "SET index_arr = index_arr - 1 "
                    "WHERE index_arr > ? AND list_type = ?"
                ),
                (waypoint_index, waypoint_list_type),
            )
            self.db_connection.commit()
        return response

    def _find_waypoint(self, name: str, waypoint_list: List[Point]) -> Optional[int]:
        """
        Search a waypoint with the given name in the given list.

        If found, returns its index, Otherwise, return None.
        """
        waypoint_index = None
        for i in range(len(waypoint_list)):
            if waypoint_list[i].name == name:
                waypoint_index = i
                break
        return waypoint_index

    def _load_waypoints_from_db(self):
        """Load all waypoints from database to the server."""
        mission_waypoints_result = self.db_cursor.execute(
            "SELECT * FROM waypoints WHERE list_type = 'mission' ORDER BY index_arr"
        )
        # Each row is a tuple of (index_arr, name, latitude, longitude, type, markers)
        for row in mission_waypoints_result:
            self.mission_waypoints.append(Point.from_sqlite_row(row))

        home_waypoints_result = self.db_cursor.execute(
            "SELECT * FROM waypoints WHERE list_type = 'home' ORDER BY index_arr"
        )
        for row in home_waypoints_result:
            self.home_waypoints.append(Point.from_sqlite_row(row))

        misc_waypoints_result = self.db_cursor.execute(
            "SELECT * FROM waypoints WHERE list_type = 'misc' ORDER BY index_arr"
        )
        for row in misc_waypoints_result:
            self.misc_waypoints.append(Point.from_sqlite_row(row))

    def _create_and_send_transform(self, waypoint: Point):
        """Create and broadcast a TransformStamped message based on a given waypoint."""
        # UTM transform broadcaster for all waypoints.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "utm"
        t.child_frame_id = waypoint.name

        # Convert GPS to UTM coordinates.
        # easting is meters in East (x axis), from zone_number origin
        # northing is meters in North (y axis)
        # zone_number: UTM zone
        # zone_letter: north or south hemisphere.
        utm_point = utm.fromLatLong(waypoint.latitude, waypoint.longitude)

        t.transform.translation.x = utm_point.easting
        t.transform.translation.y = utm_point.northing
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.waypoints_tf_broadcaster.sendTransform(t)

    def _get_associated_waypoint_list(self, waypoint: Point) -> Tuple[str, List[Point]]:
        """Get the name of the list, and the list that stores this type of waypoint."""
        waypoint_list_type = ""
        waypoint_list = None

        if waypoint.type_ == WaypointType.HOME:
            waypoint_list = self.home_waypoints
            waypoint_list_type = "home"
        elif waypoint.type_ == WaypointType.MISCELLANEOUS:
            waypoint_list = self.misc_waypoints
            waypoint_list_type = "misc"
        else:
            waypoint_list = self.mission_waypoints
            waypoint_list_type = "mission"
        return (waypoint_list_type, waypoint_list)


def main(args=None):
    """Start the server node."""
    rclpy.init(args=args)
    waypoint_server = WaypointServer()
    rclpy.spin(waypoint_server)
    # Close connection to database
    if waypoint_server.db_path is not None:
        waypoint_server.db_connection.close()

    rclpy.shutdown()
