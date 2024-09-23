#!/usr/bin/env python3
from array import array
from dataclasses import dataclass
from enum import IntEnum
from waypoint_server_interfaces.msg import Waypoint


class WaypointType(IntEnum):
    CUSTOM = 0
    AUTONOMOUS = 1
    EXTREME_DELIVERY = 2
    HOME = 3
    MISCELLANEOUS = 4


@dataclass
class Point:
    """
    Class that represents a waypoint.

    Waypoint ROS messages are converted to and stored as this Point class internally. Note that
    by default, any "array" in ROS messages such as int32[] are python arrays.
    """

    name: str
    latitude: float
    longitude: float
    type_: int
    markers: array

    def into_waypoint(self) -> Waypoint:
        """Convert into ROS message type Waypoint."""
        wp = Waypoint()
        wp.name = self.name
        wp.latitude = self.latitude
        wp.longitude = self.longitude
        wp.type = self.type_
        wp.markers = self.markers
        return wp

    @classmethod
    def from_waypoint(cls, w: Waypoint):
        """Convert from ROS message type Waypoint to Point."""
        return cls(w.name, w.latitude, w.longitude, w.type, w.markers)

    def __eq__(self, other: object):
        return isinstance(other, Point) and other.name == self.name

    def __getitem__(self, index):
        """Allow quickly inserting a waypoint into SQL statements."""
        return (self.name, self.latitude, self.longitude, self.type_, self.markers)[
            index
        ]

    @classmethod
    def from_sqlite_row(cls, row):
        """Convert from database row to Point."""
        # Convert marker from string to array.
        markers_repr_string = row[-1]
        if markers_repr_string == "":
            markers = array("i")
        else:
            # markers are represented as string with separator ";" in database.
            sep = ";"
            markers = array("i", map(int, markers_repr_string.split(sep)))
        # Discard the index, list_type, and markers columns, then create a Point object.
        return Point(*row[2:-1], markers)
