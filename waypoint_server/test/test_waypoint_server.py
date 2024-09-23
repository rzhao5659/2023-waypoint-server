import pytest
from array import array
import rclpy
from client import Point, WaypointType, Client
from waypoint_server.server import WaypointServer
from rclpy import get_global_executor

# This testing file imports the server and a client, and spins both.


@pytest.fixture()
def test_waypoints():
    # Creating some waypoints to play with.
    wp_dict = {}
    wp_dict["Waypoint1"] = Point("Waypoint1", 1.0, 1.0, WaypointType.AUTONOMOUS, array("i", []))
    wp_dict["Waypoint2"] = Point("Waypoint2", 2.0, 3.0, WaypointType.CUSTOM, array("i", [1, 2, 3]))
    wp_dict["Waypoint3"] = Point(
        "Waypoint3", 1932.31, 1231.2, WaypointType.EXTREME_DELIVERY, array("i", [])
    )
    wp_dict["Waypoint4"] = Point("Waypoint4", 1.0, 1.0, WaypointType.AUTONOMOUS, array("i", []))
    wp_dict["Waypoint5"] = Point(
        "Waypoint5", 1932.31, 1231.2, WaypointType.EXTREME_DELIVERY, array("i", [])
    )
    wp_dict["Home1"] = Point("Home1", 0.313, 1312.1, WaypointType.HOME, array("i", []))
    wp_dict["Home2"] = Point("Home2", 0.411, 1391.31, WaypointType.HOME, array("i", []))
    wp_dict["Misc1"] = Point("Misc1", 0.313, 1312.1, WaypointType.MISCELLANEOUS, array("i", []))
    wp_dict["Misc2"] = Point("Misc2", 0.411, 1391.31, WaypointType.MISCELLANEOUS, array("i", []))
    return wp_dict


@pytest.fixture(autouse=True)
def client():
    # Setup
    rclpy.init()

    # Initialize server and add to the global executor.
    server = WaypointServer(disable_tf=True)
    executor = get_global_executor()
    executor.add_node(server)

    # Initialize client and send requests.
    # The spinning is done inside client.spin_until_msg() method,
    # which will spin both server and client until messages are received.
    client = Client()
    executor.add_node(client)
    client.send_clear_all_req()
    client.send_create_wp_req("Waypoint1", 1.0, 1.0, WaypointType.AUTONOMOUS, array("i", []))
    client.send_create_wp_req("Waypoint2", 2.0, 3.0, WaypointType.CUSTOM, array("i", [1, 2, 3]))
    client.send_create_wp_req(
        "Waypoint3", 1932.31, 1231.2, WaypointType.EXTREME_DELIVERY, array("i", [])
    )
    client.send_create_wp_req("Waypoint4", 1.0, 1.0, WaypointType.AUTONOMOUS, array("i", []))
    client.send_create_wp_req(
        "Waypoint5", 1932.31, 1231.2, WaypointType.EXTREME_DELIVERY, array("i", [])
    )
    client.send_create_wp_req("Home1", 0.313, 1312.1, WaypointType.HOME, array("i", []))
    client.send_create_wp_req("Home2", 0.411, 1391.31, WaypointType.HOME, array("i", []))
    client.send_create_wp_req("Misc1", 0.313, 1312.1, WaypointType.MISCELLANEOUS, array("i", []))
    client.send_create_wp_req("Misc2", 0.411, 1391.31, WaypointType.MISCELLANEOUS, array("i", []))
    yield client
    # Teardown
    client.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def test_create(client, test_waypoints):
    client.spin_until_msg()
    # Test waypoints are created in the expected order, and with correct attribute values.
    assert client.home_waypoints_msg == [
        test_waypoints["Home1"],
        test_waypoints["Home2"],
    ]
    assert client.misc_waypoints_msg == [
        test_waypoints["Misc1"],
        test_waypoints["Misc2"],
    ]
    assert client.mission_waypoints_msg == [
        test_waypoints["Waypoint1"],
        test_waypoints["Waypoint2"],
        test_waypoints["Waypoint3"],
        test_waypoints["Waypoint4"],
        test_waypoints["Waypoint5"],
    ]

    # Test that creating waypoints with non unique names will fail
    response1 = client.send_create_wp_req(
        "Waypoint1", 31312.2, 14.1, WaypointType.AUTONOMOUS, array("i", [])
    )
    response2 = client.send_create_wp_req("Home1", 31312.2, 14.1, WaypointType.HOME, array("i", []))
    response2 = client.send_create_wp_req(
        "Misc2", 31312.2, 14.1, WaypointType.MISCELLANEOUS, array("i", [])
    )
    assert response1.success is False and response2.success is False


def test_remove(client, test_waypoints):
    client.send_remove_req("Waypoint1")
    client.send_remove_req("Waypoint3")
    client.send_remove_req("Home1")
    client.send_remove_req("Misc2")
    client.spin_until_msg()
    assert client.mission_waypoints_msg == [
        test_waypoints["Waypoint2"],
        test_waypoints["Waypoint4"],
        test_waypoints["Waypoint5"],
    ]
    assert client.home_waypoints_msg == [test_waypoints["Home2"]]
    assert client.misc_waypoints_msg == [test_waypoints["Misc1"]]

    response = client.send_remove_req("blab")
    assert response.success is False
    assert response.message == "Can't remove waypoint: No waypoint with such name."


def test_remove_current_waypoint(client, test_waypoints):
    # Tests that if remove service removed the current waypoint,
    # then the mission stops.
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    client.send_remove_req("Waypoint1")
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None


def test_edit(client, test_waypoints):
    client.send_edit_wp_req(
        "Waypoint1",
        "Waypoint1",
        5.0,
        5.0,
        WaypointType.AUTONOMOUS,
        array("i", [5, 6, 7, 8]),
    )
    client.spin_until_msg()

    # Tests that only the corresponding waypoint is edited, and other waypoints stays the same.
    assert client.mission_waypoints_msg[0] == Point(
        "Waypoint1", 5.0, 5.0, WaypointType.AUTONOMOUS, array("i", [5, 6, 7, 8])
    )
    assert client.mission_waypoints_msg[1:5] == [
        test_waypoints["Waypoint2"],
        test_waypoints["Waypoint3"],
        test_waypoints["Waypoint4"],
        test_waypoints["Waypoint5"],
    ]

    # Tests that editing a mission waypoint type to another mission waypoint type works.
    client.send_edit_wp_req(
        "Waypoint1",
        "Waypoint1",
        5.0,
        6.0,
        WaypointType.EXTREME_DELIVERY,
        array("i", [5, 6, 7, 8]),
    )
    client.spin_until_msg()
    assert client.mission_waypoints_msg[0] == Point(
        "Waypoint1", 5.0, 6.0, WaypointType.EXTREME_DELIVERY, array("i", [5, 6, 7, 8])
    )

    # Tests that editing a mission waypoint type to home type fails,
    # simply because it will search on home list and will not find waypoint with such name.
    response = client.send_edit_wp_req(
        "Waypoint1", "Waypoint1", 5.0, 5.0, WaypointType.HOME, array("i", [5, 6, 7, 8])
    )
    assert response.success is False
    assert response.message == (
        "Can't edit waypoint: No waypoint with such name" " in the corresponding waypoint list"
    )

    # Tests that editing a waypoint with no such name fails.
    response = client.send_edit_wp_req(
        "blabla",
        "blabla",
        5.0,
        5.0,
        WaypointType.EXTREME_DELIVERY,
        array("i", [5, 6, 7, 8]),
    )
    assert response.success is False
    assert response.message == (
        "Can't edit waypoint: No waypoint with such name" " in the corresponding waypoint list"
    )

    # Tests multiple edits across all waypoint lists.
    client.send_edit_wp_req(
        "Waypoint1", "Waypoint1", 0.0, 0.0, WaypointType.AUTONOMOUS, array("i", [0])
    )
    client.send_edit_wp_req(
        "Waypoint3",
        "Waypoint3",
        5.0,
        5.0,
        WaypointType.EXTREME_DELIVERY,
        array("i", [5, 6, 7, 8]),
    )
    client.send_edit_wp_req("Home1", "Home1", 5.0, 5.0, WaypointType.HOME, array("i", [5, 6, 7, 8]))
    client.send_edit_wp_req(
        "Misc2", "Misc2", 5.1, 5.1, WaypointType.MISCELLANEOUS, array("i", [5, 6, 7, 8])
    )
    client.spin_until_msg()
    assert client.home_waypoints_msg == [
        Point("Home1", 5.0, 5.0, WaypointType.HOME, array("i", [5, 6, 7, 8])),
        test_waypoints["Home2"],
    ]
    assert client.misc_waypoints_msg == [
        test_waypoints["Misc1"],
        Point("Misc2", 5.1, 5.1, WaypointType.MISCELLANEOUS, array("i", [5, 6, 7, 8])),
    ]
    assert client.mission_waypoints_msg == [
        Point("Waypoint1", 0.0, 0.0, WaypointType.AUTONOMOUS, array("i", [0])),
        test_waypoints["Waypoint2"],
        Point(
            "Waypoint3",
            5.0,
            5.0,
            WaypointType.EXTREME_DELIVERY,
            array("i", [5, 6, 7, 8]),
        ),
        test_waypoints["Waypoint4"],
        test_waypoints["Waypoint5"],
    ]


def test_edit_renaming(client, test_waypoints):
    # Test that renaming Waypoint1 to Waypoint2 will fail, because the new name "Waypoint2"
    # already exists.
    response = client.send_edit_wp_req(
        "Waypoint1", "Waypoint2", 1.0, 1.0, WaypointType.AUTONOMOUS, array("i", [])
    )
    assert response.success is False
    assert response.message == "Can't edit waypoint: The new name is already taken."

    # Test that renaming Waypoint1 to Waypoint6 works, and other waypoints is not changed.
    client.send_edit_wp_req(
        "Waypoint1",
        "Waypoint6",
        1.5,
        1.5,
        WaypointType.AUTONOMOUS,
        array("i", [1, 2, 3]),
    )
    client.spin_until_msg()
    assert client.mission_waypoints_msg[0] == Point(
        "Waypoint6", 1.5, 1.5, WaypointType.AUTONOMOUS, array("i", [1, 2, 3])
    )
    assert client.mission_waypoints_msg[1:5] == [
        test_waypoints["Waypoint2"],
        test_waypoints["Waypoint3"],
        test_waypoints["Waypoint4"],
        test_waypoints["Waypoint5"],
    ]


def test_edit_current_waypoint(client, test_waypoints):
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Test editing the current waypoint is correctly reflected.
    client.send_edit_wp_req(
        "Waypoint1",
        "Waypoint1",
        1321312.0,
        31312.0,
        WaypointType.AUTONOMOUS,
        array("i", [0, 1, 2]),
    )
    client.spin_until_msg()
    assert client.curr_waypoint_msg == Point(
        "Waypoint1", 1321312.0, 31312.0, WaypointType.AUTONOMOUS, array("i", [0, 1, 2])
    )


def test_clear(client):
    client.send_clear_home_req()
    client.spin_until_msg()
    assert len(client.home_waypoints_msg) == 0
    assert len(client.mission_waypoints_msg) == 5
    assert len(client.misc_waypoints_msg) == 2

    client.send_clear_misc_req()
    client.spin_until_msg()
    assert len(client.home_waypoints_msg) == 0
    assert len(client.mission_waypoints_msg) == 5
    assert len(client.misc_waypoints_msg) == 0

    client.send_clear_mission_req()
    client.spin_until_msg()
    assert len(client.home_waypoints_msg) == 0
    assert len(client.mission_waypoints_msg) == 0
    assert len(client.misc_waypoints_msg) == 0


def test_clear_current_mission_waypoint_1(client, test_waypoints):
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Tests clearing home and misc doesn't clear current waypoint, because it's a mission waypoint.
    client.send_clear_home_req()
    client.send_clear_misc_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Tests clearing mission will clear current waypoint.
    client.send_clear_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None


def test_clear_current_mission_waypoint_2(client, test_waypoints):
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Tests clearing all will clear current waypoint
    client.send_clear_all_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None


def test_clear_current_home_waypoint(client, test_waypoints):
    client.send_go_home_req("Home2")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Home2"]

    # Tests clearing mission and misc waypoints will not affect current waypoint
    # because its a home waypoint.
    client.send_clear_mission_req()
    client.send_clear_misc_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Home2"]

    # Tests clearing home will clear current waypoint.
    client.send_clear_home_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None


def test_move(client, test_waypoints):
    # Test case: initially we have waypoints [1,2,3,4,5]
    # I will move waypoint 3 to 1st position (result = [3,1,2,4,5])
    # then waypoint 2 to the 4th position (result = [3,1,4,2,5])
    # then move waypoint 3 to the final position (result = [1,4,2,5,3])
    client.send_move_req(0, "Waypoint3")
    client.send_move_req(3, "Waypoint2")
    client.send_move_req(4, "Waypoint3")
    client.spin_until_msg()
    assert client.mission_waypoints_msg == [
        test_waypoints["Waypoint1"],
        test_waypoints["Waypoint4"],
        test_waypoints["Waypoint2"],
        test_waypoints["Waypoint5"],
        test_waypoints["Waypoint3"],
    ]
    # Test that out of bound index (except when index is length of list) fails
    response1 = client.send_move_req(-1, "Waypoint3")
    response2 = client.send_move_req(6, "Waypoint3")
    assert response1.success is False and response2.success is False
    assert response1.message == "Can't move waypoint: The given index is out of bound."

    # Test that trying to move a waypoint with no such name fails.
    # Home1 should fail because it doesnt exist in mission waypoint list
    response = client.send_move_req(1, "Home1")
    assert response.success is False
    assert response.message == "Can't move waypoint: No waypoint with such name exist."

    # Test that it fails if the mission waypoint list is empty.
    client.send_clear_mission_req()
    response = client.send_move_req(0, "Waypoint1")
    assert response.success is False
    assert response.message == "Can't move waypoint: No waypoint with such name exist."


def test_advance_waypoint(client, test_waypoints):
    # Initially, no current waypoint.
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None

    # Tests that advance fails because mission hasn't began yet.
    response = client.send_advance_req()
    assert response.success is False
    assert response.message == "Can't advance waypoint: Needs to start mission."
    # This includes the case when the rover is going home.
    client.send_go_home_req("Home1")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Home1"]
    response = client.send_advance_req()
    assert response.success is False
    assert response.message == "Can't advance waypoint: Needs to start mission."

    # Beginning mission makes current waypoint to the first mission waypoint.
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Tests advance.
    client.send_advance_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint2"]
    client.send_advance_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint3"]
    client.send_advance_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint4"]
    client.send_advance_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint5"]

    # Tests that it fails if current waypoint is last mission waypoint already.
    response = client.send_advance_req()
    assert response.success is False
    assert response.message == "Can't advance waypoint: This is the last waypoint"


def test_set_waypoint(client, test_waypoints):
    # Initially, no current waypoint.
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None

    # Tests that set fails if mission hasn't began yet.
    response = client.send_set_req("Waypoint1")
    assert response.success is False
    assert response.message == "Can't set waypoint: Needs to start mission."
    # This includes the case when the rover is going home.
    client.send_go_home_req("Home1")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Home1"]
    response = client.send_set_req("Waypoint2")
    assert response.success is False
    assert response.message == "Can't set waypoint: Needs to start mission."

    # Beginning mission makes current waypoint to the first mission waypoint.
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Tests set.
    client.send_set_req("Waypoint5")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint5"]
    client.send_set_req("Waypoint3")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint3"]
    client.send_set_req("Waypoint2")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint2"]
    client.send_set_req("Waypoint1")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Tests that it fails if it doesnt find the waypoint.
    response = client.send_set_req("bkofewma")
    assert response.success is False
    assert response.message == "Can't set waypoint: No waypoint with such name."


def test_revert_waypoint(client, test_waypoints):
    # Initially, no current waypoint.
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None

    # Tests that revert fails because mission hasn't began yet.
    response = client.send_revert_req()
    assert response.success is False
    assert response.message == "Can't revert waypoint: Needs to start mission."
    # This includes the case when the rover is going home.
    client.send_go_home_req("Home1")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Home1"]
    response = client.send_revert_req()
    assert response.success is False
    assert response.message == "Can't revert waypoint: Needs to start mission."

    # Beginning mission makes current waypoint to the first mission waypoint.
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Tests revert fails if current waypoint is already the first waypoint.
    response = client.send_revert_req()
    assert response.success is False
    assert response.message == "Can't revert waypoint: No waypoint before this"
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]

    # Test revert in general.
    client.send_set_req("Waypoint5")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint5"]
    client.send_revert_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint4"]
    client.send_revert_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint3"]
    client.send_revert_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint2"]
    client.send_revert_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint1"]


def test_mission_and_home_interaction(client, test_waypoints):
    # Initially, no curr waypoint.
    client.spin_until_msg()
    assert client.curr_waypoint_msg is None

    # Start mission and set it to Waypoint3.
    client.send_begin_mission_req()
    client.send_set_req("Waypoint3")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint3"]

    # Pause mission by going home, which sets current waypoint to be the requested home waypoint.
    client.send_go_home_req("Home1")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Home1"]
    client.send_go_home_req("Home2")
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Home2"]

    # Continue mission by invoking begin mission again.
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint3"]


def test_begin_mission_fail(client, test_waypoints):
    # Tests that it fails if mission waypoint list is empty
    client.send_clear_all_req()
    client.spin_until_msg()
    response = client.send_begin_mission_req()
    assert response.success is False
    assert response.message == "Can't begin mission: No waypoint exists."


def test_go_home_fail(client, test_waypoints):
    # Tests that it fails if no home with such name.
    client.send_clear_home_req()
    client.spin_until_msg()
    response = client.send_go_home_req("Home1")
    assert response.success is False
    assert response.message == "Can't go home: No home waypoint with such name."


def test_begin_mission_after_edit(client, test_waypoints):
    client.send_begin_mission_req()
    client.send_set_req("Waypoint4")
    client.send_go_home_req("Home2")
    client.send_remove_req("Waypoint5")
    client.send_remove_req("Waypoint1")
    client.send_remove_req("Waypoint2")

    # The stored mission waypoint index is out of bound,
    # hence begin mission service should start from the first mission waypoint.
    client.send_begin_mission_req()
    client.spin_until_msg()
    assert client.curr_waypoint_msg == test_waypoints["Waypoint3"]
