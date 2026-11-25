#!/usr/bin/env python3
"""Exercise one fresh standalone world. Emit one machine-readable evidence record."""
import argparse
import json
import math
import subprocess
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState, ModelStates
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


def wait_until(predicate, timeout, reason):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise RuntimeError(reason)


def speed(twist):
    return max(math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z))


def rectangle_clearance(first, second):
    """Minimum planar separation of the two oriented collision footprints."""
    def corners(pose, half_x, half_y):
        yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
        c, s = math.cos(yaw), math.sin(yaw)
        return [(pose.position.x + c*x - s*y, pose.position.y + s*x + c*y)
                for x, y in ((half_x, half_y), (-half_x, half_y), (-half_x, -half_y), (half_x, -half_y))]

    def edges(poly):
        return zip(poly, poly[1:] + poly[:1])

    def distance(point, start, end):
        dx, dy = end[0] - start[0], end[1] - start[1]
        px, py = point[0] - start[0], point[1] - start[1]
        t = max(0.0, min(1.0, (px*dx + py*dy) / (dx*dx + dy*dy)))
        return math.hypot(px - t*dx, py - t*dy)

    a, b = corners(*first), corners(*second)
    for start, end in list(edges(a)) + list(edges(b)):
        nx, ny = start[1] - end[1], end[0] - start[0]
        ap = [x*nx + y*ny for x, y in a]
        bp = [x*nx + y*ny for x, y in b]
        if max(ap) < min(bp) or max(bp) < min(ap):
            return min(distance(point, start, end) for points, poly in ((a, b), (b, a))
                       for point in points for start, end in edges(poly))
    return 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("case", choices=["empty", "static", "dynamic", "wall", "cancel", "pause",
                                         "watchdog", "scan_loss", "odom_loss"])
    parser.add_argument("--physical", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("m1_check", anonymous=True)
    rospy.wait_for_message("/dog_map", OccupancyGrid, timeout=45)
    latest = {}
    contacts = set()
    samples = []
    footprint = (.33, .21) if args.physical else (.30, .18)
    commands = []
    crossing_y = []
    state = []
    initial_plan = []
    dynamic_clearances = []
    clearing = {"marked": False, "cleared": False}
    relay_enabled = True

    def on_models(msg):
        if "crossing" in msg.name:
            crossing_y.append(msg.pose[msg.name.index("crossing")].position.y)
        if "dog1" not in msg.name:
            return
        idx = msg.name.index("dog1")
        latest["dog"] = (msg.pose[idx], msg.twist[idx])
        stamp = rospy.get_time()
        if not samples or stamp - samples[-1][0] >= 0.049:
            p = msg.pose[idx].position
            samples.append((stamp, p.x, p.y, speed(msg.twist[idx])))
            if "crossing" in msg.name:
                dynamic_clearances.append(rectangle_clearance(
                    (msg.pose[idx], *footprint),
                    (msg.pose[msg.name.index("crossing")], 0.325, 0.325)))

    def on_contacts(msg, foot="body"):
        latest[foot] = True
        for item in msg.states:
            pair = (item.collision1_name, item.collision2_name)
            if (args.physical and foot == "body") or not any("ground::" in name for name in pair):
                contacts.add(pair)

    def on_costmap(msg):
        if not crossing_y:
            return
        y = crossing_y[-1]
        origin = msg.info.origin.position
        res = msg.info.resolution
        i0, i1 = (math.floor((x - origin.x) / res) for x in (-2.90, -2.10))
        j0, j1 = (math.floor((y - origin.y) / res) for y in (-0.22, 0.22))
        if not (0 <= i0 <= i1 < msg.info.width and 0 <= j0 <= j1 < msg.info.height):
            return
        values = [msg.data[j * msg.info.width + i] for j in range(j0, j1 + 1) for i in range(i0, i1 + 1)]
        if abs(y) < 0.55 and max(values) == 100:
            clearing["marked"] = True
        elif y > 0.9 and clearing["marked"] and all(0 <= value < 99 for value in values):
            clearing["cleared"] = True

    def on_plan(msg):
        if not initial_plan and len(msg.poses) > 1:
            points = [p.pose.position for p in msg.poses]
            initial_plan.append(sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:])))

    subs = [
        rospy.Subscriber("/gazebo/model_states", ModelStates, on_models, queue_size=1),
        rospy.Subscriber("/dog1/contacts", ContactsState, on_contacts, queue_size=10),
        rospy.Subscriber("/dog1/cmd_vel", Twist, lambda msg: commands.append((time.monotonic(), speed(msg))),
                         queue_size=10),
    ]
    feet = ("fl", "fr", "rl", "rr") if args.physical else ()
    subs.extend(rospy.Subscriber("/dog1/feet/" + foot, ContactsState, on_contacts, foot, queue_size=10)
                for foot in feet)
    if args.case == "static":
        subs.append(rospy.Subscriber("/dog1/move_base/NavfnROS/plan", Path, on_plan, queue_size=1))
    if args.case == "dynamic":
        subs.append(rospy.Subscriber("/dog1/move_base/local_costmap/costmap", OccupancyGrid,
                                     on_costmap, queue_size=1))
    if args.case in ("scan_loss", "odom_loss"):
        source, msg_type = ("scan", LaserScan) if args.case == "scan_loss" else ("odom", Odometry)
        relay = rospy.Publisher("/dog1/test_" + source, msg_type, queue_size=1)
        subs.append(rospy.Subscriber("/dog1/" + source, msg_type,
                                     lambda msg: relay.publish(msg) if relay_enabled else None, queue_size=1))
    client = actionlib.SimpleActionClient("/dog1/move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(10)):
        raise RuntimeError("no move_base action")
    wait_until(lambda: all(key in latest for key in ("dog", "body", *feet)), 10,
               "no Gazebo pose/contact data")
    if args.case == "dynamic":
        wait_until(lambda: bool(crossing_y), 5, "launch with dynamic_obstacle:=true")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "dog_map"
    goal.target_pose.header.stamp = rospy.Time.now()
    target_x = {"empty": -3.5, "wall": 0.0}.get(args.case, 5.0)
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.orientation.w = 1.0
    start = rospy.get_time()
    wall_start = time.monotonic()
    client.send_goal(goal, done_cb=lambda status, _: state.append(status))
    evidence = {"case": args.case}
    if args.case in ("cancel", "pause", "watchdog", "scan_loss", "odom_loss"):
        wait_until(lambda: abs(latest["dog"][1].linear.x) > 0.12, 12, "robot did not start")
        stop_start = time.monotonic()
        pose = latest["dog"][0].position
        if args.case == "cancel":
            client.cancel_goal()
        elif args.case == "pause":
            rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
        elif args.case == "watchdog":
            subprocess.run(["rosnode", "kill", "/dog1/velocity_guard"], check=True, timeout=5,
                           stdout=subprocess.DEVNULL)
        else:
            relay_enabled = False
        if args.case != "watchdog":
            wait_until(lambda: any(t >= stop_start and v < 0.01 for t, v in commands[-20:]), 0.7,
                       "final cmd_vel did not zero within 0.7 seconds")
            evidence["zero_command_wall_s"] = round(next(t - stop_start for t, v in commands if
                                                         t >= stop_start and v < 0.01), 3)
        if args.case != "pause":
            wait_until(lambda: speed(latest["dog"][1]) < 0.03, 0.7, "Gazebo model did not stop")
            stopped = latest["dog"][0].position
            evidence["stop_distance_m"] = round(math.hypot(stopped.x - pose.x, stopped.y - pose.y), 3)
        else:
            client.cancel_goal()
            rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
        if args.case == "watchdog":
            client.cancel_goal()
            assert speed(latest["dog"][1]) < 0.03
        else:
            if args.case in ("scan_loss", "odom_loss"):
                client.cancel_goal()
            wait_until(lambda: bool(state), 5, "cancel not acknowledged")
            assert state[0] == GoalStatus.PREEMPTED, state
    else:
        try:
            wait_until(lambda: bool(state), 120 if args.case == "dynamic" else 90, "action timed out")
        except RuntimeError:
            pose = latest["dog"][0].position
            rospy.logerr("Last Gazebo XY=(%.3f, %.3f), action=%d, contacts=%s",
                         pose.x, pose.y, client.get_state(), sorted(contacts))
            client.cancel_goal()
            raise
        expected = GoalStatus.ABORTED if args.case == "wall" else GoalStatus.SUCCEEDED
        assert state[0] == expected, (state, client.get_goal_status_text())
        if expected == GoalStatus.SUCCEEDED:
            pose = latest["dog"][0]
            error = math.hypot(pose.position.x - target_x, pose.position.y)
            yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
            yaw_error = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
            assert error <= 0.25 and yaw_error <= 0.35, (error, yaw_error)
            evidence.update(error_m=round(error, 3), yaw_error_rad=round(yaw_error, 3),
                            path_m=round(sum(math.hypot(b[1] - a[1], b[2] - a[2])
                                             for a, b in zip(samples, samples[1:])), 3))
        if args.case in ("static", "dynamic"):
            center_pass = [abs(y) for _, x, y, _ in samples if abs(x) < 0.3]
            assert center_pass and min(center_pass) > 1.0, "did not go around the box"
            evidence["box_pass_abs_y_m"] = round(min(center_pass), 3)
        if args.case == "static" and initial_plan and initial_plan[0] > 0:
            evidence["path_ratio"] = round(evidence["path_m"] / initial_plan[0], 3)
        if args.case == "dynamic":
            assert min(crossing_y) < -1.8 and max(crossing_y) > 1.8, "obstacle did not cross"
            assert clearing["marked"] and clearing["cleared"], clearing
            evidence["dynamic_cells_cleared"] = True
            evidence["crossing_clearance_m"] = round(min(dynamic_clearances), 3)
    finished = rospy.get_time()
    wait_until(lambda: rospy.get_time() >= finished + 2.0, 6, "cannot observe two seconds of settling")
    stop_speeds = [v for t, _, _, v in samples if t > finished + 0.3]
    assert stop_speeds, "missing Gazebo samples in stopping window"
    peak = max(stop_speeds)
    assert peak < 0.04, "robot kept moving: peak={:.4f}, evidence={}, contacts={}".format(
        peak, evidence, sorted(contacts))
    evidence["stop_speed_peak"] = round(peak, 4)
    assert not contacts, contacts
    evidence.update(action=state[0] if state else None, sim_s=round(finished - start, 3),
                    wall_s=round(time.monotonic() - wall_start, 3), contacts=len(contacts))
    print(json.dumps(evidence, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
