#!/usr/bin/env python3
"""Check concurrent, isolated move_base actions and Gazebo contacts."""
import argparse
import json
import math
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState, ModelStates
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid


def wait_until(condition, seconds, message):
    deadline = time.monotonic() + seconds
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.025)
    raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--attach", action="store_true")
    parser.add_argument("--cancel-one", action="store_true")
    parser.add_argument("--physical", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("m2_check", anonymous=True)
    names = ["dog{}".format(i) for i in range(1, 5)]
    targets = ([(-4.3, y) for y in (-4.4, -2.2, 0.4, 4.7)] if args.attach
               else [(5.0, y) for y in (-3.6, -1.65, 1.65, 3.6)])
    dog_state, result, contact_feed, commands = {}, {}, set(), {}
    collisions = set()

    def positions(msg):
        dog_state.update({n: (p, v) for n, p, v in zip(msg.name, msg.pose, msg.twist) if n in names})

    def contacts(msg, source):
        contact_feed.add(source)
        for item in msg.states:
            pair = tuple(sorted((item.collision1_name, item.collision2_name)))
            if (args.physical and source.endswith("/contacts")) or not any(
                    part.split("::")[0] in ("ground", "ground_plane") for part in pair):
                collisions.add(pair)

    subscribers = [rospy.Subscriber("/gazebo/model_states", ModelStates, positions, queue_size=1)]
    suffixes = ("contacts", "feet/fl", "feet/fr", "feet/rl", "feet/rr") if args.physical else ("contacts",)
    feeds = {"/" + dog + "/" + suffix for dog in names for suffix in suffixes}
    subscribers.extend(rospy.Subscriber(feed, ContactsState, contacts, feed, queue_size=10)
                       for feed in sorted(feeds))
    subscribers.extend(rospy.Subscriber("/" + dog + "/cmd_vel", Twist,
                       lambda msg, dog=dog: commands.update({dog: msg}), queue_size=1) for dog in names)
    rospy.wait_for_message("/dog_map", OccupancyGrid, timeout=55)
    clients = {dog: actionlib.SimpleActionClient("/" + dog + "/move_base", MoveBaseAction) for dog in names}
    for client in clients.values():
        if not client.wait_for_server(rospy.Duration(10)):
            raise RuntimeError("move_base unavailable")
    wait_until(lambda: len(dog_state) == 4 and feeds <= contact_feed, 10, "missing pose/contact feeds")

    def send(dog, target):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "dog_map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = target
        goal.target_pose.pose.orientation.w = 1
        clients[dog].send_goal(goal, done_cb=lambda status, _: result.update({dog: status}))

    start = time.monotonic()
    try:
        for dog, target in zip(names, targets):
            send(dog, target)
        if args.cancel_one:
            wait_until(lambda: all(client.get_state() == GoalStatus.ACTIVE for client in clients.values()),
                       10, "not all goals active")
            clients["dog1"].cancel_goal()
            wait_until(lambda: result.get("dog1") == GoalStatus.PREEMPTED, 6, "dog1 not preempted")
            if any(clients[dog].get_state() != GoalStatus.ACTIVE for dog in names[1:]):
                raise RuntimeError("cancelling dog1 affected another action")
            result.pop("dog1")
            send("dog1", targets[0])
        wait_until(lambda: len(result) == 4, 100, "navigation timed out")
        def heading(pose):
            return 2 * math.atan2(pose.orientation.z, pose.orientation.w)

        still = dict(dog_state)
        drift = dict.fromkeys(names, 0.0)
        settle_until = time.monotonic() + 2.0
        while time.monotonic() < settle_until:
            for dog in names:
                initial, current = still[dog][0], dog_state[dog][0]
                dyaw = heading(current) - heading(initial)
                drift[dog] = max(drift[dog],
                                 math.hypot(current.position.x-initial.position.x,
                                            current.position.y-initial.position.y),
                                 abs(math.atan2(math.sin(dyaw), math.cos(dyaw))))
            time.sleep(0.025)
        errors, yaw_errors, speeds = {}, {}, {}
        for dog, (x, y) in zip(names, targets):
            pose, twist = dog_state[dog]
            errors[dog] = round(math.hypot(x - pose.position.x, y - pose.position.y), 3)
            yaw = heading(pose)
            yaw_errors[dog] = round(abs(math.atan2(math.sin(yaw), math.cos(yaw))), 3)
            speeds[dog] = round(max(math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z)), 3)
            cmd = commands[dog]
            if (result[dog] != GoalStatus.SUCCEEDED or errors[dog] > 0.25 or yaw_errors[dog] > 0.35
                    or max(abs(cmd.linear.x), abs(cmd.angular.z)) > 0.01 or drift[dog] > 0.025):
                raise RuntimeError("{} failed: status={} error={} yaw_error={} speed={} drift={}".format(
                    dog, result[dog], errors[dog], yaw_errors[dog], speeds[dog], drift[dog]))
        if collisions:
            raise RuntimeError("non-floor contacts: {}".format(collisions))
        print(json.dumps({"mode": "attach" if args.attach else "standalone", "actions": result,
                          "error_m": errors, "yaw_error_rad": yaw_errors, "settled_speed": speeds,
                          "settle_drift_max_m_or_rad": {dog: round(value, 4) for dog, value in drift.items()},
                          "contacts": 0, "cancel_isolated": args.cancel_one,
                          "wall_s": round(time.monotonic() - start, 2)}, sort_keys=True))
    finally:
        for client in clients.values():
            if client.get_state() in (GoalStatus.ACTIVE, GoalStatus.PENDING):
                client.cancel_goal()


if __name__ == "__main__":
    main()
