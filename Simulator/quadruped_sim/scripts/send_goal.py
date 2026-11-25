#!/usr/bin/env python3
import argparse
import math
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid


def main():
    parser = argparse.ArgumentParser(description="Send a dog_map goal to one dog.")
    parser.add_argument("--robot", default="dog1", choices=["dog1", "dog2", "dog3", "dog4"])
    parser.add_argument("--x", required=True, type=float)
    parser.add_argument("--y", required=True, type=float)
    parser.add_argument("--yaw", default=0.0, type=float)
    parser.add_argument("--timeout", default=90.0, type=float, help="wall seconds after goal submission")
    args = parser.parse_args(rospy.myargv()[1:])
    if not all(map(math.isfinite, (args.x, args.y, args.yaw, args.timeout))) or args.timeout <= 0:
        parser.error("goal values must be finite, timeout must be positive")

    rospy.init_node("send_dog_goal", anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_message("/dog_map", OccupancyGrid, timeout=15)
    except (rospy.ROSException, KeyboardInterrupt):
        rospy.logerr("dog_map not ready")
        return 2
    client = actionlib.SimpleActionClient("/" + args.robot + "/move_base", MoveBaseAction)
    ready = threading.Event()
    done = threading.Event()

    def connect():
        if client.wait_for_server():
            ready.set()

    threading.Thread(target=connect, daemon=True).start()
    if not ready.wait(15):
        rospy.logerr("move_base action not available after 15 wall seconds")
        return 2
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "dog_map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = args.x
    goal.target_pose.pose.position.y = args.y
    goal.target_pose.pose.orientation.z = math.sin(args.yaw / 2)
    goal.target_pose.pose.orientation.w = math.cos(args.yaw / 2)
    result = []

    def finished(status, _):
        result.append(status)
        done.set()

    client.send_goal(goal, done_cb=finished)
    try:
        deadline = time.monotonic() + args.timeout
        while not done.is_set() and not rospy.is_shutdown() and time.monotonic() < deadline:
            done.wait(0.1)
        if done.is_set():
            if result[0] == GoalStatus.SUCCEEDED:
                rospy.loginfo("%s SUCCEEDED", args.robot)
                return 0
            rospy.logerr("%s action status=%s: %s", args.robot, result[0], client.get_goal_status_text())
            return 1
    except KeyboardInterrupt:
        pass
    client.cancel_goal()
    done.wait(2)
    rospy.logwarn("Goal cancelled or timed out; action status=%s", result[0] if result else "unconfirmed")
    return 124


if __name__ == "__main__":
    raise SystemExit(main())
