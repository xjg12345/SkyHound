#!/usr/bin/env python3
"""Verify unassisted standing, contact-driven gait and fail-stop in Gazebo."""
import json
import math
import time
from collections import defaultdict

import rospy
from gazebo_msgs.msg import ContactsState, ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


def main():
    rospy.init_node("m3_check", anonymous=True)
    rospy.wait_for_message("/dog_map", OccupancyGrid, timeout=45)
    feet = ("fl", "fr", "rl", "rr")
    contact = defaultdict(set)
    impacts = set()
    angles = defaultdict(list)
    poses = []
    status = {}
    stage = ["stand"]

    def on_pose(msg):
        if "dog1" not in msg.name:
            return
        p = msg.pose[msg.name.index("dog1")]
        q = p.orientation
        roll, pitch, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        poses.append((stage[0], p.position.x, p.position.y, p.position.z, roll, pitch, yaw))

    def on_contact(msg, foot=None):
        ground = False
        for item in msg.states:
            pair = (item.collision1_name, item.collision2_name)
            if foot and any("ground::" in name for name in pair):
                ground = True
            else:
                impacts.add(pair)
        if foot:
            contact[stage[0], foot].add(ground)

    def on_joint(msg):
        if len(msg.name) == len(msg.position) == 12:
            angles[stage[0]].append(msg.position)

    subs = [
        rospy.Subscriber("/gazebo/model_states", ModelStates, on_pose, queue_size=1),
        rospy.Subscriber("/dog1/joint_states", JointState, on_joint, queue_size=10),
        rospy.Subscriber("/dog1/physical_ready", Bool, lambda m: status.update(ready=m.data)),
        rospy.Subscriber("/dog1/physical_fault", Bool, lambda m: status.update(fault=m.data)),
        rospy.Subscriber("/dog1/contacts", ContactsState, on_contact, callback_args=None),
    ]
    # The normal guard remains the only publisher to the controller during testing.
    cmd = rospy.Publisher("/dog1/cmd_vel_nav", Twist, queue_size=1)
    subs.extend(rospy.Subscriber("/dog1/feet/" + foot, ContactsState, on_contact,
                                 callback_args=foot, queue_size=10) for foot in feet)

    def run(label, duration, v=0.0, w=0.0, publish=True):
        stage[0] = label
        message = Twist()
        message.linear.x, message.angular.z = v, w
        start = rospy.get_time()
        deadline = time.monotonic() + 35
        while rospy.get_time() - start < duration:
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                raise RuntimeError("simulation clock stopped in " + label)
            if publish:
                cmd.publish(message)
            time.sleep(.04)
        samples = [p[1:] for p in poses if p[0] == label]
        if not samples:
            raise RuntimeError("no model states in " + label)
        return samples

    stand = run("stand", 3)
    walk = run("walk", 6, v=.25)
    stop = run("stop", 4, publish=False)
    turn = run("turn", 5, w=.55)
    rest = run("rest", 3)
    motion = stand + walk + stop + turn + rest
    change = sum(math.atan2(math.sin(b[5] - a[5]), math.cos(b[5] - a[5]))
                 for a, b in zip(turn, turn[1:]))
    ranges = [max(col) - min(col) for col in zip(*angles["walk"])]
    record = {
        "stand_drift_m": math.dist(stand[0][:2], stand[-1][:2]),
        "walk_dx_m": walk[-1][0] - walk[0][0],
        "walk_dy_m": walk[-1][1] - walk[0][1],
        "turn_rad": change,
        "stop_tail_drift_m": math.dist(stop[len(stop)//2][:2], stop[-1][:2]),
        "height_range_m": [min(p[2] for p in motion), max(p[2] for p in motion)],
        "max_tilt_rad": max(max(abs(p[3]), abs(p[4])) for p in motion),
        "joint_ranges_rad": ranges,
        "four_feet_alternate": all(contact["walk", f] == {False, True} for f in feet),
        "four_feet_support": all(True in contact["stand", f] for f in feet),
        "unexpected_contacts": sorted(impacts),
        "ready_after_gait": status.get("ready") is True and status.get("fault") is False,
    }
    print(json.dumps(record, sort_keys=True), flush=True)

    # An external test disturbance, never part of the movement controller.
    state = ModelState()
    state.model_name = "dog1"
    state.reference_frame = "world"
    state.pose.position.x, state.pose.position.y, state.pose.position.z = *rest[-1][:2], .20
    state.pose.orientation.x, state.pose.orientation.w = math.sin(.6), math.cos(.6)
    if not rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)(state).success:
        raise RuntimeError("could not apply fall test disturbance")
    run("fall", 2, v=.25)
    output = rospy.wait_for_message("/dog1/cmd_vel", Twist, timeout=2)
    locked = status.get("fault") is True and status.get("ready") is False and output == Twist()
    print(json.dumps({"fall_latched_and_guard_zero": locked}), flush=True)
    return 0 if (record["stand_drift_m"] < .06 and record["walk_dx_m"] > .65
                 and abs(record["walk_dy_m"]) < .35 and change > .65
                 and record["stop_tail_drift_m"] < .06 and record["height_range_m"][0] > .25
                 and record["max_tilt_rad"] < .45 and len(ranges) == 12
                 and sum(val > .12 for val in ranges) >= 8 and record["four_feet_alternate"]
                 and record["four_feet_support"] and not record["unexpected_contacts"]
                 and record["ready_after_gait"] and locked) else 1


if __name__ == "__main__":
    raise SystemExit(main())
