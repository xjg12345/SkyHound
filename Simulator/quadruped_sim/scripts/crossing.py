#!/usr/bin/env python3
"""Move a physical obstacle across the lane once, triggered by dog1's position."""
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("crossing_controller")
    out = rospy.Publisher("/crossing/cmd_vel", Twist, queue_size=1)
    phase = "waiting"
    positions = None

    def update(msg):
        nonlocal positions
        if {"dog1", "crossing"}.issubset(msg.name):
            positions = (msg.pose[msg.name.index("dog1")].position.x,
                         msg.pose[msg.name.index("crossing")].position.y)

    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, update, queue_size=1)
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        cmd = Twist()
        if positions is not None:
            dog_x, obstacle_y = positions
            if phase == "waiting" and dog_x >= -4.1:
                phase = "crossing"
            if phase == "crossing" and obstacle_y >= 2.0:
                phase = "done"
            if phase == "crossing":
                cmd.linear.x = 0.35
        out.publish(cmd)
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            phase, positions = "waiting", None


if __name__ == "__main__":
    main()
