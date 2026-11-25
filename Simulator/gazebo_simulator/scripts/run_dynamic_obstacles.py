#!/usr/bin/env python3
import math
import os
import xml.etree.ElementTree as ET

import rospy
import rospkg
from geometry_msgs.msg import Twist


def load_command_topics():
    pkg_path = rospkg.RosPack().get_path("prometheus_gazebo")
    world_path = os.path.join(
        pkg_path,
        "gazebo_worlds",
        "planning_worlds",
        "planning_easy.world",
    )
    root = ET.parse(world_path).getroot()
    topics = []
    for elem in root.iter("commandTopic"):
        topic = (elem.text or "").strip()
        if topic:
            topics.append(topic)
    return sorted(set(topics))


def main():
    rospy.init_node("dynamic_obstacles_controller", anonymous=False)
    topics = load_command_topics()
    pubs = [rospy.Publisher(topic, Twist, queue_size=1) for topic in topics]
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        for i, pub in enumerate(pubs):
            phase = i * 0.37
            msg = Twist()
            msg.linear.x = 0.45 * math.sin(0.27 * t + phase)
            msg.linear.y = 0.45 * math.cos(0.21 * t + phase)
            msg.angular.z = 0.35 * math.sin(0.19 * t + phase)
            pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
