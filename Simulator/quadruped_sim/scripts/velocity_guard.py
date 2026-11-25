#!/usr/bin/env python3
import math
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Guard:
    def __init__(self):
        self.dog = rospy.get_param("~dog")
        self.laser_frame = self.dog + "/laser_link"
        self.odom_frame = self.dog + "/odom"
        self.sim_time = rospy.get_param("/use_sim_time", False)
        self.physical = rospy.get_param("~physical", False)
        self.scan_output = (rospy.Publisher("scan", LaserScan, queue_size=1)
                            if rospy.get_param("~bridge_scan", False) else None)
        self.lock = threading.Lock()
        self.inputs = {}
        self.clock = rospy.Time(0)
        self.clock_wall = 0.0
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.output = rospy.Publisher("cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
        self.nav_scan = rospy.Publisher("nav_scan", LaserScan, queue_size=1)
        self.subscribers = [
            rospy.Subscriber("/clock", Clock, self.on_clock, queue_size=1),
            rospy.Subscriber("scan_input", LaserScan, self.on_scan, queue_size=1),
            rospy.Subscriber("odom", Odometry, lambda m: self.store("odom", m), queue_size=1),
            rospy.Subscriber("cmd_vel_nav", Twist, lambda m: self.store("cmd", m), queue_size=1),
        ]
        if self.physical:
            self.subscribers.extend([
                rospy.Subscriber("physical_ready", Bool, lambda m: self.store("ready", m), queue_size=1),
                rospy.Subscriber("physical_fault", Bool, lambda m: self.store("fault", m), queue_size=1),
            ])

    def store(self, key, msg):
        with self.lock:
            self.inputs[key] = (time.monotonic(), msg)

    def on_scan(self, msg):
        if msg.header.frame_id == self.laser_frame and msg.ranges:
            if not self.sim_time:
                with self.lock:
                    age = (self.clock - msg.header.stamp).to_sec()
                    if not -0.15 <= age <= 0.35 or time.monotonic() - self.clock_wall > 0.35:
                        return
                msg.header.stamp = rospy.Time.now() - rospy.Duration(max(0.0, age))
            self.store("scan", msg)
            if self.scan_output:
                self.scan_output.publish(msg)
            self.nav_scan.publish(msg)

    def on_clock(self, msg):
        with self.lock:
            if msg.clock < self.clock:
                self.inputs.clear()
                self.buffer.clear()
            if msg.clock != self.clock:
                self.clock = msg.clock
                self.clock_wall = time.monotonic()

    def command(self):
        now = time.monotonic()
        with self.lock:
            if now - self.clock_wall > 0.35:
                self.inputs.clear()
                return Twist()
            clock, values = (self.clock if self.sim_time else rospy.Time.now()), dict(self.inputs)
        if self.physical:
            for name, expected in (("ready", True), ("fault", False)):
                arrival, msg = values.get(name, (0, None))
                if now - arrival > 0.5 or msg.data != expected:
                    return Twist()
        for name in ("cmd", "scan", "odom"):
            arrival, msg = values.get(name, (0, None))
            if now - arrival > 0.5:
                return Twist()
            if name != "cmd" and not -0.1 <= (clock - msg.header.stamp).to_sec() <= 0.5:
                return Twist()
        try:
            tf = self.buffer.lookup_transform(self.odom_frame, self.laser_frame, rospy.Time(0))
        except tf2_ros.TransformException:
            return Twist()
        if not -0.1 <= (clock - tf.header.stamp).to_sec() <= 0.5:
            return Twist()
        cmd = values["cmd"][1]
        if not all(math.isfinite(v) for v in (cmd.linear.x, cmd.angular.z)):
            return Twist()
        safe = Twist()
        safe.linear.x = max(0.0, min(0.30, cmd.linear.x))
        safe.angular.z = max(-0.70, min(0.70, cmd.angular.z))
        return safe

    def run(self):
        while not rospy.is_shutdown():
            self.output.publish(self.command())
            time.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("velocity_guard")
    Guard().run()
