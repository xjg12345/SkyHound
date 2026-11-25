#!/usr/bin/env python3
import argparse
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rospy
import tf2_ros
import yaml
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from prometheus_quadruped_sim.physical_model import build_physical
from prometheus_quadruped_sim.unitree_visual import apply_unitree_a1_visual
from prometheus_quadruped_sim.proxy_visual import apply_proxy_visual


def make_map(world):
    msg = OccupancyGrid()
    msg.header.frame_id = "dog_map"
    msg.info.resolution = 0.05
    msg.info.width, msg.info.height = 320, 240
    msg.info.origin.position.x, msg.info.origin.position.y = -8.0, -6.0
    msg.info.origin.orientation.w = 1.0
    model = ET.parse(world).find("./world/model[@name='map_obstacles']")
    if (model is None or model.findtext("static") != "true"
            or model.find("pose") is not None or model.find("./link/pose") is not None):
        raise ValueError("map_obstacles must be static with world-aligned model and link")
    boxes = []
    for box in model.findall("./link/collision"):
        x, y, _, roll, pitch, yaw = map(float, box.findtext("pose").split())
        sx, sy, _ = map(float, box.findtext("geometry/box/size").split())
        if any((roll, pitch, yaw)):
            raise ValueError("demo map only supports axis-aligned collision boxes")
        boxes.append((x, y, sx, sy))
    if not boxes:
        raise ValueError("demo map has no obstacle geometry")
    msg.data = [
        100 if any(abs(-8 + (i + 0.5) * 0.05 - x) <= sx / 2
                   and abs(-6 + (j + 0.5) * 0.05 - y) <= sy / 2 for x, y, sx, sy in boxes) else 0
        for j in range(msg.info.height) for i in range(msg.info.width)
    ]
    return msg


DEMO_TEAM = [(-5.0, -3.6, 0.0), (-5.0, -1.6, 0.0),
             (-5.0, 1.6, 0.0), (-5.0, 3.6, 0.0)]


def planning_map(world, config):
    settings = yaml.safe_load(Path(config).read_text())
    ox, oy = settings["origin"]
    sx, sy = settings["size"]
    resolution = settings["resolution"]
    if not all(math.isfinite(v) for v in (ox, oy, sx, sy, resolution)) or not (
            0.025 <= resolution <= 0.25 and 2 <= sx <= 60 and 2 <= sy <= 60):
        raise ValueError("invalid planning_easy map bounds or resolution")
    sdf = ET.parse(world).find("world")
    if [node.findtext("uri") for node in sdf.findall("include")] != ["model://ground_plane"]:
        raise ValueError("only the planning_easy ground plane include is supported")
    cylinders = {}
    for model in sdf.findall("model"):
        links = model.findall("link")
        if len(links) != 1 or links[0].find("pose") is not None:
            raise ValueError("unsupported link transform in " + model.get("name"))
        collision = links[0].findall("collision")
        if len(collision) != 1 or collision[0].find("pose") is not None:
            raise ValueError("unsupported collision transform in " + model.get("name"))
        x, y, _, roll, pitch, _ = map(float, model.findtext("pose").split())
        radius = float(collision[0].findtext("geometry/cylinder/radius"))
        if abs(roll) > 1e-6 or abs(pitch) > 1e-6 or radius <= 0:
            raise ValueError("expected upright collision cylinder")
        cylinders[model.get("name")] = (x, y, radius)
    if not cylinders:
        raise ValueError("planning_easy has no cylinders")
    grid = OccupancyGrid()
    grid.header.frame_id = "dog_map"
    grid.info.resolution = resolution
    width, height = round(sx / resolution), round(sy / resolution)
    grid.info.width, grid.info.height = width, height
    grid.info.origin.position.x, grid.info.origin.position.y = ox, oy
    grid.info.origin.orientation.w = 1.0
    poses = settings["spawn"]
    if len(poses) != 4:
        raise ValueError("attach map requires four birth poses")
    for index, (x, y, yaw) in enumerate(poses):
        if (not all(math.isfinite(v) for v in (x, y, yaw))
                or not (ox+0.6 < x < ox+sx-0.6 and oy+0.6 < y < oy+sy-0.6)
                or any(math.hypot(x-a, y-b) < r+0.6 for a, b, r in cylinders.values())
                or any(math.hypot(x-a, y-b) < 1.0 for a, b, _ in poses[:index])):
            raise ValueError("unsafe birth pose for dog{}".format(index + 1))
    return grid, poses, sdf.get("name"), cylinders


def rasterize_cylinders(grid, cylinders):
    width, height, res = grid.info.width, grid.info.height, grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = [0] * (width * height)
    for x, y, radius in cylinders.values():
        r = radius + res / math.sqrt(2)
        for j in range(max(0, math.floor((y-r-oy)/res)), min(height, math.ceil((y+r-oy)/res))):
            for i in range(max(0, math.floor((x-r-ox)/res)), min(width, math.ceil((x+r-ox)/res))):
                if math.hypot(ox+(i+0.5)*res-x, oy+(j+0.5)*res-y) <= r:
                    data[j*width+i] = 100
    grid.data = data


def validate_world(expected_name, cylinders, poses, uav):
    result = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
    if not result.success or result.model_names.count(uav) != 1:
        raise RuntimeError("P450 model {} not found in the active Gazebo".format(uav))
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=10)
    live = dict(zip(msg.name, msg.pose))
    velocities = dict(zip(msg.name, msg.twist))
    if set(cylinders) != {name for name in live if name.startswith("cylinder")}:
        raise RuntimeError("active Gazebo does not match the planning_easy cylinder set")
    if set(live) - set(cylinders) - {uav, "ground_plane"} or "ground_plane" not in live:
        raise RuntimeError("active Gazebo has unexpected models or no ground plane")
    settled = {}
    for name, (x, y, radius) in cylinders.items():
        p, v = live[name].position, velocities[name].linear
        # The source world contains coincident cylinders that separate on its first physics ticks.
        coincident = any(other != name and math.hypot(a-x, b-y) < 0.01
                         for other, (a, b, _) in cylinders.items())
        if math.hypot(p.x-x, p.y-y) > (0.35 if coincident else 0.15) or math.hypot(v.x, v.y) > 0.06:
            raise RuntimeError("{} moved; start {} with dynamic_obstacles_enable:=false".format(
                name, expected_name))
        settled[name] = (p.x, p.y, radius)
    if any(math.hypot(x-a, y-b) < r+0.6 for x, y, _ in poses for a, b, r in settled.values()):
        raise RuntimeError("settled cylinders obstruct a dog birth pose")
    p = live[uav].position
    if any(math.hypot(x-p.x, y-p.y) < 1.5 for x, y, _ in poses):
        raise RuntimeError("P450 is too close to a dog birth pose")
    rospy.loginfo("Verified %d planning_easy cylinders and %s", len(cylinders), uav)
    return settled


def plugin_library(filename):
    for directory in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep):
        if directory:
            candidate = Path(directory) / filename
            if candidate.is_file():
                return str(candidate.resolve())
    raise RuntimeError(filename + " not found; source the compiled quadruped overlay")


def spawn(name, path, x, y, yaw, bridge_scan=False, backend="proxy", a1_mesh_dir=""):
    sdf = ET.parse(path)
    model = sdf.find("model")
    model.set("name", name)
    if backend == "physical":
        build_physical(model)
        if a1_mesh_dir:
            apply_unitree_a1_visual(model, a1_mesh_dir)
    elif model.find("link[@name='chassis']/visual[@name='body']") is not None:
        apply_proxy_visual(model)
    for plugin in model.iter("plugin"):
        ns = plugin.find("robotNamespace")
        ns.text = "/" + name if ns.text.startswith("/") else name
        if plugin.get("filename") in ("libquadruped_proxy.so", "libquadruped_physical.so"):
            plugin.set("filename", plugin_library(plugin.get("filename")))
        if plugin.get("name") == "laser_ros":
            plugin.find("frameName").text = name + "/laser_link"
            if bridge_scan:
                plugin.find("topicName").text = "scan_sim"
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, 0.373 if backend == "physical" else 0.015
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    reply = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)(
        name, ET.tostring(sdf.getroot(), encoding="unicode"), "/", pose, "world")
    if not reply.success:
        raise RuntimeError("{}: {}".format(name, reply.status_message))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--dog", required=True)
    parser.add_argument("--crossing")
    parser.add_argument("--map-yaml")
    parser.add_argument("--uav", default="p450_D435i_0")
    parser.add_argument("--robot-count", type=int, choices=[1, 4], default=1)
    parser.add_argument("--backend", choices=["proxy", "physical"], default="proxy")
    parser.add_argument("--a1-mesh-dir", default="")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.a1_mesh_dir and args.backend != "physical":
        parser.error("--a1-mesh-dir requires --backend physical")
    rospy.init_node("quadruped_bringup")
    if args.map_yaml:
        grid, poses, world_name, cylinders = planning_map(args.world, args.map_yaml)
        poses = poses[:args.robot_count]
    else:
        grid = make_map(args.world)
        poses = DEMO_TEAM if args.robot_count == 4 else [(-5.0, 0.0, 0.0)]
    bridge_scan = bool(args.map_yaml)
    if args.crossing and args.robot_count != 1:
        raise ValueError("dynamic_obstacle is only supported in the single-dog demo")
    dogs = {"dog{}".format(i): pose for i, pose in enumerate(poses, 1)}
    map_pub = rospy.Publisher("/dog_map", OccupancyGrid, latch=True, queue_size=1)
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    scans = {dog: threading.Event() for dog in dogs}
    odoms = {dog: threading.Event() for dog in dogs}
    stands = {dog: threading.Event() for dog in dogs}
    subscribers = []
    for dog in dogs:
        if args.backend == "physical":
            subscribers.append(rospy.Subscriber(
                "/" + dog + "/physical_ready", Bool,
                lambda msg, dog=dog: stands[dog].set() if msg.data else stands[dog].clear()))
        else:
            stands[dog].set()
        subscribers.extend((
            rospy.Subscriber("/" + dog + "/scan", LaserScan,
                             lambda msg, dog=dog: scans[dog].set()
                             if msg.header.frame_id == dog + "/laser_link" and msg.ranges else None),
            rospy.Subscriber("/" + dog + "/odom", Odometry,
                             lambda msg, dog=dog: odoms[dog].set()
                             if msg.header.frame_id == dog + "/odom"
                             and msg.child_frame_id == dog + "/base_footprint" else None),
        ))
    errors = []
    done = threading.Event()
    owned = []

    def cleanup():
        while owned:
            name = owned.pop()
            try:
                result = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)(name)
                if not result.success:
                    rospy.logwarn("Cannot remove %s: %s", name, result.status_message)
            except rospy.ServiceException as exc:
                rospy.logwarn("Cannot remove %s: %s", name, exc)

    rospy.on_shutdown(cleanup)

    def spawn_models():
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=20)
            if args.map_yaml:
                rasterize_cylinders(grid, validate_world(world_name, cylinders, poses, args.uav))
            existing = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
            wanted = set(dogs) | ({"crossing"} if args.crossing else set())
            if not existing.success or wanted.intersection(existing.model_names):
                raise RuntimeError("Gazebo query failed or requested model names already exist")
            for dog, pose in dogs.items():
                if rospy.is_shutdown():
                    return
                spawn(dog, args.dog, *pose, bridge_scan=bridge_scan, backend=args.backend,
                      a1_mesh_dir=args.a1_mesh_dir)
                owned.append(dog)
                if not scans[dog].wait(5) or not odoms[dog].wait(3):
                    raise RuntimeError("{} did not publish its first scan and odometry".format(dog))
            if args.crossing:
                spawn("crossing", args.crossing, -2.5, -2.0, math.pi / 2)
                owned.append("crossing")
        except Exception as exc:
            errors.append(str(exc))
        finally:
            done.set()

    threading.Thread(target=spawn_models, daemon=True).start()
    if not done.wait(45) or errors:
        raise RuntimeError("spawn failed: {}".format(errors or "service timeout"))
    deadline = time.monotonic() + 15
    while not rospy.is_shutdown():
        pending = [dog for dog in dogs if not (scans[dog].is_set() and odoms[dog].is_set()
                   and stands[dog].is_set()
                   and buf.can_transform("dog_map", dog + "/laser_link", rospy.Time(0)))]
        if not pending and rospy.Time.now() > rospy.Time(0):
            for dog, (x, y, yaw) in dogs.items():
                tf = buf.lookup_transform("dog_map", dog + "/base_footprint", rospy.Time(0))
                p, q = tf.transform.translation, tf.transform.rotation
                delta = 2 * math.atan2(q.z, q.w) - yaw
                if math.hypot(p.x - x, p.y - y) > 0.06 or abs(math.atan2(math.sin(delta), math.cos(delta))) > 0.1:
                    raise RuntimeError(
                        "{} spawn pose and map TF disagree: got ({:.3f}, {:.3f}, {:.3f}), "
                        "expected ({:.3f}, {:.3f}, {:.3f})".format(
                            dog, p.x, p.y, delta + yaw, x, y, yaw))
            grid.header.stamp = grid.info.map_load_time = rospy.Time.now()
            map_pub.publish(grid)
            rospy.loginfo("%s ready: scan, odom, TF and /dog_map", ", ".join(dogs))
            rospy.spin()
            return
        if time.monotonic() > deadline:
            raise RuntimeError("scan, odom, TF or physical stand not ready for: " + ", ".join(pending))
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ET.ParseError) as exc:
        rospy.logfatal("%s", exc)
        raise SystemExit(1)
