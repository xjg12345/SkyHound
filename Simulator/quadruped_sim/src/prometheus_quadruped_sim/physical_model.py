"""Build the articulated SDF from the shared laser and bumper template."""
import copy
import math
import xml.etree.ElementTree as ET


def add(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    node.text = text
    return node


def shape(parent, kind, dimensions):
    geometry = add(parent, "geometry")
    add(add(geometry, kind), "size" if kind == "box" else "radius", dimensions)


_SHELL = "0.94 0.39 0.055 1"
_GRAPHITE = "0.11 0.13 0.16 1"
_ALLOY = "0.34 0.39 0.44 1"
_GLASS = "0.04 0.11 0.16 1"


def visual(link, name, pose, kind, dimensions, color):
    node = add(link, "visual", name=name)
    add(node, "pose", pose)
    if kind == "cylinder":
        radius, length = dimensions.split()
        cylinder = add(add(node, "geometry"), kind)
        add(cylinder, "radius", radius)
        add(cylinder, "length", length)
    else:
        shape(node, kind, dimensions)
    material = add(node, "material")
    add(material, "ambient", color)
    add(material, "diffuse", color)
    add(material, "specular", "0.22 0.22 0.22 1")


def chassis_visuals(body):
    body.remove(body.find("visual"))
    visual(body, "main_case", "0 0 -0.006 0 0 0", "box", "0.53 0.22 0.108", _GRAPHITE)
    visual(body, "upper_shell", "0 0 0.052 0 0 0", "box", "0.49 0.232 0.025", _SHELL)
    for side, y in (("left", 0.108), ("right", -0.108)):
        visual(body, side + "_panel", f"0 {y} 0.021 0 0 0", "box", "0.39 0.018 0.048", _SHELL)
    visual(body, "nose_bumper", "0.251 0 -0.019 0 0 0", "box", "0.038 0.184 0.060", _ALLOY)
    visual(body, "tail_bumper", "-0.252 0 -0.019 0 0 0", "box", "0.036 0.17 0.056", _ALLOY)
    visual(body, "front_sensor_cover", "0.263 0 0.027 0 0 0", "box", "0.012 0.125 0.032", _GLASS)
    visual(body, "lidar_mount", "0 0 0.082 0 0 0", "cylinder", "0.035 0.036", _GRAPHITE)
    visual(body, "lidar_cap", "0 0 0.104 0 0 0", "cylinder", "0.032 0.016", _ALLOY)
    visual(body, "lidar_window", "0 0 0.120 0 0 0", "cylinder", "0.031 0.015", _GLASS)


def leg_visuals(link, segment, y):
    # All details are in their existing link frame and follow the physics joints.
    side = math.copysign(1, y)
    if segment == "hip":
        visual(link, "hip_motor", f"0 {side * 0.013} 0 1.5708 0 0",
               "cylinder", "0.036 0.058", _GRAPHITE)
        visual(link, "hip_motor_cap", f"0 {side * 0.043} 0 1.5708 0 0",
               "cylinder", "0.026 0.008", _ALLOY)
    elif segment == "thigh":
        visual(link, "thigh_core", "0 0 -0.100 0 0 0", "box", "0.039 0.042 0.17", _GRAPHITE)
        visual(link, "thigh_shell", f"0 {side * 0.019} -0.091 0 0 0",
               "box", "0.036 0.012 0.131", _SHELL)
    else:
        visual(link, "knee_motor", "0 0 0 1.5708 0 0", "cylinder", "0.024 0.051", _GRAPHITE)
        visual(link, "shin_core", "0 0 -0.100 0 0 0", "box", "0.039 0.042 0.17", _GRAPHITE)
        visual(link, "shin_guard", f"0 {side * 0.018} -0.093 0 0 0",
               "box", "0.029 0.012 0.113", _ALLOY)
        visual(link, "rubber_foot", "0 0 -0.20 0 0 0", "sphere", "0.027", _GRAPHITE)


def build_physical(model):
    body = model.find("link")
    body.find("inertial/pose").text = "0 0 0 0 0 0"
    body.find("inertial/mass").text = "5.0"
    collision = body.find("collision")
    collision.find("pose").text = "0 0 0 0 0 0"
    collision.find("geometry/box/size").text = "0.54 0.24 0.13"
    body.find("sensor[@name='laser']/pose").text = "0 0 0.12 0 0 0"
    chassis_visuals(body)
    for name, x, y in (("fl", .22, .125), ("fr", .22, -.125),
                       ("rl", -.22, .125), ("rr", -.22, -.125)):
        parent = "chassis"
        bend = math.copysign(math.acos(.344 / .4), x)
        for segment, dx, z, pitch, axis, lo, hi in (
                ("hip", 0, 0, 0, "1 0 0", -.75, .75),
                ("thigh", 0, 0, -bend, "0 1 0", -1.5 + bend, 1.3 + bend),
                ("calf", .2 * math.sin(bend), -.172, bend, "0 1 0",
                 min(.04 - 2*abs(bend), 2.6 - 2*abs(bend)) if x > 0 else -2.6 - 2*bend,
                 2.6 - 2*bend if x > 0 else -.04 - 2*bend)):
            link_name = name + "_" + segment
            link = add(model, "link", name=link_name)
            add(link, "pose", f"{x + dx} {y} {z} 0 {pitch} 0")
            inertial = add(link, "inertial")
            hip = segment == "hip"
            add(inertial, "pose", "0 0 0 0 0 0" if hip else "0 0 -0.10 0 0 0")
            add(inertial, "mass", "0.08" if hip else "0.22")
            inertia = add(inertial, "inertia")
            for key in ("ixx", "iyy", "izz"):
                add(inertia, key, "0.0008" if not hip else "0.0002")
            leg_visuals(link, segment, y)
            if not hip:
                item = add(link, "collision", name=segment)
                add(item, "pose", "0 0 -0.10 0 0 0")
                shape(item, "box", "0.039 0.042 0.17")
            if segment == "calf":
                foot = add(link, "collision", name="foot")
                add(foot, "pose", "0 0 -0.20 0 0 0")
                shape(foot, "sphere", "0.027")
                ode = add(add(add(foot, "surface"), "friction"), "ode")
                add(ode, "mu", "1.1")
                add(ode, "mu2", "1.1")
                sensor = copy.deepcopy(body.find("sensor[@name='contacts']"))
                sensor.set("name", name + "_contact")
                sensor.find("contact/collision").text = "foot"
                sensor.find("plugin/bumperTopicName").text = "feet/" + name
                link.append(sensor)
            joint = add(model, "joint", name=link_name + "_joint", type="revolute")
            add(joint, "parent", parent)
            add(joint, "child", link_name)
            joint_axis = add(joint, "axis")
            add(joint_axis, "xyz", axis)
            limit = add(joint_axis, "limit")
            for tag, value in (("lower", lo), ("upper", hi), ("effort", 25), ("velocity", 15)):
                add(limit, tag, str(value))
            add(add(joint_axis, "dynamics"), "damping", "0.12")
            parent = link_name
    model.find("plugin").set("filename", "libquadruped_physical.so")
