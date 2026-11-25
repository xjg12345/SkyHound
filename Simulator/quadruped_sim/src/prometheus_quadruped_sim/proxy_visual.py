"""Standalone SDF visuals for the proxy; no meshes or collision changes."""
import math

from .physical_model import add


def apply_proxy_visual(model):
    body = model.find("link[@name='chassis']")
    body.remove(body.find("visual[@name='body']"))
    orange = "0.92 0.39 0.045 1"
    graphite = "0.12 0.14 0.17 1"
    metal = "0.34 0.38 0.42 1"
    glass = "0.035 0.075 0.10 1"
    cyan = "0.10 0.72 0.88 1"

    def visual(name, kind, size, xyz, color, rpy=(0, 0, 0), glow=False):
        node = add(body, "visual", name=name)
        add(node, "pose", " ".join(f"{v:.5f}" for v in (*xyz, *rpy)))
        geometry = add(add(node, "geometry"), kind)
        if kind == "box":
            add(geometry, "size", " ".join(map(str, size)))
        else:
            add(geometry, "radius", str(size[0]))
            add(geometry, "length", str(size[1]))
        material = add(node, "material")
        add(material, "ambient", color)
        add(material, "diffuse", color)
        add(material, "specular", "0.23 0.23 0.23 1")
        if glow:
            add(material, "emissive", "0.03 0.28 0.37 1")

    visual("chassis_core", "box", (.48, .196, .087), (0, 0, .183), graphite)
    visual("back_shell", "box", (.46, .222, .036), (0, 0, .239), orange)
    visual("front_bumper", "box", (.048, .184, .072), (.254, 0, .199), graphite)
    visual("rear_bumper", "box", (.036, .171, .064), (-.244, 0, .195), graphite)
    visual("camera_face", "box", (.012, .133, .042), (.282, 0, .207), glass)
    visual("lidar_pedestal", "cylinder", (.041, .018), (0, 0, .267), graphite)
    visual("lidar_housing", "cylinder", (.036, .034), (0, 0, .293), metal)
    visual("lidar_band", "cylinder", (.037, .011), (0, 0, .307), glass)
    for side in (-1, 1):
        label = "left" if side > 0 else "right"
        visual(label + "_rail", "box", (.35, .017, .029), (0, side*.104, .204), orange)
        visual(label + "_headlight", "box", (.008, .031, .014),
               (.290, side*.039, .213), cyan, glow=True)

    def segment(name, a, b, width, depth, color):
        dx, dz = b[0] - a[0], b[2] - a[2]
        center = tuple((p + q) / 2 for p, q in zip(a, b))
        visual(name, "box", (width, depth, math.hypot(dx, dz)), center, color,
               (0, math.atan2(dx, dz), 0))

    # Everything is attached to one link: the posed legs move with the chassis.
    for name, x, y in (("fl", .191, .132), ("fr", .191, -.132),
                       ("rl", -.191, .132), ("rr", -.191, -.132)):
        hip, knee, ankle = (x, y, .159), (x + math.copysign(.046, x), y, .091), (x, y, .026)
        segment(name + "_thigh", hip, knee, .041, .041, orange)
        segment(name + "_shin", knee, ankle, .026, .030, metal)
        visual(name + "_hip", "cylinder", (.035, .049), hip, graphite, (math.pi/2, 0, 0))
        visual(name + "_knee", "cylinder", (.021, .045), knee, graphite, (math.pi/2, 0, 0))
        visual(name + "_foot", "box", (.069, .057, .022), (x, y, .013), graphite)
