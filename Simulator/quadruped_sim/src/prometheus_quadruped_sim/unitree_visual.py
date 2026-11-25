"""Fit official A1 visual meshes to the existing M3 links, without URDF conversion."""
import math
from pathlib import Path

from .physical_model import add


def apply_unitree_a1_visual(model, directory):
    folder = Path(directory).expanduser().resolve()
    names = ("trunk", "hip", "thigh", "thigh_mirror", "calf")
    files = {name: folder / (name + ".dae") for name in names}
    missing = [path.name for path in (*files.values(), folder / "trunk_A1.png")
               if not path.is_file()]
    if missing:
        raise ValueError(f"Missing A1 visual assets in {folder}: {', '.join(missing)}; "
                         "run scripts/fetch_unitree_a1.sh")

    def mesh(link_name, asset, keep=(), roll=0, pitch=0, scale="1 1 1"):
        link = model.find(f"link[@name='{link_name}']")
        for node in link.findall("visual"):
            if node.get("name") not in keep:
                link.remove(node)
        node = add(link, "visual", name="a1_" + asset)
        add(node, "pose", f"0 0 0 {roll} {pitch} 0")
        geometry = add(add(node, "geometry"), "mesh")
        add(geometry, "uri", "file://" + str(files[asset]))
        add(geometry, "scale", scale)

    mesh("chassis", "trunk", keep=("lidar_mount", "lidar_cap", "lidar_window"),
         scale="1.18 1.14 1")
    for leg in ("fl", "fr", "rl", "rr"):
        right = leg.endswith("r")
        rear = leg.startswith("r")
        mesh(leg + "_hip", "hip", roll=math.pi if right else 0, pitch=math.pi if rear else 0)
        mesh(leg + "_thigh", "thigh_mirror" if right else "thigh")
        mesh(leg + "_calf", "calf", keep=("rubber_foot",))
