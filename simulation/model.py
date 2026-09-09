"""Load the licensed Menagerie arm and align it with our ROS UR5e model."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ur5e_pose_control.pose_math import JOINTS

MODEL_DIR = Path(__file__).resolve().parent / "models" / "ur5e"


def load_model():
    """Adapt nominal dimensions; leave the upstream files unchanged on disk.

    Menagerie's frame convention and rounded dimensions differ from ur_description.
    See models/SOURCE.md. The runner also checks independent ROS FK samples.
    """
    arm = ET.parse(MODEL_DIR / "ur5e.xml").getroot()
    arm.find("compiler").set("meshdir", str(MODEL_DIR / "assets"))
    arm.find('.//body[@name="base"]').set("quat", "1 0 0 0")
    for name, position in {
        "shoulder_link": "0 0 0.1625",
        "wrist_1_link": "0 0 0.3922",
        "wrist_2_link": "0 0.1263 0",
        "wrist_3_link": "0 0 0.0997",
    }.items():
        arm.find(f'.//body[@name="{name}"]').set("pos", position)
    tool = arm.find('.//site[@name="attachment_site"]')
    tool.set("pos", "0 0.0996 0")
    tool.set("name", "tool0")
    tool.set("size", "0.008")
    tool.set("rgba", "0.1 0.9 0.2 1")
    tool.set("group", "0")

    # Merge the upstream lighting and ground plane into this adapted arm.
    scene = ET.parse(MODEL_DIR / "scene.xml").getroot()
    for child in scene:
        if child.tag == "include":
            continue
        existing = arm.find(child.tag)
        if existing is not None:
            existing.extend(child)
        else:
            arm.append(child)
    ET.SubElement(
        arm.find("worldbody"),
        "site",
        {
            "name": "target",
            "type": "sphere",
            "size": "0.012",
            "rgba": "1 0.4 0.1 0.6",
            "group": "0",
        },
    )
    model = mujoco.MjModel.from_xml_string(ET.tostring(arm, encoding="unicode"))
    # This demo controls exactly six joints, in shoulder-to-wrist order. Keeping
    # that order fixed lets the physics loop work directly with six-value arrays.
    if tuple(model.joint(i).name for i in range(model.njnt)) != JOINTS:
        raise ValueError("Expected the six-joint UR5e model.")
    if model.nu != 6 or not np.array_equal(model.actuator_trnid[:, 0], range(6)):
        raise ValueError("Expected one position actuator per UR5e joint, in order.")
    return model


def tool_pose(data):
    """Return simulated tool0 in base_link, with ROS quaternion order x/y/z/w."""
    site = data.site("tool0")
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, site.xmat)  # MuJoCo uses w/x/y/z.
    return site.xpos.copy(), quaternion[[1, 2, 3, 0]]
