
from pyrr import Quaternion
import numpy as np
from .modules.core import Core
from .modules.brick import Brick
from .modules.active_joint import ActiveJoint
from .modules.root_body import RootBody

def snake(position_ctrl=False, reverse=False):
    root = RootBody(pos=[0,0,0.0603], cam_pos=[0, -3, 0.5], cam_xyaxes=[1, 0, 0, 0, 0, 1], name="root")

    core = Core(name="core")

    joints = []
    horizontal = False
    for i in range(12):
        joint = ActiveJoint(name=f"joint{i}", horizontal=horizontal, position_ctrl=position_ctrl)
        horizontal = not horizontal
        joints.append(joint)

    root.attach = core
    if reverse:
        core.front = joints[0]
    else:
        core.back = joints[0]
        
    for i in range(0,11):
        joints[i].front = joints[i+1]

    return root