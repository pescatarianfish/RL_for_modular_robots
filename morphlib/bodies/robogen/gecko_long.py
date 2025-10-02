
from pyrr import Quaternion
import numpy as np
from .modules.core import Core
from .modules.brick import Brick
from .modules.active_joint import ActiveJoint
from .modules.root_body import RootBody

def gecko_long(position_ctrl=False):
    root = RootBody(pos=[0,0,0.0603], cam_pos=[0, -3, 0.5], cam_xyaxes=[1, 0, 0, 0, 0, 1], name="root")

    core = Core(name="core")
    neck = ActiveJoint(name="neck", horizontal=True, position_ctrl=position_ctrl)
    neck2 = ActiveJoint(name="neck2", horizontal=False, position_ctrl=position_ctrl)
    abdomen = Brick(name="abdomen")
    spine = ActiveJoint(name="spine", horizontal=True, position_ctrl=position_ctrl)
    spine2 = ActiveJoint(name="spine2", horizontal=False, position_ctrl=position_ctrl)
    butt = Brick(name="butt")

    fr_leg = ActiveJoint(name="fr_leg", horizontal=False, position_ctrl=position_ctrl)
    fr_leg2 = ActiveJoint(name="fr_leg2", horizontal=True, position_ctrl=position_ctrl)
    fl_leg = ActiveJoint(name="fl_leg", horizontal=False, position_ctrl=position_ctrl)
    fl_leg2 = ActiveJoint(name="fl_leg2", horizontal=True, position_ctrl=position_ctrl)
    br_leg = ActiveJoint(name="br_leg", horizontal=False, position_ctrl=position_ctrl)
    br_leg2 = ActiveJoint(name="br_leg2", horizontal=True, position_ctrl=position_ctrl)
    bl_leg = ActiveJoint(name="bl_leg", horizontal=False, position_ctrl=position_ctrl)
    bl_leg2 = ActiveJoint(name="bl_leg2", horizontal=True, position_ctrl=position_ctrl)

    fl_flipper = Brick(name="fl_flipper")
    fr_flipper = Brick(name="fr_flipper")
    bl_flipper = Brick(name="bl_flipper")
    br_flipper = Brick(name="br_flipper")

    root.attach = core
    core.back = neck
    neck.front = neck2
    neck2.front = abdomen
    abdomen.front = spine
    spine.front = spine2
    spine2.front = butt

    butt.left = br_leg
    br_leg.front = br_leg2
    br_leg2.front = br_flipper

    butt.right = bl_leg
    bl_leg.front = bl_leg2
    bl_leg2.front = bl_flipper

    core.right = fr_leg
    fr_leg.front = fr_leg2
    fr_leg2.front = fr_flipper

    core.left = fl_leg
    fl_leg.front = fl_leg2
    fl_leg2.front = fl_flipper
                    
    return root