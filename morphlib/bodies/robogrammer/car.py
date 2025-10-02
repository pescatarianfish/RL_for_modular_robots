from pyrr import Quaternion
import numpy as np
import copy
from .modules.root_body import RootBody
from .modules.link import Link
from .modules.rigid_joint import RigidJoint
from .modules.roll_joint import RollJoint
from .modules.twist_joint import TwistJoint
from .modules.elbow_joint import ElbowJoint

def leg(name):
    femur = Link(size=[0.025, 0.025], angle=np.pi/2)
    joint = RollJoint(name=name, size=[0.025, 0.025], color=[1.0, 0.4, 0, 1.0])
    tibia = Link(size=[0.025, 0.05])
    wheel = TwistJoint(name=name+"_wheel", size=[0.1, 0.025], color=[1.0, 0.4, 0, 1.0])
    femur.front = joint
    joint.front = tibia
    tibia.front = wheel

    return femur

def car():

    # Define the root body
    root = RootBody()
    head = Link()
    joint = RollJoint(name="body_joint")
    tail = Link()

    leg1 = leg("leg1")
    leg2 = leg("leg2")
    leg3 = leg("leg3")
    leg4 = leg("leg4")

    # Attach the parts
    root.attach = head
    head.front = joint
    joint.front = tail
    head.right = leg1
    head.left = leg2
    tail.right = leg3
    tail.left = leg4

    return root