from pyrr import Quaternion
import numpy as np
import copy
from .modules.capsule import Capsule
from .modules.capsule_joint import CapsuleJoint
from .modules.sphere import Sphere
from .modules.body import Body
from .modules.target import Target

def reacher(link1_length : float = 0.2, link2_length : float = 0.2):
    
    root_geom = Capsule(size=[0.02,0.01], 
                         angles = [-np.pi/2,0], 
                         friction=[1, 0.1, 0.1], 
                         conaffinity=1, 
                         condim=1, 
                         contype=0, 
                         name="root", 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    arm_template = CapsuleJoint(name="arm", 
                         axis=[-1, 0, 0],
                         range=None, 
                         ctrllimited=True, 
                         ctrlrange=[-1,1], 
                         gear=[200], 
                         size=[0.01, 0.1], 
                         limited=False,
                         angles=[0,0], 
                         friction=[1, 0.1, 0.1], 
                         conaffinity=1, 
                         condim=1, 
                         contype=0, 
                         armature=1, 
                         damping=1, 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    body = Body(pos=[0,0,0], name="fingertip")

    fingertip = Sphere(size = [0.01],
                       name = "fingertip_geom",
                       conaffinity = None,
                       condim = None,
                       density = None,
                       friction = [1, 0.1, 0.1],
                       margin = None,
                       mass = None,
                       color = [0.8, 0.6, 0.4, 1.0])
    
    target = Target(name="target", size=0.009)
    
    joint_link1 = copy.deepcopy(arm_template)
    joint_link1.name = "joint_link1"
    joint_link1.angles = [np.pi/2, 0]
    joint_link1.size = [0.01, link1_length/2]
    joint_link2 = copy.deepcopy(arm_template)
    joint_link2.name = "joint_link2"
    joint_link2.limited = True
    joint_link2.range = [-3.0, 3.0]
    joint_link2.axis = [1, 0, 0]
    joint_link1.size = [0.01, link2_length/2]

    root_geom.back1 = joint_link1
    root_geom.back2 = target
    joint_link1.front1 = joint_link2
    joint_link2.front1 = body
    body.attach = fingertip

    return root_geom