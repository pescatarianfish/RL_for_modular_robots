from pyrr import Quaternion
import numpy as np
from .modules.capsule import Capsule
from .modules.capsule_joint import CapsuleJoint
from .modules.root_body import RootBody
from .modules.foot import Foot




def swimmer(head_length : float = 1.0, torso_length : float = 1.0, tail_length : float = 1.0):

    root = RootBody(pos=[0,0, 0.1], 
                    cam_pos = [0, -3, 3], 
                    cam_xyaxes = [1, 0, 0, 0, 1, 1], 
                    rootx_pos = [0,0,0],
                    refx = 0, 
                    margin = 0.01, 
                    dim = 2, 
                    name = "root")
    
    head = Capsule(size=[0.1,head_length/2], 
                         angles = [0,0], 
                         friction=None, 
                         density=1000, 
                         conaffinity=1, 
                         condim=3, 
                         contype=1, 
                         solimp=None, 
                         solref=None, 
                         name="head", 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    torso = CapsuleJoint(name="torso", 
                         axis=[1, 0, 0], 
                         range=[-100*np.pi/180, 100*np.pi/180], 
                         ctrllimited=True, 
                         ctrlrange=[-1,1], 
                         gear=[200], 
                         size=[0.1, torso_length/2], 
                         angles=[0,0], 
                         friction=None, 
                         density=1000, 
                         conaffinity=1, 
                         condim=3, 
                         contype=1, 
                         margin=0, 
                         solimp=None, 
                         solref=None, 
                         armature=0.1, 
                         damping=None, 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    tail = CapsuleJoint(name="tail", 
                         axis=[1, 0, 0], 
                         range=[-100*np.pi/180, 100*np.pi/180], 
                         ctrllimited=True, 
                         ctrlrange=[-1,1], 
                         gear=[200], 
                         size=[0.1, tail_length/2], 
                         angles=[0,0], 
                         friction=None, 
                         density=1000, 
                         conaffinity=1, 
                         condim=3, 
                         contype=1, 
                         margin=0, 
                         solimp=None, 
                         solref=None, 
                         armature=0.1, 
                         damping=None, 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    root.attach = head
    head.front1 = torso
    torso.front1 = tail

    return root