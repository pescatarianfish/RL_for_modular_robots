from pyrr import Quaternion
import numpy as np
import copy
from .modules.capsule import Capsule
from .modules.capsule_joint import CapsuleJoint
from .modules.root_body import RootBody
from .modules.foot import Foot




def walker2d(torso_length : float = 0.4, 
             rfemur_length : float = 0.45, 
             rtibia_length : float = 0.5, 
             rfoot_length : float = 0.2, 
             lfemur_length : float = 0.45, 
             ltibia_length : float = 0.5, 
             lfoot_length : float = 0.2):

    if rfemur_length + rtibia_length > lfemur_length + ltibia_length:
        h = torso_length + rfemur_length + rtibia_length + 0.1
    else:
        h = torso_length + lfemur_length + ltibia_length + 0.1
        
    root = RootBody(pos=[0,0, h], 
                    cam_pos = [0, -3, -0.25], 
                    cam_xyaxes = [1, 0, 0, 0, 0, 1], 
                    rootx_pos = [0,0,-h], 
                    rooty_pos = [0,0,-0.2], 
                    rootz_pos = [0,0,-h], 
                    refx = 0, 
                    refy = 0, 
                    refz = h, 
                    margin = 0, 
                    dim = 1, 
                    name = "root")
    
    torso_geom = Capsule(size=[0.05,0.2], 
                         angles = [-np.pi/2,0], 
                         friction=[0.9], 
                         density=1000, 
                         conaffinity=0, 
                         condim=3, 
                         contype=1, 
                         margin=0, 
                         solimp=None, 
                         solref=None, 
                         name="torso", 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    thigh = CapsuleJoint(name="thigh", 
                         axis=[0, 1, 0], 
                         range=[-150*np.pi/180, 0], 
                         ctrllimited=True, 
                         ctrlrange=[-1,1], 
                         gear=[100], 
                         size=[0.05, 0.225], 
                         angles=[0,0], 
                         friction=[0.9], 
                         density=1000, 
                         conaffinity=0, 
                         condim=3, 
                         contype=1, 
                         margin=0, 
                         solimp=None, 
                         solref=None, 
                         armature=0.01, 
                         damping=0.1, 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    leg = CapsuleJoint(name="leg", 
                       axis=[0,-1, 0], 
                       range=[-150*np.pi/180, 0], 
                       ctrllimited=True, 
                       ctrlrange=[-1,1], 
                       gear=[100], 
                       size=[0.04, 0.25], 
                       angles=[0,0], 
                       friction=[0.9], 
                       density=1000, 
                       conaffinity=0, 
                       condim=3, 
                       contype=1, 
                       margin=0, 
                       solimp=None, 
                       solref=None, 
                       armature=0.01, 
                       damping=0.1, 
                       color=[0.8, 0.6, 0.4, 1.0])
    
    foot = Foot(name="foot", 
                axis=[-1,0,0], 
                range=[-np.pi/4, np.pi/4], 
                ctrlrange=[-1,1], 
                gear=[100], 
                size=[0.06, 0.1], 
                ankle_ratio=1, 
                friction=[0.9], 
                density=1000, 
                conaffinity=0, 
                condim=3, 
                contype=1, 
                margin=0, 
                solimp=None, 
                solref=None, 
                armature=0.01, 
                damping=0.1, 
                color=[0.8, 0.6, 0.4, 1.0])
    
    left_thigh = copy.deepcopy(thigh)
    left_thigh.name = "left_thigh"
    left_thigh.color = [0.7,0.3,0.6,1.0]

    left_leg = copy.deepcopy(leg)
    left_leg.name = "left_leg"
    left_leg.color = [0.7,0.3,0.6,1.0]

    left_foot = copy.deepcopy(foot)
    left_foot.name = "left_foot"
    left_foot.color = [0.7,0.3,0.6,1.0]
    left_foot.friction = [1.9]

    root.attach = torso_geom
    torso_geom.front1 = thigh
    torso_geom.front2 = left_thigh
    thigh.front1 = leg
    left_thigh.front2 = left_leg
    leg.front1 = foot
    left_leg.front2 = left_foot

    return root