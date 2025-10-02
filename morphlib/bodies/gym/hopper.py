from pyrr import Quaternion
import numpy as np
from .modules.capsule import Capsule
from .modules.capsule_joint import CapsuleJoint
from .modules.root_body import RootBody
from .modules.foot import Foot




def hopper(torso_length : float = 0.4,
           femur_length : float = 0.45,
           tibia_length : float = 0.5,
           foot_length : float = 0.39):

    # torso_length/2 + femur_length + tibia_length + foot_thickness
    h = torso_length + femur_length + tibia_length + 0.1
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
    
    torso_geom = Capsule(size=[0.05,torso_length/2], 
                         angles = [-np.pi/2,0], 
                         friction=[0.9], 
                         density=1000, 
                         conaffinity=1, 
                         condim=1, 
                         contype=1, 
                         margin=0.001, 
                         solimp=[0.8, 0.8, 0.01], 
                         solref=[0.02, 1], 
                         name="torso", 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    femur = CapsuleJoint(name="femur", 
                         axis=[0, 1, 0], 
                         range=[-150*np.pi/180, 0], 
                         ctrllimited=True, 
                         ctrlrange=[-1,1], 
                         gear=[200], 
                         size=[0.05, femur_length/2], 
                         angles=[0,0], 
                         friction=[0.9], 
                         density=1000, 
                         conaffinity=1, 
                         condim=1, 
                         contype=1, 
                         margin=0.001, 
                         solimp=[0.8, 0.8, 0.01], 
                         solref=[0.02, 1], 
                         armature=1, 
                         damping=1, 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    tibia = CapsuleJoint(name="tibia", 
                       axis=[0,-1, 0], 
                       range=[-150*np.pi/180, 0], 
                       ctrllimited=True, 
                       ctrlrange=[-1,1], 
                       gear=[200], 
                       size=[0.04, tibia_length/2], 
                       angles=[0,0], 
                       friction=[0.9], 
                       density=1000, 
                       conaffinity=1, 
                       condim=1, 
                       contype=1, 
                       margin=0.001, 
                       solimp=[0.8, 0.8, 0.01], 
                       solref=[0.02, 1], 
                       armature=1, 
                       damping=1, 
                       color=[0.8, 0.6, 0.4, 1.0])
    
    foot = Foot(name="foot", 
                axis=[-1,0,0], 
                range=[-np.pi/4, np.pi/4], 
                ctrlrange=[-1,1], 
                gear=[200], 
                size=[0.06, foot_length/2], 
                ankle_ratio=1/3, 
                friction=[2.0], 
                density=1000, 
                conaffinity=1, 
                condim=1, 
                contype=1, 
                margin=0.001, 
                solimp=[0.8, 0.8, 0.01], 
                solref=[0.02, 1], 
                armature=1, 
                damping=1, 
                color=[0.8, 0.6, 0.4, 1.0])
    
    root.attach = torso_geom
    torso_geom.front1 = femur
    femur.front1 = tibia
    tibia.front1 = foot

    return root