from pyrr import Quaternion
import numpy as np
import copy 

from .modules.capsule import Capsule
from .modules.capsule_joint import CapsuleJoint
from .modules.root_body import RootBody
from .modules.foot import Foot

# Define a function to rotate a point around a center
rotate = lambda cx, cy, px, py, theta: (np.cos(theta) * (px-cx) - np.sin(theta) * (py-cy) + cx,
                                        np.sin(theta) * (px-cx) + np.cos(theta) * (py-cy) + cy)

def calc_height(thigh_length, shin_length, foot_length, thigh_angle, shin_angle, foot_angle):
    '''Calculate the height of the foot given the lengths of the thigh, shin and foot and the angles of the thigh, shin and foot.
    Calculations are based on the forward kinematics of the leg.'''

    # Thigh
    knee_x, knee_y = rotate(0, 0, thigh_length, 0, thigh_angle)
    # Shin
    lx, ly = rotate(0, 0, thigh_length+shin_length, 0, thigh_angle) # knee position + shin length
    ankle_x, ankle_y = rotate(knee_x, knee_y, lx, ly, -shin_angle)
    # Foot
    lx, ly = rotate(0, 0, thigh_length+shin_length+foot_length, 0, thigh_angle) # knee position + shin length + foot length
    lx, ly = rotate(knee_x, knee_y, lx, ly, -shin_angle) # ankle position + shin length + foot length
    _, height = rotate(ankle_x,ankle_y, lx, ly, foot_angle)

    return -height + 0.05 # 0.05 is the end of the foot/capsule

def half_cheetah(torso_length : float = 1.0, 
                 head_length : float = 0.3, 
                 bthigh_length : float = 0.29, 
                 bshin_length : float = 0.3, 
                 bfoot_length : float = 0.188, 
                 fthigh_length : float = 0.266, 
                 fshin_length : float = 0.212, 
                 ffoot_length : float = 0.14):

    '''Angles where measured from the original implementation of the HalfCheetah using a protractor and a ruler. 
    Therefore the angles are not exact but they are close enough to the original implementation.'''
    
    bthigh_angle = 2.47837 # 142 degrees
    bshin_angle = 1.74533 # 100 degrees
    bfoot_angle = 1.39626 # 80 degrees 

    fthigh_angle = -2.1293 # -122 degrees
    fshin_angle = -1.11701 # -64 degrees
    ffoot_angle = 0 # 0 degrees

    b_height = calc_height(-bthigh_length, -bshin_length, -bfoot_length, bthigh_angle, bshin_angle, bfoot_angle)
    f_height = calc_height(fthigh_length, fshin_length, ffoot_length, fthigh_angle, fshin_angle, ffoot_angle)
    h = max(b_height, f_height)

    root = RootBody(pos=[0,0,h], 
                    cam_pos = [0, -3, 0.3], 
                    cam_xyaxes = [1, 0, 0, 0, 0, 1], 
                    rootx_pos = [0,0,0], 
                    rooty_pos = [0,0,0], 
                    rootz_pos = [0,0,0], 
                    refx = 0, 
                    refy = 0, 
                    refz = 0, 
                    margin = 0, 
                    dim = 1, 
                    name = "root")
    
    torso_geom = Capsule(size=[0.046,torso_length/2], 
                         angles = [0,0],
                         friction=[0.4, 0.1, 0.1], 
                         density=1000, 
                         conaffinity=0, 
                         condim=3, 
                         contype=1, 
                         margin=0, 
                         solimp=[0.0, 0.8, 0.01], 
                         solref=[0.02, 1], 
                         name="torso", 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    head_geom = Capsule(size=[0.046,head_length/2], 
                         angles = [0.7,0], 
                         friction=[0.4, 0.1, 0.1], 
                         density=1000, 
                         conaffinity=0, 
                         condim=3, 
                         contype=1, 
                         margin=0.0, 
                         solimp=[0.0, 0.8, 0.01], 
                         solref=[0.02, 1], 
                         name="head", 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    capj_template = CapsuleJoint(name="tmp", 
                         axis=[0, 1, 0], 
                         range=[-0.52, 1.05], 
                         ctrllimited=True, 
                         ctrlrange=[-1,1], 
                         gear=[120], 
                         size=[0.046, bthigh_length/2], 
                         angles=[bthigh_angle,0], 
                         friction=[0.4, 0.1, 0.1], 
                         density=1000, 
                         conaffinity=0, 
                         condim=3, 
                         contype=1, 
                         margin=None, 
                         solimp=[0.0, 0.8, 0.01], 
                         solimplimit=[0, 0.8, 0.03], 
                         solref=[0.02, 1], 
                         solreflimit=[0.02, 1], 
                         armature=0.1, 
                         damping=6,
                         stiffness=240, 
                         color=[0.8, 0.6, 0.4, 1.0])
    
    bthigh = copy.deepcopy(capj_template)
    bthigh.name = "bthigh"

    bshin = copy.deepcopy(capj_template)
    bshin.name = "bshin"
    bshin.range, bshin.gear, bshin.size = [-0.785, 0.785], [90], [0.046, bshin_length/2]
    bshin.angles, bshin.damping, bshin.stiffness = [bshin_angle,0], 4.5, 180

    bfoot = copy.deepcopy(capj_template)
    bfoot.name = "bfoot"
    bfoot.range, bfoot.gear, bfoot.size = [-0.4, 0.785], [60], [0.046, bfoot_length/2]
    bfoot.angles, bfoot.damping, bfoot.stiffness = [bfoot_angle,0], 3, 120

    fthigh = copy.deepcopy(capj_template)
    fthigh.name = "fthigh"
    fthigh.range, fthigh.gear, fthigh.size = [-1, 0.7], [120], [0.046, fthigh_length/2]
    fthigh.angles, fthigh.damping, fthigh.stiffness = [fthigh_angle, 0], 4.5, 180
    
    fshin = copy.deepcopy(capj_template)
    fshin.name = "fshin"
    fshin.range, fshin.gear, fshin.size = [-1.2, 0.87], [60], [0.046, fshin_length/2]
    fshin.angles, fshin.damping, fshin.stiffness = [fshin_angle, 0], 3, 120

    ffoot = copy.deepcopy(capj_template)
    ffoot.name = "ffoot"
    ffoot.range, ffoot.gear, ffoot.size = [-0.5, 0.5], [30], [0.046, ffoot_length/2]
    ffoot.angles, ffoot.damping, ffoot.stiffness = [ffoot_angle, 0], 1.5, 60

    root.attach = torso_geom

    torso_geom.front1 = head_geom
    torso_geom.front2 = fthigh
    fthigh.front1 = fshin
    fshin.front1 = ffoot

    torso_geom.back1 = bthigh
    bthigh.front1 = bshin
    bshin.front1 = bfoot

    return root