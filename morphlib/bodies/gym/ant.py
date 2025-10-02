from pyrr import Quaternion
import numpy as np
import copy
from .modules.capsule import Capsule
from .modules.capsule_joint import CapsuleJoint
from .modules.root_body import RootBody
from .modules.sphere import Sphere

def create_leg_parts(name, coxa_tmp, femur_tmp, tibia_tmp, coxa_length, femur_length, tibia_length):
    coxa = copy.deepcopy(coxa_tmp)
    coxa.name = f"{name}_coxa"
    coxa.size = [0.08, coxa_length/2]
    
    femur = copy.deepcopy(femur_tmp)
    femur.name = f"{name}_femur"
    femur.size = [0.08, femur_length/2]

    tibia = copy.deepcopy(tibia_tmp)
    tibia.name = f"{name}_tibia"
    tibia.size = [0.08, tibia_length/2]

    return coxa, femur, tibia
    

def ant(sphere_radius : float = 0.25, 
        fl_coxa_length : float = 0.0328,
        fr_coxa_length : float = 0.0328,
        br_coxa_length : float = 0.0328,
        bl_coxa_length : float = 0.0328,
        fl_fem_length : float = 0.2828, 
        fr_fem_length : float = 0.2828, 
        br_fem_length : float = 0.2828, 
        bl_fem_length : float = 0.2828,
        fl_tib_length : float = 0.5657, 
        fr_tib_length : float = 0.5657,
        br_tib_length : float = 0.5657, 
        bl_tib_length : float = 0.5657):

    # Define the root body
    root = RootBody(pos=[0,0,0.75], 
                    cam_pos = [0, -3, -0.25], 
                    cam_xyaxes = [1, 0, 0, 0, 0, 1], 
                    rootx_pos = [0,0,0], 
                    refx = 0, 
                    refy = 0, 
                    refz = 0, 
                    margin = 0.01, 
                    dim = 0, 
                    name = "torso")
    
    # Define the torso
    torso = Sphere(size = [sphere_radius],
                    name = "torso_geom",
                    conaffinity = 0,
                    condim = 3,
                    density = 5.0,
                    friction = [1, 0.5, 0.5],
                    margin = 0.01,
                    mass = None,
                    color = [0.8, 0.6, 0.4, 1.0])
    
    # Define the leg templates for coxa, femur and tibia
    coxa_tmp = Capsule(size=[0.08, fl_coxa_length/2],
                        angles = [0,0],
                        friction = [1, 0.5, 0.5],
                        density = 5.0,
                        conaffinity = 0,
                        condim = 3,
                        contype = 1,
                        margin = 0.01,
                        solimp = None,
                        solref = None,
                        name = None,
                        color = [0.8, 0.6, 0.4, 1.0])
    
    femur_tmp = CapsuleJoint(name="tmp",
                            axis=[1,0,0],
                            range=[-30*np.pi/180, 30*np.pi/180],
                            ctrllimited=True,
                            ctrlrange=[-1,1],
                            gear=[150],
                            size=[0.08, fl_fem_length/2],
                            limited=True,
                            angles= [0,0],
                            friction = [1, 0.5, 0.5],
                            density = 5.0,
                            conaffinity = 0,
                            condim = 3,
                            contype = 1,
                            margin = 0.01,
                            solimp = None,
                            solimplimit = None,
                            solref = None,
                            solreflimit = None,
                            armature = 1,
                            damping = 1,
                            stiffness = None,
                            color = [0.8, 0.6, 0.4, 1.0])

    tibia_tmp = copy.deepcopy(femur_tmp)
    tibia_tmp.axis = [0,-1,0]
    tibia_tmp.range = [-70*np.pi/180, -30*np.pi/180]
    tibia_tmp.size = [0.08, fl_tib_length/2]

    # Define the leg parts
    fl_coxa, fl_femur, fl_tibia = create_leg_parts("fl", coxa_tmp, femur_tmp, tibia_tmp, fl_coxa_length, fl_fem_length, fl_tib_length)
    fr_coxa, fr_femur, fr_tibia = create_leg_parts("fr", coxa_tmp, femur_tmp, tibia_tmp, fr_coxa_length, fr_fem_length, fr_tib_length)
    br_coxa, br_femur, br_tibia = create_leg_parts("br", coxa_tmp, femur_tmp, tibia_tmp, br_coxa_length, br_fem_length, br_tib_length)
    bl_coxa, bl_femur, bl_tibia = create_leg_parts("bl", coxa_tmp, femur_tmp, tibia_tmp, bl_coxa_length, bl_fem_length, bl_tib_length)

    # Attach the parts
    root.attach = torso
    
    torso.north = fl_coxa
    torso.east = fr_coxa
    torso.south = br_coxa
    torso.west = bl_coxa

    fl_coxa.front1 = fl_femur
    fr_coxa.front1 = fr_femur
    br_coxa.front1 = br_femur
    bl_coxa.front1 = bl_femur
    
    fl_femur.front1 = fl_tibia
    fr_femur.front1 = fr_tibia
    br_femur.front1 = br_tibia
    bl_femur.front1 = bl_tibia

    return root