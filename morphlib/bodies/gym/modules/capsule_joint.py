from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

from .capsule import Capsule

class CapsuleJoint(Capsule):

    def __init__(
        self,
        name: str,
        axis : list,
        range : list,
        ctrllimited : bool,
        ctrlrange : list,
        gear : list,
        size : list,
        limited : bool = True,
        angles: list = [0,0],
        friction: list = [1, 0.005, 0.0001],
        density : int = 1000,
        conaffinity : int = 1,
        condim : int = 3,
        contype : int = 1,
        margin : int = 0,
        solimp : list = None,
        solimplimit : list = None,
        solref : list = None,
        solreflimit : list = None,
        armature : float = 0,
        damping : float = 0,
        stiffness : float = 0,
        color: list = [0.2, 1.0, 0.2, 1.0]
    ):  
        self.axis = axis
        self.limited = limited
        self.range = range
        self.ctrllimited = ctrllimited
        self.ctrlrange = ctrlrange
        self.gear = gear
        self.armature = armature
        self.damping = damping
        self.stiffness = stiffness
        self.solimplimit = solimplimit
        self.solreflimit = solreflimit
        self.color = color

        super().__init__(size, angles, friction, density, conaffinity, condim, contype, margin, solimp, solref, name, color)


    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list:
        """
        Builds the object in a simulation file.

        This method is responsible for constructing the object in a given simulation file. 
        It uses the provided parameters to determine where and how the object should be attached.

        :param mjcf: The MJCF file object to be written to.
        :type mjcf: Mujoco XML object

        :param entry_point: The body in the file the build should write to.
        :type entry_point: str

        :param attachment_point_pos: The position where the attachment point of the previous module was that this module is going to be attached to.
        :type attachment_point_pos: list

        :param attachment_point_quat: The orientation given as a quaternion where the attachment point of the previous module was that this module is going to be attached to.
        :type attachment_point_quat: list

        :return: The children modules of this module that are next to be built
        """
        # Build the module
        theta_x = self.angles[0]
        theta_y = self.angles[1]
        

        x = self.size[1]*np.cos(theta_y)*np.cos(theta_x) # cos, cos
        y = self.size[1]*np.sin(theta_y)*np.cos(theta_x) # sin, cos
        z = self.size[1]*np.sin(theta_x) # sin
        
        new_quat = attachment_point_quat * (Quaternion.from_eulers([np.pi, np.pi/2, 0]) * Quaternion.from_eulers([theta_y, theta_x, 0]))
        mj_quat = [new_quat[3], new_quat[0], new_quat[1], new_quat[2]]
        new_pos = attachment_point_pos + attachment_point_quat*Vector3([x, y, z])

        joint_body = entry_point.add("body", pos=new_pos, quat=mj_quat)
        joint_body.add("joint", name=self.name, type="hinge", axis=self.axis, pos=[0,0,-self.size[1]], limited=self.limited, range=self.range, armature=self.armature, damping=self.damping, stiffness=self.stiffness, solimplimit=self.solimplimit, solreflimit=self.solreflimit)
        joint_body.add("geom", name=self.name, type="capsule", size=self.size, pos=[0,0,0], quat=Quaternion(), rgba=self.color, friction=self.friction, density=self.density, conaffinity=self.conaffinity, condim=self.condim, contype=self.contype, margin=self.margin, solimp=self.solimp, solref=self.solref)
        mjcf.actuator.add("motor", ctrllimited=self.ctrllimited, ctrlrange=self.ctrlrange, gear=self.gear, joint=self.name)
        
        # Find the children
        tasks = []

        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=joint_body)
                unbuilt.make_pose(
                    position= Vector3([0,0,self.size[1]]), # Prev pos + offset rotated in right direction = attachment point pos
                    orientation= Quaternion.from_eulers([0,-np.pi/2,0]),
                )
                tasks.append(unbuilt)

        return tasks
