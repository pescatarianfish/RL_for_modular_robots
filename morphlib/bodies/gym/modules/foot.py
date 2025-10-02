from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

class Foot(Module):

    ATTACH = 0

    def __init__(
        self,
        name: str,
        axis : list,
        range : list,
        ctrlrange : list,
        gear : list,
        size : list,
        ankle_ratio : float = 1/3,
        friction: list = [1, 0.005, 0.0001],
        density : int = 1000,
        conaffinity : int = 1,
        condim : int = 3,
        contype : int = 1,
        margin : int = 0,
        solimp : list = [0.8, 0.8, 0.01],
        solref : list = [0.02, 1.0],
        armature : float = 0,
        damping : float = 0,
        color: list = [0.2, 1.0, 0.2, 1.0]
    ):
        self.axis = axis
        self.range = range
        self.ctrlrange = ctrlrange
        self.gear = gear
        self.size = size
        self.ankle_ratio = ankle_ratio
        self.armature = armature
        self.damping = damping
        self.friction = friction
        self.density = density
        self.conaffinity = conaffinity
        self.condim = condim
        self.contype = contype
        self.margin = margin
        self.solimp = solimp
        self.solref = solref
        self.color = color

        self.name =  name
        attachment_points = {
            self.ATTACH: AttachmentPoint(
                offset=Vector3([size[1]/3, 0.0, 0.0]),
                orientation=Quaternion.from_eulers([0.0, 0.0, 0]),
            )
        }

        super().__init__(attachment_points)

    @property
    def attach(self) -> Module:
        return self._children.get(self.ATTACH)

    @attach.setter
    def attach(self, module: Module) -> None:
        self.set_child(module, self.ATTACH)


    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list[UnbuiltChild]:
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
        
        new_quat = Quaternion.from_eulers([0, 0, np.pi/2])
        mj_quat = [new_quat[3], new_quat[0], new_quat[1], new_quat[2]]
        new_pos = attachment_point_pos

        joint_body = entry_point.add("body", pos=new_pos, quat=mj_quat)
        joint_body.add("joint", name=self.name, type="hinge", axis=[-1,0,0], pos=[0,0,0], limited=True, range=[-np.pi/4, np.pi/4], armature=self.armature, damping=self.damping)
        joint_body.add("geom", name=self.name, type="capsule", size=self.size, pos=[0,self.size[1]*self.ankle_ratio,0.6*self.size[0]], euler=[np.pi/2,0,0], rgba=self.color, friction=self.friction, density=self.density, conaffinity=self.conaffinity, condim=self.condim, contype=self.contype, margin=self.margin, solimp=self.solimp, solref=self.solref)
        mjcf.actuator.add("motor", ctrllimited=True, ctrlrange=self.ctrlrange, gear=self.gear, joint=self.name)
        
        # Find the children
        tasks = []

        return tasks
