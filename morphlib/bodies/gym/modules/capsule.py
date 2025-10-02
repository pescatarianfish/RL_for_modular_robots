from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

from .euler_to_quaternion import euler_from_quaternion

class Capsule(Module):

    FRONT1 = 0
    FRONT2 = 1
    BACK1 = 2
    BACK2 = 3

    def __init__(
        self,
        size : list,
        angles: list = [0,0],
        friction: list = [1, 0.005, 0.0001],
        density : int = 1000,
        conaffinity : int = 1,
        condim : int = 3,
        contype : int = 1,
        margin : int = 0,
        solimp : list = [0.8, 0.8, 0.01],
        solref : list = [0.02, 1.0],
        name: str = None,
        color: list = [0.8, 0.6, 0.4, 1.0] 
    ):  
        self.size = size
        self.angles = angles
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
        offset = Vector3([size[1], 0.0, 0.0])
        attachment_points = {
            self.FRONT1: AttachmentPoint(
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, 0.0]),
            ),  
            self.FRONT2: AttachmentPoint(
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, 0.0]),
            ),
            self.BACK1: AttachmentPoint(
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, np.pi, 0.0]),
            ),  
            self.BACK2: AttachmentPoint(
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, np.pi, 0.0]),
            ),
        }

        super().__init__(attachment_points)

    @property
    def front1(self) -> Module :
        return self._children.get(self.FRONT1)

    @front1.setter
    def front1(self, module: Module) -> None:
        self.set_child(module, self.FRONT1)

    @property
    def front2(self) -> Module:
        return self._children.get(self.FRONT2)

    @front2.setter
    def front2(self, module: Module) -> None:
        self.set_child(module, self.FRONT2)

    @property
    def back1(self) -> Module:
        return self._children.get(self.BACK1)

    @back1.setter
    def back1(self, module: Module) -> None:
        self.set_child(module, self.BACK1)

    @property
    def back2(self) -> Module:
        return self._children.get(self.BACK2)

    @back2.setter
    def back2(self, module: Module) -> None:
        self.set_child(module, self.BACK2)

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

        entry_point.add("geom", name=self.name, type="capsule", size=self.size, pos=new_pos, quat=mj_quat, rgba=self.color, friction=self.friction, density=self.density, conaffinity=self.conaffinity, condim=self.condim, contype=self.contype, margin=self.margin, solimp=self.solimp, solref=self.solref)
        
        # Find the children
        tasks = []

        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=entry_point)
                unbuilt.make_pose(
                    position= new_pos + new_quat * Quaternion.from_eulers([np.pi, np.pi/2, 0]) * attachment_point.orientation * attachment_point.offset, # Prev pos + offset rotated in right direction = attachment point pos
                    orientation= new_quat * Quaternion.from_eulers([np.pi, np.pi/2, 0]) * attachment_point.orientation ,
                )
                tasks.append(unbuilt)

        return tasks
