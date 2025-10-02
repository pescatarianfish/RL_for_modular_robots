from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

class Sphere(Module):

    NORTH = 0 
    EAST = 1 
    SOUTH = 2 
    WEST = 3 
    TOP_NE = 4 
    TOP_SE = 5 
    TOP_SW = 6
    TOP_NW = 7
    BOT_NE = 8 
    BOT_SE = 9 
    BOT_SW = 10
    BOT_NW = 11
    TOP = 12
    BOT = 13

    def __init__(
        self,
        size: list,
        name : str = None,
        conaffinity: float = 0,
        condim : int = 3,
        density : float = 1000,
        friction : list = [1, 0.005, 0.0001],
        margin : float = 0.0,
        mass: float = None,
        color: list = [0.2, 0.2, 1.0, 1.0]
    ):
        self.name = name
        self.size = size
        self.conaffinity = conaffinity
        self.condim = condim
        self.density = density
        self.friction = friction
        self.margin = margin
        self.mass = mass
        self.color = color

        offset = Vector3([size[0], 0.0, 0.0])
        attachment_points = { 
            self.NORTH: AttachmentPoint(
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, 0.0]),
            ),
            self.EAST: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, -np.pi/2]),
            ),
            self.SOUTH: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, np.pi]),
            ),
            self.WEST: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, np.pi/2]),
            ),
            self.TOP: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0, -np.pi/2, 0]),
            ),
            self.BOT: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, np.pi/2, 0]),
            ),
            self.TOP_NE: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0, -np.pi/4, -np.pi/4]),
            ),
            self.TOP_SE: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, -3*np.pi/4, -np.pi/4]),
            ),
            self.TOP_SW: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, -5*np.pi/4, -np.pi/4]),
            ),
            self.TOP_NW: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, -7*np.pi/4, -np.pi/4]),
            ),
            self.BOT_NE: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0, np.pi/4, np.pi/4]),
            ),
            self.BOT_SE: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 3*np.pi/4, np.pi/4]),
            ),
            self.BOT_SW: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 5*np.pi/4, np.pi/4]),
            ),
            self.BOT_NW: AttachmentPoint( 
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 7*np.pi/4, np.pi/4]),
            )
        }

        super().__init__(attachment_points)

    @property
    def north(self) -> Module:
        return self._children.get(self.NORTH)

    @north.setter
    def north(self, module: Module) -> None:
        self.set_child(module, self.NORTH)

    @property
    def east(self) -> Module:
        return self._children.get(self.EAST)

    @east.setter
    def east(self, module: Module) -> None:
        self.set_child(module, self.EAST)

    @property
    def south(self) -> Module:
        return self._children.get(self.SOUTH)

    @south.setter
    def south(self, module: Module) -> None:
        self.set_child(module, self.SOUTH)

    @property
    def west(self) -> Module:
        return self._children.get(self.WEST)

    @west.setter
    def west(self, module: Module) -> None:
        self.set_child(module, self.WEST)

    @property
    def top(self) -> Module :
        return self._children.get(self.TOP)

    @top.setter
    def top(self, module: Module) -> None:
        self.set_child(module, self.TOP)

    @property
    def bottom(self) -> Module :
        return self._children.get(self.BOT)

    @bottom.setter
    def bottom(self, module: Module) -> None:
        self.set_child(module, self.BOT)

    @property
    def top_ne(self) -> Module :
        return self._children.get(self.TOP_NE)

    @top_ne.setter
    def top_ne(self, module: Module) -> None:
        self.set_child(module, self.TOP_NE)

    @property
    def top_se(self) -> Module :
        return self._children.get(self.TOP_SE)

    @top_se.setter
    def top_se(self, module: Module) -> None:
        self.set_child(module, self.TOP_SE)

    @property
    def top_sw(self) -> Module :
        return self._children.get(self.TOP_SW)

    @top_sw.setter
    def top_sw(self, module: Module) -> None:
        self.set_child(module, self.TOP_SW)

    @property
    def top_nw(self) -> Module :
        return self._children.get(self.TOP_NW)

    @top_nw.setter
    def top_nw(self, module: Module) -> None:
        self.set_child(module, self.TOP_NW)

    @property
    def bot_ne(self) -> Module :
        return self._children.get(self.BOT_NE)

    @bot_ne.setter
    def bot_ne(self, module: Module) -> None:
        self.set_child(module, self.BOT_NE)

    @property
    def bot_se(self) -> Module :
        return self._children.get(self.BOT_SE)

    @bot_se.setter
    def bot_se(self, module: Module) -> None:
        self.set_child(module, self.BOT_SE)

    @property
    def bot_sw(self) -> Module :
        return self._children.get(self.BOT_SW)

    @bot_sw.setter
    def bot_sw(self, module: Module) -> None:
        self.set_child(module, self.BOT_SW)

    @property
    def bot_nw(self) -> Module :
        return self._children.get(self.BOT_NW)

    @bot_nw.setter
    def bot_nw(self, module: Module) -> None:
        self.set_child(module, self.BOT_NW)

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
        new_pos = attachment_point_pos + attachment_point_quat*Vector3([self.size[0], 0.0, 0.0]) # Attachment pos + offset in right direction
        mj_quat = [attachment_point_quat[3], attachment_point_quat[0], attachment_point_quat[1], attachment_point_quat[2]]
        entry_point.add("geom", name=self.name, type="sphere", pos=new_pos, quat=mj_quat, size=self.size, mass=self.mass, conaffinity=self.conaffinity, condim=self.condim, density=self.density, friction=self.friction, margin=self.margin, rgba=self.color)

        # Find the children
        tasks = []
        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=entry_point)
                unbuilt.make_pose(
                    position=new_pos+attachment_point_quat*attachment_point.orientation*attachment_point.offset, # Prev pos + offset rotated in right direction = attachment point pos
                    orientation=attachment_point_quat*attachment_point.orientation,
                )
                tasks.append(unbuilt)

        return tasks
