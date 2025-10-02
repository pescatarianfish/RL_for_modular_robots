from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

class Link(Module):

    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

    def __init__(
        self,
        size : list = [0.045, 0.075], # body_link=0.045 radius, limb_link = 0.025 radius
        angle : float = 0,
        name: str = None,
        color: list = [0, 0.25, 0.5, 1.0 ]
    ):  
        self.size = size
        self.angle = angle
        self.name =  name
        self.color = color
        front_back_offset = Vector3([size[1], 0.0, 0.0])
        sides_offset = Vector3([size[0], 0.0, 0.0])
        attachment_points = {
            self.FRONT: AttachmentPoint(
                offset=front_back_offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, 0.0]),
            ),  
            self.RIGHT: AttachmentPoint(
                offset=sides_offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, -np.pi/2]),
            ),
            self.BACK: AttachmentPoint(
                offset=front_back_offset,
                orientation=Quaternion.from_eulers([0.0, np.pi, 0.0]),
            ),
            self.LEFT: AttachmentPoint(
                offset=sides_offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, np.pi/2]),
            ),
        }

        super().__init__(attachment_points)

    @property
    def front(self) -> Module :
        return self._children.get(self.FRONT)

    @front.setter
    def front(self, module: Module) -> None:
        self.set_child(module, self.FRONT)

    @property
    def right(self) -> Module :
        return self._children.get(self.RIGHT)

    @right.setter
    def right(self, module: Module) -> None:
        self.set_child(module, self.RIGHT)

    @property
    def back(self) -> Module:
        return self._children.get(self.BACK)

    @back.setter
    def back(self, module: Module) -> None:
        self.set_child(module, self.BACK)

    @property
    def left(self) -> Module :
        return self._children.get(self.LEFT)

    @left.setter
    def left(self, module: Module) -> None:
        self.set_child(module, self.LEFT)

    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list:

        # Build the module
        if self.angle != 0 or self.angle != 0.0:
            rotation = Quaternion.from_eulers([0, 0, self.angle])
        else:
            rotation = Quaternion()

        new_quat = attachment_point_quat * (Quaternion.from_eulers([np.pi, np.pi/2, 0])) * rotation
        mj_quat = [new_quat[3], new_quat[0], new_quat[1], new_quat[2]]
        new_pos = attachment_point_pos + attachment_point_quat * Vector3([self.size[1],0,0])

        entry_point.add("geom", name=self.name, type="capsule", size=self.size, pos=new_pos, quat=mj_quat, rgba=self.color, friction=[1, 0.5, 0.5])
        
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
