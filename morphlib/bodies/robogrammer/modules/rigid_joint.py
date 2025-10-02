from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

class RigidJoint(Module):

    FRONT = 0
    BACK = 1

    def __init__(
        self,
        size : int = 0.045, # body_link=0.045 radius, limb_link = 0.025 radius
        name: str = None,
        color: list = [1.0, 0.4, 0.0, 1.0]
    ):  
        self.size = size
        self.name =  name
        offset = Vector3([size, 0.0, 0.0])
        attachment_points = {
            self.FRONT: AttachmentPoint(
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, 0.0, 0.0]),
            ),  
            self.BACK: AttachmentPoint(
                offset=offset,
                orientation=Quaternion.from_eulers([0.0, np.pi, 0.0]),
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
    def back(self) -> Module:
        return self._children.get(self.BACK)

    @back.setter
    def back(self, module: Module) -> None:
        self.set_child(module, self.BACK)

    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list:

        # Build the module
        mj_quat = [attachment_point_quat[3], attachment_point_quat[0], attachment_point_quat[1], attachment_point_quat[2]]
        new_pos = attachment_point_pos + attachment_point_quat * Vector3([self.size,0,0])

        entry_point.add("geom", name=self.name, type="box", size=[self.size,self.size,self.size], pos=new_pos, quat=mj_quat, rgba=self.color, friction=[1, 0.5, 0.5])
        
        # Find the children
        tasks = []

        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=entry_point)
                unbuilt.make_pose(
                    position= new_pos + attachment_point_quat * attachment_point.orientation * attachment_point.offset, # Prev pos + offset rotated in right direction = attachment point pos
                    orientation= attachment_point_quat * attachment_point.orientation ,
                )
                tasks.append(unbuilt)

        return tasks
