from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

class ElbowJoint(Module):

    SIDE = 0
    BACK = 1

    def __init__(
        self,
        name: str,
        size : list = [0.045, 0.045], # body_link=0.045 radius, limb_link = 0.025 radius
        limits : list = [0, np.pi/2],
        type : int = 0,
        color: list = [0.15, 0.55, 0.15, 1.0]
    ):  

        self.size = size
        self.name =  name
        self.limits = limits
        self.type = type
        self.color = color

        if type == 0:
            attachment_points = {
                self.SIDE: AttachmentPoint(
                    offset=Vector3([size[0], 0, 0]),
                    orientation=Quaternion.from_eulers([0, 0.0, 0.0]),
                ),  
                self.BACK: AttachmentPoint(
                    offset=Vector3([size[1], 0.0, 0.0]),
                    orientation=Quaternion.from_eulers([0.0, np.pi, 0.0]),
                ),
            }
        else:
            attachment_points = {
                self.SIDE: AttachmentPoint(
                    offset=Vector3([0, 0, -size[1]]),
                    orientation=Quaternion.from_eulers([0, np.pi/2, 0.0]),
                ),
                self.BACK: AttachmentPoint(
                    offset=Vector3([size[0], 0.0, 0.0]),
                    orientation=Quaternion.from_eulers([0.0, np.pi, 0.0]),
                ),
            }
        super().__init__(attachment_points)

    @property
    def side(self) -> Module :
        return self._children.get(self.SIDE)

    @side.setter
    def side(self, module: Module) -> None:
        self.set_child(module, self.SIDE)

    @property
    def back(self) -> Module:
        return self._children.get(self.BACK)

    @back.setter
    def back(self, module: Module) -> None:
        self.set_child(module, self.BACK)

    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list:
        
        if self.type == 0:
            new_quat = attachment_point_quat * Quaternion.from_eulers([np.pi, np.pi/2, 0]) 
            offset = Vector3([self.size[1], 0, 0])
        else:
            new_quat = attachment_point_quat
            offset = Vector3([self.size[0], 0, 0])
        mj_quat = [new_quat[3], new_quat[0], new_quat[1], new_quat[2]]
        new_pos = attachment_point_pos + attachment_point_quat*offset

        if self.limits != None:
            limited = True 
        else:
            limited = False

        joint_body = entry_point.add("body", pos=new_pos, quat=mj_quat)
        joint_body.add("joint", name=self.name, type="hinge", axis=[0,0,1], pos=[0,0,0], limited=limited, range=self.limits)
        joint_body.add("geom", name=self.name, type="cylinder", size=self.size, pos=[0,0,0], quat=Quaternion(), rgba=self.color, friction=[1, 0.5, 0.5])
        mjcf.actuator.add("motor", ctrllimited=True, ctrlrange=[-1,1], gear=[150], joint=self.name)
        
        # Find the children
        tasks = []

        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=joint_body)
                unbuilt.make_pose(
                    position= attachment_point.offset, # Prev pos + offset rotated in right direction = attachment point pos
                    orientation= attachment_point.orientation
                )
                tasks.append(unbuilt)

        return tasks
