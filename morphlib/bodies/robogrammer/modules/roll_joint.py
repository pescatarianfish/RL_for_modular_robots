from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

class RollJoint(Module):

    FRONT = 0
    BACK = 1

    def __init__(
        self,
        name: str,
        size : list = [0.045, 0.045], # body_link=0.045 radius, limb_link = 0.025 radius
        limits : list = [-150*np.pi/180, 150*np.pi/180],
        color: list = [0.039, 0.235, 0.204, 1.0]
    ):  

        self.size = size
        self.name =  name
        self.limits = limits
        self.color = color
        offset = Vector3([size[1], 0.0, 0.0])
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

        mj_quat = [attachment_point_quat[3], attachment_point_quat[0], attachment_point_quat[1], attachment_point_quat[2]]
        new_pos = attachment_point_pos + attachment_point_quat*Vector3([self.size[0], 0, 0])

        if self.limits != None:
            limited = True 
        else:
            limited = False

        joint_body = entry_point.add("body", pos=new_pos, quat=mj_quat)
        joint_body.add("joint", name=self.name, type="hinge", axis=[0,0,1], pos=[0,0,0], limited=limited, range=self.limits, armature=1, damping=1)
        joint_body.add("geom", name=self.name, type="cylinder", size=self.size, pos=[0,0,0], quat=Quaternion(), rgba=self.color, friction=[1, 0.5, 0.5])
        mjcf.actuator.add("motor", ctrllimited=True, ctrlrange=[-1,1], gear=[80], joint=self.name)
        
        # Find the children
        tasks = []

        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=joint_body)
                unbuilt.make_pose(
                    position= Vector3([self.size[0],0,0]), # Prev pos + offset rotated in right direction = attachment point pos
                    orientation= Quaternion(),
                )
                tasks.append(unbuilt)

        return tasks
