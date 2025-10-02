from pyrr import Quaternion, Vector3
import numpy as np
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild

class ActiveJoint(Module):

    FRONT = 0

    def __init__(
        self,
        name: str,
        horizontal: bool = False,
        color: list = [0.2, 1.0, 0.2, 1.0],
        position_ctrl: bool = True,
    ):  
        self.name = name
        self.horizontal = horizontal
        self.color = color
        self.position_ctrl = position_ctrl

        self.range=1.047197551  # 60 degrees
        self.frame_bounding_box=[0.018, 0.053, 0.0165891] 
        self.servo1_bounding_box=Vector3([0.0583, 0.0512, 0.020])
        self.servo2_bounding_box=Vector3([0.002, 0.053, 0.053])
        self.size=[0.018+0.0583+0.002, 0.053, 0.053]
        
        self.frame_mass=0.011
        self.servo1_mass=0.058
        self.servo2_mass=0.02
        self.armature=0.002 # Not used atm, seems to bug out sim
        self.pid_gain_p=5.0
        self.pid_gain_d=0.05

        offset = Vector3([self.frame_bounding_box[2]
                          +0.08-self.servo1_bounding_box[0]-self.frame_bounding_box[2]
                          +2*self.servo1_bounding_box[0]
                          +2*self.servo2_bounding_box[0], 0.0, 0.0])
        attachment_points = {
            self.FRONT: AttachmentPoint(
                offset=offset,
                orientation=Quaternion(),
            )
        }

        super().__init__(attachment_points)

    @property
    def front(self) -> Module :
        return self._children.get(self.FRONT)
    
    @front.setter
    def front(self, module: Module) -> None:
        self.set_child(module, self.FRONT)
        
    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list:

        # Build the module

        if self.horizontal:
            new_quat = attachment_point_quat * Quaternion.from_eulers([np.pi/2, 0, 0])
        else:
            new_quat = attachment_point_quat # * (Quaternion.from_eulers([np.pi, np.pi/2, 0]))
            
        mj_quat = [new_quat[3], new_quat[0], new_quat[1], new_quat[2]]
        new_pos = attachment_point_pos + attachment_point_quat * Vector3([self.frame_bounding_box[2],0,0])
        
        body = entry_point.add("body", name=self.name, pos=new_pos, quat=mj_quat)
        body.add("geom", name=self.name+"_frame", type="box", size=self.frame_bounding_box, rgba=self.color, friction=[0.9], mass=0.01632, solimp=".8 .8 .01", solref=".02 1", margin="0.001")
        joint_body = body.add("body", name=self.name+"_joint", pos=[0.08, 0, 0])
        joint_body.add("joint", name=self.name+"_hinge", pos=[-0.0299, 0, 0], axis=[0, -1, 0], range=[-self.range, self.range], type="hinge", armature=1, damping=1, limited=True)
        joint_body.add("geom", name=self.name+"_servo1", size=self.servo1_bounding_box, type="box", mass=self.servo1_mass, friction=[0.9], solimp=".8 .8 .01", solref=".02 1", margin="0.001")
        # joint_body.add("geom", name=self.name+"_servo2", size=self.servo2_bounding_box, pos=[self.servo1_bounding_box[0]+self.servo2_bounding_box[0], 0, 0], type="box", mass=self.servo2_mass, friction=[0.9], solimp=".8 .8 .01", solref=".02 1", margin="0.001")

        if self.position_ctrl:
            mjcf.actuator.add("motor", name=self.name, joint=self.name+"_hinge", gear=[0.156766])
            mjcf.actuator.add("position", name=self.name+"_position_servo", joint=self.name+"_hinge", kp=self.pid_gain_p)
            mjcf.actuator.add("velocity", name=self.name+"_velocity_servo", joint=self.name+"_hinge", kv=self.pid_gain_d)
        else:
            mjcf.actuator.add("motor", name=self.name, joint=self.name+"_hinge", gear=[20], ctrlrange=[-1, 1], ctrllimited=True)

        # Find the children
        tasks = []
        offset = Vector3([self.servo1_bounding_box[0]+2*self.servo2_bounding_box[0], 0.0, 0.0])
        if self.horizontal:
            attachment_point_orientation = Quaternion.from_eulers([-np.pi/2, 0.0, 0.0])
        else: 
            attachment_point_orientation = Quaternion()
        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=joint_body)
                unbuilt.make_pose(
                    position= attachment_point.orientation*offset,
                    orientation= attachment_point_orientation
                )
                tasks.append(unbuilt)

        return tasks
