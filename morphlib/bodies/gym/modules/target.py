from pyrr import Quaternion
from morphlib.tools.module import Module


class Target(Module):

    def __init__(self, name: str = None, size=0.009):  
        self.name =  name
        self.size = size
        attachment_points = {}
        super().__init__(attachment_points)


    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list:

        # Build the module
        target = entry_point.add("body", name=self.name, pos=[0.1, -0.1, 0.01])
        target.add("joint", armature=0, axis=[1,0,0], damping=0, limited=True, name="target_x", pos=[0,0,0], range=[-0.27, 0.27], ref=0.1, stiffness=0, type="slide")
        target.add("joint", armature=0, axis=[0,1,0], damping=0, limited=True, name="target_y", pos=[0,0,0], range=[-0.27, 0.27], ref=-0.1, stiffness=0, type="slide")
        target.add("geom", conaffinity=0, contype=0, name="target_geom", pos=[0,0,0], rgba=[0.9, 0.2, 0.2, 1], size=[self.size], type="sphere")

        # Find the children
        tasks = []
        return tasks
