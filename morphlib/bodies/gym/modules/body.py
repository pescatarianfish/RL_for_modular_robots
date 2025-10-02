from pyrr import Quaternion, Vector3
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild


class Body(Module):

    ATTACH = 0

    def __init__(
        self,
        pos : list = [0,0,0],
        name: str = None,
    ):  
        self.pos = pos
        self.name =  name
        attachment_points = { 
            self.ATTACH: AttachmentPoint(
                offset=Vector3([0.0, 0.0, 0.0]),
                orientation=Quaternion(),
            ),
        }

        super().__init__(attachment_points)

    @property
    def attach(self) -> Module:
        return self._children.get(self.ATTACH)

    @attach.setter
    def attach(self, module: Module) -> None:
        self.set_child(module, self.ATTACH)


    def build(self, mjcf, entry_point, attachment_point_pos, attachment_point_quat) -> list:

        # Build the module
        root_body = entry_point.add("body", name=self.name, pos=self.pos)
      
        # Find the children
        tasks = []

        for child_index, attachment_point in self.attachment_points.items():
            child = self.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(module=child, mj_body=root_body)
                unbuilt.make_pose(
                    position=attachment_point_pos, # Prev pos + offset rotated in right direction = attachment point pos
                    orientation=attachment_point_quat,
                )
                tasks.append(unbuilt)

        return tasks
