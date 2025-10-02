from pyrr import Quaternion, Vector3
from morphlib.tools.module import Module, AttachmentPoint
from morphlib.tools.unbuilt_child import UnbuiltChild


class RootBody(Module):

    ATTACH = 0

    def __init__(
        self,
        pos : list = [0,0,0],
        cam_pos : list = [0, -3, -0.25],
        cam_xyaxes : list = [1, 0, 0, 0, 0, 1],
        name: str = None,
    ):  
        self.pos = pos
        self.cam_pos = cam_pos
        self.cam_xyaxes = cam_xyaxes
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
        root_body = entry_point.add("body", name=self.name, pos=self.pos)
        root_body.add("camera", name="track", mode="trackcom", pos=self.cam_pos, xyaxes=self.cam_xyaxes)
        root_body.add("freejoint", name="root")
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
