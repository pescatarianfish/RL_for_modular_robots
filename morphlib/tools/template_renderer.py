import jinja2, os
import numpy as np
from math import sin, cos

class TemplateRenderer():
    """The class for rendering Jinja templates. The XML files are searched for in
    the `envs/resources/{engine}/{morphology}` directory. Meshes are searched for in
    `envs/resources/{engine}/{morphology}/meshes` directory.

    :param engine: name of the engine to be used. eg 'mujoco' or 'bullet'
    :param morphology: name of the morphology to render
    """
    def __init__(self, filepath) -> None:
        """Constructor Method"""
        # Set the relative template dir as base
        self.filepath = filepath
        self.resource_dirs = os.path.dirname(filepath)


        try:
            mesh_dir = os.path.join(dir, 'meshes')
            self.mesh_dirs = [mesh_dir]
        except:
            self.mesh_dirs = [dir]

        print(self.filepath)
        print(self.resource_dirs)
        self.loader = jinja2.FileSystemLoader(searchpath=self.resource_dirs[0])
        self.template_env = jinja2.Environment(loader=self.loader)

    def render_template(self, **kwargs):
        """
        This function renders an XML template and returns the resulting XML.

        :param template_file: name of the XML file, relative to the
            `franka_sim/templates` directory or any directory from the
            ``FRANKA_MESH_PATH`` env variable
        :type template_file: str, ending in .xml
        :param \*\*kwargs: any keyword arguments will be passed to the template.
        :return: the rendered XML data
        :rtype: ???
        """
        template = self.template_env.get_template(self.filepath)
        rendered_xml = template.render(mesh_dirs=self.mesh_dirs, pi=np.pi,
                                       sin=sin, cos=cos, **kwargs)
        return rendered_xml

    def render_to_file(self, target_file, **kwargs):
        """
        Renders a template to a file.

        :param template_file: XML template file name to be passed to
            ``render_template``,
        :type template_file: string, ending in .xml
        :param target_file: the path where the rendered XML file will be saved,
        :type target_file: string
        :param \*\*kwargs: any keyword arguments will be passed to the template.
        """ # ??? Include example of kwargs?
        xml = self.render_template(self.filepath, **kwargs)
        with open(target_file, "w") as f:
            f.write(xml)
        
        return xml
