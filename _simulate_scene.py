import logging
import math

import cv2
import mujoco
import numpy as np
import numpy.typing as npt

from revolve2.simulation.scene import Scene, SimulationState
from revolve2.simulation.simulator import RecordSettings
from revolve2.simulation.scene import UUIDKey, Pose
from revolve2.simulation.scene.sensors import IMUSensor
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot_simulation._modular_robot_simulation_handler import ModularRobotSimulationHandler
from revolve2.modular_robot_simulation._sensor_state_impl import (ModularRobotSensorStateImpl,
                                                                  IMUSensorStateImpl )
from revolve2.modular_robot.body.sensors import (
    ActiveHingeSensor,
    CameraSensor,
    IMUSensor,
)
# from revolve2.modular_robot_simulation._imu_sensor_state_impl import IMUSensorStateImpl 
from pyrr import Vector3, Quaternion

from ._control_interface_impl import ControlInterfaceImpl
from ._open_gl_vision import OpenGLVision
from ._render_backend import RenderBackend
from ._scene_to_model import scene_to_model
from ._simulation_state_impl import SimulationStateImpl
from .viewers import CustomMujocoViewer, NativeMujocoViewer, ViewerType


def simulate_scene(
    scene_id: int,
    scene: Scene,
    headless: bool,
    record_settings: RecordSettings | None,
    start_paused: bool,
    control_step: float,
    sample_step: float | None,
    simulation_time: int | None,
    simulation_timestep: float,
    cast_shadows: bool,
    fast_sim: bool,
    viewer_type: ViewerType,
    render_backend: RenderBackend = RenderBackend.EGL
) -> list[SimulationState]:
    """
    Simulate a scene.

    :param scene_id: An id for this scene, unique between all scenes ran in parallel.
    :param scene: The scene to simulate.
    :param headless: If False, a viewer will be opened that allows a user to manually view and manually interact with the simulation.
    :param record_settings: If not None, recording will be done according to these settings.
    :param start_paused: If true, the simulation will start in a paused state. Only makessense when headless is False.
    :param control_step: The time between each call to the handle function of the scene handler. In seconds.
    :param sample_step: The time between each state sample of the simulation. In seconds.
    :param simulation_time: How long to simulate for. In seconds.
    :param simulation_timestep: The duration to integrate over during each step of the simulation. In seconds.
    :param cast_shadows: If shadows are cast.
    :param fast_sim: If fancy rendering is disabled.
    :param viewer_type: The type of viewer used for the rendering in a window.
    :param render_backend: The backend to be used for rendering (EGL by default and switches to GLFW if no cameras are on the robot).
    :returns: The results of simulation. The number of returned states depends on `sample_step`.
    :raises ValueError: If the viewer is not able to record.
    """
    logging.info(f"Simulating scene {scene_id}")

    """Define mujoco data and model objects for simuating."""
    model, mapping = scene_to_model(
        scene, simulation_timestep, cast_shadows=cast_shadows, fast_sim=fast_sim
    )
    data = mujoco.MjData(model)
    # print(f'this is multi: {scene}')

    """Define a control interface for the mujoco simulation (used to control robots)."""
    control_interface = ControlInterfaceImpl(
        data=data, abstraction_to_mujoco_mapping=mapping
    )
    """Make separate viewer for camera sensors."""
    camera_viewers = {
        camera.camera_id: OpenGLVision(
            model=model, camera=camera, headless=headless, open_gl_lib=render_backend
        )
        for camera in mapping.camera_sensor.values()
    }

    """Define some additional control variables."""
    last_control_time = 0.0
    last_sample_time = 0.0
    last_video_time = 0.0  # time at which last video frame was saved

    simulation_states: list[SimulationState] = (
        []
    )  # The measured states of the simulation

    """If we dont have cameras and the backend is not set we go to the default GLFW."""
    if len(mapping.camera_sensor.values()) == 0:
        render_backend = RenderBackend.GLFW
    
    """Initialize viewer object if we need to render the scene."""
    if not headless or record_settings is not None:
        match viewer_type:
            case viewer_type.CUSTOM:
                viewer = CustomMujocoViewer
            case viewer_type.NATIVE:
                viewer = NativeMujocoViewer
            case _:
                raise ValueError(
                    f"Viewer of type {viewer_type} not defined in _simulate_scene."
                )

        viewer = viewer(
            model,
            data,
            width=None if record_settings is None else record_settings.width,
            height=None if record_settings is None else record_settings.height,
            backend=render_backend,
            start_paused=start_paused,
            render_every_frame=False,
            hide_menus=(record_settings is not None),
        )

    """Record the scene if we want to record."""
    if record_settings is not None:
        if not viewer.can_record:
            raise ValueError(
                f"Selected Viewer {type(viewer).__name__} has no functionality to record."
            )
        video_step = 1 / record_settings.fps
        video_file_path = f"{record_settings.video_directory}/{scene_id}.mp4"
        fourcc = cv2.VideoWriter.fourcc(*"mp4v")
        video = cv2.VideoWriter(
            video_file_path,
            fourcc,
            record_settings.fps,
            viewer.current_viewport_size(),
        )
    
    """
    Compute forward dynamics without actually stepping forward in time.
    This updates the data so we can read out the initial state.
    """
    mujoco.mj_forward(model, data)
    images = {
        camera_id: camera_viewer.process(model, data)
        for camera_id, camera_viewer in camera_viewers.items()
    }

    initial_simulation_state = SimulationStateImpl(
        data=data,
        abstraction_to_mujoco_mapping=mapping,
        camera_views=images
    )
   
    # delete this later
    brains_real = None
    if isinstance(scene.handler, ModularRobotSimulationHandler):
        
        brains_real = scene.handler._brains
        # print(brains_real)
    else:
        logging.error("Scene handler is not a ModularRobotSimulationHandler. Cannot retrieve robot instance.")
    
    if brains_real:
        brain_instance, body_mapping = brains_real[0]
        initial_robot_sensor_state= ModularRobotSensorStateImpl(
                simulation_state=initial_simulation_state,
                body_to_multi_body_system_mapping=body_mapping,
            )
        
       
        logging.info("Created initial ModularRobotSensorStateImpl.")
    #--------------------------------------------
    
    the_actual_imu_sensor_instance = None
    found_imu = False 
    sensor_attached_mbs = None
    imustate = None
    joints = []
    
    
    for mbs in scene._multi_body_systems:

        for rigid_body in mbs._rigid_bodies:
            # joi = mbs.get_joints_for_rigid_body(rigid_body)
            # joi_ = [print(i.uuid) for i in joi]
            
            attached_sensors = rigid_body.sensors
          

            if attached_sensors.imu_sensors: 
                
                the_actual_imu_sensor_instance = attached_sensors.imu_sensors[0]
                sensor_attached_mbs = mbs
                
                imustate = IMUSensorStateImpl(
            simulation_state=initial_simulation_state,
            multi_body_system=sensor_attached_mbs,
            imu=the_actual_imu_sensor_instance,
        )
               
                
                logging.info(f'initial imu orientation: {imustate.orientation}, angular rate: {imustate.angular_rate}, specific force; {imustate.specific_force}')
                logging.info(f"Found imusensor (UUID: {the_actual_imu_sensor_instance.uuid}) attached to rigidBody (UUID: {rigid_body.uuid})")
               
                found_imu = True
                break 

        if found_imu:
            break 

    #set the target for hinges
    #this code block must be inside of the while loop 
    # before starting i believe i need to pass action from my RL model as parameter in simulate scene so i can use it here
    # so action will update the joint range (by scaling it) and i need to use it to update target 
    for joint in mapping.hinge_joint.values():
        print(f'this came from sim: {joint.id}')
        l, h = model.jnt_range[joint.id]
        #then i need to do scaling 
        #calculate new target 
        #after that i need to pass the new target for controler with the following:
        # data.ctrl[joint.ctrl_index_position] = position
        #then i believe handler in while loop gonna set hinges to new places
        #scene.handler.handle(simulation_state, control_interface, control_step)


    # Sample initial state.
    if sample_step is not None:
        
        simulation_states.append(
            SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mapping, camera_views=images
            )
        )

        
    
    """After rendering the initial state, we enter the rendering loop."""
    while (time := data.time) < (
        float("inf") if simulation_time is None else simulation_time
    ):

        # do control if it is time
        if time >= last_control_time + control_step:
            last_control_time = math.floor(time / control_step) * control_step
            # print(scene)
            simulation_state = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mapping, camera_views=images
            )
            scene.handler.handle(simulation_state, control_interface, control_step)
            if found_imu and the_actual_imu_sensor_instance is not None and sensor_attached_mbs is not None:
                
                current_imu_state = IMUSensorStateImpl(
                        simulation_state=simulation_state,
                        multi_body_system=sensor_attached_mbs,
                        imu=the_actual_imu_sensor_instance,
                    )
                logging.info(
                        " IMU Update - "
                        f"orient: {current_imu_state.orientation}, "
                        f"angularRate: {current_imu_state.angular_rate}, "
                        f"force: {current_imu_state.specific_force}"
                    )
                
                
        # sample state if it is time
        if sample_step is not None:
            if time >= last_sample_time + sample_step:
                last_sample_time = int(time / sample_step) * sample_step
                simulation_states.append(
                    SimulationStateImpl(
                        data=data,
                        abstraction_to_mujoco_mapping=mapping,
                        camera_views=images,
                    )
                )
                if found_imu and the_actual_imu_sensor_instance is not None and sensor_attached_mbs is not None:
                    current_imu_state = IMUSensorStateImpl(
                        simulation_state=simulation_state,
                        multi_body_system=sensor_attached_mbs,
                        imu=the_actual_imu_sensor_instance,
                    )
                    logging.info(
                        f"[Time: {time:.3f}s] IMU Update - "
                        f"Orient: {current_imu_state.orientation}, "
                        f"AngRate: {current_imu_state.angular_rate}, "
                        f"SpecForce: {current_imu_state.specific_force}"
                    )
                    

        # step simulation
        mujoco.mj_step(model, data)
        # extract images from camera sensors.
        images = {
            camera_id: camera_viewer.process(model, data)
            for camera_id, camera_viewer in camera_viewers.items()
        }

        # render if not headless. also render when recording and if it time for a new video frame.
        if not headless or (
            record_settings is not None and time >= last_video_time + video_step
        ):
            _status = viewer.render()

            # Check if simulation was closed
            if _status == -1:
                break

        # capture video frame if it's time
        if record_settings is not None and time >= last_video_time + video_step:
            last_video_time = int(time / video_step) * video_step

            # https://github.com/deepmind/mujoco/issues/285 (see also record.cc)
            img: npt.NDArray[np.uint8] = np.empty(
                (*viewer.current_viewport_size(), 3),
                dtype=np.uint8,
            )

            mujoco.mjr_readPixels(
                rgb=img,
                depth=None,
                viewport=viewer.view_port,
                con=viewer.context,
            )
            # Flip the image and map to OpenCV colormap (BGR -> RGB)
            img = np.flipud(img)[:, :, ::-1]
            video.write(img)

    """Once simulation is done we close the potential viewer and release the potential video."""
    if not headless or record_settings is not None:
        viewer.close_viewer()

    if record_settings is not None:
        video.release()

    # Sample one final time.
    if sample_step is not None:
        simulation_states.append(
            SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mapping, camera_views=images
            )
        )
    # print(vars(simulation_states[0]))

    logging.info(f"Scene {scene_id} done.")
    return simulation_states
