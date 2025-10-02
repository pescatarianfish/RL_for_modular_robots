"""Main script for the example."""

import logging

from pyrr import Vector3
import numpy as np
from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot import ModularRobot, ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.body.sensors import IMUSensor
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, ModularRobotSimulationState
from revolve2.modular_robot_simulation._sensor_state_impl import ModularRobotSensorStateImpl
from revolve2.modular_robot_simulation._build_multi_body_systems import BodyToMultiBodySystemConverter
from revolve2.modular_robot.brain.dummy import BrainDummy
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulators.mujoco_simulator._simulate_scene import simulate_scene
from revolve2.simulation.scene import MultiBodySystem
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.standards import modular_robots_v2, terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.simulation.scene import Joint, UUIDKey
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.modular_robot_simulation._sensor_state_impl import IMUSensorStateImpl
from revolve2.modular_robot_simulation._build_multi_body_systems import BodyToMultiBodySystemConverter


class ANNBrainInstance(BrainInstance):
    """ANN brain instance."""

    active_hinges: list[ActiveHinge]
    imu_sensor: IMUSensor

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        imu_sensor: IMUSensor,
    ) -> None:
        """
        Initialize the Object.

        :param active_hinges: The active hinges to control.
        :param imu_sensor: The IMU sensor.
        """
        self.active_hinges = active_hinges
        self.imu_sensor = imu_sensor
        self.init_sensor = None
        # self.see = []

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot.

        :param dt: Elapsed seconds since last call to this function.
        :param sensor_state: Interface for reading the current sensor state.
        :param control_interface: Interface for controlling the robot.
        """
        # To get data from you sensors you need the sensor itself, with which you can query the sensor stare from the ModularRobotSensorState object.
        # Get the sensors from the active hinges

        sensors = [
            active_hinge.sensors.active_hinge_sensor
            for active_hinge in self.active_hinges
            if active_hinge.sensors.active_hinge_sensor is not None
        ]
        assert len(sensors) == len(
            self.active_hinges
        ), "One of the active hinges does not have a sensor set."

        # Get the current angular positions of the active hinges
        current_positions = [
            sensor_state.get_active_hinge_sensor_state(sensor).position
            for sensor in sensors
        ]
        logging.info(f"current positions: {current_positions}")

        # Get the imu sensor state

        imu_state = sensor_state.get_imu_sensor_state(self.imu_sensor)
        # print(f"imu_state: {vars(imu_state)}")
        logging.info(f"orientation: {imu_state.orientation}")
        logging.info(f"angular rate: {imu_state.angular_rate}")
        logging.info(f"specific force: {imu_state.specific_force}")



        # Here you can implement your controller.
        # The current controller does nothing except for always settings the joint positions to 0.5.
        for active_hinge, sensor in zip(self.active_hinges, sensors):
            target = 0.5
            control_interface.set_active_hinge_target(active_hinge, target)


class ANNBrain(Brain):
    """The ANN brain."""

    active_hinges: list[ActiveHinge]
    imu_sensor: IMUSensor

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        imu_sensor: IMUSensor,
    ) -> None:
        """
        Initialize the Object.

        :param active_hinges: The active hinges to control.
        :param imu_sensor: The IMU sensor.
        """
        self.active_hinges = active_hinges
        self.imu_sensor = imu_sensor

    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        :returns: The created instance.
        """
        return ANNBrainInstance(
            active_hinges=self.active_hinges,
            imu_sensor=self.imu_sensor,
        )


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Create a body for the robot.
    body = modular_robots_v2.gecko_v2()

    """Every module on the robot can have sensors, to add them you do the following: """
    # Add an IMU Sensor to the core.
    body.core.add_sensor(imu := IMUSensor(position=Vector3([0.075, 0.075, 0.14])))



    # Create a brain for the robot.
    active_hinges = body.find_modules_of_type(ActiveHinge)
    print(active_hinges)
    # rng = make_rng_time_seed()
    brain = ANNBrain(active_hinges=active_hinges, imu_sensor=imu)
    # brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)

    # Create the scene.

    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)
    batch_param = make_standard_batch_parameters()
    mujoco_model, mujoco_mapping = scene_to_model(
        scene, batch_param.simulation_timestep, cast_shadows=False, fast_sim=False
    )

    # print(vars(batch_param))
    # Simulate the scene.
    sensors = [
        active_hinge.sensors.active_hinge_sensor
        for active_hinge in active_hinges
        if active_hinge.sensors.active_hinge_sensor is not None
    ]
    simulator = LocalSimulator(viewer_type="native")
    scenes = simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_param,
        scenes=scene
    )
    for i, active_hinge in enumerate(active_hinges):
        active_hinge_info = mujoco_mapping.hinge_joint.get(UUIDKey(active_hinge))
        print(active_hinge_info)
    # action_map = {
    #     UUIDKey(hinge): float(target)
    #     for hinge, target in zip(active_hinges, action)
    # }

    # for a single-scene run, `simulation_states` is a flat list;
    # we asked for sample_step=0.0 and sim_time=0.0 so we get exactly one state
    # initial_state = scenes[0]
    #
    # robot_sim_state = initial_state.get_modular_robot_simulation_state(robot)


    # print("Initial IMU orientation:   ", imu_state.orientation)
    # print("Initial IMU angular rate:  ", imu_state.angular_rate)
    # print("Initial IMU specific force:", imu_state.specific_force)

    # scenes_in = scenes[0]
    # robot_initial = scenes_in.get_modular_robot_simulation_state(robot)
    # # print(f"robot_initial: {vars(robot_initial)}")
    # pose = robot_initial.get_pose()
    # # ssa= scenes_in._simulation_state
    # # pose =ModularRobotSimulationState.
    # converter = BodyToMultiBodySystemConverter()
    # b, mapping_ = converter.convert_robot_body(body=body, pose=pose, translate_z_aabb=True)
    # robot_sensor_state = ModularRobotSensorStateImpl(
    #     simulation_state=robot_initial._simulation_state,
    #     body_to_multi_body_system_mapping=mapping_,
    # )
    # print(f"robot_sensor_state: {vars(robot_sensor_state)}")
    # sensor_sta = robot_sensor_state.get_imu_sensor_state(imu)
    # print(f"imu angulare rate: {sensor_sta.angular_rate}")
    # print(f"imu orientation: {sensor_sta.orientation}")
    # print(f"imu specific force: {sensor_sta.specific_force}")
    # print(f"Type of robot_initial: {type(ssa)}")
    # print(f"Is ModularRobotSensorState: {isinstance(ssa, ModularRobotSensorState)}")
    # print(help(ModularRobotScene))
    # print(vars(ssa))
    # print(vars(body.core._sensors._imu_sensor))
if __name__ == "__main__":
    main()
