import numpy as np
import gym
from gym.spaces import Box
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.standards.morphological_measures import MorphologicalMeasures
from revolve2.modular_robot import ModularRobot, ModularRobotControlInterface
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import fitness_functions, modular_robots_v1, terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters

class ModularEnv(gym.Env):

    def __init__(self, body, brain, terrain):
        super(ModularEnv, self).__init__()
        setup_logging()
        self.rng = make_rng_time_seed()

        self.body = body
        self.brain =brain
        self.robot = ModularRobot(self.body, self.brain)
        self.terrain = terrain

        self.scene = ModularRobotScene(terrain= self.terrain)
        self.scene.add_robot(self.robot)
        self.simulator = LocalSimulator()
        self.params = make_standard_batch_parameters()

        self.steps_max= 50000

        self.hinges = MorphologicalMeasures(self.body).num_active_hinges
        self.active_hinges = self.body.find_modules_of_type(ActiveHinge)
        self.all_sensors = [
            active_hinge.sensors.active_hinge_sensor
            for active_hinge in self.active_hinges
            if active_hinge.sensors.active_hinge_sensor is not None
        ]
        self.action_space = Box(low=-1, high=1, shape=(self.hinges,))
        self.observation_space = Box(low=-np.inf,high=np.inf, shape=(self.hinges +10,))
        self.actions = 0
        self.current= None

    def reset(self):
        self.scene = ModularRobotScene(terrain= self.terrain)
        self.scene.add_robot(self.robot)

        scene_states = simulate_scenes(
            simulator=self.simulator,
            batch_parameters=self.params,
            scenes=self.scene,
        )
        
        robot_initial = scene_states[0].get_modular_robot_simulation_state(self.robot)
        obs = self.observation(robot_initial) #not sure about this, it is not ModularRobotSensorState


        self.actions = 0
        self.current = obs
        return self.current

    def observation(self, state): # how i can get the initial sensor state so i can pass it to observation
        current_positions = [
            state.get_active_hinge_sensor_state(sensor).position
            for sensor in self.all_sensors
        ]
        imu_state=  state.get_imu_sensor_state(self.brain.imu_sensor)
        orientation = imu_state.orientation
        angular_rate = imu_state.angular_rate
        force = imu_state.specific_force

        observation = current_positions + list(orientation) + list(angular_rate) + list(force)
        return np.array(observation)

    def step(self, action):

        flag = False
        # first apply the action
        for hinge, target in zip(self.active_hinges, action):
            ModularRobotControlInterface.set_active_hinge_target(hinge, float(target))
        # after you applied the action now you have different joint pos, so get the observation of it now this is your current state
        scene_states = simulate_scenes(
            simulator=self.simulator,
            batch_parameters=self.params,
            scenes=self.scene,
        )
        last_state = scene_states[-1].get_modular_robot_simulation_state(self.robot)
        obs = self.observation(last_state)
        # your current state = new first because we need to calculate the reward first, then return the current
        new = obs[:self.hinges]
        previous = self.current[:self.hinges]

        reward = float(np.sum(np.abs(new - previous)))
        #penalty?????
        #after you updated your current state provide the reward, it is new - current ??? look this reward thing later
        self.current = obs
        self.actions += 1
        if self.actions >= self.steps_max:
            flag = True


        return self.current, reward, flag, {}
        # add flag actions >= 50000 ??
        # dont forget to increase the actions

