from isaacgym import gymapi

def isaacgym_setup_sim(gym):
    # get default set of parameters
    sim_params = gymapi.SimParams()

    # set common parameters
    sim_params.dt = 1 / 60
    sim_params.substeps = 2
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)

    # set PhysX-specific parameters
    sim_params.physx.use_gpu = True
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 6
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.contact_offset = 0.01
    sim_params.physx.rest_offset = 0.0

    # set Flex-specific parameters
    sim_params.flex.solver_type = 5
    sim_params.flex.num_outer_iterations = 4
    sim_params.flex.num_inner_iterations = 20
    sim_params.flex.relaxation = 0.8
    sim_params.flex.warm_start = 0.5

    # create sim with these parameters
    sim = gym.create_sim(compute_device=0, graphics_device=0, type=gymapi.SIM_PHYSX, params=sim_params)

    # create env
    env_lower = gymapi.Vec3(-2.0, -2.0, 0.0)
    env_upper = gymapi.Vec3(2.0, 2.0, 0.0)
    env = gym.create_env(sim, env_lower, env_upper, 1)

    return sim, env 

def asset_setup(env_mjcf):
    env_mjcf.compiler.angle = "radian"
    env_mjcf.compiler.inertiafromgeom = True
    env_mjcf.option.timestep = 0.01
    env_mjcf.option.integrator = "RK4"

    asset_options = gymapi.AssetOptions()# load asset with default control type of position for all joints
    asset_options.fix_base_link = False
    asset_options.default_dof_drive_mode = gymapi.DOF_MODE_EFFORT
    return asset_options