import gym
from stable_baselines3 import PPO
from modularEnv import ModularEnv
from revolve2.standards import modular_robots_v1, terrains
from revolve2.modular_robot.brain.dummy import BrainDummy

body    = modular_robots_v1.gecko_v1()
brain   = BrainDummy()
terrain = terrains.flat()

env= ModularEnv(body, brain, terrain)

model_ppo = PPO(
    policy="MlpPolicy",
    env=env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    vf_coef=0.5,
    max_grad_norm=0.5,
    tensorboard_log="./logs/ppo_revolve2/",
    verbose=1,
)

model_ppo.learn(total_timesteps=1_000_000)
model_ppo.save("ppo_model")

eval_env = ModularEnv(body, brain, terrain)
obs = eval_env.reset()
for _ in range(1_000):
    action, _ = model_ppo.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    if done:
        obs = eval_env.reset()