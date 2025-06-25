import ray
from ray.rllib.algorithms import ppo
from environment.ppo.environment import UnderwaterVehicleEnv

# Initialize Ray and load the trained agent from checkpoint
ray.init()

agent = ppo.PPOTrainer(config={
    "env": "UnderwaterVehicle-v0",
    "framework": "torch",
}, env="UnderwaterVehicle-v0")

checkpoint_path = "path/to/your/checkpoint"
agent.restore(checkpoint_path)

env = UnderwaterVehicleEnv()
state = env.reset()
done = False

while not done:
    action = agent.compute_action(state)
    state, reward, done, info = env.step(action)
    # Optionally render or log your simulation results
    print("State:", state, "Reward:", reward)
