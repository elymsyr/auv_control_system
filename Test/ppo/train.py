import ray
from ray import tune
from ray.rllib.algorithms import ppo
from sys import path
path.append("./environment/ppo")

# Register your custom environment if necessary:
from gym.envs.registration import register

register(
    id='UnderwaterVehicle-v0',
    entry_point='environment.ppo.environment:UnderwaterVehicleEnv',
)

if __name__ == "__main__":
    ray.init()
    
    # Configure and run the PPO training
    tune.run("PPO",
             config={
                "env": "UnderwaterVehicle-v0",
                "num_workers": 1,  # Increase as needed for distributed training
                "framework": "torch",  # or "tf", depending on your preference
                "env_config": {},  # pass any additional parameters to your environment here
                "model": {
                    "fcnet_hiddens": [256, 256],  # adjust network architecture as needed
                    "fcnet_activation": "relu",
                },
                # Other PPO hyperparameters can be adjusted here
             })
