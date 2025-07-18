# main.py
from llm_agent import FreeLLMAgent
from dotenv import load_dotenv
import os
from hf_token import HFTOKEN
os.environ["HF_TOKEN"] = HFTOKEN

load_dotenv()

# Initialize agent
agent = FreeLLMAgent(debug=True)

# Sample sensor data
sensor_data = {
    "position": {"x": 0, "y": 0, "depth": 5},
    "battery": 95,
    "obstacles": [],
    "temperature": 12.5
}

# Process user commands
commands = [
    "Inspect wreck at (50,-30) then sample sediment at (100,-50)",
    "Search for marine life in 100m radius",
    "Take photos of coral at current position"
]

for command in commands:
    print(f"\nProcessing command: '{command}'")
    config = agent.generate_config(sensor_data, command)
    print("\nFINAL CONFIGURATION:")
    print(config.model_dump_json(indent=2))
    
    # Update sensor position with last target
    if config.target_sequence:
        last_target = config.target_sequence[-1]
        sensor_data["position"] = {
            "x": last_target.x,
            "y": last_target.y,
            "depth": last_target.depth
        }