import json
import requests
from config_schema import VehicleConfig
import os
from typing import Dict, Any
import traceback

class FreeLLMAgent:
    def __init__(self, api_source: str = "huggingface", config_path: str = "default_config.json", debug: bool = True):
        self.api_source = api_source
        self.default_config = self._load_config(config_path)
        self.debug = debug
        
    def _load_config(self, path: str) -> VehicleConfig:
        """Load configuration from JSON file"""
        if not os.path.exists(path):
            default = VehicleConfig()
            with open(path, 'w') as f:
                json.dump(default.model_dump(), f, indent=2)
            return default
        with open(path, 'r') as f:
            return VehicleConfig(**json.load(f))
    
    def generate_config(self, sensor_data: Dict[str, Any], user_command: str) -> VehicleConfig:
        prompt = self._create_prompt(sensor_data, user_command)
        
        if self.debug:
            print("\n" + "="*80)
            print("PROMPT SENT TO LLM:")
            print("="*80)
            print(prompt)
            print("="*80)
        
        try:
            if self.api_source == "huggingface":
                return self._query_huggingface(prompt)
            else:
                raise ValueError("Unsupported API source")
        except Exception as e:
            if self.debug:
                print("\n" + "!"*80)
                print("API ERROR:")
                print("!"*80)
                print(f"Error Type: {type(e).__name__}")
                print(f"Error Message: {str(e)}")
                print("Stack Trace:")
                traceback.print_exc()
                print("!"*80)
                print("Falling back to default configuration")
            return self.default_config

    def _create_prompt(self, sensor_data: Dict[str, Any], user_command: str) -> str:
        schema = json.dumps(VehicleConfig.model_json_schema(), indent=2)
        return f"""
You are an AI controller for an underwater research vehicle. Your task is to generate a JSON configuration based on the current sensor data and user command.

### SENSOR DATA:
{json.dumps(sensor_data, indent=2)}

### USER COMMAND:
{user_command}

### OUTPUT REQUIREMENTS:
1. Output MUST be raw JSON only (no additional text)
2. JSON must match this schema:
{schema}
3. Maintain all safety constraints (depth 0-200m, speed 0.1-5 m/s, etc.)
4. For multi-step missions, provide ordered targets in 'target_sequence'

### EXAMPLE OUTPUT:
{{
  "target_sequence": [
    {{
      "x": 50.0,
      "y": -30.0,
      "depth": 25.0,
      "tolerance": 2.0,
      "action": "inspect"
    }},
    {{
      "x": 100.0,
      "y": 80.0,
      "depth": 150.0,
      "tolerance": 5.0,
      "action": "sample"
    }}
  ],
  "electrical": {{
    "main_light": 100,
    "uv_light": true,
    "camera_mode": "video",
    "sonar_active": true,
    "sensor_package": "full"
  }},
  "transit_speed": 2.5,
  "operation_mode": "search",
  "replan_conditions": ["obstacle_detected", "current_change"]
}}

### YOUR RESPONSE (JSON ONLY):
"""

    def _query_huggingface(self, prompt: str) -> VehicleConfig:
        """Use Hugging Face's free LLaMA 3 API """
        url = "https://router.huggingface.co/together/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {os.environ['HF_TOKEN']}",
        }

        payload = {
            "messages": [
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            "model": "meta-llama/Llama-3.2-3B-Instruct-Turbo"
        }
        
        if self.debug:
            print("\nSending request to Hugging Face API...")
            
        response = requests.post(url, headers=headers, json=payload)
        response.raise_for_status()

        try:
            if self.debug:
                print("\n" + "-"*80)
                print("RAW RESPONSE FROM HUGGING FACE:")
                print("-"*80)
                print(json.dumps(response.json(), indent=2))
                print("-"*80)
                
            config_json = response.json()["choices"][0]["message"]["content"]
            
            if self.debug:
                print("\n" + "-"*80)
                print("EXTRACTED CONFIGURATION JSON:")
                print("-"*80)
                print(config_json)
                print("-"*80)
                
            config = VehicleConfig(**json.loads(config_json))
            
            if self.debug:
                print("\n" + "-"*80)
                print("PARSED CONFIGURATION:")
                print("-"*80)
                print(config.model_dump_json(indent=2))
                print("-"*80)
                
            return config
        except (json.JSONDecodeError, KeyError) as e:
            if self.debug:
                print("\n" + "!"*80)
                print("RESPONSE PARSE ERROR:")
                print("!"*80)
                print(f"Error Type: {type(e).__name__}")
                print(f"Error Message: {str(e)}")
                print("!"*80)
                print("Falling back to default configuration")
            return self.default_config
